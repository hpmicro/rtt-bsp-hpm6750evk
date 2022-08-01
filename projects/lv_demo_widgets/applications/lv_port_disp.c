/*
 * Copyright (c) 2021- 2022 hpmicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "board.h"
#include "hpm_l1c_drv.h"
#include "hpm_lcdc_drv.h"
#include "hpm_dma_drv.h"

#include "lv_port_disp.h"
#include <rthw.h>

/*
 * resource definition
 */

#ifndef RUNNING_CORE_INDEX
#define RUNNING_CORE_INDEX HPM_CORE0
#endif

#define LCD_CONTROLLER BOARD_LCD_BASE
#define LCD_LAYER_INDEX (0)
#define LCD_LAYER_DONE_MASK (LCD_LAYER_INDEX + 1)
#define LCD_IRQ_NUM  BOARD_LCD_IRQ

#ifndef HPM_LCD_IRQ_PRIORITY
#define HPM_LCD_IRQ_PRIORITY  1
#endif

#define LV_LCD_WIDTH BOARD_LCD_WIDTH
#define LV_LCD_HEIGHT BOARD_LCD_HEIGHT

static volatile bool back_buffer_filled;
static void init_lcd(void);
static void flush_display(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
static lcdc_layer_config_t layer;
static lv_disp_drv_t g_disp_drv;
static lv_color_t __attribute__((section(".framebuffer"), aligned(HPM_L1C_CACHELINE_SIZE))) framebuffer[2][LV_LCD_WIDTH * LV_LCD_HEIGHT];
static volatile lv_color_t *back_buffer;
static volatile lv_color_t *front_buffer;
static uint32_t ms_per_frame = 0;

void isr_lcd(void)
{
    volatile uint32_t s = lcdc_get_dma_status(LCD_CONTROLLER);
    volatile lv_color_t *tmp;
    lcdc_clear_dma_status(LCD_CONTROLLER, s);
    if ((back_buffer_filled) && (s & (LCD_LAYER_DONE_MASK << LCDC_DMA_ST_DMA0_DONE_SHIFT))) {
        tmp = front_buffer;
        front_buffer = back_buffer;
        back_buffer = tmp;
        lcdc_layer_set_next_buffer(LCD_CONTROLLER, LCD_LAYER_INDEX, (uint32_t)front_buffer);
        lv_disp_flush_ready(&g_disp_drv);
        back_buffer_filled = false;
    }
}
SDK_DECLARE_EXT_ISR_M(LCD_IRQ_NUM, isr_lcd)


__attribute__((weak)) void hpm_lv_monitor(lv_disp_drv_t * disp_drv, uint32_t time, uint32_t px)
{
    ms_per_frame = time;
}

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    init_lcd();

    static lv_disp_draw_buf_t draw_buf_dsc;
    lv_disp_draw_buf_init(&draw_buf_dsc, framebuffer[0], framebuffer[1], LV_LCD_WIDTH * LV_LCD_HEIGHT);   /*Initialize the display buffer*/

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    lv_disp_drv_init(&g_disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    g_disp_drv.hor_res = LV_LCD_WIDTH;
    g_disp_drv.ver_res = LV_LCD_HEIGHT;

    /*Used to copy the buffer's content to the display*/
    g_disp_drv.flush_cb = flush_display;
    g_disp_drv.monitor_cb = hpm_lv_monitor;

    /*Set a display buffer*/
    g_disp_drv.draw_buf = &draw_buf_dsc;
    g_disp_drv.full_refresh = 1;

    /*Finally register the driver*/
    lv_disp_drv_register(&g_disp_drv);

    front_buffer = framebuffer[0];
    back_buffer = framebuffer[0];
    back_buffer_filled = false;
}

/* Initialize your display and the required peripherals. */
static void init_lcd(void)
{
    display_pixel_format_t pixel_format;
    lcdc_config_t config = {0};
    lcdc_get_default_config(LCD_CONTROLLER, &config);

    config.resolution_x = LV_LCD_WIDTH;
    config.resolution_y = LV_LCD_HEIGHT;

#if LV_COLOR_DEPTH == 32
    pixel_format = display_pixel_format_argb8888;
#elif LV_COLOR_DEPTH == 16
    pixel_format = display_pixel_format_rgb565;
#else
#error only support 16 or 32 color depth
#endif

    lcdc_init(LCD_CONTROLLER, &config);
    lcdc_get_default_layer_config(LCD_CONTROLLER, &layer, pixel_format, LCD_LAYER_INDEX);

    rt_memset(framebuffer, 0, sizeof(framebuffer));

    layer.position_x = 0;
    layer.position_y = 0;
    layer.width = config.resolution_x;
    layer.height = config.resolution_y;
    layer.buffer = (uint32_t) core_local_mem_to_sys_address(RUNNING_CORE_INDEX, (uint32_t) framebuffer[0]);
    layer.background.u = 0;

    if (status_success != lcdc_config_layer(LCD_CONTROLLER, LCD_LAYER_INDEX, &layer, true)) {
        rt_kprintf("failed to configure layer\n");
        while (1) {
        }
    }

    lcdc_turn_on_display(LCD_CONTROLLER);
    lcdc_enable_interrupt(LCD_CONTROLLER, LCD_LAYER_DONE_MASK << 16);
    intc_m_enable_irq_with_priority(LCD_IRQ_NUM, HPM_LCD_IRQ_PRIORITY);
}

static bool is_back_buffer(lv_color_t *color_p)
{
    return (color_p == back_buffer);
}

static void flush_display(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    if (back_buffer_filled || ((front_buffer != back_buffer) && !is_back_buffer(color_p))) {
        /* it should not go here */
        while (1) {
        }
    }
    back_buffer = (lv_color_t *)core_local_mem_to_sys_address(RUNNING_CORE_INDEX, (uint32_t) color_p);
    if (l1c_dc_is_enabled()) {
        uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t)back_buffer);
        uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)back_buffer + LV_LCD_HEIGHT * LV_LCD_WIDTH * LV_COLOR_DEPTH / 8);
        uint32_t aligned_size = aligned_end - aligned_start;
        rt_base_t level = rt_hw_interrupt_disable();
        l1c_dc_writeback(aligned_start, aligned_size);
        rt_hw_interrupt_enable(level);
    }
    back_buffer_filled = true;
}
