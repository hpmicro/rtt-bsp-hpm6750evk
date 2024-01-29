/*
 * Copyright (c) 2021-2024 HPMicro
 *
 * Change Logs:
 * Date         Author          Notes
 * 2021-08-13   HPMicro         first version
 * 2024-01-03   HPMicro         change the SD card name
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <dfs_fs.h>
#include "rtt_board.h"

void thread_entry(void *arg);


int main(void)
{

    app_init_led_pins();

    static uint32_t led_thread_arg = 0;
    rt_thread_t led_thread = rt_thread_create("led_th", thread_entry, &led_thread_arg, 1024, 1, 10);
    rt_thread_startup(led_thread);

    rt_thread_mdelay(2000);

    if (dfs_mount(BOARD_SD_NAME, "/", "elm", 0, NULL) == 0)
    {
        rt_kprintf("%s mounted to /\n", BOARD_SD_NAME);
    }
    else
    {
        rt_kprintf("%s mount to / failed\n", BOARD_SD_NAME);
    }

    return 0;
}


void thread_entry(void *arg)
{
    while(1){
        app_led_write(0, APP_LED_ON);
        rt_thread_mdelay(500);
        app_led_write(0, APP_LED_OFF);
        rt_thread_mdelay(500);
        app_led_write(1, APP_LED_ON);
        rt_thread_mdelay(500);
        app_led_write(1, APP_LED_OFF);
        rt_thread_mdelay(500);
        app_led_write(2, APP_LED_ON);
        rt_thread_mdelay(500);
        app_led_write(2, APP_LED_OFF);
        rt_thread_mdelay(500);
    }
}
