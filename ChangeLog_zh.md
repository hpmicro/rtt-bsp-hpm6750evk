# 更新

## v0.1.0
- 更新：
  - 整合了 HPM_SDK v0.13.1 release
    - 注:
      - HPM_SDK根目录下 *docs*, *samples*, *middleware* 和 *cmake* 目录已移除
  - 基于hpm_dma_manager重新实现了drv_uart_v2
  - 实现了audio驱动
  - 迁移到了rt-thread v4.1.0
  - 增加了对JLINK调试器的支持
- 新增示例：
  - audio_i2s_demo
  - audio_pdm_dao_demo

## v0.7.0
- 整合了 HPM_SDK v0.12.1 release
  - 注:
    - HPM_SDK根目录下 *docs*, *samples*, *middleware* 和 *cmake* 目录已移除
- 新增示例:
  - i2c_demo
  - usb_host_msc_udisk
- Bug 修复:
  - SD卡 multi-block 读/写 问题
  - lv_demo_widgets 频率过慢问题

## v0.5.0

- 整合了SDK v0.10.0
  - 注：
    - SDK根目录下的doc目录被删除
- 更新：
  - Tolchain升级到GCC 11.1.0
- 增加了如下示例：
  - blink_led
  - lv_demo_widgets
  - uart_dma_demo
  - timer_demo
  - sdcard_demo
  - ethernet_demo
  - usb_device_generic_hid
  - flashdb_demo
  - can_example
  - ethernet_ptp_master_demo
  - ethernet_ptp_slave_demo
