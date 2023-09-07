//
// Created by Shin on 2023/4/30.
//
#include "main.h"
#include "FreeRTOS.h"
#include "../../BSP/LVGL/lvgl.h"
#include "cmsis_os.h"
#include "../LVGL_APP/lv_demo_benchmark.h"
#include "../../BSP/LVGL/porting/lv_port_disp.h"
#include "../../BSP/LVGL/porting/lv_port_indev.h"

void StartLightTask(void const * argument)
{
    while(1)
    {
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        osDelay(1000);
    }
}

void lvgl_heartbeat_func(void const * argument)
{
    while(1)
    {
        lv_timer_handler();
        osDelay(50);
    }
}

void start_init_task(void const * argument)
{
    do {
//        lv_init();
//#if USE_LVGL_DISP
//        lv_port_disp_init();
//#endif
//#if USE_LVGL_INDEV
//        lv_port_indev_init();
//#endif
//        lv_demo_benchmark();
    } while(0);
    vTaskDelete(NULL);
}

void vApplicationTickHook()
{
  // 告诉lvgl已经过去了1毫秒
  lv_tick_inc(1);
}
