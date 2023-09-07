//
// Created by Shin on 2023/4/30.
//

#ifndef MAIN_C_RTOS_TASKS_H
#define MAIN_C_RTOS_TASKS_H

void lvgl_heartbeat_func(void const * argument);
void StartLightTask(void const * argument);
void start_init_task(void const * argument);
void vApplicationTickHook();

#endif //MAIN_C_RTOS_TASKS_H
