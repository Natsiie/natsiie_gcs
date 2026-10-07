#include "sys.h"
#include "Serial.h"

TASK_STATUS Tasks_Parameters_Status;

#define SYSTEM_START_TASK_PRIO   2
#define SYSTEM_START_STK_SIZE    512
TaskHandle_t system_start_task_handler;
static void system_start_task(void *p_arg);

#define Chassis_TASK_PRIO        4
#define Chassis_TASK_STK_SIZE    512
TaskHandle_t Chassis_task_handler;

#define DATE_TASK_PRIO           3
#define DATE_STK_SIZE            1024
TaskHandle_t DATE_task_handler;

#define SIGNALING_TASK_PRIO      2
#define SIGNALING_STK_SIZE       512
TaskHandle_t SIGNALING_task_handler;

/**
  * @brief  FreeRTOS 初始化
  */
void freertos_init(void)
{
    xTaskCreate((TaskFunction_t)system_start_task,
                (const char*)"system_start_task",
                (uint16_t)SYSTEM_START_STK_SIZE,
                (void*)NULL,
                (UBaseType_t)SYSTEM_START_TASK_PRIO,
                (TaskHandle_t*)&system_start_task_handler);
    vTaskSetTaskNumber(system_start_task_handler, 10);
    vTaskStartScheduler();                        // 开始调度, 不再返回
}

/**
  * @brief  系统启动任务 (硬件已在 driver_Init 中初始化, 这里只做通信 + 应用 + 建任务)
  */
void system_start_task(void *p_arg)
{
    taskENTER_CRITICAL();

    CRSF_Init();
    Serial_Init();                       // 上位机通信 (PA9/PA10, 115200, 0xFF..0xFE)
    BSP_USART_Init(UART_3, 420000);   // ELRS 遥控 (PC10/PC11)
    BSP_USART_Init(UART_5, 420000);   // ELRS 遥控 (PD2/PC12)
//    TJC_Init(9600);   // 串口屏 (PC6/PC7, 原 VOFA 调试口已改)
    ELRS_Remote_Init();

    App_Chassis_Init();
    App_Tower_Init();

    xTaskCreate((TaskFunction_t)Chassis_Tasks,
                (const char*)"Chassis_Tasks",
                (uint16_t)Chassis_TASK_STK_SIZE,
                (void*)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t*)&Chassis_task_handler);

    xTaskCreate((TaskFunction_t)DATE_Tasks,
                (const char*)"DATE_task",
                (uint16_t)DATE_STK_SIZE,
                (void*)NULL,
                (UBaseType_t)DATE_TASK_PRIO,
                (TaskHandle_t*)&DATE_task_handler);

    xTaskCreate((TaskFunction_t)SIGNALING_Tasks,
                (const char*)"SIGNALING_Tasks",
                (uint16_t)SIGNALING_STK_SIZE,
                (void*)NULL,
                (UBaseType_t)SIGNALING_TASK_PRIO,
                (TaskHandle_t*)&SIGNALING_task_handler);

    taskEXIT_CRITICAL();
    vTaskDelete(NULL);
}

TaskHandle_t get_Chassis_task_handler(void)
{
    return Chassis_task_handler;
}

TaskHandle_t get_DATE_task_handler(void)
{
    return DATE_task_handler;
}

TaskHandle_t get_SIGNALING_task_handler(void)
{
    return SIGNALING_task_handler;
}

TASK_STATUS * Get_TASK_STATUS(void)
{
    return &Tasks_Parameters_Status;
}
