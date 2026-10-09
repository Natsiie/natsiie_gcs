/***************(C) COPYRIGHT SWUST_ROBOCON *************BEGIN OF FILE***************/
/**
  ******************************************************************************
  * @file      MyTasks.c
  * @author    SWUST_ROBOCON_2026_by_李焱
  * @version   V1.0.0
  * @date      2024-11-25
  * @brief     FreeRTOS 任务体 (只做调度, 具体逻辑在对应 App 里)
  ******************************************************************************
  */
#include "sys.h"
#include "elrs_Remote.h"

/**
  * @brief  底盘控制任务 (10ms): 遥控直控 + 定位 + 里程计 + 电池电压
  */
void Chassis_Tasks(void *p_arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xDelay10ms = pdMS_TO_TICKS(10);

    while (1)
    {
        App_Chassis_Task();   /* 底盘: 遥控直控 + 定位 + 里程计 + 视觉纠偏 + 电压 */
        vTaskDelayUntil(&xLastWakeTime, xDelay10ms);
    }
}

/**
  * @brief  串口1 上位机通信 (放在 10ms 任务里调用)
  *         每拍向 natsiie GCS 发送一次云台舵机当前角度 (TYPE=0x07 GimbalYaw),
  *         上位机 serial_bridge 收到后发布 /gimbal/joint_states。
  */
static void Serial_Test_Task(void)
{
    Serial_SendGimbalYaw(Servo_GetAngle(SERVO_GIMBAL) - 42.0f);
}

/**
  * @brief  塔吊 + 舵机任务 (10ms)
  */
void SIGNALING_Tasks(void *p_arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xDelay10ms = pdMS_TO_TICKS(10);

    while (1)
    {
//        Tower_Boot_Task();     /* 上电默认(只执行一次): 云台42->220, 电机5升抓取高度, 电机6不动 */
//        Screen_Boot();          /* 串口屏: 上电延时约400ms后写一次占位内容 */
//        Screen_SendTestLoop();  /* 连续发送: 每100ms发 "TJC_TX_OK N" (串口助手测TX用) */
//        Screen_Test_Task();    /* 串口屏测试: Watch 改 screen_test_*, trig 写 1 触发 */
//        Gimbal_Test_Task();    /* 云台角度调试 (Watch 写 gimbal_test_angle) */
//        Platform_Test_Task();  /* 物料平台角度调试 (Watch 写 platform_test_angle) */
        Serial_Test_Task();    /* 串口1 发送云台舵机角度给上位机 (natsiie GCS) */
//        App_Tower_Task();    /* 遥控直控塔吊 */
//        Tower_Test_Task();   /* sw[grab_sw] 0->2 触发抓取+放置, 2/3次转平台 */
//        Motor56_Test_Task();   /* 电机5/6 测试 (Watch 写 motor5/6_test_pos/vel/mode/dir) */
        /* 接收上位机帧并解析: 0x08 点位->存 kin_target_x/y; 0x09 GrabCmd"1"=ok授权->grab_cmd 下降抓取 */
        if (Serial_RxFrameReady) { Serial_RxFrameReady = 0; Serial_ProcessFrame(); }
        Kin_Update();        /* 正/逆解运动: Watch 写 kin_cmd=1(GotoXY, 用 kin_target_x/y) 触发; 到位点上方等 ok(0x09) 再降 */
//        Kin_ReadGripper();     /* 只读夹爪位姿: Watch 写 kin_read_trig=1 读一次, 结果 kin_pose_* */
        vTaskDelayUntil(&xLastWakeTime, xDelay10ms);
    }
}
uint8_t key;
/**
  * @brief  遥控数据刷新任务 (10ms)
  */
void DATE_Tasks(void *p_arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xDelay10ms = pdMS_TO_TICKS(10);

    while (1)
    {
//		key = Key_Read();
        ELRS_Remote_Update();
        vTaskDelayUntil(&xLastWakeTime, xDelay10ms);
    }
}

/***************(C) COPYRIGHT SWUST_ROBOCON *************END OF FILE********************/
