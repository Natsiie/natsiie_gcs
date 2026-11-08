#ifndef APP_KIN_STUB_H
#define APP_KIN_STUB_H

#include <stdint.h>

/* 主机侧最小桩：只提供 Serial.c 用到的字段，避免拉入 STM32 头文件。
 * 真实定义见 stm32_firmware/Apps/app_kin.h。 */
typedef struct {
    uint32_t cmd    : 4;   /* 0..12 */
    uint32_t active : 1;
} KinFlag_t;

typedef struct {
    float target_x, target_y;   /* mm, 塔吊系 */
} KinTgt_t;

extern KinFlag_t kin_flag;
extern KinTgt_t  kin_tgt;

#endif /* APP_KIN_STUB_H */
