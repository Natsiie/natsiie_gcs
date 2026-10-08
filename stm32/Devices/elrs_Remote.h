#ifndef __ELRS_REMOTE_H
#define __ELRS_REMOTE_H

#include "stdint.h"
#include "stdbool.h"

/* Data shared across tasks */
typedef struct {
    float Lx;          /* right stick X [-1.0f ~ 1.0f] */
    float Ly;          /* left  stick Y [-1.0f ~ 1.0f] (reserved, not parsed) */
    float Rx;          /* right stick X [-1.0f ~ 1.0f] */
    float Ry;          /* left  stick Y [-1.0f ~ 1.0f] */

    uint8_t sw[8];            /* switches: 0 / 1 / 2 */
    int16_t knobs[2];         /* knobs: 0 ~ 1000 */
    bool    btns[6];          /* buttons: false / true */
    uint8_t field_info[12];   /* field-config mode: 12 cells x 2 bits */
    uint8_t field_info_updated; /* flag: set when field_info refreshed */
    uint8_t online;           /* receiver online flag */
} ELRS_Data_t;

extern ELRS_Data_t ELRS_RC;
extern uint8_t rc_btn_pressed[6];  /* edge events: 1 = just pressed */

void ELRS_Remote_Init(void);
void ELRS_Remote_Update(void);

#endif
