#include "bsp_key.h"

/**
  * @brief  一键启动开关初始化: PC9 配置为输入 (默认上拉, 开关接 GND)
  * @note   KEY_ACTIVE_LOW=0 时自动改成下拉, 对应开关接 3.3V 的接法
  */
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = KEY_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = KEY_ACTIVE_LOW ? GPIO_PuPd_UP : GPIO_PuPd_DOWN;
    GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  读取一键启动开关电平 (已按 KEY_ACTIVE_LOW 归一化)
  * @retval 1=按下/接通, 0=松开/断开
  */
uint8_t Key_Read(void)
{
    uint8_t level = GPIO_ReadInputDataBit(KEY_GPIO_PORT, KEY_GPIO_PIN);
#if KEY_ACTIVE_LOW
    return (level == Bit_RESET) ? 1 : 0;
#else
    return (level == Bit_SET) ? 1 : 0;
#endif
}
