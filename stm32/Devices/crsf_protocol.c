#include "crsf_protocol.h"

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

// 全局变量暴露给 elrs_remote 使用
uint16_t CRSF_Channels[16];
uint32_t CRSF_LastRxTime = 0;

static uint8_t g_rx_buf[64];
static uint8_t g_rx_idx = 0;
static uint8_t g_crc8_tab[256];

typedef struct {
    unsigned ch0 : 11; unsigned ch1 : 11; unsigned ch2 : 11; unsigned ch3 : 11;
    unsigned ch4 : 11; unsigned ch5 : 11; unsigned ch6 : 11; unsigned ch7 : 11;
    unsigned ch8 : 11; unsigned ch9 : 11; unsigned ch10 : 11; unsigned ch11 : 11;
    unsigned ch12 : 11; unsigned ch13 : 11; unsigned ch14 : 11; unsigned ch15 : 11;
} __attribute__((packed)) crsf_channels_t;

static void CRC8_Init(uint8_t poly) {
    for (int i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (int j = 0; j < 8; j++) crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
        g_crc8_tab[i] = crc;
    }
}

uint8_t CRC8_Calc(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) crc = g_crc8_tab[crc ^ *data++];
    return crc;
}

void CRSF_Init(void) {
    CRC8_Init(0xD5);
}

static void ProcessPacket(void) {
    if (g_rx_buf[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER && g_rx_buf[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        crsf_channels_t *ch = (crsf_channels_t *)&g_rx_buf[3];
        // 解包
        CRSF_Channels[0] = ch->ch0; CRSF_Channels[1] = ch->ch1;
        CRSF_Channels[2] = ch->ch2; CRSF_Channels[3] = ch->ch3;
        CRSF_Channels[4] = ch->ch4; CRSF_Channels[5] = ch->ch5;
        CRSF_Channels[6] = ch->ch6; CRSF_Channels[7] = ch->ch7;
        CRSF_Channels[8] = ch->ch8; CRSF_Channels[9] = ch->ch9;
        CRSF_Channels[10]= ch->ch10; CRSF_Channels[11]= ch->ch11;
        CRSF_Channels[12]= ch->ch12; CRSF_Channels[13]= ch->ch13;
        CRSF_Channels[14]= ch->ch14; CRSF_Channels[15]= ch->ch15;
        
        // 记录时间戳，证明遥控器还活着
        CRSF_LastRxTime = xTaskGetTickCountFromISR(); // 因为这是在串口中断里调用的
    }
}

void CRSF_ProcessByte(uint8_t byte) {
    g_rx_buf[g_rx_idx++] = byte;
    if (g_rx_idx < 2) return;
    uint8_t len = g_rx_buf[1];
    if (len < 3 || len > 62) { g_rx_idx = 0; return; }
    if (g_rx_idx >= len + 2) {
        if (CRC8_Calc(&g_rx_buf[2], len - 1) == g_rx_buf[g_rx_idx - 1]) ProcessPacket();
        g_rx_idx = 0; 
    }
}
