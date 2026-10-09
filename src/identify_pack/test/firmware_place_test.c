#include <assert.h>
#include <string.h>
#include "stm32f4xx.h"
#include "Serial.h"

TestUSART test_usart;
float kin_target_x, kin_target_y;
uint8_t grab_cmd;
static uint8_t sent[256];
static unsigned sent_count;
void test_send_byte(uint8_t byte) { sent[sent_count++] = byte; }
void Screen_ShowQRCode(const char *raw) { (void)raw; }

static void receive(const char *payload)
{
    Serial_RxFrame.type = SERIAL_TYPE_PLACE_COMMAND;
    Serial_RxFrame.len = (uint8_t)strlen(payload);
    strcpy((char *)Serial_RxFrame.payload, payload);
    Serial_ProcessFrame();
}

int main(void)
{
    const char *id = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const char *one = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa,1,2,1,1,1,2,1,100,50,-42.5";
    const char *two = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa,2,2,1,2,5,3,1,100,50,-42.5";
    Serial_PlaceCommand_t command;
    unsigned i;
    uint8_t crc;
    receive("short,1,2,1,1,1,2,1,100,50,-42.5");
    assert(!Serial_TakePlaceCommand(&command));
    receive("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa,2147483648,2,1,1,1,2,1,100,50,-42.5");
    assert(!Serial_TakePlaceCommand(&command));
    receive(one);
    assert(Serial_TakePlaceCommand(&command));
    assert(command.ring == 2 && command.z_mm == -42.5f);
    assert(!Serial_TakePlaceCommand(&command));
    receive(one);
    assert(!Serial_TakePlaceCommand(&command));
    assert(sent_count == 0);
    Serial_CompletePlaceCommand(id, 999, 1);
    assert(sent_count == 0);
    Serial_CompletePlaceCommand(id, 1, 1);
    assert(sent[0] == 0xff && sent[1] == 0x0b && sent[sent_count - 1] == 0xfe);
    crc = sent[1] ^ sent[2];
    for (i = 3; i < sent_count - 2; i++) crc ^= sent[i];
    assert(crc == sent[sent_count - 2]);
    sent_count = 0;
    receive(one);
    assert(sent_count > 0 && !Serial_TakePlaceCommand(&command));
    receive(two);
    assert(Serial_TakePlaceCommand(&command));
    Serial_CompletePlaceCommand(id, 2, 0);
    receive(two);
    assert(Serial_TakePlaceCommand(&command));
    Serial_CompletePlaceCommand(id, 2, 1);
    receive(one);
    assert(!Serial_TakePlaceCommand(&command));
    return 0;
}
