#include <ets_sys.h>
#include "user_interface.h"
#include <osapi.h>
#include <os_type.h>
#include <gpio.h>
#include "driver\uart.h"
#include "driver\mfrc522.h"

LOCAL os_timer_t check_timer;

LOCAL void ICACHE_FLASH_ATTR check_cb(void *arg)
{
	uint8_t CardID[5];
	if (MFRC522_Check(CardID) == MI_OK) {
		os_printf("\r\nNEW CARD!\n\r");
		os_printf("0x%08x\n", CardID[0]);
		os_printf("0x%08x\n", CardID[1]);
		os_printf("0x%08x\n", CardID[2]);
		os_printf("0x%08x\n", CardID[3]);
		os_printf("0x%08x\n", CardID[4]);
	}
}
uint32 ICACHE_FLASH_ATTR user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 8;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void ICACHE_FLASH_ATTR user_rf_pre_init(void)
{
}

void ICACHE_FLASH_ATTR user_init(void)
{
	UARTInit(BIT_RATE_115200);
	wifi_set_opmode(0x02);
	os_printf("\r\n =============   init mfrc522   ============= \r\n");
	MFRC522_Init();
	os_printf("rc522 version: 0x%02x\n", MFRC522_ReadRegister(0x37));
	os_timer_disarm(&check_timer);
	os_timer_setfn(&check_timer, (os_timer_func_t *)check_cb, (void *)0);
	os_timer_arm(&check_timer, 500,1);
}
