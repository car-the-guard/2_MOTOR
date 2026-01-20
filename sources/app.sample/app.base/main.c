// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : main.c
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>

#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
//#include "reg_phys.h"
#include <uart.h>
#include <gpio.h>
//#include <cstdint>

#if (APLT_LINUX_SUPPORT_SPI_DEMO == 1)
    #include <spi_eccp.h>
#endif
#if (APLT_LINUX_SUPPORT_POWER_CTRL == 1)
    #include <power_app.h>
#endif
#if ( MCU_BSP_SUPPORT_APP_KEY == 1)
    #include <key.h>
#endif  // ( MCU_BSP_SUPPORT_APP_KEY == 1 )

#if ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )
    #include <console.h>
#endif  // ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )
    #include <can_demo.h>
#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#if ( MCU_BSP_SUPPORT_APP_IDLE == 1 )
    #include <idle.h>
#endif  // ( MCU_BSP_SUPPORT_APP_IDLE == 1 )

#if ( MCU_BSP_SUPPORT_APP_SPI_LED == 1 )
    #include <spi_led.h>
#endif  // ( MCU_BSP_SUPPORT_APP_SPI_LED == 1 )

#if ( MCU_BSP_SUPPORT_APP_FW_UPDATE == 1 )
    #include "fwupdate.h"
#elif ( MCU_BSP_SUPPORT_APP_FW_UPDATE_ECCP == 1 )
    #include "fwupdate.h"
#endif


#define BLUETOOTH_UART_CH        (0)
#define UART_TX_PIN         GPIO_GPA(28)
#define UART_RX_PIN         GPIO_GPA(29)
#define HC06_BAUDRATE       (9600)
#define UART_PORTCFG        0


/*
***************************************************************************************************
*                                         GLOBAL VARIABLES
***************************************************************************************************
*/
uint32                                  gALiveMsgOnOff;
static uint32                           gALiveCount;

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/

static void Main_StartTask
(
    void *                              pArg
);

static void AppTaskCreate
(
    void
);

static void DisplayAliveLog
(
    void
);

static void DisplayOTPInfo
(
    void
);

void Delay_Loop(volatile uint32 count)
{
    while (count--) { __asm("nop"); }
}



/*
***************************************************************************************************
*                                         FUNCTIONS
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                          cmain
*
* This is the standard entry point for C code.
*
* Notes
*   It is assumed that your code will call main() once you have performed all necessary
*   initialization.
*
***************************************************************************************************
*/
void cmain(void)
{
    UartParam_t uartParam;
    uint8 rx_buf[1];
    sint32 rx_len;

    // 1. 시스템 초기화
    (void)SAL_Init();
    BSP_PreInit();
    BSP_Init(); 

    // 2. UART 설정
    UART_Close(BLUETOOTH_UART_CH);

    uartParam.sCh           = BLUETOOTH_UART_CH;
    uartParam.sPriority     = 10U;                  
    uartParam.sBaudrate     = HC06_BAUDRATE;        // 9600
    uartParam.sMode         = UART_POLLING_MODE;    
    uartParam.sCtsRts       = UART_CTSRTS_OFF;      
    uartParam.sPortCfg      = UART_PORTCFG; // UART0 기본값 (bsp.c 참조)
    uartParam.sWordLength   = WORD_LEN_8;           
    uartParam.sFIFO         = ENABLE_FIFO;          
    uartParam.s2StopBit     = TWO_STOP_BIT_OFF;     
    uartParam.sParity       = PARITY_SPACE; // <-- 에러 수정 (uart_test.c 참고)
    uartParam.sFnCallback   = NULL;                 

    // UART 열기
    if (UART_Open(&uartParam) != 0) 
    {
        while(1) { Delay_Loop(100000); }
    }

    mcu_printf("\n[VCP] BT Ready (9600)\n");

    // 3. 무한 루프
    while (1)
    {
        sint32 ch;
        sint8 err = 0;

        // [변경] 한 글자씩 직접 읽어서 에러 확인
        ch = UART_GetChar(BLUETOOTH_UART_CH, 0, &err);

        // 데이터 수신 성공 (ch >= 0)
        if (err == 0 && ch >= 0) 
        {
            uint8 data = (uint8)ch;
            
            // 1. 받은 문자 PC 화면에 16진수와 문자로 동시 출력 (디버깅용)
            // (mcu_printf가 UART0을 쓰므로 폰 화면에도 뜰 수 있음)
            mcu_printf("[RX] Hex:0x%02X Char:%c\n", data, data);

            // 2. 에코 (폰으로 다시 쏴주기)
            UART_PutChar(BLUETOOTH_UART_CH, data);
        }
        else if (err != 0 && err != -1) // -1은 "데이터 없음"이므로 무시
        {
            // 에러 발생 시 로그 출력 (예: 프레임 에러, 오버런 등)
            mcu_printf("[Err] Code: %d\n", err);
        }

        // 루프 속도 조절 (너무 빠르면 디버깅 로그 폭주함)
        Delay_Loop(50000); 
    }
}

/*
***************************************************************************************************
*                                          Main_StartTask
*
* This is an example of a startup task.
*
* Notes
*   As mentioned in the book's text, you MUST initialize the ticker only once multitasking has
*   started.
*
*   1) The first line of code is used to prevent a compiler warning because 'pArg' is not used.
*      The compiler should not generate any code for this statement.
*
***************************************************************************************************
*/
static void Main_StartTask(void * pArg)
{
    (void)pArg;
    (void)SAL_OsInitFuncs();

    /* Service Init*/

    /* Create application tasks */
    AppTaskCreate();

    while (1)
    {  /* Task body, always written as an infinite loop.       */
        DisplayAliveLog();
        //mcu_printf("\n MCU Idle !!!");
        mcu_printf("whyrano\n");
        (void)SAL_TaskSleep(1000);
        mcu_printf("=======@DEBUG@=======\n");
    }
}

static void AppTaskCreate(void)
{
#if (APLT_LINUX_SUPPORT_SPI_DEMO == 1)
    ECCP_InitSPIManager();
#endif  
#if (APLT_LINUX_SUPPORT_POWER_CTRL == 1)
    POWER_APP_StartDemo();
#endif

  
#if ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )
    CreateConsoleTask();
#endif  // ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )

#if ( MCU_BSP_SUPPORT_APP_KEY == 1 )
    KEY_AppCreate();
#endif  // ( MCU_BSP_SUPPORT_APP_KEY == 1 )

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )
    CAN_DemoCreateApp();
#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#if ( MCU_BSP_SUPPORT_APP_FW_UPDATE == 1 )
    CreateFWUDTask();
#elif ( MCU_BSP_SUPPORT_APP_FW_UPDATE_ECCP == 1 )
    CreateFWUDTask();
#endif

#if ( MCU_BSP_SUPPORT_APP_IDLE == 1 )
    IDLE_CreateTask();
#endif  // ( MCU_BSP_SUPPORT_APP_IDLE == 1 )

#if ( MCU_BSP_SUPPORT_APP_SPI_LED == 1)
    SPILED_CreateAppTask();
#endif  // ( MCU_BSP_SUPPORT_APP_SPI_LED == 1 )

}

static void DisplayAliveLog(void)
{
    if (gALiveMsgOnOff != 0U)
    {
        mcu_printf("\n %d", gALiveCount);

        gALiveCount++;

        if(gALiveCount >= MAIN_UINT_MAX_NUM)
        {
            gALiveCount = 0;
        }
    }
    else
    {
        gALiveCount = 0;
    }
}

#define LDT1_AREA_ADDR  0xA1011800U
#define PMU_REG_ADDR    0xA0F28000U

static void DisplayOTPInfo(void)
{
    volatile uint32 *ldt1Addr;
    volatile uint32 *chipNameAddr;
    volatile uint32 *remapAddr;
    volatile uint32 *hsmStatusAddr;
    uint32          chipName = 0;
    uint32          dualBankVal = 0;
    uint32          dual_bank = 0;
    uint32          expandFlashVal = 0;
    uint32          expand_flash = 0;
    uint32          remap_mode = 0;
    uint32          hsm_ready = 0;

    //----------------------------------------------------------------
    // OTP LDT1 Read
    // [11:0]Dual_Bank_Selection, [59:48]EXPAND_FLASH
    // Dual_Bank_Sel: [0xC0][11: 0] & [0xD0][11: 0] & [0xE0][11: 0] & [0xF0][11: 0]
    // EXPAND_FLASH : [0xC4][27:16] & [0xD4][27:16] & [0xE4][27:16] & [0xF4][27:16]
    // HwMC_PRG_FLS_LDT1: 0xA1011800

    ldt1Addr = (volatile uint32 *)(LDT1_AREA_ADDR + 0x00C0);
    chipNameAddr = (volatile uint32 *)(LDT1_AREA_ADDR + 0x0300);
    remapAddr = (volatile uint32 *)(PMU_REG_ADDR);
    hsmStatusAddr = (volatile uint32 *)(PMU_REG_ADDR + 0x0020);

    chipName = *chipNameAddr;
    chipName &= 0x000FFFFF;

    dualBankVal = ldt1Addr[ 0];
    expandFlashVal = ldt1Addr[ 1];

    dualBankVal &= ldt1Addr[ 4];
    expandFlashVal &= ldt1Addr[ 5];

    dualBankVal &= ldt1Addr[ 8];
    expandFlashVal &= ldt1Addr[ 9];

    dualBankVal &= ldt1Addr[12];
    expandFlashVal &= ldt1Addr[13];

    dualBankVal = (dualBankVal >> 0) & 0x0FFF;
    expandFlashVal  = (expandFlashVal >> 16) & 0x0FFF;

    dual_bank = (dualBankVal == 0x0FFF) ? 0 : 1;            // (single_bank : dual_bank)
    expand_flash  = (expandFlashVal  == 0x0000) ? 0 : 1;    // (only_eFlash : use_extSNOR)

    remap_mode = remapAddr[ 0];

    mcu_printf("    CHIP   NAME  : %x\n",    chipName);
    mcu_printf("    DUAL   BANK  : %d\n",    dual_bank);
    mcu_printf("    EXPAND FLASH : %d\n",    expand_flash);
    mcu_printf("    REMAP  MODE  : %d\n",    (remap_mode >> 16));

    hsm_ready = hsmStatusAddr[ 0];
    hsm_ready = (hsm_ready >> 2) & 0x0001;
#if 0
    if(hsm_ready)
    {
        mcu_printf("    HSM    READY : %d\n",    hsm_ready);
    }
    else
    {
        while(hsm_ready != 1)
        {
            mcu_printf("    HSM    READY : %d\n",    hsm_ready);
            mcu_printf("    wait...\n");
            hsm_ready = (hsm_ready >> 2) & 0x0001;
        }
    }
#else
    mcu_printf("    HSM    READY : %d\n",    hsm_ready);
#endif
}

#endif  // ( MCU_BSP_SUPPORT_APP_BASE == 1 )

