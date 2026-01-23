// SPDX-License-Identifier: Apache-2.0
/*
***************************************************************************************************
* FileName : main.c
* Description : VCP-G RTOS Integration (Clean Build Version)
***************************************************************************************************
*/

#define MCU_BSP_SUPPORT_APP_BASE 1

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <uart.h>
#include <gpio.h>
#include <pdm.h>
#include <ictc.h>
#include <string.h>

// ★ 우리가 만든 CAN 앱 헤더 포함
#include "can_app.h"

// -------------------------------------------------------------
// [기존 헤더 포함] 시스템 태스크용
// -------------------------------------------------------------
#if (APLT_LINUX_SUPPORT_SPI_DEMO == 1)
    #include <spi_eccp.h>
#endif
#if (APLT_LINUX_SUPPORT_POWER_CTRL == 1)
    #include <power_app.h>
#endif
#if ( MCU_BSP_SUPPORT_APP_KEY == 1)
    #include <key.h>
#endif
#if ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )
    #include <console.h>
#endif
#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )
    #include <can_demo.h>
#endif
#if ( MCU_BSP_SUPPORT_APP_IDLE == 1 )
    #include <idle.h>
#endif
#if ( MCU_BSP_SUPPORT_APP_SPI_LED == 1 )
    #include <spi_led.h>
#endif
#if ( MCU_BSP_SUPPORT_APP_FW_UPDATE == 1 )
    #include "fwupdate.h"
#elif ( MCU_BSP_SUPPORT_APP_FW_UPDATE_ECCP == 1 )
    #include "fwupdate.h"
#endif

// =========================================================
// [VCP 설정] 매크로 및 상수
// =========================================================

// [Fix] ICTC 필수 매크로 정의
#ifndef ICTC_IRQ_CTRL_PRDDT_CMP_ISR
#define ICTC_IRQ_CTRL_PRDDT_CMP_ISR      (1UL << 4) 
#endif
#ifndef ICTC_OPEN_CTRL_FLTCNT_EN
#define ICTC_OPEN_CTRL_FLTCNT_EN         (1UL << 0)
#define ICTC_OPEN_CTRL_PDCMPCNT_EN       (1UL << 1)
#endif
#ifndef ICTC_OPMODE_CTRL_RISING_EDGE
#define ICTC_OPMODE_CTRL_ABS_SEL         (0UL)
#define ICTC_OPMODE_CTRL_RISING_EDGE     (1UL << 1) 
#define ICTC_OPMODE_CTRL_TCLK_BYPASS     (1UL << 2)
#define ICTC_OPMODE_CTRL_FLT_IMM_F_MODE  (0UL)
#define ICTC_OPMODE_CTRL_FLT_IMM_R_MODE  (0UL)
#define ICTC_OPMODE_CTRL_PRDDT_CMP_ISR   (1UL << 5)
#define ICTC_OPMODE_CTRL_F_IN_SEL_OFFSET (8)
#endif

#define CAN_ID_RX_SENSOR    (0x24)
#define CAN_ID_TX_SPEED     (0x28)

// 모터 핀
#define MOTOR_L_PWM_CH      (0)             
#define MOTOR_L_IN1         GPIO_GPB(23)     
#define MOTOR_L_IN2         GPIO_GPB(21)     
#define MOTOR_R_PWM_CH      (8)             
#define MOTOR_R_IN1         GPIO_GPB(24)     
#define MOTOR_R_IN2         GPIO_GPB(22)     

// 엔코더
#define ENCODER_L_ICTC_CH   (0)         
#define ENCODER_GPIO_PIN    GPIO_GPC(4)
#define ENCODER_PIN_INDEX   (64UL) 

// PWM
#define PWM_PERIOD_NS       (1000000UL)     
#define MAX_DUTY_SCALE      (1000UL) 

// [Fix] 중복 정의 경고 해결
#ifdef GPIO_PERICH_CH0
#undef GPIO_PERICH_CH0
#endif
#define GPIO_PERICH_CH0     (0UL)

#ifdef UART_DEBUG_CH
#undef UART_DEBUG_CH
#endif
#define UART_DEBUG_CH       (0)
#define UART_BAUDRATE       (115200)

#define USE_PMM_MONITOR     (0UL)
#define COUNTS_PER_REV      (1600.0f) 
#define WHEEL_DIA_M         (0.065f)  
#define PI                  (3.141592f)

// Task 설정
#define MOTOR_TASK_STK_SIZE (1024)
#define MOTOR_TASK_PRIO     (SAL_PRIO_APP_CFG + 1)

// 전역 변수
volatile int32 g_TargetF = 0;   
volatile int32 g_TargetR = 0;   
uint32 gALiveMsgOnOff = 1;
static uint32 gALiveCount = 0;

typedef struct {
    int32_t counter;       
    int32_t last_counter;  
    float speed_mps;       
} Encoder_t;

Encoder_t Enc1;
static uint32 g_MotorTaskID;
static uint32 g_MotorTaskStk[MOTOR_TASK_STK_SIZE];

// 함수 프로토타입
static void Main_StartTask(void * pArg);
static void AppTaskCreate(void);
static void DisplayAliveLog(void);
static void DisplayOTPInfo(void);

// VCP 함수 프로토타입
static void Motor_HW_Init(void);
static void Encoder_HW_Init(void);
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale);
static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed);
static void Motor_Task_Loop(void *pArg);
static void ICTC_Callback(uint32 uiChannel, uint32 uiPeriod, uint32 uiDuty);
static float Clamp_Val(float x, float lo, float hi);

// =========================================================
// [1] cmain (진입점)
// =========================================================
void cmain (void)
{
    static uint32 AppTaskStartID = 0;
    static uint32 AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    SALRetCode_t err;
    SALMcuVersionInfo_t versionInfo = {0,0,0,0};

    (void)SAL_Init();
    BSP_PreInit();
    BSP_Init();

    (void)SAL_GetVersion(&versionInfo);
    mcu_printf("\n===============================\n");
    mcu_printf("    MCU BSP Version: V%d.%d.%d\n", versionInfo.viMajorVersion, versionInfo.viMinorVersion, versionInfo.viPatchVersion);
    mcu_printf("    [VCP] Integrated Mode Booting...\n");
    mcu_printf("===============================\n\n");

    err = (SALRetCode_t)SAL_TaskCreate(&AppTaskStartID,
                                     (const uint8 *)"App Task Start",
                                     (SALTaskFunc) &Main_StartTask,
                                     &AppTaskStartStk[0],
                                     ACFG_TASK_MEDIUM_STK_SIZE,
                                     SAL_PRIO_APP_CFG,
                                     NULL);

    if (err == SAL_RET_SUCCESS) {
        (void)SAL_OsStart();
    }
}

// =========================================================
// [2] Main Start Task
// =========================================================
static void Main_StartTask(void * pArg)
{
    (void)pArg;
    (void)SAL_OsInitFuncs();

    // 애플리케이션 태스크 생성
    AppTaskCreate();

    while (1)
    {
        DisplayAliveLog();
        (void)SAL_TaskSleep(5000); 
    }
}

// =========================================================
// [3] AppTaskCreate
// =========================================================
static void AppTaskCreate(void)
{
    // 기존 시스템 태스크
    #if ( MCU_BSP_SUPPORT_APP_CONSOLE == 1 )
        CreateConsoleTask();
    #endif

    #if ( MCU_BSP_SUPPORT_APP_IDLE == 1 )
        IDLE_CreateTask();
    #endif

    // ★ VCP 하드웨어 및 태스크 시작
    mcu_printf("[VCP] Initializing Hardware...\n");
    Motor_HW_Init();
    Encoder_HW_Init();

    mcu_printf("[VCP] Starting CAN App Task...\n");
    CAN_start_task(); 

    mcu_printf("[VCP] Starting Motor Task...\n");
    SAL_TaskCreate(&g_MotorTaskID, 
                   (const uint8 *)"Motor Task", 
                   (SALTaskFunc)Motor_Task_Loop, 
                   &g_MotorTaskStk[0], 
                   MOTOR_TASK_STK_SIZE, 
                   MOTOR_TASK_PRIO, 
                   NULL);
}

// =========================================================
// [4] VCP 모터 제어 및 CAN 연결 로직
// =========================================================

void CAN_consume_rx_message(CANMessage_t *pMsg, CAN_payload_t payload)
{
    if (pMsg->mId == CAN_ID_RX_SENSOR) // 0x24
    {
        int8_t cmd_fwd = (int8_t)payload.field.data[0];
        int8_t cmd_rot = (int8_t)payload.field.data[1];
        g_TargetF = (int32)cmd_fwd;
        g_TargetR = (int32)cmd_rot;
    }
}

// [Fix] 가독성 개선 및 경고 제거
static float Clamp_Val(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void Motor_Task_Loop(void *pArg)
{
    (void)pArg;
    for (;;)
    {
        SAL_TaskSleep(10); 

        float F = (float)g_TargetF;
        float R = (float)g_TargetR;

        // [Fix] R 변수 사용
        float left_val  = Clamp_Val(F + R, -100.0f, 100.0f);
        float right_val = Clamp_Val(F - R, -100.0f, 100.0f);

        Motor_Set_VCP(MOTOR_L_PWM_CH, MOTOR_L_IN1, MOTOR_L_IN2, left_val);
        Motor_Set_VCP(MOTOR_R_PWM_CH, MOTOR_R_IN1, MOTOR_R_IN2, right_val);
        
        static int log_cnt = 0;
        if(++log_cnt >= 50) { 
            log_cnt = 0;
            // mcu_printf("[MOTOR] F=%d R=%d\n", (int)F, (int)R);
        }
    }
}

static void Motor_HW_Init(void) {
    GPIO_Config(MOTOR_L_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_L_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    PDM_Init(); 
}

static void Encoder_HW_Init(void) {
    ICTCModeConfig_t IctcConfig; 
    memset(&IctcConfig, 0, sizeof(ICTCModeConfig_t));
    
    GPIO_Config(ENCODER_GPIO_PIN, (GPIO_FUNC(0UL) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_DS(0x3UL)));

    IctcConfig.mcTimeout        = 0x0FFFFFFFUL;
    IctcConfig.mcEnableIrq      = ICTC_IRQ_CTRL_PRDDT_CMP_ISR; 
    IctcConfig.mcEnableCounter  = ICTC_OPEN_CTRL_FLTCNT_EN | ICTC_OPEN_CTRL_PDCMPCNT_EN;
    IctcConfig.mcOperationMode  = ICTC_OPMODE_CTRL_ABS_SEL | ICTC_OPMODE_CTRL_RISING_EDGE | 
                                  ICTC_OPMODE_CTRL_TCLK_BYPASS | ICTC_OPMODE_CTRL_FLT_IMM_F_MODE | 
                                  ICTC_OPMODE_CTRL_FLT_IMM_R_MODE | ICTC_OPMODE_CTRL_PRDDT_CMP_ISR |
                                  (ENCODER_PIN_INDEX << ICTC_OPMODE_CTRL_F_IN_SEL_OFFSET);

    ICTC_Init();
    ICTC_SetCallBackFunc(ENCODER_L_ICTC_CH, (void*)&ICTC_Callback);
    ICTC_SetIRQCtrlReg(ENCODER_L_ICTC_CH, IctcConfig.mcEnableIrq);
    ICTC_SetTimeoutValue(ENCODER_L_ICTC_CH, IctcConfig.mcTimeout);
    ICTC_SetEdgeMatchingValue(ENCODER_L_ICTC_CH, 0x100, 0x100, 0xFFFF);
    ICTC_SetCompareRoundValue(ENCODER_L_ICTC_CH, 0x0FFFFFF0UL, 0x0FFFFFF0UL);
    ICTC_SetOpEnCtrlCounter(ENCODER_L_ICTC_CH, IctcConfig.mcEnableCounter);
    ICTC_SetOpModeCtrlReg(ENCODER_L_ICTC_CH, IctcConfig.mcOperationMode);
    ICTC_EnableCapture(ENCODER_L_ICTC_CH);
}

static void ICTC_Callback(uint32 uiChannel, uint32 uiPeriod, uint32 uiDuty) {
    if (uiChannel == ENCODER_L_ICTC_CH) {
        if (g_TargetF >= 0) Enc1.counter++; else Enc1.counter--;
    }
}

static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale) {
    PDMModeConfig_t pdmConfig;
    memset(&pdmConfig, 0, sizeof(PDMModeConfig_t));
    pdmConfig.mcPortNumber = GPIO_PERICH_CH0; 
    pdmConfig.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1; 
    pdmConfig.mcOutputCtrl = 1; 
    pdmConfig.mcPeriodNanoSec1 = PWM_PERIOD_NS; 
    pdmConfig.mcDutyNanoSec1 = duty_1000_scale * (PWM_PERIOD_NS / MAX_DUTY_SCALE);
    
    PDM_SetConfig(channel, &pdmConfig);
    PDM_Enable(channel, USE_PMM_MONITOR);
}

static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed) {
    uint32 duty = 0;
    float abs_speed = 0.0f;
    if (speed > 0) { GPIO_Set(p1, 1); GPIO_Set(p2, 0); abs_speed = speed; }
    else if (speed < 0) { GPIO_Set(p1, 0); GPIO_Set(p2, 1); abs_speed = -speed; }
    else { GPIO_Set(p1, 0); GPIO_Set(p2, 0); abs_speed = 0; }
    
    if(abs_speed > 100.0f) abs_speed = 100.0f;
    duty = (uint32)(abs_speed * 10.0f);
    Update_PWM_Duty(channel, duty);
}

// =========================================================
// [기타] 기존 시스템 함수들
// =========================================================
static void DisplayAliveLog(void)
{
    if (gALiveMsgOnOff != 0U) {
        if(gALiveCount >= 0xFFFFFFFF) gALiveCount = 0;
    }
}

static void DisplayOTPInfo(void)
{
    // OTP 생략
}

#endif