// SPDX-License-Identifier: Apache-2.0
/*
***************************************************************************************************
* FileName : main.c (Motor Board - RTOS Based V2.0)
* Description : VCP SDK & SAL RTOS 기반 모터 제어 및 엔코더 피드백
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <gpio.h>
#include <pdm.h>
#include <ictc.h>
#include <string.h>

// [CAN & ICTC Drivers]
#include "can_config.h"
#include "can_reg.h"
#include "can.h"
#include "can_drv.h"

// =========================================================
// [0] 매크로 및 상수 정의
// =========================================================
// CAN Config
#define CAN_ID_DRIVE_CMD        (0x106)  // 2차 프로젝트용 Command ID
#define CAN_ID_SPEED_FEEDBACK   (0x13)   // 2차 프로젝트용 Feedback ID
#define CAN_RX_QUEUE_SIZE       (64)
#define CAN_CH_CMD              (0)

// Safety Config
#define CMD_TIMEOUT_MS          (500)    // 통신 두절 판단 시간

// Parameters
#define COUNTS_PER_REV          (1600.0f) 
#define WHEEL_DIA_M             (0.065f)  
#define PI                      (3.141592f)
#define ENCODER_L_ICTC_CH       (0)         
#define ENCODER_PIN_INDEX       (64UL)      
#define ENCODER_GPIO_PIN        GPIO_GPC(4)

// Motor Pins (Hardware Dependent)
#define MOTOR_L_PWM_CH          (0)
#define MOTOR_L_IN1             GPIO_GPB(23)
#define MOTOR_L_IN2             GPIO_GPB(21)

#define MOTOR_R_PWM_CH          (1) 
#define MOTOR_R_IN1             GPIO_GPB(24)
#define MOTOR_R_IN2             GPIO_GPB(22)

#define PWM_PERIOD_NS           (1000000UL)     
#define MAX_DUTY_SCALE          (1000UL) 
#define TURN_SENSITIVITY        (1.0f)          
#define USE_PMM_MONITOR         (0UL)
#ifndef GPIO_PERICH_CH0
#define GPIO_PERICH_CH0         (0UL)
#endif

// Rate Limiter
#define STEP_VAL                (2.0f) // Slew Rate 제한

// Task Config
#define TASK_CAN_PRIO           (SAL_PRIO_APP_CFG + 5) // 통신은 우선순위 높게
#define TASK_MOTOR_PRIO         (SAL_PRIO_APP_CFG + 2) // 제어는 주기성 보장
#define TASK_STK_SIZE           (1024)
#define CTRL_PERIOD_MS          (10) // 제어 주기 10ms

// ICTC Macros (기존 유지)
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

// =========================================================
// [1] 구조체 정의
// =========================================================
typedef struct {
    int32 counter;      
    int32 last_counter;  
    float speed_rpm;        
    float speed_mps;        
} Encoder_t;

typedef struct
{
    uint32 id;
    uint32 dlc;     
    uint8  data[8];
} CAN_RxPacket_t;

// =========================================================
// [2] 전역 변수
// =========================================================
static uint32 g_canRxQueueHandle = 0;
Encoder_t Enc1; 

// Control Variables
volatile int32 g_TargetF = 0;   
volatile int32 g_TargetR = 0;   
volatile uint32 g_LastCmdTick = 0; // [Safety] 마지막 명령 수신 시각

static uint32 g_LastDutyL = 0xFFFF;
static uint32 g_LastDutyR = 0xFFFF;
static float curF = 0.0f;
static float curR = 0.0f;

// =========================================================
// [3] 함수 프로토타입
// =========================================================
static void Main_StartTask(void * pArg);
static void AppTaskCreate(void);
static void Task_CAN_Rx(void *pArg);
static void Task_MotorControl(void *pArg);

static void Init_CAN_HW(void);
static void Motor_HW_Init(void);
static void Encoder_HW_Init(void);
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale);
static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed);
static void Car_Update_Logic(void);
static void Encoder_Update(Encoder_t *enc, float dt_ms);
static void ICTC_Callback(uint32 uiChannel, uint32 uiPeriod, uint32 uiDuty);
static void CAN_Send_Feedback(int16 speedL, int16 speedR);

static float Clamp_Val(float x, float lo, float hi);
static float rate_limit(float cur, float target, float step);

// Callbacks
static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError);
static void CAN_AppCallbackTxEvent(uint8 ucCh, CANTxInterruptType_t uiIntType);
static void CAN_AppCallbackErrorEvent(uint8 ucCh, CANErrorType_t uiError);
void CAN_consume_rx_message(void *pMsg, void *pPayload); 

// =========================================================
// [4] 메인 및 태스크
// =========================================================
void cmain (void)
{
    static uint32 AppTaskStartID = 0;
    static uint32 AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    
    (void)SAL_Init();
    BSP_PreInit();
    BSP_Init();

    mcu_printf("\n[MOTOR BOARD] RTOS Project V2.0 START\n");

    (void)SAL_TaskCreate(&AppTaskStartID, (const uint8 *)"StartTask", (SALTaskFunc)&Main_StartTask,
                         &AppTaskStartStk[0], ACFG_TASK_MEDIUM_STK_SIZE, SAL_PRIO_APP_CFG, NULL);
    (void)SAL_OsStart();
}

static void Main_StartTask(void * pArg)
{
    (void)pArg;
    (void)SAL_OsInitFuncs();
    
    (void)SAL_GetTickCount((uint32 *)&g_LastCmdTick); // 초기화
    AppTaskCreate();
    
    while (1) { (void)SAL_TaskSleep(5000); }
}

static void AppTaskCreate(void)
{
    static uint32 TaskCanID, TaskCanStk[TASK_STK_SIZE];
    static uint32 TaskMotorID, TaskMotorStk[TASK_STK_SIZE];

    Motor_HW_Init();
    Encoder_HW_Init(); 

    SAL_QueueCreate(&g_canRxQueueHandle, (const uint8 *)"CAN_Q", CAN_RX_QUEUE_SIZE, sizeof(CAN_RxPacket_t));
    Init_CAN_HW();

    // CAN 수신 태스크 (Event Driven)
    SAL_TaskCreate(&TaskCanID, (const uint8 *)"CAN Task", (SALTaskFunc)&Task_CAN_Rx, 
                   &TaskCanStk[0], TASK_STK_SIZE, TASK_CAN_PRIO, NULL);

    // 모터 제어 태스크 (Time Triggered)
    SAL_TaskCreate(&TaskMotorID, (const uint8 *)"Motor Task", (SALTaskFunc)&Task_MotorControl, 
                   &TaskMotorStk[0], TASK_STK_SIZE, TASK_MOTOR_PRIO, NULL);
}

// ---------------------------------------------------------
// Task 1: CAN 수신 (Queue Pending)
// ---------------------------------------------------------
static void Task_CAN_Rx(void *pArg)
{
    (void)pArg;
    CAN_RxPacket_t rxPacket;
    uint32 uiSizeCopied;

    mcu_printf("[Task] CAN Rx Running\n");

    while(1)
    {
        // 큐에서 메시지를 기다림 (Blocking)
        if (SAL_QueueGet(g_canRxQueueHandle, &rxPacket, &uiSizeCopied, 0, SAL_OPT_BLOCKING) == SAL_RET_SUCCESS)
        {
            if (rxPacket.id == CAN_ID_DRIVE_CMD) 
            {
                // [Logic] 글로벌 변수 업데이트 (Critical Section 처리가 이상적이나, 단일 쓰기라 허용)
                g_TargetF = (int8)rxPacket.data[0];
                g_TargetR = (int8)rxPacket.data[3] - (int8)rxPacket.data[2];
                
                // [Safety] 워치독 타이머 갱신
                (void)SAL_GetTickCount((uint32 *)&g_LastCmdTick);
            }
        }
    }
}

// ---------------------------------------------------------
// Task 2: 모터 제어 (Periodic: 10ms)
// ---------------------------------------------------------
static void Task_MotorControl(void *pArg)
{
    (void)pArg;
    uint32 currentTick;
    uint32 can_tx_tick = 0;

    mcu_printf("[Task] Motor Control Running (10ms)\n");

    while(1)
    {
        (void)SAL_GetTickCount(&currentTick);

        // [Safety] Failsafe: 통신 두절 체크
        if ((currentTick - g_LastCmdTick) > CMD_TIMEOUT_MS)
        {
            g_TargetF = 0;
            g_TargetR = 0;
            // 필요 시 로그 출력 (너무 자주 찍히지 않도록 주의)
        }

        // 1. 모터 구동 로직 수행 (Rate Limit & Mixing)
        Car_Update_Logic();

        // 2. 엔코더 업데이트 (dt = 10ms)
        Encoder_Update(&Enc1, (float)CTRL_PERIOD_MS); 

        // 3. CAN 피드백 전송 (250ms 주기)
        if ((currentTick - can_tx_tick) >= 250)
        {
            CAN_Send_Feedback((int16)(Enc1.speed_mps * 100.0f), 0);
            can_tx_tick = currentTick;
        }

        // 4. 주기적 실행 (Sleep)
        SAL_TaskSleep(CTRL_PERIOD_MS); 
    }
}

// =========================================================
// [5] 로직 함수 구현
// =========================================================
static void Encoder_Update(Encoder_t *enc, float dt_ms)
{
    if (dt_ms < 0.1f) return;

    int32 cur_counter = enc->counter;
    int32 diff = cur_counter - enc->last_counter;

    enc->last_counter = cur_counter;
    
    // RPM = (펄스차이 / 1회전펄스) * (60초 / dt)
    enc->speed_rpm = ((float)diff / COUNTS_PER_REV) * (60000.0f / dt_ms);
    enc->speed_mps = (enc->speed_rpm * PI * WHEEL_DIA_M) / 60.0f;
}

// ICTC ISR Callback
static void ICTC_Callback(uint32 uiChannel, uint32 uiPeriod, uint32 uiDuty)
{
    (void)uiPeriod; (void)uiDuty;
    
    if (uiChannel == ENCODER_L_ICTC_CH) {
        // 하드웨어 방향 감지가 안되므로 목표 방향으로 추정
        if (g_TargetF >= 0) Enc1.counter++;
        else Enc1.counter--;
    }
}

static void Car_Update_Logic(void)
{
    // Rate Limiting (급가속 방지)
    curF = rate_limit(curF, (float)g_TargetF, STEP_VAL);
    curR = rate_limit(curR, (float)g_TargetR, STEP_VAL);

    // Mixing (Differential Drive)
    float left_val  = Clamp_Val(curF + (curR * TURN_SENSITIVITY), -100.0f, 100.0f);
    float right_val = Clamp_Val(curF - (curR * TURN_SENSITIVITY), -100.0f, 100.0f);

    // Hardware Output
    Motor_Set_VCP(MOTOR_L_PWM_CH, MOTOR_L_IN1, MOTOR_L_IN2, left_val);
    Motor_Set_VCP(MOTOR_R_PWM_CH, MOTOR_R_IN1, MOTOR_R_IN2, right_val);
}

// =========================================================
// [6] 하드웨어 드라이버 (VCP SDK Dependent)
// =========================================================
static void Encoder_HW_Init(void)
{
    ICTCModeConfig_t IctcConfig; 
    memset(&IctcConfig, 0, sizeof(ICTCModeConfig_t));

    Enc1.counter = 0; Enc1.last_counter = 0; 

    #ifndef GPIO_PULLUP
        #define GPIO_PULLUP 0 
    #endif
    GPIO_Config(ENCODER_GPIO_PIN, (GPIO_FUNC(0UL) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_DS(0x3UL) | GPIO_PULLUP));

    IctcConfig.mcTimeout        = 0x0FFFFFFFUL;
    IctcConfig.mcEnableIrq      = ICTC_IRQ_CTRL_PRDDT_CMP_ISR; 
    IctcConfig.mcEnableCounter  = ICTC_OPEN_CTRL_FLTCNT_EN | ICTC_OPEN_CTRL_PDCMPCNT_EN;
    IctcConfig.mcOperationMode  = ICTC_OPMODE_CTRL_ABS_SEL | ICTC_OPMODE_CTRL_RISING_EDGE | 
                                  ICTC_OPMODE_CTRL_TCLK_BYPASS | ICTC_OPMODE_CTRL_FLT_IMM_F_MODE | 
                                  ICTC_OPMODE_CTRL_FLT_IMM_R_MODE | ICTC_OPMODE_CTRL_PRDDT_CMP_ISR |
                                  (ENCODER_PIN_INDEX << ICTC_OPMODE_CTRL_F_IN_SEL_OFFSET);

    ICTC_Init();
    ICTC_SetCallBackFunc(ENCODER_L_ICTC_CH, (ICTCCallback)&ICTC_Callback);
    
    ICTC_SetIRQCtrlReg(ENCODER_L_ICTC_CH, IctcConfig.mcEnableIrq);
    ICTC_SetTimeoutValue(ENCODER_L_ICTC_CH, IctcConfig.mcTimeout);
    ICTC_SetEdgeMatchingValue(ENCODER_L_ICTC_CH, 0x100, 0x100, 0xFFFF);
    ICTC_SetCompareRoundValue(ENCODER_L_ICTC_CH, 0x0FFFFFF0UL, 0x0FFFFFF0UL);
    ICTC_SetOpEnCtrlCounter(ENCODER_L_ICTC_CH, IctcConfig.mcEnableCounter);
    ICTC_SetOpModeCtrlReg(ENCODER_L_ICTC_CH, IctcConfig.mcOperationMode);
    
    ICTC_EnableCapture(ENCODER_L_ICTC_CH);
}

void Motor_HW_Init(void)
{
    GPIO_Config(MOTOR_L_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_L_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);

    PDM_Init(); 
    Update_PWM_Duty(MOTOR_L_PWM_CH, 0);
    Update_PWM_Duty(MOTOR_R_PWM_CH, 0);
}

static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale)
{
    PDMModeConfig_t pdmConfig;
    uint32 *lastDutyPtr = (channel == MOTOR_L_PWM_CH) ? &g_LastDutyL : &g_LastDutyR;
    uint32 wait_cnt = 0; 

    if (*lastDutyPtr == duty_1000_scale) return;
    *lastDutyPtr = duty_1000_scale;

    memset(&pdmConfig, 0, sizeof(PDMModeConfig_t));
    pdmConfig.mcPortNumber      = GPIO_PERICH_CH0; 
    pdmConfig.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1; 
    pdmConfig.mcOutputCtrl      = 1; 
    pdmConfig.mcClockDivide     = 0; 
    pdmConfig.mcPeriodNanoSec1  = PWM_PERIOD_NS; 
    if(duty_1000_scale > MAX_DUTY_SCALE) duty_1000_scale = MAX_DUTY_SCALE;
    pdmConfig.mcDutyNanoSec1    = duty_1000_scale * (PWM_PERIOD_NS / MAX_DUTY_SCALE);

    PDM_Disable(channel, USE_PMM_MONITOR);
    while(PDM_GetChannelStatus(channel) == PDM_STATE_BUSY) { 
        wait_cnt++; if(wait_cnt > 10000) break; __asm("nop"); 
    }
    PDM_SetConfig(channel, &pdmConfig);
    if (duty_1000_scale > 0) PDM_Enable(channel, USE_PMM_MONITOR);
}

static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed)
{
    float abs_speed = 0.0f;
    if (speed > 0) { GPIO_Set(p1, 1); GPIO_Set(p2, 0); abs_speed = speed; }
    else if (speed < 0) { GPIO_Set(p1, 0); GPIO_Set(p2, 1); abs_speed = -speed; }
    else { GPIO_Set(p1, 0); GPIO_Set(p2, 0); abs_speed = 0; }
    
    Update_PWM_Duty(channel, (uint32)(abs_speed * 10.0f));
}

static void CAN_Send_Feedback(int16 speedL, int16 speedR)
{
    CANMessage_t txMsg;
    uint8 ucTxBufferIndex;
    
    memset(&txMsg, 0, sizeof(CANMessage_t));
    txMsg.mId = CAN_ID_SPEED_FEEDBACK; 
    txMsg.mDataLength = 8;
    txMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;

    txMsg.mData[0] = (uint8)(speedL & 0xFF);
    txMsg.mData[1] = (uint8)((speedL >> 8) & 0xFF);
    txMsg.mData[2] = (uint8)(speedR & 0xFF);
    txMsg.mData[3] = (uint8)((speedR >> 8) & 0xFF);

    (void)CAN_SendMessage(CAN_CH_CMD, &txMsg, &ucTxBufferIndex);
}

// Utils
static float Clamp_Val(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}
static float rate_limit(float cur, float target, float step) {
    if (target > cur + step) return cur + step;
    if (target < cur - step) return cur - step;
    return target;
}

// CAN Callbacks (Interrupt Context)
static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError)
{
    CANMessage_t sRxMsg; CAN_RxPacket_t qPacket;
    (void)uiRxIndex; (void)uiRxBufferType;
    if (uiError == CAN_ERROR_NONE) {
        if (CAN_GetNewRxMessage(ucCh, &sRxMsg) == CAN_ERROR_NONE) {
            qPacket.id = sRxMsg.mId; qPacket.dlc = sRxMsg.mDataLength;
            memcpy(qPacket.data, sRxMsg.mData, 8);
            // ISR에서 큐로 전송 (Non-Blocking 필수)
            (void)SAL_QueuePut(g_canRxQueueHandle, &qPacket, sizeof(CAN_RxPacket_t), 0, SAL_OPT_NON_BLOCKING);
        }
    }
}
static void CAN_AppCallbackTxEvent(uint8 ucCh, CANTxInterruptType_t uiIntType) { (void)ucCh; (void)uiIntType; }
static void CAN_AppCallbackErrorEvent(uint8 ucCh, CANErrorType_t uiError) { (void)ucCh; (void)uiError; }
static void Init_CAN_HW(void) {
    CAN_RegisterCallbackFunctionTx(&CAN_AppCallbackTxEvent);
    CAN_RegisterCallbackFunctionRx(&CAN_AppCallbackRxEvent);
    CAN_RegisterCallbackFunctionError(&CAN_AppCallbackErrorEvent);
    CAN_Init();
}
void CAN_consume_rx_message(void *pMsg, void *pPayload) { (void)pMsg; (void)pPayload; }

#endif