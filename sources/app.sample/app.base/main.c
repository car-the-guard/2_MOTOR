// SPDX-License-Identifier: Apache-2.0
/*
***************************************************************************************************
* FileName : main.c (Motor Board - FW Logic Integrated)
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
#include <ictc.h> // ICTC 헤더 필수
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
#define CAN_ID_DRIVE_CMD        (0x106)  
#define CAN_ID_SPEED_FEEDBACK   (0x13)  
#define CAN_RX_QUEUE_SIZE       (64)
#define CAN_CH_CMD              (0)

// ICTC & Encoder Macros (제공해주신 코드 반영)
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

// Parameters
#define COUNTS_PER_REV          (1600.0f) 
#define WHEEL_DIA_M             (0.065f)  
#define PI                      (3.141592f)
#define ENCODER_L_ICTC_CH       (0)         
#define ENCODER_PIN_INDEX       (64UL)      
#define ENCODER_GPIO_PIN        GPIO_GPC(4)

// Motor Pins
#define MOTOR_L_PWM_CH          (0)
#define MOTOR_L_IN1             GPIO_GPB(23)
#define MOTOR_L_IN2             GPIO_GPB(21)
#define MOTOR_R_PWM_CH          (1) // 제공 코드는 Ch8이나 RTOS환경에선 보통 1 사용 (확인필요)
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
#define STEP_VAL                (2.0f)

// Task Config
#define TASK_CAN_PRIO           (SAL_PRIO_APP_CFG + 5)
#define TASK_MOTOR_PRIO         (SAL_PRIO_APP_CFG + 2)
#define TASK_STK_SIZE           (1024)

// =========================================================
// [1] 구조체 정의
// =========================================================
typedef struct {
    int32 counter;      
    int32 last_counter;  
    // int64 total_count; (SDK 타입 지원 여부에 따라 제외 가능)
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
uint32 gALiveMsgOnOff = 1; 
static uint32 g_canRxQueueHandle = 0;

Encoder_t Enc1; // 엔코더 인스턴스

volatile int32 g_TargetF = 0;   
volatile int32 g_TargetR = 0;   

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

    mcu_printf("\n[MOTOR BOARD] START (FW Logic Ported)\n");

    (void)SAL_TaskCreate(&AppTaskStartID, (const uint8 *)"StartTask", (SALTaskFunc)&Main_StartTask,
                                     &AppTaskStartStk[0], ACFG_TASK_MEDIUM_STK_SIZE, SAL_PRIO_APP_CFG, NULL);
    (void)SAL_OsStart();
}

static void Main_StartTask(void * pArg)
{
    (void)pArg;
    (void)SAL_OsInitFuncs();
    AppTaskCreate();
    while (1) { (void)SAL_TaskSleep(5000); }
}

static void AppTaskCreate(void)
{
    static uint32 TaskCanID, TaskCanStk[TASK_STK_SIZE];
    static uint32 TaskMotorID, TaskMotorStk[TASK_STK_SIZE];

    // 하드웨어 초기화
    Motor_HW_Init();
    Encoder_HW_Init(); // [중요] 엔코더(ICTC) 설정

    SAL_QueueCreate(&g_canRxQueueHandle, (const uint8 *)"CAN_Q", CAN_RX_QUEUE_SIZE, sizeof(CAN_RxPacket_t));
    Init_CAN_HW();

    SAL_TaskCreate(&TaskCanID, (const uint8 *)"CAN Task", (SALTaskFunc)&Task_CAN_Rx, 
                   &TaskCanStk[0], TASK_STK_SIZE, TASK_CAN_PRIO, NULL);

    SAL_TaskCreate(&TaskMotorID, (const uint8 *)"Motor Task", (SALTaskFunc)&Task_MotorControl, 
                   &TaskMotorStk[0], TASK_STK_SIZE, TASK_MOTOR_PRIO, NULL);
}

// ---------------------------------------------------------
// Task 1: CAN 수신
// ---------------------------------------------------------
static void Task_CAN_Rx(void *pArg)
{
    (void)pArg;
    CAN_RxPacket_t rxPacket;
    uint32 uiSizeCopied;

    mcu_printf("[Task] CAN Rx Ready\n");

    while(1)
    {
        if (SAL_QueueGet(g_canRxQueueHandle, &rxPacket, &uiSizeCopied, 0, SAL_OPT_BLOCKING) == SAL_RET_SUCCESS)
        {
            if (rxPacket.id == CAN_ID_DRIVE_CMD) 
            {
                g_TargetF = (char)rxPacket.data[0];
                g_TargetR = (char)rxPacket.data[3] - (char)rxPacket.data[2];
                mcu_printf("[CMD] F:%d\n", g_TargetF);
            }
        }
    }
}

// ---------------------------------------------------------
// Task 2: 모터 제어 및 엔코더 업데이트 (20ms 주기)
// ---------------------------------------------------------
static void Task_MotorControl(void *pArg)
{
    (void)pArg;
    mcu_printf("[Task] Motor Control Ready\n");

    while(1)
    {
        // 1. 모터 구동
        Car_Update_Logic();

        // 2. 엔코더 업데이트 (주기: 20ms)
        // FW 코드 로직 그대로 사용
        Encoder_Update(&Enc1, 20.0f); 

        // 3. CAN 피드백 전송 (m/s -> cm/s 변환)
        // 왼쪽만 엔코더가 있다면 오른쪽은 0 또는 동일 값 전송
        CAN_Send_Feedback((int16)(Enc1.speed_mps * 100.0f), 0);

        // 4. 주기 설정 (20ms)
        SAL_TaskSleep(20); 
    }
}

// =========================================================
// [5] 엔코더 로직 (FW 코드 이식)
// =========================================================
static void Encoder_Update(Encoder_t *enc, float dt_ms)
{
    if (dt_ms < 0.1f) return;

    int32 cur_counter = enc->counter;
    int32 diff = cur_counter - enc->last_counter;

    enc->last_counter = cur_counter;
    
    // RPM 및 속도 계산 (제공된 수식)
    enc->speed_rpm = ((float)diff / COUNTS_PER_REV) * (60000.0f / dt_ms);
    enc->speed_mps = (enc->speed_rpm * PI * WHEEL_DIA_M) / 60.0f;
    
    // [디버그용 로그 - 필요시 주석 해제]
    mcu_printf("Cnt:%d Spd:%d\n", cur_counter, (int)(enc->speed_mps*100));
}

// ICTC 인터럽트 콜백 (펄스 카운팅)
static void ICTC_Callback(uint32 uiChannel, uint32 uiPeriod, uint32 uiDuty)
{
    (void)uiPeriod; (void)uiDuty;
    
    if (uiChannel == ENCODER_L_ICTC_CH) {
        // FW 코드 로직: 목표 속도 부호에 따라 증감 결정
        if (g_TargetF >= 0) Enc1.counter++;
        else Enc1.counter--;
    }
}

// =========================================================
// [6] 하드웨어 초기화
// =========================================================
static void Encoder_HW_Init(void)
{
    ICTCModeConfig_t IctcConfig; 
    memset(&IctcConfig, 0, sizeof(ICTCModeConfig_t));

    Enc1.counter = 0; Enc1.last_counter = 0; 

    // GPC4 설정 (Pull-up)
    #ifndef GPIO_PULLUP
        #define GPIO_PULLUP 0 
    #endif
    GPIO_Config(ENCODER_GPIO_PIN, (GPIO_FUNC(0UL) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_DS(0x3UL) | GPIO_PULLUP));

    // FW 코드 설정값 그대로 적용
    IctcConfig.mcTimeout        = 0x0FFFFFFFUL;
    IctcConfig.mcEnableIrq      = ICTC_IRQ_CTRL_PRDDT_CMP_ISR; 
    IctcConfig.mcEnableCounter  = ICTC_OPEN_CTRL_FLTCNT_EN | ICTC_OPEN_CTRL_PDCMPCNT_EN;
    IctcConfig.mcOperationMode  = ICTC_OPMODE_CTRL_ABS_SEL | ICTC_OPMODE_CTRL_RISING_EDGE | 
                                  ICTC_OPMODE_CTRL_TCLK_BYPASS | ICTC_OPMODE_CTRL_FLT_IMM_F_MODE | 
                                  ICTC_OPMODE_CTRL_FLT_IMM_R_MODE | ICTC_OPMODE_CTRL_PRDDT_CMP_ISR |
                                  (ENCODER_PIN_INDEX << ICTC_OPMODE_CTRL_F_IN_SEL_OFFSET);

    ICTC_Init();
    
    // 콜백 등록
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

// =========================================================
// [7] 유틸리티 및 로직
// =========================================================
static void CAN_Send_Feedback(int16 speedL, int16 speedR)
{
    CANMessage_t txMsg;
    uint8 ucTxBufferIndex;
    
    memset(&txMsg, 0, sizeof(CANMessage_t));
    txMsg.mId = CAN_ID_SPEED_FEEDBACK; 
    txMsg.mDataLength = 8;
    txMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;
    // txMsg.mIdType = CAN_ID_TYPE_STANDARD; // SDK에 따라 필요시 주석 해제

    txMsg.mData[0] = (uint8)(speedL & 0xFF);
    txMsg.mData[1] = (uint8)((speedL >> 8) & 0xFF);
    txMsg.mData[2] = (uint8)(speedR & 0xFF);
    txMsg.mData[3] = (uint8)((speedR >> 8) & 0xFF);

    (void)CAN_SendMessage(CAN_CH_CMD, &txMsg, &ucTxBufferIndex);
}

static void Car_Update_Logic(void)
{
    curF = rate_limit(curF, (float)g_TargetF, STEP_VAL);
    curR = rate_limit(curR, (float)g_TargetR, STEP_VAL);

    float left_val  = Clamp_Val(curF + (curR * TURN_SENSITIVITY), -100.0f, 100.0f);
    float right_val = Clamp_Val(curF - (curR * TURN_SENSITIVITY), -100.0f, 100.0f);

    Motor_Set_VCP(MOTOR_L_PWM_CH, MOTOR_L_IN1, MOTOR_L_IN2, left_val);
    Motor_Set_VCP(MOTOR_R_PWM_CH, MOTOR_R_IN1, MOTOR_R_IN2, right_val);
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

// Utils
static float Clamp_Val(float x, float lo, float hi) {
    if (x < lo) return lo; if (x > hi) return hi; return x;
}
static float rate_limit(float cur, float target, float step) {
    if (target > cur + step) return cur + step;
    if (target < cur - step) return cur - step;
    return target;
}

// CAN Callbacks
static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError)
{
    CANMessage_t sRxMsg; CAN_RxPacket_t qPacket;
    (void)uiRxIndex; (void)uiRxBufferType;
    if (uiError == CAN_ERROR_NONE) {
        if (CAN_GetNewRxMessage(ucCh, &sRxMsg) == CAN_ERROR_NONE) {
            qPacket.id = sRxMsg.mId; qPacket.dlc = sRxMsg.mDataLength;
            memcpy(qPacket.data, sRxMsg.mData, 8);
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