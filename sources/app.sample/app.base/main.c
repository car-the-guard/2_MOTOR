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
#include <uart.h>
#include <gpio.h>
#include <pdm.h>
#include <ictc.h>
#include <string.h>
#include <stdint.h> 

// #include <can.h>
// #include <can_drv.h>

// CAN Drivers
#include "can_config.h"  
#include "can_reg.h"    
#include "can.h"
#include "can_drv.h"

// =========================================================
// [0] 매크로 및 상수 정의
// =========================================================
#define FLOAT_TO_INT(x)  (int)(x), (int)(((x) - (int)(x)) * 100.0f)

// ★ [CAN 프로토콜 설정] 센터 보드와 맞춰야 함!
#define CAN_CHANNEL             (0)         // CAN 채널
#define CAN_ID_TX_SPEED         (0x101)     // 송신: 현재 속도 (VCP -> Center)
#define CAN_ID_RX_CMD           (0x111)     // 수신: 주행 명령 (Center -> VCP)

// ★ [안전장치] 통신 끊김 감지 시간 (ms)
#define CAN_TIMEOUT_MS          (500)       

// --- 기존 ICTC 매크로 ---
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

// UART & Hardware Config
#define BLUETOOTH_UART_CH       (0)             
#define HC06_BAUDRATE           (9600)          
#define CMD_BUF_SIZE            (64)   
#define USE_PMM_MONITOR         (0UL) 
#ifdef GPIO_PERICH_CH0
    #undef GPIO_PERICH_CH0
#endif
#define GPIO_PERICH_CH0         (0UL)

// Motors
#define MOTOR_L_PWM_CH          (0)             
#define MOTOR_L_IN1             GPIO_GPB(23)     
#define MOTOR_L_IN2             GPIO_GPB(21)     
#define MOTOR_R_PWM_CH          (8)             
#define MOTOR_R_IN1             GPIO_GPB(24)     
#define MOTOR_R_IN2             GPIO_GPB(22)     

// Encoder
#define ENCODER_L_ICTC_CH       (0)         
#define ENCODER_PIN_INDEX       (64UL)      
#define ENCODER_GPIO_PIN        GPIO_GPC(4)

// Control Params
#define PWM_PERIOD_NS           (1000000UL)     
#define MAX_DUTY_SCALE          (1000UL) 
#define TURN_SENSITIVITY        (1.0f)          
#define COUNTS_PER_REV          (1600.0f) 
#define WHEEL_DIA_M             (0.065f)  
#define PI                      (3.141592f)

// =========================================================
// [1] 구조체 정의
// =========================================================
typedef struct {
    int32_t counter;       
    int32_t last_counter;  
    int64_t total_count;   
    float speed_rpm;       
    float speed_mps;       
    int32_t direction;     
} Encoder_t;

// =========================================================
// [2] 전역 변수
// =========================================================
Encoder_t Enc1;

// 제어 목표값 (CAN에 의해 업데이트됨)
volatile int32 g_TargetF = 0;   
volatile int32 g_TargetR = 0;   

static uint32 g_LastDutyL = 0xFFFF;
static uint32 g_LastDutyR = 0xFFFF;

// CAN 수신 버퍼 및 플래그
volatile uint8_t g_CanRxFlag = 0;
CANMessage_t g_RxMsgBuffer;

// 안전장치용 타이머
uint32 g_LastCanCmdTick = 0; 

// =========================================================
// [3] 함수 프로토타입
// =========================================================
static void Delay_Loop(volatile uint32 count) { while (count--) { __asm("nop"); }}
static float Clamp_Val(float x, float lo, float hi);

static void Motor_HW_Init(void);
static void Encoder_HW_Init(void);
static void CAN_HW_Init(void);

static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale);
static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed);
static void Car_Update_Logic(void);
static void Encoder_Update(Encoder_t *enc, float dt_ms);
static void ICTC_Callback(uint32 uiChannel, uint32 uiPeriod, uint32 uiDuty);

// CAN Functions
static void CAN_TxCallback(uint8 ucCh, CANTxInterruptType_t uiIntType);
static void CAN_RxCallback(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError);
static void CAN_ErrorCallback(uint8 ucCh, CANErrorType_t uiError);
static void CAN_Send_Speed(float speed_mps);
static void CAN_Parse_Command(void); // [New] 수신 메시지 해석

// =========================================================
// [4] 함수 구현
// =========================================================
static float Clamp_Val(float x, float lo, float hi) {
    if (x < lo) return lo; if (x > hi) return hi; return x;
}

static void Encoder_Update(Encoder_t *enc, float dt_ms)
{
    if (dt_ms < 0.1f) return;
    int32_t cur_counter = enc->counter;
    int32_t diff = cur_counter - enc->last_counter;
    enc->last_counter = cur_counter;
    enc->total_count += diff;
    enc->speed_rpm = ((float)diff / COUNTS_PER_REV) * (60000.0f / dt_ms);
    enc->speed_mps = (enc->speed_rpm * PI * WHEEL_DIA_M) / 60.0f;
}

void Motor_HW_Init(void)
{
    GPIO_Config(MOTOR_L_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_L_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    PDM_Init(); 
    g_LastDutyL = 0xFFFF; g_LastDutyR = 0xFFFF;
    Update_PWM_Duty(MOTOR_L_PWM_CH, 0);
    Update_PWM_Duty(MOTOR_R_PWM_CH, 0);
}

static void Encoder_HW_Init(void)
{
    ICTCModeConfig_t IctcConfig; 
    memset(&IctcConfig, 0, sizeof(ICTCModeConfig_t));
    Enc1.counter = 0; Enc1.last_counter = 0; Enc1.total_count = 0; Enc1.direction = 1;

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
    ICTC_SetCallBackFunc(ENCODER_L_ICTC_CH, (void*)&ICTC_Callback);
    ICTC_SetIRQCtrlReg(ENCODER_L_ICTC_CH, IctcConfig.mcEnableIrq);
    ICTC_SetTimeoutValue(ENCODER_L_ICTC_CH, IctcConfig.mcTimeout);
    ICTC_SetEdgeMatchingValue(ENCODER_L_ICTC_CH, 0x100, 0x100, 0xFFFF);
    ICTC_SetCompareRoundValue(ENCODER_L_ICTC_CH, 0x0FFFFFF0UL, 0x0FFFFFF0UL);
    ICTC_SetOpEnCtrlCounter(ENCODER_L_ICTC_CH, IctcConfig.mcEnableCounter);
    ICTC_SetOpModeCtrlReg(ENCODER_L_ICTC_CH, IctcConfig.mcOperationMode);
    ICTC_EnableCapture(ENCODER_L_ICTC_CH);
    mcu_printf("[Init] Encoder Initialized (Pin: GPC_04)\n");
}

static void CAN_HW_Init(void)
{
    CANErrorType_t ret;
    ret = CAN_Init();
    if (ret != CAN_ERROR_NONE) {
        CAN_Deinit();
        ret = CAN_Init();
    }
    CAN_RegisterCallbackFunctionTx(CAN_TxCallback);
    CAN_RegisterCallbackFunctionRx(CAN_RxCallback);
    CAN_RegisterCallbackFunctionError(CAN_ErrorCallback);
    mcu_printf("[Init] CAN Initialized (CH: %d)\n", CAN_CHANNEL);
}

static void ICTC_Callback(uint32 uiChannel, uint32 uiPeriod, uint32 uiDuty)
{
    if (uiChannel == ENCODER_L_ICTC_CH) {
        if (g_TargetF >= 0) Enc1.counter++;
        else Enc1.counter--;
    }
}

// =========================================================
// [7] CAN Callback & Handlers
// =========================================================
static void CAN_TxCallback(uint8 ucCh, CANTxInterruptType_t uiIntType) { (void)ucCh; (void)uiIntType; }
static void CAN_ErrorCallback(uint8 ucCh, CANErrorType_t uiError) { (void)ucCh; (void)uiError; }

static void CAN_RxCallback(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError)
{
    if (uiError == CAN_ERROR_NONE) {
        CAN_GetNewRxMessage(CAN_CHANNEL, &g_RxMsgBuffer);
        g_CanRxFlag = 1; // Flag set
    }
}

// ★ [핵심] CAN 메시지 파싱 및 제어값 업데이트
static void CAN_Parse_Command(void)
{
    // 센터 보드에서 보낸 주행 명령 ID인지 확인
    if (g_RxMsgBuffer.mId == CAN_ID_RX_CMD) // 0x111
    {
        // Data[0,1]: Forward (int16)
        // Data[2,3]: Rotation (int16)
        
        int16_t cmd_fwd = (int16_t)(g_RxMsgBuffer.mData[0] | (g_RxMsgBuffer.mData[1] << 8));
        int16_t cmd_rot = (int16_t)(g_RxMsgBuffer.mData[2] | (g_RxMsgBuffer.mData[3] << 8));

        // 전역 변수 업데이트 -> Car_Update_Logic에서 반영됨
        g_TargetF = (int32)cmd_fwd;
        g_TargetR = (int32)cmd_rot;

        // Watchdog 리셋 (명령을 받았으므로 시간 갱신)
        SAL_GetTickCount(&g_LastCanCmdTick);

        // 디버깅 (필요시 주석 해제)
        // mcu_printf("CAN CMD: F=%d, R=%d\n", cmd_fwd, cmd_rot);
    }
}

static void CAN_Send_Speed(float speed_mps)
{
    CANMessage_t sTxMsg;
    uint8 ucTxBufferIndex;
    int16_t speed_cmps = (int16_t)(speed_mps * 100.0f);

    SAL_MemSet(&sTxMsg, 0, sizeof(CANMessage_t));
    sTxMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;
    sTxMsg.mId = CAN_ID_TX_SPEED; // 0x101
    sTxMsg.mExtendedId = 0;
    sTxMsg.mDataLength = 8;

    sTxMsg.mData[0] = (uint8_t)(speed_cmps & 0xFF);
    sTxMsg.mData[1] = (uint8_t)((speed_cmps >> 8) & 0xFF);

    CAN_SendMessage(CAN_CHANNEL, &sTxMsg, &ucTxBufferIndex);
}

// =========================================================
// [8] 모터 제어
// =========================================================
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

void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed)
{
    uint32 duty = 0;
    float abs_speed = 0.0f;
    if (speed > 0) { GPIO_Set(p1, 1); GPIO_Set(p2, 0); abs_speed = speed; }
    else if (speed < 0) { GPIO_Set(p1, 0); GPIO_Set(p2, 1); abs_speed = -speed; }
    else { GPIO_Set(p1, 0); GPIO_Set(p2, 0); abs_speed = 0; }
    duty = (uint32)(abs_speed * 10.0f);
    Update_PWM_Duty(channel, duty);
}

void Car_Update_Logic(void)
{
    float F = (float)g_TargetF;
    float R = (float)g_TargetR;
    float left_val  = Clamp_Val(F + (R * TURN_SENSITIVITY), -100.0f, 100.0f);
    float right_val = Clamp_Val(F - (R * TURN_SENSITIVITY), -100.0f, 100.0f);
    Motor_Set_VCP(MOTOR_L_PWM_CH, MOTOR_L_IN1, MOTOR_L_IN2, left_val);
    Motor_Set_VCP(MOTOR_R_PWM_CH, MOTOR_R_IN1, MOTOR_R_IN2, right_val);
}

// =========================================================
// [9] Main Loop
// =========================================================
void cmain(void)
{
    UartParam_t uartParam;
    static uint8 rx_char;
    uint32 current_tick = 0;
    uint32 last_tick = 0;

    (void)SAL_Init();
    BSP_PreInit();
    BSP_Init(); 

    Motor_HW_Init();
    Encoder_HW_Init();
    CAN_HW_Init();

    UART_Close(BLUETOOTH_UART_CH);
    uartParam.sCh = BLUETOOTH_UART_CH;
    uartParam.sPriority = 10U;
    uartParam.sBaudrate = HC06_BAUDRATE;
    uartParam.sMode = UART_POLLING_MODE;
    uartParam.sCtsRts = UART_CTSRTS_OFF;
    uartParam.sPortCfg = 0; 
    uartParam.sWordLength = WORD_LEN_8;
    uartParam.sFIFO = ENABLE_FIFO;
    uartParam.s2StopBit = TWO_STOP_BIT_OFF;
    uartParam.sParity = 0;
    uartParam.sFnCallback = NULL;
    UART_Open(&uartParam);

    mcu_printf("\n[VCP] CAN Control Mode (ID: 0x%X)\n", CAN_ID_RX_CMD);

    SAL_GetTickCount(&last_tick);
    g_LastCanCmdTick = last_tick; // Watchdog 초기화

    while (1)
    {
        // 1. CAN 수신 처리 (최우선)
        if (g_CanRxFlag)
        {
            g_CanRxFlag = 0;
            CAN_Parse_Command(); // 여기서 g_TargetF, g_TargetR 업데이트
        }

        // 2. Watchdog: 통신 끊기면 정지
        SAL_GetTickCount(&current_tick);
        if ((current_tick - g_LastCanCmdTick) > CAN_TIMEOUT_MS)
        {
            // 타임아웃 발생! 안전하게 정지
            if (g_TargetF != 0 || g_TargetR != 0) {
                mcu_printf("[SAFE] CAN Timeout -> Stop Motors\n");
                g_TargetF = 0;
                g_TargetR = 0;
            }
        }

        // 3. 모터 제어 (매 루프 반영)
        Car_Update_Logic();

        // 4. UART 로그 (선택사항)
        if (UART_Read(BLUETOOTH_UART_CH, &rx_char, 1) > 0) {
            // 블루투스로는 이제 제어 안 함 (로그용으로만 수신)
        }

        // 5. 주기적 엔코더 & CAN 송신 (100ms)
        if ((current_tick - last_tick) >= 100) 
        {
            float real_dt = (float)(current_tick - last_tick);
            Encoder_Update(&Enc1, real_dt);
            
            // CAN으로 현재 속도 전송
            CAN_Send_Speed(Enc1.speed_mps);

            mcu_printf("F:%d R:%d | Spd: %d.%02d m/s\n", 
                       g_TargetF, g_TargetR,
                       FLOAT_TO_INT(Enc1.speed_mps));

            last_tick = current_tick;
        }
        
        Delay_Loop(1000); 
    }
}

// SDK Dummies
static void AppTaskCreate(void) {}
static void DisplayAliveLog(void) {}
static void DisplayOTPInfo(void) {}

#endif