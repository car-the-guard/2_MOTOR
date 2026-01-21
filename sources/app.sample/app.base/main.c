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

// =========================================================
// [0] 매크로 및 상수 정의
// =========================================================
#define FLOAT_TO_INT(x)  (int)(x), (int)(((x) - (int)(x)) * 100.0f)

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

// UART
#define BLUETOOTH_UART_CH       (0)             
#define HC06_BAUDRATE           (9600)          
#define CMD_BUF_SIZE            (64)   

// PWM
#define USE_PMM_MONITOR         (0UL) 
#ifdef GPIO_PERICH_CH0
    #undef GPIO_PERICH_CH0
#endif
#define GPIO_PERICH_CH0         (0UL)

// Motor Pins
#define MOTOR_L_PWM_CH          (0)             
#define MOTOR_L_IN1             GPIO_GPB(23)     
#define MOTOR_L_IN2             GPIO_GPB(21)     

// Right Motor (Ch 8 = GPA_18)
#define MOTOR_R_PWM_CH          (8)             
#define MOTOR_R_IN1             GPIO_GPB(24)     
#define MOTOR_R_IN2             GPIO_GPB(22)     

// Encoder Pin (GPC_04)
#define ENCODER_L_ICTC_CH       (0)         
#define ENCODER_PIN_INDEX       (64UL)      
#define ENCODER_GPIO_PIN        GPIO_GPC(4)

// Parameters
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

volatile int32 g_TargetF = 0;   
volatile int32 g_TargetR = 0;   

static uint32 g_LastDutyL = 0xFFFF;
static uint32 g_LastDutyR = 0xFFFF;

uint32 gALiveMsgOnOff;

// =========================================================
// [3] 함수 프로토타입
// =========================================================
static void Delay_Loop(volatile uint32 count) { while (count--) { __asm("nop"); }}
static int32 Simple_Atoi(char *str);
static float Clamp_Val(float x, float lo, float hi);

static void Motor_HW_Init(void);
static void Encoder_HW_Init(void);
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale);
static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed);
static void Car_Update_Logic(void);
static void Parse_Command(char *cmd);
static void Encoder_Update(Encoder_t *enc, float dt_ms);
static void ICTC_Callback(uint32 uiChannel, uint32 uiPeriod, uint32 uiDuty);

// Dummy
static void AppTaskCreate(void) {}
static void DisplayAliveLog(void) {}
static void DisplayOTPInfo(void) {}

// =========================================================
// [4] 유틸리티
// =========================================================
static int32 Simple_Atoi(char *str) {
    int32 res = 0, i = 0, sign = 1;
    if(str[0] == '-') { sign = -1; i++; }
    while (str[i] >= '0' && str[i] <= '9') {
        res = res * 10 + (str[i] - '0'); i++;
    }
    return res * sign;
}

// [Fix] Clamp_Val 포맷 수정 (경고 해결)
static float Clamp_Val(float x, float lo, float hi) {
    if (x < lo) return lo; 
    if (x > hi) return hi; 
    return x;
}

// =========================================================
// [5] 엔코더 업데이트
// =========================================================
static void Encoder_Update(Encoder_t *enc, float dt_ms)
{
    if (dt_ms < 0.1f) return;

    int32_t cur_counter = enc->counter;
    int32_t diff = cur_counter - enc->last_counter;

    enc->last_counter = cur_counter;
    enc->total_count += diff;

    // RPM 및 속도 계산
    enc->speed_rpm = ((float)diff / COUNTS_PER_REV) * (60000.0f / dt_ms);
    enc->speed_mps = (enc->speed_rpm * PI * WHEEL_DIA_M) / 60.0f;
}

// =========================================================
// [6] 하드웨어 초기화
// =========================================================
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

static void ICTC_Callback(uint32 uiChannel, uint32 uiPeriod, uint32 uiDuty)
{
    if (uiChannel == ENCODER_L_ICTC_CH) {
        if (g_TargetF >= 0) Enc1.counter++;
        else Enc1.counter--;
    }
}

// =========================================================
// [7] 모터 제어
// =========================================================
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale)
{
    PDMModeConfig_t pdmConfig;
    uint32 *lastDutyPtr = (channel == MOTOR_L_PWM_CH) ? &g_LastDutyL : &g_LastDutyR;
    uint32 wait_cnt = 0; 

    if (*lastDutyPtr == duty_1000_scale) return;
    *lastDutyPtr = duty_1000_scale;

    if (duty_1000_scale > 0) {
        mcu_printf("[PWM] Ch:%d Duty:%d\n", channel, duty_1000_scale);
    }

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

void Parse_Command(char *cmd)
{
    int32 i = 0, val = 0;
    while (cmd[i] != '\0') {
        if (cmd[i] == 'F') { val = Simple_Atoi(&cmd[i + 1]); g_TargetF = val; }
        else if (cmd[i] == 'B') { val = Simple_Atoi(&cmd[i + 1]); g_TargetF = -val; }
        else if (cmd[i] == 'R') { val = Simple_Atoi(&cmd[i + 1]); g_TargetR = val; }
        else if (cmd[i] == 'L') { val = Simple_Atoi(&cmd[i + 1]); g_TargetR = -val; }
        i++;
    }
}

// =========================================================
// [8] 메인 (수정됨: 시간 측정 방식 변경)
// =========================================================
void cmain(void)
{
    UartParam_t uartParam;
    static uint8 rx_char;
    static char cmd_buf[CMD_BUF_SIZE];
    static uint8 cmd_len = 0;

    // 루프 카운터 (타이머 대신 사용)
    volatile uint32 loop_cnt = 0; 

    (void)SAL_Init();
    BSP_PreInit();
    BSP_Init(); 

    Motor_HW_Init();
    Encoder_HW_Init();

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

    mcu_printf("\n[VCP] Force Print Mode (Loop Counter)\n");

    while (1)
    {
        // 1. UART
        if (UART_Read(BLUETOOTH_UART_CH, &rx_char, 1) > 0)
        {
            if (rx_char == '\n' || rx_char == '\r') {
                if (cmd_len > 0) {
                    cmd_buf[cmd_len] = '\0';
                    //mcu_printf("[BT RX] %s\n", cmd_buf);
                    Parse_Command(cmd_buf);
                    cmd_len = 0;
                }
            } else {
                if (cmd_len < 63) cmd_buf[cmd_len++] = (char)rx_char;
                else cmd_len = 0; 
            }
        }

        // 2. 모터 제어
        Car_Update_Logic();

        // 3. 엔코더 출력 (타이머 함수 제거 -> 카운터 방식)
        loop_cnt++;
        
        // 20000번 루프마다 한 번씩 출력 (VCP 칩이 빠르니 숫자를 크게 잡음)
        if (loop_cnt >= 20000) 
        {
            loop_cnt = 0; // 카운터 초기화

            // dt를 100ms(0.1s)라고 가정하고 계산
            // (속도값이 정확하지 않아도, 일단 변화량을 보기 위함)
            Encoder_Update(&Enc1, 100.0f); 

            // ★ 디버깅: 원본 카운터값(counter)과 계산된 속도 출력
            mcu_printf("Enc: %d | Spd: %d.%02d m/s\n", 
                       Enc1.counter,
                       FLOAT_TO_INT(Enc1.speed_mps));
        }

        Delay_Loop(100); // 딜레이를 줄여서 루프 회전수 확보
    }
}

#endif