/*
 * =============================================================
 * Drawing Application - MCU Side (MCXC444 + FreeRTOS)
 * =============================================================
 *
 * Project Setup (MCUXpresso):
 *   1. New C/C++ Project -> FRDM-MCXC444
 *   2. SDK Components:
 *      - Operating System -> RTOS
 *      - Others -> RTOS -> Heap -> FreeRTOS heap 4 (UNCHECK heap 5)
 *      - Project Template -> RTOS
 *   3. Delete generated main code, paste this file.
 *   4. In FreeRTOSConfig_Gen.h set:
 *      - configUSE_TIME_SLICING  1
 *      - configUSE_PREEMPTION    1
 *
 * Sensors (all on MCU, directly wired):
 *   - 2x Rotary Encoders  -> pen X/Y   (interrupt-driven)
 *   - External Button      -> pen up/down (interrupt-driven)
 *   - Touch Sensor (DO)    -> submit     (polled)
 *   - MPU-6050 Gyro/Accel  -> erase      (polled via I2C)
 *
 * Actuators (on MCU):
 *   - SMD RGB LED  -> pen-state / rating feedback
 *   - Passive Buzzer -> boundary-hit warning tone
 *
 * Communication:
 *   UART2 <-> ESP32 (9600 baud, bidirectional, packetized)
 *   MCU -> ESP32:  $S,x,y,penDown,erase,submit,promptRequest\n
 *   ESP32 -> MCU:  $C,RESULT,<0|1>\n   $C,PROMPT,1\n
 *
 * =============================================================
 * PIN MAP - ACTIVE-LOW / ACTIVE-HIGH assumptions noted.
 *           VERIFY against your FRDM-MCXC444 header pinout.
 * =============================================================
 *
 *  Function          Pin      Port   Notes
 *  ----------------  -------  -----  -------------------------
 *  Encoder 1 CLK     PTA5    PORTA  Interrupt (falling edge)
 *  Encoder 2 CLK     PTA13   PORTA  Interrupt (falling edge)
 *  Button             PTA12   PORTA  Interrupt (rising edge) KY-004
 *  Encoder 1 DT      PTC1    PORTC  GPIO input (read in ISR)
 *  Encoder 2 DT      PTC2    PORTC  GPIO input (read in ISR)
 *  Touch Sensor DO   PTB0    PORTB  GPIO input (polled, HIGH=touched)
 *  LED Red            PTD4    PORTD  GPIO output
 *  LED Green          PTD2    PORTD  GPIO output
 *  LED Blue           PTD6    PORTD  GPIO output
 *  Buzzer (active)    PTE20   PORTE  GPIO output (boundary beep)
 *  Buzzer (passive)   PTE30   PORTE  GPIO output (result tunes)
 *  UART2 TX           PTE22   PORTE  MUX 4 -> ESP32 RX
 *  UART2 RX           PTE23   PORTE  MUX 4 -> ESP32 TX
 *  I2C0 SCL           PTB2    PORTB  MUX 2 -> MPU-6050 SCL
 *  I2C0 SDA           PTB3    PORTB  MUX 2 -> MPU-6050 SDA
 *
 *  Also connect:
 *    - MCU GND <-> ESP32 GND (common ground!)
 *    - MPU-6050 VCC -> 3.3 V, GND -> GND
 *    - Encoders / touch / button VCC -> 3.3 V, GND -> GND
 *
 * =============================================================
 */

/* -- Standard Libraries -------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* -- NXP Board Support --------------------------------------- */
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* -- FreeRTOS ------------------------------------------------ */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* =============================================================
 * SECTION 1 - Pin Definitions
 * =============================================================
 * Change these if your wiring differs.
 */

/* Encoder 1 (X-axis) */
#define ENC1_CLK_PIN    5       /* PTA5 - interrupt source     */
#define ENC1_DT_PIN     1       /* PTC1 - direction sense      */

/* Encoder 2 (Y-axis) */
#define ENC2_CLK_PIN    13      /* PTA13 - interrupt source    */
#define ENC2_DT_PIN     2       /* PTC2 - direction sense      */

/* External push-button (pen up / pen down) */
#define BTN_PIN         12      /* PTA12 - external KY-004 button */

/* Touch sensor digital output */
#define TOUCH_DO_PIN    0       /* PTB0 - polled               */

/* SMD RGB LED (common-cathode assumed: HIGH = ON)
 * If yours is common-anode, swap PSOR <-> PCOR in setLedColor() */
#define LED_R_PIN       2       /* PTD2 */
#define LED_G_PIN       4       /* PTD4 */
#define LED_B_PIN       6       /* PTD6 */

/* Active buzzer (boundary beep) - just needs HIGH/LOW */
#define ACTIVE_BUZZER_PIN   20      /* PTE20 */

/* Passive buzzer (result tunes) - needs square wave */
#define PASSIVE_BUZZER_PIN  30      /* PTE30 */

/* UART2 to ESP32 */
#define UART_TX_PIN     22      /* PTE22, MUX 4 */
#define UART_RX_PIN     23      /* PTE23, MUX 4 */

/* I2C0 to MPU-6050 */
#define I2C_SCL_PIN     2       /* PTB2, MUX 2 */
#define I2C_SDA_PIN     3       /* PTB3, MUX 2 */

/* Set to 0 to disable gyroscope code (for testing without hardware) */
#define ENABLE_GYRO     1

/* =============================================================
 * SECTION 2 - Application Constants
 * ============================================================= */

#define BAUD_RATE           115200
#define UART2_INT_PRIO      128     /* Lower urgency than GPIO */

#define MAX_MSG_LEN         128
#define QUEUE_LEN           10

#define CANVAS_WIDTH        128
#define CANVAS_HEIGHT       128
#define CANVAS_CENTER_X     (CANVAS_WIDTH / 2)
#define CANVAS_CENTER_Y     (CANVAS_HEIGHT / 2)

/* MPU-6050 I2C address (AD0 pin LOW -> 0x68) */
#define MPU6050_ADDR            0x68
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

/* Shake detection: delta-based on HIGH bytes only (avoids tearing noise).
 * Each axis high byte is 8-bit: ±128 counts, 1 count = 256 raw = ~0.12g.
 * At rest delta ~ 0-5.  A firm shake produces delta > 40.  */
#define SHAKE_THRESHOLD     40
#define SHAKE_COOLDOWN_MS   1000    /* ignore shakes for 1s after trigger */

/* Timing */
#define DEBOUNCE_MS         200
#define SENSOR_POLL_MS      100
#define GYRO_POLL_DIVIDER   2       /* read gyro every 2nd poll = 200ms */
#define UART_SEND_MS        5
#define BUZZER_TONE_TICKS   1       /* ~ 1 ms half-period -> ~500 Hz */
#define BUZZER_DURATION     50      /* half-periods per beep        */
#define ERASE_RETRY_FRAMES  20      /* keep erase asserted up to 100ms unless ESP32 ACKs */

/* =============================================================
 * SECTION 3 - Data Types
 * ============================================================= */

/* Encoder event pushed from ISR -> encoder task via queue */
typedef struct {
    int8_t  delta;   /* +1 or -1                */
    uint8_t axis;    /* 0 = X (enc1), 1 = Y (enc2) */
} EncoderEvent_t;

/* UART message pushed from ISR -> recv task via queue */
typedef struct {
    char message[MAX_MSG_LEN];
} UartMessage_t;

/* Shared pen state (protected by mutex) */
typedef struct {
    int16_t x;
    int16_t y;
    uint8_t penDown;    /* 1 = drawing, 0 = lifted          */
    uint8_t erase;      /* 1 = erase requested, retry until ACK/timeout */
    uint8_t submit;     /* 1 = submit requested, held until RESULT */
    uint8_t promptRequest; /* 1 = request next prompt, held until PROMPT */
    uint8_t eraseRetryFrames;
} PenState_t;

/* LED colour helper */
typedef enum {
    LED_OFF,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_PURPLE,
    LED_ORANGE,
    LED_WHITE
} LedColor_t;

typedef enum {
    ROUND_PHASE_FREE_DRAW = 0,
    ROUND_PHASE_WAITING_GUESS,
    ROUND_PHASE_RESULT_LOCKED,
    ROUND_PHASE_WAITING_PROMPT,
} RoundPhase_t;

/* =============================================================
 * SECTION 4 - Global Handles & Shared State
 * ============================================================= */

/* FreeRTOS synchronisation objects */
static QueueHandle_t      encoderQueue;     /* EncoderEvent_t   */
static QueueHandle_t      uartRecvQueue;    /* UartMessage_t    */
static SemaphoreHandle_t  penStateMutex;    /* protects penState */
static SemaphoreHandle_t  buttonSemaphore;  /* binary, from ISR */

/* Shared pen state - always access under penStateMutex */
static PenState_t penState = {
    .x       = CANVAS_CENTER_X,
    .y       = CANVAS_CENTER_Y,
    .penDown = 0,
    .erase   = 0,
    .submit  = 0,
    .promptRequest = 1,
    .eraseRetryFrames = 0
};
static volatile RoundPhase_t s_roundPhase = ROUND_PHASE_WAITING_PROMPT;

/* UART TX buffer - written by task, consumed by ISR */
static char          uartSendBuf[MAX_MSG_LEN];
static volatile int  uartSendIdx = 0;
static volatile uint32_t s_uartRecvQueueDrops = 0;

/* Buzzer mode - set by tasks, cleared by buzzer task */
typedef enum {
    BUZZ_NONE,
    BUZZ_BOUNDARY,     /* short beep - pen hit canvas edge */
    BUZZ_CORRECT,      /* happy tune - AI guessed correctly */
    BUZZ_WRONG         /* sad tone  - AI guessed wrong */
} BuzzerMode_t;
static volatile BuzzerMode_t buzzerMode = BUZZ_NONE;

/* =============================================================
 * SECTION 5 - Low-Level Initialisation
 * ============================================================= */

/* -- Clock gating for every port & peripheral we use -------- */
static void initClocks(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK
                |  SIM_SCGC5_PORTB_MASK
                |  SIM_SCGC5_PORTC_MASK
                |  SIM_SCGC5_PORTD_MASK
                |  SIM_SCGC5_PORTE_MASK;

    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK
                |  SIM_SCGC4_I2C0_MASK;
}

/* -- UART2 (same pattern as lab uart_rtos.c) ---------------- */
static void initUART2(uint32_t baud)
{
    NVIC_DisableIRQ(UART2_FLEXIO_IRQn);

    UART2->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

    PORTE->PCR[UART_TX_PIN] = PORT_PCR_MUX(4);
    PORTE->PCR[UART_RX_PIN] = PORT_PCR_MUX(4);

    uint32_t busClk = CLOCK_GetBusClkFreq();
    uint32_t sbr    = (busClk + (baud * 8)) / (baud * 16);

    UART2->BDH &= ~UART_BDH_SBR_MASK;
    UART2->BDH |= (uint8_t)((sbr >> 8) & UART_BDH_SBR_MASK);
    UART2->BDL  = (uint8_t)(sbr & 0xFF);

    UART2->C1 = 0;                          /* 8-N-1, no loops */
    UART2->C2 |= UART_C2_RIE_MASK          /* RX interrupt on  */
              |  UART_C2_RE_MASK            /* receiver on      */
              |  UART_C2_TE_MASK;           /* transmitter on (kept enabled; ISR only toggles TIE) */

    NVIC_SetPriority(UART2_FLEXIO_IRQn, UART2_INT_PRIO);
    NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
    NVIC_EnableIRQ(UART2_FLEXIO_IRQn);
}

/* -- Rotary encoders  (CLK on PORTA w/ interrupt, DT on PORTC) */
static void initEncoders(void)
{
    /* CLK pins: PTA1, PTA2 - falling-edge interrupt, pull-up */
    PORTA->PCR[ENC1_CLK_PIN] = PORT_PCR_MUX(1)
                              | PORT_PCR_PE_MASK
                              | PORT_PCR_PS_MASK
                              | PORT_PCR_IRQC(0b1010);
    PORTA->PCR[ENC2_CLK_PIN] = PORT_PCR_MUX(1)
                              | PORT_PCR_PE_MASK
                              | PORT_PCR_PS_MASK
                              | PORT_PCR_IRQC(0b1010);

    GPIOA->PDDR &= ~((1 << ENC1_CLK_PIN) | (1 << ENC2_CLK_PIN));

    /* DT pins: PTC1, PTC2 - plain input, pull-up */
    PORTC->PCR[ENC1_DT_PIN] = PORT_PCR_MUX(1)
                             | PORT_PCR_PE_MASK
                             | PORT_PCR_PS_MASK;
    PORTC->PCR[ENC2_DT_PIN] = PORT_PCR_MUX(1)
                             | PORT_PCR_PE_MASK
                             | PORT_PCR_PS_MASK;

    GPIOC->PDDR &= ~((1 << ENC1_DT_PIN) | (1 << ENC2_DT_PIN));
}

/* -- External KY-004 button on PTA12 ------------------------ */
static void initButton(void)
{
    /* KY-004 has on-board pull-DOWN (R1 = 10k).
     * S = LOW normally, goes HIGH on press.
     * Use rising edge to detect press. */
    PORTA->PCR[BTN_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[BTN_PIN] = PORT_PCR_MUX(1);

    GPIOA->PDDR &= ~(1 << BTN_PIN);

    PORTA->PCR[BTN_PIN] &= ~PORT_PCR_PS_MASK;
                                              /* PS=0 = pull-down */

    PORTA->PCR[BTN_PIN] &= ~PORT_PCR_PE_MASK;
    PORTA->PCR[BTN_PIN] |= PORT_PCR_PE(1);   /* pull enable */

    PORTA->PCR[BTN_PIN] &= ~PORT_PCR_IRQC_MASK;
    PORTA->PCR[BTN_PIN] |= PORT_PCR_IRQC(0b1001);/* rising edge */
}

/* -- Enable PORTA interrupt (shared by encoders + button) --- */
static void initPortAIRQ(void)
{
    NVIC_SetPriority(PORTA_IRQn, 64);
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
}

/* -- Touch sensor (PTB0, digital input, polled) ------------- */
static void initTouchSensor(void)
{
    PORTB->PCR[TOUCH_DO_PIN] = PORT_PCR_MUX(1);
    GPIOB->PDDR &= ~(1 << TOUCH_DO_PIN);
}

/* -- SMD RGB LED (PTD4 / PTD2 / PTD6, GPIO output) --------- */
static void initRgbLed(void)
{
    PORTD->PCR[LED_R_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED_G_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED_B_PIN] = PORT_PCR_MUX(1);

    GPIOD->PDDR |= (1 << LED_R_PIN)
                 |  (1 << LED_G_PIN)
                 |  (1 << LED_B_PIN);

    /* Start OFF (common cathode: LOW = off) */
    GPIOD->PCOR = (1 << LED_R_PIN)
                | (1 << LED_G_PIN)
                | (1 << LED_B_PIN);
}

/* -- Buzzers (PTE20 active, PTE29 passive, GPIO outputs) ---- */
static void initBuzzers(void)
{
    PORTE->PCR[ACTIVE_BUZZER_PIN] = PORT_PCR_MUX(1);
    GPIOE->PDDR |= (1 << ACTIVE_BUZZER_PIN);
    GPIOE->PCOR = (1 << ACTIVE_BUZZER_PIN);

    PORTE->PCR[PASSIVE_BUZZER_PIN] = PORT_PCR_MUX(1);
    GPIOE->PDDR |= (1 << PASSIVE_BUZZER_PIN);
    GPIOE->PCOR = (1 << PASSIVE_BUZZER_PIN);
}

/* -- I2C0 for MPU-6050 (PTB2 SCL, PTB3 SDA) ---------------- */
static void initI2C0(void)
{
    PORTB->PCR[I2C_SCL_PIN] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[I2C_SDA_PIN] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    I2C0->C1 = 0;

    /* I2C clock: MULT=2 (div 4), ICR=0x2F (div 960)
     * Bus clock ~24 MHz -> 24M / 4 / 960 = ~6.25 kHz
     * Very slow, but reliable with weak internal pull-ups.  */
    I2C0->F = I2C_F_ICR(0x2F) | I2C_F_MULT(2);

    I2C0->C1 |= I2C_C1_IICEN_MASK;
}

/* =============================================================
 * SECTION 6 - I2C Helpers (bare-metal, blocking)
 *
 * These are called ONLY from sensorPollTask, never from ISRs.
 * ============================================================= */

static void i2cBusyWait(void)
{
    uint32_t timeout = 200000;
    while (!(I2C0->S & I2C_S_IICIF_MASK) && --timeout)
        ;
    I2C0->S |= I2C_S_IICIF_MASK;              /* clear flag */
}

static void i2cStart(void)
{
    I2C0->C1 |= I2C_C1_TX_MASK;               /* transmit mode */
    I2C0->C1 |= I2C_C1_MST_MASK;              /* START         */
}

static void i2cStop(void)
{
    I2C0->C1 &= ~I2C_C1_MST_MASK;             /* STOP          */
    I2C0->C1 &= ~I2C_C1_TX_MASK;
}

static void i2cRestart(void)
{
    I2C0->C1 |= I2C_C1_RSTA_MASK;             /* repeated START */
}

static void i2cWriteByte(uint8_t data)
{
    I2C0->D = data;
    i2cBusyWait();
}

/* Burst-read N bytes starting at register 'startReg'.
 * One I2C transaction instead of N separate ones.
 * Sequence: START -> addr+W -> reg -> RESTART -> addr+R ->
 *           [read byte, ACK] x (n-1) -> read byte, NACK -> STOP */
static void mpuBurstRead(uint8_t startReg, uint8_t *buf, uint8_t n)
{
    if (n == 0) return;

    /* Use vTaskSuspendAll instead of taskENTER_CRITICAL so that
     * UART and other interrupts still fire during the I2C read. */
    vTaskSuspendAll();

    i2cStart();
    i2cWriteByte((uint8_t)(MPU6050_ADDR << 1));       /* addr + W */
    i2cWriteByte(startReg);

    i2cRestart();
    i2cWriteByte((uint8_t)((MPU6050_ADDR << 1) | 1)); /* addr + R */

    I2C0->C1 &= ~I2C_C1_TX_MASK;                     /* RX mode  */

    if (n == 1) {
        I2C0->C1 |= I2C_C1_TXAK_MASK;                /* NACK     */
    } else {
        I2C0->C1 &= ~I2C_C1_TXAK_MASK;               /* ACK      */
    }

    (void)I2C0->D;                                    /* dummy read */
    i2cBusyWait();

    for (uint8_t i = 0; i < n; i++) {
        if (i == n - 2) {
            /* Next-to-last byte: set NACK for the last read */
            I2C0->C1 |= I2C_C1_TXAK_MASK;
        }
        if (i == n - 1) {
            /* Last byte: issue STOP before reading */
            i2cStop();
        }
        buf[i] = I2C0->D;
        if (i < n - 1) {
            i2cBusyWait();
        }
    }

    xTaskResumeAll();
}

/* Write a single register on MPU-6050.
 * Called during init (before scheduler) so no critical section needed. */
static void mpuWriteReg(uint8_t reg, uint8_t val)
{
    i2cStart();
    i2cWriteByte((uint8_t)(MPU6050_ADDR << 1));
    i2cWriteByte(reg);
    i2cWriteByte(val);
    i2cStop();
}

/* Single-byte read using STOP+START instead of RESTART.
 * The MPU-6050 latches the register pointer after the write phase,
 * so a separate read transaction works reliably even with weak pull-ups. */
static uint8_t mpuReadRegInit(uint8_t reg)
{
    /* Write phase: set the register pointer */
    i2cStart();
    i2cWriteByte((uint8_t)(MPU6050_ADDR << 1));
    i2cWriteByte(reg);
    i2cStop();

    /* Small delay for bus turnaround */
    for (volatile int d = 0; d < 1000; d++) ;

    /* Read phase: read one byte from the pointed register */
    i2cStart();
    i2cWriteByte((uint8_t)((MPU6050_ADDR << 1) | 1));

    I2C0->C1 &= ~I2C_C1_TX_MASK;
    I2C0->C1 |=  I2C_C1_TXAK_MASK;

    (void)I2C0->D;
    i2cBusyWait();

    i2cStop();
    return I2C0->D;
}

/* I2C bus recovery: bit-bang 9 SCL clocks to unstick a slave holding SDA LOW.
 * Temporarily switches pins to GPIO, toggles SCL, then restores I2C mux. */
static void i2cBusRecovery(void)
{
    /* Disable I2C peripheral while we bit-bang */
    I2C0->C1 &= ~I2C_C1_IICEN_MASK;

    /* Switch SCL to GPIO output, SDA to GPIO input */
    PORTB->PCR[I2C_SCL_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[I2C_SDA_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOB->PDDR |=  (1 << I2C_SCL_PIN);   /* SCL = output */
    GPIOB->PDDR &= ~(1 << I2C_SDA_PIN);   /* SDA = input  */

    PRINTF("I2C recovery: toggling SCL...\r\n");

    /* Toggle SCL up to 9 times; check SDA after each */
    for (int i = 0; i < 9; i++) {
        GPIOB->PCOR = (1 << I2C_SCL_PIN);            /* SCL LOW  */
        for (volatile int d = 0; d < 5000; d++) ;
        GPIOB->PSOR = (1 << I2C_SCL_PIN);            /* SCL HIGH */
        for (volatile int d = 0; d < 5000; d++) ;

        uint8_t sda = (GPIOB->PDIR >> I2C_SDA_PIN) & 1;
        PRINTF("  clk %d: SDA=%d\r\n", i + 1, sda);
        if (sda) {
            PRINTF("  SDA released after %d clocks\r\n", i + 1);
            break;
        }
    }

    /* Generate a STOP condition: SDA LOW -> SCL HIGH -> SDA HIGH */
    GPIOB->PDDR |= (1 << I2C_SDA_PIN);   /* SDA = output */
    GPIOB->PCOR = (1 << I2C_SDA_PIN);     /* SDA LOW  */
    for (volatile int d = 0; d < 5000; d++) ;
    GPIOB->PSOR = (1 << I2C_SCL_PIN);     /* SCL HIGH */
    for (volatile int d = 0; d < 5000; d++) ;
    GPIOB->PSOR = (1 << I2C_SDA_PIN);     /* SDA HIGH = STOP */
    for (volatile int d = 0; d < 5000; d++) ;

    /* Read final pin states */
    uint8_t scl = (GPIOB->PDIR >> I2C_SCL_PIN) & 1;
    uint8_t sda = (GPIOB->PDIR >> I2C_SDA_PIN) & 1;
    PRINTF("  Final: SCL=%d SDA=%d\r\n", scl, sda);

    /* Restore I2C mux */
    PORTB->PCR[I2C_SCL_PIN] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[I2C_SDA_PIN] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    I2C0->C1 |= I2C_C1_IICEN_MASK;
}

static void initMPU6050(void)
{
    /* Small delay for MPU-6050/6500 power-on */
    for (volatile int i = 0; i < 500000; i++) ;

    /* Recover I2C bus in case it's stuck from a previous boot */
    i2cBusRecovery();

    /* Device reset (works for both MPU-6050 and MPU-6500) */
    mpuWriteReg(MPU6050_REG_PWR_MGMT_1, 0x80);
    /* MPU-6500 needs up to 100ms after reset */
    for (volatile int i = 0; i < 1000000; i++) ;

    /* Wake the sensor: write 0x01 (SLEEP=0, CLKSEL=auto-best).
     * Retry several times because I2C writes can fail with weak pull-ups. */
    uint8_t pwr = 0xFF;
    for (int attempt = 0; attempt < 10; attempt++) {
        mpuWriteReg(MPU6050_REG_PWR_MGMT_1, 0x01);
        for (volatile int i = 0; i < 200000; i++) ;
        pwr = mpuReadRegInit(0x6B);
        PRINTF("MPU wake attempt %d: PWR=0x%02X\r\n", attempt + 1, pwr);
        if ((pwr & 0x40) == 0) {   /* SLEEP bit (bit 6) cleared? */
            PRINTF("  -> Sensor awake!\r\n");
            break;
        }
    }

    /* Verify communication */
    uint8_t whoami = mpuReadRegInit(0x75);
    PRINTF("MPU WHO_AM_I=0x%02X (0x68=MPU6050, 0x70=MPU6500), PWR=0x%02X\r\n",
           whoami, pwr);
}

/* Single-byte register read for use during runtime (scheduler running).
 * Uses STOP+START instead of RESTART for reliability with weak pull-ups. */
static uint8_t mpuReadRegRuntime(uint8_t reg)
{
    uint8_t val;
    vTaskSuspendAll();

    /* Write phase: set register pointer */
    i2cStart();
    i2cWriteByte((uint8_t)(MPU6050_ADDR << 1));
    i2cWriteByte(reg);
    i2cStop();

    for (volatile int d = 0; d < 1000; d++) ;

    /* Read phase */
    i2cStart();
    i2cWriteByte((uint8_t)((MPU6050_ADDR << 1) | 1));

    I2C0->C1 &= ~I2C_C1_TX_MASK;
    I2C0->C1 |=  I2C_C1_TXAK_MASK;

    (void)I2C0->D;
    i2cBusyWait();

    i2cStop();
    val = I2C0->D;

    xTaskResumeAll();
    return val;
}

/* Write a single register during runtime (scheduler running).
 * Uses STOP+START and suspends tasks like mpuReadRegRuntime. */
static void mpuWriteRegRuntime(uint8_t reg, uint8_t val)
{
    vTaskSuspendAll();
    i2cStart();
    i2cWriteByte((uint8_t)(MPU6050_ADDR << 1));
    i2cWriteByte(reg);
    i2cWriteByte(val);
    i2cStop();
    xTaskResumeAll();
}

/* Returns 1 when the board is being shaken hard enough.
 * Reads only the HIGH byte of each accel axis (3 reads instead of 6)
 * to avoid tearing noise from split high/low byte reads.
 * Uses delta between consecutive readings to eliminate gravity. */
static uint8_t mpuDetectShake(void)
{
    static int8_t prev_ax = 0, prev_ay = 0, prev_az = 0;
    static uint8_t first = 1;
    static uint8_t staleCount = 0;

    /* Read only the high byte of each axis — no tearing possible */
    int8_t ax = (int8_t)mpuReadRegRuntime(0x3B);
    int8_t ay = (int8_t)mpuReadRegRuntime(0x3D);
    int8_t az = (int8_t)mpuReadRegRuntime(0x3F);

    if (first) {
        prev_ax = ax; prev_ay = ay; prev_az = az;
        first = 0;
        return 0;
    }

    int32_t delta = (int32_t)abs((int)(ax - prev_ax))
                  + (int32_t)abs((int)(ay - prev_ay))
                  + (int32_t)abs((int)(az - prev_az));

    prev_ax = ax; prev_ay = ay; prev_az = az;

    /* If delta is 0 for 5+ consecutive reads, sensor is likely asleep.
     * Re-write PWR_MGMT_1 to wake it back up. */
    if (delta == 0) {
        staleCount++;
        if (staleCount >= 5) {
            mpuWriteRegRuntime(MPU6050_REG_PWR_MGMT_1, 0x01);
            staleCount = 0;
            first = 1;     /* reset baseline on next read */
        }
    } else {
        staleCount = 0;
    }

    return (delta > SHAKE_THRESHOLD) ? 1 : 0;
}

/* =============================================================
 * SECTION 7 - Actuator Helpers
 * ============================================================= */

/* -- RGB LED ------------------------------------------------ */
static void setLedColor(LedColor_t color)
{
    /*
     * Common-cathode: HIGH turns the channel ON.
     * If your LED is common-anode, swap the PSOR/PCOR lines
     * (set = off, clear = on).
     */
    uint8_t r = 0, g = 0, b = 0;

    switch (color) {
    case LED_RED:    r = 1;              break;
    case LED_GREEN:           g = 1;     break;
    case LED_BLUE:                  b=1; break;
    case LED_PURPLE: r = 1;         b=1; break;
    case LED_ORANGE: r = 1;  g = 1;      break;
    case LED_WHITE:  r = 1;  g = 1; b=1;break;
    default: break;
    }

    if (r) GPIOD->PSOR = (1 << LED_R_PIN);
    else   GPIOD->PCOR = (1 << LED_R_PIN);
    if (g) GPIOD->PSOR = (1 << LED_G_PIN);
    else   GPIOD->PCOR = (1 << LED_G_PIN);
    if (b) GPIOD->PSOR = (1 << LED_B_PIN);
    else   GPIOD->PCOR = (1 << LED_B_PIN);
}

/* =============================================================
 * SECTION 8 - UART Transmit Helper
 *
 * Copies string into the ISR-consumed send buffer, then
 * enables TIE + TE so the ISR drains it char-by-char.
 * ============================================================= */

static void uartSendMessage(const char *msg)
{
    /* Wait until the previous transmission is done.
     * (TIE is cleared by the ISR when it reaches '\0'.) */
    while (UART2->C2 & UART_C2_TIE_MASK)
        ;

    strncpy(uartSendBuf, msg, MAX_MSG_LEN - 1);
    uartSendBuf[MAX_MSG_LEN - 1] = '\0';
    uartSendIdx = 0;

    UART2->C2 |= UART_C2_TIE_MASK;
    UART2->C2 |= UART_C2_TE_MASK;
}

/* =============================================================
 * SECTION 9 - Interrupt Service Routines
 *
 * Rules followed:
 *   * No blocking calls (PRINTF, vTaskDelay, xQueueSend, etc.)
 *   * Use only ...FromISR() FreeRTOS API
 *   * Keep as short as possible
 * ============================================================= */

/* -- PORTA: encoder CLK edges + button press ---------------- */
void PORTA_IRQHandler(void)
{
    BaseType_t hpw = pdFALSE;
    uint32_t   isfr = PORTA->ISFR;

    /* ISR-level debounce using tick count.
     * Reject edges that arrive within 20ms of the last accepted
     * edge on the same encoder. This prevents bounce and
     * double-count from detent settle. */
    static TickType_t lastEnc1 = 0;
    static TickType_t lastEnc2 = 0;
    TickType_t now = xTaskGetTickCountFromISR();
    const TickType_t debounce = pdMS_TO_TICKS(20);

    /* --- Encoder 1 (PTA5 CLK) --------------------------------
     * On falling edge of CLK, read DT (PTC1) to find direction.
     * Also verify CLK is actually LOW (not a bounce back to HIGH). */
    if (isfr & (1 << ENC1_CLK_PIN)) {
        uint8_t clk = (GPIOA->PDIR >> ENC1_CLK_PIN) & 1u;
        if (!clk && (now - lastEnc1) >= debounce) {
            EncoderEvent_t evt;
            uint8_t dt = (GPIOC->PDIR >> ENC1_DT_PIN) & 1u;
            evt.axis  = 0;
            evt.delta = dt ? +1 : -1;
            xQueueSendFromISR(encoderQueue, &evt, &hpw);
            lastEnc1 = now;
        }
    }

    /* --- Encoder 2 (PTA13 CLK) ------------------------------- */
    if (isfr & (1 << ENC2_CLK_PIN)) {
        uint8_t clk = (GPIOA->PDIR >> ENC2_CLK_PIN) & 1u;
        if (!clk && (now - lastEnc2) >= debounce) {
            EncoderEvent_t evt;
            uint8_t dt = (GPIOC->PDIR >> ENC2_DT_PIN) & 1u;
            evt.axis  = 1;
            evt.delta = dt ? +1 : -1;
            xQueueSendFromISR(encoderQueue, &evt, &hpw);
            lastEnc2 = now;
        }
    }

    /* --- Button (PTA5) --------------------------------------- */
    if (isfr & (1 << BTN_PIN)) {
        xSemaphoreGiveFromISR(buttonSemaphore, &hpw);
    }

    /* Clear all PORTA flags at once */
    PORTA->ISFR = isfr;
    portYIELD_FROM_ISR(hpw);
}

/* -- UART2 TX / RX ------------------------------------------ */
void UART2_FLEXIO_IRQHandler(void)
{
    static char recvBuf[MAX_MSG_LEN];
    static int  recvIdx = 0;
    BaseType_t  hpw = pdFALSE;

    /* ---- TX: send next character ----------------------------- */
    if ((UART2->C2 & UART_C2_TIE_MASK) &&
        (UART2->S1 & UART_S1_TDRE_MASK))
    {
        if (uartSendBuf[uartSendIdx] == '\0') {
            /* Last byte has already been handed to the shifter; just stop
             * asking for more TX interrupts. TE stays enabled so the shifter
             * is allowed to finish clocking the final character out on the
             * wire — clearing TE here would truncate it. */
            uartSendIdx = 0;
            UART2->C2 &= ~UART_C2_TIE_MASK;  /* stop TX IRQ   */
        } else {
            UART2->D = uartSendBuf[uartSendIdx++];
        }
    }

    /* ---- RX: accumulate until '\n', then enqueue ------------ */
    if (UART2->S1 & UART_S1_RDRF_MASK) {
        char ch = UART2->D;
        if (recvIdx < MAX_MSG_LEN - 1) {
            recvBuf[recvIdx++] = ch;
        }
        if (ch == '\n') {
            UartMessage_t msg;
            recvBuf[recvIdx] = '\0';
            strncpy(msg.message, recvBuf, MAX_MSG_LEN);
            if (xQueueSendFromISR(uartRecvQueue, &msg, &hpw) != pdPASS) {
                s_uartRecvQueueDrops++;
            }
            recvIdx = 0;
        }
    }

    portYIELD_FROM_ISR(hpw);
}

/* =============================================================
 * SECTION 10 - FreeRTOS Tasks
 *
 * Task                    Priority  Blocks on
 * --------------------    --------  ----------------------
 * encoderProcessTask      3 (high)  encoderQueue
 * buttonTask              3 (high)  buttonSemaphore
 * sensorPollTask          2 (med)   vTaskDelayUntil
 * uartSendTask            2 (med)   vTaskDelayUntil
 * uartRecvTask            2 (med)   uartRecvQueue
 * buzzerTask              1 (low)   vTaskDelay
 *
 * With configUSE_TIME_SLICING = 1, the three priority-2 tasks
 * share the CPU in round-robin when no higher-priority task
 * is ready.
 * ============================================================= */

/*
 * encoderProcessTask - reads encoder deltas from the queue,
 * updates pen position, clamps to canvas, triggers buzzer.
 */
static void encoderProcessTask(void *pvParam)
{
    (void)pvParam;
    EncoderEvent_t evt;

    while (1) {
        if (xQueueReceive(encoderQueue, &evt,
                          portMAX_DELAY) == pdTRUE)
        {
            if (xSemaphoreTake(penStateMutex,
                               portMAX_DELAY) == pdTRUE)
            {
                uint8_t hitEdge = 0;

                if (s_roundPhase != ROUND_PHASE_FREE_DRAW) {
                    xSemaphoreGive(penStateMutex);
                    continue;
                }

                if (evt.axis == 0) {
                    int16_t nx = penState.x + evt.delta;
                    if (nx < 0)                { nx = 0;                hitEdge = 1; }
                    if (nx >= CANVAS_WIDTH)     { nx = CANVAS_WIDTH-1;  hitEdge = 1; }
                    penState.x = nx;
                } else {
                    int16_t ny = penState.y + evt.delta;
                    if (ny < 0)                { ny = 0;                hitEdge = 1; }
                    if (ny >= CANVAS_HEIGHT)    { ny = CANVAS_HEIGHT-1; hitEdge = 1; }
                    penState.y = ny;
                }

                PRINTF("ENC: x=%d y=%d\r\n", penState.x, penState.y);

                if (hitEdge) {
                    buzzerMode = BUZZ_BOUNDARY;
                }

                xSemaphoreGive(penStateMutex);
            }
        }
    }
}

/*
 * buttonTask - toggles pen up/down on each press, with
 * simple software debounce.
 */
static void buttonTask(void *pvParam)
{
    (void)pvParam;
    TickType_t lastPress = 0;

    while (1) {
        if (xSemaphoreTake(buttonSemaphore,
                           portMAX_DELAY) == pdTRUE)
        {
            TickType_t now = xTaskGetTickCount();
            if ((now - lastPress) > pdMS_TO_TICKS(DEBOUNCE_MS)) {
                if (xSemaphoreTake(penStateMutex,
                                   pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    if (s_roundPhase != ROUND_PHASE_FREE_DRAW) {
                        xSemaphoreGive(penStateMutex);
                        lastPress = now;
                        continue;
                    }

                    penState.penDown = !penState.penDown;
                    setLedColor(penState.penDown ? LED_BLUE
                                                 : LED_PURPLE);
                    PRINTF("BTN: penDown=%d\r\n", penState.penDown);
                    xSemaphoreGive(penStateMutex);
                }
                lastPress = now;
            }
        }
    }
}

/*
 * sensorPollTask - periodically reads touch sensor and
 * MPU-6050 accelerometer.
 */
static void sensorPollTask(void *pvParam)
{
    (void)pvParam;
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastShake = 0;
    uint8_t gyroPollCount = 0;
    uint8_t lastTouched = 0;

    while (1) {
        /* -- Touch sensor (fast GPIO read, every 100ms) ------ */
        uint8_t touched = (GPIOB->PDIR >> TOUCH_DO_PIN) & 1u;
        uint8_t touchRising = touched && !lastTouched;
        lastTouched = touched;
        uint8_t shakeDetected = 0;

        /* -- Gyro (slow I2C, only every 500ms) --------------- */
#if ENABLE_GYRO
        gyroPollCount++;
        if (gyroPollCount >= GYRO_POLL_DIVIDER) {
            gyroPollCount = 0;
            TickType_t now = xTaskGetTickCount();
            if ((now - lastShake) >= pdMS_TO_TICKS(SHAKE_COOLDOWN_MS)) {
                if (mpuDetectShake()) {
                    shakeDetected = 1;
                    lastShake = now;
                }
            }
        }
#endif

        /* -- Take mutex only to update penState (fast) ------- */
        if (xSemaphoreTake(penStateMutex,
                           portMAX_DELAY) == pdTRUE)
        {
            if (touchRising) {
                if (s_roundPhase == ROUND_PHASE_FREE_DRAW) {
                    penState.submit = 1;
                    s_roundPhase = ROUND_PHASE_WAITING_GUESS;
                    /* Waiting for guess should pulse white; steady white is temporary. */
                    setLedColor(LED_WHITE);
                    PRINTF("TOUCH: submit (waiting guess)\r\n");
                } else if (s_roundPhase == ROUND_PHASE_RESULT_LOCKED) {
                    penState.promptRequest = 1;
                    penState.x = CANVAS_CENTER_X;
                    penState.y = CANVAS_CENTER_Y;
                    penState.penDown = 1;
                    s_roundPhase = ROUND_PHASE_WAITING_PROMPT;
                    setLedColor(LED_ORANGE);
                    PRINTF("TOUCH: request prompt\r\n");
                } else if (s_roundPhase == ROUND_PHASE_WAITING_PROMPT) {
                    penState.promptRequest = 1;
                    setLedColor(LED_ORANGE);
                    PRINTF("TOUCH: still waiting prompt\r\n");
                }
            }

            if (shakeDetected && s_roundPhase == ROUND_PHASE_FREE_DRAW) {
                penState.erase = 1;
                penState.eraseRetryFrames = ERASE_RETRY_FRAMES;
                PRINTF("SHAKE: erase\r\n");
            }
            xSemaphoreGive(penStateMutex);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SENSOR_POLL_MS));
    }
}

/*
 * uartSendTask - packs current pen state into a protocol
 * string and transmits to ESP32 over UART2.
 *
 * Packet format:  $S,x,y,penDown,erase,submit,promptRequest\n
 * Example:        $S,32,17,1,0,0,0\n
 */
static void uartSendTask(void *pvParam)
{
    (void)pvParam;
    TickType_t lastWake = xTaskGetTickCount();
    char buf[MAX_MSG_LEN];

    while (1) {
        if (xSemaphoreTake(penStateMutex,
                           pdMS_TO_TICKS(10)) == pdTRUE)
        {
            snprintf(buf, sizeof(buf), "$S,%d,%d,%d,%d,%d,%d\n",
                     penState.x,
                     penState.y,
                     penState.penDown,
                     penState.erase,
                     penState.submit,
                     penState.promptRequest);

            /* Erase retries briefly until ACK; submit/prompt stay asserted until RESULT/PROMPT. */
            if (penState.eraseRetryFrames > 0) {
                penState.eraseRetryFrames--;
                if (penState.eraseRetryFrames == 0) {
                    penState.erase = 0;
                }
            } else {
                penState.erase = 0;
            }

            xSemaphoreGive(penStateMutex);

            uartSendMessage(buf);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(UART_SEND_MS));
    }
}

/*
 * uartRecvTask - receives command packets from ESP32
 * and drives actuators accordingly.
 *
 * Expected packets from ESP32:
 *   $C,RESULT,<0|1>\n -> AI result (0=wrong, 1=correct)
 *   $C,PROMPT,1\n     -> new prompt is ready; unlock drawing
 *   $C,ACK,0\n        -> erase accepted; clear erase retry
 */
static void uartRecvTask(void *pvParam)
{
    (void)pvParam;
    UartMessage_t msg;
    uint32_t reportedDropCount = 0;

    while (1) {
        if (xQueueReceive(uartRecvQueue, &msg,
                          portMAX_DELAY) == pdTRUE)
        {
            uint32_t dropCount = s_uartRecvQueueDrops;
            if (dropCount != reportedDropCount) {
                uint32_t newDrops = dropCount - reportedDropCount;
                reportedDropCount = dropCount;
                PRINTF("[RX] UART receive queue full, dropped %u message(s)\r\n",
                       (unsigned)newDrops);
            }

            char cmd[16] = {0};
            int  val     = 0;

            if (sscanf(msg.message, "$C,%15[^,],%d", cmd, &val) >= 1) {
                if (xSemaphoreTake(penStateMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
                    continue;
                }

                if (strcmp(cmd, "RESULT") == 0) {
                    penState.submit = 0;
                    s_roundPhase = ROUND_PHASE_RESULT_LOCKED;
                    if (val >= 1) {
                        setLedColor(LED_GREEN);
                        buzzerMode = BUZZ_CORRECT;
                        PRINTF("[RX] RESULT=1 (correct)\r\n");
                    } else {
                        setLedColor(LED_RED);
                        buzzerMode = BUZZ_WRONG;
                        PRINTF("[RX] RESULT=0 (wrong)\r\n");
                    }

                } else if (strcmp(cmd, "PROMPT") == 0 && val >= 1) {
                    penState.x = CANVAS_CENTER_X;
                    penState.y = CANVAS_CENTER_Y;
                    penState.penDown = 1;
                    penState.erase = 0;
                    penState.eraseRetryFrames = 0;
                    penState.submit = 0;
                    penState.promptRequest = 0;
                    s_roundPhase = ROUND_PHASE_FREE_DRAW;
                    setLedColor(LED_BLUE);
                    PRINTF("[RX] PROMPT=1 (center + unlock)\r\n");

                } else if (strcmp(cmd, "ACK") == 0) {
                    penState.erase = 0;
                    penState.eraseRetryFrames = 0;
                }

                xSemaphoreGive(penStateMutex);
            }
        }
    }
}

/*
 * buzzerTask - handles two separate buzzers:
 *   Active buzzer (PTE20):  boundary beep - just pulse HIGH/LOW
 *   Passive buzzer (PTE29): result tunes - square wave for tones
 *
 * Runs at lowest priority so it never starves real work.
 */

/* Play a tone on the PASSIVE buzzer using busy-wait for precise timing.
 * halfPeriodUs controls pitch (microseconds), durationMs controls length.
 * Busy-wait gives much cleaner square waves than vTaskDelay (which has
 * ~1ms granularity and produces muddy/quiet tones). */
static void playTone(uint32_t halfPeriodUs, uint32_t durationMs)
{
    /* Approx loop iterations per microsecond at 48 MHz.
     * Tune this if pitch sounds off on your board. */
    const uint32_t loopsPerUs = 6;
    uint32_t cycles = (durationMs * 1000) / (2 * halfPeriodUs);

    for (uint32_t i = 0; i < cycles; i++) {
        GPIOE->PSOR = (1 << PASSIVE_BUZZER_PIN);
        for (volatile uint32_t d = 0; d < halfPeriodUs * loopsPerUs; d++) ;
        GPIOE->PCOR = (1 << PASSIVE_BUZZER_PIN);
        for (volatile uint32_t d = 0; d < halfPeriodUs * loopsPerUs; d++) ;
    }
}

static void buzzerTask(void *pvParam)
{
    (void)pvParam;

    while (1) {
        BuzzerMode_t mode = buzzerMode;

        if (mode == BUZZ_BOUNDARY) {
            /* Active buzzer: just pulse HIGH for 50ms then LOW */
            GPIOE->PSOR = (1 << ACTIVE_BUZZER_PIN);
            vTaskDelay(pdMS_TO_TICKS(50));
            GPIOE->PCOR = (1 << ACTIVE_BUZZER_PIN);
            buzzerMode = BUZZ_NONE;

        } else if (mode == BUZZ_CORRECT) {
            /* Happy victory tune - 6 ascending notes
             * halfPeriodUs values for approximate notes:
             *   C5=956  D5=851  E5=758  G5=637  A5=568  C6=478 */
            playTone(956, 100);   /* C5 */
            playTone(758, 100);   /* E5 */
            playTone(637, 100);   /* G5 */
            playTone(568, 100);   /* A5 */
            playTone(478, 150);   /* C6 */
            vTaskDelay(pdMS_TO_TICKS(30));
            playTone(478, 300);   /* C6 (hold) */
            buzzerMode = BUZZ_NONE;

        } else if (mode == BUZZ_WRONG) {
            /* Sad descending tune - 4 notes */
            playTone(568, 150);   /* A5 */
            playTone(637, 150);   /* G5 */
            playTone(758, 200);   /* E5 */
            vTaskDelay(pdMS_TO_TICKS(50));
            playTone(956, 400);   /* C5 (long hold) */
            buzzerMode = BUZZ_NONE;

        } else {
            vTaskDelay(pdMS_TO_TICKS(20));  /* idle poll */
        }
    }
}

/* =============================================================
 * SECTION 11 - main()
 * ============================================================= */

int main(void)
{
    /* -- Board-level init (generated by MCUXpresso) ---------- */

	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif


    /* -- Create FreeRTOS objects FIRST -----------------------
     * Must exist before any ISR that uses them is enabled.   */
    encoderQueue    = xQueueCreate(QUEUE_LEN, sizeof(EncoderEvent_t));
    uartRecvQueue   = xQueueCreate(QUEUE_LEN, sizeof(UartMessage_t));
    penStateMutex   = xSemaphoreCreateMutex();
    buttonSemaphore = xSemaphoreCreateBinary();

    if (!encoderQueue || !uartRecvQueue ||
        !penStateMutex || !buttonSemaphore)
    {
        PRINTF("FATAL: could not create RTOS objects\r\n");
        while (1) ;
    }

    PRINTF("=== Drawing App MCU - Starting ===\r\n");

    /* -- Peripheral init (safe now - RTOS objects exist) ---- */
    initClocks();
    initUART2(BAUD_RATE);
    initEncoders();
    initButton();
    initPortAIRQ();
    initTouchSensor();
    initRgbLed();
    initBuzzers();

    /* -- Quick buzzer test (remove after testing) ------------- */
    PRINTF("Passive buzzer test (PTE30)...\r\n");
    for (int i = 0; i < 500; i++) {
        GPIOE->PTOR = (1 << PASSIVE_BUZZER_PIN);
        for (volatile int d = 0; d < 1000; d++) ;
    }
    GPIOE->PCOR = (1 << PASSIVE_BUZZER_PIN);
    PRINTF("Active buzzer test (PTE20)...\r\n");
    GPIOE->PSOR = (1 << ACTIVE_BUZZER_PIN);
    for (volatile int d = 0; d < 2000000; d++) ;
    GPIOE->PCOR = (1 << ACTIVE_BUZZER_PIN);
    PRINTF("Buzzer tests done\r\n");

    /* -- Quick LED test (remove after testing) ---------------- */
    PRINTF("LED test: RED...\r\n");
    setLedColor(LED_RED);
    for (volatile int d = 0; d < 2000000; d++) ;
    PRINTF("LED test: GREEN...\r\n");
    setLedColor(LED_GREEN);
    for (volatile int d = 0; d < 2000000; d++) ;
    PRINTF("LED test: BLUE...\r\n");
    setLedColor(LED_BLUE);
    for (volatile int d = 0; d < 2000000; d++) ;
    PRINTF("LED test: PURPLE...\r\n");
    setLedColor(LED_PURPLE);
    for (volatile int d = 0; d < 2000000; d++) ;
    PRINTF("LED test: ORANGE...\r\n");
    setLedColor(LED_ORANGE);
    for (volatile int d = 0; d < 2000000; d++) ;
    PRINTF("LED test: WHITE...\r\n");
    setLedColor(LED_WHITE);
    for (volatile int d = 0; d < 2000000; d++) ;
    setLedColor(LED_OFF);
    PRINTF("LED tests done\r\n");

#if ENABLE_GYRO
    initI2C0();
    initMPU6050();
#endif

    PRINTF("Peripherals OK\r\n");

    /* -- Create tasks --------------------------------------- */
    xTaskCreate(encoderProcessTask, "enc",
                configMINIMAL_STACK_SIZE + 100, NULL, 3, NULL);
    xTaskCreate(buttonTask,         "btn",
                configMINIMAL_STACK_SIZE + 100, NULL, 3, NULL);
    xTaskCreate(sensorPollTask,     "sensor",
                configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL);
    xTaskCreate(uartSendTask,       "tx",
                configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL);
    xTaskCreate(uartRecvTask,       "rx",
                configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL);
    xTaskCreate(buzzerTask,         "buzz",
                configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL);

    if (xSemaphoreTake(penStateMutex, portMAX_DELAY) == pdTRUE) {
        penState.x = CANVAS_CENTER_X;
        penState.y = CANVAS_CENTER_Y;
        penState.penDown = 0;
        penState.erase = 0;
        penState.eraseRetryFrames = 0;
        penState.submit = 0;
        penState.promptRequest = 1;
        s_roundPhase = ROUND_PHASE_WAITING_PROMPT;
        xSemaphoreGive(penStateMutex);
    }

    /* Initial runtime state: yellow, waiting for ESP32 prompt at center. */
    setLedColor(LED_ORANGE);

    PRINTF("Scheduler starting (6 tasks)\r\n");
    vTaskStartScheduler();

    /* Should never reach here */
    while (1) {
        __asm volatile ("nop");
    }
    return 0;
}
