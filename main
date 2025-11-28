/**
 * @file main.c
 * @brief Receiver Node (Gate B) - Wireless Speed Gate System
 * @author Speed Gate Project
 * @date 2025
 * 
 * This node receives timestamps from Gate A, detects objects passing through
 * the second gate, and calculates the speed based on the time difference.
 */

#include "stm32f1xx_hal.h"
#include "nrf24.h"
#include "speed_gate.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * PRIVATE VARIABLES
 * ============================================================================ */

// Peripheral handles
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;

// nRF24L01+ handle
NRF24_Handle_t nrf;

// System state
static volatile SystemState_t system_state = STATE_IDLE;
static volatile uint32_t gate_a_timestamp = 0;
static volatile uint32_t gate_b_timestamp = 0;
static volatile uint8_t gate_a_received = 0;
static volatile uint8_t object_detected_flag = 0;

// Measurement data
static SpeedMeasurement_t current_measurement;
static uint32_t measurement_count = 0;

// Pipe address
static uint8_t rx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void SPI1_Init(void);
static void USART1_Init(void);
static void TIM2_Init(void);
static void NRF24_Setup(void);
static uint32_t GetMicroseconds(void);
static void CheckWirelessData(void);
static void ProcessMeasurement(void);
static void DisplayResult(SpeedMeasurement_t *result);
static void BlinkLED(GPIO_TypeDef *port, uint16_t pin, uint8_t count);

// UART redirect for printf
int _write(int file, char *ptr, int len);

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */

int main(void) {
    // Initialize HAL
    HAL_Init();
    
    // Configure system clock (72 MHz)
    SystemClock_Config();
    
    // Initialize peripherals
    GPIO_Init();
    SPI1_Init();
    USART1_Init();
    TIM2_Init();
    
    // Start microsecond timer
    HAL_TIM_Base_Start(&htim2);
    
    // Initialize nRF24L01+
    NRF24_Setup();
    
    // Startup indication
    printf("\r\n========================================\r\n");
    printf("  Wireless Speed Gate System - Gate B\r\n");
    printf("  (Receiver Node)\r\n");
    printf("========================================\r\n");
    printf("Gate distance: %.2f m\r\n", GATE_DISTANCE_M);
    printf("System initialized. Waiting for objects...\r\n\r\n");
    
    BlinkLED(LED_BUILTIN_PORT, LED_BUILTIN_PIN, 3);
    
    // Variables for debouncing
    uint32_t last_detection_time = 0;
    uint8_t last_ir_state = 1;  // 1 = not blocked
    uint32_t wait_start_time = 0;
    
    // Main loop
    while (1) {
        // Check for incoming wireless data
        CheckWirelessData();
        
        // Read IR sensor
        uint8_t current_ir_state = HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN);
        uint32_t current_time = HAL_GetTick();
        
        // State machine
        switch (system_state) {
            case STATE_IDLE:
                // Waiting for Gate A packet
                if (gate_a_received) {
                    system_state = STATE_WAITING_GATE_B;
                    wait_start_time = HAL_GetTick();
                    LED_ON(LED_STATUS_PORT, LED_STATUS_PIN);
                    printf("[WAITING] Gate A triggered. Waiting for object at Gate B...\r\n");
                }
                break;
                
            case STATE_WAITING_GATE_B:
                // Check for object at Gate B (falling edge)
                if (last_ir_state == GPIO_PIN_SET && current_ir_state == GPIO_PIN_RESET) {
                    if ((current_time - last_detection_time) > DEBOUNCE_TIME_MS) {
                        // Capture Gate B timestamp
                        gate_b_timestamp = GetMicroseconds();
                        last_detection_time = current_time;
                        
                        LED_ON(LED_DETECT_PORT, LED_DETECT_PIN);
                        printf("[DETECT] Object at Gate B - Timestamp: %lu us\r\n", 
                               gate_b_timestamp);
                        
                        system_state = STATE_MEASUREMENT_COMPLETE;
                    }
                }
                
                // Check for timeout
                if ((current_time - wait_start_time) > TIMEOUT_MS) {
                    printf("[TIMEOUT] No object detected at Gate B within %d ms\r\n", 
                           TIMEOUT_MS);
                    system_state = STATE_TIMEOUT;
                }
                break;
                
            case STATE_MEASUREMENT_COMPLETE:
                ProcessMeasurement();
                
                // Reset for next measurement
                HAL_Delay(MEASUREMENT_RESET_MS);
                LED_OFF(LED_DETECT_PORT, LED_DETECT_PIN);
                LED_OFF(LED_STATUS_PORT, LED_STATUS_PIN);
                
                gate_a_received = 0;
                gate_a_timestamp = 0;
                gate_b_timestamp = 0;
                system_state = STATE_IDLE;
                
                printf("\r\n[READY] Waiting for next object...\r\n\r\n");
                break;
                
            case STATE_TIMEOUT:
                // Reset system after timeout
                LED_OFF(LED_STATUS_PORT, LED_STATUS_PIN);
                BlinkLED(LED_BUILTIN_PORT, LED_BUILTIN_PIN, 2);
                
                gate_a_received = 0;
                gate_a_timestamp = 0;
                system_state = STATE_IDLE;
                
                printf("[RESET] System reset. Waiting for objects...\r\n\r\n");
                break;
                
            default:
                system_state = STATE_IDLE;
                break;
        }
        
        last_ir_state = current_ir_state;
        
        // Small delay
        HAL_Delay(1);
    }
}

/* ============================================================================
 * WIRELESS COMMUNICATION
 * ============================================================================ */

static void CheckWirelessData(void) {
    if (NRF24_Available(&nrf)) {
        SpeedGatePacket_t packet;
        
        if (NRF24_Receive(&nrf, (uint8_t *)&packet)) {
            // Toggle LED to indicate reception
            LED_TOGGLE(LED_BUILTIN_PORT, LED_BUILTIN_PIN);
            
            // Verify packet integrity
            if (!verify_checksum(&packet)) {
                printf("[RX] WARNING: Checksum mismatch, packet discarded\r\n");
                return;
            }
            
            // Verify it's from Gate A
            if (packet.gate_id != GATE_ID_A) {
                printf("[RX] WARNING: Unknown gate ID %d\r\n", packet.gate_id);
                return;
            }
            
            // Store Gate A timestamp
            gate_a_timestamp = packet.timestamp_us;
            gate_a_received = 1;
            
            printf("[RX] Gate A packet received\r\n");
            printf("     Timestamp: %lu us, Seq: %d\r\n", 
                   packet.timestamp_us, packet.sequence);
        }
    }
}

/* ============================================================================
 * MEASUREMENT PROCESSING
 * ============================================================================ */

static void ProcessMeasurement(void) {
    // Calculate speed
    // Note: We need to account for the fact that Gate A and Gate B have
    // independent timers. For accurate measurement, we use the time
    // difference measured at Gate B's timer reference.
    
    // Since both gates have synchronized timers starting from power-on,
    // we can directly compare timestamps. For more precision in real
    // applications, consider adding timer synchronization.
    
    // For this implementation, we assume:
    // 1. Both nodes are powered on simultaneously
    // 2. Crystal accuracy is sufficient for short measurements
    
    // Calculate transit time (Gate B timestamp should be > Gate A timestamp)
    calculate_speed(gate_a_timestamp, gate_b_timestamp, &current_measurement);
    
    measurement_count++;
    
    // Display results
    DisplayResult(&current_measurement);
}

static void DisplayResult(SpeedMeasurement_t *result) {
    printf("\r\n");
    printf("╔════════════════════════════════════════╗\r\n");
    printf("║       SPEED MEASUREMENT RESULT         ║\r\n");
    printf("╠════════════════════════════════════════╣\r\n");
    printf("║  Measurement #%-4lu                     ║\r\n", measurement_count);
    printf("╠════════════════════════════════════════╣\r\n");
    
    if (result->valid) {
        printf("║  Gate A Time:    %10lu us        ║\r\n", result->time_gate_a_us);
        printf("║  Gate B Time:    %10lu us        ║\r\n", result->time_gate_b_us);
        printf("║  Transit Time:   %10.4f s         ║\r\n", result->transit_time_s);
        printf("╠════════════════════════════════════════╣\r\n");
        printf("║  SPEED:          %7.2f m/s          ║\r\n", result->speed_mps);
        printf("║                  %7.2f km/h         ║\r\n", result->speed_kmh);
        printf("║                  %7.2f mph          ║\r\n", result->speed_kmh * 0.621371f);
    } else {
        printf("║  ERROR: Invalid measurement            ║\r\n");
        printf("║  Gate B timestamp <= Gate A timestamp  ║\r\n");
    }
    
    printf("╚════════════════════════════════════════╝\r\n");
    printf("\r\n");
}

/* ============================================================================
 * PERIPHERAL INITIALIZATION
 * ============================================================================ */

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    // Configure HSE and PLL for 72 MHz
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    
    // Configure clock dividers
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // Configure onboard LED (PC13) - active low
    GPIO_InitStruct.Pin = LED_BUILTIN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_BUILTIN_PORT, &GPIO_InitStruct);
    LED_OFF(LED_BUILTIN_PORT, LED_BUILTIN_PIN);
    
    // Configure status LED (PB1) - indicates waiting for Gate B
    GPIO_InitStruct.Pin = LED_STATUS_PIN;
    HAL_GPIO_Init(LED_STATUS_PORT, &GPIO_InitStruct);
    LED_OFF(LED_STATUS_PORT, LED_STATUS_PIN);
    
    // Configure detection LED (PB10)
    GPIO_InitStruct.Pin = LED_DETECT_PIN;
    HAL_GPIO_Init(LED_DETECT_PORT, &GPIO_InitStruct);
    LED_OFF(LED_DETECT_PORT, LED_DETECT_PIN);
    
    // Configure IR sensor input (PB0) with pull-up
    GPIO_InitStruct.Pin = IR_SENSOR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(IR_SENSOR_PORT, &GPIO_InitStruct);
    
    // Configure nRF24 CE pin (PA4)
    GPIO_InitStruct.Pin = NRF_CE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF_CE_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET);
    
    // Configure nRF24 CSN pin (PA3)
    GPIO_InitStruct.Pin = NRF_CSN_PIN;
    HAL_GPIO_Init(NRF_CSN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET);
}

static void SPI1_Init(void) {
    __HAL_RCC_SPI1_CLK_ENABLE();
    
    // Configure SPI1 pins (PA5=SCK, PA6=MISO, PA7=MOSI)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;  // SCK, MOSI
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_6;  // MISO
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Configure SPI1
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // ~4.5 MHz
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);
}

static void USART1_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    
    // Configure USART1 pins (PA9=TX, PA10=RX)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Configure USART1
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

static void TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    // Configure TIM2 for microsecond counting
    // 72 MHz / 72 = 1 MHz (1 us per tick)
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 72 - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;  // 32-bit counter
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);
}

static void NRF24_Setup(void) {
    // Configure nRF24 handle
    nrf.hspi = &hspi1;
    nrf.ce_port = NRF_CE_PORT;
    nrf.ce_pin = NRF_CE_PIN;
    nrf.csn_port = NRF_CSN_PORT;
    nrf.csn_pin = NRF_CSN_PIN;
    nrf.payload_size = RF_PAYLOAD_SIZE;
    nrf.channel = RF_CHANNEL;
    
    // Initialize nRF24L01+
    if (!NRF24_Init(&nrf)) {
        printf("[ERROR] nRF24L01+ initialization failed!\r\n");
        while (1) {
            BlinkLED(LED_BUILTIN_PORT, LED_BUILTIN_PIN, 1);
            HAL_Delay(500);
        }
    }
    
    // Set RX address
    NRF24_SetRXAddress(&nrf, 0, rx_address);
    
    // Configure for reception
    NRF24_SetChannel(&nrf, RF_CHANNEL);
    NRF24_SetPALevel(&nrf, NRF_PA_MAX);
    NRF24_SetDataRate(&nrf, NRF_DR_2MBPS);
    
    // Enter RX mode
    NRF24_SetRXMode(&nrf);
    
    printf("[NRF24] Initialized successfully\r\n");
    printf("        Channel: %d, PA Level: MAX, Mode: RX\r\n", RF_CHANNEL);
}

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

static uint32_t GetMicroseconds(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);
}

static void BlinkLED(GPIO_TypeDef *port, uint16_t pin, uint8_t count) {
    for (int i = 0; i < count; i++) {
        LED_ON(port, pin);
        HAL_Delay(100);
        LED_OFF(port, pin);
        HAL_Delay(100);
    }
}

// Redirect printf to UART
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* ============================================================================
 * INTERRUPT HANDLERS
 * ============================================================================ */

void SysTick_Handler(void) {
    HAL_IncTick();
}

void NMI_Handler(void) { }
void HardFault_Handler(void) { while (1); }
void MemManage_Handler(void) { while (1); }
void BusFault_Handler(void) { while (1); }
void UsageFault_Handler(void) { while (1); }
void SVC_Handler(void) { }
void DebugMon_Handler(void) { }
void PendSV_Handler(void) { }
