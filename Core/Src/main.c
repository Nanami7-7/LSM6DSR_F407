#include <stdio.h>
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "bsp_lsm6dsr.h"

void SystemClock_Config(void);

static char vofa_buf[128];
static volatile uint8_t vofa_tx_busy = 0;

int fputc(int ch, FILE *f) { uint8_t c = ch; HAL_UART_Transmit(&huart1, &c, 1, 100); return ch; }

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        vofa_tx_busy = 0;
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    bsp_lsm6dsr_init();

    printf("\r\n--- VOFA+ Live Data (10ch: ax,ay,az,gx,gy,gz,pitch,roll,yaw,temp) ---\r\n");

    while (1) {
        bsp_lsm6dsr_data_t d;
        bsp_lsm6dsr_update(&d);

        while (vofa_tx_busy) { /* spin */ }

        int len = bsp_lsm6dsr_vofa_format(vofa_buf, sizeof(vofa_buf), &d);
        if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)vofa_buf, len) == HAL_OK) {
            vofa_tx_busy = 1;
        }

        HAL_Delay(9);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

void Error_Handler(void) { __disable_irq(); while (1) { } }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert failed: %s %lu\r\n", file, (unsigned long)line);
}
#endif
