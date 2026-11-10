/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "i2c.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
    LL_DMA_ClearFlag_TC1(DMA1);
  /* USER CODE END DMA1_Channel1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */
    if (LL_DMA_IsActiveFlag_TC3(DMA1))
    {
        LL_DMA_ClearFlag_TC3(DMA1);
        I2C_SlaveDMARxCpltCallback();
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 5);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)i2cDataRx);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    }
  /* USER CODE END DMA1_Channel2_3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}
/**
  * @brief This function handles ADC interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */
    LL_ADC_ClearFlag_ADRDY(ADC1);
  /* USER CODE END ADC1_IRQn 0 */
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */
    TIM14_PeriodElapsedCallback();
    LL_TIM_ClearFlag_UPDATE(TIM14);
  /* USER CODE END TIM14_IRQn 0 */
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
  */
void I2C1_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_IRQn 0 */
    if (LL_I2C_IsActiveFlag_ADDR(I2C1))
    {
        if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_WRITE)
        {
            LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
            LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 5);
            LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)i2cDataRx);
            LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
            LL_I2C_ClearFlag_ADDR(I2C1);
        }
        else
        {
            // READ direction: master wants response data.
            // Clear any lingering STOP / NACK flags
            LL_I2C_ClearFlag_STOP(I2C1);
            LL_I2C_ClearFlag_NACK(I2C1);

            // Write first byte (byte 0) to TXDR BEFORE clearing ADDR flag!
            // When ADDR is cleared, hardware moves TXDR into shift register
            // and asserts TXIS for byte 1. This prevents sending stale data.
            LL_I2C_TransmitData8(I2C1, i2cDataTx[0]);
            LL_I2C_ClearFlag_ADDR(I2C1);

            // Transmit remaining 4 bytes (bytes 1..4)
            for (int i = 1; i < 5; i++)
            {
                uint32_t timeout = 50000;
                while (!LL_I2C_IsActiveFlag_TXIS(I2C1))
                {
                    if (LL_I2C_IsActiveFlag_NACK(I2C1) || LL_I2C_IsActiveFlag_STOP(I2C1))
                        goto i2c_tx_done;
                    if (--timeout == 0)
                    {
                        // Timeout: reset peripheral to un-stretch SCL and re-enable addresses
                        LL_I2C_Disable(I2C1);
                        LL_I2C_EnableOwnAddress1(I2C1);
                        LL_I2C_EnableOwnAddress2(I2C1);
                        LL_I2C_Enable(I2C1);
                        LL_I2C_EnableIT_ADDR(I2C1);
                        LL_I2C_EnableIT_STOP(I2C1);
                        LL_I2C_EnableIT_NACK(I2C1);
                        LL_I2C_EnableIT_ERR(I2C1);
                        LL_I2C_EnableDMAReq_RX(I2C1);
                        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
                        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 5);
                        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)i2cDataRx);
                        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
                        goto i2c_tx_done;
                    }
                }
                LL_I2C_TransmitData8(I2C1, i2cDataTx[i]);
            }

        i2c_tx_done:
            LL_I2C_ClearFlag_STOP(I2C1);
            LL_I2C_ClearFlag_NACK(I2C1);

            // Apply deferred Set_ID after response is sent
            if (pendingSetId)
            {
                pendingSetId = 0;
                Set_ID(pendingNewNodeId);
            }
        }
    }
    if (LL_I2C_IsActiveFlag_STOP(I2C1))
    {
        LL_I2C_ClearFlag_STOP(I2C1);
        // Only reset DMA if a partial write was aborted mid-stream (1..4 bytes received)
        uint32_t rem = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);
        if (rem > 0 && rem < 5)
        {
            LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
            LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 5);
            LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)i2cDataRx);
            LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
        }
    }
    if (LL_I2C_IsActiveFlag_NACK(I2C1))
    {
        LL_I2C_ClearFlag_NACK(I2C1);
    }
    if (LL_I2C_IsActiveFlag_BERR(I2C1))
    {
        LL_I2C_ClearFlag_BERR(I2C1);
        uint32_t rem = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);
        if (rem > 0 && rem < 5)
        {
            LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
            LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 5);
            LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)i2cDataRx);
            LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
        }
    }
    if (LL_I2C_IsActiveFlag_OVR(I2C1))
    {
        LL_I2C_ClearFlag_OVR(I2C1);
    }
  /* USER CODE END I2C1_IRQn 0 */

  /* USER CODE BEGIN I2C1_IRQn 1 */

  /* USER CODE END I2C1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
