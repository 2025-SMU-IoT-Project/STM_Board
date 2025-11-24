/*!
 * \file      sx1272mb2das-board.c
 *
 * \brief     Target board SX1272MB2DAS shield driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdlib.h>
#include "utilities.h"
//#include "board-config.h"
#include "radio.h"
#include "sx1272-board.h"

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] channel Channel frequency in Hz
 * \retval PaSelect RegPaConfig PaSelect value
 */
static uint8_t SX1272GetPaSelect( uint32_t channel );

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1272Init,
    SX1272GetStatus,
    SX1272SetModem,
    SX1272SetChannel,
    NULL,
    SX1272Random,
    SX1272SetRxConfig,
    SX1272SetTxConfig,
    SX1272CheckRfFrequency,
    SX1272GetTimeOnAir,
    SX1272Send,
    SX1272SetSleep,
    SX1272SetStby,
    SX1272SetRx,
    NULL,
    SX1272SetTxContinuousWave,
    SX1272ReadRssi,
    SX1272Write,
    SX1272Read,
    SX1272WriteBuffer,
    SX1272ReadBuffer,
    SX1272SetMaxPayloadLength,
    SX1272SetPublicNetwork,
    SX1272GetWakeupTime,
    NULL, // void ( *IrqProcess )( void )
    NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
    NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
};

void SX1272SetBoardTcxo( uint8_t state )
{

}

uint32_t SX1272GetBoardTcxoWakeupTime( void )
{
    return 0;
}

void SX1272Reset( void )
{

	// GPIO 초기화 구조체 선언
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Step 1: Configure RESET as OUTPUT and set HIGH
	GPIO_InitStruct.Pin = RADIO_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;      // Output Push Pull
	GPIO_InitStruct.Pull = GPIO_NOPULL;              // No Pull
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;     // Low Speed
	HAL_GPIO_Init(RADIO_RESET_GPIO_Port, &GPIO_InitStruct);

	// Set pin to HIGH (1)
	HAL_GPIO_WritePin(RADIO_RESET_GPIO_Port, RADIO_RESET_Pin, GPIO_PIN_SET);

	// Wait 1 ms
	HAL_Delay(1);

	// Step 2: Configure RESET as INPUT
	GPIO_InitStruct.Pin = RADIO_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;          // Input mode
	GPIO_InitStruct.Pull = GPIO_NOPULL;              // No Pull
	HAL_GPIO_Init(RADIO_RESET_GPIO_Port, &GPIO_InitStruct);

	// Wait 6 ms
	HAL_Delay(6);

}

void SX1272SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1272Read( REG_PACONFIG );
    paDac = SX1272Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1272GetPaSelect( SX1272.Settings.Channel );

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1272Write( REG_PACONFIG, paConfig );
    SX1272Write( REG_PADAC, paDac );
}

static uint8_t SX1272GetPaSelect( uint32_t channel )
{
    return RF_PACONFIG_PASELECT_RFO;
}

void SX1272SetAntSwLowPower( bool status )
{
	if (RadioIsActive != status )
    {
        RadioIsActive = status;

        if( status == false )
        {
            SX1272AntSwInit( );
        }
        else
        {
            SX1272AntSwDeInit( );
        }
    }
}

void SX1272AntSwInit( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE(); // RADIO_ANT_SWITCH가 PC0라고 가정

    // Configure ANT_SWITCH pin as OUTPUT
    GPIO_InitStruct.Pin = RADIO_ANT_SWITCH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-Pull Output
    GPIO_InitStruct.Pull = GPIO_PULLUP;          // Pull-up
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RADIO_ANT_SWITCH_GPIO_Port, &GPIO_InitStruct);

    // Set initial state to LOW
    HAL_GPIO_WritePin(RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, GPIO_PIN_RESET);
}

void SX1272AntSwDeInit( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure ANT_SWITCH pin as ANALOG (high impedance)
    GPIO_InitStruct.Pin = RADIO_ANT_SWITCH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;     // Analog mode (high impedance)
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // No pull
    HAL_GPIO_Init(RADIO_ANT_SWITCH_GPIO_Port, &GPIO_InitStruct);
}


void SX1272SetAntSw( uint8_t opMode )
{
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        HAL_GPIO_WritePin(RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, GPIO_PIN_SET);
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
        HAL_GPIO_WritePin(RADIO_ANT_SWITCH_GPIO_Port, RADIO_ANT_SWITCH_Pin, GPIO_PIN_RESET);
        break;
    }
}

bool SX1272CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}
