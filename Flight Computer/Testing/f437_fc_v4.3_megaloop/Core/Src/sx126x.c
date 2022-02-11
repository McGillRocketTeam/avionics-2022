/**
 * @file      sx126x.c
 *
 * @brief     SX126x radio driver implementation
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <string.h>  // memcpy
#include <stdio.h>
#include <stdlib.h>
#include "sx126x.h"
#include "sx126x_regs.h"
#include "main.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Internal frequency of the radio
 */
#define SX126X_XTAL_FREQ 32000000UL

/**
 * @brief Internal frequency of the radio
 */
#define SX126X_RTC_FREQ_IN_HZ 64000UL

/**
 * @brief Scaling factor used to perform fixed-point operations
 */
#define SX126X_PLL_STEP_SHIFT_AMOUNT ( 14 )

/**
 * @brief PLL step - scaled with SX126X_PLL_STEP_SHIFT_AMOUNT
 */
#define SX126X_PLL_STEP_SCALED ( SX126X_XTAL_FREQ >> ( 25 - SX126X_PLL_STEP_SHIFT_AMOUNT ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * Commands Interface
 */
typedef enum sx126x_commands_e
{
    // Operational Modes Functions
    SX126X_SET_SLEEP                  = 0x84,
    SX126X_SET_STANDBY                = 0x80,
    SX126X_SET_FS                     = 0xC1,
    SX126X_SET_TX                     = 0x83,
    SX126X_SET_RX                     = 0x82,
    SX126X_SET_STOP_TIMER_ON_PREAMBLE = 0x9F,
    SX126X_SET_RX_DUTY_CYCLE          = 0x94,
    SX126X_SET_CAD                    = 0xC5,
    SX126X_SET_TX_CONTINUOUS_WAVE     = 0xD1,
    SX126X_SET_TX_INFINITE_PREAMBLE   = 0xD2,
    SX126X_SET_REGULATOR_MODE         = 0x96,
    SX126X_CALIBRATE                  = 0x89,
    SX126X_CALIBRATE_IMAGE            = 0x98,
    SX126X_SET_PA_CFG                 = 0x95,
    SX126X_SET_RX_TX_FALLBACK_MODE    = 0x93,
    // Registers and buffer Access
    SX126X_WRITE_REGISTER = 0x0D,
    SX126X_READ_REGISTER  = 0x1D,
    SX126X_WRITE_BUFFER   = 0x0E,
    SX126X_READ_BUFFER    = 0x1E,
    // DIO and IRQ Control Functions
    SX126X_SET_DIO_IRQ_PARAMS         = 0x08,
    SX126X_GET_IRQ_STATUS             = 0x12,
    SX126X_CLR_IRQ_STATUS             = 0x02,
    SX126X_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
    SX126X_SET_DIO3_AS_TCXO_CTRL      = 0x97,
    // RF Modulation and Packet-Related Functions
    SX126X_SET_RF_FREQUENCY          = 0x86,
    SX126X_SET_PKT_TYPE              = 0x8A,
    SX126X_GET_PKT_TYPE              = 0x11,
    SX126X_SET_TX_PARAMS             = 0x8E,
    SX126X_SET_MODULATION_PARAMS     = 0x8B,
    SX126X_SET_PKT_PARAMS            = 0x8C,
    SX126X_SET_CAD_PARAMS            = 0x88,
    SX126X_SET_BUFFER_BASE_ADDRESS   = 0x8F,
    SX126X_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,
    // Communication Status Information
    SX126X_GET_STATUS           = 0xC0,
    SX126X_GET_RX_BUFFER_STATUS = 0x13,
    SX126X_GET_PKT_STATUS       = 0x14,
    SX126X_GET_RSSI_INST        = 0x15,
    SX126X_GET_STATS            = 0x10,
    SX126X_RESET_STATS          = 0x00,
    // Miscellaneous
    SX126X_GET_DEVICE_ERRORS = 0x17,
    SX126X_CLR_DEVICE_ERRORS = 0x07,
} sx126x_commands_t;

/**
 * Commands Interface buffer sizes
 */
typedef enum sx126x_commands_size_e
{
    // Operational Modes Functions
    SX126X_SIZE_SET_SLEEP                  = 2,
    SX126X_SIZE_SET_STANDBY                = 2,
    SX126X_SIZE_SET_FS                     = 1,
    SX126X_SIZE_SET_TX                     = 4,
    SX126X_SIZE_SET_RX                     = 4,
    SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE = 2,
    SX126X_SIZE_SET_RX_DUTY_CYCLE          = 7,
    SX126X_SIZE_SET_CAD                    = 1,
    SX126X_SIZE_SET_TX_CONTINUOUS_WAVE     = 1,
    SX126X_SIZE_SET_TX_INFINITE_PREAMBLE   = 1,
    SX126X_SIZE_SET_REGULATOR_MODE         = 2,
    SX126X_SIZE_CALIBRATE                  = 2,
    SX126X_SIZE_CALIBRATE_IMAGE            = 3,
    SX126X_SIZE_SET_PA_CFG                 = 5,
    SX126X_SIZE_SET_RX_TX_FALLBACK_MODE    = 2,
    // Registers and buffer Access
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_REGISTER = 3,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_REGISTER = 4,
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_BUFFER = 2,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_BUFFER = 2,
    // DIO and IRQ Control Functions
    SX126X_SIZE_SET_DIO_IRQ_PARAMS         = 9,
    SX126X_SIZE_GET_IRQ_STATUS             = 4,
    SX126X_SIZE_CLR_IRQ_STATUS             = 3,
    SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL = 2,
    SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL      = 5,
    // RF Modulation and Packet-Related Functions
    SX126X_SIZE_SET_RF_FREQUENCY           = 5,
    SX126X_SIZE_SET_PKT_TYPE               = 2,
    SX126X_SIZE_GET_PKT_TYPE               = 3,
    SX126X_SIZE_SET_TX_PARAMS              = 3,
    SX126X_SIZE_SET_MODULATION_PARAMS_GFSK = 9,
    SX126X_SIZE_SET_MODULATION_PARAMS_LORA = 5,
    SX126X_SIZE_SET_PKT_PARAMS_GFSK        = 10,
    SX126X_SIZE_SET_PKT_PARAMS_LORA        = 7,
    SX126X_SIZE_SET_CAD_PARAMS             = 8,
    SX126X_SIZE_SET_BUFFER_BASE_ADDRESS    = 3,
    SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT  = 2,
    // Communication Status Information
    SX126X_SIZE_GET_STATUS           = 1,
    SX126X_SIZE_GET_RX_BUFFER_STATUS = 4,
    SX126X_SIZE_GET_PKT_STATUS       = 2,
    SX126X_SIZE_GET_RSSI_INST        = 2,
    SX126X_SIZE_GET_STATS            = 2,
    SX126X_SIZE_RESET_STATS          = 7,
    // Miscellaneous
    SX126X_SIZE_GET_DEVICE_ERRORS = 2,
    SX126X_SIZE_CLR_DEVICE_ERRORS = 3,
    SX126X_SIZE_MAX_BUFFER        = 255,
    SX126X_SIZE_DUMMY_BYTE        = 1,
} sx126x_commands_size_t;

/**
 * The offset from the command buffer compared to the receieved buffer
 * Mostly for read commands on SPI lines
 */
typedef enum sx126x_commands_offset_e {
    SX126X_OFFSET_GET_IRQ_STATUS            = 1,
    SX126X_OFFSET_READ_BUFFER               = 2,
    SX126X_OFFSET_GET_RX_BUFFER_STATUS      = 1,
} sx126x_commands_offset_t;

typedef struct
{
    uint32_t bw;
    uint8_t  param;
} gfsk_bw_t;

gfsk_bw_t gfsk_bw[] = {
    { 4800, SX126X_GFSK_BW_4800 },     { 5800, SX126X_GFSK_BW_5800 },     { 7300, SX126X_GFSK_BW_7300 },
    { 9700, SX126X_GFSK_BW_9700 },     { 11700, SX126X_GFSK_BW_11700 },   { 14600, SX126X_GFSK_BW_14600 },
    { 19500, SX126X_GFSK_BW_19500 },   { 23400, SX126X_GFSK_BW_23400 },   { 29300, SX126X_GFSK_BW_29300 },
    { 39000, SX126X_GFSK_BW_39000 },   { 46900, SX126X_GFSK_BW_46900 },   { 58600, SX126X_GFSK_BW_58600 },
    { 78200, SX126X_GFSK_BW_78200 },   { 93800, SX126X_GFSK_BW_93800 },   { 117300, SX126X_GFSK_BW_117300 },
    { 156200, SX126X_GFSK_BW_156200 }, { 187200, SX126X_GFSK_BW_187200 }, { 234300, SX126X_GFSK_BW_234300 },
    { 312000, SX126X_GFSK_BW_312000 }, { 373600, SX126X_GFSK_BW_373600 }, { 467000, SX126X_GFSK_BW_467000 },
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static uint16_t NSS;
static GPIO_TypeDef* NSS_GPIO;
static uint16_t NRESET;
static GPIO_TypeDef* NRESET_GPIO;
static uint16_t BUSY;
static GPIO_TypeDef* BUSY_GPIO;
static uint16_t DIO1;
static GPIO_TypeDef* DIO1_GPIO;
static uint16_t DIO2;
static GPIO_TypeDef* DIO2_GPIO;
static uint16_t DIO3;
static GPIO_TypeDef* DIO3_GPIO;

static SPI_HandleTypeDef hspi;

// ---------------- RADIOS PARAMS -----------------------------------------------------------------
static uint32_t frequency = 448000000;
static sx126x_pkt_type_t packet_type = SX126X_PKT_TYPE_LORA;
static sx126x_fallback_modes_t fallback_mode = SX126X_FALLBACK_STDBY_RC;
static sx126x_tcxo_ctrl_voltages_t tcxo_voltage_ctrl = SX126X_TCXO_CTRL_3_0V;
static sx126x_reg_mod_t regulator_mode = SX126X_REG_MODE_DCDC;

// Calibration parameters
static sx126x_cal_mask_t cal_mask = SX126X_CAL_ALL;
static uint8_t cal_low_freq = 0x6F; //440MHz
static uint8_t cal_hi_freq = 0x75; //470MHz

// Power Amplifier
static sx126x_pa_duty_cycle_t pa_duty_cyc = SX126X_PA_DUTY_CYCLE_17dBm;
static sx126x_pa_hp_max_t pa_hp_max = SX126X_PA_HP_MAX_14dBm;
static sx126x_pa_device_sel_t pa_device_sel = SX126X_PA_DEVICE_SEL_1262;
static uint8_t pa_lut = 0x01;

// Tx Params
static sx126x_output_tx_power_t tx_power = SX126X_TX_POWER_PLUS_22_dBm;
static sx126x_ramp_time_t ramp_time = SX126X_PA_RAMP_200_US;

// Buffer addresses
static uint8_t tx_address_base = 0x00;
static uint8_t rx_address_base = 0x00;

// LoRa modulation params
static sx126x_lora_sf_t lora_sf = SX126X_LORA_SF5;      //spreading factor
static sx126x_lora_bw_t lora_bw = SX126X_LORA_BW_500;   //bandwidth
static sx126x_lora_cr_t lora_cr = SX126X_LORA_CR_4_5;   //coding rate
static uint8_t lora_ldro = 0x00;                        //low data rate optimizer

// LoRa params
static uint16_t pkt_preamble_len = 12;
static sx126x_lora_pkt_len_modes_t header_type = SX126X_LORA_PKT_EXPLICIT;
static uint8_t payload_len = 0xFF;                      //variable length
static bool crc_is_on = true;
static bool invert_iq_is_on = false;                    //iq polarity setup
static bool timer_is_stopped_at_header = false;         //for rx timer for timeout is stopped at header if true
static uint8_t nb_symbols_for_valid_rx = 0;             //how many lora chars received to make sure its lora

// IRQ and DIO params
static sx126x_irq_mask_t irq_mask = SX126X_IRQ_ALL;
static sx126x_irq_mask_t dio1_mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT;
static sx126x_irq_mask_t dio2_mask = SX126X_IRQ_NONE;   //tcxo
static sx126x_irq_mask_t dio3_mask = SX126X_IRQ_NONE;   //ant switch

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief 15.1.2 Workaround
 *
 * @remark Before any packet transmission, bit #2 of SX126X_REG_TX_MODULATION shall be set to:
 * 0 if the LoRa BW = 500 kHz
 * 1 for any other LoRa BW
 * 1 for any (G)FSK configuration
 *
 * @param [in] context Chip implementation context.
 * @param [in] pkt_type The modulation type (G)FSK/LoRa
 * @param [in] bw In case of LoRa modulation the bandwith must be specified
 *
 * @returns Operation status
 */
static sx126x_status_t sx126x_tx_modulation_workaround( const void* context, sx126x_pkt_type_t pkt_type,
                                                        sx126x_lora_bw_t bw );

static inline uint32_t sx126x_get_gfsk_crc_len_in_bytes( sx126x_gfsk_crc_types_t crc_type );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * Radio data transfer - write
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to the opcode be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to the data be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */


sx126x_hal_status_t sx126x_hal_write( const void* hspi, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length ){
    HAL_StatusTypeDef status;
    while(HAL_GPIO_ReadPin(BUSY_GPIO,BUSY) == GPIO_PIN_SET);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi, command, command_length, 100);
    status = HAL_SPI_Transmit(hspi, data, data_length, 100);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_SET);
    return status;
}

/**
 * Radio data transfer - read
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to the opcode be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer that receives the data
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_read( const void* hspi, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint8_t offset ){
    HAL_StatusTypeDef status;
    while(HAL_GPIO_ReadPin(BUSY_GPIO, BUSY) == GPIO_PIN_SET);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi, command, offset, 100);
    status = HAL_SPI_TransmitReceive(hspi, command+offset, data, command_length-offset, 100);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_SET);
    return status;
}

/**
 * Reset the radio
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_reset( const void* hspi ){
    HAL_StatusTypeDef status = HAL_OK;
    HAL_GPIO_WritePin(NRESET_GPIO, NRESET, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(NRESET_GPIO, NRESET, GPIO_PIN_SET);
    return status;
}

/**
 * Wake the radio up.
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_wakeup( const void* hspi ){
    HAL_StatusTypeDef status = HAL_OK;
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_RESET);
    return status;
}

HAL_StatusTypeDef writeCommand(uint8_t opcode, uint8_t params[], uint16_t numOfParams){
    HAL_StatusTypeDef status;
    while(HAL_GPIO_ReadPin(BUSY_GPIO, BUSY) == GPIO_PIN_SET);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&hspi, &opcode, 1, 100);
    status = HAL_SPI_Transmit(&hspi, (uint8_t*)params, numOfParams, 100);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_SET);
    return status;
}

HAL_StatusTypeDef readCommand(uint8_t opcode, uint8_t params[], uint8_t response[], uint16_t numOfParams){
    HAL_StatusTypeDef status;
    while(HAL_GPIO_ReadPin(BUSY_GPIO, BUSY) == GPIO_PIN_SET);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&hspi, &opcode, 1, 100);
    status = HAL_SPI_TransmitReceive(&hspi, (uint8_t*)params, (uint8_t*)response, numOfParams, 100);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_SET);
    return status;
}

void set_NSS_pin(GPIO_TypeDef* _NSS_GPIO, uint16_t _NSS){
    NSS = _NSS;
    NSS_GPIO = _NSS_GPIO;
}

void set_BUSY_pin(GPIO_TypeDef* _BUSY_GPIO, uint16_t _BUSY){
    BUSY = _BUSY;
    BUSY_GPIO = _BUSY_GPIO;
}

void set_NRESET_pin(GPIO_TypeDef* _NRESET_GPIO, uint16_t _NRESET){
    NRESET = _NRESET;
    NRESET_GPIO = _NRESET_GPIO;
}

void set_DIO1_pin(GPIO_TypeDef* _DIO1_GPIO, uint16_t _DIO1){
    DIO1 = _DIO1;
    DIO1_GPIO = _DIO1_GPIO;
}

void set_DIO2_pin(GPIO_TypeDef* _DIO2_GPIO, uint16_t _DIO2){
    DIO2 = _DIO2;
    DIO2_GPIO = _DIO2_GPIO;
}

void set_DIO3_pin(GPIO_TypeDef* _DIO3_GPIO, uint16_t _DIO3){
    DIO3 = _DIO3;
    DIO3_GPIO = _DIO3_GPIO;
}

void set_hspi(SPI_HandleTypeDef _hspi){
    hspi = _hspi;
}

void Tx_setup(){
    //NEED TO ADD COMMAND ERROR HANDLING
    HAL_GPIO_WritePin(NRESET_GPIO, NRESET, GPIO_PIN_SET);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_SET);             //make sure chip select is off

    //set to standby for setup
    sx126x_set_standby(&hspi, 0);

    //set general parameters
    sx126x_set_rf_freq(&hspi, frequency);                       //set rf frequency
    sx126x_set_pkt_type(&hspi, packet_type);                    //set packet type
    sx126x_set_rx_tx_fallback_mode(&hspi, fallback_mode);       //set rx tx fallback mode
    sx126x_set_dio2_as_rf_sw_ctrl(&hspi, 1);                    //set dio2 as rf sw -> 1 is to activate it 0 would be to have it as regular irq
    sx126x_set_dio3_as_tcxo_ctrl(&hspi, tcxo_voltage_ctrl, 100);//set dio3 as tcxo ctrl, 100 is for the delay in ms

    //calibrate functions
    sx126x_cal(&hspi, cal_mask);                                //calibrate radio, mask chooses what to calibrate
    HAL_Delay(50);
    sx126x_set_reg_mode(&hspi, regulator_mode);                 //ldo or dc-dc
    sx126x_cal_img_hex(&hspi, cal_low_freq, cal_hi_freq);       //image calibration frequencies

    //set pa config
    struct sx126x_pa_cfg_params_s *params = malloc(sizeof(sx126x_pa_cfg_params_t));
    params->pa_duty_cycle=pa_duty_cyc;
    params->hp_max=pa_hp_max;
    params->device_sel=pa_device_sel;
    params->pa_lut=pa_lut;
    sx126x_set_pa_cfg(&hspi, params);
    free(params);

    //set transmission parameters
    sx126x_set_tx_params(&hspi, tx_power, ramp_time);
    sx126x_set_buffer_base_address(&hspi, tx_address_base, rx_address_base);

    //set modulation parameters
    struct sx126x_mod_params_lora_s *mod_params = malloc(sizeof(sx126x_mod_params_lora_t));
    mod_params->sf=lora_sf;
    mod_params->bw=lora_bw;
    mod_params->cr=lora_cr;
    mod_params->ldro=lora_ldro;
    sx126x_set_lora_mod_params(&hspi, mod_params);
    free(mod_params);

    //set lora packet params
    struct sx126x_pkt_params_lora_s *lora_params = malloc(sizeof(sx126x_pkt_params_lora_t));
    lora_params->preamble_len_in_symb=pkt_preamble_len;
    lora_params->header_type=header_type;
    lora_params->pld_len_in_bytes=payload_len;
    lora_params->crc_is_on=crc_is_on;
    lora_params->invert_iq_is_on=invert_iq_is_on;
    sx126x_set_lora_pkt_params(&hspi, lora_params);
    free(lora_params);

    //set dio and irq parameters
    sx126x_set_dio_irq_params(&hspi, irq_mask, dio1_mask, dio2_mask, dio3_mask);

}

void TxProtocol(uint8_t data[], uint8_t data_length){

    HAL_StatusTypeDef command_status;
    command_status = sx126x_clear_irq_status(&hspi, dio1_mask);
    command_status = sx126x_write_buffer(&hspi, 0, data, data_length); // 0 is the offset
    command_status = sx126x_set_tx(&hspi, 6000, data_length);
    HAL_Delay(1000);

    sx126x_irq_mask_t irq;
    do {
        command_status = sx126x_get_irq_status(&hspi, &irq); //reading the irq into irq
    } while ( (!(irq & SX126X_IRQ_TX_DONE)) && (!(irq & SX126X_IRQ_TIMEOUT)) );
}

void Rx_setup(){
    //NEED TO ADD COMMAND ERROR HANDLING
    HAL_GPIO_WritePin(NRESET_GPIO, NRESET, GPIO_PIN_SET);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(NSS_GPIO, NSS, GPIO_PIN_SET);             //make sure chip select is off

    //set to standby for setup
    sx126x_set_standby(&hspi, 0);

    //set general parameters
    sx126x_set_rf_freq(&hspi, frequency);                       //set rf frequency
    sx126x_set_pkt_type(&hspi, packet_type);                    //set packet type
    sx126x_set_rx_tx_fallback_mode(&hspi, fallback_mode);       //set rx tx fallback mode
    sx126x_set_dio2_as_rf_sw_ctrl(&hspi, 1);                    //set dio2 as rf sw -> 1 is to activate it 0 would be to have it as regular irq
    sx126x_set_dio3_as_tcxo_ctrl(&hspi, tcxo_voltage_ctrl, 100);//set dio3 as tcxo ctrl, 100 is for the delay in ms

    //calibrate functions
    sx126x_cal(&hspi, cal_mask);                                //calibrate radio, mask chooses what to calibrate
    HAL_Delay(50);
    sx126x_set_reg_mode(&hspi, regulator_mode);                 //ldo or dc-dc
    sx126x_cal_img_hex(&hspi, cal_low_freq, cal_hi_freq);       //image calibration frequencies

    sx126x_set_buffer_base_address(&hspi, tx_address_base, rx_address_base);

    //set modulation parameters
    struct sx126x_mod_params_lora_s *mod_params = malloc(sizeof(sx126x_mod_params_lora_t));
    mod_params->sf=lora_sf;
    mod_params->bw=lora_bw;
    mod_params->cr=lora_cr;
    mod_params->ldro=lora_ldro;
    sx126x_set_lora_mod_params(&hspi, mod_params);
    free(mod_params);

    //set lora packet params
    struct sx126x_pkt_params_lora_s *lora_params = malloc(sizeof(sx126x_pkt_params_lora_t));
    lora_params->preamble_len_in_symb=pkt_preamble_len;
    lora_params->header_type=header_type;
    lora_params->pld_len_in_bytes=payload_len;
    lora_params->crc_is_on=crc_is_on;
    lora_params->invert_iq_is_on=invert_iq_is_on;
    sx126x_set_lora_pkt_params(&hspi, lora_params);
    free(lora_params);

    //set dio and irq parameters
    sx126x_set_dio_irq_params(&hspi, irq_mask, dio1_mask, dio2_mask, dio3_mask);

    //timer for TIMEOUT params
    sx126x_stop_timer_on_preamble(&hspi, timer_is_stopped_at_header);
    sx126x_set_lora_symb_nb_timeout(&hspi, nb_symbols_for_valid_rx);


}

void RxProtocol(uint8_t buffer_received[]){

    HAL_StatusTypeDef command_status;
    command_status = sx126x_clear_irq_status(&hspi, dio1_mask);
    command_status = sx126x_set_rx(&hspi, 3000);
    HAL_Delay(1400);

    sx126x_irq_mask_t irq;
    command_status = sx126x_get_irq_status(&hspi, &irq); //reading the irq into irq
    do {
        command_status = sx126x_get_irq_status(&hspi, &irq); //reading the irq into irq
    } while ( (!(irq & SX126X_IRQ_RX_DONE)) && (!(irq & SX126X_IRQ_TIMEOUT)) );

    if (irq & SX126X_IRQ_TIMEOUT) {
        transmitBuffer("RX TIMEOUT!");
        sx126x_clear_irq_status(&hspi, SX126X_IRQ_TIMEOUT);
    } else {
        if (irq & SX126X_IRQ_HEADER_ERROR || irq & SX126X_IRQ_CRC_ERROR) {
            transmitBuffer("CRC ERROR");
            sx126x_clear_irq_status(&hspi, SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_CRC_ERROR);
        } else if (irq & SX126X_IRQ_RX_DONE) {
            command_status = sx126x_read_buffer(&hspi, buffer_received);

            if (command_status != SX126X_STATUS_OK) {
                transmitBuffer("Receive Buffer failed, command status: ");
                transmitStatus(command_status);

                transmitBuffer("IRQ status: ");
                transmitIRQ(irq);

                //maybe add device status
            } else {
                sx126x_get_irq_status(&hspi, &irq);
                transmitBuffer("IRQ Status: ");
                transmitIRQ(irq);

                transmitBuffer("----- RECEIVED DATA -----");
                transmitBuffer(buffer_received);
            }
        }
    }
}

sx126x_status_t sx126x_set_sleep( const void* context, const sx126x_sleep_cfgs_t cfg )
{
    uint8_t buf[SX126X_SIZE_SET_SLEEP] = { 0 };

    buf[0] = SX126X_SET_SLEEP;

    buf[1] = ( uint8_t ) cfg;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_SLEEP, 0, 0 );
}

sx126x_status_t sx126x_set_standby( const void* context, const sx126x_standby_cfg_t cfg )
{
    uint8_t buf[SX126X_SIZE_SET_STANDBY] = { 0 };

    buf[0] = SX126X_SET_STANDBY;

    buf[1] = ( uint8_t ) cfg;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_STANDBY, 0, 0 );
}

sx126x_status_t sx126x_set_fs( const void* context )
{
    uint8_t buf[SX126X_SIZE_SET_FS] = { 0 };

    buf[0] = SX126X_SET_FS;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_FS, 0, 0 );
}

sx126x_status_t sx126x_set_tx( const void* context, const uint32_t timeout_in_ms , uint8_t lora_data_length)
{
    if (packet_type == SX126X_PKT_TYPE_LORA) {
        struct sx126x_pkt_params_lora_s *lora_params = malloc(sizeof(sx126x_pkt_params_lora_t));
        lora_params->preamble_len_in_symb=pkt_preamble_len;
        lora_params->header_type=header_type;
        lora_params->pld_len_in_bytes=lora_data_length;
        lora_params->crc_is_on=crc_is_on;
        lora_params->invert_iq_is_on=invert_iq_is_on;
        sx126x_set_lora_pkt_params(&hspi, lora_params);
        free(lora_params);
    }

    if( timeout_in_ms > SX126X_MAX_TIMEOUT_IN_MS )
    {
        return SX126X_STATUS_UNKNOWN_VALUE;
    }

    const uint32_t timeout_in_rtc_step = sx126x_convert_timeout_in_ms_to_rtc_step( timeout_in_ms );

    return sx126x_set_tx_with_timeout_in_rtc_step( context, timeout_in_rtc_step );
}

sx126x_status_t sx126x_set_tx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step )
{
    uint8_t buf[SX126X_SIZE_SET_TX] = { 0 };

    buf[0] = SX126X_SET_TX;

    buf[1] = ( uint8_t )( timeout_in_rtc_step >> 16 );
    buf[2] = ( uint8_t )( timeout_in_rtc_step >> 8 );
    buf[3] = ( uint8_t )( timeout_in_rtc_step >> 0 );

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX, 0, 0 );
}

sx126x_status_t sx126x_set_rx( const void* context, const uint32_t timeout_in_ms )
{
    if( timeout_in_ms > SX126X_MAX_TIMEOUT_IN_MS )
    {
        return SX126X_STATUS_UNKNOWN_VALUE;
    }

    const uint32_t timeout_in_rtc_step = sx126x_convert_timeout_in_ms_to_rtc_step( timeout_in_ms );

    return sx126x_set_rx_with_timeout_in_rtc_step( context, timeout_in_rtc_step );
}

sx126x_status_t sx126x_set_rx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step )
{
    uint8_t buf[SX126X_SIZE_SET_RX] = { 0 };

    buf[0] = SX126X_SET_RX;

    buf[1] = ( uint8_t )( timeout_in_rtc_step >> 16 );
    buf[2] = ( uint8_t )( timeout_in_rtc_step >> 8 );
    buf[3] = ( uint8_t )( timeout_in_rtc_step >> 0 );

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX, 0, 0 );
}

sx126x_status_t sx126x_stop_timer_on_preamble( const void* context, const bool enable )
{
    uint8_t buf[SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE] = { 0 };

    buf[0] = SX126X_SET_STOP_TIMER_ON_PREAMBLE;

    buf[1] = ( enable == true ) ? 1 : 0;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE, 0, 0 );
}

sx126x_status_t sx126x_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms,
                                          const uint32_t sleep_time_in_ms )
{
    const uint32_t rx_time_in_rtc_step    = sx126x_convert_timeout_in_ms_to_rtc_step( rx_time_in_ms );
    const uint32_t sleep_time_in_rtc_step = sx126x_convert_timeout_in_ms_to_rtc_step( sleep_time_in_ms );

    return sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( context, rx_time_in_rtc_step, sleep_time_in_rtc_step );
}

sx126x_status_t sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( const void*    context,
                                                                   const uint32_t rx_time_in_rtc_step,
                                                                   const uint32_t sleep_time_in_rtc_step )
{
    uint8_t buf[SX126X_SIZE_SET_RX_DUTY_CYCLE] = { 0 };

    buf[0] = SX126X_SET_RX_DUTY_CYCLE;

    buf[1] = ( uint8_t )( rx_time_in_rtc_step >> 16 );
    buf[2] = ( uint8_t )( rx_time_in_rtc_step >> 8 );
    buf[3] = ( uint8_t )( rx_time_in_rtc_step >> 0 );

    buf[4] = ( uint8_t )( sleep_time_in_rtc_step >> 16 );
    buf[5] = ( uint8_t )( sleep_time_in_rtc_step >> 8 );
    buf[6] = ( uint8_t )( sleep_time_in_rtc_step >> 0 );

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX_DUTY_CYCLE, 0, 0 );
}

sx126x_status_t sx126x_set_cad( const void* context )
{
    uint8_t buf[SX126X_SIZE_SET_CAD] = { 0 };

    buf[0] = SX126X_SET_CAD;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_CAD, 0, 0 );
}

sx126x_status_t sx126x_set_tx_cw( const void* context )
{
    uint8_t buf[SX126X_SIZE_SET_TX_CONTINUOUS_WAVE] = { 0 };

    buf[0] = SX126X_SET_TX_CONTINUOUS_WAVE;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_CONTINUOUS_WAVE, 0, 0 );
}

sx126x_status_t sx126x_set_tx_infinite_preamble( const void* context )
{
    uint8_t buf[SX126X_SIZE_SET_TX_INFINITE_PREAMBLE] = { 0 };

    buf[0] = SX126X_SET_TX_INFINITE_PREAMBLE;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_INFINITE_PREAMBLE, 0, 0 );
}

sx126x_status_t sx126x_set_reg_mode( const void* context, const sx126x_reg_mod_t mode )
{
    uint8_t buf[SX126X_SIZE_SET_REGULATOR_MODE] = { 0 };

    buf[0] = SX126X_SET_REGULATOR_MODE;

    buf[1] = ( uint8_t ) mode;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_REGULATOR_MODE, 0, 0 );
}

sx126x_status_t sx126x_cal( const void* context, const sx126x_cal_mask_t param )
{
    uint8_t buf[SX126X_SIZE_CALIBRATE] = { 0 };

    buf[0] = SX126X_CALIBRATE;

    buf[1] = param;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CALIBRATE, 0, 0 );
}

sx126x_status_t sx126x_cal_img_hex( const void* context, const uint8_t low_freq, const uint8_t high_freq)
{
    uint8_t buf[SX126X_SIZE_CALIBRATE_IMAGE] = { 0 };

    buf[0] = SX126X_CALIBRATE_IMAGE;
    buf[1] = low_freq;
    buf[2] = high_freq;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CALIBRATE_IMAGE, 0, 0 );
}

sx126x_status_t sx126x_cal_img_freq( const void* context, const uint32_t freq_in_hz )
{
    uint8_t buf[SX126X_SIZE_CALIBRATE_IMAGE] = { 0 };

    buf[0] = SX126X_CALIBRATE_IMAGE;

    if( freq_in_hz > 900000000 )
    {
        buf[1] = 0xE1;
        buf[2] = 0xE9;
    }
    else if( freq_in_hz > 850000000 )
    {
        buf[1] = 0xD7;
        buf[2] = 0xDB;
    }
    else if( freq_in_hz > 770000000 )
    {
        buf[1] = 0xC1;
        buf[2] = 0xC5;
    }
    else if( freq_in_hz > 460000000 )
    {
        buf[1] = 0x75;
        buf[2] = 0x81;
    }
    else
    {
        buf[1] = 0x6B;
        buf[2] = 0x6F;
    }

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CALIBRATE_IMAGE, 0, 0 );
}

sx126x_status_t sx126x_set_pa_cfg( const void* context, const sx126x_pa_cfg_params_t* params )
{
    uint8_t buf[SX126X_SIZE_SET_PA_CFG] = { 0 };

    buf[0] = SX126X_SET_PA_CFG;
    buf[1] = params->pa_duty_cycle;
    buf[2] = params->hp_max;
    buf[3] = params->device_sel;
    buf[4] = params->pa_lut;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PA_CFG, 0, 0 );
}

sx126x_status_t sx126x_set_rx_tx_fallback_mode( const void* context, const sx126x_fallback_modes_t fallback_mode )
{
    uint8_t buf[SX126X_SIZE_SET_RX_TX_FALLBACK_MODE] = { 0 };

    buf[0] = SX126X_SET_RX_TX_FALLBACK_MODE;

    buf[1] = ( uint8_t ) fallback_mode;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX_TX_FALLBACK_MODE, 0, 0 );
}

//
// Registers and buffer Access
//

sx126x_status_t sx126x_write_register( const void* context, const uint16_t address, const uint8_t* buffer,
                                       const uint8_t size )
{
    uint8_t buf[SX126X_SIZE_WRITE_REGISTER] = { 0 };

    buf[0] = SX126X_WRITE_REGISTER;

    buf[1] = ( uint8_t )( address >> 8 );
    buf[2] = ( uint8_t )( address >> 0 );

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_WRITE_REGISTER, buffer, size );
}

sx126x_status_t sx126x_read_register( const void* context, const uint16_t address, uint8_t* buffer, const uint8_t size )
{
    uint8_t         buf[SX126X_SIZE_READ_REGISTER] = { 0 };
    sx126x_status_t status                         = SX126X_STATUS_ERROR;

    buf[0] = SX126X_READ_REGISTER;

    buf[1] = ( uint8_t )( address >> 8 );
    buf[2] = ( uint8_t )( address >> 0 );

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_READ_REGISTER, buffer, size );

    return status;
}

sx126x_status_t sx126x_write_buffer( const void* context, const uint8_t buffer_offset, const uint8_t* buffer,
                                     const uint8_t size )
{
    uint8_t buf[SX126X_SIZE_WRITE_BUFFER] = { 0 };

    buf[0] = SX126X_WRITE_BUFFER;

    buf[1] = buffer_offset;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_WRITE_BUFFER, buffer, size );
}

sx126x_status_t sx126x_read_buffer( const void* context, uint8_t* buffer)
{
    sx126x_status_t status                       = SX126X_STATUS_ERROR;
	sx126x_rx_buffer_status_t buffer_status;
    sx126x_get_rx_buffer_status(&hspi, &buffer_status);
    uint8_t size = buffer_status.pld_len_in_bytes;
    uint8_t offset = buffer_status.buffer_start_pointer;

    uint8_t received_buf[size + 1];
    uint8_t cmd_buf[size + 1 + SX126X_OFFSET_READ_BUFFER];

    for (uint8_t i=2; i<size+1+SX126X_OFFSET_READ_BUFFER; i++){
        cmd_buf[i] = 0x00;
    }

    cmd_buf[0] = SX126X_READ_BUFFER;
    cmd_buf[1] = offset;

    status = ( sx126x_status_t ) sx126x_hal_read( context, cmd_buf, SX126X_OFFSET_READ_BUFFER + size, received_buf,  SX126X_OFFSET_READ_BUFFER);

    if (status == SX126X_STATUS_OK) {
        status = received_buf[0];

        //CO U LD T R Y T O U SE SPRINTF
        for (uint8_t i=1; i<=size; i++){
            buffer[i-1] = received_buf[i];
        }
    }
    //ADD FUCKING RETURN STATUS FROM RECEIVED_BUF[0]
    return status;
}

//
// DIO and IRQ Control Functions
//
sx126x_status_t sx126x_set_dio_irq_params( const void* context, const uint16_t irq_mask, const uint16_t dio1_mask,
                                           const uint16_t dio2_mask, const uint16_t dio3_mask )
{
    uint8_t buf[SX126X_SIZE_SET_DIO_IRQ_PARAMS] = { 0 };

    buf[0] = SX126X_SET_DIO_IRQ_PARAMS;

    buf[1] = ( uint8_t )( irq_mask >> 8 );
    buf[2] = ( uint8_t )( irq_mask >> 0 );

    buf[3] = ( uint8_t )( dio1_mask >> 8 );
    buf[4] = ( uint8_t )( dio1_mask >> 0 );

    buf[5] = ( uint8_t )( dio2_mask >> 8 );
    buf[6] = ( uint8_t )( dio2_mask >> 0 );

    buf[7] = ( uint8_t )( dio3_mask >> 8 );
    buf[8] = ( uint8_t )( dio3_mask >> 0 );

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_DIO_IRQ_PARAMS, 0, 0 );
}

sx126x_status_t sx126x_get_irq_status( const void* context, sx126x_irq_mask_t* irq )
{
    uint8_t         buf[SX126X_SIZE_GET_IRQ_STATUS]             = { 0x00 }; //0x00 is no operational, its just so that theyre equal
    uint8_t         received_buf[sizeof( sx126x_irq_mask_t )+1] = { 0x00 };
    sx126x_status_t status                                      = SX126X_STATUS_ERROR;

    buf[0] = SX126X_GET_IRQ_STATUS;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_IRQ_STATUS, received_buf,
                                                  SX126X_OFFSET_GET_IRQ_STATUS );

    if( status == SX126X_STATUS_OK )
    {
        *irq = ( ( sx126x_irq_mask_t ) received_buf[1] << 8 ) | ( ( sx126x_irq_mask_t ) received_buf[2] << 0 );
        status = received_buf[0]; //its the status
    }

    return status;
}

sx126x_status_t sx126x_clear_irq_status( const void* context, const sx126x_irq_mask_t irq_mask )
{
    uint8_t buf[SX126X_SIZE_CLR_IRQ_STATUS] = { 0 };

    buf[0] = SX126X_CLR_IRQ_STATUS;

    buf[1] = ( uint8_t )( irq_mask >> 8 );
    buf[2] = ( uint8_t )( irq_mask >> 0 );

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CLR_IRQ_STATUS, 0, 0 );
}

sx126x_status_t sx126x_get_and_clear_irq_status( const void* context, sx126x_irq_mask_t* irq )
{
    sx126x_irq_mask_t sx126x_irq_mask = SX126X_IRQ_NONE;

    sx126x_status_t status = sx126x_get_irq_status( context, &sx126x_irq_mask );

    if( ( status == SX126X_STATUS_OK ) && ( sx126x_irq_mask != 0 ) )
    {
        status = sx126x_clear_irq_status( context, sx126x_irq_mask );
    }
    if( ( status == SX126X_STATUS_OK ) && ( irq != NULL ) )
    {
        *irq = sx126x_irq_mask;
    }
    return status;
}

sx126x_status_t sx126x_set_dio2_as_rf_sw_ctrl( const void* context, const bool enable )
{
    uint8_t buf[SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL] = { 0 };

    buf[0] = SX126X_SET_DIO2_AS_RF_SWITCH_CTRL;

    buf[1] = ( enable == true ) ? 1 : 0;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL, 0, 0 );
}

sx126x_status_t sx126x_set_dio3_as_tcxo_ctrl( const void* context, const sx126x_tcxo_ctrl_voltages_t tcxo_voltage,
                                              const uint32_t timeout )
{
    uint8_t buf[SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL] = { 0 };

    buf[0] = SX126X_SET_DIO3_AS_TCXO_CTRL;

    buf[1] = ( uint8_t ) tcxo_voltage;

    buf[2] = ( uint8_t )( timeout >> 16 );
    buf[3] = ( uint8_t )( timeout >> 8 );
    buf[4] = ( uint8_t )( timeout >> 0 );

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL, 0, 0 );
}

//
// RF Modulation and Packet-Related Functions
//

sx126x_status_t sx126x_set_rf_freq( const void* context, const uint32_t freq_in_hz )
{
    const uint32_t freq = sx126x_convert_freq_in_hz_to_pll_step( freq_in_hz );

    return sx126x_set_rf_freq_in_pll_steps( context, freq );
}

sx126x_status_t sx126x_set_rf_freq_in_pll_steps( const void* context, const uint32_t freq )
{
    uint8_t buf[SX126X_SIZE_SET_RF_FREQUENCY] = { 0 };

    buf[0] = SX126X_SET_RF_FREQUENCY;

    buf[1] = ( uint8_t )( freq >> 24 );
    buf[2] = ( uint8_t )( freq >> 16 );
    buf[3] = ( uint8_t )( freq >> 8 );
    buf[4] = ( uint8_t )( freq >> 0 );

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RF_FREQUENCY, 0, 0 );
}

sx126x_status_t sx126x_set_pkt_type( const void* context, const sx126x_pkt_type_t pkt_type )
{
    uint8_t buf[SX126X_SIZE_SET_PKT_TYPE] = { 0 };

    buf[0] = SX126X_SET_PKT_TYPE;

    buf[1] = ( uint8_t ) pkt_type;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_TYPE, 0, 0 );
}

sx126x_status_t sx126x_get_pkt_type( const void* context, sx126x_pkt_type_t* pkt_type )
{
    uint8_t         buf[SX126X_SIZE_GET_PKT_TYPE] = { 0 };
    sx126x_status_t status                        = SX126X_STATUS_ERROR;
    uint8_t         pkt_type_raw[2];

    buf[0] = SX126X_GET_PKT_TYPE;

    status    = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_PKT_TYPE, pkt_type_raw, 1 );
    *pkt_type = ( sx126x_pkt_type_t ) pkt_type_raw[1];

    return status;
}

sx126x_status_t sx126x_set_tx_params( const void* context, const int8_t pwr_in_dbm, const sx126x_ramp_time_t ramp_time )
{
    uint8_t buf[SX126X_SIZE_SET_TX_PARAMS] = { 0 };

    buf[0] = SX126X_SET_TX_PARAMS;

    buf[1] = ( uint8_t ) pwr_in_dbm;
    buf[2] = ( uint8_t ) ramp_time;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_PARAMS, 0, 0 );
}

sx126x_status_t sx126x_set_gfsk_mod_params( const void* context, const sx126x_mod_params_gfsk_t* params )
{
    sx126x_status_t status                                      = SX126X_STATUS_ERROR;
    uint8_t         buf[SX126X_SIZE_SET_MODULATION_PARAMS_GFSK] = { 0 };
    uint32_t        bitrate = ( uint32_t )( 32 * SX126X_XTAL_FREQ / params->br_in_bps );
    const uint32_t  fdev    = sx126x_convert_freq_in_hz_to_pll_step( params->fdev_in_hz );

    buf[0] = SX126X_SET_MODULATION_PARAMS;

    buf[1] = ( uint8_t )( bitrate >> 16 );
    buf[2] = ( uint8_t )( bitrate >> 8 );
    buf[3] = ( uint8_t )( bitrate >> 0 );

    buf[4] = ( uint8_t )( params->pulse_shape );

    buf[5] = params->bw_dsb_param;

    buf[6] = ( uint8_t )( fdev >> 16 );
    buf[7] = ( uint8_t )( fdev >> 8 );
    buf[8] = ( uint8_t )( fdev >> 0 );

    status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_MODULATION_PARAMS_GFSK, 0, 0 );

    if( status == SX126X_STATUS_OK )
    {
        // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
        status = sx126x_tx_modulation_workaround( context, SX126X_PKT_TYPE_GFSK, ( sx126x_lora_bw_t ) 0 );
        // WORKAROUND END
    }
    return status;
}

sx126x_status_t sx126x_set_lora_mod_params( const void* context, const sx126x_mod_params_lora_t* params )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;

    uint8_t buf[SX126X_SIZE_SET_MODULATION_PARAMS_LORA] = { 0 };

    buf[0] = SX126X_SET_MODULATION_PARAMS;

    buf[1] = ( uint8_t )( params->sf );
    buf[2] = ( uint8_t )( params->bw );
    buf[3] = ( uint8_t )( params->cr );
    buf[4] = params->ldro & 0x01;

    status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_MODULATION_PARAMS_LORA, 0, 0 );
    /*
    if( status == SX126X_STATUS_OK )
    {
        // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see datasheet DS_SX1261-2_V1.2 §15.1
        status = sx126x_tx_modulation_workaround( context, SX126X_PKT_TYPE_LORA, params->bw );
        // WORKAROUND END
    }
    */

    return status;
}

sx126x_status_t sx126x_set_gfsk_pkt_params( const void* context, const sx126x_pkt_params_gfsk_t* params )
{
    uint8_t buf[SX126X_SIZE_SET_PKT_PARAMS_GFSK] = { 0 };

    buf[0] = SX126X_SET_PKT_PARAMS;

    buf[1] = ( uint8_t )( params->preamble_len_in_bits >> 8 );
    buf[2] = ( uint8_t )( params->preamble_len_in_bits >> 0 );
    buf[3] = ( uint8_t )( params->preamble_detector );
    buf[4] = params->sync_word_len_in_bits;
    buf[5] = ( uint8_t )( params->address_filtering );
    buf[6] = ( uint8_t )( params->header_type );
    buf[7] = params->pld_len_in_bytes;
    buf[8] = ( uint8_t )( params->crc_type );
    buf[9] = ( uint8_t )( params->dc_free );

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_PARAMS_GFSK, 0, 0 );
}

sx126x_status_t sx126x_set_lora_pkt_params( const void* context, const sx126x_pkt_params_lora_t* params )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;

    uint8_t buf[SX126X_SIZE_SET_PKT_PARAMS_LORA] = { 0 };

    buf[0] = SX126X_SET_PKT_PARAMS;

    buf[1] = ( uint8_t )( params->preamble_len_in_symb >> 8 );
    buf[2] = ( uint8_t )( params->preamble_len_in_symb >> 0 );
    buf[3] = ( uint8_t )( params->header_type );
    buf[4] = params->pld_len_in_bytes;
    buf[5] = ( uint8_t )( params->crc_is_on ? 1 : 0 );
    buf[6] = ( uint8_t )( params->invert_iq_is_on ? 1 : 0 );

    status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_PARAMS_LORA, 0, 0 );

    // WORKAROUND - Optimizing the Inverted IQ Operation, see datasheet DS_SX1261-2_V1.2 §15.4
    if( status == SX126X_STATUS_OK )
    {
        uint8_t reg_value = 0;

        status = sx126x_read_register( context, SX126X_REG_IRQ_POLARITY, &reg_value, 1 );
        if( status == SX126X_STATUS_OK )
        {
            if( params->invert_iq_is_on == true )
            {
                reg_value &= ~( 1 << 2 );  // Bit 2 set to 0 when using inverted IQ polarity
            }
            else
            {
                reg_value |= ( 1 << 2 );  // Bit 2 set to 1 when using standard IQ polarity
            }
            status = sx126x_write_register( context, SX126X_REG_IRQ_POLARITY, &reg_value, 1 );
        }
    }
    // WORKAROUND END

    return status;
}

sx126x_status_t sx126x_set_cad_params( const void* context, const sx126x_cad_params_t* params )
{
    uint8_t buf[SX126X_SIZE_SET_CAD_PARAMS] = { 0 };

    buf[0] = SX126X_SET_CAD_PARAMS;

    buf[1] = ( uint8_t ) params->cad_symb_nb;
    buf[2] = params->cad_detect_peak;
    buf[3] = params->cad_detect_min;
    buf[4] = ( uint8_t ) params->cad_exit_mode;
    buf[5] = ( uint8_t )( params->cad_timeout >> 16 );
    buf[6] = ( uint8_t )( params->cad_timeout >> 8 );
    buf[7] = ( uint8_t )( params->cad_timeout >> 0 );

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_CAD_PARAMS, 0, 0 );
}

sx126x_status_t sx126x_set_buffer_base_address( const void* context, const uint8_t tx_base_address,
                                                const uint8_t rx_base_address )
{
    uint8_t buf[SX126X_SIZE_SET_BUFFER_BASE_ADDRESS] = { 0 };

    buf[0] = SX126X_SET_BUFFER_BASE_ADDRESS;

    buf[1] = tx_base_address;
    buf[2] = rx_base_address;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_BUFFER_BASE_ADDRESS, 0, 0 );
}

sx126x_status_t sx126x_set_lora_symb_nb_timeout( const void* context, const uint8_t nb_of_symbs )
{
    uint8_t         buf[SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT] = { 0 };
    sx126x_status_t status                                     = SX126X_STATUS_ERROR;

    buf[0] = SX126X_SET_LORA_SYMB_NUM_TIMEOUT;

    buf[1] = nb_of_symbs;

    status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT, 0, 0 );

    if( ( status == SX126X_STATUS_OK ) && ( nb_of_symbs >= 64 ) )
    {
        uint8_t mant = nb_of_symbs >> 1;
        uint8_t exp  = 0;
        uint8_t reg  = 0;

        while( mant > 31 )
        {
            mant >>= 2;
            exp++;
        }

        reg    = exp + ( mant << 3 );
        status = sx126x_write_register( context, SX126X_REG_LR_SYNCH_TIMEOUT, &reg, 1 );
    }

    return status;
}

//
// Communication Status Information
//

sx126x_status_t sx126x_get_status( const void* context, sx126x_chip_status_t* radio_status )
{
    uint8_t         buf[SX126X_SIZE_GET_STATUS] = { 0 };
    uint8_t         status_local                = 0;
    sx126x_status_t status                      = SX126X_STATUS_ERROR;

    buf[0] = SX126X_GET_STATUS;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_STATUS, &status_local, 1 );

    if( status == SX126X_STATUS_OK )
    {
        radio_status->cmd_status =
            ( sx126x_cmd_status_t )( ( status_local & SX126X_CMD_STATUS_MASK ) >> SX126X_CMD_STATUS_POS );
        radio_status->chip_mode =
            ( sx126x_chip_modes_t )( ( status_local & SX126X_CHIP_MODES_MASK ) >> SX126X_CHIP_MODES_POS );
    }

    return status;
}

sx126x_status_t sx126x_get_rx_buffer_status( const void* context, sx126x_rx_buffer_status_t* rx_buffer_status )
{
    uint8_t         buf[SX126X_SIZE_GET_RX_BUFFER_STATUS]               = { 0x00 };
    uint8_t         status_local[sizeof( sx126x_rx_buffer_status_t )+1] = { 0x00 };
    sx126x_status_t status                                              = SX126X_STATUS_ERROR;

    buf[0] = SX126X_GET_RX_BUFFER_STATUS;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_RX_BUFFER_STATUS, status_local,
                                                  SX126X_OFFSET_GET_RX_BUFFER_STATUS );

    if( status == SX126X_STATUS_OK )
    {
        rx_buffer_status->pld_len_in_bytes     = status_local[1];
        rx_buffer_status->buffer_start_pointer = status_local[2];
    }

    return (sx126x_status_t) status_local[0];
}

sx126x_status_t sx126x_get_gfsk_pkt_status( const void* context, sx126x_pkt_status_gfsk_t* pkt_status )
{
    uint8_t         buf[SX126X_SIZE_GET_PKT_STATUS] = { 0 };
    uint8_t         pkt_status_local[3]             = { 0x00 };
    sx126x_status_t status                          = SX126X_STATUS_ERROR;

    buf[0] = SX126X_GET_PKT_STATUS;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_PKT_STATUS, pkt_status_local, 3 );

    if( status == SX126X_STATUS_OK )
    {
        pkt_status->rx_status.pkt_sent =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_PKT_SENT_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.pkt_received =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_PKT_RECEIVED_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.abort_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_ABORT_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.length_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_LENGTH_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.crc_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_CRC_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.adrs_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_ADRS_ERROR_MASK ) != 0 ) ? true : false;

        pkt_status->rssi_sync = ( int8_t )( -pkt_status_local[1] >> 1 );
        pkt_status->rssi_avg  = ( int8_t )( -pkt_status_local[2] >> 1 );
    }

    return status;
}

sx126x_status_t sx126x_get_lora_pkt_status( const void* context, sx126x_pkt_status_lora_t* pkt_status )
{
    uint8_t         buf[SX126X_SIZE_GET_PKT_STATUS]                      = { 0 };
    uint8_t         pkt_status_local[sizeof( sx126x_pkt_status_lora_t )] = { 0x00 };
    sx126x_status_t status                                               = SX126X_STATUS_ERROR;

    buf[0] = SX126X_GET_PKT_STATUS;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_PKT_STATUS, pkt_status_local,
                                                  sizeof( sx126x_pkt_status_lora_t ) );

    if( status == SX126X_STATUS_OK )
    {
        pkt_status->rssi_pkt_in_dbm        = ( int8_t )( -pkt_status_local[0] >> 1 );
        pkt_status->snr_pkt_in_db          = ( ( ( int8_t ) pkt_status_local[1] ) + 2 ) >> 2;
        pkt_status->signal_rssi_pkt_in_dbm = ( int8_t )( -pkt_status_local[2] >> 1 );
    }

    return status;
}

sx126x_status_t sx126x_get_rssi_inst( const void* context, int16_t* rssi_in_dbm )
{
    uint8_t         buf[SX126X_SIZE_GET_RSSI_INST] = { 0 };
    uint8_t         rssi_local                     = 0x00;
    sx126x_status_t status                         = SX126X_STATUS_ERROR;

    buf[0] = SX126X_GET_RSSI_INST;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_RSSI_INST, &rssi_local, 1 );

    if( status == SX126X_STATUS_OK )
    {
        *rssi_in_dbm = ( int8_t )( -rssi_local >> 1 );
    }

    return status;
}

sx126x_status_t sx126x_get_gfsk_stats( const void* context, sx126x_stats_gfsk_t* stats )
{
    uint8_t         buf[SX126X_SIZE_GET_STATS]                 = { 0 };
    uint8_t         stats_local[sizeof( sx126x_stats_gfsk_t )] = { 0 };
    sx126x_status_t status                                     = SX126X_STATUS_ERROR;

    buf[0] = SX126X_GET_STATS;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_STATS, stats_local,
                                                  sizeof( sx126x_stats_gfsk_t ) );

    if( status == SX126X_STATUS_OK )
    {
        stats->nb_pkt_received  = ( ( uint16_t ) stats_local[0] << 8 ) + ( uint16_t ) stats_local[1];
        stats->nb_pkt_crc_error = ( ( uint16_t ) stats_local[2] << 8 ) + ( uint16_t ) stats_local[3];
        stats->nb_pkt_len_error = ( ( uint16_t ) stats_local[4] << 8 ) + ( uint16_t ) stats_local[5];
    }

    return status;
}

sx126x_status_t sx126x_get_lora_stats( const void* context, sx126x_stats_lora_t* stats )
{
    uint8_t         buf[SX126X_SIZE_GET_STATS]                 = { 0 };
    uint8_t         stats_local[sizeof( sx126x_stats_lora_t )] = { 0 };
    sx126x_status_t status                                     = SX126X_STATUS_ERROR;

    buf[0] = SX126X_GET_STATS;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_STATS, stats_local,
                                                  sizeof( sx126x_stats_lora_t ) );

    if( status == SX126X_STATUS_OK )
    {
        stats->nb_pkt_received     = ( ( uint16_t ) stats_local[0] << 8 ) + ( uint16_t ) stats_local[1];
        stats->nb_pkt_crc_error    = ( ( uint16_t ) stats_local[2] << 8 ) + ( uint16_t ) stats_local[3];
        stats->nb_pkt_header_error = ( ( uint16_t ) stats_local[4] << 8 ) + ( uint16_t ) stats_local[5];
    }
    return status;
}

sx126x_status_t sx126x_reset_stats( const void* context )
{
    uint8_t buf[SX126X_SIZE_RESET_STATS] = { 0 };

    buf[0] = SX126X_RESET_STATS;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_RESET_STATS, 0, 0 );
}

//
// Miscellaneous
//

sx126x_status_t sx126x_reset( const void* context )
{
    return ( sx126x_status_t ) sx126x_hal_reset( context );
}

sx126x_status_t sx126x_wakeup( const void* context )
{
    return ( sx126x_status_t ) sx126x_hal_wakeup( context );
}

sx126x_status_t sx126x_get_device_errors( const void* context, sx126x_errors_mask_t* errors )
{
    uint8_t         buf[SX126X_SIZE_GET_DEVICE_ERRORS]           = { 0 };
    uint8_t         errors_local[sizeof( sx126x_errors_mask_t )] = { 0x00 };
    sx126x_status_t status                                       = SX126X_STATUS_ERROR;

    buf[0] = SX126X_GET_DEVICE_ERRORS;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_DEVICE_ERRORS, errors_local,
                                                  sizeof( sx126x_errors_mask_t ) );

    if( status == SX126X_STATUS_OK )
    {
        *errors = ( ( sx126x_errors_mask_t ) errors_local[0] << 8 ) + ( ( sx126x_errors_mask_t ) errors_local[1] << 0 );
    }

    return status;
}

sx126x_status_t sx126x_clear_device_errors( const void* context )
{
    uint8_t buf[SX126X_SIZE_CLR_DEVICE_ERRORS] = { 0 };

    buf[0] = SX126X_CLR_DEVICE_ERRORS;

    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CLR_DEVICE_ERRORS, 0, 0 );
}

sx126x_status_t sx126x_get_gfsk_bw_param( const uint32_t bw, uint8_t* param )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;

    if( bw != 0 )
    {
        status = SX126X_STATUS_UNKNOWN_VALUE;
        for( uint8_t i = 0; i < ( sizeof( gfsk_bw ) / sizeof( gfsk_bw_t ) ); i++ )
        {
            if( bw <= gfsk_bw[i].bw )
            {
                *param = gfsk_bw[i].param;
                status = SX126X_STATUS_OK;
                break;
            }
        }
    }

    return status;
}

uint32_t sx126x_get_lora_bw_in_hz( sx126x_lora_bw_t bw )
{
    uint32_t bw_in_hz = 0;

    switch( bw )
    {
    case SX126X_LORA_BW_007:
        bw_in_hz = 7812UL;
        break;
    case SX126X_LORA_BW_010:
        bw_in_hz = 10417UL;
        break;
    case SX126X_LORA_BW_015:
        bw_in_hz = 15625UL;
        break;
    case SX126X_LORA_BW_020:
        bw_in_hz = 20833UL;
        break;
    case SX126X_LORA_BW_031:
        bw_in_hz = 31250UL;
        break;
    case SX126X_LORA_BW_041:
        bw_in_hz = 41667UL;
        break;
    case SX126X_LORA_BW_062:
        bw_in_hz = 62500UL;
        break;
    case SX126X_LORA_BW_125:
        bw_in_hz = 125000UL;
        break;
    case SX126X_LORA_BW_250:
        bw_in_hz = 250000UL;
        break;
    case SX126X_LORA_BW_500:
        bw_in_hz = 500000UL;
        break;
    }

    return bw_in_hz;
}

uint32_t sx126x_get_lora_time_on_air_numerator( const sx126x_pkt_params_lora_t* pkt_p,
                                                const sx126x_mod_params_lora_t* mod_p )
{
    const int32_t pld_len_in_bytes = pkt_p->pld_len_in_bytes;
    const int32_t sf               = mod_p->sf;
    const bool    pld_is_fix       = pkt_p->header_type == SX126X_LORA_PKT_IMPLICIT;
    const int32_t cr_denom         = mod_p->cr + 4;

    int32_t ceil_denominator;
    int32_t ceil_numerator =
        ( pld_len_in_bytes << 3 ) + ( pkt_p->crc_is_on ? 16 : 0 ) - ( 4 * sf ) + ( pld_is_fix ? 0 : 20 );

    if( sf <= 6 )
    {
        ceil_denominator = 4 * sf;
    }
    else
    {
        ceil_numerator += 8;

        if( mod_p->ldro )
        {
            ceil_denominator = 4 * ( sf - 2 );
        }
        else
        {
            ceil_denominator = 4 * sf;
        }
    }

    if( ceil_numerator < 0 )
    {
        ceil_numerator = 0;
    }

    // Perform integral ceil()
    int32_t intermed =
        ( ( ceil_numerator + ceil_denominator - 1 ) / ceil_denominator ) * cr_denom + pkt_p->preamble_len_in_symb + 12;

    if( sf <= 6 )
    {
        intermed += 2;
    }

    return ( uint32_t )( ( 4 * intermed + 1 ) * ( 1 << ( sf - 2 ) ) );
}

uint32_t sx126x_get_lora_time_on_air_in_ms( const sx126x_pkt_params_lora_t* pkt_p,
                                            const sx126x_mod_params_lora_t* mod_p )
{
    uint32_t numerator   = 1000U * sx126x_get_lora_time_on_air_numerator( pkt_p, mod_p );
    uint32_t denominator = sx126x_get_lora_bw_in_hz( mod_p->bw );
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

uint32_t sx126x_get_gfsk_time_on_air_numerator( const sx126x_pkt_params_gfsk_t* pkt_p )
{
    return pkt_p->preamble_len_in_bits + ( pkt_p->header_type == SX126X_GFSK_PKT_VAR_LEN ? 8 : 0 ) +
           pkt_p->sync_word_len_in_bits +
           ( ( pkt_p->pld_len_in_bytes + ( pkt_p->address_filtering == SX126X_GFSK_ADDRESS_FILTERING_DISABLE ? 0 : 1 ) +
               sx126x_get_gfsk_crc_len_in_bytes( pkt_p->crc_type ) )
             << 3 );
}

uint32_t sx126x_get_gfsk_time_on_air_in_ms( const sx126x_pkt_params_gfsk_t* pkt_p,
                                            const sx126x_mod_params_gfsk_t* mod_p )
{
    uint32_t numerator   = 1000U * sx126x_get_gfsk_time_on_air_numerator( pkt_p );
    uint32_t denominator = mod_p->br_in_bps;

    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

sx126x_status_t sx126x_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n )
{
    sx126x_status_t status;

    uint8_t tmp_ana_lna   = 0x00;
    uint8_t tmp_ana_mixer = 0x00;
    uint8_t tmp           = 0x00;

    // Configure for random number generation
    status = sx126x_read_register( context, SX126X_REG_ANA_LNA, &tmp_ana_lna, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }
    tmp    = tmp_ana_lna & ~( 1 << 0 );
    status = sx126x_write_register( context, SX126X_REG_ANA_LNA, &tmp, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    status = sx126x_read_register( context, SX126X_REG_ANA_MIXER, &tmp_ana_mixer, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }
    tmp    = tmp_ana_mixer & ~( 1 << 7 );
    status = sx126x_write_register( context, SX126X_REG_ANA_MIXER, &tmp, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    // Start RX continuous
    status = sx126x_set_rx_with_timeout_in_rtc_step( context, 0xFFFFFF );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    // Store values
    for( unsigned int i = 0; i < n; i++ )
    {
        status = sx126x_read_register( context, SX126X_REG_RNGBASEADDRESS, ( uint8_t* ) &numbers[i], 4 );
        if( status != SX126X_STATUS_OK )
        {
            return status;
        }
    }

    status = sx126x_set_standby( context, SX126X_STANDBY_CFG_RC );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    // Restore registers
    status = sx126x_write_register( context, SX126X_REG_ANA_LNA, &tmp_ana_lna, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }
    status = sx126x_write_register( context, SX126X_REG_ANA_MIXER, &tmp_ana_mixer, 1 );

    return status;
}

uint32_t sx126x_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz )
{
    uint32_t steps_int;
    uint32_t steps_frac;

    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    steps_int  = freq_in_hz / SX126X_PLL_STEP_SCALED;
    steps_frac = freq_in_hz - ( steps_int * SX126X_PLL_STEP_SCALED );

    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return ( steps_int << SX126X_PLL_STEP_SHIFT_AMOUNT ) +
           ( ( ( steps_frac << SX126X_PLL_STEP_SHIFT_AMOUNT ) + ( SX126X_PLL_STEP_SCALED >> 1 ) ) /
             SX126X_PLL_STEP_SCALED );
}

uint32_t sx126x_convert_timeout_in_ms_to_rtc_step( uint32_t timeout_in_ms )
{
    return ( uint32_t )( timeout_in_ms * ( SX126X_RTC_FREQ_IN_HZ / 1000 ) );
}

//
// Registers access
//

sx126x_status_t sx126x_cfg_rx_boosted( const void* context, const bool state )
{
    if( state == true )
    {
        return sx126x_write_register( context, SX126X_REG_RXGAIN, ( const uint8_t[] ){ 0x96 }, 1 );
    }
    else
    {
        return sx126x_write_register( context, SX126X_REG_RXGAIN, ( const uint8_t[] ){ 0x94 }, 1 );
    }
}

sx126x_status_t sx126x_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;
    uint8_t         buf[8] = { 0 };

    if( sync_word_len <= 8 )
    {
        memcpy( buf, sync_word, sync_word_len );
        status = sx126x_write_register( context, SX126X_REG_SYNCWORDBASEADDRESS, buf, 8 );
    }

    return status;
}

sx126x_status_t sx126x_set_lora_sync_word( const void* context, const uint8_t sync_word )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         buffer[2] = { 0x00 };

    status = sx126x_read_register( context, SX126X_REG_LR_SYNCWORD, buffer, 2 );

    if( status == SX126X_STATUS_OK )
    {
        buffer[0] = ( buffer[0] & ~0xF0 ) + ( sync_word & 0xF0 );
        buffer[1] = ( buffer[1] & ~0xF0 ) + ( ( sync_word & 0x0F ) << 4 );

        status = sx126x_write_register( context, SX126X_REG_LR_SYNCWORD, buffer, 2 );
    }

    return status;
}

sx126x_status_t sx126x_set_gfsk_crc_seed( const void* context, uint16_t seed )
{
    uint8_t s[] = { ( uint8_t )( seed >> 8 ), ( uint8_t ) seed };

    return sx126x_write_register( context, SX126X_REG_CRCSEEDBASEADDRESS, s, sizeof( s ) );
}

sx126x_status_t sx126x_set_gfsk_crc_polynomial( const void* context, const uint16_t polynomial )
{
    uint8_t poly[] = { ( uint8_t )( polynomial >> 8 ), ( uint8_t ) polynomial };

    return sx126x_write_register( context, SX126X_REG_CRCPOLYBASEADDRESS, poly, sizeof( poly ) );
}

sx126x_status_t sx126x_set_gfsk_whitening_seed( const void* context, const uint16_t seed )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         reg_value = 0;

    // The SX126X_REG_WHITSEEDBASEADDRESS @ref LSBit is used for the seed value. The 7 MSBits must not be modified.
    // Thus, we first need to read the current value and then change the LSB according to the provided seed @ref value.
    status = sx126x_read_register( context, SX126X_REG_WHITSEEDBASEADDRESS, &reg_value, 1 );
    if( status == SX126X_STATUS_OK )
    {
        reg_value = ( reg_value & 0xFE ) | ( ( uint8_t )( seed >> 8 ) & 0x01 );
        status    = sx126x_write_register( context, SX126X_REG_WHITSEEDBASEADDRESS, &reg_value, 1 );
        if( status == SX126X_STATUS_OK )
        {
            reg_value = ( uint8_t ) seed;
            status    = sx126x_write_register( context, SX126X_REG_WHITSEEDBASEADDRESS + 1, &reg_value, 1 );
        }
    }

    return status;
}

sx126x_status_t sx126x_cfg_tx_clamp( const void* context )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         reg_value = 0x00;

    status = sx126x_read_register( context, SX126X_REG_TX_CLAMP_CFG, &reg_value, 1 );

    if( status == SX126X_STATUS_OK )
    {
        reg_value |= SX126X_REG_TX_CLAMP_CFG_MASK;
        status = sx126x_write_register( context, SX126X_REG_TX_CLAMP_CFG, &reg_value, 1 );
    }

    return status;
}

sx126x_status_t sx126x_stop_rtc( const void* context )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         reg_value = 0;

    reg_value = 0;
    status    = sx126x_write_register( context, SX126X_REG_RTC_CTRL, &reg_value, 1 );

    if( status == SX126X_STATUS_OK )
    {
        status = sx126x_read_register( context, SX126X_REG_EVT_CLR, &reg_value, 1 );

        if( status == SX126X_STATUS_OK )
        {
            reg_value |= SX126X_REG_EVT_CLR_TIMEOUT_MASK;
            status = sx126x_write_register( context, SX126X_REG_EVT_CLR, &reg_value, 1 );
        }
    }

    return status;
}

sx126x_status_t sx126x_set_ocp_value( const void* context, const uint8_t ocp_in_step_of_2_5_ma )
{
    return ( sx126x_status_t ) sx126x_write_register( context, SX126X_REG_OCP, &ocp_in_step_of_2_5_ma, 1 );
}

sx126x_status_t sx126x_set_trimming_capacitor_values( const void* context, const uint8_t trimming_cap_xta,
                                                      const uint8_t trimming_cap_xtb )
{
    uint8_t trimming_capacitor_values[2] = { trimming_cap_xta, trimming_cap_xtb };

    return ( sx126x_status_t ) sx126x_write_register( context, SX126X_REG_XTATRIM, trimming_capacitor_values, 2 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static sx126x_status_t sx126x_tx_modulation_workaround( const void* context, sx126x_pkt_type_t pkt_type,
                                                        sx126x_lora_bw_t bw )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         reg_value = 0;

    status = sx126x_read_register( context, SX126X_REG_TX_MODULATION, &reg_value, 1 );

    if( status == SX126X_STATUS_OK )
    {
        if( pkt_type == SX126X_PKT_TYPE_LORA )
        {
            if( bw == SX126X_LORA_BW_500 )
            {
                reg_value &= ~( 1 << 2 );  // Bit 2 set to 0 if the LoRa BW = 500 kHz
            }
            else
            {
                reg_value |= ( 1 << 2 );  // Bit 2 set to 1 for any other LoRa BW
            }
        }
        else
        {
            reg_value |= ( 1 << 2 );  // Bit 2 set to 1 for any (G)FSK configuration
        }

        status = sx126x_write_register( context, SX126X_REG_TX_MODULATION, &reg_value, 1 );
    }
    return status;
}

static inline uint32_t sx126x_get_gfsk_crc_len_in_bytes( sx126x_gfsk_crc_types_t crc_type )
{
    switch( crc_type )
    {
    case SX126X_GFSK_CRC_OFF:
        return 0;
    case SX126X_GFSK_CRC_1_BYTE:
        return 1;
    case SX126X_GFSK_CRC_2_BYTES:
        return 2;
    case SX126X_GFSK_CRC_1_BYTE_INV:
        return 1;
    case SX126X_GFSK_CRC_2_BYTES_INV:
        return 2;
    }

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
