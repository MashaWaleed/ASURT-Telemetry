/*
 *  ============================================================================
 *   [Mohamed Waleed]
 *   [Low Voltage Sub-Team]
 *
 *  File: [nrf.h]
 *
 *  Description: [Header file for the NRF24L01 + PA + LNA module for the telemetry system]
 *
 *  ============================================================================
 */

#ifndef __NRF_H__
#define __NRF_H__

/***********< Includes*************/
#include <stdint.h>
#include <stdbool.h>
#include "spi.h"
/********< End of Includes*********/

/****************< enums for SPECS*************/
typedef enum {
    NRF_DATA_RATE_250KBPS, /**< Data rate: 250 kbps */
    NRF_DATA_RATE_1MBPS,   /**< Data rate: 1 Mbps */
    NRF_DATA_RATE_2MBPS    /**< Data rate: 2 Mbps */
} NRF_DATA_RATE_t;

typedef enum {
    NRF_PWR_18dBm = 0, /**< Power: -18 dBm */
    NRF_PWR_12dBm = 1, /**< Power: -12 dBm */
    NRF_PWR_6dBm = 2,  /**< Power: -6 dBm */
    NRF_PWR_0dBm = 3   /**< Power: 0 dBm */
} NRF_PWR_t;

typedef enum {
    NRF_ADDR_WIDTH_3 = 3, /**< Address width: 3 bytes */
    NRF_ADDR_WIDTH_4 = 4, /**< Address width: 4 bytes */
    NRF_ADDR_WIDTH_5 = 5  /**< Address width: 5 bytes */
} NRF_ADDR_WIDTH_t;

typedef enum { 
    NRF_CRC_WIDTH_1 = 1, /**< CRC width: 1 byte */
    NRF_CRC_WIDTH_2 = 2  /**< CRC width: 2 bytes */
} NRF_CRC_WIDTH_t;
/****************< end of enums for SPECS*************/

/*********< Configurations Struct *******/
typedef struct
{
    NRF_PWR_t         NRF_Power;
    NRF_DATA_RATE_t   NRF_Rate;
    NRF_ADDR_WIDTH_t  NRF_Addr_Width;
    NRF_CRC_WIDTH_t   NRF_CRC_Width;
    
    uint8_t           NRF_Channel;
    uint8_t           NRF_Payload_Length;
    
    uint8_t           NRF_Re_Count;
    uint8_t           NRF_Re_Delay;
    
    uint8_t*          NRF_Address;
} NRF_Config_t;


/**********< Configurations ************/
//#define VISUAL_LED                       /**< Uncomment for visual transceiving */
//#define twoWay													1	/**< auto ack and DPL and feature */

#define NRF_CFG_CS_PORT                  GPIOB 
#define NRF_CFG_CS_PIN                   GPIO_PIN_0

#define NRF_CFG_CE_PORT                  GPIOB
#define NRF_CFG_CE_PIN                   GPIO_PIN_1

#define NRF_CFG_IRQ_PORT                 GPIOA
#define NRF_CFG_IRQ_PIN                  GPIO_PIN_8

#define NRF_CFG_SPI_HANDLE               (&hspi1)

/**********< End of Configurations ************/



/**********< Register Mapping   P.53 ************/
//R AND W
#define NRF_REG_CONFIG            0x00
#define NRF_REG_EN_AA             0x01
#define NRF_REG_EN_RXADDR         0x02
#define NRF_REG_SETUP_AW          0x03
#define NRF_REG_SETUP_RETR        0x04
#define NRF_REG_RF_CH             0x05
#define NRF_REG_RF_SETUP          0x06
#define NRF_REG_STATUS            0x07
#define NRF_REG_RX_ADDR_P0        0x0A
#define NRF_REG_RX_ADDR_P1        0x0B
#define NRF_REG_RX_ADDR_P2        0x0C
#define NRF_REG_RX_ADDR_P3        0x0D
#define NRF_REG_RX_ADDR_P4        0x0E
#define NRF_REG_RX_ADDR_P5        0x0F
#define NRF_REG_TX_ADDR           0x10
#define NRF_REG_RX_PW_P0          0x11
#define NRF_REG_RX_PW_P1          0x12
#define NRF_REG_RX_PW_P2          0x13
#define NRF_REG_RX_PW_P3          0x14
#define NRF_REG_RX_PW_P4          0x15
#define NRF_REG_RX_PW_P5          0x16
#define NRF_REG_FIFO_STATUS       0x17
#define NRF_REG_DYNPD             0x1C
#define NRF_REG_FEATURE           0x1D  

//R ONLY
#define NRF_REG_OBSERVE_TX        0x08    
#define NRF_REG_CD                0x09    

/**********< End Of Register Mapping P.53 ************/


/**********< SPI COMMANDS   P.46 ************/
#define NRF_CMD_R_REGISTER                  0x00
#define NRF_CMD_W_REGISTER                  0x20
#define NRF_CMD_R_RX_PAYLOAD                0x61
#define NRF_CMD_W_TX_PAYLOAD                0xA0
#define NRF_CMD_FLUSH_TX                    0xE1
#define NRF_CMD_FLUSH_RX                    0xE2
#define NRF_CMD_REUSE_TX_PL                 0xE3
#define NRF_CMD_ACTIVATE                    0x50
#define NRF_CMD_R_RX_PL_WID                 0x60
#define NRF_CMD_W_ACK_PAYLOAD               0xA8
#define NRF_CMD_W_TX_PAYLOAD_NOACK          0xB0
#define NRF_CMD_NOP                         0xFF    
/*******< end of SPI COMMANDS   P.46 **********/

/****************< Function Prototypes *************/

/****************< MAIN FUNCTIONS *************/
/**
 * @brief Initialize NRF in Receiver mode.
 * @param cfg Pointer to NRF configuration struct.
 */
void ECUAL_NRF_Rx_Init(NRF_Config_t * cfg);

/**
 * @brief Initialize NRF in Transmitter mode.
 * @param cfg Pointer to NRF configuration struct.
 */
void ECUAL_NRF_Tx_Init(NRF_Config_t * cfg);


/**
 * @brief Receive a single packet.
 * @param RxBuf Buffer to store received data.
 * @return Status of the reception.
 */
uint8_t ECUAL_NRF_RX_Receive_Single(uint8_t* RxBuf);

/**
 * @brief Polling-based reception of data.
 * @param RxBuf Buffer to store received data.
 * @return Status of the reception.
 */
uint8_t ECUAL_NRF_RX_Receive_POLL(uint8_t* RxBuf);

/**
 * @brief Transmit a packet.
 * @param TxBuf Buffer containing data to transmit.
 * @param size Size of the data to transmit.
 */
uint8_t ECUAL_NRF_TX_Transmit(uint8_t* TxBuf, uint8_t size);

/****************< INTERRUPTS AND FLAGS FUNCTIONS *************/
/**
 * @brief Handle transmit interrupt.
 */
void ECUAL_NRF_tx_irq(void);

/**
 * @brief Clear receive data ready interrupt flag.
 */
void ECUAL_NRF_clear_rx_dr(void);

/**
 * @brief Clear transmit data sent interrupt flag.
 */
void ECUAL_NRF_clear_tx_ds(void);

/**
 * @brief Clear maximum number of retransmits interrupt flag.
 */
void ECUAL_NRF_clear_max_rt(void);

/**
 * @brief Get current status.
 * @return Current status.
 */
uint8_t ECUAL_NRF_get_status(void);

/**
 * @brief Get FIFO status.
 * @return FIFO status.
 */
uint8_t ECUAL_NRF_get_fifo_status(void);

/****************< FIFO FUNCTIONS *************/
/**
 * @brief Read data from receive FIFO.
 * @param rx_payload Buffer to store received payload.
 * @return Status of the operation.
 */
uint8_t ECUAL_NRF_read_rx_fifo(uint8_t* rx_payload);

/**
 * @brief Write data to transmit FIFO.
 * @param tx_payload Buffer containing data to transmit.
 * @param size Size of the data to transmit.
 * @return Status of the operation.
 */
uint8_t ECUAL_NRF_write_tx_fifo(uint8_t* tx_payload, uint8_t size);

/**
 * @brief Flush receive FIFO.
 */
void ECUAL_NRF_flush_rx_fifo(void);

/**
 * @brief Flush transmit FIFO.
 */
void ECUAL_NRF_flush_tx_fifo(void);

/****************< WAKE UP ACTIONS *************/
/**
 * @brief Reset NRF module.
 */
void ECUAL_NRF_reset(void);

/**
 * @brief Power up NRF module.
 */
void ECUAL_NRF_power_up(void);

/**
 * @brief Power down NRF module.
 */
void ECUAL_NRF_power_down(void);

/**
 * @brief Check power status.
 * @return Power status.
 */
uint8_t ECUAL_NRF_chech_pwr(void);
uint8_t ECUAL_NRF_chech_rx(void);
uint8_t ECUAL_NRF_chech_tx(void);

/****************< Configurations and packet details *************/
/**
 * @brief Set NRF module to receive mode.
 */
void ECUAL_NRF_prx_mode(void);

/**
 * @brief Set NRF module to transmit mode.
 */
void ECUAL_NRF_ptx_mode(void);

/**
 * @brief Set payload width for received packets.
 * @param bytes Payload width in bytes.
 */
void ECUAL_NRF_rx_set_payload_widths(uint8_t bytes);

/**
 * @brief Set RF channel.
 * @param ch RF channel number.
 */
void ECUAL_NRF_set_rf_channel(uint8_t ch);

/**
 * @brief Set RF transmission output power.
 * @param dBm Power level in dBm.
 */
void ECUAL_NRF_set_rf_tx_output_power(NRF_PWR_t dBm);

/**
 * @brief Set RF air data rate.
 * @param bps Data rate in bits per second.
 */
void ECUAL_NRF_set_rf_air_data_rate(NRF_DATA_RATE_t bps);

/**
 * @brief Set CRC length.
 * @param bytes Length of CRC in bytes.
 */
void ECUAL_NRF_set_crc_length(NRF_CRC_WIDTH_t bytes);

/**
 * @brief Set address widths.
 * @param bytes Address width in bytes.
 */
void ECUAL_NRF_set_address_widths(NRF_ADDR_WIDTH_t bytes);

///**
// * @brief Set auto retransmit count.
// * @param cnt Number of auto retransmits.
// */
//void ECUAL_NRF_auto_retransmit_count(uint8_t cnt);

///**
// * @brief Set auto retransmit delay.
// * @param us Retransmit delay in microseconds.
// */
//void ECUAL_NRF_auto_retransmit_delay(uint8_t us);

void ECUAL_NRF_setRetries(uint8_t delay, uint8_t count);

void ECUAL_NRF_Enable_twoWay(void);
/**
 * @brief Get payload length of received packet.
 * @return Length of payload in bytes.
 */
uint8_t ECUAL_NRF_Get_PL_Length(void);
void ECUAL_NRF_Wrtie_ACK(uint8_t pipe,uint8_t* ack_payload,uint8_t size);
void ECUAL_NRF_ACTIVATE_cmd(void);
void ECUAL_NRF_Distable_ACK(void);

/****************< end of Function Prototypes *************/




#endif /* __NRF_H__ */
