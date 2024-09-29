/*
 *  ============================================================================
 *   [Mohamed Waleed]
 *   [Low Voltage Sub-Team]
 *
 *  File: [nrf.c]
 *
 *  Description: [Source file for the NRF24L01 + PA + LNA module for the telemetry system]
 *
 *  ============================================================================
 */


#include "nrf.h"

/***************************************< Util Functions******************************************/
static void PIN_SET_CS()
{
    HAL_GPIO_WritePin(NRF_CFG_CS_PORT, NRF_CFG_CS_PIN, GPIO_PIN_SET);
}

static void PIN_RESET_CS()
{
    HAL_GPIO_WritePin(NRF_CFG_CS_PORT, NRF_CFG_CS_PIN, GPIO_PIN_RESET);
}

static void PIN_SET_CE()
{
    HAL_GPIO_WritePin(NRF_CFG_CE_PORT, NRF_CFG_CE_PIN, GPIO_PIN_SET);
}

static void PIN_RESET_CE()
{
    HAL_GPIO_WritePin(NRF_CFG_CE_PORT, NRF_CFG_CE_PIN, GPIO_PIN_RESET);
}

static uint8_t REG_READ(uint8_t reg)
{
    uint8_t command = NRF_CMD_R_REGISTER | reg;
    uint8_t status;
    uint8_t read_val;

    PIN_RESET_CS();
    HAL_SPI_TransmitReceive(NRF_CFG_SPI_HANDLE, &command, &status, 1, 100);
    HAL_SPI_Receive(NRF_CFG_SPI_HANDLE, &read_val, 1, 100);
    PIN_SET_CS();

    return read_val;
}

static uint8_t REG_WRITE(uint8_t reg, uint8_t value)
{
    uint8_t command = NRF_CMD_W_REGISTER | reg;
    uint8_t status;
    uint8_t write_val = value;

    PIN_RESET_CS();
    HAL_SPI_TransmitReceive(NRF_CFG_SPI_HANDLE, &command, &status, 1, 100);
    HAL_SPI_Transmit(NRF_CFG_SPI_HANDLE, &write_val, 1, 100);
    PIN_SET_CS();

    return write_val;
}

static void REG_WRITE_V(uint8_t reg, uint8_t size, uint8_t* val)
{
		uint8_t command = NRF_CMD_W_REGISTER | reg;
    uint8_t status;
	
    PIN_RESET_CS();
    HAL_SPI_TransmitReceive(NRF_CFG_SPI_HANDLE, &command, &status, 1, 100);
    HAL_SPI_Transmit(NRF_CFG_SPI_HANDLE, val, size, 100);
    PIN_SET_CS();

}


void ECUAL_NRF_power_up(void)
{
    uint8_t new_config = REG_READ(NRF_REG_CONFIG);
    new_config |= 1 << 1;
		new_config |= 1 << 6;   // reset error checking, rx dr

    REG_WRITE(NRF_REG_CONFIG, new_config);
}

void ECUAL_NRF_power_down(void)
{
    uint8_t new_config = REG_READ(NRF_REG_CONFIG);
    new_config &= 0xFD;

    REG_WRITE(NRF_REG_CONFIG, new_config);
}

uint8_t ECUAL_NRF_chech_pwr(void)
{
	uint8_t config = REG_READ(NRF_REG_CONFIG);
	if(config & 2)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t ECUAL_NRF_chech_rx(void)
{
	uint8_t config = REG_READ(NRF_REG_CONFIG);
	if((config & 0x41) == 0x41)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t ECUAL_NRF_chech_tx(void)
{
	uint8_t config = REG_READ(NRF_REG_CONFIG);
	if((config & 0x41) == 0x40)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


/***************************************<End of Util Functions******************************************/



/***************************************< Main Functions ******************************************/

void ECUAL_NRF_Rx_Init(NRF_Config_t * cfg)
{
	// THE STARTING RITUAL :)
	
    ECUAL_NRF_reset();	
    ECUAL_NRF_prx_mode();
    ECUAL_NRF_power_up();
		
		ECUAL_NRF_set_address_widths(cfg->NRF_Addr_Width);
		REG_WRITE_V(NRF_REG_RX_ADDR_P1,5,cfg->NRF_Address); //pipe 1
		ECUAL_NRF_rx_set_payload_widths(cfg->NRF_Payload_Length); //PIPE 1
		REG_WRITE(NRF_REG_EN_RXADDR, 0x3);	//both pipe 1 and  0
		ECUAL_NRF_Distable_ACK();
	
		#if twoWay
		ECUAL_NRF_Enable_twoWay();
		#endif
	
    ECUAL_NRF_set_rf_channel(cfg->NRF_Channel);
    ECUAL_NRF_set_rf_air_data_rate(cfg->NRF_Rate);		
    ECUAL_NRF_set_rf_tx_output_power(cfg->NRF_Power);
    ECUAL_NRF_set_crc_length(cfg->NRF_CRC_Width);
		ECUAL_NRF_setRetries(cfg->NRF_Re_Delay,cfg->NRF_Re_Count);
		
    PIN_SET_CE();
		
}



void ECUAL_NRF_Tx_Init(NRF_Config_t* cfg)
{
    ECUAL_NRF_reset();
    ECUAL_NRF_ptx_mode();
    ECUAL_NRF_power_up();
	
		REG_WRITE_V(NRF_REG_TX_ADDR,5,cfg->NRF_Address);
		REG_WRITE_V(NRF_REG_RX_ADDR_P0,5,cfg->NRF_Address);
		REG_WRITE(NRF_REG_RX_PW_P0, 1); // ack
		ECUAL_NRF_Distable_ACK();	
	
		#if twoWay 
		ECUAL_NRF_Enable_twoWay();
	  #endif
	
    ECUAL_NRF_set_rf_channel(cfg->NRF_Channel);
    ECUAL_NRF_set_rf_air_data_rate(cfg->NRF_Rate);
    ECUAL_NRF_set_rf_tx_output_power(cfg->NRF_Power);
    ECUAL_NRF_set_crc_length(cfg->NRF_CRC_Width);
    ECUAL_NRF_set_address_widths(cfg->NRF_Addr_Width);
    ECUAL_NRF_setRetries(cfg->NRF_Re_Delay,cfg->NRF_Re_Count);
		
		
    PIN_SET_CE();
}

uint8_t ECUAL_NRF_RX_Receive_Single(uint8_t* RxBuf)
{
		uint8_t status, size;
    status = ECUAL_NRF_get_status();

    if((status & 0x40) == 0) // no data was received
    {
        return 0;
    }
    size = ECUAL_NRF_read_rx_fifo(RxBuf);
    ECUAL_NRF_clear_rx_dr();

    #ifdef VISUAL_LED
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    #endif
		return size;
}

uint8_t ECUAL_NRF_RX_Receive_POLL(uint8_t* RxBuf)
{
    uint8_t status, size;
    status = ECUAL_NRF_get_status();

    while((status & 0x40) == 0)
    {
        status = ECUAL_NRF_get_status();
    }
    size = ECUAL_NRF_read_rx_fifo(RxBuf);
    ECUAL_NRF_clear_rx_dr();

    #ifdef VISUAL_LED
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    #endif
		return size;
}

uint8_t ECUAL_NRF_TX_Transmit(uint8_t* TxBuf, uint8_t size)
{
		uint8_t status;
    ECUAL_NRF_write_tx_fifo(TxBuf, size);
		status = ECUAL_NRF_get_status();
		if((status & 0x10)) // MAX RT POTENTIALLY STUCK
		{
			ECUAL_NRF_flush_tx_fifo();
			ECUAL_NRF_clear_max_rt();
		}
		if((status & 0x20) == 0)
		{
			return 0;
		}
		return 1;
}

/***************************************< End of Main Functions ******************************************/




/***************************************< FIFOS ******************************************/
uint8_t ECUAL_NRF_read_rx_fifo(uint8_t* rx_payload)
{
    uint8_t command = NRF_CMD_R_RX_PAYLOAD;
    uint8_t status, size;
	
		#if twoWay
		size = ECUAL_NRF_Get_PL_Length();
		#else
		size = 10;
		#endif

    PIN_RESET_CS();
    HAL_SPI_TransmitReceive(NRF_CFG_SPI_HANDLE, &command, &status, 1, 100);
    HAL_SPI_Receive(NRF_CFG_SPI_HANDLE, rx_payload, size, 100);
    PIN_SET_CS();

    return size;
}

uint8_t ECUAL_NRF_write_tx_fifo(uint8_t* tx_payload, uint8_t size)
{
    uint8_t command = NRF_CMD_W_TX_PAYLOAD;
    uint8_t status;

    PIN_RESET_CS();
    HAL_SPI_TransmitReceive(NRF_CFG_SPI_HANDLE, &command, &status, 1, 100);
    HAL_SPI_Transmit(NRF_CFG_SPI_HANDLE, tx_payload, size, 100);
    PIN_SET_CS(); 

    return status;
}

void ECUAL_NRF_flush_rx_fifo(void)
{
    uint8_t command = NRF_CMD_FLUSH_RX;
    uint8_t status;

    PIN_RESET_CS();
    HAL_SPI_TransmitReceive(NRF_CFG_SPI_HANDLE, &command, &status, 1, 100);
    PIN_SET_CS();
}

void ECUAL_NRF_flush_tx_fifo(void)
{
    uint8_t command = NRF_CMD_FLUSH_TX;
    uint8_t status;

    PIN_RESET_CS();
    HAL_SPI_TransmitReceive(NRF_CFG_SPI_HANDLE, &command, &status, 1, 100);
    PIN_SET_CS();
}

uint8_t ECUAL_NRF_get_fifo_status(void)
{
    return REG_READ(NRF_REG_FIFO_STATUS);
}
/***************************************< END OF FIFOS ******************************************/





/***************************************< Interrupts and Flags ******************************************/
uint8_t ECUAL_NRF_get_status(void)
{
    uint8_t command = NRF_CMD_NOP;
    uint8_t status;

    PIN_RESET_CS();
    HAL_SPI_TransmitReceive(NRF_CFG_SPI_HANDLE, &command, &status, 1, 100);
    PIN_SET_CS(); 

    return status;
}

uint8_t ECUAL_NRF_Get_PL_Length(void)
{
		uint8_t command = NRF_CMD_R_RX_PL_WID;
    uint8_t status, size;

    PIN_RESET_CS();
    HAL_SPI_TransmitReceive(NRF_CFG_SPI_HANDLE, &command, &status, 1, 100);
    HAL_SPI_Receive(NRF_CFG_SPI_HANDLE, &size, 1, 100);
    PIN_SET_CS(); 

    return size;
}

void ECUAL_NRF_clear_rx_dr(void)
{
    uint8_t new_status = ECUAL_NRF_get_status();
    new_status |= 0x40;
    PIN_RESET_CE();
    REG_WRITE(NRF_REG_STATUS, new_status);
    PIN_SET_CE();
}

void ECUAL_NRF_clear_tx_ds(void)
{
    uint8_t new_status = ECUAL_NRF_get_status();
    new_status |= 0x20;

    REG_WRITE(NRF_REG_STATUS, new_status);     
}

void ECUAL_NRF_clear_max_rt(void)
{
    uint8_t new_status = ECUAL_NRF_get_status();
    new_status |= 0x10;

    REG_WRITE(NRF_REG_STATUS, new_status); 
}

void ECUAL_NRF_tx_irq(void)
{
    uint8_t tx_ds = ECUAL_NRF_get_status();
		uint8_t max_rt = tx_ds & 0x10;
    tx_ds &= 0x20;

    if(tx_ds)
    {   
        // TX_DS
        //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        ECUAL_NRF_clear_tx_ds();
				//ECUAL_NRF_flush_tx_fifo();
    }

    else if(max_rt)
    {
        // MAX_RT
        //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        ECUAL_NRF_clear_max_rt();
        ECUAL_NRF_flush_tx_fifo();
    }
}
/***********************************< End of Interrupts and Flags *************************************/


/*********************************<Configurations and Packet Layout************************************/

void ECUAL_NRF_prx_mode(void)
{
    uint8_t new_config = REG_READ(NRF_REG_CONFIG);
    new_config |= 1 << 0;

    REG_WRITE(NRF_REG_CONFIG, new_config);
}

void ECUAL_NRF_ptx_mode(void)
{
    uint8_t new_config = REG_READ(NRF_REG_CONFIG);
    new_config &= 0xFE;

    REG_WRITE(NRF_REG_CONFIG, new_config);
}


void ECUAL_NRF_set_crc_length(NRF_CRC_WIDTH_t bytes)
{
    uint8_t new_config = REG_READ(NRF_REG_CONFIG);
    
    switch(bytes)
    {
        // CRCO bit in CONFIG resiger set 0
        case 1:
            new_config &= 0xFB;
            break;
        // CRCO bit in CONFIG resiger set 1
        case 2:
            new_config |= 1 << 2;
            break;
    }

    REG_WRITE(NRF_REG_CONFIG, new_config);
}


void ECUAL_NRF_rx_set_payload_widths(uint8_t bytes)
{
    REG_WRITE(NRF_REG_RX_PW_P1, bytes);
		//REG_WRITE(NRF_REG_RX_PW_P0, bytes);
}

void ECUAL_NRF_set_address_widths(NRF_ADDR_WIDTH_t bytes)
{
    REG_WRITE(NRF_REG_SETUP_AW, bytes - 2);
}

//void ECUAL_NRF_auto_retransmit_count(uint8_t cnt)
//{
//    uint8_t new_setup_retr = REG_READ(NRF_REG_SETUP_RETR);
//    
//    // Reset ARC register 0
//    new_setup_retr |= 0xF0;
//    new_setup_retr |= cnt;
//    REG_WRITE(NRF_REG_SETUP_RETR, new_setup_retr);
//}

//void ECUAL_NRF_auto_retransmit_delay(uint8_t us)
//{
//    uint8_t new_setup_retr = REG_READ(NRF_REG_SETUP_RETR);

//    // Reset ARD register 0
//    new_setup_retr |= 0x0F;
//    new_setup_retr |= ((us / 250) - 1) << 4;
//    REG_WRITE(NRF_REG_SETUP_RETR, new_setup_retr);
//}

void ECUAL_NRF_setRetries(uint8_t delay, uint8_t count)
{
	REG_WRITE(NRF_REG_SETUP_RETR,(delay&0xf)<<4 | (count&0xf)<<0);
}

void ECUAL_NRF_set_rf_channel(uint8_t ch)
{
    REG_WRITE(NRF_REG_RF_CH, ch);
}

void ECUAL_NRF_set_rf_tx_output_power(NRF_PWR_t dBm)
{
    uint8_t new_rf_setup = REG_READ(NRF_REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (dBm << 1);

    REG_WRITE(NRF_REG_RF_SETUP, new_rf_setup);
}

void ECUAL_NRF_set_rf_air_data_rate(NRF_DATA_RATE_t bps)
{
    uint8_t new_rf_setup = REG_READ(NRF_REG_RF_SETUP) & 0xD7;
    
    switch(bps)
    {
        case NRF_DATA_RATE_1MBPS: 
            new_rf_setup &= ~(1<<3);
            new_rf_setup &= ~(1<<5);
            break;
        case NRF_DATA_RATE_2MBPS: 
            new_rf_setup |= 1 << 3;
            new_rf_setup &= ~(1 << 5);
            break;
        case NRF_DATA_RATE_250KBPS:
            new_rf_setup |= 1 << 5;
            new_rf_setup &= ~(1 << 3);
            break;
    }
    REG_WRITE(NRF_REG_RF_SETUP, new_rf_setup);
}

void ECUAL_NRF_Enable_twoWay(void)
{
		ECUAL_NRF_ACTIVATE_cmd();
	
		// ENABLING AUTO ACK
		REG_WRITE(NRF_REG_EN_AA, 0x3F);
		
		// ENABLING DPL AND ACK PAYLOAD FEATURES
		uint8_t feature = REG_READ(NRF_REG_FEATURE);
		feature |= ((1<<2) | (1<<1));
		REG_WRITE(NRF_REG_FEATURE, feature);
	
		// ENABLLNG DPL ON ALL PIPES
		uint8_t dyn = REG_READ(NRF_REG_DYNPD);
		dyn |= ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5));
		REG_WRITE(NRF_REG_DYNPD,dyn);
		
}

/*********************************<End of Packet Layout and configuration************************************/


void ECUAL_NRF_Wrtie_ACK(uint8_t pipe, uint8_t* ack_payload,uint8_t size){
		uint8_t cmd = NRF_CMD_W_ACK_PAYLOAD | (pipe & 0x7); // pipe 1
		uint8_t status;
	
		PIN_RESET_CS();
    HAL_SPI_TransmitReceive(NRF_CFG_SPI_HANDLE, &cmd, &status, 1, 100);
    HAL_SPI_Transmit(NRF_CFG_SPI_HANDLE, ack_payload, size, 100);
    PIN_SET_CS();
}

void ECUAL_NRF_ACTIVATE_cmd(void)
{
	uint8_t cmdRxBuf[2];
	//Read data from Rx payload buffer
	PIN_RESET_CS();
	cmdRxBuf[0] = NRF_CMD_ACTIVATE;
	cmdRxBuf[1] = 0x73;
	HAL_SPI_Transmit(NRF_CFG_SPI_HANDLE, cmdRxBuf, 2, 100);
	PIN_SET_CS();
}

void ECUAL_NRF_Distable_ACK(void)
{
	REG_WRITE(NRF_REG_EN_AA, 0x00);
}


/*********************************<   Extra    **********************/

void ECUAL_NRF_reset(void)
{
    PIN_SET_CS();
    PIN_RESET_CE();

    REG_WRITE(NRF_REG_CONFIG, 0x08);
    REG_WRITE(NRF_REG_EN_AA, 0x3F);
    REG_WRITE(NRF_REG_EN_RXADDR, 0x03);
    REG_WRITE(NRF_REG_SETUP_AW, 0x03);
    REG_WRITE(NRF_REG_SETUP_RETR, 0x03);
    REG_WRITE(NRF_REG_RF_CH, 0x02);
    REG_WRITE(NRF_REG_RF_SETUP, 0x0E);
    REG_WRITE(NRF_REG_STATUS, 0x7E);      //interrupt Masks are cleared this way :)
    REG_WRITE(NRF_REG_RX_PW_P0, 0x00);
    REG_WRITE(NRF_REG_RX_PW_P0, 0x00);
    REG_WRITE(NRF_REG_RX_PW_P1, 0x00);
    REG_WRITE(NRF_REG_RX_PW_P2, 0x00);
    REG_WRITE(NRF_REG_RX_PW_P3, 0x00);
    REG_WRITE(NRF_REG_RX_PW_P4, 0x00);
    REG_WRITE(NRF_REG_RX_PW_P5, 0x00);
    REG_WRITE(NRF_REG_FIFO_STATUS, 0x11); //Might want to have a look at non-interrupt flags
    REG_WRITE(NRF_REG_DYNPD, 0x00);       
    REG_WRITE(NRF_REG_FEATURE, 0x00);

    ECUAL_NRF_flush_rx_fifo();
    ECUAL_NRF_flush_tx_fifo();
}