#include "rfal_rfst25r3911b.h";

static uint32_t st25r3911NoResponseTime_64fcs;

void rfal_rfst25r3911b::st25r3911Initialize()
{
    uint16_t vdd_mV;

    /* first, reset the st25r3911 */
    st25r3911ExecuteCommand(ST25R3911_CMD_SET_DEFAULT);
    
    /* Set Operation Control Register to default value */
    st25r3911WriteRegister(ST25R3911_REG_OP_CONTROL, 0x00);
    
    /* enable pull downs on miso line */
    st25r3911ModifyRegister(ST25R3911_REG_IO_CONF2, 0, 
            ST25R3911_REG_IO_CONF2_miso_pd1 |
            ST25R3911_REG_IO_CONF2_miso_pd2);
    
    /* after reset all interrupts are enabled. so disable them at first */
    st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_ALL);
    
    /* and clear them, just to be sure... */
    st25r3911ClearInterrupts();
    
    st25r3911OscOn();
    
    /* Measure vdd and set sup3V bit accordingly */
    vdd_mV = st25r3911MeasureVoltage(ST25R3911_REG_REGULATOR_CONTROL_mpsv_vdd);
    
    st25r3911ModifyRegister(ST25R3911_REG_IO_CONF2,
                         ST25R3911_REG_IO_CONF2_sup3V,
                         (uint8_t)((vdd_mV < 3600U)?ST25R3911_REG_IO_CONF2_sup3V:0U));
    
    /* Make sure Transmitter and Receiver are disabled */
    st25r3911TxRxOff();
    
}

void rfal_rfst25r3911b::st25r3911ExecuteCommand(uint8_t cmd)
{
  /* Setting Transaction Parameters */
  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));

  digitalWrite(cs_pin, LOW);
  
  dev_spi->transfer(cmd | ST25R3911_CMD_MODE);

  digitalWrite(cs_pin, HIGH);

  dev_spi->endTransaction();
}

void rfal_rfst25r3911b::st25r3911ExecuteCommands(const uint8_t *cmds, uint8_t length)
{
    /* Setting Transaction Parameters */
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
    digitalWrite(cs_pin, LOW);
    
    for(uint8_t i = 0; i < length; i++){
      dev_spi->transfer(cmds[i]);
    }
    
    
    digitalWrite(cs_pin, HIGH);
    dev_spi->endTransaction();

    return;
}

void rfal_rfst25r3911b::st25r3911ReadMultipleRegisters(uint8_t reg, uint8_t* value, uint8_t len)
{
  /* ST25R391X_COM_SINGLETXRX ??? */

  /* Setting Transaction Parameters */
  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  digitalWrite(cs_pin, LOW);

  /* Write Reg Address */
  dev_spi->transfer((reg | ST25R3911_READ_MODE));

  dev_spi->transfer(value,len);  
  
  /* Read the data *
  for (uint8_t i = 0; i < len; i++) {
     *(value + i) = dev_spi->transfer(0x00);
  }*/

  digitalWrite(cs_pin, HIGH);

  dev_spi->endTransaction();
}

void  rfal_rfst25r3911b::st25r3911ReadRegister(uint8_t reg, uint8_t* value)
{
  uint8_t buf[2];
  
  buf[0] = (reg | ST25R3911_READ_MODE);
  buf[1] = 0;
  
  /* Setting Transaction Parameters */
  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  digitalWrite(cs_pin, LOW);

  dev_spi->transfer(buf,2);

  digitalWrite(cs_pin, HIGH);
  dev_spi->endTransaction();

  *value = buf[1];
}

void rfal_rfst25r3911b::st25r3911WriteMultipleRegisters(uint8_t reg, const uint8_t* values, uint8_t len)
{
  
  if(len > 0U){
    
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  
    digitalWrite(cs_pin, LOW);
  
    /* Write Reg Address */
    dev_spi->transfer((reg | ST25R3911_WRITE_MODE));
    
    /* Write the data */
    for (uint8_t i = 0; i < len; i++) {
       dev_spi->transfer(values[i]);
    }
  
    digitalWrite(cs_pin, HIGH);
  
    dev_spi->endTransaction();
  }
}

void rfal_rfst25r3911b::st25r3911WriteRegister(uint8_t reg, uint8_t value)
{
  uint8_t buf[2];
  
  buf[0] = (reg | ST25R3911_WRITE_MODE);
  buf[1] = value;
  
  /* Setting Transaction Parameters */
  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  digitalWrite(cs_pin, LOW);

  dev_spi->transfer(buf,2);

  digitalWrite(cs_pin, HIGH);
  dev_spi->endTransaction();

}

void rfal_rfst25r3911b::st25r3911ReadFifo(uint8_t* buf, uint8_t length)
{
    if(length > 0U)
    {
       /* Setting Transaction Parameters */
      dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
      digitalWrite(cs_pin, LOW);

      dev_spi->transfer(( ST25R3911_FIFO_READ));
      
      for (uint8_t i = 0; i < length; i++) {
        *(buf + i) = dev_spi->transfer(0x00);
      }
      
      digitalWrite(cs_pin, HIGH);
      dev_spi->endTransaction();
    }
}

void rfal_rfst25r3911b::st25r3911WriteFifo(const uint8_t* values, uint8_t length)
{
    if (length > 0U)
    {            
      /* Setting Transaction Parameters */
      dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
      digitalWrite(cs_pin, LOW);

      dev_spi->transfer(( ST25R3911_FIFO_LOAD));
      
      for (uint8_t i = 0; i < length; i++) {
       dev_spi->transfer(values[i]);
      }
      
      digitalWrite(cs_pin, HIGH);
      dev_spi->endTransaction();
    } 
}
void rfal_rfst25r3911b::st25r3911ModifyRegister(uint8_t reg, uint8_t clr_mask, uint8_t set_mask)
{
  uint8_t tmp;

  st25r3911ReadRegister(reg, &tmp);
  
  /* mask out the bits we don't want to change */
  tmp &= ~clr_mask;
  
  /* set the new value */
  tmp |= set_mask;
  
  st25r3911WriteRegister(reg, tmp);

}

void rfal_rfst25r3911b::st25r3911OscOn()
{
  /* Check if oscillator is already turned on and stable                                                */        
    /* Use ST25R3911_REG_OP_CONTROL_en instead of ST25R3911_REG_AUX_DISPLAY_osc_ok to be on the safe side */    
    if( !st25r3911CheckReg( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_en, ST25R3911_REG_OP_CONTROL_en ) )
    {
        /* Clear any eventual previous oscillator IRQ */
        st25r3911GetInterrupt( ST25R3911_IRQ_MASK_OSC );
        
        /* enable oscillator frequency stable interrupt */
        st25r3911EnableInterrupts(ST25R3911_IRQ_MASK_OSC);
        
        /* enable oscillator and regulator output */
        st25r3911ModifyRegister(ST25R3911_REG_OP_CONTROL, 0x00, ST25R3911_REG_OP_CONTROL_en);
        
        /* wait for the oscillator interrupt */
        st25r3911WaitForInterruptsTimed(ST25R3911_IRQ_MASK_OSC, ST25R3911_OSC_STABLE_TIMEOUT);
        
        st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_OSC);
        
    }
}

bool rfal_rfst25r3911b::st25r3911CheckReg( uint8_t reg, uint8_t mask, uint8_t value )
{
    uint8_t regVal;
    
    regVal = 0;
    st25r3911ReadRegister( reg, &regVal );
    
    return ((regVal & mask) == value );
}

uint16_t rfal_rfst25r3911b::st25r3911MeasureVoltage(uint8_t mpsv)
{
    uint8_t result; 
    uint16_t mV;
    
    result = st25r3911MeasurePowerSupply( mpsv );
    
    mV = ((uint16_t)result) * 23U;
    mV += ((((uint16_t)result) * 438U) + 500U) / 1000U;
    
    return mV;
}

uint8_t rfal_rfst25r3911b::st25r3911MeasurePowerSupply( uint8_t mpsv )
{
    uint8_t result; 
   
    /* Set the source of direct command: Measure Power Supply Voltage */
    st25r3911ChangeRegisterBits( ST25R3911_REG_REGULATOR_CONTROL, ST25R3911_REG_REGULATOR_CONTROL_mask_mpsv, mpsv );
    /* Execute command: Measure Power Supply Voltage */
    st25r3911ExecuteCommandAndGetResult( ST25R3911_CMD_MEASURE_VDD, ST25R3911_REG_AD_RESULT, 10, &result);
    
     return result;
}

void rfal_rfst25r3911b::st25r3911ChangeRegisterBits(uint8_t reg, uint8_t valueMask, uint8_t value)
{
  st25r3911ModifyRegister(reg, valueMask, (valueMask & value) );
}

ReturnCode rfal_rfst25r3911b::st25r3911ExecuteCommandAndGetResult(uint8_t cmd, uint8_t resreg, uint8_t sleeptime, uint8_t* result)
{

    st25r3911EnableInterrupts(ST25R3911_IRQ_MASK_DCT);
    
    st25r3911GetInterrupt(ST25R3911_IRQ_MASK_DCT);
   
    st25r3911ExecuteCommand(cmd);
    
    st25r3911WaitForInterruptsTimed(ST25R3911_IRQ_MASK_DCT, sleeptime);
    
    st25r3911DisableInterrupts(ST25R3911_IRQ_MASK_DCT);
   
    /* read out the result if the pointer is not NULL */
    if (result != NULL)
    {
        st25r3911ReadRegister(resreg, result);
    }

    return ERR_NONE;
}

void rfal_rfst25r3911b::st25r3911TxRxOff()
{
    st25r3911ClrRegisterBits(ST25R3911_REG_OP_CONTROL, (ST25R3911_REG_OP_CONTROL_rx_en | ST25R3911_REG_OP_CONTROL_tx_en) );
}

void rfal_rfst25r3911b::st25r3911ClrRegisterBits( uint8_t reg, uint8_t clr_mask )
{
  uint8_t tmp;
  st25r3911ReadRegister(reg, &tmp);
  tmp &= ~clr_mask;     /* and_eq , ~ deconstructor*/
  st25r3911WriteRegister(reg, tmp);
}

bool rfal_rfst25r3911b::st25r3911CheckChipID( uint8_t *rev )
{
    uint8_t ID;
    
    ID = 0;    
    st25r3911ReadRegister( ST25R3911_REG_IC_IDENTITY, &ID );
    
    /* Check if IC Identity Register contains ST25R3911's IC type code */
    if( (ID & ST25R3911_REG_IC_IDENTITY_mask_ic_type) != ST25R3911_REG_IC_IDENTITY_ic_type )
    {
        return false;
    }
        
    if(rev != NULL)
    {
        *rev = (ID & ST25R3911_REG_IC_IDENTITY_mask_ic_rev);
    }
    
    return true;
}

void rfal_rfst25r3911b::st25r3911WriteTestRegister(uint8_t reg, uint8_t value)
{
  /* ST25R391X_COM_SINGLETXRX ??? */
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  
    digitalWrite(cs_pin, LOW);
  
    /* Write Reg Address */
    dev_spi->transfer(ST25R3911_CMD_TEST_ACCESS);
    dev_spi->transfer(reg | ST25R3911_WRITE_MODE);

    /* Write the data */
    dev_spi->transfer(value);
  
    digitalWrite(cs_pin, HIGH);
  
    dev_spi->endTransaction();
}

void rfal_rfst25r3911b::st25r3911SetRegisterBits( uint8_t reg, uint8_t set_mask )
{
  uint8_t tmp;

  st25r3911ReadRegister(reg, &tmp);
  tmp |= set_mask;    /* or_eq */
  st25r3911WriteRegister(reg, tmp);
}

void rfal_rfst25r3911b::st25r3911ReadTestRegister(uint8_t reg, uint8_t* value)
{
  /* Setting Transaction Parameters */
  dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  digitalWrite(cs_pin, LOW);

  dev_spi->transfer(ST25R3911_CMD_TEST_ACCESS);

 /* Write Reg Address */ 
  dev_spi->transfer(reg | ST25R3911_READ_MODE);
  
  /* Read the data */
  *value  = dev_spi->transfer(0x00);


  digitalWrite(cs_pin, HIGH);

  dev_spi->endTransaction();
}

void rfal_rfst25r3911b::st25r3911ChangeTestRegisterBits( uint8_t reg, uint8_t valueMask, uint8_t value )
{
  uint8_t    readVal;
  uint8_t    writeVal;
    
    /* Read current reg value */
    st25r3911ReadTestRegister(reg, &readVal);
    
    /* Compute new value */
    writeVal  = (readVal & ~valueMask);
    writeVal |= (value & valueMask);
    
    /* Write new reg value */
    st25r3911WriteTestRegister(reg, writeVal );
}

void rfal_rfst25r3911b::rfalFIFOStatusClear( void )
{
    gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] = RFAL_FIFO_STATUS_INVALID;
}

ReturnCode rfal_rfst25r3911b::rfalChipChangeRegBits( uint16_t reg, uint8_t valueMask, uint8_t value )
{
    st25r3911ChangeRegisterBits( (uint8_t)reg, valueMask, value );
    return ERR_NONE;
}

ReturnCode rfal_rfst25r3911b::rfalChipChangeTestRegBits( uint16_t reg, uint8_t valueMask, uint8_t value )
{
    st25r3911ChangeTestRegisterBits( (uint8_t)reg, valueMask, value );
    return ERR_NONE;
}

void rfal_rfst25r3911b::st25r3911SetNumTxBits( uint32_t nBits )
{
    st25r3911WriteRegister(ST25R3911_REG_NUM_TX_BYTES2, (uint8_t)((nBits >> 0) & 0xffU)); 
    st25r3911WriteRegister(ST25R3911_REG_NUM_TX_BYTES1, (uint8_t)((nBits >> 8) & 0xffU));    
}

void rfal_rfst25r3911b::st25r3911StartGPTimer_8fcs(uint16_t gpt_8fcs, uint8_t trigger_source)
{
    st25r3911WriteRegister(ST25R3911_REG_GPT1, (uint8_t)(gpt_8fcs >> 8));
    st25r3911WriteRegister(ST25R3911_REG_GPT2, (uint8_t)(gpt_8fcs & 0xffU));
    
    st25r3911ModifyRegister(ST25R3911_REG_GPT_CONTROL, 
            ST25R3911_REG_GPT_CONTROL_gptc_mask, 
            trigger_source);
    if (trigger_source == 0U)
    {
        st25r3911ExecuteCommand(ST25R3911_CMD_START_GP_TIMER);
    }

    return;
}

void rfal_rfst25r3911b::rfalCleanupTransceive( void )
{
    /*******************************************************************************/
    /* Transceive flags                                                            */
    /*******************************************************************************/
    
    /* Restore default settings on NFCIP1 mode, Receiving parity + CRC bits and manual Tx Parity*/
    st25r3911ClrRegisterBits( ST25R3911_REG_ISO14443A_NFC, (ST25R3911_REG_ISO14443A_NFC_no_tx_par | ST25R3911_REG_ISO14443A_NFC_no_rx_par | ST25R3911_REG_ISO14443A_NFC_nfc_f0) );
    
    /* Restore AGC enabled */
    st25r3911SetRegisterBits( ST25R3911_REG_RX_CONF2, ST25R3911_REG_RX_CONF2_agc_en );
    
    /*******************************************************************************/
    
    
    
    /*******************************************************************************/
    /* Execute Post Transceive Callback                                            */
    /*******************************************************************************/
    if( gRFAL.callbacks.postTxRx != NULL )
    {
        gRFAL.callbacks.postTxRx();
    }
    /*******************************************************************************/

}

ReturnCode rfal_rfst25r3911b::st25r3911SetNoResponseTime_64fcs(uint32_t nrt_64fcs)
{
    ReturnCode err = ERR_NONE;
    uint8_t nrt_step = 0;
    uint32_t noResponseTime_64fcs = nrt_64fcs;      /* MISRA 17.8: Use intermediate variable */

    st25r3911NoResponseTime_64fcs = noResponseTime_64fcs;
    if (noResponseTime_64fcs > (uint32_t)0xFFFFU)
    {
        nrt_step = ST25R3911_REG_GPT_CONTROL_nrt_step;
        noResponseTime_64fcs = (noResponseTime_64fcs + 63U) / 64U;
        if (noResponseTime_64fcs > (uint32_t)0xFFFFU)
        {
            noResponseTime_64fcs = 0xFFFFU;
            err = ERR_PARAM;
        }
        st25r3911NoResponseTime_64fcs = 64U * noResponseTime_64fcs;
    }

    st25r3911ModifyRegister(ST25R3911_REG_GPT_CONTROL, ST25R3911_REG_GPT_CONTROL_nrt_step, nrt_step);
    st25r3911WriteRegister(ST25R3911_REG_NO_RESPONSE_TIMER1, (uint8_t)(noResponseTime_64fcs >> 8));
    st25r3911WriteRegister(ST25R3911_REG_NO_RESPONSE_TIMER2, (uint8_t)(noResponseTime_64fcs & 0xffU));

    return err;
}
