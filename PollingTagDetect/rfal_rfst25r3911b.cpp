#include "rfal_rfst25r3911b.h"
#define LED5_PIN D4


/* FUNCTION */
rfal_rfst25r3911b::rfal_rfst25r3911b(SPIClass *spi,int cs_pin, int int_pin, uint32_t spi_speed): dev_spi(spi), cs_pin(cs_pin), int_pin(int_pin), spi_speed(spi_speed)
{
}

ReturnCode rfal_rfst25r3911b::rfalInitialize()
{
    
    pinMode(cs_pin,OUTPUT);
      
    st25r3911InitInterrupts();
    /* Initialize chip */
    st25r3911Initialize();
    
    /* Check expected chip: ST25R3911 */
    if( !st25r3911CheckChipID( NULL ) )
    {
        return ERR_HW_MISMATCH;
    }
    
    
    /* Disable any previous observation mode */
    st25r3911WriteTestRegister(0x01U, 0x00U);                 /*!< Disable ST25R3911 Observation mode      */      
    
    /*******************************************************************************/    
    /* Apply RF Chip generic initialization */
    rfalSetAnalogConfig( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_INIT) );
    
    /*******************************************************************************/
    /* Set FIFO Water Levels to be used */
    st25r3911ChangeRegisterBits( ST25R3911_REG_IO_CONF1, (ST25R3911_REG_IO_CONF1_fifo_lt | ST25R3911_REG_IO_CONF1_fifo_lr), (ST25R3911_REG_IO_CONF1_fifo_lt_32bytes | ST25R3911_REG_IO_CONF1_fifo_lr_64bytes) );
   
    /* Always have CRC in FIFO upon reception  */
    st25r3911SetRegisterBits( ST25R3911_REG_AUX, ST25R3911_REG_AUX_crc_2_fifo );
   
    /* Enable External Field Detector */
    st25r3911SetRegisterBits( ST25R3911_REG_AUX, ST25R3911_REG_AUX_en_fd );
    
    /* Clear FIFO status local copy */
    rfalFIFOStatusClear();
    /*******************************************************************************/
    gRFAL.state              = RFAL_STATE_INIT;
    gRFAL.mode               = RFAL_MODE_NONE;
    gRFAL.field              = false;
    
    /* Set RFAL default configs */
    gRFAL.conf.obsvModeTx    = RFAL_OBSMODE_DISABLE;
    gRFAL.conf.obsvModeRx    = RFAL_OBSMODE_DISABLE;
    gRFAL.conf.eHandling     = RFAL_ERRORHANDLING_NONE;
    
    /* Transceive set to IDLE */
    gRFAL.TxRx.lastState     = RFAL_TXRX_STATE_IDLE;
    gRFAL.TxRx.state         = RFAL_TXRX_STATE_IDLE;
    
    /* Disable all timings */
    gRFAL.timings.FDTListen  = RFAL_TIMING_NONE;
    gRFAL.timings.FDTPoll    = RFAL_TIMING_NONE;
    gRFAL.timings.GT         = RFAL_TIMING_NONE;
    
    gRFAL.tmr.GT             = RFAL_TIMING_NONE;
    
    gRFAL.callbacks.preTxRx  = NULL;
    gRFAL.callbacks.postTxRx = NULL;

    /* Initialize NFC-V Data *
    if(RFAL_FEATURE_NFCV)    
       gRFAL.nfcvData.ignoreBits = 0;

    
    if(RFAL_FEATURE_LISTEN_MODE);    
    /* Initialize Listen Mode *
    gRFAL.Lm.state           = RFAL_LM_STATE_NOT_INIT;
    gRFAL.Lm.brDetected      = RFAL_BR_KEEP;
#endif /* RFAL_FEATURE_LISTEN_MODE *

#if RFAL_FEATURE_WAKEUP_MODE
    /* Initialize Wake-Up Mode *
    gRFAL.wum.state = RFAL_WUM_STATE_NOT_INIT;
#endif /* RFAL_FEATURE_WAKEUP_MODE */
    
    
    /*******************************************************************************/    
    /* Perform Automatic Calibration (if configured to do so).                     *
     * Registers set by rfalSetAnalogConfig will tell rfalCalibrate what to perform*/
    
    rfalCalibrate();

    return ERR_NONE;
}

ReturnCode rfal_rfst25r3911b::rfalCalibrate()
{
  uint16_t resValue;
    
    /* Check if RFAL is not initialized */
    if( gRFAL.state == RFAL_STATE_IDLE )
    {
        return ERR_WRONG_STATE;
    }

    /*******************************************************************************/
    /* Perform ST25R3911 regulators and antenna calibration                        */
    /*******************************************************************************/
    
    /* Automatic regulator adjustment only performed if not set manually on Analog Configs */
    if( st25r3911CheckReg( ST25R3911_REG_REGULATOR_CONTROL, ST25R3911_REG_REGULATOR_CONTROL_reg_s, 0x00 ) )       
    {
        /* Adjust the regulators so that Antenna Calibrate has better Regulator values */
        rfalAdjustRegulators( &resValue );
    }
    
    /* Automatic Antenna calibration only performed if not set manually on Analog Configs */
    if( st25r3911CheckReg( ST25R3911_REG_ANT_CAL_CONTROL, ST25R3911_REG_ANT_CAL_CONTROL_trim_s, 0x00 ) )
    {
        st25r3911CalibrateAntenna( (uint8_t*) &resValue );
      
        /*******************************************************************************/
        /* REMARK: Silicon workaround ST25R3911 Errata #1.5                            */
        /* Always run the command Calibrate Antenna twice                              *
        st25r3911CalibrateAntenna( (uint8_t*) &resValue );                
        /*******************************************************************************/
        
    }
    else
    {
        /* If no antenna calibration is performed there is no need to perform second regulator adjustment again */
        return ERR_NONE; 
    }
   
    if( st25r3911CheckReg( ST25R3911_REG_REGULATOR_CONTROL, ST25R3911_REG_REGULATOR_CONTROL_reg_s, 0x00 ) )
    {
        /* Adjust the regulators again with the Antenna calibrated */
        rfalAdjustRegulators( &resValue );
    }
    
    return ERR_NONE;
}

ReturnCode rfal_rfst25r3911b::rfalAdjustRegulators( uint16_t* result_mV)
{
    uint8_t result;
    uint8_t io_conf2;
    ReturnCode err = ERR_NONE;

    /* Reset logic and set regulated voltages to be defined by result of Adjust Regulators command */
    st25r3911SetRegisterBits( ST25R3911_REG_REGULATOR_CONTROL, ST25R3911_REG_REGULATOR_CONTROL_reg_s );
    st25r3911ClrRegisterBits( ST25R3911_REG_REGULATOR_CONTROL, ST25R3911_REG_REGULATOR_CONTROL_reg_s );

    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_ADJUST_REGULATORS,
                                    ST25R3911_REG_REGULATOR_RESULT,
                                    5,
                                    &result);
  
    st25r3911ReadRegister(ST25R3911_REG_IO_CONF2, &io_conf2);

    result >>= ST25R3911_REG_REGULATOR_RESULT_shift_reg;
    result -= 5U;
    if (result_mV != NULL)
    {
        if((io_conf2 & ST25R3911_REG_IO_CONF2_sup3V) != 0U)
        {
            *result_mV = 2400;
            *result_mV += (uint16_t)result * 100U;
        }
        else
        {
            *result_mV = 3900;
            *result_mV += (uint16_t)result * 120U;
        }
    }
    return err;
}


void rfal_rfst25r3911b::st25r3911CalibrateAntenna(uint8_t* result)
{
    st25r3911ExecuteCommandAndGetResult(ST25R3911_CMD_CALIBRATE_ANTENNA,
                                    ST25R3911_REG_ANT_CAL_RESULT,
                                    10,
                                    result);
}

void rfal_rfst25r3911b::CheckInterrupts() {
  st25r3911Isr();
}

void rfal_rfst25r3911b::rfalWorker() {
    
    switch( gRFAL.state )
    {
        case RFAL_STATE_TXRX:
            rfalRunTransceiveWorker();
            break;

        case RFAL_STATE_LM:
            rfalRunListenModeWorker();
            break;
            
        case RFAL_STATE_WUM:
            rfalRunWakeUpModeWorker();
            break;
             
        /* Nothing to be done */
        default:            
            /* MISRA 16.4: no empty default statement (a comment being enough) */
            break;
    }
    
}

ReturnCode rfal_rfst25r3911b::rfalRunTransceiveWorker()
{
    if( gRFAL.state == RFAL_STATE_TXRX )
    {     
        /* Run Tx or Rx state machines */
        if( rfalIsTransceiveInTx() )
        {
            rfalTransceiveTx();
            return rfalGetTransceiveStatus();
        }
        
        if( rfalIsTransceiveInRx() )
        {
            rfalTransceiveRx();
            return rfalGetTransceiveStatus();
        }
    }    
    return ERR_WRONG_STATE;
}

bool rfal_rfst25r3911b::rfalIsTransceiveInTx()
{
    return ( (gRFAL.TxRx.state >= RFAL_TXRX_STATE_TX_IDLE) && (gRFAL.TxRx.state < RFAL_TXRX_STATE_RX_IDLE) );
}

bool rfal_rfst25r3911b::rfalIsTransceiveInRx( void )
{
    return (gRFAL.TxRx.state >= RFAL_TXRX_STATE_RX_IDLE);
}

ReturnCode rfal_rfst25r3911b::rfalGetTransceiveStatus()
{
    return ((gRFAL.TxRx.state == RFAL_TXRX_STATE_IDLE) ? gRFAL.TxRx.status : ERR_BUSY);
}

bool rfal_rfst25r3911b::rfalIsGTExpired( void )
{
    if( gRFAL.tmr.GT != RFAL_TIMING_NONE )
    {
        uint32_t uDiff = (gRFAL.tmr.GT - millis());
        int32_t sDiff = uDiff;
        if( sDiff > 0)
        {
            return false;
        }
    }    
    return true;
}

void rfal_rfst25r3911b::rfalTransceiveTx( void )
{
    volatile uint32_t irqs;
    uint16_t          tmp;
    ReturnCode        ret;
    
    /* Supress warning in case NFC-V feature is disabled */
    ret = ERR_NONE;
    
    
    irqs = ST25R3911_IRQ_MASK_NONE;
    
    if( gRFAL.TxRx.state != gRFAL.TxRx.lastState )
    {        
        /* rfalLogD( "RFAL: lastSt: %d curSt: %d \r\n", gRFAL.TxRx.lastState, gRFAL.TxRx.state ); */
        gRFAL.TxRx.lastState = gRFAL.TxRx.state;
    }
    
    switch( gRFAL.TxRx.state )
    {
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_IDLE:
            
            /* Nothing to do */
            
            gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_WAIT_GT ;
            /* fall through */
            
            
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_WAIT_GT:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            if( !rfalIsGTExpired() )
            {
                break;
            }
            
            gRFAL.tmr.GT = RFAL_TIMING_NONE;
            
            gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_WAIT_FDT;
            /* fall through */
            
            
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_WAIT_FDT:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            /* Only in Passive communications GPT is used to measure FDT Poll */
            if( rfalIsModePassiveComm( gRFAL.mode ) )
            {
                if( st25r3911IsGPTRunning() )
                {                
                   break;
                }
            }
            
            gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_TRANSMIT;
            /* fall through */
            
        
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_TRANSMIT:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            /* Clear FIFO, Clear and Enable the Interrupts */
            rfalPrepareTransceive( );

            /* Calculate when Water Level Interrupt will be triggered */
            gRFAL.fifo.expWL = (uint16_t)( st25r3911CheckReg( ST25R3911_REG_IO_CONF1, ST25R3911_REG_IO_CONF1_fifo_lt, ST25R3911_REG_IO_CONF1_fifo_lt_16bytes) ? RFAL_FIFO_OUT_LT_16 : RFAL_FIFO_OUT_LT_32 );
            
        #if RFAL_FEATURE_NFCV
            /*******************************************************************************/
            /* In NFC-V streaming mode, the FIFO needs to be loaded with the coded bits    */
            if( (RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode) )
            {
            #if 0
                /* Debugging code: output the payload bits by writing into the FIFO and subsequent clearing */
                st25r3911WriteFifo(gRFAL.TxRx.ctx.txBuf, rfalConvBitsToBytes(gRFAL.TxRx.ctx.txBufLen));
                st25r3911ExecuteCommand( ST25R3911_CMD_CLEAR_FIFO );
            #endif
                /* Calculate the bytes needed to be Written into FIFO (a incomplete byte will be added as 1byte) */
                gRFAL.nfcvData.nfcvOffset = 0;
                ret = iso15693VCDCode(gRFAL.TxRx.ctx.txBuf, rfalConvBitsToBytes(gRFAL.TxRx.ctx.txBufLen), (((gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL) != 0U)?false:true),(((gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_NFCV_FLAG_MANUAL) != 0U)?false:true), (RFAL_MODE_POLL_PICOPASS == gRFAL.mode),
                          &gRFAL.fifo.bytesTotal, &gRFAL.nfcvData.nfcvOffset, gRFAL.nfcvData.codingBuffer, MIN( (uint16_t)ST25R3911_FIFO_DEPTH, (uint16_t)sizeof(gRFAL.nfcvData.codingBuffer) ), &gRFAL.fifo.bytesWritten);

                if( (ret != ERR_NONE) && (ret != ERR_AGAIN) )
                {
                    gRFAL.TxRx.status = ret;
                    gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
                    break;
                }
                /* Set the number of full bytes and bits to be transmitted */
                st25r3911SetNumTxBits( rfalConvBytesToBits(gRFAL.fifo.bytesTotal) );

                /* Load FIFO with coded bytes */
                /* TODO: check bytesWritten does not exceed 255 */
                st25r3911WriteFifo( gRFAL.nfcvData.codingBuffer, (uint8_t)gRFAL.fifo.bytesWritten );

            }
            /*******************************************************************************/
            else
        #endif /* RFAL_FEATURE_NFCV */
            {
                /* Calculate the bytes needed to be Written into FIFO (a incomplete byte will be added as 1byte) */
                gRFAL.fifo.bytesTotal = (uint16_t)rfalCalcNumBytes(gRFAL.TxRx.ctx.txBufLen);
                
                /* Set the number of full bytes and bits to be transmitted */
                st25r3911SetNumTxBits( gRFAL.TxRx.ctx.txBufLen );
                
                /* Load FIFO with total length or FIFO's maximum */
                gRFAL.fifo.bytesWritten = MIN( gRFAL.fifo.bytesTotal, ST25R3911_FIFO_DEPTH );
                st25r3911WriteFifo( gRFAL.TxRx.ctx.txBuf, (uint8_t)gRFAL.fifo.bytesWritten );
            }
        
            /*Check if Observation Mode is enabled and set it on ST25R391x */
            /*rfalCheckEnableObsModeTx(); */
            if(gRFAL.conf.obsvModeTx != 0U){ 
                st25r3911WriteTestRegister(0x01U, gRFAL.conf.obsvModeTx);  /*!< Enable Observation mode 0x0A CSI: Digital TX modulation signal CSO: none                         */
            }         /*!< Checks if the observation mode is enabled, and applies on ST25R3911 */
            
            /*******************************************************************************/
            /* Trigger/Start transmission                                                  */
            if( (gRFAL.TxRx.ctx.flags & (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL) != 0U )
            {
                st25r3911ExecuteCommand( ST25R3911_CMD_TRANSMIT_WITHOUT_CRC );
            }
            else
            {
                st25r3911ExecuteCommand( ST25R3911_CMD_TRANSMIT_WITH_CRC );
            }
             
            /* Check if a WL level is expected or TXE should come */
            gRFAL.TxRx.state = (( gRFAL.fifo.bytesWritten < gRFAL.fifo.bytesTotal ) ? RFAL_TXRX_STATE_TX_WAIT_WL : RFAL_TXRX_STATE_TX_WAIT_TXE);
            break;

        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_WAIT_WL:
            
            irqs = st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_FWL | ST25R3911_IRQ_MASK_TXE) );            
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
               break;  /* No interrupt to process */
            }
            
            if( ((irqs & ST25R3911_IRQ_MASK_FWL) != 0U) && ((irqs & ST25R3911_IRQ_MASK_TXE) == 0U) )
            {
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_RELOAD_FIFO;
            }
            else
            {
                gRFAL.TxRx.status = ERR_IO;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
                break;
            }
            
            /* fall through */
            
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_RELOAD_FIFO:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
        #if RFAL_FEATURE_NFCV
            /*******************************************************************************/
            /* In NFC-V streaming mode, the FIFO needs to be loaded with the coded bits    */
            if( (RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode) )
            {
                uint16_t maxLen;
                                
                /* Load FIFO with the remaining length or maximum available (which fit on the coding buffer) */
                maxLen = (uint16_t)MIN( (gRFAL.fifo.bytesTotal - gRFAL.fifo.bytesWritten), gRFAL.fifo.expWL);
                maxLen = (uint16_t)MIN( maxLen, sizeof(gRFAL.nfcvData.codingBuffer) );
                tmp    = 0;

                /* Calculate the bytes needed to be Written into FIFO (a incomplete byte will be added as 1byte) */
                ret = iso15693VCDCode(gRFAL.TxRx.ctx.txBuf, rfalConvBitsToBytes(gRFAL.TxRx.ctx.txBufLen), (((gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL) != 0U)?false:true), (((gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_NFCV_FLAG_MANUAL) != 0U)?false:true), (RFAL_MODE_POLL_PICOPASS == gRFAL.mode),
                          &gRFAL.fifo.bytesTotal, &gRFAL.nfcvData.nfcvOffset, gRFAL.nfcvData.codingBuffer, maxLen, &tmp);

                if( (ret != ERR_NONE) && (ret != ERR_AGAIN) )
                {
                    gRFAL.TxRx.status = ret;
                    gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
                    break;
                }

                /* Load FIFO with coded bytes */
                /* TODO: check tmp does not exceed 255 */
                st25r3911WriteFifo( gRFAL.nfcvData.codingBuffer, (uint8_t)tmp );
            }
            /*******************************************************************************/
            else
        #endif /* RFAL_FEATURE_NFCV */
            {
                /* Load FIFO with the remaining length or maximum available */
                tmp = MIN( (gRFAL.fifo.bytesTotal - gRFAL.fifo.bytesWritten), gRFAL.fifo.expWL);       /* tmp holds the number of bytes written on this iteration */
                /* TODO: check tmp does not exceed 255 */
                st25r3911WriteFifo( &gRFAL.TxRx.ctx.txBuf[gRFAL.fifo.bytesWritten], (uint8_t)tmp );
            }
            
            /* Update total written bytes to FIFO */
            gRFAL.fifo.bytesWritten += tmp;
            
            /* Check if a WL level is expected or TXE should come */
            gRFAL.TxRx.state = (( gRFAL.fifo.bytesWritten < gRFAL.fifo.bytesTotal ) ? RFAL_TXRX_STATE_TX_WAIT_WL : RFAL_TXRX_STATE_TX_WAIT_TXE);
            break;
            
            
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_WAIT_TXE:
           
            irqs = st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_FWL | ST25R3911_IRQ_MASK_TXE) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
               break;  /* No interrupt to process */
            }
                        
            
            if( (irqs & ST25R3911_IRQ_MASK_TXE) != 0U )
            {
                /* In Active comm start SW timer to measure FWT */
                if( rfalIsModeActiveComm( gRFAL.mode) && (gRFAL.TxRx.ctx.fwt != RFAL_FWT_NONE) && (gRFAL.TxRx.ctx.fwt != 0U) ) 
                {
                    /*rfalTimerStart( gRFAL.tmr.FWT, rfalConv1fcToMs( gRFAL.TxRx.ctx.fwt ) );
                    #define rfalTimerStart( timer, time_ms )         (timer) = (platformGetSysTick() + time_ms);      /*!< Configures and starts the RTOX timer          */
                    ( gRFAL.tmr.FWT ) = millis() + rfalConv1fcToMs( gRFAL.TxRx.ctx.fwt );
                }
                
                gRFAL.TxRx.state = RFAL_TXRX_STATE_TX_DONE;
            }
            else if( (irqs & ST25R3911_IRQ_MASK_FWL) != 0U )
            {
                /*******************************************************************************/
                /* REMARK: Silicon workaround ST25R3911 Errata #TBD                            */
                /* ST25R3911 may send a WL even when all bytes have been written to FIFO       */
                /*******************************************************************************/
                break;  /* Ignore ST25R3911 FIFO WL if total TxLen is already on the FIFO */
            }
            else
            {
               gRFAL.TxRx.status = ERR_IO;
               gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
               break;
            }
            
            /* fall through */
           
        
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_DONE:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            /* If no rxBuf is provided do not wait/expect Rx */
            if( gRFAL.TxRx.ctx.rxBuf == NULL )
            {
                /*Check if Observation Mode was enabled and disable it on ST25R391x */
                /* rfalCheckDisableObsMode() */
                if(gRFAL.conf.obsvModeRx != 0U){ 
                  st25r3911WriteTestRegister(0x01U, 0x00U);                 /*!< Disable ST25R3911 Observation mode                                                               */                  
                }    /*!< Checks if the observation mode is enabled, and applies on ST25R3911 */
                
                /* Clean up Transceive */
                rfalCleanupTransceive();
                                
                gRFAL.TxRx.status = ERR_NONE;
                gRFAL.TxRx.state  =  RFAL_TXRX_STATE_IDLE;
                break;
            }
            
            if(gRFAL.conf.obsvModeRx != 0U){ 
              st25r3911WriteTestRegister(0x01U, gRFAL.conf.obsvModeRx); /*!< Enable Observation mode 0x04 CSI: Digital output of AM channel CSO: Digital output of PM channel */
            }         /*!< Checks if the observation mode is enabled, and applies on ST25R3911 */
            
            /* Goto Rx */
            gRFAL.TxRx.state  =  RFAL_TXRX_STATE_RX_IDLE;
            break;
           
        /*******************************************************************************/
        case RFAL_TXRX_STATE_TX_FAIL:
            
            /* Error should be assigned by previous state */
            if( gRFAL.TxRx.status == ERR_BUSY )
            {
                gRFAL.TxRx.status = ERR_SYSTEM;
            }
            
            /*Check if Observation Mode was enabled and disable it on ST25R391x */
            /* rfalCheckDisableObsMode() */            
            if(gRFAL.conf.obsvModeRx != 0U){ 
              /*rfalST25R3911ObsModeDisable()*/
              st25r3911WriteTestRegister(0x01U, 0x00U);                 /*!< Disable ST25R3911 Observation mode                                                               */
            }    /*!< Checks if the observation mode is enabled, and applies on ST25R3911 */
            
            /* Clean up Transceive */
            rfalCleanupTransceive();
            
            gRFAL.TxRx.state = RFAL_TXRX_STATE_IDLE;
            break;
        
        /*******************************************************************************/
        default:
            gRFAL.TxRx.status = ERR_SYSTEM;
            gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_FAIL;
            break;
    }
}

void rfal_rfst25r3911b::rfalTransceiveRx( void )
{
    volatile uint32_t irqs;
    uint8_t           tmp;
    uint8_t           aux;

    irqs = ST25R3911_IRQ_MASK_NONE;
    
    if( gRFAL.TxRx.state != gRFAL.TxRx.lastState )
    {        
        /* rfalLogD( "RFAL: lastSt: %d curSt: %d \r\n", gRFAL.TxRx.lastState, gRFAL.TxRx.state ); */
        gRFAL.TxRx.lastState = gRFAL.TxRx.state;
    }
    
    switch( gRFAL.TxRx.state )
    {
      case RFAL_TXRX_STATE_RX_IDLE:
            
            /* Clear rx counters */
            gRFAL.fifo.bytesWritten   = 0;    // Total bytes written on RxBuffer
            gRFAL.fifo.bytesTotal     = 0;    // Total bytes in FIFO will now be from Rx
            if( gRFAL.TxRx.ctx.rxRcvdLen != NULL )
            {
                *gRFAL.TxRx.ctx.rxRcvdLen = 0;
            }
            
            gRFAL.TxRx.state = ( rfalIsModeActiveComm( gRFAL.mode ) ? RFAL_TXRX_STATE_RX_WAIT_EON : RFAL_TXRX_STATE_RX_WAIT_RXS );
            break;
            
       case RFAL_TXRX_STATE_RX_WAIT_RXS:
        
            /*******************************************************************************/
            /* If in Active comm, Check if FWT SW timer has expired */
            if( rfalIsModeActiveComm( gRFAL.mode ) && (gRFAL.TxRx.ctx.fwt != RFAL_FWT_NONE) && (gRFAL.TxRx.ctx.fwt != 0U) )
            {

                uint32_t uDiff;
                int32_t sDiff;
                
                uDiff = (gRFAL.tmr.FWT - millis());   /* Calculate the diff between the timers */
                sDiff = uDiff;                            /* Convert the diff to a signed var      */

                if( sDiff < 0 )  
                {
                    gRFAL.TxRx.status = ERR_TIMEOUT;
                    gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                    break;
                }
            }
            
            /*******************************************************************************/
            irqs = st25r3911GetInterrupt( ( ST25R3911_IRQ_MASK_RXS | ST25R3911_IRQ_MASK_NRE | ST25R3911_IRQ_MASK_EOF | ST25R3911_IRQ_MASK_RXE) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
                break;  /* No interrupt to process */
            }
            
                        
            /*******************************************************************************/
            /* REMARK: Silicon workaround ST25R3911 Errata #1.7                            */
            /* NRE interrupt may be triggered twice                                        */
            /* Ignore NRE if is detected together with no Rx Start                         */
            /*******************************************************************************/
            
            /* Only raise Timeout if NRE is detected with no Rx Start (NRT EMV mode)       */
            if( ((irqs & ST25R3911_IRQ_MASK_NRE) != 0U) && ((irqs & ST25R3911_IRQ_MASK_RXS) == 0U) )
            {
                gRFAL.TxRx.status = ERR_TIMEOUT;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                break;
            }
            
            /* Only raise Link Loss if EOF is detected with no Rx Start */
            if( ((irqs & ST25R3911_IRQ_MASK_EOF) != 0U) && ((irqs & ST25R3911_IRQ_MASK_RXS) == 0U) )
            {
                /* In AP2P a Field On has already occurred - treat this as timeout | mute */
                gRFAL.TxRx.status = ( rfalIsModeActiveComm( gRFAL.mode ) ? ERR_TIMEOUT : ERR_LINK_LOSS );
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                break;
            }
            
            if( (irqs & ST25R3911_IRQ_MASK_RXS) != 0U )
            {
                /* If we got RXS + RXE together, jump directly into RFAL_TXRX_STATE_RX_ERR_CHECK */
                if( (irqs & ST25R3911_IRQ_MASK_RXE) != 0U )
                {
                    gRFAL.TxRx.rxse  = true;
                    gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_ERR_CHECK;
                    break;
                }
                else
                {
                    /*******************************************************************************/
                    /* REMARK: Silicon workaround ST25R3911 Errata #1.1                            */
                    /* Rarely on corrupted frames I_rxs gets signaled but I_rxe is not signaled    */
                    /* Use a SW timer to handle an eventual missing RXE                            */
                    //rfalTimerStart( gRFAL.tmr.RXE, RFAL_NORXE_TOUT );
                    gRFAL.tmr.RXE = millis() + RFAL_NORXE_TOUT;
                    /*******************************************************************************/
                    
                    gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_WAIT_RXE;
                }
            }
            else if( (irqs & ST25R3911_IRQ_MASK_RXE) != 0U )
            {
                /*******************************************************************************/
                /* REMARK: Silicon workaround ST25R3911 Errata #1.9                            */
                /* ST25R3911 may indicate RXE without RXS previously, this happens upon some   */
                /* noise or incomplete byte frames with less than 4 bits                       */
                /*******************************************************************************/
                
                gRFAL.TxRx.status = ERR_IO;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                
                rfalErrorHandling();
                break;
            }
            else
            {
               gRFAL.TxRx.status = ERR_IO;
               gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
               break;
            }
            
            /* fall through */

        case RFAL_TXRX_STATE_RX_WAIT_RXE:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            irqs = st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_RXE | ST25R3911_IRQ_MASK_FWL | ST25R3911_IRQ_MASK_EOF) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
                /*******************************************************************************/
                /* REMARK: Silicon workaround ST25R3911B Errata #1.1                           */
                /* ST25R3911 may indicate RXS without RXE afterwards, this happens rarely on   */
                /* corrupted frames.                                                           */
                /* SW timer is used to timeout upon a missing RXE                              */
                uint32_t uDiff = (gRFAL.tmr.RXE - millis());
                int32_t sDiff  = uDiff;
                
                if( sDiff < 0 )
                {
                    gRFAL.TxRx.status = ERR_FRAMING;
                    gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                }
                /*******************************************************************************/
                    
                break;  /* No interrupt to process */
            }
            
            if( ((irqs & ST25R3911_IRQ_MASK_FWL) != 0U) && ((irqs & ST25R3911_IRQ_MASK_RXE) == 0U) )
            {
                gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_READ_FIFO;
                break;
            }
            
            gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_ERR_CHECK;
            /* fall through */
        case RFAL_TXRX_STATE_RX_ERR_CHECK:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
        
            /* Retrieve and check for any error irqs */
            irqs |= st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_CRC | ST25R3911_IRQ_MASK_PAR | ST25R3911_IRQ_MASK_ERR1 | ST25R3911_IRQ_MASK_ERR2 | ST25R3911_IRQ_MASK_COL) );
        
            if( (irqs & ST25R3911_IRQ_MASK_ERR1) != 0U )
            {
                gRFAL.TxRx.status = ERR_FRAMING;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;
                
                /* Check if there's a specific error handling for this */
                rfalErrorHandling();
                break;
            }
            /* Discard Soft Framing errors if not in EMVCo error handling */
            else if( ((irqs & ST25R3911_IRQ_MASK_ERR2) != 0U) && (gRFAL.conf.eHandling == RFAL_ERRORHANDLING_EMVCO) )
            {
                gRFAL.TxRx.status = ERR_FRAMING;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;
                
                /* Check if there's a specific error handling for this */
                rfalErrorHandling();
                break;
            }
            else if( (irqs & ST25R3911_IRQ_MASK_PAR) != 0U )
            {
                gRFAL.TxRx.status = ERR_PAR;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;
                
                /* Check if there's a specific error handling for this */
                rfalErrorHandling();
                break;
            }
            else if( (irqs & ST25R3911_IRQ_MASK_CRC) != 0U )
            {
                gRFAL.TxRx.status = ERR_CRC;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;
                
                /* Check if there's a specific error handling for this */
                rfalErrorHandling();
                break;
            }
            else if( (irqs & ST25R3911_IRQ_MASK_COL) != 0U )
            {
                gRFAL.TxRx.status = ERR_RF_COLLISION;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_READ_DATA;
                
                /* Check if there's a specific error handling for this */
                rfalErrorHandling();
                break;
            }
            else if( ((irqs & ST25R3911_IRQ_MASK_EOF) != 0U) && ((irqs & ST25R3911_IRQ_MASK_RXE) == 0U) )
            {
                 gRFAL.TxRx.status = ERR_LINK_LOSS;
                 gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                 break;
            }
            else if( ((irqs & ST25R3911_IRQ_MASK_RXE) != 0U) || (gRFAL.TxRx.rxse) )
            {
                /* Reception ended without any error indication,                  *
                 * check FIFO status for malformed or incomplete frames           */
                
                /* Check if the reception ends with an incomplete byte (residual bits) */
                if( rfalFIFOStatusIsIncompleteByte() )
                {
                   gRFAL.TxRx.status = ERR_INCOMPLETE_BYTE;
                }
                /* Check if the reception ends with missing parity bit */
                else if( rfalFIFOStatusIsMissingPar() )
                {
                   gRFAL.TxRx.status = ERR_FRAMING;
                }
                else
                {
                    /* MISRA 15.7 - Empty else */
                }
                
                gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_READ_DATA;
            }
            else
            {
                gRFAL.TxRx.status = ERR_IO;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
                break;
            }
                        
            /* fall through */
       case RFAL_TXRX_STATE_RX_READ_DATA:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
                        
            tmp = rfalFIFOStatusGetNumBytes();
                        
            /*******************************************************************************/
            /* Check if CRC should not be placed in rxBuf                                  */
            if( ((gRFAL.TxRx.ctx.flags & (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP) == 0U) )
            {
                /* Check if CRC is being placed into the FIFO and if received frame was bigger than CRC */
                if( st25r3911IsCRCinFIFO() && ((gRFAL.fifo.bytesTotal + tmp) > 0U) )
                {
                    /* By default CRC will not be placed into the rxBuffer */
                    if( ( tmp > (uint8_t)RFAL_CRC_LEN) )  
                    {
                        tmp -= (uint8_t)RFAL_CRC_LEN;
                    }
                    /* If the CRC was already placed into rxBuffer (due to WL interrupt where CRC was already in FIFO Read)
                     * cannot remove it from rxBuf. Can only remove it from rxBufLen not indicate the presence of CRC    */ 
                    else if(gRFAL.fifo.bytesTotal > (uint16_t)RFAL_CRC_LEN)                       
                    {                        
                        gRFAL.fifo.bytesTotal -= (uint16_t)RFAL_CRC_LEN;
                    }
                    else
                    {
                        /* MISRA 15.7 - Empty else */
                    }
                }
            }
            
            gRFAL.fifo.bytesTotal += tmp;                    /* add to total bytes counter */
            
            /*******************************************************************************/
            /* Check if remaining bytes fit on the rxBuf available                         */
            if( gRFAL.fifo.bytesTotal > rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen) )
            {
                tmp = (uint8_t)( rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen) - gRFAL.fifo.bytesWritten);
                
                gRFAL.TxRx.status = ERR_NOMEM;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
            }

            /*******************************************************************************/
            /* Retrieve remaining bytes from FIFO to rxBuf, and assign total length rcvd   */
            st25r3911ReadFifo( &gRFAL.TxRx.ctx.rxBuf[gRFAL.fifo.bytesWritten], tmp);
            if( (gRFAL.TxRx.ctx.rxRcvdLen != NULL) )
            {
                (*gRFAL.TxRx.ctx.rxRcvdLen) = (uint16_t)rfalConvBytesToBits( gRFAL.fifo.bytesTotal );
                if( rfalFIFOStatusIsIncompleteByte() )
                {
                    (*gRFAL.TxRx.ctx.rxRcvdLen) -= (RFAL_BITS_IN_BYTE - rfalFIFOGetNumIncompleteBits());
                }
            }
            
        #if RFAL_FEATURE_NFCV
            /*******************************************************************************/
            /* Decode sub bit stream into payload bits for NFCV, if no error found so far  */
            if( ((RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode)) && (gRFAL.TxRx.status == ERR_BUSY) )
            {
                ReturnCode ret;
                uint16_t offset = 0;

                ret = iso15693VICCDecode(gRFAL.TxRx.ctx.rxBuf, gRFAL.fifo.bytesTotal,
                        gRFAL.nfcvData.origCtx.rxBuf, rfalConvBitsToBytes(gRFAL.nfcvData.origCtx.rxBufLen), &offset, gRFAL.nfcvData.origCtx.rxRcvdLen, gRFAL.nfcvData.ignoreBits, (RFAL_MODE_POLL_PICOPASS == gRFAL.mode) );

                if( ((ERR_NONE == ret) || (ERR_CRC == ret))
                     && (((uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP & gRFAL.nfcvData.origCtx.flags) == 0U)
                     &&  ((*gRFAL.nfcvData.origCtx.rxRcvdLen % RFAL_BITS_IN_BYTE) == 0U)
                     &&  (*gRFAL.nfcvData.origCtx.rxRcvdLen >= rfalConvBytesToBits(RFAL_CRC_LEN) )
                   )
                {
                   *gRFAL.nfcvData.origCtx.rxRcvdLen -= (uint16_t)rfalConvBytesToBits(RFAL_CRC_LEN); /* Remove CRC */
                }
                
                /* Restore original ctx */
                gRFAL.TxRx.ctx    = gRFAL.nfcvData.origCtx;
                gRFAL.TxRx.status = ((ret != ERR_NONE) ? ret : ERR_BUSY);
            }
        #endif /* RFAL_FEATURE_NFCV */
            
            /*******************************************************************************/
            /* If an error as been marked/detected don't fall into to RX_DONE  */
            if( gRFAL.TxRx.status != ERR_BUSY )
            {
                gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_FAIL;
                break;
            }
            
            if( rfalIsModeActiveComm( gRFAL.mode ) )
            {
                gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_WAIT_EOF;
                break;
            }
            
            gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_DONE;
            /* fall through */ 
            
       case RFAL_TXRX_STATE_RX_DONE:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            /*Check if Observation Mode was enabled and disable it on ST25R391x */
            /* rfalCheckDisableObsMode(); */
            if(gRFAL.conf.obsvModeRx != 0U){ 
                st25r3911WriteTestRegister(0x01U, 0x00U);                 /*!< Disable ST25R3911 Observation mode                                                               */
             }    /*!< Checks if the observation mode is enabled, and applies on ST25R3911 */
            /* Clean up Transceive */
            rfalCleanupTransceive();

            
            gRFAL.TxRx.status = ERR_NONE;
            gRFAL.TxRx.state  = RFAL_TXRX_STATE_IDLE;
            break; 
       case RFAL_TXRX_STATE_RX_READ_FIFO:
        
            /*******************************************************************************/
            /* REMARK: Silicon workaround ST25R3911B Errata #1.1                           */
            /* ST25R3911 may indicate RXS without RXE afterwards, this happens rarely on   */
            /* corrupted frames.                                                           */
            /* Re-Start SW timer to handle an eventual missing RXE                         */
            gRFAL.tmr.RXE = millis() + RFAL_NORXE_TOUT;
            /*******************************************************************************/        
                    
        
            tmp = rfalFIFOStatusGetNumBytes();
            gRFAL.fifo.bytesTotal += tmp;
            
            /*******************************************************************************/
            /* Calculate the amount of bytes that still fits in rxBuf                      */
            aux = (uint8_t)(( gRFAL.fifo.bytesTotal > rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen) ) ? (rfalConvBitsToBytes(gRFAL.TxRx.ctx.rxBufLen) - gRFAL.fifo.bytesWritten) : tmp);
            
            /*******************************************************************************/
            /* Retrieve incoming bytes from FIFO to rxBuf, and store already read amount   */
            st25r3911ReadFifo( &gRFAL.TxRx.ctx.rxBuf[gRFAL.fifo.bytesWritten], aux);
            gRFAL.fifo.bytesWritten += aux;
            
            /*******************************************************************************/
            /* If the bytes already read were not the full FIFO WL, dump the remaining     *
             * FIFO so that ST25R391x can continue with reception                          */
            if( aux < tmp )
            {
                st25r3911ReadFifo( NULL, (tmp - aux) );
            }
            
            rfalFIFOStatusClear();
            gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_WAIT_RXE;
            break;

        case RFAL_TXRX_STATE_RX_FAIL:
            
            /*Check if Observation Mode was enabled and disable it on ST25R391x */
            /* rfalCheckDisableObsMode(); */
            if(gRFAL.conf.obsvModeRx != 0U){ 
                  st25r3911WriteTestRegister(0x01U, 0x00U);                 /*!< Disable ST25R3911 Observation mode                                                               */
            }    /*!< Checks if the observation mode is enabled, and applies on ST25R3911 */
            
            /* Clean up Transceive */
            rfalCleanupTransceive();
            
            /* Error should be assigned by previous state */
            if( gRFAL.TxRx.status == ERR_BUSY )
            {                
                gRFAL.TxRx.status = ERR_SYSTEM;
            }
             
            /*rfalLogD( "RFAL: curSt: %d  Error: %d \r\n", gRFAL.TxRx.state, gRFAL.TxRx.status );*/
            gRFAL.TxRx.state = RFAL_TXRX_STATE_IDLE;
            break;
            
        case RFAL_TXRX_STATE_RX_WAIT_EON:
            
            irqs = st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_EON | ST25R3911_IRQ_MASK_NRE) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
                break;  /* No interrupt to process */
            }
            
            if( (irqs & ST25R3911_IRQ_MASK_EON) != 0U )
            {
                gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_WAIT_RXS;
            }
            
            if( (irqs & ST25R3911_IRQ_MASK_NRE) != 0U )
            {
                /* ST25R3911 uses the NRT to measure other device's Field On max time: Tadt + (n x Trfw)  */
                gRFAL.TxRx.status = ERR_LINK_LOSS;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
            }
            break;

        
        /*******************************************************************************/    
        case RFAL_TXRX_STATE_RX_WAIT_EOF:
           
            irqs = st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_CAT | ST25R3911_IRQ_MASK_CAC) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
               break;  /* No interrupt to process */
            }
            
            if( (irqs & ST25R3911_IRQ_MASK_CAT) != 0U )
            {
               gRFAL.TxRx.state = RFAL_TXRX_STATE_RX_DONE;
            }
            else if( (irqs & ST25R3911_IRQ_MASK_CAC) != 0U )
            {
               gRFAL.TxRx.status = ERR_RF_COLLISION;
               gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
            }
            else
            {
               gRFAL.TxRx.status = ERR_IO;
               gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
            }
            break;
            
            
        /*******************************************************************************/
        default:
            gRFAL.TxRx.status = ERR_SYSTEM;
            gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
            break;        
    }
}

void rfal_rfst25r3911b::rfalErrorHandling( void )
{
    bool    rxHasIncParError;
    uint8_t fifoBytesToRead;
    uint8_t reEnRx[] = { ST25R3911_CMD_CLEAR_FIFO, ST25R3911_CMD_UNMASK_RECEIVE_DATA };
    

    fifoBytesToRead = rfalFIFOStatusGetNumBytes();
    
    
    /*******************************************************************************/
    /* EMVCo                                                                       */
    /*******************************************************************************/
    if( gRFAL.conf.eHandling == RFAL_ERRORHANDLING_EMVCO )
    {
        /*******************************************************************************/
        /* EMD Handling - NFC Forum Digital 1.1  4.1.1.1 ; EMVCo 2.6  4.9.2            */
        /* ReEnable the receiver on frames with a length < 4 bytes, upon:              */
        /*   - Collision or Framing error detected                                     */
        /*   - Residual bits are detected (hard framing error)                         */
        /*   - Parity error                                                            */
        /*   - CRC error                                                               */
        /*******************************************************************************/        
     
        /* Check if reception has incompete bytes or parity error */
        rxHasIncParError = ( rfalFIFOStatusIsIncompleteByte() ? true : rfalFIFOStatusIsMissingPar() );   /* MISRA 13.5 */
        
        /* In case there are residual bits decrement FIFO bytes */
        if( (fifoBytesToRead > 0U) && rxHasIncParError)
        {
            fifoBytesToRead--;
        }
            
        if( ( (gRFAL.fifo.bytesTotal + fifoBytesToRead) < RFAL_EMVCO_RX_MAXLEN )            &&
            ( (gRFAL.TxRx.status == ERR_RF_COLLISION) || (gRFAL.TxRx.status == ERR_FRAMING) || 
              (gRFAL.TxRx.status == ERR_PAR)          || (gRFAL.TxRx.status == ERR_CRC)     || 
              rxHasIncParError                                                               ) )
        {
            /* Ignore this reception, ReEnable receiver */
            st25r3911ExecuteCommands( reEnRx, sizeof(reEnRx) );
            
            rfalFIFOStatusClear();
            gRFAL.fifo.bytesTotal = 0;
            gRFAL.TxRx.status = ERR_BUSY;
            gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_WAIT_RXS;
        }
        return;
    }

    /*******************************************************************************/
    /* ISO14443A Mode                                                              */
    /*******************************************************************************/
    if( gRFAL.mode == RFAL_MODE_POLL_NFCA )
    {
        
        /*******************************************************************************/
        /* If we received one incomplete byte (not a block and a incomplete byte at    *
         * the end) we will raise a specific error ( support for T2T 4 bit ACK / NAK )   *
         * Otherwise just leave it as an CRC/FRAMING/PAR error                         */    
        /*******************************************************************************/
        if( (gRFAL.TxRx.status == ERR_PAR) || (gRFAL.TxRx.status == ERR_CRC) )
        {
            if( rfalFIFOStatusIsIncompleteByte() && (fifoBytesToRead == RFAL_NFC_RX_INCOMPLETE_LEN) )
            {
                st25r3911ReadFifo( (uint8_t*)(gRFAL.TxRx.ctx.rxBuf), fifoBytesToRead );
                if( (gRFAL.TxRx.ctx.rxRcvdLen) != NULL )
                {
                    *gRFAL.TxRx.ctx.rxRcvdLen = rfalFIFOGetNumIncompleteBits();
                }
                
                gRFAL.TxRx.status = ERR_INCOMPLETE_BYTE;
                gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_FAIL;
            }
        }
    }
    
}

uint8_t rfal_rfst25r3911b::rfalFIFOStatusGetNumBytes( void )
{
    rfalFIFOStatusUpdate();
    
    return gRFAL.fifo.status[RFAL_FIFO_STATUS_REG1]; 
   
}


/*******************************************************************************/
bool rfal_rfst25r3911b::rfalFIFOStatusIsIncompleteByte( void )
{
    rfalFIFOStatusUpdate();
    return ((gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] & (ST25R3911_REG_FIFO_RX_STATUS2_mask_fifo_lb | ST25R3911_REG_FIFO_RX_STATUS2_fifo_ncp)) != 0U);
}


/*******************************************************************************/
bool rfal_rfst25r3911b::rfalFIFOStatusIsMissingPar( void )
{
    rfalFIFOStatusUpdate();
    return ((gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] & ST25R3911_REG_FIFO_RX_STATUS2_np_lb) != 0U);
}


/*******************************************************************************/
uint8_t rfal_rfst25r3911b::rfalFIFOGetNumIncompleteBits( void )
{
    rfalFIFOStatusUpdate();
    return ((gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] & ST25R3911_REG_FIFO_RX_STATUS2_mask_fifo_lb) >> ST25R3911_REG_FIFO_RX_STATUS2_shift_fifo_lb);
}

void rfal_rfst25r3911b::rfalFIFOStatusUpdate( void )
{
    if(gRFAL.fifo.status[RFAL_FIFO_STATUS_REG2] == RFAL_FIFO_STATUS_INVALID)
    {
        st25r3911ReadMultipleRegisters( ST25R3911_REG_FIFO_RX_STATUS1, gRFAL.fifo.status, ST25R3911_FIFO_STATUS_LEN );
    }
}

ReturnCode rfal_rfst25r3911b::rfalRunListenModeWorker()
{
    volatile uint32_t irqs;
    uint8_t           tmp;
    
    if( gRFAL.state != RFAL_STATE_LM )
    {
        return ERR_WRONG_STATE;
    }
    
    switch( gRFAL.Lm.state )
    {
        /*******************************************************************************/
        case RFAL_LM_STATE_POWER_OFF:
            
            irqs = st25r3911GetInterrupt( (  ST25R3911_IRQ_MASK_EON ) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
              break;  /* No interrupt to process */
            }
            
            if( (irqs & ST25R3911_IRQ_MASK_EON) != 0U )
            {
                rfalListenSetState( RFAL_LM_STATE_IDLE );
            }
            else
            {
                break;
            }
            /* fall through */
            
              
        /*******************************************************************************/
        case RFAL_LM_STATE_IDLE:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            irqs = st25r3911GetInterrupt( ( ST25R3911_IRQ_MASK_NFCT | ST25R3911_IRQ_MASK_RXE | ST25R3911_IRQ_MASK_EOF ) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
                break;  /* No interrupt to process */
            }
            
            if( (irqs & ST25R3911_IRQ_MASK_NFCT) != 0U )
            {
                /* Retrieve detected bitrate */
                uint8_t    newBr;
                st25r3911ReadRegister( ST25R3911_REG_NFCIP1_BIT_RATE, &newBr );
                newBr >>= ST25R3911_REG_NFCIP1_BIT_RATE_nfc_rate_shift;

                if (newBr > ST25R3911_REG_BIT_RATE_rxrate_424)
                {
                    newBr = ST25R3911_REG_BIT_RATE_rxrate_424;
                }

                gRFAL.Lm.brDetected = (rfalBitRate)(newBr); /* PRQA S 4342 # MISRA 10.5 - Guaranteed that no invalid enum values may be created. See also equalityGuard_RFAL_BR_106 ff.*/
            }
            if( ((irqs & ST25R3911_IRQ_MASK_RXE) != 0U) && (gRFAL.Lm.brDetected != RFAL_BR_KEEP) )
            {
                irqs = st25r3911GetInterrupt( ( ST25R3911_IRQ_MASK_RXE | ST25R3911_IRQ_MASK_EOF | ST25R3911_IRQ_MASK_CRC | ST25R3911_IRQ_MASK_PAR | ST25R3911_IRQ_MASK_ERR2 | ST25R3911_IRQ_MASK_ERR1 ) );
                
                if( ((irqs & ST25R3911_IRQ_MASK_CRC) != 0U) || ((irqs & ST25R3911_IRQ_MASK_PAR) != 0U) || ((irqs & ST25R3911_IRQ_MASK_ERR1) != 0U) )
                {
                    /* nfc_ar may have triggered RF Collision Avoidance, disable it before executing Clear (Stop All activities) */
                    st25r3911ClrRegisterBits( ST25R3911_REG_MODE, ST25R3911_REG_MODE_nfc_ar );
                    st25r3911ExecuteCommand( ST25R3911_CMD_CLEAR_FIFO );
                    st25r3911ExecuteCommand( ST25R3911_CMD_UNMASK_RECEIVE_DATA );
                    st25r3911SetRegisterBits( ST25R3911_REG_MODE, ST25R3911_REG_MODE_nfc_ar );
                    st25r3911TxOff();
                    break; /* A bad reception occurred, remain in same state */
                }
                
                /* Retrieve received data */
                st25r3911ReadRegister(ST25R3911_REG_FIFO_RX_STATUS1, &tmp);
                *gRFAL.Lm.rxLen = tmp;
                
                st25r3911ReadFifo( gRFAL.Lm.rxBuf, (uint8_t)MIN( *gRFAL.Lm.rxLen, rfalConvBitsToBytes(gRFAL.Lm.rxBufLen) ) );
                
                /* Check if the data we got has at least the CRC and remove it, otherwise leave at 0 */
                *gRFAL.Lm.rxLen  -= ((*gRFAL.Lm.rxLen > RFAL_CRC_LEN) ? RFAL_CRC_LEN : *gRFAL.Lm.rxLen);
                *gRFAL.Lm.rxLen   = (uint16_t)rfalConvBytesToBits( *gRFAL.Lm.rxLen );
                gRFAL.Lm.dataFlag = true;
                
                /*Check if Observation Mode was enabled and disable it on ST25R391x */
                /* rfalCheckDisableObsMode(); */
                if(gRFAL.conf.obsvModeRx != 0U){ 
                  st25r3911WriteTestRegister(0x01U, 0x00U);                 /*!< Disable ST25R3911 Observation mode                                                               */
                }    /*!< Checks if the observation mode is enabled, and applies on ST25R3911 */
            }
            else if( ((irqs & ST25R3911_IRQ_MASK_EOF) != 0U) && (!gRFAL.Lm.dataFlag) )
            {
                rfalListenSetState( RFAL_LM_STATE_POWER_OFF );
            }
            else
            {
                /* MISRA 15.7 - Empty else */
            }
            break;
            
            /*******************************************************************************/
            case RFAL_LM_STATE_READY_F:
            case RFAL_LM_STATE_READY_A:
            case RFAL_LM_STATE_ACTIVE_A:
            case RFAL_LM_STATE_ACTIVE_Ax:
            case RFAL_LM_STATE_SLEEP_A:
            case RFAL_LM_STATE_SLEEP_B:
            case RFAL_LM_STATE_SLEEP_AF:
            case RFAL_LM_STATE_READY_Ax:
            case RFAL_LM_STATE_CARDEMU_4A:
            case RFAL_LM_STATE_CARDEMU_4B:
            case RFAL_LM_STATE_CARDEMU_3:
                return ERR_INTERNAL;
                
            case RFAL_LM_STATE_TARGET_F:
            case RFAL_LM_STATE_TARGET_A:
                break;
                
            /*******************************************************************************/
            default:
                return ERR_WRONG_STATE;
    }
    return ERR_NONE;
}

ReturnCode rfal_rfst25r3911b::rfalListenSetState( rfalLmState newSt )
{
    ReturnCode  ret;
    uint8_t     tmp;
    rfalLmState newState;
    bool        reSetState;
        
    /*rfalLogD( "RFAL: curState: %02X newState: %02X \r\n", gRFAL.Lm.state, newSt );*/
    
    /* SetState clears the Data flag */
    gRFAL.Lm.dataFlag = false;
    newState          = newSt;
    ret               = ERR_NONE;
    
    do{
        reSetState = false;

        /*******************************************************************************/
        switch( newState )
        {
            /*******************************************************************************/
            case RFAL_LM_STATE_POWER_OFF:
                
                /*******************************************************************************/
                /* Disable nfc_ar as RF Collision Avoidance timer may have already started */
                st25r3911ClrRegisterBits( ST25R3911_REG_MODE, ST25R3911_REG_MODE_nfc_ar );
                
                st25r3911ExecuteCommand( ST25R3911_CMD_CLEAR_FIFO );
                    
                /* Ensure that our field is Off, as automatic response RF Collision Avoidance may have been triggered */
                st25r3911TxOff();
                
                /*******************************************************************************/
                /* Ensure that the NFCIP1 mode is disabled */
                st25r3911ClrRegisterBits( ST25R3911_REG_ISO14443A_NFC, ST25R3911_REG_ISO14443A_NFC_nfc_f0 );
                
                
                /*******************************************************************************/
                /* Clear and enable required IRQs */
                st25r3911DisableInterrupts( ST25R3911_IRQ_MASK_ALL );
                
                
                st25r3911GetInterrupt( (ST25R3911_IRQ_MASK_NFCT | ST25R3911_IRQ_MASK_RXS | ST25R3911_IRQ_MASK_CRC | ST25R3911_IRQ_MASK_ERR1 |
                                        ST25R3911_IRQ_MASK_ERR2 | ST25R3911_IRQ_MASK_PAR | ST25R3911_IRQ_MASK_EON | ST25R3911_IRQ_MASK_EOF  | ST25R3911_IRQ_MASK_RXE ) );
                
                
                /*******************************************************************************/
                /* REMARK: Silicon workaround ST25R3911 Errata TDB                             */
                /* RXS and NFCT are triggered very close (specially in higher bitrates).       *
                 * If the interrupt status register is being read when NFCT is trigerred, the  *
                 * IRQ line might go low and NFCT is not signalled on the status register.     *
                 * For initial bitrate detection, mask RXS, only wait for NFCT and RXE.        */
                /*******************************************************************************/
                
                st25r3911EnableInterrupts( (ST25R3911_IRQ_MASK_NFCT | ST25R3911_IRQ_MASK_CRC | ST25R3911_IRQ_MASK_ERR1 |
                                            ST25R3911_IRQ_MASK_ERR2 | ST25R3911_IRQ_MASK_PAR | ST25R3911_IRQ_MASK_EON | ST25R3911_IRQ_MASK_EOF  | ST25R3911_IRQ_MASK_RXE ) );
                
                /*******************************************************************************/
                /* Clear the bitRate previously detected */
                gRFAL.Lm.brDetected = RFAL_BR_KEEP;
                
                
                /*******************************************************************************/
                /* Apply the BitRate detection mode mode */
                st25r3911WriteRegister( ST25R3911_REG_MODE, (ST25R3911_REG_MODE_targ_targ | ST25R3911_REG_MODE_om_bit_rate_detection | ST25R3911_REG_MODE_nfc_ar_on)  );
                
                
                /*******************************************************************************/
                /* REMARK: Silicon workaround ST25R3911 Errata #1.3                            */
                /* Even though bitrate is going to be detected the bitrate must be set to      *
                 * 106kbps to get correct 106kbps parity                                       */
                st25r3911WriteRegister( ST25R3911_REG_BIT_RATE, (ST25R3911_REG_BIT_RATE_txrate_106 | ST25R3911_REG_BIT_RATE_rxrate_106) );
                /*******************************************************************************/
                
                
                /*******************************************************************************/
                /* Check if external Field is already On */
                if( ( st25r3911CheckReg(ST25R3911_REG_AUX_DISPLAY, ST25R3911_REG_AUX_DISPLAY_efd_o, ST25R3911_REG_AUX_DISPLAY_efd_o ) ) )
                {
                    reSetState = true;
                    newState   = RFAL_LM_STATE_IDLE;                         /* Set IDLE state */
                }
                break;
            
            /*******************************************************************************/
            case RFAL_LM_STATE_IDLE:
            
                /*******************************************************************************/
                /* In Active P2P the Initiator may:  Turn its field On;  LM goes into IDLE state;
                 *      Initiator sends an unexpected frame raising a Protocol error; Initiator 
                 *      turns its field Off and ST25R3911 performs the automatic RF Collision 
                 *      Avoidance keeping our field On; upon a Protocol error upper layer sets 
                 *      again the state to IDLE to clear dataFlag and wait for next data.
                 *      
                 * Ensure that when upper layer calls SetState(IDLE), it restores initial 
                 * configuration and that check whether an external Field is still present     */
                 
                /* nfc_ar may have triggered RF Collision Avoidance, disable it before executing Clear (Stop All activities) */
                st25r3911ClrRegisterBits( ST25R3911_REG_MODE, ST25R3911_REG_MODE_nfc_ar );
                st25r3911ExecuteCommand( ST25R3911_CMD_CLEAR_FIFO );
                st25r3911SetRegisterBits( ST25R3911_REG_MODE, ST25R3911_REG_MODE_nfc_ar );
                
                /* Ensure that our field is Off, as automatic response RF Collision Avoidance may have been triggered */
                st25r3911TxOff();

                
                /* Load 2nd/3rd stage gain setting from registers into the receiver */
                st25r3911ExecuteCommand( ST25R3911_CMD_CLEAR_SQUELCH );
                
                /*******************************************************************************/
                /* REMARK: Silicon workaround ST25R3911 Errata #1.4                            */
                /* Enable; disable; enable mixer to make sure the digital decoder is in        *
                 * high state. This also switches the demodulator to mixer mode.               */
                st25r3911ReadRegister( ST25R3911_REG_RX_CONF1, &tmp );
                st25r3911WriteRegister( ST25R3911_REG_RX_CONF1, (tmp | ST25R3911_REG_RX_CONF1_amd_sel) );
                st25r3911WriteRegister( ST25R3911_REG_RX_CONF1, (uint8_t)(tmp & ~ST25R3911_REG_RX_CONF1_amd_sel) );
                st25r3911WriteRegister( ST25R3911_REG_RX_CONF1, (tmp | ST25R3911_REG_RX_CONF1_amd_sel) );
                /*******************************************************************************/
                
                /* ReEnable the receiver */
                st25r3911ExecuteCommand( ST25R3911_CMD_UNMASK_RECEIVE_DATA );
                
                
                /* If external Field is no longer detected go back to POWER_OFF */
                if( !st25r3911IsExtFieldOn() )
                {
                    reSetState = true;
                    newState   = RFAL_LM_STATE_POWER_OFF;                    /* Set POWER_OFF state */
                }

                /*******************************************************************************/
                /*Check if Observation Mode is enabled and set it on ST25R391x */
                /* rfalCheckEnableObsModeRx(); */
                if(gRFAL.conf.obsvModeRx != 0U){ 
                  st25r3911WriteTestRegister(0x01U, gRFAL.conf.obsvModeRx); /*!< Enable Observation mode 0x04 CSI: Digital output of AM channel CSO: Digital output of PM channel */
                }         /*!< Checks if the observation mode is enabled, and applies on ST25R3911 */
                break;
                
            /*******************************************************************************/
            case RFAL_LM_STATE_TARGET_A:
            case RFAL_LM_STATE_TARGET_F:
                /* States not handled by the LM, just keep state context */
                break;
                
            /*******************************************************************************/
            case RFAL_LM_STATE_READY_F:
            case RFAL_LM_STATE_CARDEMU_3:
            case RFAL_LM_STATE_READY_Ax:
            case RFAL_LM_STATE_READY_A:
            case RFAL_LM_STATE_ACTIVE_Ax:
            case RFAL_LM_STATE_ACTIVE_A:
            case RFAL_LM_STATE_SLEEP_A:
            case RFAL_LM_STATE_SLEEP_B:
            case RFAL_LM_STATE_SLEEP_AF:
            case RFAL_LM_STATE_CARDEMU_4A:
            case RFAL_LM_STATE_CARDEMU_4B:
                return ERR_NOTSUPP;
                
            /*******************************************************************************/
            default:
                return ERR_WRONG_STATE;
        }
    }
    while( reSetState );
    
    gRFAL.Lm.state = newState;
    
    return ret;
}

void rfal_rfst25r3911b::rfalRunWakeUpModeWorker( void )
{
    uint32_t irqs;
    
    if( gRFAL.state != RFAL_STATE_WUM )
    {
        return;
    }
    
    switch( gRFAL.wum.state )
    {
        case RFAL_WUM_STATE_ENABLED:
        case RFAL_WUM_STATE_ENABLED_WOKE:
            
            irqs = st25r3911GetInterrupt( ( ST25R3911_IRQ_MASK_WT | ST25R3911_IRQ_MASK_WAM | ST25R3911_IRQ_MASK_WPH | ST25R3911_IRQ_MASK_WCAP ) );
            if( irqs == ST25R3911_IRQ_MASK_NONE )
            {
               break;  /* No interrupt to process */
            }
            
            /*******************************************************************************/
            /* Check and mark which measurement(s) cause interrupt */
            if((irqs & ST25R3911_IRQ_MASK_WAM) != 0U)
            {
                gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
            }
            
            if((irqs & ST25R3911_IRQ_MASK_WPH) != 0U)
            {
                gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
            }
            
            if((irqs & ST25R3911_IRQ_MASK_WCAP) != 0U)
            {
                gRFAL.wum.state = RFAL_WUM_STATE_ENABLED_WOKE;
            }
            break;
            
        default:
            /* MISRA 16.4: no empty default statement (a comment being enough) */
            break;
    }
}

ReturnCode rfal_rfst25r3911b::rfalTransceiveBlockingTx( uint8_t* txBuf, uint16_t txBufLen, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t* actLen, uint32_t flags, uint32_t fwt )
{
    ReturnCode               ret;
    rfalTransceiveContext    ctx;
    
    rfalCreateByteFlagsTxRxContext( ctx, txBuf, txBufLen, rxBuf, rxBufLen, actLen, flags, fwt );
    EXIT_ON_ERR( ret, rfalStartTransceive( &ctx ) );
    
    return rfalTransceiveRunBlockingTx();
}

ReturnCode rfal_rfst25r3911b::rfalStartTransceive( const rfalTransceiveContext *ctx )
{
    uint32_t FxTAdj;  /* FWT or FDT adjustment calculation */
    
    /* Check for valid parameters */
    if( ctx == NULL )
    {
        return ERR_PARAM;
    }
    
    /* Ensure that RFAL is already Initialized and the mode has been set */
    if( (gRFAL.state >= RFAL_STATE_MODE_SET) /*&& (gRFAL.TxRx.state == RFAL_TXRX_STATE_INIT )*/ )
    {
        /*******************************************************************************/
        /* Check whether the field is already On, otherwise no TXE will be received  */
        if( !st25r3911IsTxEnabled() && (!rfalIsModePassiveListen( gRFAL.mode ) && (ctx->txBuf != NULL)) )
        {
            return ERR_WRONG_STATE;
        }
        
        gRFAL.TxRx.ctx = *ctx;
        
        /*******************************************************************************/
        if( gRFAL.timings.FDTListen != RFAL_TIMING_NONE )
        {
            /* Calculate MRT adjustment accordingly to the current mode */
            FxTAdj = RFAL_FDT_LISTEN_MRT_ADJUSTMENT;
            if(gRFAL.mode == RFAL_MODE_POLL_NFCA)      { FxTAdj += (uint32_t)RFAL_FDT_LISTEN_A_ADJUSTMENT; }
            if(gRFAL.mode == RFAL_MODE_POLL_NFCA_T1T)  { FxTAdj += (uint32_t)RFAL_FDT_LISTEN_A_ADJUSTMENT; }
            if(gRFAL.mode == RFAL_MODE_POLL_NFCB)      { FxTAdj += (uint32_t)RFAL_FDT_LISTEN_B_ADJUSTMENT; }
            if(gRFAL.mode == RFAL_MODE_POLL_NFCV)      { FxTAdj += (uint32_t)RFAL_FDT_LISTEN_V_ADJUSTMENT; }
            
            
            /* Set Minimum FDT(Listen) in which PICC is not allowed to send a response */
            st25r3911WriteRegister( ST25R3911_REG_MASK_RX_TIMER, (uint8_t)rfalConv1fcTo64fc( (FxTAdj > gRFAL.timings.FDTListen) ? RFAL_ST25R3911_MRT_MIN_1FC : (gRFAL.timings.FDTListen - FxTAdj) ) );
        }
        
        /*******************************************************************************/
        /* FDT Poll will be loaded in rfalPrepareTransceive() once the previous was expired */
        
        /*******************************************************************************/
        if( rfalIsModePassiveComm( gRFAL.mode ) )  /* Passive Comms */
        {
            if( (gRFAL.TxRx.ctx.fwt != RFAL_FWT_NONE) && (gRFAL.TxRx.ctx.fwt != 0U) )
            {
                /* Ensure proper timing configuration */
                if( gRFAL.timings.FDTListen >= gRFAL.TxRx.ctx.fwt )
                {
                    return ERR_PARAM;
                }
        
                FxTAdj = RFAL_FWT_ADJUSTMENT;
                if(gRFAL.mode == RFAL_MODE_POLL_NFCA)      { FxTAdj += (uint32_t)RFAL_FWT_A_ADJUSTMENT;    }
                if(gRFAL.mode == RFAL_MODE_POLL_NFCA_T1T)  { FxTAdj += (uint32_t)RFAL_FWT_A_ADJUSTMENT;    }
                if(gRFAL.mode == RFAL_MODE_POLL_NFCB)      { FxTAdj += (uint32_t)RFAL_FWT_B_ADJUSTMENT;    }
                if(gRFAL.mode == RFAL_MODE_POLL_NFCF)      
                {
                    FxTAdj += (uint32_t)((gRFAL.txBR == RFAL_BR_212) ? RFAL_FWT_F_212_ADJUSTMENT : RFAL_FWT_F_424_ADJUSTMENT );
                }
                
                /* Ensure that the given FWT doesn't exceed NRT maximum */
                gRFAL.TxRx.ctx.fwt = MIN( (gRFAL.TxRx.ctx.fwt + FxTAdj), RFAL_ST25R3911_NRT_MAX_1FC );
                
                /* Set FWT in the NRT */
                st25r3911SetNoResponseTime_64fcs( rfalConv1fcTo64fc( gRFAL.TxRx.ctx.fwt ) );
            }
            else
            {
                /* Disable NRT, no NRE will be triggered, therefore wait endlessly for Rx */
                st25r3911SetNoResponseTime_64fcs( RFAL_ST25R3911_NRT_DISABLED );
            }
        }
        else /* Active Comms */
        {
            /* Setup NRT timer for rf response RF collision timeout. */
            st25r3911SetNoResponseTime_64fcs( rfalConv1fcTo64fc(RFAL_AP2P_FIELDON_TADTTRFW) );
            
            /* In Active Mode No Response Timer cannot be used to measure FWT a SW timer is used instead */
        }
        
        gRFAL.state       = RFAL_STATE_TXRX;
        gRFAL.TxRx.state  = RFAL_TXRX_STATE_TX_IDLE;
        gRFAL.TxRx.status = ERR_BUSY;
        gRFAL.TxRx.rxse   = false;
        
    #if RFAL_FEATURE_NFCV        
        /*******************************************************************************/
        if( (RFAL_MODE_POLL_NFCV == gRFAL.mode) || (RFAL_MODE_POLL_PICOPASS == gRFAL.mode) )
        { /* Exchange receive buffer with internal buffer */
            gRFAL.nfcvData.origCtx = gRFAL.TxRx.ctx;

            gRFAL.TxRx.ctx.rxBuf    = ((gRFAL.nfcvData.origCtx.rxBuf != NULL) ? gRFAL.nfcvData.codingBuffer : NULL);
            gRFAL.TxRx.ctx.rxBufLen = (uint16_t)rfalConvBytesToBits(sizeof(gRFAL.nfcvData.codingBuffer));
            gRFAL.TxRx.ctx.flags = (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_MANUAL
                                 | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_KEEP
                                 | (uint32_t)RFAL_TXRX_FLAGS_NFCIP1_OFF
                                 | (uint32_t)(gRFAL.nfcvData.origCtx.flags & (uint32_t)RFAL_TXRX_FLAGS_AGC_OFF)
                                 | (uint32_t)RFAL_TXRX_FLAGS_PAR_RX_KEEP
                                 | (uint32_t)RFAL_TXRX_FLAGS_PAR_TX_NONE;
            
            /* In NFCV a TxRx with a valid txBuf and txBufSize==0 indicates to send an EOF */
            /* Skip logic below that would go directly into receive                        */
            if ( gRFAL.TxRx.ctx.txBuf != NULL )
            {
                return  ERR_NONE;
            }
        }
    #endif /* RFAL_FEATURE_NFCV */

        
        /*******************************************************************************/
        /* Check if the Transceive start performing Tx or goes directly to Rx          */
        if( (gRFAL.TxRx.ctx.txBuf == NULL) || (gRFAL.TxRx.ctx.txBufLen == 0U) )
        {
            /* Clear FIFO, Clear and Enable the Interrupts */
            rfalPrepareTransceive( );
            
            /* Disable our field upon a Rx reEnable on AP2P */
            if( rfalIsModeActiveComm(gRFAL.mode) )
            {
                st25r3911TxOff();
            }
            
            /* No Tx done, enable the Receiver */
            st25r3911ExecuteCommand( ST25R3911_CMD_UNMASK_RECEIVE_DATA );

            /* Start NRT manually, if FWT = 0 (wait endlessly for Rx) chip will ignore anyhow */
            st25r3911ExecuteCommand( ST25R3911_CMD_START_NO_RESPONSE_TIMER );
            
            gRFAL.TxRx.state  = RFAL_TXRX_STATE_RX_IDLE;
        }
        
        return ERR_NONE;
    }
    
    return ERR_WRONG_STATE;
}

ReturnCode rfal_rfst25r3911b::rfalTransceiveRunBlockingTx( void )
{
    ReturnCode  ret;
        
    do{
        rfalWorker();
        ret = rfalGetTransceiveStatus();
    }
    while( rfalIsTransceiveInTx() && (ret == ERR_BUSY) );
    
    if( rfalIsTransceiveInRx() )
    {
        return ERR_NONE;
    }
    
    return ret;
}

ReturnCode rfal_rfst25r3911b::rfalTransceiveBlockingRx( void )
{
    ReturnCode ret;
    
    do{
        rfalWorker();
        ret = rfalGetTransceiveStatus();
    }
    while( rfalIsTransceiveInRx() && (ret == ERR_BUSY) );
        
    return ret;
}
