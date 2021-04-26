#include "rfal_rfst25r3911b.h"

#define LED4_PIN D5

void rfal_rfst25r3911b::st25r3911InitInterrupts( void )
{
  
    st25r3911interrupt.callback     = NULL;
    st25r3911interrupt.prevCallback = NULL;
    st25r3911interrupt.status       = ST25R3911_IRQ_MASK_NONE;
    st25r3911interrupt.mask         = ST25R3911_IRQ_MASK_NONE;
    
    
}

void rfal_rfst25r3911b::st25r3911Isr( void )
{
    st25r3911CheckForReceivedInterrupts();
    
    if (NULL != st25r3911interrupt.callback)
    {
        st25r3911interrupt.callback();
    }
}

void rfal_rfst25r3911b::st25r3911CheckForReceivedInterrupts()
{
    
    uint8_t  iregs[ST25R3911_INT_REGS_LEN];
    uint32_t irqStatus; 

    irqStatus = ST25R3911_IRQ_MASK_NONE;
    memset( iregs, (int32_t)(ST25R3911_IRQ_MASK_ALL & 0xFFU), ST25R3911_INT_REGS_LEN );  /* MISRA 10.3 */

    Serial.println("CHECK");
    /* In case the IRQ is Edge (not Level) triggered read IRQs until done */
    /* DOVREBBE ESSERE WHILE MA NON FUNZIONA*/
    while( digitalRead(int_pin) == HIGH )
    {
        
        st25r3911ReadMultipleRegisters(ST25R3911_REG_IRQ_MAIN, iregs, sizeof(iregs));
         /*Serial.println(iregs[0]);
         Serial.println(iregs[1]);
         Serial.println(iregs[2]);*/
        // Serial.println(digitalRead(int_pin));
        irqStatus |= (uint32_t)iregs[0];
        irqStatus |= (uint32_t)iregs[1]<<8;
        irqStatus |= (uint32_t)iregs[2]<<16;
        
    }
    Serial.println("END CHECK");
    /* Forward all interrupts, even masked ones to application. */
    st25r3911interrupt.status |= irqStatus;
    
}

void rfal_rfst25r3911b::st25r3911ModifyInterrupts(uint32_t clr_mask, uint32_t set_mask)
{
  uint8_t i;
    uint32_t old_mask;
    uint32_t new_mask;

    old_mask = st25r3911interrupt.mask;
    new_mask = (~old_mask & set_mask) | (old_mask & clr_mask);
    st25r3911interrupt.mask &= ~clr_mask;
    st25r3911interrupt.mask |= set_mask;
    for (i=0; i<3U ; i++)
    { 
        if (((new_mask >> (i*8U)) & 0xffU) == 0U) {
            continue;
        }
        st25r3911WriteRegister((ST25R3911_REG_IRQ_MASK_MAIN + i), (uint8_t)((st25r3911interrupt.mask>>(i*8U))&0xffU));
    }
}

void rfal_rfst25r3911b::st25r3911DisableInterrupts(uint32_t mask)
{
    st25r3911ModifyInterrupts(0,mask);
}

void rfal_rfst25r3911b::st25r3911ClearInterrupts()
{
  uint8_t iregs[3];

    st25r3911ReadMultipleRegisters(ST25R3911_REG_IRQ_MAIN, iregs, 3);
    st25r3911interrupt.status = 0;
}

uint32_t rfal_rfst25r3911b::st25r3911GetInterrupt(uint32_t mask)
{
  uint32_t irqs;

    irqs = (st25r3911interrupt.status & mask);
    if (irqs != ST25R3911_IRQ_MASK_NONE)
    {
        st25r3911interrupt.status &= ~irqs;
    }
    return irqs;
}

void rfal_rfst25r3911b::st25r3911EnableInterrupts(uint32_t mask)
{
  st25r3911ModifyInterrupts(mask,0);
}

uint32_t rfal_rfst25r3911b::st25r3911WaitForInterruptsTimed(uint32_t mask, uint16_t tmo)
{
    uint32_t tmr;
    uint32_t status;
    bool expired = false;
    
    tmr = millis() + tmo;
     
    do 
    {
       
        status = (st25r3911interrupt.status & mask);
        
        if((tmr - millis()) < 0)   /* Calculate the diff between the timers */
            expired  = true;            
    } while(( !expired || (tmo == 0U)) && (status == 0U) );

    
    status = st25r3911interrupt.status & mask;
    
    st25r3911interrupt.status &= ~status;
  
    return status;
}

void rfal_rfst25r3911b::rfalPrepareTransceive( void )
{
    uint32_t maskInterrupts;
    uint8_t  reg;
    
    /*******************************************************************************/
    /* In the EMVCo mode the NRT will continue to run.                             *
     * For the clear to stop it, the EMV mode has to be disabled before            */
    st25r3911ClrRegisterBits( ST25R3911_REG_GPT_CONTROL, ST25R3911_REG_GPT_CONTROL_nrt_emv );
    
    /* Reset receive logic */
    st25r3911ExecuteCommand( ST25R3911_CMD_CLEAR_FIFO );
    
    /* Reset Rx Gain */
    st25r3911ExecuteCommand( ST25R3911_CMD_CLEAR_SQUELCH );
    
    
    /*******************************************************************************/
    /* FDT Poll                                                                    */
    /*******************************************************************************/
    if( rfalIsModePassiveComm( gRFAL.mode ) )  /* Passive Comms */
    {
       /* In Passive communications General Purpose Timer is used to measure FDT Poll */
       if( gRFAL.timings.FDTPoll != RFAL_TIMING_NONE )
       {
           /* Configure GPT to start at RX end */
           st25r3911StartGPTimer_8fcs( (uint16_t)rfalConv1fcTo8fc( MIN( gRFAL.timings.FDTPoll, (gRFAL.timings.FDTPoll - RFAL_FDT_POLL_ADJUSTMENT) ) ), ST25R3911_REG_GPT_CONTROL_gptc_erx );
       }
    }
    
    
    /*******************************************************************************/
    /* Execute Pre Transceive Callback                                             */
    /*******************************************************************************/
    if( gRFAL.callbacks.preTxRx != NULL )
    {
        gRFAL.callbacks.preTxRx();
    }
    /*******************************************************************************/
    
    maskInterrupts = ( ST25R3911_IRQ_MASK_FWL  | ST25R3911_IRQ_MASK_TXE  |
                       ST25R3911_IRQ_MASK_RXS  | ST25R3911_IRQ_MASK_RXE  |
                       ST25R3911_IRQ_MASK_FWL  | ST25R3911_IRQ_MASK_NRE  |
                       ST25R3911_IRQ_MASK_PAR  | ST25R3911_IRQ_MASK_CRC  |
                       ST25R3911_IRQ_MASK_ERR1 | ST25R3911_IRQ_MASK_ERR2  );
    
    
    /*******************************************************************************/
    /* Transceive flags                                                            */
    /*******************************************************************************/
    
    reg = (ST25R3911_REG_ISO14443A_NFC_no_tx_par_off | ST25R3911_REG_ISO14443A_NFC_no_rx_par_off | ST25R3911_REG_ISO14443A_NFC_nfc_f0_off);
    
    /* Check if NFCIP1 mode is to be enabled */
    if( (gRFAL.TxRx.ctx.flags & (uint8_t)RFAL_TXRX_FLAGS_NFCIP1_ON) != 0U )
    {
        reg |= ST25R3911_REG_ISO14443A_NFC_nfc_f0;
    }
    
    /* Check if Parity check is to be skipped and to keep the parity + CRC bits in FIFO */
    if( (gRFAL.TxRx.ctx.flags & (uint8_t)RFAL_TXRX_FLAGS_PAR_RX_KEEP) != 0U )
    {
        reg |= ST25R3911_REG_ISO14443A_NFC_no_rx_par;
    }

    /* Check if automatic Parity bits is to be disabled */
    if( (gRFAL.TxRx.ctx.flags & (uint8_t)RFAL_TXRX_FLAGS_PAR_TX_NONE) != 0U )
    {
        reg |= ST25R3911_REG_ISO14443A_NFC_no_tx_par;
    }
    
    /* Apply current TxRx flags on ISO14443A and NFC 106kb/s Settings Register */
    st25r3911ChangeRegisterBits( ST25R3911_REG_ISO14443A_NFC, (ST25R3911_REG_ISO14443A_NFC_no_tx_par | ST25R3911_REG_ISO14443A_NFC_no_rx_par | ST25R3911_REG_ISO14443A_NFC_nfc_f0), reg );
    
    
    /* Check if AGC is to be disabled */
    if( (gRFAL.TxRx.ctx.flags & (uint8_t)RFAL_TXRX_FLAGS_AGC_OFF) != 0U )
    {
        st25r3911ClrRegisterBits( ST25R3911_REG_RX_CONF2, ST25R3911_REG_RX_CONF2_agc_en );
    }
    else
    {
        st25r3911SetRegisterBits( ST25R3911_REG_RX_CONF2, ST25R3911_REG_RX_CONF2_agc_en );
    }
    /*******************************************************************************/
    
    

    /*******************************************************************************/
    /* EMVCo NRT mode                                                              */
    /*******************************************************************************/
    if( gRFAL.conf.eHandling == RFAL_ERRORHANDLING_EMVCO )
    {
        st25r3911SetRegisterBits( ST25R3911_REG_GPT_CONTROL, ST25R3911_REG_GPT_CONTROL_nrt_emv );
    }
    else
    {
        st25r3911ClrRegisterBits( ST25R3911_REG_GPT_CONTROL, ST25R3911_REG_GPT_CONTROL_nrt_emv );
    }
    /*******************************************************************************/
    
    
    
    /* In Active comms enable also External Field interrupts  */
    if( rfalIsModeActiveComm( gRFAL.mode ) )
    {
        maskInterrupts |= ( ST25R3911_IRQ_MASK_EOF  | ST25R3911_IRQ_MASK_EON | ST25R3911_IRQ_MASK_CAT | ST25R3911_IRQ_MASK_CAC );
    }
    
    
    /*******************************************************************************/
    /* clear and enable these interrupts */
    st25r3911GetInterrupt( maskInterrupts );
    st25r3911EnableInterrupts( maskInterrupts );
    
    /* Clear FIFO status local copy */
    rfalFIFOStatusClear();
}
