#include "rfal_rfst25r3911b.h"
#include "rfal_nfcb.h"

bool timerIsExpired( uint32_t timer );
static rfalIsoDep gIsoDep;    /*!< ISO-DEP Module instance               */

/*FUNCTION */

static void isoDepClearCounters( void );
static void rfalIsoDepInitialize( void );

ReturnCode rfal_rfst25r3911b::rfalIsoDepDeselect( void )
{
    ReturnCode ret;
    uint32_t   cntRerun;
    bool       dummyB;
    
    /*******************************************************************************/
    /* Check if  rx parameters have been set before, otherwise use global variable *
     * To cope with a Deselect after RATS\ATTRIB without any I-Block exchanged     */
    if( (gIsoDep.rxLen == NULL) || (gIsoDep.rxBuf == NULL) )
    {
        /* Using local vars would be safe as rfalIsoDepInitialize will clear the   *
         * reference to local vars before exiting (no EXIT_ON_ERR),                *
         * but MISRA 18.6 3217 would be still be flagged. Using static variables   */
        gIsoDep.rxLen       = &gIsoDep.ctrlRxLen;
        gIsoDep.rxBuf       = gIsoDep.ctrlRxBuf;
        
        gIsoDep.rxBufLen    = ISODEP_CONTROLMSG_BUF_LEN;
        gIsoDep.rxBufInfPos = (RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN);
        gIsoDep.txBufInfPos = (RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN);
    }
    
    
    /*******************************************************************************/
    /* The Deselect process is being done blocking, Digital 1.0 - 13.2.7.1 MUST wait response and retry*/
    /* Set the maximum reruns while we will wait for a response */
    cntRerun = ISODEP_MAX_RERUNS;
    
    /* Send DSL request and run protocol until get a response, error or "timeout" */    
    EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_S_DSL, RFAL_ISODEP_NO_PARAM ));
    do{
        ret = isoDepDataExchangePCD( gIsoDep.rxLen, &dummyB );
        rfalWorker();
    }
    while( ((cntRerun--) != 0U) && (ret == ERR_BUSY) );
        
    rfalIsoDepInitialize();
    return ((cntRerun == 0U) ? ERR_TIMEOUT : ret);
}

ReturnCode rfal_rfst25r3911b::isoDepHandleControlMsg( rfalIsoDepControlMsg controlMsg, uint8_t param )
{
    uint8_t  pcb;   
    uint8_t  ctrlMsgBuf[ISODEP_CONTROLMSG_BUF_LEN];
    uint8_t  infLen;
    uint32_t fwtTemp;
    
    infLen  = 0;
    fwtTemp = (gIsoDep.fwt + gIsoDep.dFwt);
    memset( ctrlMsgBuf, 0x00, ISODEP_CONTROLMSG_BUF_LEN );
    
    switch( controlMsg )
    {
        /*******************************************************************************/
        case ISODEP_R_ACK:
            
            if( gIsoDep.cntRRetrys++ > gIsoDep.maxRetriesR )
            {
                return ERR_PROTO;
            }
            
            pcb = isoDep_PCBRACK( gIsoDep.blockNumber );
            break;
            
        /*******************************************************************************/
        case ISODEP_R_NAK:
            if( gIsoDep.cntRRetrys++ > gIsoDep.maxRetriesR )
            {
                return ERR_TIMEOUT;
            }
            
            pcb = isoDep_PCBRNAK( gIsoDep.blockNumber );            
            break;
            
        /*******************************************************************************/
        case ISODEP_S_WTX:
            if( gIsoDep.cntSRetrys++ > gIsoDep.maxRetriesS )
            {
                return ERR_PROTO;
            }
            
            /* Check if WTXM is valid */
            if( ! isoDep_isWTXMValid(param) )
            {
                return ERR_PROTO;
            }
            
            if( gIsoDep.role == ISODEP_ROLE_PCD )
            {
                /* Calculate temp Wait Time eXtension */ 
                fwtTemp = (gIsoDep.fwt * param);
                fwtTemp = MIN( RFAL_ISODEP_MAX_FWT, fwtTemp );
                fwtTemp += gIsoDep.dFwt;
            }
            
            pcb = ISODEP_PCB_SWTX;
            ctrlMsgBuf[ RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN + infLen++] = param;
            break;
            
        /*******************************************************************************/
        case ISODEP_S_DSL:
            if( gIsoDep.cntSRetrys++ > gIsoDep.maxRetriesS )
            {
                return ERR_PROTO;
            }
            
            if( gIsoDep.role == ISODEP_ROLE_PCD )
            {
                /* Digital 1.0 - 13.2.7.3 Poller must wait fwtDEACTIVATION */
                fwtTemp = ISODEP_FWT_DEACTIVATION;
                gIsoDep.state = ISODEP_ST_PCD_WAIT_DSL;
            }
            pcb = ISODEP_PCB_SDSL;
            break;
        
        /*******************************************************************************/
        default:
            return ERR_INTERNAL;
    }
    
    return isoDepTx( pcb, ctrlMsgBuf, &ctrlMsgBuf[RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN], infLen, fwtTemp );
}

ReturnCode rfal_rfst25r3911b::isoDepDataExchangePCD( uint16_t *outActRxLen, bool *outIsChaining )
{
    ReturnCode ret;
    uint8_t    rxPCB;
    
    /* Check out parameters */
    if( (outActRxLen == NULL) || (outIsChaining == NULL) )
    {
        return ERR_PARAM;
    }    
    
    *outIsChaining = false;
        
    /* Calculate header required and check if the buffers InfPositions are suitable */    
    gIsoDep.hdrLen = RFAL_ISODEP_PCB_LEN;
    if (gIsoDep.did != RFAL_ISODEP_NO_DID)  { gIsoDep.hdrLen  += RFAL_ISODEP_DID_LEN;  }
    if (gIsoDep.nad != RFAL_ISODEP_NO_NAD)  { gIsoDep.hdrLen  += RFAL_ISODEP_NAD_LEN;  }
    
    /* check if there is enough space before the infPos to append ISO-DEP headers on rx and tx */
    if( (gIsoDep.rxBufInfPos < gIsoDep.hdrLen) || (gIsoDep.txBufInfPos < gIsoDep.hdrLen) )
    {
        return ERR_PARAM;
    }
    
    /*******************************************************************************/
    /* Wait until SFGT has been fulfilled (as a PCD) */
    if(gIsoDep.SFGTTimer != 0U)
    {
        if( !isoDepTimerisExpired( gIsoDep.SFGTTimer ) )
        {
            return ERR_BUSY;
        }
    }
    /* Once done, clear SFGT timer */
    gIsoDep.SFGTTimer = 0;
    
    
    /*******************************************************************************/
    switch( gIsoDep.state )
    {
        /*******************************************************************************/
        case ISODEP_ST_IDLE:
            return ERR_NONE;
        
        /*******************************************************************************/
        case ISODEP_ST_PCD_TX:
            ret = isoDepTx( isoDep_PCBIBlock( gIsoDep.blockNumber ), gIsoDep.txBuf, &gIsoDep.txBuf[gIsoDep.txBufInfPos], gIsoDep.txBufLen, (gIsoDep.fwt + gIsoDep.dFwt) );
            switch( ret )
            {
              case ERR_NONE:
                  gIsoDep.state = ISODEP_ST_PCD_RX;
                  break;
              
              default:
                  return ret;
            }
            /* fall through */
          
        /*******************************************************************************/
        case ISODEP_ST_PCD_WAIT_DSL:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
        case ISODEP_ST_PCD_RX:
                      
            ret = rfalGetTransceiveStatus();
            switch( ret )
            {
                /* Data rcvd with error or timeout -> Send R-NAK */
                case ERR_TIMEOUT:
                case ERR_CRC:
                case ERR_PAR:
                case ERR_FRAMING:          /* added to handle test cases scenario TC_POL_NFCB_T4AT_BI_82_x_y & TC_POL_NFCB_T4BT_BI_82_x_y */
                case ERR_INCOMPLETE_BYTE:  /* added to handle test cases scenario TC_POL_NFCB_T4AT_BI_82_x_y & TC_POL_NFCB_T4BT_BI_82_x_y  */
                    
                    if( gIsoDep.isRxChaining )
                    {   /* Rule 5 - In PICC chaining when a invalid/timeout occurs -> R-ACK */                        
                        EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_R_ACK, RFAL_ISODEP_NO_PARAM ) );
                    }
                    else if( gIsoDep.state == ISODEP_ST_PCD_WAIT_DSL )
                    {   /* Rule 8 - If s-Deselect response fails MAY retransmit */
                        EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_S_DSL, RFAL_ISODEP_NO_PARAM ) );
                    }
                    else
                    {   /* Rule 4 - When a invalid block or timeout occurs -> R-NACK */
                        EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_R_NAK, RFAL_ISODEP_NO_PARAM ) );
                    }
                    return ERR_BUSY;
                    
                case ERR_NONE:
                    break;
                    
                case ERR_BUSY:
                    return ERR_BUSY;  /* Debug purposes */
                    
                default:
                    return ret;
            }
            
            /*******************************************************************************/
            /* No error, process incoming msg                                              */
            /*******************************************************************************/
            
            (*outActRxLen) = rfalConvBitsToBytes( *outActRxLen );
            
            
            /* Check rcvd msg length, cannot be less then the expected header */
            if( ((*outActRxLen) < gIsoDep.hdrLen) || ((*outActRxLen) >= gIsoDep.ourFsx) )
            {
                return ERR_PROTO;
            }
            
            /* Grab rcvd PCB */
            rxPCB = gIsoDep.rxBuf[ ISODEP_PCB_POS ];
            
            
            /* EMVCo doesn't allow usage of for CID or NAD   EMVCo 2.6 TAble 10.2 */
            if( (gIsoDep.compMode == RFAL_COMPLIANCE_MODE_EMV) && ( isoDep_PCBhasDID(rxPCB) || isoDep_PCBhasNAD(rxPCB)) )
            {
                return ERR_PROTO;
            }
            
            /* If we are expecting DID, check if PCB signals its presence and if device ID match*/
            if( (gIsoDep.did != RFAL_ISODEP_NO_DID) && ( !isoDep_PCBhasDID(rxPCB) || (gIsoDep.did != gIsoDep.rxBuf[ ISODEP_DID_POS ])) )
            {
                return ERR_PROTO;
            }
            
            
            /*******************************************************************************/
            /* Process S-Block                                                             */
            /*******************************************************************************/
            if( isoDep_PCBisSBlock(rxPCB) )
            {
                /* Check if is a Wait Time eXtension */
                if( isoDep_PCBisSWTX(rxPCB) )
                {
                    /* Rule 3 - respond to S-block: get 1st INF byte S(STW): Power + WTXM */
                    EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_S_WTX, isoDep_GetWTXM(gIsoDep.rxBuf[gIsoDep.hdrLen]) ) );                    
                    return ERR_BUSY;
                }
                
                /* Check if is a deselect response */
                if( isoDep_PCBisSDeselect(rxPCB) )
                {
                    if( gIsoDep.state == ISODEP_ST_PCD_WAIT_DSL )
                    {
                        rfalIsoDepInitialize();         /* Session finished reInit vars */
                        return ERR_NONE;
                    }
                    
                    /* Deselect response not expected  */
                    /* fall through to PROTO error */
                }
                /* Unexpected S-Block */
                return ERR_PROTO;
            }
            
            /*******************************************************************************/
            /* Process R-Block                                                             */
            /*******************************************************************************/
            else if( isoDep_PCBisRBlock(rxPCB) )
            {
                if( isoDep_PCBisRACK(rxPCB) )                            /* Check if is a R-ACK */
                {
                    if( isoDep_GetBN(rxPCB) == gIsoDep.blockNumber )     /* Expected block number  */
                    {
                        /* Rule B - ACK with expected bn -> Increment block number */
                        gIsoDep.blockNumber = isoDep_PCBNextBN( gIsoDep.blockNumber );
                                                
                        /* R-ACK only allowed when PCD chaining */
                        if( !gIsoDep.isTxChaining )
                        {
                            return ERR_PROTO;
                        }
                        
                        /* Rule 7 - Chaining transaction done, continue chaining */
                        isoDepClearCounters();
                        return ERR_NONE;  /* This block has been transmitted */
                    }
                    else
                    {
                        /* Rule 6 - R-ACK with wrong block number retransmit */
                        if( gIsoDep.cntIRetrys++ < gIsoDep.maxRetriesI )
                        {
                            gIsoDep.cntRRetrys = 0;            /* Clear R counter only */
                            gIsoDep.state = ISODEP_ST_PCD_TX;
                            return ERR_BUSY;
                        }
                        return ERR_PROTO;
                    }
                }
                else  /* Unexcpected R-Block */
                {
                    return ERR_PROTO;
                }
            }
            
            /*******************************************************************************/
            /* Process I-Block                                                             */
            /*******************************************************************************/
            else if( isoDep_PCBisIBlock(rxPCB) )
            {
                /*******************************************************************************/
                /* is PICC performing chaining                                                 */
                if( isoDep_PCBisChaining(rxPCB) )
                {
                    gIsoDep.isRxChaining = true;
                    *outIsChaining       = true;
                    
                    if( isoDep_GetBN(rxPCB) == gIsoDep.blockNumber )
                    {
                        /* Rule B - ACK with correct block number -> Increase Block number */
                        isoDep_ToggleBN( gIsoDep.blockNumber );
                        
                        isoDepClearCounters();  /* Clear counters in case R counter is already at max */
                        
                        /* Rule 2 - Send ACK */
                        EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_R_ACK, RFAL_ISODEP_NO_PARAM ) );
                        
                        /* Received I-Block with chaining, send current data to DH */
                        
                        /* remove ISO DEP header, check is necessary to move the INF data on the buffer */
                        *outActRxLen -= gIsoDep.hdrLen;
                        if( (gIsoDep.hdrLen != gIsoDep.rxBufInfPos) && (*outActRxLen > 0U) )
                        {
                            memmove( &gIsoDep.rxBuf[gIsoDep.rxBufInfPos], &gIsoDep.rxBuf[gIsoDep.hdrLen], *outActRxLen );
                        }
                        
                        isoDepClearCounters();
                        return ERR_AGAIN;       /* Send Again signalling to run again, but some chaining data has arrived */
                    }
                    else
                    {
                        /* Rule 5 - PICC chaining invalid I-Block -> R-ACK */
                        EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_R_ACK, RFAL_ISODEP_NO_PARAM ) );                        
                    }
                    return ERR_BUSY;
                }
                
                gIsoDep.isRxChaining = false; /* clear PICC chaining flag */                
                
                if( isoDep_GetBN(rxPCB) == gIsoDep.blockNumber )
                {
                    /* Rule B - I-Block with correct block number -> Increase Block number */
                    isoDep_ToggleBN( gIsoDep.blockNumber );
                    
                    /* I-Block transaction done successfully */
                    
                    /* remove ISO DEP header, check is necessary to move the INF data on the buffer */
                    *outActRxLen -= gIsoDep.hdrLen;
                    if( (gIsoDep.hdrLen != gIsoDep.rxBufInfPos) && (*outActRxLen > 0U) )
                    {
                        memmove( &gIsoDep.rxBuf[gIsoDep.rxBufInfPos], &gIsoDep.rxBuf[gIsoDep.hdrLen], *outActRxLen );
                    }
                    
                    gIsoDep.state = ISODEP_ST_IDLE;
                    isoDepClearCounters();
                    return ERR_NONE;
                }
                else
                {
                    if( (gIsoDep.compMode != RFAL_COMPLIANCE_MODE_ISO) )
                    {
                        /* Invalid Block (not chaining) -> Raise error   Digital 1.1  15.2.6.4   EMVCo 2.6  10.3.5.4 */
                        return ERR_PROTO;
                    }

                    /* Rule 4 - Invalid Block -> R-NAK */
                    EXIT_ON_ERR( ret, isoDepHandleControlMsg( ISODEP_R_NAK, RFAL_ISODEP_NO_PARAM ) );
                    return ERR_BUSY;
                }
            }
            else /* not S/R/I - Block */
            {
                return ERR_PROTO;
            }
            /* fall through */
          
        /*******************************************************************************/
        default:               /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            /* MISRA 16.4: no empty default (comment will suffice) */
            break;
    }
    
    return ERR_INTERNAL;
}

static void isoDepClearCounters( void )
{
    gIsoDep.cntIRetrys   = 0;
    gIsoDep.cntRRetrys   = 0;
    gIsoDep.cntSRetrys   = 0;
}

static void rfalIsoDepInitialize( void )
{
    gIsoDep.state        = ISODEP_ST_IDLE;
    gIsoDep.role         = ISODEP_ROLE_PCD;
    gIsoDep.did          = RFAL_ISODEP_NO_DID;
    gIsoDep.nad          = RFAL_ISODEP_NO_NAD;
    gIsoDep.blockNumber  = 0;
    gIsoDep.isTxChaining = false;
    gIsoDep.isRxChaining = false;
    gIsoDep.lastDID00    = false;
    gIsoDep.lastPCB      = ISODEP_PCB_INVALID;
    gIsoDep.fsx          = (uint16_t)RFAL_ISODEP_FSX_16;
    gIsoDep.ourFsx       = (uint16_t)RFAL_ISODEP_FSX_16;
    gIsoDep.hdrLen       = RFAL_ISODEP_PCB_LEN;
    
    gIsoDep.rxLen        = NULL;
    gIsoDep.rxBuf        = NULL;
    
    gIsoDep.isTxPending  = false;
    gIsoDep.isWait4WTX   = false;
    
    gIsoDep.compMode       = RFAL_COMPLIANCE_MODE_NFC;
    gIsoDep.maxRetriesR    = RFAL_ISODEP_MAX_R_RETRYS;
    gIsoDep.maxRetriesS    = RFAL_ISODEP_MAX_S_RETRYS;
    gIsoDep.maxRetriesI    = RFAL_ISODEP_MAX_I_RETRYS;
    gIsoDep.maxRetriesRATS = RFAL_ISODEP_RATS_RETRIES;
    
    isoDepClearCounters();
}

ReturnCode rfal_rfst25r3911b::isoDepTx( uint8_t pcb, const uint8_t* txBuf, uint8_t *infBuf, uint16_t infLen, uint32_t fwt )
{
    uint8_t    *txBlock;
    uint16_t   txBufLen;
    uint8_t    computedPcb;

    
    txBlock         = infBuf;                      /* Point to beginning of the INF, and go backwards     */
    gIsoDep.lastPCB = pcb;                         /* Store the last PCB sent                             */
    
    
    if ( infLen > 0U )
    {
        if ( ((uint32_t)infBuf - (uint32_t)txBuf) < gIsoDep.hdrLen ) /* Check that we can fit the header in the given space */
        {
            return ERR_NOMEM;
        }
    }
    
    
    /*******************************************************************************/
    /* Compute optional PCB bits */
    computedPcb = pcb;
    if ((gIsoDep.did != RFAL_ISODEP_NO_DID) || ((gIsoDep.did == RFAL_ISODEP_DID_00) && gIsoDep.lastDID00) ) {   computedPcb |= ISODEP_PCB_DID_BIT;            }
    if (gIsoDep.nad != RFAL_ISODEP_NO_NAD)                                                                  {   computedPcb |= ISODEP_PCB_NAD_BIT;            }
    if ((gIsoDep.isTxChaining) && (isoDep_PCBisIBlock(computedPcb)) )                                       {   computedPcb |= ISODEP_PCB_CHAINING_BIT;       } 

    
    /*******************************************************************************/
    /* Compute Payload on the given txBuf, start by the PCB | DID | NAD | before INF */
    
    if (gIsoDep.nad != RFAL_ISODEP_NO_NAD) 
    {
        *(--txBlock) = gIsoDep.nad;                /* NAD is optional */
    }
    
    if ( (gIsoDep.did != RFAL_ISODEP_NO_DID) || ((gIsoDep.did == RFAL_ISODEP_DID_00) && gIsoDep.lastDID00) ) 
    {
        *(--txBlock)  = gIsoDep.did;               /* DID is optional */
    }
    
    *(--txBlock)      = computedPcb;               /* PCB always present */
    
    txBufLen = (infLen + (uint16_t)((uint32_t)infBuf - (uint32_t)txBlock)); /* Calculate overall buffer size */
    
    if ( txBufLen > (gIsoDep.fsx - ISODEP_CRC_LEN) )                        /* Check if msg length violates the maximum frame size FSC */
    {
        return ERR_NOTSUPP;
    }
        
    return rfalTransceiveBlockingTx( txBlock, txBufLen, gIsoDep.rxBuf, gIsoDep.rxBufLen, gIsoDep.rxLen, RFAL_TXRX_FLAGS_DEFAULT, ((gIsoDep.role == ISODEP_ROLE_PICC) ? RFAL_FWT_NONE : fwt ) );
}

bool timerIsExpired( uint32_t timer )
{
  uint32_t uDiff;
  int32_t sDiff;
  
  uDiff = (timer - millis());   /* Calculate the diff between the timers */
  sDiff = uDiff;                            /* Convert the diff to a signed var      */
  /* Having done this has two side effects: 
   * 1) all differences smaller than -(2^31) ms (~25d) will become positive
   *    Signaling not expired: acceptable!
   * 2) Time roll-over case will be handled correctly: super!
   */
  
  /* Check if the given timer has expired already */
  if( sDiff < 0 )
  {
    return true;
  }
  
  return false;
}

ReturnCode rfal_rfst25r3911b::rfalIsoDepStartTransceive( rfalIsoDepTxRxParam param )
{
    gIsoDep.txBuf        = param.txBuf->prologue;
    gIsoDep.txBufInfPos  = (uint8_t)((uint32_t)param.txBuf->inf - (uint32_t)param.txBuf->prologue);
    gIsoDep.txBufLen     = param.txBufLen;
    gIsoDep.isTxChaining = param.isTxChaining;
    
    gIsoDep.rxBuf        = param.rxBuf->prologue;
    gIsoDep.rxBufInfPos  = (uint8_t)((uint32_t)param.rxBuf->inf - (uint32_t)param.rxBuf->prologue);
    gIsoDep.rxBufLen     = sizeof(rfalIsoDepBufFormat);
    
    gIsoDep.rxLen        = param.rxLen;
    gIsoDep.rxChaining   = param.isRxChaining;
    
    
    gIsoDep.fwt          = param.FWT;
    gIsoDep.dFwt         = param.dFWT;
    gIsoDep.fsx          = param.FSx;
    gIsoDep.did          = param.DID;
    
    /* Only change the FSx from activation if no to Keep */
    gIsoDep.ourFsx = (( param.ourFSx != RFAL_ISODEP_FSX_KEEP ) ? param.ourFSx : gIsoDep.ourFsx);
    
    /* Clear inner control params for next dataExchange */
    gIsoDep.isRxChaining  = false;
    isoDepClearCounters();
    
    if(gIsoDep.role == ISODEP_ROLE_PICC)
    {
       if(gIsoDep.txBufLen > 0U)
       {
           /* Ensure that an RTOX Ack is not being expected at moment */
           if( !gIsoDep.isWait4WTX )
           {
               gIsoDep.state = ISODEP_ST_PICC_TX;
               return ERR_NONE;
           }
           else
           {
               /* If RTOX Ack is expected, signal a pending Tx to be transmitted right after */
               gIsoDep.isTxPending = true;
           }
       }
       
       /* Digital 1.1  15.2.5.1 The first block SHALL be sent by the Reader/Writer */
       gIsoDep.state = ISODEP_ST_PICC_RX;
       return ERR_NONE;
    }
    
    gIsoDep.state = ISODEP_ST_PCD_TX;
    return ERR_NONE;
}

ReturnCode rfal_rfst25r3911b::rfalIsoDepGetTransceiveStatus( void )
{
    ReturnCode err;
    
    if( gIsoDep.role == ISODEP_ROLE_PICC)
    {
#if RFAL_FEATURE_ISO_DEP_LISTEN
        err = isoDepDataExchangePICC();
#else
        err = ERR_NOTSUPP;
#endif /* RFAL_FEATURE_ISO_DEP_LISTEN */
    }
    else
    {
#if RFAL_FEATURE_ISO_DEP_POLL
        err = isoDepDataExchangePCD( gIsoDep.rxLen, gIsoDep.rxChaining );
#else
        err = ERR_NOTSUPP;
#endif /* RFAL_FEATURE_ISO_DEP_POLL */
    }

    return err;
}
