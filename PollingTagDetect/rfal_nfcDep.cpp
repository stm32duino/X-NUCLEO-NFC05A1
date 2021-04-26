#include "rfal_nfc.h"

static void nfcipTimerStart(uint16_t timer, uint16_t time );

ReturnCode rfal_nfc::rfalNfcDepRLS( void )
{   
    ReturnCode ret;
    uint8_t    txBuf[RFAL_NFCDEP_HEADER_PAD + NFCIP_RLSREQ_LEN];
    uint8_t    rxBuf[NFCIP_RLSRES_LEN];    
    uint8_t    rxMsgIt;
    uint16_t   rxLen = 0;
    
    if ( gNfcip.cfg.role == RFAL_NFCDEP_ROLE_TARGET )  /* Target has no release procedure */
    {
        return ERR_NONE;
    }
        
    /* Repeating a RLS REQ is optional, not doing it */
    EXIT_ON_ERR( ret, nfcipTxRx( NFCIP_CMD_RLS_REQ, txBuf, nfcipRWTActivation(), NULL, 0, rxBuf, RFAL_NFCDEP_ATRRES_MAX_LEN, &rxLen  ) );
    
    /*******************************************************************************/
    rxMsgIt = 0;
       
    if( rxBuf[rxMsgIt++] < NFCIP_RLSRES_MIN )             /* Checking length: LEN + RLS_RES */
    {
        return ERR_PROTO;
    }
        
    if( rxBuf[rxMsgIt++] != NFCIP_RES )                   /* Checking if is a response      */
    {
        return ERR_PROTO;
    }
    
    if( rxBuf[rxMsgIt++] != (uint8_t)NFCIP_CMD_RLS_RES )  /* Checking if is RLS RES         */
    {
        return ERR_PROTO;
    }
     
    if( gNfcip.cfg.did != RFAL_NFCDEP_DID_NO ) 
    {
        if ( rxBuf[rxMsgIt++] != gNfcip.cfg.did ) 
        {
            return ERR_PROTO;
        }
    }
    
    return ERR_NONE;
}

ReturnCode rfal_nfc::nfcipTxRx( rfalNfcDepCmd cmd, uint8_t* txBuf, uint32_t fwt, uint8_t* paylBuf, uint8_t paylBufLen, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t *rxActLen )
{
    ReturnCode ret;
    
    if( (cmd == NFCIP_CMD_DEP_REQ) || (cmd == NFCIP_CMD_DEP_RES) ) /* this method cannot be used for DEPs */
    {
        return ERR_PARAM;
    }
    
    /* Assign the global params for this TxRx */
    gNfcip.rxBuf       = rxBuf;
    gNfcip.rxBufLen    = rxBufLen;
    gNfcip.rxRcvdLen   = rxActLen;
    
    
    /*******************************************************************************/
  /* Transmission                                                                */
  /*******************************************************************************/
    if(txBuf != NULL)                                              /* if nothing to Tx, just do Rx */               
    {
        EXIT_ON_ERR( ret, nfcipTx( cmd, txBuf, paylBuf, paylBufLen, 0, fwt ) ); 
    }
    
    /*******************************************************************************/
  /* Reception                                                                   */
  /*******************************************************************************/
    ret = nfcipDataRx( true );
    if( ret != ERR_NONE )
    {
        return ret;
    }
    
    /*******************************************************************************/    
    *rxActLen = *rxBuf;                                         /* Use LEN byte instead due to with/without CRC modes */
    return ERR_NONE;                                            /* Tx and Rx completed successfully                   */
}

ReturnCode rfal_nfc::nfcipTx( rfalNfcDepCmd cmd, uint8_t* txBuf, uint8_t *paylBuf, uint16_t paylLen, uint8_t pfbData, uint32_t fwt )
{
    uint16_t txBufIt;
    uint8_t *txBlock;
    uint8_t *payloadBuf;
    uint8_t  pfb;
    
   
    if( txBuf == NULL )
    {
        return ERR_PARAM;
    }
    

    payloadBuf = paylBuf;                                               /* MISRA 17.8: Use intermediate variable */
    
    if( (paylLen == 0U) || (payloadBuf == NULL) )
    {
        payloadBuf = (uint8_t*) &txBuf[RFAL_NFCDEP_DEPREQ_HEADER_LEN];  /* If not a DEP (no Data) ensure enough space for header */
    }
    
    
    txBufIt  = 0;
    pfb      = pfbData;                                                 /* MISRA 17.8: Use intermediate variable */
    
    txBlock  = payloadBuf;                                              /* Point to beginning of the Data, and go backwards     */    
        
    
    gNfcip.lastCmd = (uint8_t)cmd;                                      /* Store last cmd sent    */
    gNfcip.lastPFB = NFCIP_PFB_INVALID;                                 /* Reset last pfb sent    */
    
    /*******************************************************************************/
    /* Compute outgoing NFCIP message                                              */
    /*******************************************************************************/
    switch( cmd )
    {
        /*******************************************************************************/
        case NFCIP_CMD_ATR_RES:
        case NFCIP_CMD_ATR_REQ:
            
            rfalNfcDepSetNFCID( payloadBuf, gNfcip.cfg.nfcid, gNfcip.cfg.nfcidLen );    /* NFCID */
            txBufIt += RFAL_NFCDEP_NFCID3_LEN;
            
            payloadBuf[txBufIt++] = gNfcip.cfg.did;                                     /* DID   */
            payloadBuf[txBufIt++] = gNfcip.cfg.bs;                                      /* BS    */
            payloadBuf[txBufIt++] = gNfcip.cfg.br;                                      /* BR    */
            
            if( cmd == NFCIP_CMD_ATR_RES )
            {
                payloadBuf[txBufIt++] = gNfcip.cfg.to;                                  /* ATR_RES[ TO ] */
            }
                                    
            if( gNfcip.cfg.gbLen > 0U)
            {
                payloadBuf[txBufIt++] = nfcip_PPwGB( gNfcip.cfg.lr );                   /* PP signalling GB  */
                memcpy( &payloadBuf[txBufIt], gNfcip.cfg.gb, gNfcip.cfg.gbLen );     /* set General Bytes */
                txBufIt += gNfcip.cfg.gbLen;
            }
            else
            {
                payloadBuf[txBufIt++] = rfalNfcDepLR2PP( gNfcip.cfg.lr );               /* PP without GB     */
            }
            
            if( (txBufIt + RFAL_NFCDEP_CMDTYPE_LEN + RFAL_NFCDEP_CMD_LEN) > RFAL_NFCDEP_ATRREQ_MAX_LEN )   /* Check max ATR length (ATR_REQ = ATR_RES)*/
            {
                return ERR_PARAM;
            }
            break;
            
        /*******************************************************************************/
        case NFCIP_CMD_WUP_REQ:                               /* ISO 18092 - 12.5.2.1 */
            
            rfalNfcDepSetNFCID( (payloadBuf), gNfcip.cfg.nfcid, gNfcip.cfg.nfcidLen );   /* NFCID */
            txBufIt += RFAL_NFCDEP_NFCID3_LEN;
            
            *(--txBlock) = gNfcip.cfg.did;                                               /* DID   */
            break;
                    
        /*******************************************************************************/
        case NFCIP_CMD_WUP_RES:                               /* ISO 18092 - 12.5.2.2 */
        case NFCIP_CMD_PSL_REQ:
        case NFCIP_CMD_PSL_RES:
            
            *(--txBlock) = gNfcip.cfg.did;                                               /* DID   */
            break;
            
        /*******************************************************************************/
        case NFCIP_CMD_RLS_REQ:
        case NFCIP_CMD_RLS_RES:
        case NFCIP_CMD_DSL_REQ:
        case NFCIP_CMD_DSL_RES:
            
            /* Digital 1.0 - 14.8.1.1 & 14.9.1.1 & 14.10.1.1 Only add DID if not 0 */
            if( gNfcip.cfg.did != RFAL_NFCDEP_DID_NO )
            {
                *(--txBlock) = gNfcip.cfg.did;                                           /* DID   */
            }
            break;
            
        /*******************************************************************************/
        case NFCIP_CMD_DEP_REQ:
        case NFCIP_CMD_DEP_RES:
            
            /* Compute optional PFB bits */
            if (gNfcip.cfg.did != RFAL_NFCDEP_DID_NO)                {   pfb |= NFCIP_PFB_DID_BIT;       }
            if (gNfcip.cfg.nad != RFAL_NFCDEP_NAD_NO)                {   pfb |= NFCIP_PFB_NAD_BIT;       }
            if ((gNfcip.isTxChaining) && (nfcip_PFBisIPDU(pfb)) )    {   pfb |= NFCIP_PFB_MI_BIT;        }
            
            /* Store PFB for future handling */
            gNfcip.lastPFB       = pfb;                                                  /* store PFB sent */
            
            if( !nfcip_PFBisSATN(pfb) )
            {
                gNfcip.lastPFBnATN   = pfb;                                              /* store last PFB different then ATN */
            }
            
            
            /* Add NAD if it is to be supported */
            if( gNfcip.cfg.nad != RFAL_NFCDEP_NAD_NO )      
            {
                *(--txBlock) = gNfcip.cfg.nad;                                           /* NAD   */
            }
            
            /* Digital 1.0 - 14.8.1.1 & 14.8.1.1 Only add DID if not 0 */
            if( gNfcip.cfg.did != RFAL_NFCDEP_DID_NO )
            {
                *(--txBlock) = gNfcip.cfg.did;                                           /* DID   */
            }
            
            *(--txBlock) = pfb;                                                          /* PFB */
                        
            
            /* NCI 1.0 - Check if Empty frames are allowed */
            if( (paylLen == 0U) && nfcipIsEmptyDEPDisabled(gNfcip.cfg.oper) && nfcip_PFBisIPDU(pfb) )
            {
                return ERR_PARAM;
            }
            break;

        /*******************************************************************************/
        default:
            return ERR_PARAM;
    }
    
    /*******************************************************************************/
    /* Prepend Header                                                              */
    /*******************************************************************************/    
    *(--txBlock) = (uint8_t)cmd;                                                         /* CMD     */
    *(--txBlock) = (uint8_t)( nfcipCmdIsReq(cmd) ? NFCIP_REQ : NFCIP_RES );              /* CMDType */
        
    
    txBufIt += paylLen + (uint16_t)((uint32_t)payloadBuf - (uint32_t)txBlock);           /* Calculate overall buffer size */
    
    
    if( txBufIt > gNfcip.fsc )                                                           /* Check if msg length violates the maximum payload size FSC */
    {
        return ERR_NOTSUPP;
    }
        
    /*******************************************************************************/
    return nfcipDataTx( txBlock, txBufIt, fwt );
}

ReturnCode rfal_nfc::nfcipDataRx( bool blocking )
{
    ReturnCode ret;
    
    /* Perform Rx either blocking or non-blocking */
    if( blocking )
    {
        ret = RF->rfalTransceiveBlockingRx();
    }
    else
    {
        ret = RF->rfalGetTransceiveStatus();
    }
    
    if( ret != ERR_BUSY )
    {
        if( gNfcip.rxRcvdLen != NULL )
        {
            (*gNfcip.rxRcvdLen) = rfalConvBitsToBytes( *gNfcip.rxRcvdLen );
                        
            if( (ret == ERR_NONE) && (gNfcip.rxBuf != NULL) )
            {
                /* Digital 1.1  16.4.1.3 - Length byte LEN SHALL have a value between 3 and 255 -> otherwise treat as Transmission Error *
                 *                       - Ensure that actual received and frame length do match, otherwise treat as Transmission error  */
                if( (*gNfcip.rxRcvdLen != (uint16_t)*gNfcip.rxBuf) || (*gNfcip.rxRcvdLen < RFAL_NFCDEP_LEN_MIN) || (*gNfcip.rxRcvdLen > RFAL_NFCDEP_LEN_MAX) )
                {
                    return ERR_FRAMING;
                }
            }
        }
    }
    
    return ret;
}

ReturnCode rfal_nfc::nfcipDataTx( uint8_t* txBuf, uint16_t txBufLen, uint32_t fwt )
{
   return RF->rfalTransceiveBlockingTx( txBuf, txBufLen, gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen, (RFAL_TXRX_FLAGS_DEFAULT | (uint32_t)RFAL_TXRX_FLAGS_NFCIP1_ON), ((fwt == NFCIP_NO_FWT) ? RFAL_FWT_NONE : fwt) );
}

ReturnCode rfal_nfc::rfalNfcDepStartTransceive( rfalNfcDepTxRxParam *param )
{
    rfalNfcDepDEPParams nfcDepParams;
    
    nfcDepParams.txBuf        = (uint8_t *)param->txBuf;
    nfcDepParams.txBufLen     = param->txBufLen;
    nfcDepParams.txChaining   = param->isTxChaining;
    nfcDepParams.txBufPaylPos = RFAL_NFCDEP_DEPREQ_HEADER_LEN;  /* position in txBuf where actual outgoing data is located */
    nfcDepParams.did          = RFAL_NFCDEP_DID_KEEP;
    nfcDepParams.rxBufPaylPos = RFAL_NFCDEP_DEPREQ_HEADER_LEN;
    nfcDepParams.rxBuf        = (uint8_t *)param->rxBuf;
    nfcDepParams.rxBufLen     = sizeof(rfalNfcDepBufFormat);
    nfcDepParams.fsc          = param->FSx;
    nfcDepParams.fwt          = param->FWT;
    nfcDepParams.dFwt         = param->dFWT;

    gNfcip.rxRcvdLen          = param->rxLen;
    gNfcip.isChaining         = param->isRxChaining;

    nfcipSetDEPParams(&nfcDepParams);
    
    return ERR_NONE;
}

void rfal_nfc::nfcipSetDEPParams( rfalNfcDepDEPParams *DEPParams )
{
    Serial.print( " NFCIP SetDEP() txLen: ");
    Serial.print(DEPParams->txBufLen );
    Serial.print("\r\n");
    
    gNfcip.isTxChaining = DEPParams->txChaining;
    gNfcip.txBuf        = DEPParams->txBuf;
    gNfcip.rxBuf        = DEPParams->rxBuf;
    gNfcip.txBufLen     = DEPParams->txBufLen;
    gNfcip.rxBufLen     = DEPParams->rxBufLen;
    gNfcip.txBufPaylPos = DEPParams->txBufPaylPos;
    gNfcip.rxBufPaylPos = DEPParams->rxBufPaylPos;
    
    if( DEPParams->did != RFAL_NFCDEP_DID_KEEP )
    {
        gNfcip.cfg.did  = nfcip_DIDMax( DEPParams->did );
    }
    
    gNfcip.cfg.fwt      = DEPParams->fwt;
    gNfcip.cfg.dFwt     = DEPParams->dFwt;
    gNfcip.fsc          = DEPParams->fsc;
    
    
    
    if(gNfcip.cfg.role == RFAL_NFCDEP_ROLE_TARGET)
    {
        /* If there's any data to be sent go for Tx */
        if(DEPParams->txBufLen > 0U)
        {
            /* Ensure that an RTOX Ack is not being expected at moment */
            if( !gNfcip.isWait4RTOX )
            {
                gNfcip.state = NFCIP_ST_TARG_DEP_TX;
                return;
            }
            else
            {
                /* If RTOX Ack is expected, signal a pending Tx to be transmitted right after */
                gNfcip.isTxPending = true;
                Serial.print( " NFCIP(T) Waiting RTOX, queueing outgoing DEP Block \r\n" );                
            }
        }    
    
        /*Digital 1.0  14.12.4.1 In target mode the first PDU MUST be sent by the Initiator */
        gNfcip.state = NFCIP_ST_TARG_DEP_RX;
        return;
    }

    /* New data TxRx request clear previous error counters for consecutive TxRx without reseting communication/protocol layer*/
    nfcipClearCounters();
    
    gNfcip.state = NFCIP_ST_INIT_DEP_TX;
}

void rfal_nfc::nfcipClearCounters( void )
{
    gNfcip.cntATNRetrys  = 0;
    gNfcip.cntNACKRetrys = 0;
    gNfcip.cntTORetrys   = 0;
    gNfcip.cntTxRetrys   = 0;
    gNfcip.cntRTOXRetrys = 0;
}

ReturnCode rfal_nfc::nfcipRun( uint16_t *outActRxLen, bool *outIsChaining  )
{
    ReturnCode ret;
    
    ret = ERR_SYNTAX;
    
    Serial.print(" NFCIP Run() state: ");
    Serial.print(gNfcip.state );
    Serial.print(" \r\n"); 
    
    switch( gNfcip.state )
    {
        /*******************************************************************************/
        case NFCIP_ST_IDLE:
        case NFCIP_ST_INIT_DEP_IDLE:
        case NFCIP_ST_TARG_DEP_IDLE:
        case NFCIP_ST_TARG_DEP_SLEEP:
            return ERR_NONE;
            
        /*******************************************************************************/
        case NFCIP_ST_INIT_DEP_TX:
            
            Serial.print(" NFCIP(I) Tx PNI: ");
            Serial.print(gNfcip.pni);
            Serial.print(" txLen: ");
            Serial.print(gNfcip.txBufLen ); 
            Serial.print(" \r\n");
             
            ret = nfcipTx( NFCIP_CMD_DEP_REQ, gNfcip.txBuf, &gNfcip.txBuf[gNfcip.txBufPaylPos], gNfcip.txBufLen, nfcip_PFBIPDU( gNfcip.pni ), (gNfcip.cfg.fwt + gNfcip.cfg.dFwt) );
                        
            switch( ret )
            {
                case ERR_PARAM:
                default:
                  gNfcip.state = NFCIP_ST_INIT_DEP_IDLE;
                  return ret;
                  
              case ERR_NONE:
                  gNfcip.state = NFCIP_ST_INIT_DEP_RX;
                  break;
            }
            /* fall through */
            
        /*******************************************************************************/
        case NFCIP_ST_INIT_DEP_RX:          /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */

            ret = nfcipDataRx( false );
            
            if( ret != ERR_BUSY )
            {
                ret = nfcipInitiatorHandleDEP( ret, *gNfcip.rxRcvdLen, outActRxLen, outIsChaining );
            }
            
            break;
            
        /*******************************************************************************/    
        case NFCIP_ST_TARG_DEP_RTOX:
            
            if( !nfcipTimerisExpired( gNfcip.RTOXTimer ) )                    /* Do nothing until RTOX timer has expired */
            {
                return ERR_BUSY;
            }
            
            /* If we cannot send a RTOX raise a Timeout error so that we do not   
             * hold the field On forever in AP2P                                  */
            if( nfcipIsRTOXReqDisabled(gNfcip.cfg.oper) )
            {
                /* We should reEnable Rx, and measure time between our field Off to 
                 * either report link loss or recover               #287          */
                Serial.print( " NFCIP(T) RTOX not sent due to config, NOT reenabling Rx \r\n" );
                return ERR_TIMEOUT;
            } 

            if( gNfcip.cntRTOXRetrys++ > NFCIP_MAX_RTOX_RETRYS )              /* Check maximum consecutive RTOX requests */
            {
                return ERR_PROTO;
            }
            
            Serial.print( " NFCIP(T) RTOX sent \r\n" );
            
            gNfcip.lastRTOX = nfcip_RTOXTargMax(gNfcip.cfg.to);               /* Calculate requested RTOX value, and send it */                        
            EXIT_ON_ERR( ret, nfcipDEPControlMsg( nfcip_PFBSPDU_TO(), gNfcip.lastRTOX ) );
            
            /* Set waiting for RTOX Ack Flag */
            gNfcip.isWait4RTOX = true;
            
            gNfcip.state = NFCIP_ST_TARG_DEP_RX;                              /* Go back to Rx to process RTOX ack       */
            return ERR_BUSY;
            
        /*******************************************************************************/
        case NFCIP_ST_TARG_DEP_TX:
            
            Serial.print( " NFCIP(T) Tx PNI: ");
            Serial.print(gNfcip.pni);
            Serial.print(" txLen: ");
            Serial.print(gNfcip.txBufLen );
            Serial.print(" \r\n"); 
            ret = nfcipTx( NFCIP_CMD_DEP_RES, gNfcip.txBuf, &gNfcip.txBuf[gNfcip.txBufPaylPos], gNfcip.txBufLen, nfcip_PFBIPDU( gNfcip.pni ), NFCIP_NO_FWT );
            
            /* Clear flags */
            gNfcip.isTxPending = false;
            gNfcip.isWait4RTOX = false;
            
            /* Digital 1.0 14.12.3.4 Increment the current PNI after Tx */
            gNfcip.pni = nfcip_PNIInc( gNfcip.pni );
            
            switch( ret )
            {
                case ERR_PARAM:
                default:
                  gNfcip.state = NFCIP_ST_TARG_DEP_IDLE;                      /* Upon Tx error, goto IDLE state */
                  return ret;
                  
              case ERR_NONE:
                  gNfcip.state = NFCIP_ST_TARG_DEP_RX;                        /* All OK, goto Rx state          */
                  break;
            }
            /* fall through */
            
        /*******************************************************************************/
        case NFCIP_ST_TARG_DEP_RX:   /*  PRQA S 2003 # MISRA 16.3 - Intentional fall through */
            
            if( gNfcip.isReqPending )    /* if already has Data should be from a DEP from nfcipTargetHandleActivation()  */
            {
                Serial.print( " NFCIP(T) Skipping Rx Using DEP from Activation \r\n" );
                
                gNfcip.isReqPending = false;
                ret = ERR_NONE;
            }
            else
            {
                ret = nfcipDataRx( false );
            }
            
            if( ret != ERR_BUSY )
            {
                ret = nfcipTargetHandleRX( ret, outActRxLen, outIsChaining );
            }
            
            break;
            
        /*******************************************************************************/
        default:
            /* MISRA 16.4: no empty default statement (a comment being enough) */
            break;
    }
    return ret;
}

ReturnCode rfal_nfc::nfcipInitiatorHandleDEP( ReturnCode rxRes, uint16_t rxLen, uint16_t *outActRxLen, bool *outIsChaining )
{
    ReturnCode ret;
    uint8_t    nfcDepLen;
    uint8_t    rxMsgIt;
    uint8_t    rxPFB;
    uint8_t    rxRTOX;
    uint8_t    optHdrLen;
    
    ret        = ERR_INTERNAL;
    rxMsgIt    = 0;
    optHdrLen  = 0;
    
    *outActRxLen    = 0;
    *outIsChaining  = false;
    
    
    /*******************************************************************************/
    /* Handle reception errors                                                     */
    /*******************************************************************************/
    switch( rxRes )
    {
        /*******************************************************************************/
        /* Timeout ->  Digital 1.0 14.15.5.6 */
        case ERR_TIMEOUT:
            
            Serial.print( " NFCIP(I) TIMEOUT  TORetrys: ");
            Serial.print(gNfcip.cntTORetrys);
            Serial.print("\r\n");
            
            /* Digital 1.0 14.15.5.6 - If nTO >= Max raise protocol error */
            if( gNfcip.cntTORetrys++ >= NFCIP_MAX_TO_RETRYS )
            {
                return ERR_PROTO;
            }

            /*******************************************************************************/
            /* Upon Timeout error, if Deactivation is pending, no more error recovery 
             * will be done #54. 
             * This is used to address the issue some devices that havea big TO. 
             * Normally LLCP layer has timeout already, and NFCIP layer is still
             * running error handling, retrying ATN/NACKs                                  */
            /*******************************************************************************/
            if( nfcipIsDeactivationPending() )
            {
                Serial.print( " skipping error recovery due deactivation pending \r\n");
                return ERR_TIMEOUT;
            }
            
            /* Digital 1.0 14.15.5.6 1)  If last PDU was NACK */
            if( nfcip_PFBisRNACK(gNfcip.lastPFB) )
            {
                /* Digital 1.0 14.15.5.6 2)  if NACKs failed raise protocol error  */
                if( gNfcip.cntNACKRetrys++ >= NFCIP_MAX_NACK_RETRYS )
                {
                    return ERR_PROTO;
                }
                
                /* Send NACK */
                Serial.print( " NFCIP(I) Sending NACK retry: ");
                Serial.print(gNfcip.cntNACKRetrys );
                Serial.print("\r\n"); 
                EXIT_ON_ERR( ret, nfcipDEPControlMsg( nfcip_PFBRPDU_NACK(gNfcip.pni), 0 ) );
                return ERR_BUSY;
            }
            
            Serial.print( " NFCIP(I) Checking if to send ATN  ATNRetrys: ");
            Serial.print(gNfcip.cntATNRetrys);
            Serial.print("\r\n");
            
            /* Digital 1.0 14.15.5.6 3)  Otherwise send ATN */                            
            if( gNfcip.cntATNRetrys++ >= NFCIP_MAX_NACK_RETRYS )
            {
                return ERR_PROTO;
            }
                            
            /* Send ATN */
            Serial.print( " NFCIP(I) Sending ATN \r\n" );
            EXIT_ON_ERR( ret, nfcipDEPControlMsg( nfcip_PFBSPDU_ATN(), 0 ) );
            return ERR_BUSY;
        
        /*******************************************************************************/
        /* Data rcvd with error ->  Digital 1.0 14.12.5.4 */
        case ERR_CRC:
        case ERR_PAR:
        case ERR_FRAMING:
        case ERR_RF_COLLISION:
            
            Serial.print( " NFCIP(I) rx Error: ");
            Serial.print(rxRes );
            Serial.print("\r\n"); 
           
            /* Digital 1.0 14.12.5.4 Tx Error with data, ignore */
            if( rxLen < NFCIP_MIN_TXERROR_LEN )
            {
                Serial.print( " NFCIP(I) Transmission error w data  \r\n" );
#if 0
                if(gNfcip.cfg.commMode == RFAL_NFCDEP_COMM_PASSIVE)
                {
                    nfcipLogI( " NFCIP(I) Transmission error w data -> reEnabling Rx \r\n" );
                    nfcipReEnableRxTout( NFCIP_TRECOV );
                    return ERR_BUSY;
                }
#endif /* 0 */
            }

            /* Digital 1.1 16.12.5.4  if NACKs failed raise Transmission error  */
            if( gNfcip.cntNACKRetrys++ >= NFCIP_MAX_NACK_RETRYS )
            {
                return ERR_FRAMING;
            }
                            
            /* Send NACK */
            Serial.print( " NFCIP(I) Sending NACK  \r\n" );
            EXIT_ON_ERR( ret, nfcipDEPControlMsg( nfcip_PFBRPDU_NACK(gNfcip.pni), 0 ) );
            return ERR_BUSY;
            
        case ERR_NONE:
            break;
            
        case ERR_BUSY:
            return ERR_BUSY;  /* Debug purposes */
            
        default:
            Serial.print(" NFCIP(I) Error: ");
            Serial.print(rxRes );
            Serial.print("\r\n");
             
            return rxRes;
    }    
        
    /*******************************************************************************/
    /* Rx OK check if valid DEP PDU                                                */
    /*******************************************************************************/
    
    /* Due to different modes on ST25R391x (with/without CRC) use NFC-DEP LEN instead of bytes retrieved */
    nfcDepLen = gNfcip.rxBuf[rxMsgIt++];
    
    Serial.print( " NFCIP(I) rx OK: ");
    Serial.print(nfcDepLen );
    Serial.print(" bytes \r\n");
    
    /* Digital 1.0 14.15.5.5 Protocol Error  */
    if( gNfcip.rxBuf[rxMsgIt++] != NFCIP_RES )
    {
      Serial.print( " NFCIP(I) error ");
      Serial.print(gNfcip.rxBuf[--rxMsgIt]);
      Serial.print(" instead of ");
      Serial.print(NFCIP_RES);
      Serial.print("\r\n");
        return ERR_PROTO;
    }
    
    /* Digital 1.0 14.15.5.5 Protocol Error  */
    if( gNfcip.rxBuf[rxMsgIt++] != (uint8_t)NFCIP_CMD_DEP_RES )
    {
      Serial.print( " NFCIP(I) error ");
      Serial.print(gNfcip.rxBuf[--rxMsgIt]);
      Serial.print(" instead of ");
      Serial.print( NFCIP_CMD_DEP_RES );
      Serial.print("\r\n");
      
        return ERR_PROTO;
    }
    
    rxPFB = gNfcip.rxBuf[rxMsgIt++];
    
    /*******************************************************************************/
    /* Check for valid PFB type                                                    */
    if( !(nfcip_PFBisSPDU( rxPFB ) || nfcip_PFBisRPDU( rxPFB ) || nfcip_PFBisIPDU( rxPFB )) )
    {
        return ERR_PROTO;
    }
        
    /*******************************************************************************/
    /* Digital 1.0 14.8.2.1  check if DID is expected and match -> Protocol Error  */
    if( gNfcip.cfg.did != RFAL_NFCDEP_DID_NO ) 
    {
        if( (gNfcip.rxBuf[rxMsgIt++] != gNfcip.cfg.did) || !nfcip_PFBhasDID( rxPFB ) )
        {
            return ERR_PROTO;
        }
        optHdrLen++;                                    /* Inc header optional field cnt*/
    }
    else if( nfcip_PFBhasDID( rxPFB ) )                 /* DID not expected but rcv */
    {
        return ERR_PROTO;
    }
    else
    {
        /* MISRA 15.7 - Empty else */
    }
    
    /*******************************************************************************/
    /* Digital 1.0 14.6.2.8 & 14.6.3.11 NAD must not be used  */
    if( gNfcip.cfg.nad != RFAL_NFCDEP_NAD_NO ) 
    {
        if( (gNfcip.rxBuf[rxMsgIt++] != gNfcip.cfg.nad) || !nfcip_PFBhasNAD( rxPFB ) )
        {
            return ERR_PROTO;
        }        
        optHdrLen++;                                    /* Inc header optional field cnt*/
    }
    else if( nfcip_PFBhasNAD( rxPFB ) )                 /* NAD not expected but rcv */
    {
        return ERR_PROTO;
    }
    else
    {
        /* MISRA 15.7 - Empty else */
    }
       
    /*******************************************************************************/
    /* Process R-PDU                                                               */
    /*******************************************************************************/
    if( nfcip_PFBisRPDU( rxPFB ) )
    {
        /*******************************************************************************/
        /* R ACK                                                                       */
        /*******************************************************************************/
        if( nfcip_PFBisRACK( rxPFB ) )
        {
            Serial.print( " NFCIP(I) Rcvd ACK  \r\n" );
            if( gNfcip.pni == nfcip_PBF_PNI( rxPFB ) )
            {
                /* 14.12.3.3 R-ACK with correct PNI -> Increment */
                gNfcip.pni = nfcip_PNIInc( gNfcip.pni );
                                
                /* R-ACK while not performing chaining -> Protocol error*/
                if( !gNfcip.isTxChaining )
                {
                    return ERR_PROTO;
                }
                
                nfcipClearCounters();
                gNfcip.state = NFCIP_ST_INIT_DEP_IDLE;
                return ERR_NONE;                            /* This block has been transmitted */
            }
            else  /* Digital 1.0 14.12.4.5 ACK with wrong PNI Initiator may retransmit */
            {
                if( gNfcip.cntTxRetrys++ >= NFCIP_MAX_TX_RETRYS )
                {
                    return ERR_PROTO;
                }
                
                /* Extended the MAY in Digital 1.0 14.12.4.5 to only reTransmit if the ACK
                 * is for the previous DEP, otherwise raise Protocol immediately 
                 * If the PNI difference is more than 1 it is worthless to reTransmit 3x
                 * and after raise the error                                              */
                
                if( nfcip_PNIDec( gNfcip.pni ) ==  nfcip_PBF_PNI( rxPFB ) )
                {
                    /* ReTransmit */
                    Serial.print( " NFCIP(I) Rcvd ACK prev PNI -> reTx \r\n" );
                    gNfcip.state = NFCIP_ST_INIT_DEP_TX;
                    return ERR_BUSY;
                }
                
                Serial.print( " NFCIP(I) Rcvd ACK unexpected far PNI -> Error \r\n" );
                return ERR_PROTO;
            }
        }
        else /* Digital 1.0 - 14.12.5.2 Target must never send NACK  */
        {            
            return ERR_PROTO;
        }
    }
    
    /*******************************************************************************/
    /* Process S-PDU                                                               */
    /*******************************************************************************/
    if( nfcip_PFBisSPDU( rxPFB ) )                                
    {
        Serial.print( " NFCIP(I) Rcvd S-PDU  \r\n" );
        /*******************************************************************************/
        /* S ATN                                                                       */
        /*******************************************************************************/
        if( nfcip_PFBisSATN( rxPFB ) )                         /* If is a S-ATN        */
        {
            Serial.print( " NFCIP(I) Rcvd ATN  \r\n" );
            if( nfcip_PFBisSATN( gNfcip.lastPFB ) )            /* Check if is expected */
            {  
                gNfcip.cntATNRetrys = 0;                       /* Clear ATN counter    */
                
                /* Although spec is not clear NFC Forum Digital test is expecting to
                 * retransmit upon receiving ATN_RES */
                if( nfcip_PFBisSTO( gNfcip.lastPFBnATN ) )
                {
                    Serial.print( " NFCIP(I) Rcvd ATN  -> reTx RTOX_RES \r\n" );
                    EXIT_ON_ERR( ret, nfcipDEPControlMsg( nfcip_PFBSPDU_TO(), gNfcip.lastRTOX ) );
                }
                else
                {
                    /* ReTransmit ? */
                    if( gNfcip.cntTxRetrys++ >= NFCIP_MAX_TX_RETRYS )
                    {
                        return ERR_PROTO;
                    }
                    
                    Serial.print( " NFCIP(I) Rcvd ATN  -> reTx  PNI: ");
                    Serial.print(gNfcip.pni);
                    Serial.print("\r\n");
                    gNfcip.state = NFCIP_ST_INIT_DEP_TX;
                }
                
                return ERR_BUSY;
            }
            else                                               /* Digital 1.0  14.12.4.4 & 14.12.4.8 */
            {
                return ERR_PROTO;
            }
        }
        /*******************************************************************************/
        /* S TO                                                                        */
        /*******************************************************************************/
        else if( nfcip_PFBisSTO( rxPFB ) )                     /* If is a S-TO (RTOX)  */
        {
            Serial.print( " NFCIP(I) Rcvd TO  \r\n" );
            
            rxRTOX = gNfcip.rxBuf[rxMsgIt++];
            
            /* Digital 1.1 16.12.4.3 - Initiator MAY stop accepting subsequent RTOX Req   *
             *                       - RTOX request to an ATN -> Protocol error           */
            if( (gNfcip.cntRTOXRetrys++ > NFCIP_MAX_RTOX_RETRYS) || nfcip_PFBisSATN( gNfcip.lastPFB ) )
            {
                return ERR_PROTO;
            }
            
            /* Digital 1.1 16.8.4.1 RTOX must be between [1,59] */
            if( (rxRTOX < NFCIP_INIT_MIN_RTOX) || (rxRTOX > NFCIP_INIT_MAX_RTOX) )
            {
                return ERR_PROTO;
            }
            
            EXIT_ON_ERR( ret, nfcipDEPControlMsg( nfcip_PFBSPDU_TO(), rxRTOX ) );
            gNfcip.lastRTOX = rxRTOX;
            
            return ERR_BUSY;
        }
        else
        {
            /* Unexpected S-PDU */
            return ERR_PROTO;                       /*  PRQA S  2880 # MISRA 2.1 - Guard code to prevent unexpected behavior */
        }
    }
    
    /*******************************************************************************/
    /* Process I-PDU                                                               */
    /*******************************************************************************/
    if( nfcip_PFBisIPDU( rxPFB ) )
    {
        if( gNfcip.pni != nfcip_PBF_PNI( rxPFB ) )
        {
            Serial.print( " NFCIP(I) Rcvd IPDU wrong PNI     curPNI: ");
            Serial.print(gNfcip.pni);
            Serial.print("rxPNI: ");
            Serial.print(nfcip_PBF_PNI( rxPFB ));
            Serial.print("r\n");
            return ERR_PROTO;
        }
        
        Serial.print(" NFCIP(I) Rcvd IPDU OK    PNI: ");
        Serial.print(gNfcip.pni );
        Serial.print(" \r\n"); 
        
        /* 14.12.3.3 I-PDU with correct PNI -> Increment */
        gNfcip.pni = nfcip_PNIInc( gNfcip.pni );
                
        
        /* Successful data Exchange */
        nfcipClearCounters();
        *outActRxLen  = ((uint16_t)nfcDepLen - RFAL_NFCDEP_DEP_HEADER - (uint16_t)optHdrLen);
        
        if( (&gNfcip.rxBuf[gNfcip.rxBufPaylPos] != &gNfcip.rxBuf[RFAL_NFCDEP_DEP_HEADER + optHdrLen]) && (*outActRxLen > 0U) )
        {
            memmove( &gNfcip.rxBuf[gNfcip.rxBufPaylPos], &gNfcip.rxBuf[RFAL_NFCDEP_DEP_HEADER + optHdrLen], *outActRxLen );
        }

        /*******************************************************************************/
        /* Check if target is indicating chaining MI                                   */
        /*******************************************************************************/
        if( nfcip_PFBisIMI( rxPFB ) )
        {
            gNfcip.isRxChaining = true;
            *outIsChaining      = true;
            
            Serial.print(" NFCIP(I) Rcvd IPDU OK w MI -> ACK \r\n" );
            EXIT_ON_ERR( ret, nfcipDEPControlMsg( nfcip_PFBRPDU_ACK( gNfcip.pni ), gNfcip.rxBuf[rxMsgIt++] ) );
            
            return ERR_AGAIN;  /* Send Again signalling to run again, but some chaining data has arrived*/
        }
        else
        {
            gNfcip.isRxChaining = false;
            gNfcip.state        = NFCIP_ST_INIT_DEP_IDLE;
            
            ret = ERR_NONE;    /* Data exchange done */
        }
    }
    return ret;
}

ReturnCode rfal_nfc::nfcipDEPControlMsg( uint8_t pfb, uint8_t RTOX  )
{
    uint8_t        ctrlMsg[20];
    rfalNfcDepCmd  depCmd;
    uint32_t       fwt;


    /*******************************************************************************/
    /* Calculate Cmd and fwt to be used                                            */
    /*******************************************************************************/
    depCmd = ((gNfcip.cfg.role == RFAL_NFCDEP_ROLE_TARGET) ? NFCIP_CMD_DEP_RES : NFCIP_CMD_DEP_REQ);        
    fwt    = ((gNfcip.cfg.role == RFAL_NFCDEP_ROLE_TARGET) ? NFCIP_NO_FWT : (nfcip_PFBisSTO( pfb ) ? ( (RTOX*gNfcip.cfg.fwt) + gNfcip.cfg.dFwt) : (gNfcip.cfg.fwt + gNfcip.cfg.dFwt) ) );
    
    if( nfcip_PFBisSTO( pfb ) )
    {
        ctrlMsg[RFAL_NFCDEP_DEPREQ_HEADER_LEN] = RTOX;
        return nfcipTx( depCmd, ctrlMsg, &ctrlMsg[RFAL_NFCDEP_DEPREQ_HEADER_LEN], sizeof(uint8_t), pfb, fwt );
    }
    else
    {
        return nfcipTx( depCmd, ctrlMsg, NULL, 0, pfb, fwt );
    }
}

ReturnCode rfal_nfc::nfcipTargetHandleRX( ReturnCode rxRes, uint16_t *outActRxLen, bool *outIsChaining )
{
    ReturnCode ret;
    uint8_t    nfcDepLen;
    uint8_t    rxMsgIt;
    uint8_t    rxPFB;
    uint8_t    optHdrLen;
    uint8_t    resBuf[RFAL_NFCDEP_HEADER_PAD + NFCIP_TARGET_RES_MAX];
        
    
    ret        = ERR_INTERNAL;
    rxMsgIt    = 0;
    optHdrLen  = 0;
    
    *outActRxLen    = 0;
    *outIsChaining  = false;
    
    
    /*******************************************************************************/
    /* Handle reception errors                                                     */
    /*******************************************************************************/
    switch( rxRes )
    {
        /*******************************************************************************/        
        case ERR_NONE:
            break;
            
        case ERR_LINK_LOSS:
            Serial.print( " NFCIP(T) Error: ");
            Serial.print(rxRes );
            Serial.print("\r\n"); 
            return rxRes;
            
        case ERR_BUSY:
            return ERR_BUSY;  /* Debug purposes */
            
        case ERR_TIMEOUT:
        case ERR_CRC:
        case ERR_PAR:
        case ERR_FRAMING:
        case ERR_PROTO:
        default:
            /* Digital 1.1  16.12.5.2 The Target MUST NOT attempt any error recovery.      *
             * The Target MUST always stay in receive mode when a                          *
             * Transmission Error or a Protocol Error occurs.                              *
             *                                                                             *
             * Do not push Transmission/Protocol Errors to upper layer in Listen Mode #766 */
            
            nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
            return ERR_BUSY;
    }    
        
    /*******************************************************************************/
    /* Rx OK check if valid DEP PDU                                                */
    /*******************************************************************************/
    
    /* Due to different modes on ST25R391x (with/without CRC) use NFC-DEP LEN instead of bytes retrieved */
    nfcDepLen = gNfcip.rxBuf[rxMsgIt++];
        
    Serial.print( " NFCIP(T) rx OK: ");
    Serial.print(nfcDepLen);
    Serial.print(" bytes \r\n");
    
    if( gNfcip.rxBuf[rxMsgIt++] != NFCIP_REQ )
    {
        nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
        return ERR_BUSY; /* ERR_PROTO - Ignore bad request */
    }
    
    
    /*******************************************************************************/
    /* Check whether target rcvd a normal DEP or deactivation request              */
    /*******************************************************************************/
    switch( gNfcip.rxBuf[rxMsgIt++] )
    {               
        /*******************************************************************************/
        case (uint8_t)NFCIP_CMD_DEP_REQ:
            break;                                /* Continue to normal DEP processing */
    
        /*******************************************************************************/
        case (uint8_t)NFCIP_CMD_DSL_REQ:
            
            Serial.print( " NFCIP(T) rx DSL \r\n" );
            
            /* Digital 1.0  14.9.1.2 If DID is used and incorrect ignore it */
            /* [Digital 1.0, 16.9.1.2]: If DID == 0, Target SHALL ignore DSL_REQ with DID */
            if (   (((gNfcip.rxBuf[rxMsgIt++] != gNfcip.cfg.did) || (nfcDepLen != RFAL_NFCDEP_DSL_RLS_LEN_DID)) && (gNfcip.cfg.did != RFAL_NFCDEP_DID_NO) )
                || ((gNfcip.cfg.did == RFAL_NFCDEP_DID_NO) && (nfcDepLen != RFAL_NFCDEP_DSL_RLS_LEN_NO_DID))
               )
            {
                Serial.print( " NFCIP(T) DSL wrong DID, ignoring \r\n" );
                return ERR_BUSY;
            }
            
            nfcipTx( NFCIP_CMD_DSL_RES, resBuf, NULL, 0, 0, NFCIP_NO_FWT );
            
            gNfcip.state = NFCIP_ST_TARG_DEP_SLEEP;
            return ERR_SLEEP_REQ;
            
        /*******************************************************************************/
        case (uint8_t)NFCIP_CMD_RLS_REQ:
            
            Serial.print( " NFCIP(T) rx RLS \r\n" );
            
            /* Digital 1.0  14.10.1.2 If DID is used and incorrect ignore it */
            /* [Digital 1.0, 16.10.2.2]: If DID == 0, Target SHALL ignore DSL_REQ with DID */
            if (   (((gNfcip.rxBuf[rxMsgIt++] != gNfcip.cfg.did) || (nfcDepLen != RFAL_NFCDEP_DSL_RLS_LEN_DID)) && (gNfcip.cfg.did != RFAL_NFCDEP_DID_NO) )
                || ((gNfcip.cfg.did == RFAL_NFCDEP_DID_NO) && (nfcDepLen > RFAL_NFCDEP_DSL_RLS_LEN_NO_DID))
               )
            {
                Serial.print( " NFCIP(T) RLS wrong DID, ignoring \r\n" );
                return ERR_BUSY;
            }
                
            nfcipTx( NFCIP_CMD_RLS_RES, resBuf, NULL, 0, 0, NFCIP_NO_FWT );
            
            gNfcip.state = NFCIP_ST_TARG_DEP_IDLE;
            return ERR_RELEASE_REQ;
            
        /*******************************************************************************/
        /*case NFCIP_CMD_PSL_REQ:              PSL must be handled in Activation only */
        /*case NFCIP_CMD_WUP_REQ:              WUP not in NFC Forum Digital 1.0       */
        default:
            
            /* Don't go to NFCIP_ST_TARG_DEP_IDLE state as it needs to ignore this    *
             * invalid frame, and keep waiting for more frames                        */
            
            nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
            return ERR_BUSY; /* ERR_PROTO - Ignore bad frame */
    }
    
    /*******************************************************************************/
    
    rxPFB = gNfcip.rxBuf[rxMsgIt++];                    /* Store rcvd PFB  */
    
    /*******************************************************************************/
    /* Check for valid PFB type                                                    */
    if( !(nfcip_PFBisSPDU( rxPFB ) || nfcip_PFBisRPDU( rxPFB ) || nfcip_PFBisIPDU( rxPFB )) )
    {
        nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
        return ERR_BUSY; /* ERR_PROTO - Ignore invalid PFB  */
    }
    
    /*******************************************************************************/   
    if( gNfcip.cfg.did != RFAL_NFCDEP_DID_NO ) 
    {
        if( !nfcip_PFBhasDID( rxPFB ) )
        {
            nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
            return ERR_BUSY; /* ERR_PROTO - Ignore bad/missing DID  */
        }
        if( gNfcip.rxBuf[rxMsgIt++] != gNfcip.cfg.did ) /* MISRA 13.5 */
        {
            nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
            return ERR_BUSY; /* ERR_PROTO - Ignore bad/missing DID  */
        }
        optHdrLen++;                                    /* Inc header optional field cnt*/
    }
    else if( nfcip_PFBhasDID( rxPFB ) )                 /* DID not expected but rcv     */
    {
        nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
        return ERR_BUSY; /* ERR_PROTO - Ignore unexpected DID  */
    }
    else
    {
        /* MISRA 15.7 - Empty else */
    }
                                  
        
    /*******************************************************************************/
    if( gNfcip.cfg.nad != RFAL_NFCDEP_NAD_NO ) 
    {
        if( (gNfcip.rxBuf[rxMsgIt++] != gNfcip.cfg.did) || !nfcip_PFBhasDID( rxPFB ) )
        {
            nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
            return ERR_BUSY;                            /* ERR_PROTO - Ignore bad/missing DID  */
        }
        optHdrLen++;                                    /* Inc header optional field cnt*/
    }
    else if( nfcip_PFBhasNAD( rxPFB ) )                 /* NAD not expected but rcv */
    {
        nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
        return ERR_BUSY;                                /* ERR_PROTO - Ignore unexpected NAD  */
    }
    else
    {
        /* MISRA 15.7 - Empty else */
    }
    
       
    /*******************************************************************************/
    /* Process R-PDU                                                               */
    /*******************************************************************************/
    if( nfcip_PFBisRPDU( rxPFB ) )
    {
        Serial.print( " NFCIP(T) Rcvd R-PDU  \r\n" );
        /*******************************************************************************/
        /* R ACK                                                                       */
        /*******************************************************************************/
        if( nfcip_PFBisRACK( rxPFB ) )
        {
            Serial.print( " NFCIP(T) Rcvd ACK  \r\n" );
            if( gNfcip.pni == nfcip_PBF_PNI( rxPFB ) )
            {
                /* R-ACK while not performing chaining -> Protocol error */
                if( !gNfcip.isTxChaining )
                {
                    nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
                    return ERR_BUSY;                    /* ERR_PROTO - Ignore unexpected ACK  */
                }
                
                /* This block has been transmitted and acknowledged, perform RTOX until next data is provided  */
                
                /* Digital 1.1  16.12.4.7 - If ACK rcvd continue with chaining or an RTOX */
                nfcipTimerStart( gNfcip.RTOXTimer, nfcipRTOXAdjust( nfcipConv1FcToMs( rfalNfcDepWT2RWT( gNfcip.cfg.to ) )) );
                gNfcip.state = NFCIP_ST_TARG_DEP_RTOX;
                
                return ERR_NONE;                        /* This block has been transmitted */
            }
            
            /* Digital 1.0 14.12.3.4 - If last send was ATN and rx PNI is minus 1 */
            else if( nfcip_PFBisSATN( gNfcip.lastPFB ) &&  (nfcip_PNIDec(gNfcip.pni) == nfcip_PBF_PNI( rxPFB )) )    
            {   
                Serial.print( " NFCIP(T) wrong PNI, last was ATN reTx  \r\n" );
                /* Spec says to leave current PNI as is, but will be Inc after Tx, remaining the same */
                gNfcip.pni = nfcip_PNIDec( gNfcip.pni );
                
                gNfcip.state = NFCIP_ST_TARG_DEP_TX;
                return ERR_BUSY;
            }
            else
            {
                /* MISRA 15.7 - Empty else */
            }
        }
        /*******************************************************************************/
        /* R NACK                                                                      */
        /*******************************************************************************/
        /* ISO 18092 12.6.1.3.3 When rcv NACK if PNI = prev PNI sent ->  reTx          */
        else if( nfcip_PFBisRNACK( rxPFB ) && (nfcip_PNIDec(gNfcip.pni) == nfcip_PBF_PNI( rxPFB ) ) )
        {
            Serial.print( " NFCIP(T) Rcvd NACK  \r\n" );
            
            gNfcip.pni = nfcip_PNIDec( gNfcip.pni );   /* Dec so that has the prev PNI */
            
            gNfcip.state = NFCIP_ST_TARG_DEP_TX;
            return ERR_BUSY;
        }
        else
        {        
            Serial.print( " NFCIP(T) Unexpected R-PDU \r\n" );
            
            nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
            return ERR_BUSY; /* ERR_PROTO - Ignore unexpected R-PDU  */
        }
    }
    
    /*******************************************************************************/
    /* Process S-PDU                                                               */
    /*******************************************************************************/
    if( nfcip_PFBisSPDU( rxPFB ) )
    {
        Serial.print( " NFCIP(T) Rcvd S-PDU  \r\n" );
        
        /*******************************************************************************/
        /* S ATN                                                                       */
        /*******************************************************************************/
        /* ISO 18092 12.6.3 Attention                                                  */
        if( nfcip_PFBisSATN( rxPFB ) )                         /*    If is a S-ATN     */
        {            
            Serial.print( " NFCIP(T) Rcvd ATN  curPNI: ");
            Serial.print( gNfcip.pni );
            Serial.print(" \r\n");
            EXIT_ON_ERR( ret, nfcipDEPControlMsg( nfcip_PFBSPDU_ATN(), 0 ) );
            return ERR_BUSY;
        }
        
        /*******************************************************************************/
        /* S TO                                                                        */
        /*******************************************************************************/
        else if( nfcip_PFBisSTO( rxPFB ) )                     /* If is a S-TO (RTOX)  */
        {
            if( nfcip_PFBisSTO( gNfcip.lastPFBnATN ) )
            {
                Serial.print( " NFCIP(T) Rcvd TO  \r\n" );
                
                /* Digital 1.1  16.8.4.6  RTOX value in RES different that in REQ -> Protocol Error */
                if( gNfcip.lastRTOX != gNfcip.rxBuf[rxMsgIt++] )
                {
                    Serial.print( " NFCIP(T) Mismatched RTOX value \r\n" );
                    
                    nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
                    return ERR_BUSY; /* ERR_PROTO - Ignore unexpected RTOX value  */
                }
                
                /* Clear waiting for RTOX Ack Flag */
                gNfcip.isWait4RTOX = false;
                
                /* Check if a Tx is already pending */
                if( gNfcip.isTxPending )
                {
                    Serial.print( " NFCIP(T) Tx pending, go immediately to TX \r\n" );
                    
                    gNfcip.state = NFCIP_ST_TARG_DEP_TX;
                    return ERR_BUSY;
                }
                
                /* Start RTOX timer and change to check state  */
                nfcipTimerStart( gNfcip.RTOXTimer, nfcipRTOXAdjust( nfcipConv1FcToMs( gNfcip.lastRTOX * rfalNfcDepWT2RWT(gNfcip.cfg.to ) ) ) );
                gNfcip.state = NFCIP_ST_TARG_DEP_RTOX;
                
                return ERR_BUSY;
            }
        }
        else
        {
            /* Unexpected S-PDU */
            Serial.print( " NFCIP(T) Unexpected S-PDU \r\n" );         /*  PRQA S  2880 # MISRA 2.1 - Guard code to prevent unexpected behavior */
            
            nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
            return ERR_BUSY; /* ERR_PROTO - Ignore unexpected S-PDU  */
        }
    }
    
    /*******************************************************************************/
    /* Process I-PDU                                                               */
    /*******************************************************************************/
    if( nfcip_PFBisIPDU( rxPFB ) )
    {
        if( gNfcip.pni != nfcip_PBF_PNI( rxPFB ) )
        {
            Serial.print( " NFCIP(T) Rcvd IPDU wrong PNI     curPNI: ");
            Serial.print(gNfcip.pni);
            Serial.print(" rxPNI: ");
            Serial.print(nfcip_PBF_PNI( rxPFB ) );
            Serial.print("\r\n"); 
            
            /* Digital 1.1 16.12.3.4 - If last send was ATN and rx PNI is minus 1 */
            if( nfcip_PFBisSATN(gNfcip.lastPFB ) &&  (nfcip_PNIDec(gNfcip.pni) == nfcip_PBF_PNI( rxPFB )) ) 
            {
                /* Spec says to leave current PNI as is, but will be Inc after Data Tx, remaining the same */
                gNfcip.pni = nfcip_PNIDec(gNfcip.pni);
                
                if( nfcip_PFBisIMI( rxPFB ) )
                {
                    Serial.print( " NFCIP(T) PNI = prevPNI && ATN before && chaining -> send ACK  \r\n" );
                    EXIT_ON_ERR( ret, nfcipDEPControlMsg( nfcip_PFBRPDU_ACK( gNfcip.pni ), gNfcip.rxBuf[rxMsgIt++] ) );
                    
                    /* Digital 1.1 16.12.3.4 (...) leave the current PNI unchanged afterwards */
                    gNfcip.pni = nfcip_PNIInc( gNfcip.pni );
                }
                else
                {
                    Serial.print( " NFCIP(T) PNI = prevPNI && ATN before -> reTx last I-PDU  \r\n" );
                    gNfcip.state = NFCIP_ST_TARG_DEP_TX;
                }
                
                return ERR_BUSY;
            }
                        
            nfcDepReEnableRx( gNfcip.rxBuf, gNfcip.rxBufLen, gNfcip.rxRcvdLen );
            return ERR_BUSY;            /* ERR_PROTO - Ignore bad PNI value  */
        }
        
        Serial.print( " NFCIP(T) Rcvd IPDU OK PNI: ");
        Serial.print(gNfcip.pni );
        Serial.print("\r\n");
        /*******************************************************************************/
        /* Successful data exchange                                                    */
        /*******************************************************************************/
        *outActRxLen  = ((uint16_t)nfcDepLen - RFAL_NFCDEP_DEP_HEADER - (uint16_t)optHdrLen);
        
        nfcipClearCounters();

        if( (&gNfcip.rxBuf[gNfcip.rxBufPaylPos] != &gNfcip.rxBuf[RFAL_NFCDEP_DEP_HEADER + optHdrLen]) && (*outActRxLen > 0U) )
        {
            memmove( &gNfcip.rxBuf[gNfcip.rxBufPaylPos], &gNfcip.rxBuf[RFAL_NFCDEP_DEP_HEADER + optHdrLen], *outActRxLen );
        }
        
        
        /*******************************************************************************/
        /* Check if Initiator is indicating chaining MI                                */
        /*******************************************************************************/
        if( nfcip_PFBisIMI( rxPFB ) )
        {
            gNfcip.isRxChaining = true;
            *outIsChaining      = true;
            
            Serial.print( " NFCIP(T) Rcvd IPDU OK w MI -> ACK \r\n" );
            EXIT_ON_ERR( ret, nfcipDEPControlMsg( nfcip_PFBRPDU_ACK( gNfcip.pni ), gNfcip.rxBuf[rxMsgIt++] ) );
            
            gNfcip.pni = nfcip_PNIInc( gNfcip.pni );
            
            return ERR_AGAIN;  /* Send Again signalling to run again, but some chaining data has arrived*/
        }
        else
        {
            if(gNfcip.isRxChaining)
            {
                Serial.print( " NFCIP(T) Rcvd last IPDU chaining finished \r\n" );
            }
            
            /*******************************************************************************/
            /* Reception done, send to DH and start RTOX timer                             */
            /*******************************************************************************/
            nfcipTimerStart( gNfcip.RTOXTimer, nfcipRTOXAdjust( nfcipConv1FcToMs( rfalNfcDepWT2RWT( gNfcip.cfg.to ) )) );
            gNfcip.state = NFCIP_ST_TARG_DEP_RTOX;
            
            gNfcip.isRxChaining = false;
            ret = ERR_NONE;                            /* Data exchange done */
        }
    }
    return ret;
}


static void  nfcipTimerStart(uint16_t timer, uint16_t time )
{  
  timer = (millis() + time);
}
