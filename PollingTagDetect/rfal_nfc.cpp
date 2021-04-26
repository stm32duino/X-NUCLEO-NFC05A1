#include "rfal_nfc.h"

rfal_nfc::rfal_nfc(rfal_rf* rfal)
{
  RF = rfal;
}

ReturnCode rfal_nfc::rfalNfcInitialize( void )
{
    ReturnCode err;    
    gNfcDev.state = RFAL_NFC_STATE_NOTINIT;
    
    RF->rfalAnalogConfigInitialize();              /* Initialize RFAL's Analog Configs */
    
    EXIT_ON_ERR( err, RF->rfalInitialize() );      /* Initialize RFAL */
    
    if(err == ERR_NONE){
      gNfcDev.state = RFAL_NFC_STATE_IDLE;         /* Go to initialized */  
    }
    
    return err;
}

void rfal_nfc::rfalNfcWorker()
{
  ReturnCode err;

  RF->rfalWorker();
  switch(gNfcDev.state){
    case RFAL_NFC_STATE_NOTINIT:                            
    case RFAL_NFC_STATE_IDLE:                               
            break;

    case RFAL_NFC_STATE_START_DISCOVERY:
        
            /* Initialize context for discovery cycle */
            gNfcDev.devCnt      = 0;
            gNfcDev.selDevIdx   = 0;
            gNfcDev.techsFound  = RFAL_NFC_TECH_NONE;
            gNfcDev.techs2do    = gNfcDev.disc.techs2Find;
            gNfcDev.state       = RFAL_NFC_STATE_POLL_TECHDETECT;
        
        /* Check if Low power Wake-Up is to be performed */
            if( gNfcDev.disc.wakeupEnabled )
            {
                /* Initialize Low power Wake-up mode and wait */
                //err = rfalWakeUpModeStart( (gNfcDev.disc.wakeupConfigDefault ? NULL : &gNfcDev.disc.wakeupConfig) );
                if( err == ERR_NONE )
                {
                    gNfcDev.state = RFAL_NFC_STATE_WAKEUP_MODE;
                    //rfalNfcNfcNotify( gNfcDev.state );                                /* Notify caller that WU was started */
                }
            }
          break;
     
        case RFAL_NFC_STATE_ACTIVATED:
        case RFAL_NFC_STATE_POLL_SELECT:
        case RFAL_NFC_STATE_DATAEXCHANGE_DONE:
        default:
            return;  
  }
}

void rfal_nfc::CheckInterrupts() {
  RF->CheckInterrupts();
}

ReturnCode rfal_nfc::rfalNfcGetDevicesFound( rfalNfcDevice **devList, uint8_t *devCnt )
{
    /* Check for valid state */
    if( gNfcDev.state < RFAL_NFC_STATE_POLL_SELECT )
    {
        return ERR_WRONG_STATE;
    }
    
    /* Check valid parameters */
    if( (devList == NULL) || (devCnt == NULL) )
    {
        return ERR_PARAM;
    }
    
    *devCnt  = gNfcDev.devCnt;
    *devList = gNfcDev.devList;
    
    return ERR_NONE;
}

ReturnCode rfal_nfc::rfalNfcSelect( uint8_t devIdx )
{
    /* Check for valid state */
    if( gNfcDev.state != RFAL_NFC_STATE_POLL_SELECT )
    {
        return ERR_WRONG_STATE;
    }
    
    gNfcDev.selDevIdx = devIdx;
    gNfcDev.state     = RFAL_NFC_STATE_POLL_ACTIVATION;
    
    return ERR_NONE;
}

ReturnCode rfal_nfc::rfalNfcDeactivate( bool discovery )
{
    /* Check for valid state */
    if( gNfcDev.state <= RFAL_NFC_STATE_IDLE )
    {
        return ERR_WRONG_STATE;
    }
    
    /* Check if discovery is to continue afterwards */
    if( discovery == true )
    {
        /* If so let the state machine continue*/
        gNfcDev.discRestart = discovery;
        gNfcDev.state       = RFAL_NFC_STATE_DEACTIVATION;
    }
    else
    {
        /* Otherwise deactivate immediately and go to IDLE */
        rfalNfcDeactivation();
        gNfcDev.state = RFAL_NFC_STATE_IDLE;
    }
    
    return ERR_NONE;
}

ReturnCode rfal_nfc::rfalNfcDeactivation( void )
{
    /* Check if a device has been activated */
    if( gNfcDev.activeDev != NULL )
    {
        switch( gNfcDev.activeDev->rfInterface )
        {
            /*******************************************************************************/
            case RFAL_NFC_INTERFACE_RF:
                break;                                                                /* No specific deactivation to be performed */
            
            /*******************************************************************************/
        #if RFAL_FEATURE_ISO_DEP_POLL
            case RFAL_NFC_INTERFACE_ISODEP:
                RF->rfalIsoDepDeselect();                                                 /* Send a Deselect to device */
                break;
        #endif /* RFAL_FEATURE_ISO_DEP_POLL */
                
            /*******************************************************************************/
        #if RFAL_FEATURE_NFC_DEP
            case RFAL_NFC_INTERFACE_NFCDEP:
                rfalNfcDepRLS();                                                      /* Send a Release to device */
                break;
        #endif /* RFAL_FEATURE_NFC_DEP */
                
            default:
                return ERR_REQUEST;
        }
    }
    
    #if RFAL_FEATURE_WAKEUP_MODE
        //rfalWakeUpModeStop();
    #endif /* RFAL_FEATURE_WAKEUP_MODE */
    
    #if RFAL_FEATURE_LISTEN_MODE
        //rfalListenStop();
    #else
        //rfalFieldOff();
    #endif
    
    gNfcDev.activeDev = NULL;
    return ERR_NONE;
}

ReturnCode rfal_nfc::rfalNfcDiscover( const rfalNfcDiscoverParam *disParams )
{
    /* Check if initialization has been performed */
    if( gNfcDev.state != RFAL_NFC_STATE_IDLE )
    {
        return ERR_WRONG_STATE;
    }
    
    /* Check valid parameters */
    if( (disParams == NULL) || (disParams->devLimit > RFAL_NFC_MAX_DEVICES) || (disParams->devLimit == 0U)                                                ||
        ( ((disParams->techs2Find & RFAL_NFC_POLL_TECH_F) != 0U)     && (disParams->nfcfBR != RFAL_BR_212) && (disParams->nfcfBR != RFAL_BR_424) )        ||
        ( (((disParams->techs2Find & RFAL_NFC_POLL_TECH_AP2P) != 0U) && (disParams->ap2pBR > RFAL_BR_424)) || (disParams->GBLen > RFAL_NFCDEP_GB_MAX_LEN)) )
    {
        return ERR_PARAM;
    }
    
    if( (((disParams->techs2Find & RFAL_NFC_POLL_TECH_A) != 0U)      && !((bool)RFAL_FEATURE_NFCA))    ||
        (((disParams->techs2Find & RFAL_NFC_POLL_TECH_B) != 0U)      && !((bool)RFAL_FEATURE_NFCB))    ||
        (((disParams->techs2Find & RFAL_NFC_POLL_TECH_F) != 0U)      && !((bool)RFAL_FEATURE_NFCF))    ||
        (((disParams->techs2Find & RFAL_NFC_POLL_TECH_V) != 0U)      && !((bool)RFAL_FEATURE_NFCV))    ||
        (((disParams->techs2Find & RFAL_NFC_POLL_TECH_ST25TB) != 0U) && !((bool)RFAL_FEATURE_ST25TB))  ||
        (((disParams->techs2Find & RFAL_NFC_POLL_TECH_AP2P) != 0U)   && !((bool)RFAL_FEATURE_NFC_DEP)) ||
        (((disParams->techs2Find & RFAL_NFC_LISTEN_TECH_A) != 0U)    && !((bool)RFAL_FEATURE_NFCA))    ||
        (((disParams->techs2Find & RFAL_NFC_LISTEN_TECH_B) != 0U)    && !((bool)RFAL_FEATURE_NFCB))    ||
        (((disParams->techs2Find & RFAL_NFC_LISTEN_TECH_F) != 0U)    && !((bool)RFAL_FEATURE_NFCF))    ||
        (((disParams->techs2Find & RFAL_NFC_LISTEN_TECH_AP2P) != 0U) && !((bool)RFAL_FEATURE_NFC_DEP))   )
    {
        return ERR_DISABLED;   /*  PRQA S  2880 # MISRA 2.1 - Unreachable code due to configuration option being set/unset  */ 
    }
    
    /* Initialize context for discovery */
    gNfcDev.activeDev       = NULL;
    gNfcDev.techsFound      = RFAL_NFC_TECH_NONE;
    gNfcDev.devCnt          = 0;
    gNfcDev.discRestart     = true;
    gNfcDev.disc            = *disParams;
    
    
    /* Calculate Listen Mask */
    gNfcDev.lmMask  = 0U;
    gNfcDev.lmMask |= (((gNfcDev.disc.techs2Find & RFAL_NFC_LISTEN_TECH_A) != 0U) ? RFAL_LM_MASK_NFCA : 0U);
    gNfcDev.lmMask |= (((gNfcDev.disc.techs2Find & RFAL_NFC_LISTEN_TECH_B) != 0U) ? RFAL_LM_MASK_NFCB : 0U);
    gNfcDev.lmMask |= (((gNfcDev.disc.techs2Find & RFAL_NFC_LISTEN_TECH_F) != 0U) ? RFAL_LM_MASK_NFCF : 0U);
    gNfcDev.lmMask |= (((gNfcDev.disc.techs2Find & RFAL_NFC_LISTEN_TECH_AP2P) != 0U) ? RFAL_LM_MASK_ACTIVE_P2P : 0U);
    
#if !RFAL_FEATURE_LISTEN_MODE
    /* Check if Listen Mode is supported/Enabled */
    if( gNfcDev.lmMask != 0U )
    {
        return ERR_NOTSUPP;
    }
#endif
    
    gNfcDev.state = RFAL_NFC_STATE_START_DISCOVERY;
    
    return ERR_NONE;
}

rfalNfcState rfal_nfc::rfalNfcGetState( void )
{
    return gNfcDev.state;
}

ReturnCode rfal_nfc::rfalNfcGetActiveDevice( rfalNfcDevice **dev )
{
    /* Check for valid state */
    if( gNfcDev.state < RFAL_NFC_STATE_ACTIVATED )
    {
        return ERR_WRONG_STATE;
    }
    
    /* Check valid parameter */
    if( dev == NULL )
    {
        return ERR_PARAM;
    }
    
    /* Check for valid state */
    if( (gNfcDev.devCnt == 0U) || (gNfcDev.activeDev == NULL)  )
    {
        return ERR_REQUEST;
    }
    
    *dev = gNfcDev.activeDev;
    return ERR_NONE;
}

ReturnCode rfal_nfc::rfalNfcDataExchangeStart( uint8_t *txData, uint16_t txDataLen, uint8_t **rxData, uint16_t **rvdLen, uint32_t fwt )
{
    ReturnCode            err;
    rfalTransceiveContext ctx;
    
    /*******************************************************************************/
    /* The Data Exchange is divided in two different moments, the trigger/Start of *
     *  the transfer followed by the check until its completion                    */
    if( (gNfcDev.state >= RFAL_NFC_STATE_ACTIVATED) && (gNfcDev.activeDev != NULL) )
    {
        
        /*******************************************************************************/
        /* In Listen mode is the Poller that initiates the communicatation             */
        /* Assign output parameters and rfalNfcDataExchangeGetStatus will return       */
        /* incoming data from Poller/Initiator                                         */
        if( (gNfcDev.state == RFAL_NFC_STATE_ACTIVATED) && rfalNfcIsRemDevPoller( gNfcDev.activeDev->type ) )
        {
            if( txDataLen > 0U )
            {
                return ERR_WRONG_STATE;
            }
            
            *rvdLen = (uint16_t*)&gNfcDev.rxLen;
            *rxData = (uint8_t*)(  (gNfcDev.activeDev->rfInterface == RFAL_NFC_INTERFACE_ISODEP) ? gNfcDev.rxBuf.isoDepBuf.inf : 
                                  ((gNfcDev.activeDev->rfInterface == RFAL_NFC_INTERFACE_NFCDEP) ? gNfcDev.rxBuf.nfcDepBuf.inf : gNfcDev.rxBuf.rfBuf) );
            return ERR_NONE;
        }
        
        
        /*******************************************************************************/
        switch( gNfcDev.activeDev->rfInterface )                                      /* Check which RF interface shall be used/has been activated */
        {
            /*******************************************************************************/
            case RFAL_NFC_INTERFACE_RF:
    
                rfalCreateByteFlagsTxRxContext( ctx, (uint8_t*)txData, txDataLen, gNfcDev.rxBuf.rfBuf, sizeof(gNfcDev.rxBuf.rfBuf), &gNfcDev.rxLen, RFAL_TXRX_FLAGS_DEFAULT, fwt );
                *rxData = (uint8_t*)gNfcDev.rxBuf.rfBuf;
                *rvdLen = (uint16_t*)&gNfcDev.rxLen;
                err = RF->rfalStartTransceive( &ctx );
                break;
                
        #if RFAL_FEATURE_ISO_DEP
            /*******************************************************************************/
            case RFAL_NFC_INTERFACE_ISODEP:
            {
                rfalIsoDepTxRxParam isoDepTxRx;
                
                if( txDataLen > 0U )
                {
                    memcpy( (uint8_t*)gNfcDev.txBuf.isoDepBuf.inf, txData, txDataLen );
                }
                
                isoDepTxRx.DID          = RFAL_ISODEP_NO_DID;
                isoDepTxRx.ourFSx       = RFAL_ISODEP_FSX_KEEP;
                isoDepTxRx.FSx          = gNfcDev.activeDev->proto.isoDep.info.FSx;
                isoDepTxRx.dFWT         = gNfcDev.activeDev->proto.isoDep.info.dFWT;
                isoDepTxRx.FWT          = gNfcDev.activeDev->proto.isoDep.info.FWT;
                isoDepTxRx.txBuf        = &gNfcDev.txBuf.isoDepBuf;
                isoDepTxRx.txBufLen     = txDataLen;
                isoDepTxRx.isTxChaining = false;
                isoDepTxRx.rxBuf        = &gNfcDev.rxBuf.isoDepBuf;
                isoDepTxRx.rxLen        = &gNfcDev.rxLen;
                isoDepTxRx.isRxChaining = &gNfcDev.isRxChaining;
                *rxData                 = (uint8_t*)gNfcDev.rxBuf.isoDepBuf.inf;
                *rvdLen                 = (uint16_t*)&gNfcDev.rxLen;
                
                /*******************************************************************************/
                /* Trigger a RFAL ISO-DEP Transceive                                           */
                err = RF->rfalIsoDepStartTransceive( isoDepTxRx );
                break;
            }
        #endif /* RFAL_FEATURE_ISO_DEP */
                
        #if RFAL_FEATURE_NFC_DEP
            /*******************************************************************************/
            case RFAL_NFC_INTERFACE_NFCDEP:
            {
                rfalNfcDepTxRxParam nfcDepTxRx;
                
                if( txDataLen > 0U)
                {
                    memcpy( (uint8_t*)gNfcDev.txBuf.nfcDepBuf.inf, txData, txDataLen );
                }
                
                nfcDepTxRx.DID          = RFAL_NFCDEP_DID_KEEP;
                nfcDepTxRx.FSx          = rfalNfcDepLR2FS( (uint8_t)rfalNfcDepPP2LR( gNfcDev.activeDev->proto.nfcDep.activation.Target.ATR_RES.PPt ) );
                nfcDepTxRx.dFWT         = gNfcDev.activeDev->proto.nfcDep.info.dFWT;
                nfcDepTxRx.FWT          = gNfcDev.activeDev->proto.nfcDep.info.FWT;
                nfcDepTxRx.txBuf        = &gNfcDev.txBuf.nfcDepBuf;
                nfcDepTxRx.txBufLen     = txDataLen;
                nfcDepTxRx.isTxChaining = false;
                nfcDepTxRx.rxBuf        = &gNfcDev.rxBuf.nfcDepBuf;
                nfcDepTxRx.rxLen        = &gNfcDev.rxLen;
                nfcDepTxRx.isRxChaining = &gNfcDev.isRxChaining;
                *rxData                 = (uint8_t*)gNfcDev.rxBuf.nfcDepBuf.inf;
                *rvdLen                 = (uint16_t*)&gNfcDev.rxLen;
                
                /*******************************************************************************/
                /* Trigger a RFAL NFC-DEP Transceive                                           */
                err = rfalNfcDepStartTransceive( &nfcDepTxRx );                          
                break;
            }
        #endif /* RFAL_FEATURE_NFC_DEP */

            /*******************************************************************************/
            default:
                err = ERR_PARAM;
                break;
        }
        
        /* If a transceive has succesfully started flag Data Exchange as ongoing */
        if( err == ERR_NONE )
        {
            gNfcDev.dataExErr = ERR_BUSY;
            gNfcDev.state     = RFAL_NFC_STATE_DATAEXCHANGE;
        }
        
        return err;
    }
    
    return ERR_WRONG_STATE;
}

ReturnCode rfal_nfc::rfalNfcDataExchangeGetStatus( void )
{
    /*******************************************************************************/
    /* Check if it's the first frame received in Listen mode */
    if( gNfcDev.state == RFAL_NFC_STATE_ACTIVATED )
    {
        /* Continue data exchange as normal */
        gNfcDev.dataExErr = ERR_BUSY;
        gNfcDev.state     = RFAL_NFC_STATE_DATAEXCHANGE;
        
        /* Check if we performing in T3T CE */
        if( (gNfcDev.activeDev->type == RFAL_NFC_POLL_TYPE_NFCF) && (gNfcDev.activeDev->rfInterface == RFAL_NFC_INTERFACE_RF) )
        {
            /* The first frame has been retrieved by rfalListenMode, flag data immediately                  */
            /* Can only call rfalGetTransceiveStatus() after starting a transceive with rfalStartTransceive */
            gNfcDev.dataExErr = ERR_NONE;
        }
    }
    
    
    /*******************************************************************************/
    /* Check if we are in we have been placed to sleep, and return last error     */
    if( gNfcDev.state == RFAL_NFC_STATE_LISTEN_SLEEP )
    {
        return gNfcDev.dataExErr;                                /* ERR_SLEEP_REQ */
    }

    
    /*******************************************************************************/    
    /* Check if Data exchange has been started */
    if( (gNfcDev.state != RFAL_NFC_STATE_DATAEXCHANGE) && (gNfcDev.state != RFAL_NFC_STATE_DATAEXCHANGE_DONE)  )
    {
        return ERR_WRONG_STATE;
    }
    
    /* Check if Data exchange is still ongoing */
    if( gNfcDev.dataExErr == ERR_BUSY )
    {
        switch( gNfcDev.activeDev->rfInterface )
        {
            /*******************************************************************************/
            case RFAL_NFC_INTERFACE_RF:
                gNfcDev.dataExErr = RF->rfalGetTransceiveStatus();
                break;
        
        #if RFAL_FEATURE_ISO_DEP
            /*******************************************************************************/
            case RFAL_NFC_INTERFACE_ISODEP:
                gNfcDev.dataExErr = RF->rfalIsoDepGetTransceiveStatus();
                break;
        #endif /* RFAL_FEATURE_ISO_DEP */
                
            /*******************************************************************************/
        #if RFAL_FEATURE_NFC_DEP
            case RFAL_NFC_INTERFACE_NFCDEP:
                gNfcDev.dataExErr = rfalNfcDepGetTransceiveStatus();
                break;
        #endif /* RFAL_FEATURE_NFC_DEP */
                
            /*******************************************************************************/
            default:
                gNfcDev.dataExErr = ERR_PARAM;
                break;
        }
        
        
    #if  RFAL_FEATURE_LISTEN_MODE
        /*******************************************************************************/
        /* If a Sleep request has been received (Listen Mode) go to sleep immediately  */
        if( gNfcDev.dataExErr == ERR_SLEEP_REQ )
        {
            EXIT_ON_ERR( gNfcDev.dataExErr, rfalListenSleepStart( RFAL_LM_STATE_SLEEP_A, gNfcDev.rxBuf.rfBuf, sizeof(gNfcDev.rxBuf.rfBuf), &gNfcDev.rxLen ) );
            
            /* If set Sleep was succesfull keep restore the Sleep request signal */
            gNfcDev.dataExErr = ERR_SLEEP_REQ;
        }
    #endif /* RFAL_FEATURE_LISTEN_MODE */
        
    }
    
    return gNfcDev.dataExErr;
}

ReturnCode rfal_nfc::rfalNfcDepGetTransceiveStatus( void )
{
    return nfcipRun( gNfcip.rxRcvdLen, gNfcip.isChaining );
}


bool rfal_nfc::nfcipTimerisExpired( uint32_t timer )
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
