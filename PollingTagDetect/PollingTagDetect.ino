#include "rfal_nfc.h"

/* DEFINE */
#define SPI_MOSI D11
#define SPI_MISO D12
#define SPI_SCK D13
#define CS_PIN D10 
#define LED1_PIN A1 
#define LED2_PIN A2 
#define LED3_PIN A3
#define LED4_PIN D5
#define LED5_PIN D4
#define LED6_PIN D3
#define IRQ_PIN A0
#define USE_LOGGER 0 /* to enable the debug, must set USE_LOGGER to 1*/

/* USE_LOGGER == 1 */
#define MAX_HEX_STR         4
#define MAX_HEX_STR_LENGTH  128
char hexStr[MAX_HEX_STR][MAX_HEX_STR_LENGTH];
uint8_t hexStrIdx = 0;
/* #if USE_LOGGER == LOGGER_ON */

static uint8_t state = DEMO_ST_NOTINIT;
int PushButtonState = 0;
static rfalNfcDevice *nfcDevice;

/* FUNCTION */
static void demoNotif( rfalNfcState st );
static void demoCycle();
static void demoAPDU();
static void demoP2P();
static void demoNfcf( rfalNfcfListenDevice *nfcfDev );
static void demoNfcv( rfalNfcvListenDevice *nfcvDev );
static void demoCE( rfalNfcDevice *nfcDev );

void IRQCallback();
char* hex2Str(unsigned char * data, size_t dataLen);
ReturnCode demoTransceiveBlocking( uint8_t *txBuf, uint16_t txBufSize, uint8_t **rxData, uint16_t **rcvLen, uint32_t fwt );

/* P2P communication data */
static uint8_t NFCID3[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t GB[] = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};
    
/* APDUs communication data */    
static uint8_t ndefSelectApp[] = { 0x00, 0xA4, 0x04, 0x00, 0x07, 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01, 0x00 };
static uint8_t ccSelectFile[] = { 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03};
static uint8_t readBynary[] = { 0x00, 0xB0, 0x00, 0x00, 0x0F };
/*static uint8_t ppseSelectApp[] = { 0x00, 0xA4, 0x04, 0x00, 0x0E, 0x32, 0x50, 0x41, 0x59, 0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0x00 };*/

/* P2P communication data */    
static uint8_t ndefLLCPSYMM[] = {0x00, 0x00};
static uint8_t ndefInit[] = {0x05, 0x20, 0x06, 0x0F, 0x75, 0x72, 0x6E, 0x3A, 0x6E, 0x66, 0x63, 0x3A, 0x73, 0x6E, 0x3A, 0x73, 0x6E, 0x65, 0x70, 0x02, 0x02, 0x07, 0x80, 0x05, 0x01, 0x02};
static uint8_t ndefUriSTcom[] = {0x13, 0x20, 0x00, 0x10, 0x02, 0x00, 0x00, 0x00, 0x19, 0xc1, 0x01, 0x00, 0x00, 0x00, 0x12, 0x55, 0x00, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 0x2f, 0x77, 0x77, 0x77, 0x2e, 0x73, 0x74, 0x2e, 0x63, 0x6f, 0x6d};

/* SPI, Component and NFC */
SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);
rfal_rfst25r3911b rfst25r3911b(&dev_spi, CS_PIN,IRQ_PIN);  /* 0 -> default speed */
rfal_nfc rfal_nfc(&rfst25r3911b);


void setup() {
  Serial.begin(115200);
  dev_spi.begin();

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(LED5_PIN, OUTPUT);
  pinMode(IRQ_PIN, INPUT);
  pinMode(USER_BTN, INPUT);
  
  /* Check what is the Push Button State when the button is not pressed. It can change across families */
  PushButtonState = (digitalRead(USER_BTN)) ?  0 : 1;
  
  attachInterrupt(IRQ_PIN,IRQCallback,RISING);

  Serial.println("Welcome to X-NUCLEO-NFC05A1");
  
  /* Inizialize Component */
  if(rfal_nfc.rfalNfcInitialize() == ERR_NONE){
      rfal_nfc.discParam.compMode      = RFAL_COMPLIANCE_MODE_NFC;
      rfal_nfc.discParam.devLimit      = 1U;
      rfal_nfc.discParam.nfcfBR        = RFAL_BR_212;
      rfal_nfc.discParam.ap2pBR        = RFAL_BR_424;
            
      memcpy( &(rfal_nfc.discParam.nfcid3), NFCID3, sizeof(NFCID3) );
      memcpy( &(rfal_nfc.discParam.GB), GB, sizeof(GB) );
      rfal_nfc.discParam.GBLen         = sizeof(GB);
    
      rfal_nfc.discParam.notifyCb           = demoNotif;
      rfal_nfc.discParam.totalDuration        = 1000U;
      rfal_nfc.discParam.wakeupEnabled        = false;
      rfal_nfc.discParam.wakeupConfigDefault  = true;
      rfal_nfc.discParam.techs2Find           = ( RFAL_NFC_POLL_TECH_A | RFAL_NFC_POLL_TECH_B | RFAL_NFC_POLL_TECH_F | RFAL_NFC_POLL_TECH_V | RFAL_NFC_POLL_TECH_ST25TB );
           
      rfal_nfc.discParam.techs2Find   |= RFAL_NFC_POLL_TECH_AP2P;
      state = DEMO_ST_START_DISCOVERY;
      LEDCycle();
      Serial.println("Correctly Inizialized");  
  }else{
    Serial.println("Initialization failed...");
    while(1){
      digitalWrite(LED1_PIN, HIGH);
      delay(100);
      digitalWrite(LED1_PIN, LOW);
      delay(100);
    }
  }
    

  Serial.println("Start POLLING");
}

void loop() {
 demoCycle();
}

void IRQCallback()
{
  rfal_nfc.CheckInterrupts();
}

void demoCycle(){

  rfal_nfc.rfalNfcWorker();                                    /* Run RFAL worker periodically */  

  if( digitalRead(USER_BTN) == PushButtonState )
  {
        rfal_nfc.discParam.wakeupEnabled = !(rfal_nfc.discParam.wakeupEnabled);    /* enable/disable wakeup */
        state = DEMO_ST_START_DISCOVERY;                       /* restart loop          */
        Serial.print("Toggling Wake Up mode");
        Serial.print(rfal_nfc.discParam.wakeupEnabled ? "ON": "OFF");
        Serial.println("\r\n"); 
        
        /* Debouncing */
        delay(50);

        /* Wait until the button is released */
        while ((digitalRead( USER_BTN) == PushButtonState));

        /* Debouncing */
        delay(50);
  }
    
  switch(state){
    case DEMO_ST_START_DISCOVERY:
        digitalWrite(LED1_PIN, HIGH);
        delay(500);
        digitalWrite(LED1_PIN, LOW);
        delay(500);

        rfal_nfc.rfalNfcDeactivate( false );
        rfal_nfc.rfalNfcDiscover( &(rfal_nfc.discParam) );
          
        state = DEMO_ST_DISCOVERY;
        break;
        
    case DEMO_ST_DISCOVERY:
        digitalWrite(LED2_PIN, HIGH);
        delay(500);
        digitalWrite(LED2_PIN, LOW);
        delay(500);
        
        if( rfalNfcIsDevActivated(rfal_nfc.rfalNfcGetState()) )
        {
                rfal_nfc.rfalNfcGetActiveDevice( &nfcDevice );
                
                switch( nfcDevice->type )
                {
                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCA:                    
                        
                        //platformLedOn(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
                        digitalWrite(LED1_PIN,HIGH);
                        
                        switch( nfcDevice->dev.nfca.type )
                        {
                            case RFAL_NFCA_T1T:
                                
                                Serial.print("ISO14443A/Topaz (NFC-A T1T) TAG found. UID: ");
                                Serial.print(hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                                Serial.println("\r\n"); 
                                break;
                            
                            case RFAL_NFCA_T4T:
                                Serial.print("ISO14443A/Topaz (NFC-A T1T) TAG found. UID: ");
                                Serial.print(hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                                Serial.println("\r\n"); 
                                demoAPDU();
                                break;
                            
                            case RFAL_NFCA_T4T_NFCDEP:
                            case RFAL_NFCA_NFCDEP:
                                Serial.print("NFCA Passive P2P device found. NFCID: ");
                                Serial.print(hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                                Serial.println("\r\n"); 
                                demoP2P();    
                                break;
                                
                            default:
                                Serial.print("SO14443A/NFC-A card found. UID: ");
                                Serial.print(hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                                Serial.println("\r\n"); 
                                break;
                        }
                        break;
                        /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCB:
                        
                        Serial.print("ISO14443B/NFC-B card found. UID: ");
                        Serial.print(hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                        Serial.print("\r\n");

                        /* platformLedOn(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN); */
                        digitalWrite(LED2_PIN,HIGH);
                        
                    
                        if(rfalNfcbIsIsoDepSupported( &nfcDevice->dev.nfcb ) )
                        {
                            demoAPDU();
                        }
                        break;
                        
                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCF:
                        
                        if(rfalNfcfIsNfcDepSupported( &nfcDevice->dev.nfcf ) )
                        {
                            Serial.print("NFCF Passive P2P device found. NFCID: ");
                            Serial.print(hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                            Serial.print("\r\n"); 
                            demoP2P(); 
                        }
                        else
                        {
                            Serial.print("Felica/NFC-F card found. UID: ");
                            Serial.print(hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ));
                            Serial.print("\r\n"); 
                            
                            demoNfcf( &nfcDevice->dev.nfcf );
                        }
                        
                        /*platformLedOn(PLATFORM_LED_F_PORT, PLATFORM_LED_F_PIN);*/
                        digitalWrite(LED3_PIN,HIGH);
                        break;
                    
                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCV:
                        {
                            uint8_t devUID[RFAL_NFCV_UID_LEN];
                            
                            memcpy( devUID, nfcDevice->nfcid, nfcDevice->nfcidLen );   /* Copy the UID into local var */
                            REVERSE_BYTES( devUID, RFAL_NFCV_UID_LEN );                 /* Reverse the UID for display purposes */
                            Serial.print("ISO15693/NFC-V card found. UID: ");
                            Serial.print(hex2Str(devUID, RFAL_NFCV_UID_LEN));
                            Serial.print("\r\n");
                        
                            /*platformLedOn(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);*/
                            digitalWrite(LED4_PIN,HIGH);
                            
                            demoNfcv( &nfcDevice->dev.nfcv );
                        }
                        break;
                        
                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_ST25TB:
                        
                        Serial.print("ST25TB card found. UID: ");
                        Serial.print(hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ));
                        Serial.print("\r\n");
                         
                        /*platformLedOn(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);*/
                        digitalWrite(LED5_PIN,HIGH);
                        break;
                    
                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_AP2P:
                        
                        Serial.print("NFC Active P2P device found. NFCID3: ");
                        Serial.print(hex2Str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                        Serial.print("\r\n"); 
                        
                        /*platformLedOn(PLATFORM_LED_AP2P_PORT, PLATFORM_LED_AP2P_PIN);*/
                        digitalWrite(LED6_PIN,HIGH);
                    
                        demoP2P();
                        break;
                    
                    /*******************************************************************************/
                    case RFAL_NFC_POLL_TYPE_NFCA:
                    case RFAL_NFC_POLL_TYPE_NFCF:
                        
                        Serial.print("Activated in CE ");
                        Serial.print((nfcDevice->type == RFAL_NFC_POLL_TYPE_NFCA) ? "NFC-A" : "NFC-F");
                        Serial.print(" mode.\r\n");

                         /*
                        platformLedOn((nfcDevice->type == RFAL_NFC_POLL_TYPE_NFCA)  ? PLATFORM_LED_A_PORT : PLATFORM_LED_F_PORT), 
                                       ((nfcDevice->type == RFAL_NFC_POLL_TYPE_NFCA) ? PLATFORM_LED_A_PIN  : PLATFORM_LED_F_PIN)  );*/ 
                                                        
                        //demoCE( nfcDevice );
                        break;
                    
                    /*******************************************************************************/
                    default:
                        break;
                }
                
                //rfalNfcDeactivate( false );
                delay( 500 );
                state = DEMO_ST_START_DISCOVERY;
            }
            break;
   }
}

void demoNotif( rfalNfcState st )
{
    uint8_t       devCnt;
    rfalNfcDevice *dev;
    
    
    if( st == RFAL_NFC_STATE_WAKEUP_MODE )
    {
        Serial.println("Wake Up mode started \r\n");
    }
    else if( st == RFAL_NFC_STATE_POLL_TECHDETECT )
    {
        Serial.println("Wake Up mode terminated. Polling for devices \r\n");
    }
    else if( st == RFAL_NFC_STATE_POLL_SELECT )
    {
        /* Multiple devices were found, activate first of them */
        rfal_nfc.rfalNfcGetDevicesFound( &dev, &devCnt );
        rfal_nfc.rfalNfcSelect( 0 );
        
        Serial.println("Multiple Tags detected: ");
        Serial.println(devCnt);
        Serial.println("\r\n");
    }
}

void LEDCycle()
{
  digitalWrite(LED1_PIN, HIGH);
  delay(500);
  digitalWrite(LED2_PIN, HIGH);
  digitalWrite(LED1_PIN, LOW);
  delay(500);
  digitalWrite(LED3_PIN, HIGH);
  digitalWrite(LED2_PIN, LOW);
  delay(500);
  digitalWrite(LED4_PIN, HIGH);
  digitalWrite(LED3_PIN, LOW);
  delay(500);
  digitalWrite(LED5_PIN, HIGH);
  digitalWrite(LED4_PIN, LOW);
  delay(500);
  digitalWrite(LED5_PIN, LOW);
  delay(500);
}

void demoAPDU( void )
{
    ReturnCode err;
    uint16_t   *rxLen;
    uint8_t    *rxData;


    /* Exchange APDU: NDEF Tag Application Select command */
    err = demoTransceiveBlocking( ndefSelectApp, sizeof(ndefSelectApp), &rxData, &rxLen, RFAL_FWT_NONE );
    Serial.print(" Select NDEF Application: ");
    Serial.print((err != ERR_NONE) ? "FAIL": "OK");
    Serial.print(" Data: ");
    Serial.print(hex2Str( rxData, *rxLen) );
    Serial.println("\r\n"); 

    if( (err == ERR_NONE) && rxData[0] == 0x90 && rxData[1] == 0x00)
    {
        /* Exchange APDU: Select Capability Container File */
        err = demoTransceiveBlocking( ccSelectFile, sizeof(ccSelectFile), &rxData, &rxLen, RFAL_FWT_NONE );
        Serial.print(" Select CC: ");
        Serial.print((err != ERR_NONE) ? "FAIL": "OK");
        Serial.print(" Data: ");
        Serial.print(hex2Str( rxData, *rxLen) );
        Serial.println("\r\n"); 

        /* Exchange APDU: Read Capability Container File  */
        err = demoTransceiveBlocking( readBynary, sizeof(readBynary), &rxData, &rxLen, RFAL_FWT_NONE );
        Serial.print(" Read CC: ");
        Serial.print((err != ERR_NONE) ? "FAIL": "OK"); 
        Serial.print(" Data: ");
        Serial.print(hex2Str( rxData, *rxLen) );
        Serial.println("\r\n"); 
    }
}

void demoP2P()
{
    uint16_t   *rxLen;
    uint8_t    *rxData;
    ReturnCode err;

    Serial.print(" Initalize device .. ");
    err = demoTransceiveBlocking( ndefInit, sizeof(ndefInit), &rxData, &rxLen, RFAL_FWT_NONE);
    if( err != ERR_NONE )
    {
        Serial.println("failed.");
        return;
    }
    Serial.print("succeeded.\r\n");

    Serial.print(" Push NDEF Uri: www.ST.com .. ");
    err = demoTransceiveBlocking( ndefUriSTcom, sizeof(ndefUriSTcom), &rxData, &rxLen, RFAL_FWT_NONE);
    if( err != ERR_NONE )
    {
        Serial.println("failed.");
        return;
    }
    Serial.print("succeeded.\r\n");


    Serial.print(" Device present, maintaining connection ");
    while(err == ERR_NONE) 
    {
        err = demoTransceiveBlocking( ndefLLCPSYMM, sizeof(ndefLLCPSYMM), &rxData, &rxLen, RFAL_FWT_NONE);
        Serial.println(".");
        delay(50);
    }
    Serial.print("\r\n Device removed.\r\n");
}

void demoNfcf( rfalNfcfListenDevice *nfcfDev )
{
    ReturnCode                 err;
    uint8_t                    buf[ (RFAL_NFCF_NFCID2_LEN + RFAL_NFCF_CMD_LEN + (3*RFAL_NFCF_BLOCK_LEN)) ];
    uint16_t                   rcvLen;
    rfalNfcfServ               srv = RFAL_NFCF_SERVICECODE_RDWR;
    rfalNfcfBlockListElem      bl[3];
    rfalNfcfServBlockListParam servBlock;
    //uint8_t                    wrData[] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
    
    servBlock.numServ   = 1;                            /* Only one Service to be used           */
    servBlock.servList  = &srv;                         /* Service Code: NDEF is Read/Writeable  */
    servBlock.numBlock  = 1;                            /* Only one block to be used             */
    servBlock.blockList = bl;
    bl[0].conf     = RFAL_NFCF_BLOCKLISTELEM_LEN;       /* Two-byte Block List Element           */     
    bl[0].blockNum = 0x0001;                            /* Block: NDEF Data                      */
    
    /*
     * TO IMPLEMENT
     *err = rfalNfcfPollerCheck( nfcfDev->sensfRes.NFCID2, &servBlock, buf, sizeof(buf), &rcvLen);
     */
    Serial.print(" Check Block: ");
    Serial.print((err != ERR_NONE) ? "FAIL": "OK");
    Serial.print(" Data:  ");
    Serial.print((err != ERR_NONE) ? "" : hex2Str( &buf[1], RFAL_NFCF_BLOCK_LEN) );
    Serial.print("\r\n"); 
    
    #if 0  /* Writing example */
        err = rfalNfcfPollerUpdate( nfcfDev->sensfRes.NFCID2, &servBlock, buf , sizeof(buf), wrData, buf, sizeof(buf) );
        Serial.print(" Update Block: ");
        Serial.print((err != ERR_NONE) ? "FAIL": "OK");
        Serial.print(" Data:  ");
        Serial.print((err != ERR_NONE) ? "" : hex2Str( wrData, RFAL_NFCF_BLOCK_LEN) );
        Serial.print("\r\n"); 
        err = rfalNfcfPollerCheck( nfcfDev->sensfRes.NFCID2, &servBlock, buf, sizeof(buf), &rcvLen);
        Serial.print(" Check Block: ");
        Serial.print((err != ERR_NONE) ? "FAIL": "OK");
        Serial.print(" Data:  ");
        Serial.print((err != ERR_NONE) ? "" : hex2Str( &buf[1], RFAL_NFCF_BLOCK_LEN) );
        Serial.print("\r\n"); 
    #endif
}

void demoNfcv( rfalNfcvListenDevice *nfcvDev )
{
    ReturnCode            err;
    uint16_t              rcvLen;
    uint8_t               blockNum = 1;
    uint8_t               rxBuf[ 1 + DEMO_NFCV_BLOCK_LEN + RFAL_CRC_LEN ];                        /* Flags + Block Data + CRC */
    uint8_t *             uid; 
    #if DEMO_NFCV_WRITE_TAG
        uint8_t               wrData[DEMO_NFCV_BLOCK_LEN] = { 0x11, 0x22, 0x33, 0x99 };             /* Write block example */
    #endif
              

    uid = nfcvDev->InvRes.UID;
    
    #if DEMO_NFCV_USE_SELECT_MODE
        /*
        * Activate selected state
        */
        /*TO IMPLEMENT
        *     err = rfalNfcvPollerSelect(RFAL_NFCV_REQ_FLAG_DEFAULT, nfcvDev->InvRes.UID );
        */
        Serial.print(" Select ");
        Serial.print( (err != ERR_NONE) ? "FAIL (revert to addressed mode)": "OK" );
        Serial.print("\r\n");
        
        if( err == ERR_NONE )
        {
            uid = NULL;
        }
    #endif    

    /*
    * Read block using Read Single Block command
    * with addressed mode (uid != NULL) or selected mode (uid == NULL)
    */
    
    /*TO IMPLEMENT
    * err = rfalNfcvPollerReadSingleBlock(RFAL_NFCV_REQ_FLAG_DEFAULT, uid, blockNum, rxBuf, sizeof(rxBuf), &rcvLen);
    */
    Serial.print(" Read Block: ");
    Serial.print((err != ERR_NONE) ? "FAIL": "OK Data:");
    Serial.print( (err != ERR_NONE) ? "" : hex2Str( &rxBuf[1], DEMO_NFCV_BLOCK_LEN));
    Serial.print("\r\n");
    
    #if DEMO_NFCV_WRITE_TAG /* Writing example */
        err = rfalNfcvPollerWriteSingleBlock(RFAL_NFCV_REQ_FLAG_DEFAULT, uid, blockNum, wrData, sizeof(wrData));
        Serial.print(" Write Block: ");
        Serial.print((err != ERR_NONE) ? "FAIL": "OK Data:");
        Serial.print( hex2Str( wrData, DEMO_NFCV_BLOCK_LEN) );
        Serial.print("\r\n");
        err = rfalNfcvPollerReadSingleBlock(RFAL_NFCV_REQ_FLAG_DEFAULT, uid, blockNum, rxBuf, sizeof(rxBuf), &rcvLen);
        Serial.print(" Read Block: ");
        Serial.print((err != ERR_NONE) ? "FAIL": "OK Data:");
        Serial.print( (err != ERR_NONE) ? "" : hex2Str( &rxBuf[1], DEMO_NFCV_BLOCK_LEN));
        Serial.print("\r\n");
    #endif
}

void demoCE( rfalNfcDevice *nfcDev )
{
#if defined(ST25R3916) && defined(RFAL_FEATURE_LISTEN_MODE)
    
    ReturnCode err;
    uint8_t *rxData;
    uint16_t *rcvLen;
    uint8_t  txBuf[100];
    uint16_t txLen;
    
    demoCeInit( ceNFCF_nfcid2 );
    
    do
    {
        rfalNfcWorker();
        
        switch( rfalNfcGetState() )
        {
            case RFAL_NFC_STATE_ACTIVATED:
                err = demoTransceiveBlocking( NULL, 0, &rxData, &rcvLen, 0);
                break;
            
            case RFAL_NFC_STATE_DATAEXCHANGE:
            case RFAL_NFC_STATE_DATAEXCHANGE_DONE:
                
                txLen = ( (nfcDev->type == RFAL_NFC_POLL_TYPE_NFCA) ? demoCeT4T( rxData, *rcvLen, txBuf, sizeof(txBuf) ): demoCeT3T( rxData, *rcvLen, txBuf, sizeof(txBuf) ) );
                err   = demoTransceiveBlocking( txBuf, txLen, &rxData, &rcvLen, RFAL_FWT_NONE );
                break;
            
            case RFAL_NFC_STATE_LISTEN_SLEEP:
            default:
                break;
        }
    }
    while( (err == ERR_NONE) || (err == ERR_SLEEP_REQ) );
    
#endif /* RFAL_FEATURE_LISTEN_MODE */
}

ReturnCode demoTransceiveBlocking( uint8_t *txBuf, uint16_t txBufSize, uint8_t **rxData, uint16_t **rcvLen, uint32_t fwt )
{
    ReturnCode err;
    
    err = rfal_nfc.rfalNfcDataExchangeStart( txBuf, txBufSize, rxData, rcvLen, fwt );
    if( err == ERR_NONE )
    {
        do{
            rfal_nfc.rfalNfcWorker();
            err = rfal_nfc.rfalNfcDataExchangeGetStatus();
        }
        while( err == ERR_BUSY );
    }
    return err;
}

char* hex2Str(unsigned char * data, size_t dataLen)
{
  
  if (USE_LOGGER == 1)
  {
    unsigned char * pin = data;
    const char * hex = "0123456789ABCDEF";
    char * pout = hexStr[hexStrIdx];
    uint8_t i = 0;
    uint8_t idx = hexStrIdx;
    size_t len;  
      
    if(dataLen == 0)
    {
      pout[0] = 0;     
    } 
    else     
    {
      /* Trim data that doesn't fit in buffer */
      len = MIN( dataLen , (MAX_HEX_STR_LENGTH / 2) );
        
      for(; i < (len - 1); ++i)
      {
          *pout++ = hex[(*pin>>4)&0xF];
          *pout++ = hex[(*pin++)&0xF];
      }
      *pout++ = hex[(*pin>>4)&0xF];
      *pout++ = hex[(*pin)&0xF];
      *pout = 0;
    }    
    
    hexStrIdx++;
    hexStrIdx %= MAX_HEX_STR;
    
    return hexStr[idx];
  }
  else
  {
    return NULL;
  }
}
