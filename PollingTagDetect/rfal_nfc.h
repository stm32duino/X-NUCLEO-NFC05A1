#ifndef RFAL_NFC_H 
#define RFAL_NFC_H

#include "rfal_rfst25r3911b.h"
#include "rfal_nfcDep.h"
#include "rfal_nfca.h"
#include "rfal_nfcb.h"
#include "rfal_nfcf.h"
#include "rfal_nfcv.h"

#define LED1_PIN A1 
#define LED2_PIN A2 
#define LED3_PIN A3
#define LED5_PIN D4
#define LED4_PIN D5
/*
 * 
    
    
    #include "rfal_st25tb.h"    
    #include "rfal_isoDep.h"
 */
 
/* DEFINE */

/* Definition of possible states the demo state machine could have */
#define DEMO_ST_NOTINIT               0     /*!< Demo State:  Not initialized        */
#define DEMO_ST_START_DISCOVERY       1     /*!< Demo State:  Start Discovery        */
#define DEMO_ST_DISCOVERY             2     /*!< Demo State:  Discovery              */

#define DEMO_NFCV_BLOCK_LEN           4     /*!< NFCV Block len                      */

#define DEMO_NFCV_USE_SELECT_MODE     false /*!< NFCV demonstrate select mode        */
#define DEMO_NFCV_WRITE_TAG           false /*!< NFCV demonstrate Write Single Block */

#define RFAL_NFC_RF_BUF_LEN              255U     /*!< No technology             */

#define RFAL_NFC_TECH_NONE               0x0000U  /*!< No technology             */
#define RFAL_NFC_POLL_TECH_A             0x0001U  /*!< NFC-A technology Flag     */
#define RFAL_NFC_POLL_TECH_B             0x0002U  /*!< NFC-B technology Flag     */
#define RFAL_NFC_POLL_TECH_F             0x0004U  /*!< NFC-F technology Flag     */
#define RFAL_NFC_POLL_TECH_V             0x0008U  /*!< NFC-V technology Flag     */
#define RFAL_NFC_POLL_TECH_AP2P          0x0010U  /*!< AP2P technology Flag      */
#define RFAL_NFC_POLL_TECH_ST25TB        0x0020U  /*!< ST25TB technology Flag    */
#define RFAL_NFC_LISTEN_TECH_A           0x1000U  /*!< NFC-V technology Flag     */
#define RFAL_NFC_LISTEN_TECH_B           0x2000U  /*!< NFC-V technology Flag     */
#define RFAL_NFC_LISTEN_TECH_F           0x4000U  /*!< NFC-V technology Flag     */
#define RFAL_NFC_LISTEN_TECH_AP2P        0x8000U  /*!< NFC-V technology Flag     */

/* MACROS */
/*! Checks if a device is currently activated */
#define rfalNfcIsDevActivated( st )        ( ((st)>= RFAL_NFC_STATE_ACTIVATED) && ((st)<RFAL_NFC_STATE_DEACTIVATION) )

/*! Checks if a device is in discovery */
#define rfalNfcIsInDiscovery( st )         ( ((st)>= RFAL_NFC_STATE_START_DISCOVERY) && ((st)<RFAL_NFC_STATE_ACTIVATED) )

#define RFAL_NFC_MAX_DEVICES          5U    /* Max number of devices supported */

#define rfalNfcIsRemDevPoller( tp )    ( ((tp)>= RFAL_NFC_POLL_TYPE_NFCA) && ((tp)<=RFAL_NFC_POLL_TYPE_AP2P ) )
#define rfalNfcIsRemDevListener( tp )  ( /*((tp)>= RFAL_NFC_LISTEN_TYPE_NFCA) && */ ((tp)<=RFAL_NFC_LISTEN_TYPE_AP2P) )

#define rfalNfcNfcNotify( st )         if( gNfcDev.disc.notifyCb != NULL )  gNfcDev.disc.notifyCb( st )

#define REVERSE_BYTES(pData, nDataSize) \
  {unsigned char swap, *lo = ((unsigned char *)(pData)), *hi = ((unsigned char *)(pData)) + (nDataSize) - 1; \
  while (lo < hi) { swap = *lo; *lo++ = *hi; *hi-- = swap; }}
  
/* TYPEDEF */

/*! Main state                                                                       */
typedef enum{
    RFAL_NFC_STATE_NOTINIT                  =  0,   /*!< Not Initialized state       */
    RFAL_NFC_STATE_IDLE                     =  1,   /*!< Initialize state            */
    RFAL_NFC_STATE_START_DISCOVERY          =  2,   /*!< Start Discovery loop state  */
    RFAL_NFC_STATE_WAKEUP_MODE              =  3,   /*!< Wake-Up state               */
    RFAL_NFC_STATE_POLL_TECHDETECT          =  10,  /*!< Technology Detection state  */
    RFAL_NFC_STATE_POLL_COLAVOIDANCE        =  11,  /*!< Collision Avoidance state   */
    RFAL_NFC_STATE_POLL_SELECT              =  12,  /*!< Wait for Selection state    */
    RFAL_NFC_STATE_POLL_ACTIVATION          =  13,  /*!< Activation state            */
    RFAL_NFC_STATE_LISTEN_TECHDETECT        =  20,  /*!< Listen Tech Detect          */
    RFAL_NFC_STATE_LISTEN_COLAVOIDANCE      =  21,  /*!< Listen Collision Avoidance  */
    RFAL_NFC_STATE_LISTEN_ACTIVATION        =  22,  /*!< Listen Activation state     */
    RFAL_NFC_STATE_LISTEN_SLEEP             =  23,  /*!< Listen Sleep state          */
    RFAL_NFC_STATE_ACTIVATED                =  30,  /*!< Activated state             */
    RFAL_NFC_STATE_DATAEXCHANGE             =  31,  /*!< Data Exchange Start state   */
    RFAL_NFC_STATE_DATAEXCHANGE_DONE        =  33,  /*!< Data Exchange terminated    */
    RFAL_NFC_STATE_DEACTIVATION             =  34   /*!< Deactivation state          */
}rfalNfcState;


/*! Device type                                                                       */
typedef enum{
    RFAL_NFC_LISTEN_TYPE_NFCA               =  0,   /*!< NFC-A Listener device type  */
    RFAL_NFC_LISTEN_TYPE_NFCB               =  1,   /*!< NFC-B Listener device type  */
    RFAL_NFC_LISTEN_TYPE_NFCF               =  2,   /*!< NFC-F Listener device type  */
    RFAL_NFC_LISTEN_TYPE_NFCV               =  3,   /*!< NFC-V Listener device type  */
    RFAL_NFC_LISTEN_TYPE_ST25TB             =  4,   /*!< ST25TB Listener device type */
    RFAL_NFC_LISTEN_TYPE_AP2P               =  5,   /*!< AP2P Listener device type   */
    RFAL_NFC_POLL_TYPE_NFCA                 =  10,  /*!< NFC-A Poller device type    */
    RFAL_NFC_POLL_TYPE_NFCB                 =  11,  /*!< NFC-B Poller device type    */
    RFAL_NFC_POLL_TYPE_NFCF                 =  12,  /*!< NFC-F Poller device type    */
    RFAL_NFC_POLL_TYPE_NFCV                 =  13,  /*!< NFC-V Poller device type    */
    RFAL_NFC_POLL_TYPE_AP2P                 =  15   /*!< AP2P Poller device type     */
}rfalNfcDevType;


/*! Device interface                                                                 */
typedef enum{
    RFAL_NFC_INTERFACE_RF                   = 0,    /*!< RF Frame interface          */
    RFAL_NFC_INTERFACE_ISODEP               = 1,    /*!< ISO-DEP interface           */
    RFAL_NFC_INTERFACE_NFCDEP               = 2     /*!< NFC-DEP interface           */
}rfalNfcRfInterface;


/*! Device struct containing all its details                                          */
typedef struct{
    rfalNfcDevType type;                            /*!< Device's type                */
    union{                              /*  PRQA S 0750 # MISRA 19.2 - Members of the union will not be used concurrently, only one technology at a time */
        rfalNfcaListenDevice   nfca;                /*!< NFC-A Listen Device instance */
        rfalNfcbListenDevice   nfcb;                /*!< NFC-B Listen Device instance */
        rfalNfcfListenDevice   nfcf;                /*!< NFC-F Listen Device instance */
        rfalNfcvListenDevice   nfcv;                /*!< NFC-V Listen Device instance *
        rfalSt25tbListenDevice st25tb;              /*!< ST25TB Listen Device instance*/
    }dev;                                         /*!< Device's instance            */
                                                    
    uint8_t                    *nfcid;              /*!< Device's NFCID               */
    uint8_t                    nfcidLen;            /*!< Device's NFCID length        */
    rfalNfcRfInterface         rfInterface;         /*!< Device's interface           */
    
    union{                              /*  PRQA S 0750 # MISRA 19.2 - Members of the union will not be used concurrently, only one protocol at a time */            
        rfalIsoDepDevice       isoDep;              /*!< ISO-DEP instance             */
        rfalNfcDepDevice       nfcDep;              /*!< NFC-DEP instance             */
    }proto;                                         /*!< Device's protocol            */
}rfalNfcDevice;


/*! Discovery parameters                                                                                           */
typedef struct{
    rfalComplianceMode compMode;                        /*!< Compliancy mode to be used                            */
    uint16_t           techs2Find;                      /*!< Technologies to search for                            */
    uint16_t           totalDuration;                   /*!< Duration of a whole Poll + Listen cycle               */
    uint8_t            devLimit;                        /*!< Max number of devices                                 */
    
    rfalBitRate        nfcfBR;                          /*!< Bit rate to poll for NFC-F                            */
    uint8_t            nfcid3[RFAL_NFCDEP_NFCID3_LEN];  /*!< NFCID3 to be used on the ATR_REQ/ATR_RES              */
    uint8_t            GB[RFAL_NFCDEP_GB_MAX_LEN];      /*!< General bytes to be used on the ATR-REQ               */
    uint8_t            GBLen;                           /*!< Length of the General Bytes                           */
    rfalBitRate        ap2pBR;                          /*!< Bit rate to poll for AP2P                             */
    
    rfalLmConfPA       lmConfigPA;                      /*!< Configuration for Passive Listen mode NFC-A           */
    rfalLmConfPF       lmConfigPF;                      /*!< Configuration for Passive Listen mode NFC-A           */
    
    void               (*notifyCb)( rfalNfcState st );  /*!< Callback to Notify upper layer                        */
                                                        
    bool               wakeupEnabled;                   /*!< Enable Wake-Up mode before polling                    */
    bool               wakeupConfigDefault;             /*!< Wake-Up mode default configuration                    */
    rfalWakeUpConfig   wakeupConfig;                    /*!< Wake-Up mode configuration                            */
}rfalNfcDiscoverParam;


/*! Buffer union, only one interface is used at a time                                                             */
typedef union{  /*  PRQA S 0750 # MISRA 19.2 - Members of the union will not be used concurrently, only one interface at a time */
    uint8_t                 rfBuf[RFAL_NFC_RF_BUF_LEN]; /*!< RF buffer                                             */
    rfalIsoDepBufFormat     isoDepBuf;                  /*!< ISO-DEP Tx buffer format (with header/prologue)       */
    rfalNfcDepBufFormat     nfcDepBuf;                  /*!< NFC-DEP Rx buffer format (with header/prologue)       */
}rfalNfcBuffer;

typedef struct{
    rfalNfcState            state;              /* Main state                                      */
    uint16_t                techsFound;         /* Technologies found bitmask                      */
    uint16_t                techs2do;           /* Technologies still to be performed              */
    rfalBitRate             ap2pBR;             /* Bit rate to poll for AP2P                       */
    uint8_t                 selDevIdx;          /* Selected device index                           */
    rfalNfcDevice           *activeDev;         /* Active device pointer                           */
    rfalNfcDiscoverParam    disc;               /* Discovery parameters pointer                    */
    rfalNfcDevice           devList[RFAL_NFC_MAX_DEVICES];   /*!< Location of device list          */
    uint8_t                 devCnt;             /* Decices found counter                           */
    uint32_t                discTmr;            /* Discovery Total duration timer                  */
    ReturnCode              dataExErr;          /* Last Data Exchange error                        */
    bool                    discRestart;        /* Restart discover after deactivation flag        */
    bool                    isRxChaining;       /* Flag indicating Other device is chaining        */
    uint32_t                lmMask;             /* Listen Mode mask                                */
    
    rfalNfcBuffer           txBuf;              /* Tx buffer for Data Exchange                     */
    rfalNfcBuffer           rxBuf;              /* Rx buffer for Data Exchange                     */
    uint16_t                rxLen;              /* Length of received data on Data Exchange        */
}rfalNfc;


#define NFCIP_ATR_RETRY_MAX             2U                              /*!< Max consecutive retrys of an ATR REQ with transm error*/

#define NFCIP_PSLPAY_LEN                (2U)                            /*!< PSL Payload length (BRS + FSL)                        */
#define NFCIP_PSLREQ_LEN                (3U + RFAL_NFCDEP_LEN_LEN)      /*!< PSL REQ length (incl LEN)                             */
#define NFCIP_PSLRES_LEN                (3U + RFAL_NFCDEP_LEN_LEN)      /*!< PSL RES length (incl LEN)                             */

#define NFCIP_ATRREQ_BUF_LEN            (RFAL_NFCDEP_ATRREQ_MAX_LEN + RFAL_NFCDEP_LEN_LEN) /*!< ATR REQ max length (incl LEN)     */
#define NFCIP_ATRRES_BUF_LEN            (RFAL_NFCDEP_ATRRES_MAX_LEN + RFAL_NFCDEP_LEN_LEN) /*!< ATR RES max length (incl LEN)     */

#define NFCIP_RLSREQ_LEN                (3U + RFAL_NFCDEP_LEN_LEN)      /*!< RLS REQ length (incl LEN)                             */
#define NFCIP_RLSRES_LEN                (3U + RFAL_NFCDEP_LEN_LEN)      /*!< RSL RES length (incl LEN)                             */
#define NFCIP_RLSRES_MIN                (2U + RFAL_NFCDEP_LEN_LEN)      /*!< Minimum length for a RLS RES (incl LEN)               */

#define NFCIP_DSLREQ_LEN                (3U + RFAL_NFCDEP_LEN_LEN)      /*!< DSL REQ length (incl LEN)                             */
#define NFCIP_DSLRES_LEN                (3U + RFAL_NFCDEP_LEN_LEN)      /*!< DSL RES length (incl LEN)                             */
#define NFCIP_DSLRES_MIN                (2U + RFAL_NFCDEP_LEN_LEN)      /*!< Minimum length for a DSL RES (incl LEN)               */

#define NFCIP_DSLRES_MAX_LEN            (3U + RFAL_NFCDEP_LEN_LEN)      /*!< Maximum length for a DSL RES (incl LEN)               */
#define NFCIP_RLSRES_MAX_LEN            (3U + RFAL_NFCDEP_LEN_LEN)      /*!< Minimum length for a RLS RES (incl LEN)               */
#define NFCIP_TARGET_RES_MAX            ( MAX( NFCIP_RLSRES_MAX_LEN, NFCIP_DSLRES_MAX_LEN) ) /*!< Max target control res length    */



#define NFCIP_NO_FWT                    RFAL_FWT_NONE                   /*!< No FWT value - Target Mode                            */
#define NFCIP_INIT_MIN_RTOX             1U                              /*!< Minimum RTOX value  Digital 1.0  14.8.4.1             */
#define NFCIP_INIT_MAX_RTOX             59U                             /*!< Maximum RTOX value  Digital 1.0  14.8.4.1             */

#define NFCIP_TARG_MIN_RTOX             1U                              /*!< Minimum target RTOX value  Digital 1.0  14.8.4.1      */
#define NFCIP_TARG_MAX_RTOX             59U                             /*!< Maximum target RTOX value  Digital 1.0  14.8.4.1      */

#define NFCIP_TRECOV                    1280U                           /*!< Digital 1.0  A.10  Trecov                             */
 
#define NFCIP_TIMEOUT_ADJUSTMENT        512U                            /*!< Timeout Adjustment to compensate timing from end of Tx to end of frame  */
#define NFCIP_RWT_ACTIVATION            (0x1000001U + NFCIP_TIMEOUT_ADJUSTMENT) /*!< Digital 2.0  B.10  RWT ACTIVATION  2^24 + RWT Delta + Adjustment*/
#define NFCIP_RWT_ACM_ACTIVATION        (0x200001U + NFCIP_TIMEOUT_ADJUSTMENT)  /*!< Digital 2.0  B.10  RWT ACTIVATION  2^21 + RWT Delta + Adjustment*/

#define RFAL_NFCDEP_HEADER_PAD          (RFAL_NFCDEP_DEPREQ_HEADER_LEN - RFAL_NFCDEP_LEN_MIN) /*!< Difference between expected rcvd header len and max foreseen */


#define NFCIP_MAX_TX_RETRYS             (uint8_t)3U      /*!< Number of retransmit retyrs                           */
#define NFCIP_MAX_TO_RETRYS             (uint8_t)3U      /*!< Number of retrys for Timeout                          */
#define NFCIP_MAX_RTOX_RETRYS           (uint8_t)3U      /*!< Number of retrys for RTOX                             */
#define NFCIP_MAX_NACK_RETRYS           (uint8_t)3U      /*!< Number of retrys for NACK                             */
#define NFCIP_MAX_ATN_RETRYS            (uint8_t)3U      /*!< Number of retrys for ATN                              */

#define NFCIP_MIN_TXERROR_LEN           4U               /*!< Minimum frame length with error to be ignored  Digital 1.0 14.12.5.4 */

#define NFCIP_REQ                       (uint8_t)0xD4U   /*!<NFCIP REQuest code                                     */
#define NFCIP_RES                       (uint8_t)0xD5U   /*!<NFCIP RESponce code                                    */

#define NFCIP_BS_MASK                   0x0FU            /*!< Bit mask for BS value on a ATR REQ/RES                */
#define NFCIP_BR_MASK                   NFCIP_BS_MASK    /*!< Bit mask for BR value on a ATR REQ/RES                */

#define NFCIP_PP_GB_MASK                0x02U            /*!< Bit mask for GB value in PP byte on a ATR REQ/RES     */
#define NFCIP_PP_NAD_MASK               0x01U            /*!< Bit mask for NAD value in PP byte on a ATR REQ/RES    */

#define NFCIP_PFB_xPDU_MASK             0xE0U            /*!< Bit mask for PDU type                                 */
#define NFCIP_PFB_IPDU                  0x00U            /*!< Bit mask indicating a Information PDU                 */
#define NFCIP_PFB_RPDU                  0x40U            /*!< Bit mask indicating a Response PDU                    */
#define NFCIP_PFB_SPDU                  0x80U            /*!< Bit mask indicating a Supervisory PDU                 */

#define NFCIP_PFB_MI_BIT                0x10U            /*!< Bit mask for the chaining bit (MI) of PFB             */
#define NFCIP_PFB_DID_BIT               0x04U            /*!< Bit mask for the DID presence bit of PFB              */
#define NFCIP_PFB_NAD_BIT               0x08U            /*!< Bit mask for the NAD presence bit of PFB              */
#define NFCIP_PFB_PNI_MASK              0x03U            /*!< Bit mask for the Packet Number Information            */

#define NFCIP_PFB_Rx_MASK               0x10U            /*!< Bit mask for the R-PDU type                           */
#define NFCIP_PFB_ACK                   0x00U            /*!< Bit mask for R-PDU indicating ACK                     */
#define NFCIP_PFB_NACK                  0x10U            /*!< Bit mask for R-PDU indicating NAK                     */

#define NFCIP_PFB_Sx_MASK               0x10U            /*!< Bit mask for the R-PDU type                           */
#define NFCIP_PFB_ATN                   0x00U            /*!< Bit mask for R-PDU indicating ACK                     */
#define NFCIP_PFB_TO                    0x10U            /*!< Bit mask for R-PDU indicating NAK                     */

#define NFCIP_PFB_INVALID               0xFFU            /*!< Invalid PFB value                                     */

/*
 ******************************************************************************
 * MACROS
 ******************************************************************************
 */

#define nfcipIsTransmissionError(e)    ( ((e) == ERR_CRC) || ((e) == ERR_FRAMING) || ((e) == ERR_PAR) ) /*!< Checks if is a Trasmission error */


#define nfcipConv1FcToMs( v )          (rfalConv1fcToMs((v)) + 1U)                                     /*!< Converts value v 1fc into milliseconds (fc=13.56)     */

#define nfcipCmdIsReq( cmd )           (((uint8_t)(cmd) % 2U) == 0U)                                    /*!< Checks if the nfcip cmd is a REQ                      */

#define nfcip_PFBhasDID( pfb )         ( ((pfb) & NFCIP_PFB_DID_BIT) == NFCIP_PFB_DID_BIT)              /*!< Checks if pfb is signalling DID                       */
#define nfcip_PFBhasNAD( pfb )         ( ((pfb) & NFCIP_PFB_NAD_BIT) == NFCIP_PFB_NAD_BIT)              /*!< Checks if pfb is signalling NAD                       */

#define nfcip_PFBisIPDU( pfb )         ( ((pfb) & NFCIP_PFB_xPDU_MASK) == NFCIP_PFB_IPDU)               /*!< Checks if pfb is a Information PDU                    */
#define nfcip_PFBisRPDU( pfb )         ( ((pfb) & NFCIP_PFB_xPDU_MASK) == NFCIP_PFB_RPDU)               /*!< Checks if pfb is Response PDU                         */
#define nfcip_PFBisSPDU( pfb )         ( ((pfb) & NFCIP_PFB_xPDU_MASK) == NFCIP_PFB_SPDU)               /*!< Checks if pfb is a Supervisory PDU                    */

#define nfcip_PFBisIMI( pfb )          ( nfcip_PFBisIPDU( pfb ) && (((pfb) & NFCIP_PFB_MI_BIT) == NFCIP_PFB_MI_BIT))  /*!< Checks if pfb is a Information PDU indicating MI chaining */

#define nfcip_PFBisRNACK( pfb )        ( nfcip_PFBisRPDU( pfb ) && (((pfb) & NFCIP_PFB_Rx_MASK) == NFCIP_PFB_NACK)) /*!< Checks if pfb is a R-PDU indicating NACK  */
#define nfcip_PFBisRACK( pfb )         ( nfcip_PFBisRPDU( pfb ) && (((pfb) & NFCIP_PFB_Rx_MASK) == NFCIP_PFB_ACK )) /*!< Checks if pfb is a R-PDU indicating ACK   */

#define nfcip_PFBisSATN( pfb )         ( nfcip_PFBisSPDU( pfb ) && (((pfb) & NFCIP_PFB_Sx_MASK) == NFCIP_PFB_ATN))  /*!< Checks if pfb is a R-PDU indicating ATN   */
#define nfcip_PFBisSTO( pfb )          ( nfcip_PFBisSPDU( pfb ) && (((pfb) & NFCIP_PFB_Sx_MASK) == NFCIP_PFB_TO) )  /*!< Checks if pfb is a R-PDU indicating TO    */


#define nfcip_PFBIPDU( pni )           ( (uint8_t)( 0x00U | NFCIP_PFB_IPDU | ((pni) & NFCIP_PFB_PNI_MASK) ))/*!< Returns a PFB I-PDU with the given packet number (pni)                   */
#define nfcip_PFBIPDU_MI( pni )        ( (uint8_t)(isoDep_PCBIBlock(pni) | NFCIP_PFB_MI_BIT))               /*!< Returns a PFB I-PDU with the given packet number (pni) indicating chaing */

#define nfcip_PFBRPDU( pni )           ( (uint8_t)( 0x00U | NFCIP_PFB_RPDU | ((pni) & NFCIP_PFB_PNI_MASK) ))/*!< Returns a PFB R-PDU with the given packet number (pni)                   */
#define nfcip_PFBRPDU_NACK( pni )      ( (uint8_t)(nfcip_PFBRPDU(pni) | NFCIP_PFB_NACK))                    /*!< Returns a PFB R-PDU with the given packet number (pni) indicating NACK   */
#define nfcip_PFBRPDU_ACK( pni )       ( (uint8_t)(nfcip_PFBRPDU(pni) | NFCIP_PFB_ACK))                     /*!< Returns a PFB R-PDU with the given packet number (pni) indicating ACK    */

#define nfcip_PFBSPDU()                ( (uint8_t)( 0x00U | NFCIP_PFB_SPDU ))                           /*!< Returns a PFB S-PDU                                   */
#define nfcip_PFBSPDU_ATN()            ( (uint8_t)(nfcip_PFBSPDU() | NFCIP_PFB_ATN))                    /*!< Returns a PFB S-PDU indicating ATN                    */
#define nfcip_PFBSPDU_TO()             ( (uint8_t)(nfcip_PFBSPDU() | NFCIP_PFB_TO))                     /*!< Returns a PFB S-PDU indicating TO                     */


#define nfcip_PNIInc( pni )            ( (uint8_t) (((pni)+1U) & NFCIP_PFB_PNI_MASK) )                  /*!< Returns a incremented PNI from the given (pni)        */
#define nfcip_PNIDec( pni )            ( (uint8_t) (((pni)-1U) & NFCIP_PFB_PNI_MASK) )                  /*!< Returns a decremented PNI from the given (pni)        */

#define nfcip_PBF_PNI( pfb )           ( (uint8_t) ((pfb) & NFCIP_PFB_PNI_MASK ))                       /*!< Returns the Packet Number Information (pni)           */

#define nfcip_PPwGB( lr )              ( rfalNfcDepLR2PP( lr ) | NFCIP_PP_GB_MASK)                      /*!< Returns a PP byte containing the given PP value indicating GB                  */

#define nfcip_DIDMax( did )            ( MIN( (did), RFAL_NFCDEP_DID_MAX) )                             /*!< Ensures that the given did has proper value  Digital 14.6.2.3 DID [0 14]       */
#define nfcip_RTOXTargMax( wt )        (uint8_t)( MIN( (RFAL_NFCDEP_RWT_TRG_MAX / rfalNfcDepWT2RWT(wt)), NFCIP_TARG_MAX_RTOX) )/*!< Calculates the Maximum RTOX value for the given wt as a Target */

#define nfcipIsInitiator( st )         ( ((st) >= NFCIP_ST_INIT_IDLE) && ((st) <= NFCIP_ST_INIT_RLS) )  /*!< Checks if module is set as Initiator                                           */
#define nfcipIsTarget( st )            (!nfcipIsInitiator(st))                                          /*!< Checks if module is set as Target                                              */

#define nfcipIsBRAllowed( br, mBR )    (((1U<<(br)) & (mBR)) != 0U)                                     /*!< Checks bit rate is allowed by given mask                                       */

#define nfcipIsEmptyDEPEnabled( op )   (!nfcipIsEmptyDEPDisabled(op))                                   /*!< Checks if empty payload is allowed by operation config  NCI 1.0 Table 81       */
#define nfcipIsEmptyDEPDisabled( op )  (((op) & RFAL_NFCDEP_OPER_EMPTY_DEP_DIS) != 0U)                  /*!< Checks if empty payload is not allowed by operation config  NCI 1.0 Table 81   */

#define nfcipIsRTOXReqEnabled( op )    (!nfcipIsRTOXReqDisabled(op))                                    /*!< Checks if send a RTOX_REQ is allowed by operation config  NCI 1.0 Table 81     */
#define nfcipIsRTOXReqDisabled( op )   (((op) & RFAL_NFCDEP_OPER_RTOX_REQ_DIS) != 0U)                   /*!< Checks if send a RTOX_REQ is not allowed by operation config  NCI 1.0 Table 81 */


/*! Checks if isDeactivating callback is set and calls it, otherwise returns false */
#define nfcipIsDeactivationPending()   ( (gNfcip.isDeactivating == NULL) ? false : gNfcip.isDeactivating() )
    
/*! Returns the RWT Activation according to the current communication mode */
#define nfcipRWTActivation()            ((gNfcip.cfg.commMode == RFAL_NFCDEP_COMM_ACTIVE) ? NFCIP_RWT_ACM_ACTIVATION : NFCIP_RWT_ACTIVATION)


#define nfcipRTOXAdjust( v )           ((v) - ((v)>>3))                                                 /*!< Adjust RTOX timer value to a percentage of the total, current 88% */ 

/*******************************************************************************/


/*! Digital 1.1 - 16.12.5.2  The Target SHALL NOT attempt any error recovery and remains in Rx mode upon Transmission or a Protocol Error */
#define nfcDepReEnableRx( rxB, rxBL, rxL )       RF->rfalTransceiveBlockingTx( NULL, 0, (rxB), (rxBL), (rxL), ( RFAL_TXRX_FLAGS_DEFAULT | (uint32_t)RFAL_TXRX_FLAGS_NFCIP1_ON ), RFAL_FWT_NONE )

/*
 ******************************************************************************
 * LOCAL DATA TYPES
 ******************************************************************************
 */

/*! Struct that holds all DEP parameters/configs for the following communications */
typedef struct{
    uint8_t   did;           /*!< Device ID (DID) to be used                      */
    
    uint8_t*  txBuf;         /*!< Pointer to the Tx buffer to be sent             */
    uint16_t  txBufLen;      /*!< Length of the data in the txBuf                 */
    uint8_t   txBufPaylPos;  /*!< Position inside txBuf where data starts         */
    bool      txChaining;    /*!< Flag indicating chaining on transmission        */
    
    uint8_t*  rxBuf;         /*!< Pointer to the Rx buffer for incoming data      */
    uint16_t  rxBufLen;      /*!< Length of the data in the rxBuf                 */
    uint8_t   rxBufPaylPos;  /*!< Position inside rxBuf where data is to be placed*/
    
    uint32_t  fwt;           /*!< Frame Waiting Time (FWT) to be used             */
    uint32_t  dFwt;          /*!< Delta Frame Waiting Time (dFWT) to be used      */
    uint16_t  fsc;           /*!< Frame Size (FSC) to be used                     */
    
} rfalNfcDepDEPParams;

/*! NFCIP module states */
typedef enum
{
    NFCIP_ST_IDLE,
    NFCIP_ST_INIT_IDLE,
    NFCIP_ST_INIT_ATR,
    NFCIP_ST_INIT_PSL,
    NFCIP_ST_INIT_DEP_IDLE,
    NFCIP_ST_INIT_DEP_TX,
    NFCIP_ST_INIT_DEP_RX,
    NFCIP_ST_INIT_DEP_ATN,
    NFCIP_ST_INIT_DSL,
    NFCIP_ST_INIT_RLS,
        
    NFCIP_ST_TARG_WAIT_ATR,
    NFCIP_ST_TARG_WAIT_ACTV,
    NFCIP_ST_TARG_DEP_IDLE,
    NFCIP_ST_TARG_DEP_RX,
    NFCIP_ST_TARG_DEP_RTOX,
    NFCIP_ST_TARG_DEP_TX,
    NFCIP_ST_TARG_DEP_SLEEP
} rfalNfcDepState;

/*! NFCIP commands (Request, Response) */
typedef enum{
    NFCIP_CMD_ATR_REQ = 0x00,
    NFCIP_CMD_ATR_RES = 0x01,
    NFCIP_CMD_WUP_REQ = 0x02,
    NFCIP_CMD_WUP_RES = 0x03,
    NFCIP_CMD_PSL_REQ = 0x04,
    NFCIP_CMD_PSL_RES = 0x05,
    NFCIP_CMD_DEP_REQ = 0x06,
    NFCIP_CMD_DEP_RES = 0x07,
    NFCIP_CMD_DSL_REQ = 0x08,
    NFCIP_CMD_DSL_RES = 0x09,
    NFCIP_CMD_RLS_REQ = 0x0A,
    NFCIP_CMD_RLS_RES = 0x0B
} rfalNfcDepCmd;


/*! Struct that holds all NFCIP data */
typedef struct{  
  rfalNfcDepConfigs       cfg;               /*!< Holds the current configuration to be used    */
  
  rfalNfcDepState         state;             /*!< Current state of the NFCIP module             */
  uint8_t                 pni;               /*!< Packet Number Information (PNI) counter       */
  
  uint8_t                 lastCmd;           /*!< Last command sent                             */
  uint8_t                 lastPFB;           /*!< Last PFB sent                                 */
  uint8_t                 lastPFBnATN;       /*!< Last PFB sent (excluding  ATN)                */
  uint8_t                 lastRTOX;          /*!< Last RTOX value sent                          */
  
  uint8_t                 cntTxRetrys;       /*!< Retransmissions counter                       */
  uint8_t                 cntTORetrys;       /*!< Timeouts counter                              */
  uint8_t                 cntRTOXRetrys;     /*!< RTOX counter                                  */
  uint8_t                 cntNACKRetrys;     /*!< NACK counter                                  */
  uint8_t                 cntATNRetrys;      /*!< Attention (ATN) counter                       */
  
  uint16_t                fsc;               /*!< Current Frame Size (FSC) to be used           */
  bool                    isTxChaining;      /*!< Flag for chaining on Transmission             */
  bool                    isRxChaining;      /*!< Flag for chaining on Reception                */
  uint8_t*                txBuf;             /*!< Pointer to the Tx buffer to be sent           */
  uint8_t*                rxBuf;             /*!< Pointer to the Rx buffer for incoming data    */
  uint16_t                txBufLen;          /*!< Length of the data in the txBuf               */
  uint16_t                rxBufLen;          /*!< Length of rxBuf buffer                        */
  uint16_t*               rxRcvdLen;         /*!< Length of the data in the rxBuf               */
  uint8_t                 txBufPaylPos;      /*!< Position in txBuf where data starts           */
  uint8_t                 rxBufPaylPos;      /*!< Position in rxBuf where data is to be placed  */
  bool                    *isChaining;       /*!< Flag for chaining on Reception                */
  
  rfalNfcDepDevice        *nfcDepDev;        /*!< Pointer to NFC-DEP device info                */

  uint32_t                RTOXTimer;         /*!< Timer used for RTOX                           */  
  rfalNfcDepDeactCallback isDeactivating;    /*!< Deactivating flag check callback              */
  
  bool                    isReqPending;      /*!< Flag pending REQ from Target activation       */
  bool                    isTxPending;       /*!< Flag pending DEP Block while waiting RTOX Ack */
  bool                    isWait4RTOX;       /*!< Flag for waiting RTOX Ack                     */
}rfalNfcDep;

/* CLASS */
class rfal_nfc{
	public:
    /* FUNCTION */
		rfal_nfc(rfal_rf *rfal); //funzione che imposta quale componente utilizziamo (DA FARE) e inizializza SPI
    ReturnCode rfalNfcInitialize();
    void CheckInterrupts();  //richiama check interrupts rfal_rf
    void rfalNfcWorker();    
    ReturnCode rfalNfcGetDevicesFound( rfalNfcDevice **devList, uint8_t *devCnt );
    ReturnCode rfalNfcSelect( uint8_t devIdx );
    ReturnCode rfalNfcDeactivate( bool discovery );
    ReturnCode rfalNfcDeactivation( void );
    ReturnCode rfalNfcDepRLS( void );
    ReturnCode rfalNfcDiscover( const rfalNfcDiscoverParam *disParams );
    rfalNfcState rfalNfcGetState( void );
    ReturnCode rfalNfcGetActiveDevice( rfalNfcDevice **dev );
    ReturnCode rfalNfcDataExchangeStart( uint8_t *txData, uint16_t txDataLen, uint8_t **rxData, uint16_t **rvdLen, uint32_t fwt );
    ReturnCode rfalNfcDataExchangeGetStatus( void );
    ReturnCode rfalNfcDepGetTransceiveStatus( void );
    bool nfcipTimerisExpired( uint32_t timer );
    
    /*NFC_DEP*/
    ReturnCode nfcipTxRx( rfalNfcDepCmd cmd, uint8_t* txBuf, uint32_t fwt, uint8_t* paylBuf, uint8_t paylBufLen, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t *rxActLen );
    ReturnCode nfcipTx( rfalNfcDepCmd cmd, uint8_t* txBuf, uint8_t *paylBuf, uint16_t paylLen, uint8_t pfbData, uint32_t fwt );
    ReturnCode nfcipDataRx( bool blocking );
    ReturnCode nfcipDataTx( uint8_t* txBuf, uint16_t txBufLen, uint32_t fwt );
    ReturnCode rfalNfcDepStartTransceive( rfalNfcDepTxRxParam *param );
    void nfcipSetDEPParams( rfalNfcDepDEPParams *DEPParams );
    void nfcipClearCounters( void );
    ReturnCode nfcipRun( uint16_t *outActRxLen, bool *outIsChaining  );
    ReturnCode nfcipInitiatorHandleDEP( ReturnCode rxRes, uint16_t rxLen, uint16_t *outActRxLen, bool *outIsChaining );
    ReturnCode nfcipDEPControlMsg( uint8_t pfb, uint8_t RTOX  );
    ReturnCode nfcipTargetHandleRX( ReturnCode rxRes, uint16_t *outActRxLen, bool *outIsChaining );
    
    /* PARAMETER */
    rfalNfcDiscoverParam discParam;
    
  private:    
    rfal_rf* RF;
    rfalNfc gNfcDev;
    rfalNfcDep gNfcip;                    /*!< NFCIP module instance                         */
};
#endif
