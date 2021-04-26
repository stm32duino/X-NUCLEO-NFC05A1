#ifndef RFAL_RF_H
#define RFAL_RF_H

/* INCLUDES */
#include "rfal_error.h"		//GESTIORE ERRORI
#include "Arduino.h"
#include "SPI.h"

/* DEFINE */
#define RFAL_VERSION                               (uint32_t)0x02000aU                          /*!< RFAL Current Version: v2.0.10                     */

#define RFAL_FWT_NONE                              0xFFFFFFFFU                                  /*!< Disabled FWT: Wait forever for a response         */
#define RFAL_GT_NONE                               RFAL_TIMING_NONE                             /*!< Disabled GT: No GT will be applied after Field On */

#define RFAL_TIMING_NONE                           0x00U                                        /*!< Timing disabled | Don't apply                     */

#define RFAL_1FC_IN_4096FC                         (uint32_t)4096U                              /*!< Number of 1/fc cycles in one 4096/fc              */
#define RFAL_1FC_IN_512FC                          (uint32_t)512U                               /*!< Number of 1/fc cycles in one 512/fc               */
#define RFAL_1FC_IN_64FC                           (uint32_t)64U                                /*!< Number of 1/fc cycles in one 64/fc                */
#define RFAL_1FC_IN_8FC                            (uint32_t)8U                                 /*!< Number of 1/fc cycles in one 8/fc                 */
#define RFAL_US_IN_MS                              (uint32_t)1000U                              /*!< Number of us in one ms                            */
#define RFAL_1MS_IN_1FC                            (uint32_t)13560U                             /*!< Number of 1/fc cycles in 1ms                      */
#define RFAL_BITS_IN_BYTE                          (uint16_t)8U                                 /*!< Number of bits in one byte                        */

#define RFAL_CRC_LEN                               2U                                           /*!< RF CRC LEN                                        */

/*! Default TxRx flags: Tx CRC automatic, Rx CRC removed, NFCIP1 mode off, AGC On, Tx Parity automatic, Rx Parity removed */
#define RFAL_TXRX_FLAGS_DEFAULT                    ( (uint32_t)RFAL_TXRX_FLAGS_CRC_TX_AUTO | (uint32_t)RFAL_TXRX_FLAGS_CRC_RX_REMV | (uint32_t)RFAL_TXRX_FLAGS_NFCIP1_OFF | (uint32_t)RFAL_TXRX_FLAGS_AGC_ON | (uint32_t)RFAL_TXRX_FLAGS_PAR_RX_REMV | (uint32_t)RFAL_TXRX_FLAGS_PAR_TX_AUTO | (uint32_t)RFAL_TXRX_FLAGS_NFCV_FLAG_AUTO)



#define RFAL_LM_MASK_NFCA                          (1U<<(uint8_t)RFAL_MODE_LISTEN_NFCA)         /*!< Bitmask for Listen Mode enabling Listen NFCA      */
#define RFAL_LM_MASK_NFCB                          (1U<<(uint8_t)RFAL_MODE_LISTEN_NFCB)         /*!< Bitmask for Listen Mode enabling Listen NFCB      */
#define RFAL_LM_MASK_NFCF                          (1U<<(uint8_t)RFAL_MODE_LISTEN_NFCF)         /*!< Bitmask for Listen Mode enabling Listen NFCF      */
#define RFAL_LM_MASK_ACTIVE_P2P                    (1U<<(uint8_t)RFAL_MODE_LISTEN_ACTIVE_P2P)   /*!< Bitmask for Listen Mode enabling Listen AP2P      */

#define RFAL_LM_SENS_RES_LEN                       2U                                           /*!< NFC-A SENS_RES (ATQA) length                      */
#define RFAL_LM_SENSB_RES_LEN                      13U                                          /*!< NFC-B SENSB_RES (ATQB) length                     */
#define RFAL_LM_SENSF_RES_LEN                      19U                                          /*!< NFC-F SENSF_RES  length                           */
#define RFAL_LM_SENSF_SC_LEN                       2U                                           /*!< NFC-F System Code length                          */

#define RFAL_NFCID3_LEN                            10U                                          /*!< NFCID3 length                                     */
#define RFAL_NFCID2_LEN                            8U                                           /*!< NFCID2 length                                     */
#define RFAL_NFCID1_TRIPLE_LEN                     10U                                          /*!< NFCID1 length                                     */
#define RFAL_NFCID1_DOUBLE_LEN                     7U                                           /*!< NFCID1 length                                     */
#define RFAL_NFCID1_SIMPLE_LEN                     4U       

/* MACROS */
/*! Returns the maximum supported bit rate for RW mode. Caller must check if mode is supported before, as even if mode is not supported will return the min  */
#define rfalGetMaxBrRW()                     ( ((RFAL_SUPPORT_BR_RW_6780)  ? RFAL_BR_6780 : ((RFAL_SUPPORT_BR_RW_3390)  ? RFAL_BR_3390 : ((RFAL_SUPPORT_BR_RW_1695)  ? RFAL_BR_1695 : ((RFAL_SUPPORT_BR_RW_848)  ? RFAL_BR_848 : ((RFAL_SUPPORT_BR_RW_424)  ? RFAL_BR_424 : ((RFAL_SUPPORT_BR_RW_212)  ? RFAL_BR_212 : RFAL_BR_106 ) ) ) ) ) ) )

/*! Returns the maximum supported bit rate for AP2P mode. Caller must check if mode is supported before, as even if mode is not supported will return the min  */
#define rfalGetMaxBrAP2P()                   ( ((RFAL_SUPPORT_BR_AP2P_848) ? RFAL_BR_848  : ((RFAL_SUPPORT_BR_AP2P_424) ? RFAL_BR_424  : ((RFAL_SUPPORT_BR_AP2P_212) ? RFAL_BR_212  : RFAL_BR_106 ) ) ) )

/*! Returns the maximum supported bit rate for CE-A mode. Caller must check if mode is supported before, as even if mode is not supported will return the min  */
#define rfalGetMaxBrCEA()                    ( ((RFAL_SUPPORT_BR_CE_A_848) ? RFAL_BR_848  : ((RFAL_SUPPORT_BR_CE_A_424) ? RFAL_BR_424  : ((RFAL_SUPPORT_BR_CE_A_212) ? RFAL_BR_212  : RFAL_BR_106 ) ) ) )

/*! Returns the maximum supported bit rate for CE-B mode. Caller must check if mode is supported before, as even if mode is not supported will return the min  */
#define rfalGetMaxBrCEB()                    ( ((RFAL_SUPPORT_BR_CE_B_848) ? RFAL_BR_848  : ((RFAL_SUPPORT_BR_CE_B_424) ? RFAL_BR_424  : ((RFAL_SUPPORT_BR_CE_B_212) ? RFAL_BR_212  : RFAL_BR_106 ) ) ) )

/*! Returns the maximum supported bit rate for CE-F mode. Caller must check if mode is supported before, as even if mode is not supported will return the min  */
#define rfalGetMaxBrCEF()                    ( ((RFAL_SUPPORT_BR_CE_F_424) ? RFAL_BR_424  : RFAL_BR_212 ) )


#define rfalIsModeActiveComm( md )           ( ((md) == RFAL_MODE_POLL_ACTIVE_P2P) || ((md) == RFAL_MODE_LISTEN_ACTIVE_P2P) )                          /*!< Checks if mode md is Active Communication  */
#define rfalIsModePassiveComm( md )          ( !rfalIsModeActiveComm(md) )                                                                             /*!< Checks if mode md is Passive Communication */
#define rfalIsModePassiveListen( md )        ( ((md) == RFAL_MODE_LISTEN_NFCA) || ((md) == RFAL_MODE_LISTEN_NFCB) || ((md) == RFAL_MODE_LISTEN_NFCF) ) /*!< Checks if mode md is Passive Listen        */
#define rfalIsModePassivePoll( md )          ( rfalIsModePassiveComm(md) && !rfalIsModePassiveListen(md) )                                             /*!< Checks if mode md is Passive Poll          */


#define rfalConv1fcTo8fc( t )                (uint32_t)( (uint32_t)(t) / RFAL_1FC_IN_8FC )                               /*!< Converts the given t from 1/fc to 8/fc     */
#define rfalConv8fcTo1fc( t )                (uint32_t)( (uint32_t)(t) * RFAL_1FC_IN_8FC )                               /*!< Converts the given t from 8/fc to 1/fc     */

#define rfalConv1fcTo64fc( t )               (uint32_t)( (uint32_t)(t) / RFAL_1FC_IN_64FC )                              /*!< Converts the given t from 1/fc  to 64/fc   */
#define rfalConv64fcTo1fc( t )               (uint32_t)( (uint32_t)(t) * RFAL_1FC_IN_64FC )                              /*!< Converts the given t from 64/fc to 1/fc    */

#define rfalConv1fcTo512fc( t )              (uint32_t)( (uint32_t)(t) / RFAL_1FC_IN_512FC )                             /*!< Converts the given t from 1/fc  to 512/fc  */
#define rfalConv512fcTo1fc( t )              (uint32_t)( (uint32_t)(t) * RFAL_1FC_IN_512FC )                             /*!< Converts the given t from 512/fc to 1/fc   */

#define rfalConv1fcTo4096fc( t )             (uint32_t)( (uint32_t)(t) / RFAL_1FC_IN_4096FC )                            /*!< Converts the given t from 1/fc to 4096/fc  */
#define rfalConv4096fcTo1fc( t )             (uint32_t)( (uint32_t)(t) * RFAL_1FC_IN_4096FC )                            /*!< Converts the given t from 4096/fc to 1/fc  */

#define rfalConv1fcToMs( t )                 (uint32_t)( (uint32_t)(t) / RFAL_1MS_IN_1FC )                               /*!< Converts the given t from 1/fc to ms       */
#define rfalConvMsTo1fc( t )                 (uint32_t)( (uint32_t)(t) * RFAL_1MS_IN_1FC )                               /*!< Converts the given t from ms to 1/fc       */

#define rfalConv1fcToUs( t )                 (uint32_t)( ((uint32_t)(t) * RFAL_US_IN_MS) / RFAL_1MS_IN_1FC)              /*!< Converts the given t from 1/fc to us       */
#define rfalConvUsTo1fc( t )                 (uint32_t)( ((uint32_t)(t) * RFAL_1MS_IN_1FC) / RFAL_US_IN_MS)              /*!< Converts the given t from us to 1/fc       */

#define rfalConv64fcToMs( t )                (uint32_t)( (uint32_t)(t) / (RFAL_1MS_IN_1FC / RFAL_1FC_IN_64FC) )          /*!< Converts the given t from 64/fc to ms      */
#define rfalConvMsTo64fc( t )                (uint32_t)( (uint32_t)(t) * (RFAL_1MS_IN_1FC / RFAL_1FC_IN_64FC) )          /*!< Converts the given t from ms to 64/fc      */

#define rfalConvBitsToBytes( n )             (uint16_t)( ((uint16_t)(n)+(RFAL_BITS_IN_BYTE-1U)) / (RFAL_BITS_IN_BYTE) )  /*!< Converts the given n from bits to bytes    */
#define rfalConvBytesToBits( n )             (uint32_t)( (uint32_t)(n) * (RFAL_BITS_IN_BYTE) )                           /*!< Converts the given n from bytes to bits    */


/* UTILS MACRO */
#define SIZEOF_ARRAY(a)     (sizeof(a) / sizeof((a)[0]))  /*!< Compute the size of an array           */
#define MAX(a, b)           (((a) > (b)) ? (a) : (b))     /*!< Return the maximum of the 2 values     */
#define MIN(a, b)           (((a) < (b)) ? (a) : (b))     /*!< Return the minimum of the 2 values     */
#define BITMASK_1           (0x01)                        /*!< Bit mask for lsb bit                   */
#define BITMASK_2           (0x03)                        /*!< Bit mask for two lsb bits              */
#define BITMASK_3           (0x07)                        /*!< Bit mask for three lsb bits            */
#define BITMASK_4           (0x0F)                        /*!< Bit mask for four lsb bits             */
#define U16TOU8(a)          ((a) & 0x00FF)                /*!< Cast 16-bit unsigned to 8-bit unsigned */
#define GETU16(a)           (((uint16_t)(a)[0] << 8) | (uint16_t)(a)[1])/*!< Cast two Big Endian 8-bits byte array to 16-bits unsigned */
#define GETU32(a)           (((uint32_t)(a)[0] << 24) | ((uint32_t)(a)[1] << 16) | ((uint32_t)(a)[2] << 8) | ((uint32_t)(a)[3])) /*!< Cast four Big Endian 8-bit byte array to 32-bit unsigned */

/*! Computes a Transceive context \a ctx with default flags and the lengths 
 * in bytes with the given arguments
 *    \a ctx   : Transceive context to be assigned  
 *    \a tB    : txBuf the pointer to the buffer to be sent
 *    \a tBL   : txBuf length in bytes
 *    \a rB    : rxBuf the pointer to the buffer to place the received frame
 *    \a rBL   : rxBuf length in bytes
 *    \a rBL   : rxBuf length in bytes
 *    \a t     : FWT to be used on this transceive in 1/fc
 */
#define rfalCreateByteTxRxContext( ctx, tB, tBL, rB, rBL, rdL, t ) \
    (ctx).txBuf     = (uint8_t*)(tB);                                      \
    (ctx).txBufLen  = (uint16_t)rfalConvBytesToBits(tBL);                  \
    (ctx).rxBuf     = (uint8_t*)(rB);                                      \
    (ctx).rxBufLen  = (uint16_t)rfalConvBytesToBits(rBL);                  \
    (ctx).rxRcvdLen = (uint16_t*)(rdL);                                    \
    (ctx).flags     = (uint32_t)RFAL_TXRX_FLAGS_DEFAULT;                   \
    (ctx).fwt       = (uint32_t)(t);


/*! Computes a Transceive context \a ctx using lengths in bytes 
 * with the given flags and arguments
 *    \a ctx   : Transceive context to be assigned  
 *    \a tB    : txBuf the pointer to the buffer to be sent
 *    \a tBL   : txBuf length in bytes
 *    \a rB    : rxBuf the pointer to the buffer to place the received frame
 *    \a rBL   : rxBuf length in bytes
 *    \a rBL   : rxBuf length in bytes
 *    \a t     : FWT to be used on this transceive in 1/fc
 */
#define rfalCreateByteFlagsTxRxContext( ctx, tB, tBL, rB, rBL, rdL, fl, t ) \
    (ctx).txBuf     = (uint8_t*)(tB);                                       \
    (ctx).txBufLen  = (uint16_t)rfalConvBytesToBits(tBL);                   \
    (ctx).rxBuf     = (uint8_t*)(rB);                                       \
    (ctx).rxBufLen  = (uint16_t)rfalConvBytesToBits(rBL);                   \
    (ctx).rxRcvdLen = (uint16_t*)(rdL);                                     \
    (ctx).flags     = (uint32_t)(fl);                                       \
    (ctx).fwt       = (uint32_t)(t);


#define rfalLogE(...)             platformLog(__VA_ARGS__)        /*!< Macro for the error log method                  */
#define rfalLogW(...)             platformLog(__VA_ARGS__)        /*!< Macro for the warning log method                */
#define rfalLogI(...)             platformLog(__VA_ARGS__)        /*!< Macro for the info log method                   */
#define rfalLogD(...)             platformLog(__VA_ARGS__)        /*!< Macro for the debug log method                  */

/* ENUM */

/* RFAL Guard Time (GT) default values                 */
#define    RFAL_GT_NFCA                      rfalConvMsTo1fc(5U)     /*!< GTA  Digital 2.0  6.10.4.1 & B.2                                                                 */
#define    RFAL_GT_NFCB                      rfalConvMsTo1fc(5U)     /*!< GTB  Digital 2.0  7.9.4.1  & B.3                                                                 */
#define    RFAL_GT_NFCF                      rfalConvMsTo1fc(20U)    /*!< GTF  Digital 2.0  8.7.4.1  & B.4                                                                 */
#define    RFAL_GT_NFCV                      rfalConvMsTo1fc(5U)     /*!< GTV  Digital 2.0  9.7.5.1  & B.5                                                                 */
#define    RFAL_GT_PICOPASS                  rfalConvMsTo1fc(1U)     /*!< GT Picopass                                                                                      */
#define    RFAL_GT_AP2P                      rfalConvMsTo1fc(5U)     /*!< TIRFG  Ecma 340  11.1.1                                                                          */
#define    RFAL_GT_AP2P_ADJUSTED             rfalConvMsTo1fc(5U+25U) /*!< Adjusted GT for greater interoperability (Sony XPERIA P, Nokia N9, Huawei P2)                    */

/* RFAL Frame Delay Time (FDT) Listen default values   */
#define    RFAL_FDT_LISTEN_NFCA_POLLER       1172U    /*!< FDTA,LISTEN,MIN (n=9) Last bit: Logic "1" - tnn,min/2 Digital 1.1  6.10 ;  EMV CCP Spec Book D v2.01  4.8.1.3   */
#define    RFAL_FDT_LISTEN_NFCB_POLLER       1008U    /*!< TR0B,MIN         Digital 1.1  7.1.3 & A.3  ; EMV CCP Spec Book D v2.01  4.8.1.3 & Table A.5                     */
#define    RFAL_FDT_LISTEN_NFCF_POLLER       2672U    /*!< TR0F,LISTEN,MIN  Digital 1.1  8.7.1.1 & A.4                                                                     */
#define    RFAL_FDT_LISTEN_NFCV_POLLER       4310U    /*!< FDTV,LISTEN,MIN  t1 min       Digital 2.1  B.5  ;  ISO15693-3 2009  9.1                                          */
#define    RFAL_FDT_LISTEN_PICOPASS_POLLER   3400U    /*!< ISO15693 t1 min - observed adjustment                                                                           */
#define    RFAL_FDT_LISTEN_AP2P_POLLER       64U      /*!< FDT AP2P No actual FDTListen is required as fields switch and collision avoidance                               */
#define    RFAL_FDT_LISTEN_NFCA_LISTENER     1172U    /*!< FDTA,LISTEN,MIN  Digital 1.1  6.10                                                                              */
#define    RFAL_FDT_LISTEN_NFCB_LISTENER     1024U    /*!< TR0B,MIN         Digital 1.1  7.1.3 & A.3  ;  EMV CCP Spec Book D v2.01  4.8.1.3 & Table A.5                    */
#define    RFAL_FDT_LISTEN_NFCF_LISTENER     2688U    /*!< TR0F,LISTEN,MIN  Digital 1.1  8.7.1.1 & A.4                                                                     */
#define    RFAL_FDT_LISTEN_AP2P_LISTENER     64U      /*!< FDT AP2P No actual FDTListen exists as fields switch and collision avoidance                                    */

/*  RFAL Frame Delay Time (FDT) Poll default values    */
#define    RFAL_FDT_POLL_NFCA_POLLER         6780U    /*!< FDTA,POLL,MIN   Digital 1.1  6.10.3.1 & A.2                                                                     */
#define    RFAL_FDT_POLL_NFCA_T1T_POLLER     384U     /*!< RRDDT1T,MIN,B1  Digital 1.1  10.7.1 & A.5                                                                       */
#define    RFAL_FDT_POLL_NFCB_POLLER         6780U    /*!< FDTB,POLL,MIN = TR2B,MIN,DEFAULT Digital 1.1 7.9.3 & A.3  ;  EMVCo 3.0 FDTB,PCD,MIN  Table A.5                  */
#define    RFAL_FDT_POLL_NFCF_POLLER         2672U    /*!< FDTF,POLL,MIN   Digital 1.1  8.7.3 & A.4                                                                        */
#define    RFAL_FDT_POLL_NFCV_POLLER         4192U    /*!< FDTV,POLL  Digital 2.1  9.7.3.1  & B.5                                                                          */
#define    RFAL_FDT_POLL_PICOPASS_POLLER     1790U    /*!< FDT Max                                                                                                         */
#define    RFAL_FDT_POLL_AP2P_POLLER         0U       /*!< FDT AP2P No actual FDTPoll exists as fields switch and collision avoidance                                      */

/* TYPES */

/*! RFAL modes    */
typedef enum {
    RFAL_MODE_NONE                   = 0,    /*!< No mode selected/defined                                         */
    RFAL_MODE_POLL_NFCA              = 1,    /*!< Mode to perform as NFCA (ISO14443A) Poller (PCD)                 */
    RFAL_MODE_POLL_NFCA_T1T          = 2,    /*!< Mode to perform as NFCA T1T (Topaz) Poller (PCD)                 */
    RFAL_MODE_POLL_NFCB              = 3,    /*!< Mode to perform as NFCB (ISO14443B) Poller (PCD)                 */
    RFAL_MODE_POLL_B_PRIME           = 4,    /*!< Mode to perform as B' Calypso (Innovatron) (PCD)                 */
    RFAL_MODE_POLL_B_CTS             = 5,    /*!< Mode to perform as CTS Poller (PCD)                              */
    RFAL_MODE_POLL_NFCF              = 6,    /*!< Mode to perform as NFCF (FeliCa) Poller (PCD)                    */
    RFAL_MODE_POLL_NFCV              = 7,    /*!< Mode to perform as NFCV (ISO15963) Poller (PCD)                  */
    RFAL_MODE_POLL_PICOPASS          = 8,    /*!< Mode to perform as PicoPass / iClass Poller (PCD)                */
    RFAL_MODE_POLL_ACTIVE_P2P        = 9,    /*!< Mode to perform as Active P2P (ISO18092) Initiator               */
    RFAL_MODE_LISTEN_NFCA            = 10,   /*!< Mode to perform as NFCA (ISO14443A) Listener (PICC)              */
    RFAL_MODE_LISTEN_NFCB            = 11,   /*!< Mode to perform as NFCA (ISO14443B) Listener (PICC)              */
    RFAL_MODE_LISTEN_NFCF            = 12,   /*!< Mode to perform as NFCA (ISO15963) Listener (PICC)               */
    RFAL_MODE_LISTEN_ACTIVE_P2P      = 13    /*!< Mode to perform as Active P2P (ISO18092) Target                  */
} rfalMode;


/*! RFAL Bit rates    */
typedef enum {
    RFAL_BR_106                      = 0,    /*!< Bit Rate 106 kbit/s (fc/128)                                     */
    RFAL_BR_212                      = 1,    /*!< Bit Rate 212 kbit/s (fc/64)                                      */
    RFAL_BR_424                      = 2,    /*!< Bit Rate 424 kbit/s (fc/32)                                      */
    RFAL_BR_848                      = 3,    /*!< Bit Rate 848 kbit/s (fc/16)                                      */
    RFAL_BR_1695                     = 4,    /*!< Bit Rate 1695 kbit/s (fc/8)                                      */
    RFAL_BR_3390                     = 5,    /*!< Bit Rate 3390 kbit/s (fc/4)                                      */
    RFAL_BR_6780                     = 6,    /*!< Bit Rate 6780 kbit/s (fc/2)                                      */
    RFAL_BR_13560                    = 7,    /*!< Bit Rate 13560 kbit/s (fc)                                       */
    RFAL_BR_52p97                    = 0xEB, /*!< Bit Rate 52.97 kbit/s (fc/256) Fast Mode VICC->VCD               */
    RFAL_BR_26p48                    = 0xEC, /*!< Bit Rate 26,48 kbit/s (fc/512) NFCV VICC->VCD & VCD->VICC 1of4   */
    RFAL_BR_1p66                     = 0xED, /*!< Bit Rate 1,66 kbit/s (fc/8192) NFCV VCD->VICC 1of256             */
    RFAL_BR_KEEP                     = 0xFF  /*!< Value indicating to keep the same previous bit rate              */
} rfalBitRate;


/*! RFAL Compliance modes for upper modules  */
typedef enum {
    RFAL_COMPLIANCE_MODE_NFC,                /*!< Perform with NFC Forum 1.1 compliance                            */
    RFAL_COMPLIANCE_MODE_EMV,                /*!< Perform with EMVCo compliance                                    */
    RFAL_COMPLIANCE_MODE_ISO                 /*!< Perform with ISO10373 compliance                                 */
}rfalComplianceMode;


/*! RFAL main states flags    */
typedef enum {
    RFAL_STATE_IDLE                  = 0,
    RFAL_STATE_INIT                  = 1,
    RFAL_STATE_MODE_SET              = 2,
    
    RFAL_STATE_TXRX                  = 3,
    RFAL_STATE_LM                    = 4,
    RFAL_STATE_WUM                   = 5
    
} rfalState;

/*! RFAL transceive states    */
typedef enum {
    RFAL_TXRX_STATE_IDLE             = 0,
    RFAL_TXRX_STATE_INIT             = 1,
    RFAL_TXRX_STATE_START            = 2,
        
    RFAL_TXRX_STATE_TX_IDLE          = 11,
    RFAL_TXRX_STATE_TX_WAIT_GT       = 12,
    RFAL_TXRX_STATE_TX_WAIT_FDT      = 13,
    RFAL_TXRX_STATE_TX_TRANSMIT      = 14,
    RFAL_TXRX_STATE_TX_WAIT_WL       = 15,
    RFAL_TXRX_STATE_TX_RELOAD_FIFO   = 16,
    RFAL_TXRX_STATE_TX_WAIT_TXE      = 17,
    RFAL_TXRX_STATE_TX_DONE          = 18,
    RFAL_TXRX_STATE_TX_FAIL          = 19,
    
    RFAL_TXRX_STATE_RX_IDLE          = 81,
    RFAL_TXRX_STATE_RX_WAIT_EON      = 82,
    RFAL_TXRX_STATE_RX_WAIT_RXS      = 83,
    RFAL_TXRX_STATE_RX_WAIT_RXE      = 84,
    RFAL_TXRX_STATE_RX_READ_FIFO     = 85,
    RFAL_TXRX_STATE_RX_ERR_CHECK     = 86,
    RFAL_TXRX_STATE_RX_READ_DATA     = 87,
    RFAL_TXRX_STATE_RX_WAIT_EOF      = 88,
    RFAL_TXRX_STATE_RX_DONE          = 89,
    RFAL_TXRX_STATE_RX_FAIL          = 90,
    
} rfalTransceiveState;


/*! RFAL transceive flags                                                                                                                    */
enum {
    RFAL_TXRX_FLAGS_CRC_TX_AUTO      = (0U<<0),   /*!< CRC will be generated automatic upon transmission                                     */
    RFAL_TXRX_FLAGS_CRC_TX_MANUAL    = (1U<<0),   /*!< CRC was calculated manually, included in txBuffer                                     */
    RFAL_TXRX_FLAGS_CRC_RX_KEEP      = (1U<<1),   /*!< Upon Reception keep the CRC in rxBuffer (reflected on rcvd length)                    */
    RFAL_TXRX_FLAGS_CRC_RX_REMV      = (0U<<1),   /*!< Upon Reception remove the CRC from rxBuffer                                           */
    RFAL_TXRX_FLAGS_NFCIP1_ON        = (1U<<2),   /*!< Enable NFCIP1 mode: Add SB(F0) and LEN bytes during Tx and skip SB(F0) byte during Rx */
    RFAL_TXRX_FLAGS_NFCIP1_OFF       = (0U<<2),   /*!< Disable NFCIP1 mode: do not append protocol bytes while Tx nor skip while Rx          */
    RFAL_TXRX_FLAGS_AGC_OFF          = (1U<<3),   /*!< Disable Automatic Gain Control, improving multiple devices collision detection        */
    RFAL_TXRX_FLAGS_AGC_ON           = (0U<<3),   /*!< Enable Automatic Gain Control, improving single device reception                      */
    RFAL_TXRX_FLAGS_PAR_RX_KEEP      = (1U<<4),   /*!< Disable Parity and CRC check and keep the Parity and CRC bits in the received buffer  */
    RFAL_TXRX_FLAGS_PAR_RX_REMV      = (0U<<0),   /*!< Enable Parity check and remove the parity bits from the received buffer               */
    RFAL_TXRX_FLAGS_PAR_TX_NONE      = (1U<<5),   /*!< Disable automatic Parity generation (ISO14443A) and use the one provided in the buffer*/
    RFAL_TXRX_FLAGS_PAR_TX_AUTO      = (0U<<5),   /*!< Enable automatic Parity generation (ISO14443A)                                        */
    RFAL_TXRX_FLAGS_NFCV_FLAG_MANUAL = (1U<<6),   /*!< Disable automatic adaption of flag byte (ISO15693) according to current comm params   */
    RFAL_TXRX_FLAGS_NFCV_FLAG_AUTO   = (0U<<6),   /*!< Enable automatic adaption of flag byte (ISO115693) according to current comm params   */
};


/*! RFAL error handling                                                                                                                      */
typedef enum {
    RFAL_ERRORHANDLING_NONE          = 0,         /*!< No special error handling will be performed                                           */
    RFAL_ERRORHANDLING_NFC           = 1,         /*!< Error handling set to perform as NFC complaint device                                 */
    RFAL_ERRORHANDLING_EMVCO         = 2          /*!< Error handling set to perform as EMVCo complaint device                               */
} rfalEHandling;


/*! Struct that holds all context to be used on a Transceive                                                */
typedef struct {
    uint8_t*              txBuf;                  /*!< (In)  Buffer where outgoing message is located       */
    uint16_t              txBufLen;               /*!< (In)  Length of the outgoing message in bits         */
    
    uint8_t*              rxBuf;                  /*!< (Out) Buffer where incoming message will be placed   */
    uint16_t              rxBufLen;               /*!< (In)  Maximum length of the incoming message in bits */
    uint16_t*             rxRcvdLen;              /*!< (Out) Actual received length in bits                 */
    
    uint32_t              flags;                  /*!< (In)  TransceiveFlags indication special handling    */
    uint32_t              fwt;                    /*!< (In)  Frame Waiting Time in 1/fc                     */
} rfalTransceiveContext;


/*! System callback to indicate an event that requires a system reRun        */
typedef void (* rfalUpperLayerCallback)(void);

/*! Callback to be executed before a Transceive                              */
typedef void (* rfalPreTxRxCallback)(void);

/*! Callback to be executed after a Transceive                               */
typedef void (* rfalPostTxRxCallback)(void);


/*******************************************************************************/
/*  ISO14443A                                                                  */
/*******************************************************************************/

/*! RFAL ISO 14443A Short Frame Command */
typedef enum
{
     RFAL_14443A_SHORTFRAME_CMD_WUPA = 0x52,  /*!< ISO14443A WUPA / NFC-A ALL_REQ  */
     RFAL_14443A_SHORTFRAME_CMD_REQA = 0x26   /*!< ISO14443A REQA / NFC-A SENS_REQ */    
} rfal14443AShortFrameCmd;

/*******************************************************************************/


/*******************************************************************************/
/*  FeliCa                                                                     */
/*******************************************************************************/

#define RFAL_FELICA_LEN_LEN                        1U                                           /*!< FeliCa LEN byte length                                              */
#define RFAL_FELICA_POLL_REQ_LEN                   (RFAL_FELICA_LEN_LEN + 1U + 2U + 1U + 1U)    /*!< FeliCa Poll Request length (LEN + CMD + SC + RC + TSN)              */
#define RFAL_FELICA_POLL_RES_LEN                   (RFAL_FELICA_LEN_LEN + 1U + 8U + 8U + 2U)    /*!< Maximum FeliCa Poll Response length (LEN + CMD + NFCID2 + PAD + RD) */
#define RFAL_FELICA_POLL_MAX_SLOTS                 16U                                          /*!< Maximum number of slots (TSN) on FeliCa Poll                        */


/*! NFC-F RC (Request Code) codes  NFC Forum Digital 1.1 Table 42                                                                                                        */
enum 
{
    RFAL_FELICA_POLL_RC_NO_REQUEST        =     0x00,                                           /*!< RC: No System Code information requested                            */
    RFAL_FELICA_POLL_RC_SYSTEM_CODE       =     0x01,                                           /*!< RC: System Code information requested                               */
    RFAL_FELICA_POLL_RC_COM_PERFORMANCE   =     0x02                                            /*!< RC: Advanced protocol features supported                            */
};


/*! NFC-F TSN (Time Slot Number) codes  NFC Forum Digital 1.1 Table 43   */
typedef enum 
{
    RFAL_FELICA_1_SLOT    =  0,   /*!< TSN with number of Time Slots: 1  */
    RFAL_FELICA_2_SLOTS   =  1,   /*!< TSN with number of Time Slots: 2  */
    RFAL_FELICA_4_SLOTS   =  3,   /*!< TSN with number of Time Slots: 4  */
    RFAL_FELICA_8_SLOTS   =  7,   /*!< TSN with number of Time Slots: 8  */
    RFAL_FELICA_16_SLOTS  =  15   /*!< TSN with number of Time Slots: 16 */
} rfalFeliCaPollSlots;


/*! NFCF Poll Response  NFC Forum Digital 1.1 Table 44 */
typedef uint8_t rfalFeliCaPollRes[RFAL_FELICA_POLL_RES_LEN];


/*******************************************************************************/


/*******************************************************************************/
/*  Listen Mode                                                                */  
/*******************************************************************************/

/*! RFAL Listen Mode NFCID Length */
typedef enum 
{
    RFAL_LM_NFCID_LEN_04               = 4,         /*!< Listen mode indicates  4 byte NFCID         */
    RFAL_LM_NFCID_LEN_07               = 7,         /*!< Listen mode indicates  7 byte NFCID         */
    RFAL_LM_NFCID_LEN_10               = 10,        /*!< Listen mode indicates 10 byte NFCID         */   
} rfalLmNfcidLen;


/*! RFAL Listen Mode States */
typedef enum 
{
    RFAL_LM_STATE_NOT_INIT              = 0x00,     /*!< Not Initialized state                       */
    RFAL_LM_STATE_POWER_OFF             = 0x01,     /*!< Power Off state                             */
    RFAL_LM_STATE_IDLE                  = 0x02,     /*!< Idle state  Activity 1.1  5.2               */
    RFAL_LM_STATE_READY_A               = 0x03,     /*!< Ready A state  Activity 1.1  5.3 5.4 & 5.5  */
    RFAL_LM_STATE_READY_B               = 0x04,     /*!< Ready B state  Activity 1.1  5.11 5.12      */
    RFAL_LM_STATE_READY_F               = 0x05,     /*!< Ready F state  Activity 1.1  5.15           */
    RFAL_LM_STATE_ACTIVE_A              = 0x06,     /*!< Active A state  Activity 1.1  5.6           */
    RFAL_LM_STATE_CARDEMU_4A            = 0x07,     /*!< Card Emulation 4A state  Activity 1.1  5.10 */
    RFAL_LM_STATE_CARDEMU_4B            = 0x08,     /*!< Card Emulation 4B state  Activity 1.1  5.14 */
    RFAL_LM_STATE_CARDEMU_3             = 0x09,     /*!< Card Emulation 3 state  Activity 1.1  5.18  */
    RFAL_LM_STATE_TARGET_A              = 0x0A,     /*!< Target A state  Activity 1.1  5.9           */
    RFAL_LM_STATE_TARGET_F              = 0x0B,     /*!< Target F state  Activity 1.1  5.17          */
    RFAL_LM_STATE_SLEEP_A               = 0x0C,     /*!< Sleep A state  Activity 1.1  5.7            */
    RFAL_LM_STATE_SLEEP_B               = 0x0D,     /*!< Sleep B state  Activity 1.1  5.13           */
    RFAL_LM_STATE_READY_Ax              = 0x0E,     /*!< Ready A* state  Activity 1.1  5.3 5.4 & 5.5 */
    RFAL_LM_STATE_ACTIVE_Ax             = 0x0F,     /*!< Active A* state  Activity 1.1  5.6          */
    RFAL_LM_STATE_SLEEP_AF              = 0x10,     /*!< Sleep AF state  Activity 1.1  5.19          */
} rfalLmState;


/*! RFAL Listen Mode Passive A configs */
typedef struct 
{
    rfalLmNfcidLen   nfcidLen;                        /*!< NFCID Len (00: 4bytes ; 01: 7bytes)       */
    uint8_t          nfcid[RFAL_NFCID1_TRIPLE_LEN];   /*!< NFCID                                     */
    uint8_t          SENS_RES[RFAL_LM_SENS_RES_LEN];  /*!< NFC-106k; SENS_REQ Response               */
    uint8_t          SEL_RES;                         /*!< SEL_RES (SAK) with complete NFCID1 (UID)  */
} rfalLmConfPA;


/*! RFAL Listen Mode Passive B configs */
typedef struct 
{
    uint8_t          SENSB_RES[RFAL_LM_SENSB_RES_LEN];  /*!< SENSF_RES                               */
} rfalLmConfPB;


/*! RFAL Listen Mode Passive F configs */
typedef struct 
{
    uint8_t          SC[RFAL_LM_SENSF_SC_LEN];          /*!< System Code to listen for               */
    uint8_t          SENSF_RES[RFAL_LM_SENSF_RES_LEN];  /*!< SENSF_RES                               */
} rfalLmConfPF;

/*******************************************************************************/


/*******************************************************************************/
/*  Wake-Up Mode                                                               */  
/*******************************************************************************/

#define RFAL_WUM_REFERENCE_AUTO           0xFFU      /*!< Indicates new reference is set by the driver*/

/*! RFAL Wake-Up Mode States */
typedef enum 
{
    RFAL_WUM_STATE_NOT_INIT              = 0x00,     /*!< Not Initialized state                       */
    RFAL_WUM_STATE_ENABLED               = 0x01,     /*!< Wake-Up mode is enabled                     */
    RFAL_WUM_STATE_ENABLED_WOKE          = 0x02,     /*!< Wake-Up mode enabled and has received IRQ(s)*/
} rfalWumState;

/*! RFAL Wake-Up Period/Timer */
typedef enum 
{
    RFAL_WUM_PERIOD_10MS      = 0x00,     /*!< Wake-Up timer 10ms                          */
    RFAL_WUM_PERIOD_20MS      = 0x01,     /*!< Wake-Up timer 20ms                          */
    RFAL_WUM_PERIOD_30MS      = 0x02,     /*!< Wake-Up timer 30ms                          */
    RFAL_WUM_PERIOD_40MS      = 0x03,     /*!< Wake-Up timer 40ms                          */
    RFAL_WUM_PERIOD_50MS      = 0x04,     /*!< Wake-Up timer 50ms                          */
    RFAL_WUM_PERIOD_60MS      = 0x05,     /*!< Wake-Up timer 60ms                          */
    RFAL_WUM_PERIOD_70MS      = 0x06,     /*!< Wake-Up timer 70ms                          */
    RFAL_WUM_PERIOD_80MS      = 0x07,     /*!< Wake-Up timer 80ms                          */
    RFAL_WUM_PERIOD_100MS     = 0x10,     /*!< Wake-Up timer 100ms                         */
    RFAL_WUM_PERIOD_200MS     = 0x11,     /*!< Wake-Up timer 200ms                         */
    RFAL_WUM_PERIOD_300MS     = 0x12,     /*!< Wake-Up timer 300ms                         */
    RFAL_WUM_PERIOD_400MS     = 0x13,     /*!< Wake-Up timer 400ms                         */
    RFAL_WUM_PERIOD_500MS     = 0x14,     /*!< Wake-Up timer 500ms                         */
    RFAL_WUM_PERIOD_600MS     = 0x15,     /*!< Wake-Up timer 600ms                         */
    RFAL_WUM_PERIOD_700MS     = 0x16,     /*!< Wake-Up timer 700ms                         */
    RFAL_WUM_PERIOD_800MS     = 0x17,     /*!< Wake-Up timer 800ms                         */
} rfalWumPeriod;


/*! RFAL Wake-Up Period/Timer */
typedef enum 
{
    RFAL_WUM_AA_WEIGHT_4       = 0x00,     /*!< Wake-Up Auto Average Weight 4              */
    RFAL_WUM_AA_WEIGHT_8       = 0x01,     /*!< Wake-Up Auto Average Weight 8              */
    RFAL_WUM_AA_WEIGHT_16      = 0x02,     /*!< Wake-Up Auto Average Weight 16             */
    RFAL_WUM_AA_WEIGHT_32      = 0x03,     /*!< Wake-Up Auto Average Weight 32             */
} rfalWumAAWeight;


/*! RFAL Wake-Up Mode configuration */
typedef struct 
{
    rfalWumPeriod        period;     /*!< Wake-Up Timer period;how often measurement(s) is performed */
    bool                 irqTout;    /*!< IRQ at every timeout will refresh the measurement(s)       */
    bool                 swTagDetect;/*!< Use SW Tag Detection instead of HW Wake-Up mode            */
  
    struct{
        bool             enabled;    /*!< Inductive Amplitude measurement enabled                   */
        uint8_t          delta;      /*!< Delta between the reference and measurement to wake-up    */
        uint8_t          reference;  /*!< Reference to be used;RFAL_WUM_REFERENCE_AUTO sets it auto */
        bool             autoAvg;    /*!< Use the HW Auto Averaging feature                         */
        bool             aaInclMeas; /*!< When AutoAvg is enabled, include IRQ measurement          */
        rfalWumAAWeight  aaWeight;   /*!< When AutoAvg is enabled, last measure weight              */
    }indAmp;                         /*!< Inductive Amplitude Configuration                         */
    struct{
        bool             enabled;    /*!< Inductive Phase measurement enabled                       */
        uint8_t          delta;      /*!< Delta between the reference and measurement to wake-up    */
        uint8_t          reference;  /*!< Reference to be used;RFAL_WUM_REFERENCE_AUTO sets it auto */
        bool             autoAvg;    /*!< Use the HW Auto Averaging feature                         */
        bool             aaInclMeas; /*!< When AutoAvg is enabled, include IRQ measurement          */
        rfalWumAAWeight  aaWeight;   /*!< When AutoAvg is enabled, last measure weight              */
    }indPha;                         /*!< Inductive Phase Configuration                             */
    struct{
        bool             enabled;    /*!< Capacitive measurement enabled                            */
        uint8_t          delta;      /*!< Delta between the reference and measurement to wake-up    */
        uint8_t          reference;  /*!< Reference to be used;RFAL_WUM_REFERENCE_AUTO sets it auto */
        bool             autoAvg;    /*!< Use the HW Auto Averaging feature                         */
        bool             aaInclMeas; /*!< When AutoAvg is enabled, include IRQ measurement          */
        rfalWumAAWeight  aaWeight;   /*!< When AutoAvg is enabled, last measure weight              */
    }cap;                            /*!< Capacitive Configuration                                  */
} rfalWakeUpConfig;

/*! RFAL features support (rfal_features.h)*/
typedef struct{
    boolean PollNFCA;                  /* RFAL Poll NFCA mode support switch    */
    boolean PollNFCB;                  /* RFAL Poll NFCB mode support switch    */
    boolean PollNFCF;                  /* RFAL Poll NFCF mode support switch    */
    boolean PollNFCV;                  /* RFAL Poll NFCV mode support switch    */
    boolean PollActiveP2P;             /* RFAL Poll AP2P mode support switch    */

    boolean ListenNFCA;                /* RFAL Listen NFCA mode support switch  */
    boolean ListenNFCB;                /* RFAL Listen NFCB mode support switch  */
    boolean ListenNFCF;                /* RFAL Listen NFCF mode support switch  */
    boolean ListenActiveP2P;           /* RFAL Listen AP2P mode support switch  */
    boolean ListenMode;               /* RFAL support for Listen Mode */

    boolean BR_RW_106;                  /* RFAL RW  106 Bit Rate support switch   */
    boolean BR_RW_212;                  /* RFAL RW  212 Bit Rate support switch   */
    boolean BR_RW_424;                 /* RFAL RW  424 Bit Rate support switch   */
    boolean BR_RW_848;                  /* RFAL RW  848 Bit Rate support switch   */
    boolean BR_RW_1695;               /* RFAL RW 1695 Bit Rate support switch   */
    boolean BR_RW_3390;                 /* RFAL RW 3390 Bit Rate support switch   */
    boolean BR_RW_6780;                 /* RFAL RW 6780 Bit Rate support switch   */
    boolean BR_RW_13560;                /* RFAL RW 6780 Bit Rate support switch   */

    boolean BR_AP2P_106;                  /* RFAL AP2P  106 Bit Rate support switch */
    boolean BR_AP2P_212;                  /* RFAL AP2P  212 Bit Rate support switch */
    boolean BR_AP2P_424;                  /* RFAL AP2P  424 Bit Rate support switch */
    boolean BR_AP2P_848;                  /* RFAL AP2P  848 Bit Rate support switch */
    boolean BR_AP2P_1695;                 /*!< RFAL AP2P 1695 Bit Rate support switch */
    boolean BR_AP2P_3390;                 /*!< RFAL AP2P 3390 Bit Rate support switch */
    boolean BR_AP2P_6780;

    boolean BR_CE_A_106;                  /* RFAL CE A 106 Bit Rate support switch  */
    boolean BR_CE_A_212;                  /* RFAL CE A 212 Bit Rate support switch  */
    boolean BR_CE_A_424;                  /* RFAL CE A 424 Bit Rate support switch  */
    boolean BR_CE_A_848;                  /* RFAL CE A 848 Bit Rate support switch  */

     boolean BR_CE_B_106;                  /* RFAL CE B 106 Bit Rate support switch  */
    boolean BR_CE_B_212;                  /* RFAL CE B 212 Bit Rate support switch  */
    boolean BR_CE_B_424;                  /* RFAL CE B 424 Bit Rate support switch  */
    boolean BR_CE_B_848;                  /* RFAL CE B 848 Bit Rate support switch  */

    boolean BR_CE_F_212;                  /* RFAL CE F 212 Bit Rate support switch  */
    boolean BR_CE_F_424;                  /* RFAL CE F 424 Bit Rate support switch  */

    boolean WAKEUP_MODE;                 /*  RFAL support for the Wake-Up mode                          */
    boolean NFCA;                         /*  RFAL support for NFC-A (ISO14443A)                         */
    boolean NFCB;                        /*  RFAL support for NFC-B (ISO14443B)                         */
    boolean NFCF;                         /*  RFAL support for NFC-F (FeliCa)                            */
    boolean NFCV;                         /*  RFAL support for NFC-V (ISO15693)                          */
    boolean T1T;                         /*  RFAL support for T1T (Topaz)                               */
    boolean T2T;                          /*  RFAL support for T2T                                       */
    boolean T4T;                          /*  RFAL support for T4T                                       */
    boolean ST25TB;                       /*  RFAL support for ST25TB                                    */
    boolean ST25xV;                      /*  RFAL support for ST25TV/ST25DV                             */
    boolean DYNAMIC_ANALOG_CONFIG;        /*  Analog Configs to be dynamically updated (RAM)             */
    boolean DYNAMIC_POWER;                /*  RFAL dynamic power support                                 */
    boolean ISO_DEP;                      /*  RFAL support for ISO-DEP (ISO14443-4)                      */
    boolean ISO_DEP_POLL;                 /*  RFAL support for Poller mode (PCD) ISO-DEP (ISO14443-4)    */
    boolean ISO_DEP_LISTEN;               /*  RFAL support for Listen mode (PICC) ISO-DEP (ISO14443-4)   */
    boolean NFC_DEP;                      /*  RFAL support for NFC-DEP (NFCIP1/P2P)                      */


    boolean ISO_DEP_IBLOCK_MAX_LEN;       /*  ISO-DEP I-Block max length. Please use values as defined by rfalIsoDepFSx */
    boolean ISO_DEP_APDU_MAX_LEN;         /*  ISO-DEP APDU max length. Please use multiples of I-Block max length       */

}rfalFeatures;

/* ISO DEP */

#define RFAL_ISODEP_PROLOGUE_SIZE               (3U)     /*!< Length of Prologue Field for I-Block Format                       */
#define RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN    256U       /*!< ISO-DEP I-Block max length. Please use values as defined by rfalIsoDepFSx */
/*! Structure of I-Block Buffer format from caller */
typedef struct
{
    uint8_t  prologue[RFAL_ISODEP_PROLOGUE_SIZE];      /*!< Prologue/SoD buffer                      */
    uint8_t  inf[RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN]; /*!< INF/Payload buffer                       */
} rfalIsoDepBufFormat;

/*! Structure of parameters used on ISO DEP Transceive */
typedef struct
{
    rfalIsoDepBufFormat  *txBuf;                       /*!< Transmit Buffer struct reference         */
    uint16_t             txBufLen;                     /*!< Transmit Buffer INF field length in Bytes*/
    bool                 isTxChaining;                 /*!< Transmit data is not complete            */
    rfalIsoDepBufFormat  *rxBuf;                       /*!< Receive Buffer struct reference in Bytes */
    uint16_t             *rxLen;                       /*!< Received INF data length in Bytes        */
    bool                 *isRxChaining;                /*!< Received data is not complete            */
    uint32_t             FWT;                          /*!< FWT to be used (ignored in Listen Mode)  */
    uint32_t             dFWT;                         /*!< Delta FWT to be used                     */
    uint16_t             ourFSx;                       /*!< Our device Frame Size (FSD or FSC)       */
    uint16_t             FSx;                          /*!< Other device Frame Size (FSD or FSC)     */
    uint8_t              DID;                          /*!< Device ID (RFAL_ISODEP_NO_DID if no DID) */
} rfalIsoDepTxRxParam;

class rfal_rf{
  public:
    virtual ReturnCode rfalInitialize(); 
    virtual void rfalAnalogConfigInitialize();
    virtual void CheckInterrupts();  
    virtual void rfalWorker();
    virtual ReturnCode rfalStartTransceive( const rfalTransceiveContext *ctx );
    
    /* RFAL_ISO_DEP */
    virtual ReturnCode rfalIsoDepDeselect();
    virtual ReturnCode rfalTransceiveBlockingRx( void );
    virtual ReturnCode rfalTransceiveBlockingTx( uint8_t* txBuf, uint16_t txBufLen, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t* actLen, uint32_t flags, uint32_t fwt );    
    virtual ReturnCode rfalGetTransceiveStatus();
    virtual ReturnCode rfalIsoDepStartTransceive( rfalIsoDepTxRxParam param );
    virtual ReturnCode rfalIsoDepGetTransceiveStatus( void );
    //virtual ReturnCode isoDepDataExchangePCD( uint16_t *outActRxLen, bool *outIsChaining );
};

#endif /* RFAL_RF_H */
