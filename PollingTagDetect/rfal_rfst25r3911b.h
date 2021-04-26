/*NF05*/
#ifndef RFAL_RFST25R3911B_H
#define RFAL_RFST25R3911B_H

/* INCLUDE */
#include "rfal_rf.h"
#include "rfal_rfst25r3911b_com.h"
#include "rfal_rfst25r3911b_analogConfig.h"
#include "rfal_rfst25r3911b_features.h"
#include "rfal_nfcb.h"

/* DEFINE */
#define RFAL_LM_GT                      rfalConvUsTo1fc(100U)                          /*!< Listen Mode Guard Time enforced (GT - Passive; TIRFG - Active)                  */
#define RFAL_FDT_POLL_ADJUSTMENT        rfalConvUsTo1fc(80U)                           /*!< FDT Poll adjustment: Time between the expiration of GPT to the actual Tx        */
#define RFAL_FDT_LISTEN_MRT_ADJUSTMENT  64U                                            /*!< MRT jitter adjustment: timeout will be between [ tout ; tout + 64 cycles ]      */
#define RFAL_AP2P_FIELDOFF_TRFW         rfalConv8fcTo1fc(64U)                          /*!< Time after TXE and Field Off in AP2P Trfw: 37.76us -> 64  (8/fc)                */


/*! FWT adjustment: 
 *    64 : NRT jitter between TXE and NRT start      */
#define RFAL_FWT_ADJUSTMENT             64U

/*! FWT ISO14443A adjustment:  
 *   512  : Initial 4bit length                      */
#define RFAL_FWT_A_ADJUSTMENT           512U

/*! FWT ISO14443B adjustment:  
 *   2784 : Adjustment for the SOF and initial byte  */
#define RFAL_FWT_B_ADJUSTMENT           2784U


/*! FWT FeliCa 212 adjustment:  
 *    1024 : Length of the two Sync bytes at 212kbps */
#define RFAL_FWT_F_212_ADJUSTMENT       1024U

/*! FWT FeliCa 424 adjustment:  
 *    512 : Length of the two Sync bytes at 424kbps  */
#define RFAL_FWT_F_424_ADJUSTMENT       512U


/*! Time between our field Off and other peer field On : Tadt + (n x Trfw)
 * Ecma 340 11.1.2 - Tadt: [56.64 , 188.72] us ;  n: [0 , 3]  ; Trfw = 37.76 us        
 * Should be: 189 + (3*38) = 303us ; we'll use a more relaxed setting: 605 us    */
#define RFAL_AP2P_FIELDON_TADTTRFW      rfalConvUsTo1fc(605U)


/*! FDT Listen adjustment for ISO14443A   EMVCo 2.6  4.8.1.3  ;  Digital 1.1  6.10
 */
#define RFAL_FDT_LISTEN_A_ADJUSTMENT    276U


/*! FDT Listen adjustment for ISO14443B   
 */
#define RFAL_FDT_LISTEN_B_ADJUSTMENT    (340U - 64U)


/*! FDT Listen adjustment for ISO15693
 */
#define RFAL_FDT_LISTEN_V_ADJUSTMENT    128U

/* COMPONENT */
#define ST25R3911_FDT_NONE                     0x00U    /*!< Value indicating not to perform FDT                        */

#define MS_TO_64FCS(A)                  ((A) * 212U)    /*!< Converts from ms to 64/fc steps                            */
#define MS_FROM_64FCS(A)                ((A) / 212U)    /*!< Converts from 64/fc steps to ms                            */
#define rfalCalcNumBytes( nBits )                (((uint32_t)(nBits) + 7U) / 8U)                          /*!< Returns the number of bytes required to fit given the number of bits */
#define RFAL_FDT_POLL_ADJUSTMENT        rfalConvUsTo1fc(80U)                           /*!< FDT Poll adjustment: Time between the expiration of GPT to the actual Tx        */

/* ST25R3911 direct commands */
#define ST25R3911_CMD_SET_DEFAULT              0xC1U    /*!< Puts the chip in default state (same as after power-up)    */
#define ST25R3911_CMD_CLEAR_FIFO               0xC2U    /*!< Stops all activities and clears FIFO                       */
#define ST25R3911_CMD_TRANSMIT_WITH_CRC        0xC4U    /*!< Transmit with CRC                                          */
#define ST25R3911_CMD_TRANSMIT_WITHOUT_CRC     0xC5U    /*!< Transmit without CRC                                       */
#define ST25R3911_CMD_TRANSMIT_REQA            0xC6U    /*!< Transmit REQA                                              */
#define ST25R3911_CMD_TRANSMIT_WUPA            0xC7U    /*!< Transmit WUPA                                              */
#define ST25R3911_CMD_INITIAL_RF_COLLISION     0xC8U    /*!< NFC transmit with Initial RF Collision Avoidance           */
#define ST25R3911_CMD_RESPONSE_RF_COLLISION_N  0xC9U    /*!< NFC transmit with Response RF Collision Avoidance          */
#define ST25R3911_CMD_RESPONSE_RF_COLLISION_0  0xCAU    /*!< NFC transmit with Response RF Collision Avoidance with n=0 */
#define ST25R3911_CMD_NORMAL_NFC_MODE          0xCBU    /*!< NFC switch to normal NFC mode                              */
#define ST25R3911_CMD_ANALOG_PRESET            0xCCU    /*!< Analog Preset                                              */
#define ST25R3911_CMD_MASK_RECEIVE_DATA        0xD0U    /*!< Mask recive data                                           */
#define ST25R3911_CMD_UNMASK_RECEIVE_DATA      0xD1U    /*!< Unmask recive data                                         */
#define ST25R3911_CMD_MEASURE_AMPLITUDE        0xD3U    /*!< Measure singal amplitude on RFI inputs                     */
#define ST25R3911_CMD_SQUELCH                  0xD4U    /*!< Squelch                                                    */
#define ST25R3911_CMD_CLEAR_SQUELCH            0xD5U    /*!< Clear Squelch                                              */
#define ST25R3911_CMD_ADJUST_REGULATORS        0xD6U    /*!< Adjust regulators                                          */
#define ST25R3911_CMD_CALIBRATE_MODULATION     0xD7U    /*!< Calibrate modulation depth                                 */
#define ST25R3911_CMD_CALIBRATE_ANTENNA        0xD8U    /*!< Calibrate antenna                                          */
#define ST25R3911_CMD_MEASURE_PHASE            0xD9U    /*!< Measure phase between RFO and RFI signal                   */
#define ST25R3911_CMD_CLEAR_RSSI               0xDAU    /*!< clear RSSI bits and restart the measurement                */
#define ST25R3911_CMD_TRANSPARENT_MODE         0xDCU    /*!< Transparent mode                                           */
#define ST25R3911_CMD_CALIBRATE_C_SENSOR       0xDDU    /*!< Calibrate the capacitive sensor                            */
#define ST25R3911_CMD_MEASURE_CAPACITANCE      0xDEU    /*!< Measure capacitance                                        */
#define ST25R3911_CMD_MEASURE_VDD              0xDFU    /*!< Measure power supply voltage                               */
#define ST25R3911_CMD_START_GP_TIMER           0xE0U    /*!< Start the general purpose timer                            */
#define ST25R3911_CMD_START_WUP_TIMER          0xE1U    /*!< Start the wake-up timer                                    */
#define ST25R3911_CMD_START_MASK_RECEIVE_TIMER 0xE2U    /*!< Start the mask-receive timer                               */
#define ST25R3911_CMD_START_NO_RESPONSE_TIMER  0xE3U    /*!< Start the no-repsonse timer                                */
#define ST25R3911_CMD_TEST_CLEARA              0xFAU    /*!< Clear Test register                                        */
#define ST25R3911_CMD_TEST_CLEARB              0xFBU    /*!< Clear Test register                                        */
#define ST25R3911_CMD_TEST_ACCESS              0xFCU    /*!< Enable R/W access to the test registers                    */
#define ST25R3911_CMD_LOAD_PPROM               0xFDU    /*!< Load data from the poly fuses to RAM                       */
#define ST25R3911_CMD_FUSE_PPROM               0xFEU    /*!< Fuse poly fuses with data from the RAM                     */


#define ST25R3911_FIFO_DEPTH                   96U      /*!< Depth of FIFO                                              */

#define ST25R3911_THRESHOLD_DO_NOT_SET         0xFFU    /*!< Indicates not to change this Threshold                     */

#define ST25R3911_BR_DO_NOT_SET                0xFFU    /*!< Indicates not to change this Bit Rate                      */
#define ST25R3911_BR_106                       0x00U    /*!< ST25R3911 Bit Rate 106 kbit/s (fc/128)                     */
#define ST25R3911_BR_212                       0x01U    /*!< ST25R3911 Bit Rate 212 kbit/s (fc/64)                      */
#define ST25R3911_BR_424                       0x02U    /*!< ST25R3911 Bit Rate 424 kbit/s (fc/32)                      */
#define ST25R3911_BR_848                       0x03U    /*!< ST25R3911 Bit Rate 848 kbit/s (fc/16)                      */
#define ST25R3911_BR_1695                      0x04U    /*!< ST25R3911 Bit Rate 1696 kbit/s (fc/8)                      */
#define ST25R3911_BR_3390                      0x05U    /*!< ST25R3911 Bit Rate 3390 kbit/s (fc/4)                      */
#define ST25R3911_BR_6780                      0x06U    /*!< ST25R3911 Bit Rate 6780 kbit/s (fc/2)                      */

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/

/*! Checks if General Purpose Timer is still running by reading gpt_on flag */
#define st25r3911IsGPTRunning( )     ( st25r3911CheckReg(ST25R3911_REG_REGULATOR_RESULT, ST25R3911_REG_REGULATOR_RESULT_gpt_on, ST25R3911_REG_REGULATOR_RESULT_gpt_on) )

/*! Checks if CRC is configured to be in FIFO                               */
#define st25r3911IsCRCinFIFO( )      ( st25r3911CheckReg(ST25R3911_REG_AUX, ST25R3911_REG_AUX_crc_2_fifo, ST25R3911_REG_AUX_crc_2_fifo) )

/*! Checks if External Filed is detected by reading ST25R3911 External Field  
 * Detector output                                                          */
#define st25r3911IsExtFieldOn()      ( st25r3911CheckReg(ST25R3911_REG_AUX_DISPLAY, ST25R3911_REG_AUX_DISPLAY_efd_o, ST25R3911_REG_AUX_DISPLAY_efd_o ) )

/*! Checks if Transmitter is enabled (Field On) */
#define st25r3911IsTxEnabled()       ( st25r3911CheckReg(ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_tx_en, ST25R3911_REG_OP_CONTROL_tx_en ) )

/*! Turn Off Tx (Field Off) */
#define st25r3911TxOff()              st25r3911ClrRegisterBits(ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_tx_en );

/*! Checks if last FIFO byte is complete */
#define st25r3911IsLastFIFOComplete() st25r3911CheckReg( ST25R3911_REG_FIFO_RX_STATUS2, ST25R3911_REG_FIFO_RX_STATUS2_mask_fifo_lb, 0 )

/*! Checks if the Oscillator is enabled  */
#define st25r3911IsOscOn()            st25r3911CheckReg( ST25R3911_REG_OP_CONTROL, ST25R3911_REG_OP_CONTROL_en, ST25R3911_REG_OP_CONTROL_en )

/*
******************************************************************************
* RFAL FEATURES CONFIGURATION
******************************************************************************
*/

#define RFAL_FEATURE_LISTEN_MODE               false      /*!< Enable/Disable RFAL support for Listen Mode                               */
#define RFAL_FEATURE_WAKEUP_MODE               true       /*!< Enable/Disable RFAL support for the Wake-Up mode                          */
#define RFAL_FEATURE_NFCA                      true       /*!< Enable/Disable RFAL support for NFC-A (ISO14443A)                         */
#define RFAL_FEATURE_NFCB                      true       /*!< Enable/Disable RFAL support for NFC-B (ISO14443B)                         */
#define RFAL_FEATURE_NFCF                      true       /*!< Enable/Disable RFAL support for NFC-F (FeliCa)                            */
#define RFAL_FEATURE_NFCV                      true       /*!< Enable/Disable RFAL support for NFC-V (ISO15693)                          */
#define RFAL_FEATURE_T1T                       true       /*!< Enable/Disable RFAL support for T1T (Topaz)                               */
#define RFAL_FEATURE_T2T                       true       /*!< Enable/Disable RFAL support for T2T                                       */
#define RFAL_FEATURE_T4T                       true       /*!< Enable/Disable RFAL support for T4T                                       */
#define RFAL_FEATURE_ST25TB                    true       /*!< Enable/Disable RFAL support for ST25TB                                    */
#define RFAL_FEATURE_ST25xV                    true       /*!< Enable/Disable RFAL support for ST25TV/ST25DV                             */
#define RFAL_FEATURE_DYNAMIC_ANALOG_CONFIG     false      /*!< Enable/Disable Analog Configs to be dynamically updated (RAM)             */
#define RFAL_FEATURE_DYNAMIC_POWER             false      /*!< Enable/Disable RFAL dynamic power support                                 */
#define RFAL_FEATURE_ISO_DEP                   true       /*!< Enable/Disable RFAL support for ISO-DEP (ISO14443-4)                      */
#define RFAL_FEATURE_ISO_DEP_POLL              true       /*!< Enable/Disable RFAL support for Poller mode (PCD) ISO-DEP (ISO14443-4)    */
#define RFAL_FEATURE_ISO_DEP_LISTEN            false      /*!< Enable/Disable RFAL support for Listen mode (PICC) ISO-DEP (ISO14443-4)   */
#define RFAL_FEATURE_NFC_DEP                   true       /*!< Enable/Disable RFAL support for NFC-DEP (NFCIP1/P2P)                      */



#define RFAL_FEATURE_ISO_DEP_APDU_MAX_LEN      1024U      /*!< ISO-DEP APDU max length. Please use multiples of I-Block max length       */

#define RFAL_FIFO_IN_LT_32              32U                                            /*!< Number of bytes in the FIFO when WL interrupt occurs while Tx ( fifo_lt: 0 )    */
#define RFAL_FIFO_IN_LT_16              16U                                            /*!< Number of bytes in the FIFO when WL interrupt occurs while Tx ( fifo_lt: 1 )    */

#define RFAL_FIFO_OUT_LT_32             (ST25R3911_FIFO_DEPTH - RFAL_FIFO_IN_LT_32)    /*!< Number of bytes sent/out of the FIFO when WL interrupt occurs while Tx ( fifo_lt: 0 ) */
#define RFAL_FIFO_OUT_LT_16             (ST25R3911_FIFO_DEPTH - RFAL_FIFO_IN_LT_16)    /*!< Number of bytes sent/out of the FIFO when WL interrupt occurs while Tx ( fifo_lt: 1 ) */

#define RFAL_FIFO_STATUS_REG1           0U                                             /*!< Location of FIFO status register 1 in local copy                                */
#define RFAL_FIFO_STATUS_REG2           1U                                             /*!< Location of FIFO status register 2 in local copy                                */
#define RFAL_FIFO_STATUS_INVALID        0xFFU                                          /*!< Value indicating that the local FIFO status in invalid|cleared                  */

#define RFAL_ST25R3911_GPT_MAX_1FC      rfalConv8fcTo1fc(  0xFFFFU )                   /*!< Max GPT steps in 1fc (0xFFFF steps of 8/fc    => 0xFFFF * 590ns  = 38,7ms)      */
#define RFAL_ST25R3911_NRT_MAX_1FC      rfalConv4096fcTo1fc( 0xFFFFU )                 /*!< Max NRT steps in 1fc (0xFFFF steps of 4096/fc => 0xFFFF * 302us  = 19.8s )      */
#define RFAL_ST25R3911_NRT_DISABLED     0U                                             /*!< NRT Disabled: All 0 No-response timer is not started, wait forever              */
#define RFAL_ST25R3911_MRT_MAX_1FC      rfalConv64fcTo1fc( 0x00FFU )                   /*!< Max MRT steps in 1fc (0x00FF steps of 64/fc   => 0x00FF * 4.72us = 1.2ms )      */
#define RFAL_ST25R3911_MRT_MIN_1FC      rfalConv64fcTo1fc( 0x0004U )                   /*!< Min MRT steps in 1fc ( 0<=mrt<=4 ; 4 (64/fc)  => 0x0004 * 4.72us = 18.88us )    */
#define RFAL_ST25R3911_GT_MAX_1FC       rfalConvMsTo1fc( 5000U )                       /*!< Max GT value allowed in 1/fc                                                    */
#define RFAL_ST25R3911_GT_MIN_1FC       rfalConvMsTo1fc(RFAL_ST25R3911_SW_TMR_MIN_1MS) /*!< Min GT value allowed in 1/fc                                                    */
#define RFAL_ST25R3911_SW_TMR_MIN_1MS   1U                                             /*!< Min value of a SW timer in ms                                                   */

#define RFAL_OBSMODE_DISABLE            0x00U                                          /*!< Observation Mode disabled                                                       */

#define RFAL_NFC_RX_INCOMPLETE_LEN      (uint8_t)1U                                    /*!< Threshold value where incoming rx may be considered as incomplete in NFC        */
#define RFAL_EMVCO_RX_MAXLEN            (uint8_t)4U                                    /*!< Maximum value where EMVCo to apply special error handling                       */
#define RFAL_EMVCO_RX_MINLEN            (uint8_t)2U                                    /*!< Minimum value where EMVCo to apply special error handling                       */

#define RFAL_NORXE_TOUT                 10U                                            /*!< Timeout to be used on a potential missing RXE - Silicon ST25R3911B Errata #1.1  */

#define RFAL_ISO14443A_SDD_RES_LEN      5U                                             /*!< SDD_RES | Anticollision (UID CLn) length  -  rfalNfcaSddRes                     */

#define RFAL_FELICA_POLL_DELAY_TIME     512U                                           /*!<  FeliCa Poll Processing time is 2.417 ms ~512*64/fc Digital 1.1 A4              */
#define RFAL_FELICA_POLL_SLOT_TIME      256U                                           /*!<  FeliCa Poll Time Slot duration is 1.208 ms ~256*64/fc Digital 1.1 A4           */

#define RFAL_ISO15693_IGNORE_BITS       rfalConvBytesToBits(2U)                        /*!< Ignore collisions before the UID (RES_FLAG + DSFID)                             */

/* TYPEDEF */
/*! Struct that holds all involved on a Transceive including the context passed by the caller     */
typedef struct{
    rfalTransceiveState     state;       /*!< Current transceive state                            */
    rfalTransceiveState     lastState;   /*!< Last transceive state (debug purposes)              */
    ReturnCode              status;      /*!< Current status/error of the transceive              */
    bool                    rxse;        /*!< Flag indicating if RXE was received with RXS        */
    
    rfalTransceiveContext   ctx;         /*!< The transceive context given by the caller          */
} rfalTxRx;


/*! Struct that holds all context for the Listen Mode                                             */
typedef struct{
    rfalLmState             state;       /*!< Current Listen Mode state                           */
    rfalBitRate             brDetected;  /*!< Last bit rate detected                              */
    
    uint8_t*                rxBuf;       /*!< Location to store incoming data in Listen Mode      */
    uint16_t                rxBufLen;    /*!< Length of rxBuf                                     */
    uint16_t*               rxLen;       /*!< Pointer to write the data length placed into rxBuf  */
    bool                    dataFlag;    /*!< Listen Mode current Data Flag                       */
} rfalLm;


/*! Struct that holds all context for the Wake-Up Mode                                            */
typedef struct{
    rfalWumState            state;       /*!< Current Wake-Up Mode state                          */
    rfalWakeUpConfig        cfg;         /*!< Current Wake-Up Mode context                        */     
} rfalWum;


/*! Struct that holds the timings GT and FDTs                             */
typedef struct{
    uint32_t                GT;          /*!< GT in 1/fc                  */
    uint32_t                FDTListen;   /*!< FDTListen in 1/fc           */
    uint32_t                FDTPoll;     /*!< FDTPoll in 1/fc             */
} rfalTimings;


/*! Struct that holds the software timers                                 */
typedef struct{
    uint32_t                GT;          /*!< RFAL's GT timer             */
    uint32_t                FWT;         /*!< FWT/RWT timer for Active P2P*/
    uint32_t                RXE;         /*!< Timer between RXS and RXE   */ 
} rfalTimers;


/*! Struct that holds the RFAL's callbacks                                */
typedef struct{
    rfalPreTxRxCallback     preTxRx;     /*!< RFAL's Pre TxRx callback    */
    rfalPostTxRxCallback    postTxRx;    /*!< RFAL's Post TxRx callback   */
} rfalCallbacks;


/*! Struct that holds counters to control the FIFO on Tx and Rx                                                                          */
typedef struct{    
    uint16_t                expWL;       /*!< The amount of bytes expected to be Tx when a WL interrupt occours                          */
    uint16_t                bytesTotal;  /*!< Total bytes to be transmitted OR the total bytes received                                  */
    uint16_t                bytesWritten;/*!< Amount of bytes already written on FIFO (Tx) OR read (RX) from FIFO and written on rxBuffer*/
    uint8_t                 status[ST25R3911_FIFO_STATUS_LEN];   /*!< FIFO Status Registers                                              */
} rfalFIFO;


/*! Struct that holds RFAL's configuration settings                                                      */
typedef struct{    
    uint8_t                 obsvModeTx;  /*!< RFAL's config of the ST25R3911's observation mode while Tx */
    uint8_t                 obsvModeRx;  /*!< RFAL's config of the ST25R3911's observation mode while Rx */
    rfalEHandling           eHandling;   /*!< RFAL's error handling config/mode                          */
} rfalConfigs;


/*! Struct that holds NFC-F data - Used only inside rfalFelicaPoll() (static to avoid adding it into stack) */
typedef struct{    
    rfalFeliCaPollRes pollResponses[RFAL_FELICA_POLL_MAX_SLOTS];   /* FeliCa Poll response container for 16 slots */
} rfalNfcfWorkingData;


/*! Struct that holds NFC-V current context
 *
 * 96 bytes is FIFO size of ST25R3911, codingBuffer has to be big enough for coping with maximum response size (Manchester coded)
 *    - current implementation expects it be written in one bulk into FIFO
 *    - needs to be above FIFO water level of ST25R3911 (64)
 *    - 65 is actually 1 byte too much, but ~75us in 1of256 another byte is already gone
 *    
 *    - inventory requests responses: 14 bytes 
 *    - Max read single block responses: 32 bytes
 *    - Read multiple block responses: variable    
 *    
 *    ISO15693 frame: SOF + Flags + Data + CRC + EOF  
 */
typedef struct{    
    uint8_t               codingBuffer[((2 + 255 + 3)*2)];/*!< Coding buffer,   length MUST be above 64: [65; ...]                   */
    uint16_t              nfcvOffset;                     /*!< Offset needed for ISO15693 coding function                            */
    rfalTransceiveContext origCtx;                        /*!< Context provided by user                                              */
    uint16_t              ignoreBits;                     /*!< Number of bits at the beginning of a frame to be ignored when decoding*/
} rfalNfcvWorkingData;

/*! RFAL instance  */
typedef struct{
    rfalState               state;     /*!< RFAL's current state                          */
    rfalMode                mode;      /*!< RFAL's current mode                           */
    rfalBitRate             txBR;      /*!< RFAL's current Tx Bit Rate                    */
    rfalBitRate             rxBR;      /*!< RFAL's current Rx Bit Rate                    */
    bool                    field;     /*!< Current field state (On / Off)                */
                            
    rfalConfigs             conf;      /*!< RFAL's configuration settings                 */
    rfalTimings             timings;   /*!< RFAL's timing setting                         */
    rfalTxRx                TxRx;      /*!< RFAL's transceive management                  */
    rfalFIFO                fifo;      /*!< RFAL's FIFO management                        */
    rfalTimers              tmr;       /*!< RFAL's Software timers                        */
    rfalCallbacks           callbacks; /*!< RFAL's callbacks                              */
    rfalLm                  Lm;        /*!< RFAL's listen mode management                 */

    
#if RFAL_FEATURE_WAKEUP_MODE     
    rfalWum                 wum;       /*!< RFAL's Wake-up mode management                */
#endif /* RFAL_FEATURE_WAKEUP_MODE */

#if RFAL_FEATURE_NFCF
    rfalNfcfWorkingData     nfcfData; /*!< RFAL's working data when supporting NFC-F      */
#endif /* RFAL_FEATURE_NFCF */
    
#if RFAL_FEATURE_NFCV
    rfalNfcvWorkingData     nfcvData; /*!< RFAL's working data when supporting NFC-V      */
#endif /* RFAL_FEATURE_NFCV */
    
} rfal;

/* DEFINE */
/* Main interrupt register. */
#define ST25R3911_IRQ_MASK_ALL             (uint32_t)(0xFFFFFFU) /*!< All ST25R3911 interrupt sources                              */
#define ST25R3911_IRQ_MASK_NONE            (uint32_t)(0U)        /*!< No ST25R3911 interrupt source                                */
#define ST25R3911_IRQ_MASK_OSC             (uint32_t)(0x80U)     /*!< ST25R3911 oscillator stable interrupt                        */
#define ST25R3911_IRQ_MASK_FWL             (uint32_t)(0x40U)     /*!< ST25R3911 FIFO water level interrupt                         */
#define ST25R3911_IRQ_MASK_RXS             (uint32_t)(0x20U)     /*!< ST25R3911 start of receive interrupt                         */
#define ST25R3911_IRQ_MASK_RXE             (uint32_t)(0x10U)     /*!< ST25R3911 end of receive interrupt                           */
#define ST25R3911_IRQ_MASK_TXE             (uint32_t)(0x08U)     /*!< ST25R3911 end of transmission interrupt                      */
#define ST25R3911_IRQ_MASK_COL             (uint32_t)(0x04U)     /*!< ST25R3911 bit collision interrupt                            */

/* Timer and NFC interrupt register. */
#define ST25R3911_IRQ_MASK_DCT             (uint32_t)(0x8000U)   /*!< ST25R3911 termination of direct command interrupt            */
#define ST25R3911_IRQ_MASK_NRE             (uint32_t)(0x4000U)   /*!< ST25R3911 no-response timer expired interrupt                */
#define ST25R3911_IRQ_MASK_GPE             (uint32_t)(0x2000U)   /*!< ST25R3911 general purpose timer expired interrupt            */
#define ST25R3911_IRQ_MASK_EON             (uint32_t)(0x1000U)   /*!< ST25R3911 external field on interrupt                        */
#define ST25R3911_IRQ_MASK_EOF             (uint32_t)(0x0800U)   /*!< ST25R3911 external field off interrupt                       */
#define ST25R3911_IRQ_MASK_CAC             (uint32_t)(0x0400U)   /*!< ST25R3911 collision during RF collision avoidance interrupt  */
#define ST25R3911_IRQ_MASK_CAT             (uint32_t)(0x0200U)   /*!< ST25R3911 minimum guard time expired interrupt               */
#define ST25R3911_IRQ_MASK_NFCT            (uint32_t)(0x0100U)   /*!< ST25R3911 initiator bit rate recognized interrupt            */

/* Error and wake-up interrupt register. */
#define ST25R3911_IRQ_MASK_CRC             (uint32_t)(0x800000U) /*!< ST25R3911 CRC error interrupt                                */
#define ST25R3911_IRQ_MASK_PAR             (uint32_t)(0x400000U) /*!< ST25R3911 parity error interrupt                             */
#define ST25R3911_IRQ_MASK_ERR2            (uint32_t)(0x200000U) /*!< ST25R3911 soft framing error interrupt                       */
#define ST25R3911_IRQ_MASK_ERR1            (uint32_t)(0x100000U) /*!< ST25R3911 hard framing error interrupt                       */
#define ST25R3911_IRQ_MASK_WT              (uint32_t)(0x080000U) /*!< ST25R3911 wake-up interrupt                                  */
#define ST25R3911_IRQ_MASK_WAM             (uint32_t)(0x040000U) /*!< ST25R3911 wake-up due to amplitude interrupt                 */
#define ST25R3911_IRQ_MASK_WPH             (uint32_t)(0x020000U) /*!< ST25R3911 wake-up due to phase interrupt                     */
#define ST25R3911_IRQ_MASK_WCAP            (uint32_t)(0x010000U) /*!< ST25R3911 wake-up due to capacitance measurement             */


#define ST25R3911_IRQ_MASK_TIM             (0x02U)               /*!< additional interrupts in ST25R3911_REG_IRQ_TIMER_NFC         */
#define ST25R3911_IRQ_MASK_ERR             (0x01U)               /*!< additional interrupts in ST25R3911_REG_IRQ_ERROR_WUP         */

/*! Length of the interrupt registers       */
#define ST25R3911_INT_REGS_LEN          ( (ST25R3911_REG_IRQ_ERROR_WUP - ST25R3911_REG_IRQ_MAIN) + 1U )

#define ST25R3911_OSC_STABLE_TIMEOUT           10U /*!< Timeout for Oscillator to get stable, datasheet: 700us, take 5 ms */
#define ST25R3911_CA_TIMEOUT                   10U /*!< Timeout for Collision Avoidance command                           */
#define ST25R3911_TOUT_CALIBRATE_CAP_SENSOR    4U  /*!< Max duration Calibrate Capacitive Sensor command   Datasheet: 3ms */

/* TYPEDEF */

/*! Holds current and previous interrupt callback pointer as well as current Interrupt status and mask */
typedef struct
{
    void      (*prevCallback)(void); /*!< call back function for 3911 interrupt               */
    void      (*callback)(void);     /*!< call back function for 3911 interrupt               */
    uint32_t  status;                /*!< latest interrupt status                             */
    uint32_t  mask;                  /*!< Interrupt mask. Negative mask = ST25R3911 mask regs */
}t_st25r3911Interrupt;

/* RFAL ISO */
/*! Enum holding possible VCD codings  */
typedef enum
{
    ISO15693_VCD_CODING_1_4,
    ISO15693_VCD_CODING_1_256
}iso15693VcdCoding_t;

/*! Enum holding possible VICC datarates */

/*! Configuration parameter used by #iso15693PhyConfigure  */
typedef struct
{
    iso15693VcdCoding_t coding;           /*!< desired VCD coding                                       */
    uint32_t                speedMode;    /*!< 0: normal mode, 1: 2^1 = x2 Fast mode, 2 : 2^2 = x4 mode, 3 : 2^3 = x8 mode - all rx pulse numbers and times are divided by 1,2,4,8 */
}iso15693PhyConfig_t;

/*! Parameters how the stream mode should work */
struct iso15693StreamConfig {
    uint8_t useBPSK;              /*!< 0: subcarrier, 1:BPSK */
    uint8_t din;                  /*!< the divider for the in subcarrier frequency: fc/2^din  */
    uint8_t dout;                 /*!< the divider for the in subcarrier frequency fc/2^dout */
    uint8_t report_period_length; /*!< the length of the reporting period 2^report_period_length*/
};
/*
******************************************************************************
* GLOBAL CONSTANTS
******************************************************************************
*/

#define ISO15693_REQ_FLAG_TWO_SUBCARRIERS 0x01U   /*!< Flag indication that communication uses two subcarriers */
#define ISO15693_REQ_FLAG_HIGH_DATARATE   0x02U   /*!< Flag indication that communication uses high bitrate    */
#define ISO15693_MASK_FDT_LISTEN         (65)     /*!< t1min = 308,2us = 4192/fc = 65.5 * 64/fc                */

/*! t1max = 323,3us = 4384/fc = 68.5 * 64/fc
 *         12 = 768/fc unmodulated time of single subcarrior SoF */
#define ISO15693_FWT (69 + 12)

#define ISO15693_DAT_SOF_1_4     0x21 /* LSB constants */
#define ISO15693_DAT_EOF_1_4     0x04
#define ISO15693_DAT_00_1_4      0x02
#define ISO15693_DAT_01_1_4      0x08
#define ISO15693_DAT_10_1_4      0x20
#define ISO15693_DAT_11_1_4      0x80

#define ISO15693_DAT_SOF_1_256   0x81
#define ISO15693_DAT_EOF_1_256   0x04
#define ISO15693_DAT_SLOT0_1_256 0x02
#define ISO15693_DAT_SLOT1_1_256 0x08
#define ISO15693_DAT_SLOT2_1_256 0x20
#define ISO15693_DAT_SLOT3_1_256 0x80

#define ISO15693_PHY_DAT_MANCHESTER_1 0xaaaa

#define ISO15693_PHY_BIT_BUFFER_SIZE 1000 /*!< size of the receiving buffer. Might be adjusted if longer datastreams are expected. */

/*
 ******************************************************************************
 * ISO DEP
 ******************************************************************************
 */
#define ISODEP_CRC_LEN                  RFAL_CRC_LEN   /*!< ISO1443 CRC Length */


#define ISODEP_PCB_POS                  (0U)         /*!< PCB position on message header*/
#define ISODEP_SWTX_INF_POS             (1U)         /*!< INF position in a S-WTX       */

#define ISODEP_DID_POS                  (1U)         /*!< DID position on message header*/
#define ISODEP_SWTX_PARAM_LEN           (1U)         /*!< SWTX parameter length         */

#define ISODEP_DSL_MAX_LEN              ( RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN ) /*!< Deselect Req/Res length */

#define ISODEP_PCB_xBLOCK_MASK          (0xC0U)      /*!< Bit mask for Block type       */
#define ISODEP_PCB_IBLOCK               (0x00U)      /*!< Bit mask indicating a I-Block */
#define ISODEP_PCB_RBLOCK               (0x80U)      /*!< Bit mask indicating a R-Block */
#define ISODEP_PCB_SBLOCK               (0xC0U)      /*!< Bit mask indicating a S-Block */
#define ISODEP_PCB_INVALID              (0x40U)      /*!< Bit mask of an Invalid PCB    */

#define ISODEP_HDR_MAX_LEN              (RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN + RFAL_ISODEP_NAD_LEN)          /*!< Max header length (PCB + DID + NAD)      */

#define ISODEP_PCB_IB_VALID_MASK        (ISODEP_PCB_B6_BIT | ISODEP_PCB_B2_BIT)                     /*!< Bit mask for the MUST bits on I-Block    */
#define ISODEP_PCB_IB_VALID_VAL         (ISODEP_PCB_B2_BIT)                                         /*!< Value for the MUST bits on I-Block       */
#define ISODEP_PCB_RB_VALID_MASK        (ISODEP_PCB_B6_BIT | ISODEP_PCB_B3_BIT | ISODEP_PCB_B2_BIT) /*!< Bit mask for the MUST bits on R-Block    */
#define ISODEP_PCB_RB_VALID_VAL         (ISODEP_PCB_B6_BIT | ISODEP_PCB_B2_BIT)                     /*!< Value for the MUST bits on R-Block       */
#define ISODEP_PCB_SB_VALID_MASK        (ISODEP_PCB_B3_BIT | ISODEP_PCB_B2_BIT | ISODEP_PCB_B1_BIT) /*!< Bit mask for the MUST bits on I-Block    */
#define ISODEP_PCB_SB_VALID_VAL         (ISODEP_PCB_B2_BIT)                                         /*!< Value for the MUST bits on I-Block       */


#define ISODEP_PCB_B1_BIT               (0x01U)      /*!< Bit mask for the RFU S Blocks                                        */
#define ISODEP_PCB_B2_BIT               (0x02U)      /*!< Bit mask for the RFU bit2 in I,S,R Blocks                            */
#define ISODEP_PCB_B3_BIT               (0x04U)      /*!< Bit mask for the RFU bit3 in R Blocks                                */
#define ISODEP_PCB_B6_BIT               (0x20U)      /*!< Bit mask for the RFU bit2 in R Blocks                                */
#define ISODEP_PCB_CHAINING_BIT         (0x10U)      /*!< Bit mask for the chaining bit of an ISO DEP I-Block in PCB.          */
#define ISODEP_PCB_DID_BIT              (0x08U)      /*!< Bit mask for the DID presence bit of an ISO DEP I,S,R Blocks PCB.    */
#define ISODEP_PCB_NAD_BIT              (0x04U)      /*!< Bit mask for the NAD presence bit of an ISO DEP I,S,R Blocks in PCB  */
#define ISODEP_PCB_BN_MASK              (0x01U)      /*!< Bit mask for the block number of an ISO DEP I,R Block in PCB         */

#define ISODEP_SWTX_PL_MASK             (0xC0U)      /*!< Bit mask for the Power Level bits of the inf byte of an WTX request or response */
#define ISODEP_SWTX_WTXM_MASK           (0x3FU)      /*!< Bit mask for the WTXM bits of the inf byte of an WTX request or response        */


#define ISODEP_RBLOCK_INF_LEN           (0U)         /*!< INF length of R-Block               Digital 1.1 15.1.3 */
#define ISODEP_SDSL_INF_LEN             (0U)         /*!< INF length of S(DSL)                Digital 1.1 15.1.3 */
#define ISODEP_SWTX_INF_LEN             (1U)         /*!< INF length of S(WTX)                Digital 1.1 15.2.2 */

#define ISODEP_WTXM_MIN                 (1U)         /*!< Minimum allowed value for the WTXM, Digital 1.0 13.2.2 */
#define ISODEP_WTXM_MAX                 (59U)        /*!< Maximum allowed value for the WTXM, Digital 1.0 13.2.2 */

#define ISODEP_PCB_Sxx_MASK             (0x30U)      /*!< Bit mask for the S-Block type                          */
#define ISODEP_PCB_DESELECT             (0x00U)      /*!< Bit mask for S-Block indicating Deselect               */
#define ISODEP_PCB_WTX                  (0x30U)      /*!< Bit mask for S-Block indicating Waiting Time eXtension */

#define ISODEP_PCB_Rx_MASK              (0x10U)      /*!< Bit mask for the R-Block type       */
#define ISODEP_PCB_ACK                  (0x00U)      /*!< Bit mask for R-Block indicating ACK */
#define ISODEP_PCB_NAK                  (0x10U)      /*!< Bit mask for R-Block indicating NAK */

/*! Maximum length of control message (no INF) */
#define ISODEP_CONTROLMSG_BUF_LEN       (RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_DID_LEN + RFAL_ISODEP_NAD_LEN + ISODEP_SWTX_PARAM_LEN)

#define ISODEP_FWT_DEACTIVATION         (71680U)     /*!< FWT to be used after DESELECT, Digital 1.0 A9   */
#define ISODEP_MAX_RERUNS               (0x0FFFFFFFU)/*!< Maximum rerun retrys for a blocking protocol run*/


#define ISODEP_PCBSBLOCK                ( 0x00U | ISODEP_PCB_SBLOCK | ISODEP_PCB_B2_BIT ) /*!< PCB Value of a S-Block                 */ 
#define ISODEP_PCB_SDSL                 ( ISODEP_PCBSBLOCK | ISODEP_PCB_DESELECT )        /*!< PCB Value of a S-Block with DESELECT   */
#define ISODEP_PCB_SWTX                 ( ISODEP_PCBSBLOCK | ISODEP_PCB_WTX )             /*!< PCB Value of a S-Block with WTX        */
#define ISODEP_PCB_SPARAMETERS          ( ISODEP_PCB_SBLOCK | ISODEP_PCB_WTX )            /*!< PCB Value of a S-Block with PARAMETERS */

#define ISODEP_FWI_LIS_MAX_NFC          8U                            /*!< FWT Listener Max FWIT4ATmax FWIBmax  Digital 1.1  A6 & A3  */
#define ISODEP_FWI_LIS_MAX_EMVCO        7U                            /*!< FWT Listener Max FWIMAX       EMVCo 2.6 A.5                */
#define ISODEP_FWI_LIS_MAX              (uint8_t)((gIsoDep.compMode == RFAL_COMPLIANCE_MODE_EMV) ? ISODEP_FWI_LIS_MAX_EMVCO : ISODEP_FWI_LIS_MAX_NFC)  /*!< FWI Listener Max as NFC / EMVCo */
#define ISODEP_FWT_LIS_MAX              rfalIsoDepFWI2FWT(ISODEP_FWI_LIS_MAX)             /*!< FWT Listener Max                       */

#define ISODEP_FWI_MIN_10               (1U)      /*!< Minimum value for FWI Digital 1.0 11.6.2.17 */
#define ISODEP_FWI_MIN_11               (0U)      /*!< Default value for FWI Digital 1.1 13.6.2    */
#define ISODEP_FWI_MAX                  (14U)     /*!< Maximum value for FWI Digital 1.0 11.6.2.17 */
#define ISODEP_SFGI_MIN                 (0U)      /*!< Default value for FWI Digital 1.1 13.6.2.22 */
#define ISODEP_SFGI_MAX                 (14U)     /*!< Maximum value for FWI Digital 1.1 13.6.2.22 */


#define RFAL_ISODEP_SPARAM_TVL_HDR_LEN  (2U)                                                   /*!< S(PARAMETERS) TVL header length: Tag + Len */
#define RFAL_ISODEP_SPARAM_HDR_LEN      (RFAL_ISODEP_PCB_LEN + RFAL_ISODEP_SPARAM_TVL_HDR_LEN) /*!< S(PARAMETERS) header length: PCB + Tag + Len */


/**********************************************************************************************************************/
/**********************************************************************************************************************/
#define RFAL_ISODEP_NO_PARAM                   (0U)     /*!< No parameter flag for isoDepHandleControlMsg()     */

#define RFAL_ISODEP_CMD_RATS                   (0xE0U)  /*!< RATS command   Digital 1.1  13.6.1                 */

#define RFAL_ISODEP_ATS_MIN_LEN                (1U)                                                   /*!< Minimum ATS length   Digital 1.1  13.6.2 */
#define RFAL_ISODEP_ATS_HDR_LEN                (5U)                                                   /*!< ATS headerlength     Digital 1.1  13.6.2 */
#define RFAL_ISODEP_ATS_MAX_LEN                (RFAL_ISODEP_ATS_HDR_LEN + RFAL_ISODEP_ATS_HB_MAX_LEN) /*!< Maximum ATS length   Digital 1.1  13.6.2 */
#define RFAL_ISODEP_ATS_T0_FSCI_MASK           (0x0FU)                                                /*!< ATS T0's FSCI mask   Digital 1.1  13.6.2 */
#define RFAL_ISODEP_ATS_TB_FWI_SHIFT           (4U)                                                   /*!< ATS TB's FWI shift   Digital 1.1  13.6.2 */
#define RFAL_ISODEP_ATS_FWI_MASK               (0x0FU)                                                /*!< ATS TB's FWI shift   Digital 1.1  13.6.2 */


#define RFAL_ISODEP_PPS_SB                     (0xD0U)  /*!< PPS REQ PPSS's SB value (no CID)   ISO14443-4  5.3 */
#define RFAL_ISODEP_PPS_MASK                   (0xF0U)  /*!< PPS REQ PPSS's SB mask             ISO14443-4  5.3 */
#define RFAL_ISODEP_PPS_SB_DID_MASK            (0x0FU)  /*!< PPS REQ PPSS's DID|CID mask        ISO14443-4  5.3 */
#define RFAL_ISODEP_PPS_PPS0_PPS1_PRESENT      (0x11U)  /*!< PPS REQ PPS0 indicating that PPS1 is present       */
#define RFAL_ISODEP_PPS_PPS1                   (0x00U)  /*!< PPS REQ PPS1 fixed value           ISO14443-4  5.3 */
#define RFAL_ISODEP_PPS_PPS1_DSI_SHIFT         (2U)     /*!< PPS REQ PPS1 fixed value           ISO14443-4  5.3 */
#define RFAL_ISODEP_PPS_PPS1_DXI_MASK          (0x0FU)  /*!< PPS REQ PPS1 fixed value           ISO14443-4  5.3 */
#define RFAL_ISODEP_PPS_RES_LEN                (1U)     /*!< PPS Response length                ISO14443-4  5.4 */
#define RFAL_ISODEP_PPS_STARTBYTE_POS          (0U)     /*!< PPS REQ PPSS's byte position       ISO14443-4  5.4 */
#define RFAL_ISODEP_PPS_PPS0_POS               (1U)     /*!< PPS REQ PPS0's byte position       ISO14443-4  5.4 */
#define RFAL_ISODEP_PPS_PPS1_POS               (2U)     /*!< PPS REQ PPS1's byte position       ISO14443-4  5.4 */
#define RFAL_ISODEP_PPS0_VALID_MASK            (0xEFU)  /*!< PPS REQ PPS0 valid coding mask     ISO14443-4  5.4 */

#define RFAL_ISODEP_CMD_ATTRIB                 (0x1DU)  /*!< ATTRIB command                 Digital 1.1  14.6.1 */
#define RFAL_ISODEP_ATTRIB_PARAM2_DSI_SHIFT    (6U)     /*!< ATTRIB PARAM2 DSI shift        Digital 1.1  14.6.1 */
#define RFAL_ISODEP_ATTRIB_PARAM2_DRI_SHIFT    (4U)     /*!< ATTRIB PARAM2 DRI shift        Digital 1.1  14.6.1 */
#define RFAL_ISODEP_ATTRIB_PARAM2_DXI_MASK     (0xF0U)  /*!< ATTRIB PARAM2 DxI mask         Digital 1.1  14.6.1 */
#define RFAL_ISODEP_ATTRIB_PARAM2_FSDI_MASK    (0x0FU)  /*!< ATTRIB PARAM2 FSDI mask        Digital 1.1  14.6.1 */
#define RFAL_ISODEP_ATTRIB_PARAM4_DID_MASK     (0x0FU)  /*!< ATTRIB PARAM4 DID mask         Digital 1.1  14.6.1 */
#define RFAL_ISODEP_ATTRIB_HDR_LEN             (9U)     /*!< ATTRIB REQ header length       Digital 1.1  14.6.1 */

#define RFAL_ISODEP_ATTRIB_RES_HDR_LEN         (1U)     /*!< ATTRIB RES header length       Digital 1.1  14.6.2 */
#define RFAL_ISODEP_ATTRIB_RES_DID_MASK        (0x0FU)  /*!< ATTRIB RES DID mask            Digital 1.1  14.6.2 */
#define RFAL_ISODEP_ATTRIB_RES_MBLI_MASK       (0x0FU)  /*!< ATTRIB RES MBLI mask           Digital 1.1  14.6.2 */
#define RFAL_ISODEP_ATTRIB_RES_MBLI_SHIFT      (4U)     /*!< ATTRIB RES MBLI shift          Digital 1.1  14.6.2 */

#define RFAL_ISODEP_DID_MASK                   (0x0FU)  /*!< ISODEP's DID mask                                  */
#define RFAL_ISODEP_DID_00                     (0U)     /*!< ISODEP's DID value 0                               */

#define RFAL_ISODEP_FSDI_MAX_NFC               (8U)     /*!< Max FSDI value   Digital 2.0  14.6.1.9 & B7 & B8   */
#define RFAL_ISODEP_FSDI_MAX_EMV               (0x0CU)  /*!< Max FSDI value   EMVCo 3.0  5.7.2.5                */

#define RFAL_ISODEP_RATS_PARAM_FSDI_MASK       (0xF0U)  /*!< Mask bits for FSDI in RATS                         */
#define RFAL_ISODEP_RATS_PARAM_FSDI_SHIFT      (4U)     /*!< Shift for FSDI in RATS                             */
#define RFAL_ISODEP_RATS_PARAM_DID_MASK        (0x0FU)  /*!< Mask bits for DID in RATS                          */

#define RFAL_ISODEP_ATS_TL_OFFSET              (0x00U)  /*!< Offset of TL on ATS                                */
#define RFAL_ISODEP_ATS_TA_OFFSET              (0x02U)  /*!< Offset of TA if it is present on ATS               */
#define RFAL_ISODEP_ATS_TB_OFFSET              (0x03U)  /*!< Offset of TB if both TA and TB is present on ATS   */
#define RFAL_ISODEP_ATS_TC_OFFSET              (0x04U)  /*!< Offset of TC if both TA,TB & TC are present on ATS */
#define RFAL_ISODEP_ATS_HIST_OFFSET            (0x05U)  /*!< Offset of Historical Bytes if TA, TB & TC are present on ATS          */
#define RFAL_ISODEP_ATS_TC_ADV_FEAT            (0x10U)  /*!< Bit mask indicating support for Advanced protocol features: DID & NAD */
#define RFAL_ISODEP_ATS_TC_DID                 (0x02U)  /*!< Bit mask indicating support for DID                 */
#define RFAL_ISODEP_ATS_TC_NAD                 (0x01U)  /*!< Bit mask indicating support for NAD                 */

#define RFAL_ISODEP_PPS0_PPS1_PRESENT          (0x11U) /*!< PPS0 byte indicating that PPS1 is present            */
#define RFAL_ISODEP_PPS0_PPS1_NOT_PRESENT      (0x01U) /*!< PPS0 byte indicating that PPS1 is NOT present        */
#define RFAL_ISODEP_PPS1_DRI_MASK              (0x03U) /*!< PPS1 byte DRI mask bits                              */
#define RFAL_ISODEP_PPS1_DSI_MASK              (0x0CU) /*!< PPS1 byte DSI mask bits                              */
#define RFAL_ISODEP_PPS1_DSI_SHIFT             (2U)    /*!< PPS1 byte DSI shift                                  */
#define RFAL_ISODEP_PPS1_DxI_MASK              (0x03U) /*!< PPS1 byte DSI/DRS mask bits                          */


/*! Delta Time for polling during Activation (ATS) : 20ms    Digital 1.0 11.7.1.1 & A.7    */
#define RFAL_ISODEP_T4T_DTIME_POLL_10          rfalConvMsTo1fc(20)

/*! Delta Time for polling during Activation (ATS) : 16.4ms  Digital 1.1 13.8.1.1 & A.6
 *  Use 16 ms as testcase T4AT_BI_10_03 sends a frame exactly at the border */
#define RFAL_ISODEP_T4T_DTIME_POLL_11          216960U

/*! Activation frame waiting time FWT(act) = 71680/fc (~5286us) Digital 1.1 13.8.1.1 & A.6 */
#define RFAL_ISODEP_T4T_FWT_ACTIVATION         (71680U + RFAL_ISODEP_T4T_DTIME_POLL_11)


/*! Delta frame waiting time = 16/fc  Digital 1.0  11.7.1.3 & A.7*/
#define RFAL_ISODEP_DFWT_10                      16U

/*! Delta frame waiting time = 16/fc  Digital 2.0  14.8.1.3 & B.7*/
#define RFAL_ISODEP_DFWT_20                      49152U




#define RFAL_ISODEP_PCB_LEN                     (1U)     /*!< PCB length                                                        */
#define RFAL_ISODEP_DID_LEN                     (1U)     /*!< DID length                                                        */
#define RFAL_ISODEP_NAD_LEN                     (1U)     /*!< NAD length                                                        */
#define RFAL_ISODEP_NO_DID                      (0x00U)  /*!< DID value indicating the ISO-DEP layer not to use DID             */
#define RFAL_ISODEP_NO_NAD                      (0xFFU)  /*!< NAD value indicating the ISO-DEP layer not to use NAD             */

#define RFAL_ISODEP_FWI_MASK                    (0xF0U)  /*!< Mask bits of FWI                                                  */
#define RFAL_ISODEP_FWI_SHIFT                   (4U)     /*!< Shift val of FWI                                                  */
#define RFAL_ISODEP_FWI_DEFAULT                 (4U)     /*!< Default value for FWI Digital 1.0 11.6.2.17                       */
#define RFAL_ISODEP_ADV_FEATURE                 (0x0FU)  /*!< Indicate 256 Bytes FSD and Advanc Proto Feature support:NAD & DID */

#define RFAL_ISODEP_DID_MAX                     (14U)    /*!< Maximum DID value                                                 */

#define RFAL_ISODEP_BRI_MASK                    (0x07U)  /*!< Mask bits for Poll to Listen Send bitrate                         */
#define RFAL_ISODEP_BSI_MASK                    (0x70U)  /*!< Mask bits for Listen to Poll Send bitrate                         */
#define RFAL_ISODEP_SAME_BITRATE_MASK           (0x80U)  /*!< Mask bit indicate only same bit rate D for both direction support */
#define RFAL_ISODEP_BITRATE_RFU_MASK            (0x08U)  /*!< Mask bit for RFU                                                  */

/*! Maximum Frame Waiting Time = ((256 * 16/fc) * 2^FWImax) = ((256*16/fc)*2^14) = (67108864)/fc = 2^26 (1/fc)                  */
#define RFAL_ISODEP_MAX_FWT                     ((uint32_t)1U<<26)



#define RFAL_ISODEP_FSDI_DEFAULT                RFAL_ISODEP_FSXI_256  /*!< Default Frame Size Integer in Poll mode              */
#define RFAL_ISODEP_FSX_KEEP                    (0xFFU)               /*!< Flag to keep FSX from activation                     */
#define RFAL_ISODEP_DEFAULT_FSCI                RFAL_ISODEP_FSXI_256  /*!< FSCI default value to be used  in Listen Mode        */
#define RFAL_ISODEP_DEFAULT_FSC                 RFAL_ISODEP_FSX_256   /*!< FSC default value (aligned RFAL_ISODEP_DEFAULT_FSCI) */
#define RFAL_ISODEP_DEFAULT_SFGI                (0U)                  /*!< SFGI Default value to be used  in Listen Mode        */
#define RFAL_ISODEP_DEFAULT_FWI                 (8U)                  /*!< Default Listener FWI (Max)      Digital 2.0  B7 & B3 */

#define RFAL_ISODEP_APDU_MAX_LEN                RFAL_ISODEP_FSX_1024  /*!< Max APDU length                                      */

#define RFAL_ISODEP_ATTRIB_RES_MBLI_NO_INFO     (0x00U)  /*!< MBLI indicating no information on its internal input buffer size  */
#define RFAL_ISODEP_ATTRIB_REQ_PARAM1_DEFAULT   (0x00U)  /*!< Default values of Param 1 of ATTRIB_REQ Digital 1.0  12.6.1.3-5   */
#define RFAL_ISODEP_ATTRIB_HLINFO_LEN           (32U)    /*!< Maximum Size of Higher Layer Information                          */
#define RFAL_ISODEP_ATS_HB_MAX_LEN              (15U)    /*!< Maximum length of Historical Bytes  Digital 1.1  13.6.2.23        */
#define RFAL_ISODEP_ATTRIB_REQ_MIN_LEN          (9U)     /*!< Minimum Length of ATTRIB_REQ command                              */
#define RFAL_ISODEP_ATTRIB_RES_MIN_LEN          (1U)     /*!< Minimum Length of ATTRIB_RES response                             */

#define RFAL_ISODEP_SPARAM_VALUES_MAX_LEN       (16U)    /*!< Maximum Length of the value field on S(PARAMETERS)                */
#define RFAL_ISODEP_SPARAM_TAG_BLOCKINFO        (0xA0U)  /*!< S(PARAMETERS) tag Block information                               */
#define RFAL_ISODEP_SPARAM_TAG_BRREQ            (0xA1U)  /*!< S(PARAMETERS) tag Bit rates Request                               */
#define RFAL_ISODEP_SPARAM_TAG_BRIND            (0xA2U)  /*!< S(PARAMETERS) tag Bit rates Indication                            */
#define RFAL_ISODEP_SPARAM_TAG_BRACT            (0xA3U)  /*!< S(PARAMETERS) tag Bit rates Activation                            */
#define RFAL_ISODEP_SPARAM_TAG_BRACK            (0xA4U)  /*!< S(PARAMETERS) tag Bit rates Acknowledgement                       */

#define RFAL_ISODEP_SPARAM_TAG_SUP_PCD2PICC     (0x80U)  /*!< S(PARAMETERS) tag Supported bit rates from PCD to PICC            */
#define RFAL_ISODEP_SPARAM_TAG_SUP_PICC2PCD     (0x81U)  /*!< S(PARAMETERS) tag Supported bit rates from PICC to PCD            */
#define RFAL_ISODEP_SPARAM_TAG_SUP_FRAME        (0x82U)  /*!< S(PARAMETERS) tag Supported framing options PICC to PCD           */
#define RFAL_ISODEP_SPARAM_TAG_SEL_PCD2PICC     (0x83U)  /*!< S(PARAMETERS) tag Selected bit rate from PCD to PICC              */
#define RFAL_ISODEP_SPARAM_TAG_SEL_PICC2PCD     (0x84U)  /*!< S(PARAMETERS) tag Selected bit rate from PICC to PCD              */
#define RFAL_ISODEP_SPARAM_TAG_SEL_FRAME        (0x85U)  /*!< S(PARAMETERS) tag Selected framing options PICC to PCD            */

#define RFAL_ISODEP_SPARAM_TAG_LEN              (1)      /*!< S(PARAMETERS) Tag Length                                          */
#define RFAL_ISODEP_SPARAM_TAG_BRREQ_LEN        (0U)     /*!< S(PARAMETERS) tag Bit rates Request Length                        */
#define RFAL_ISODEP_SPARAM_TAG_PICC2PCD_LEN     (2U)     /*!< S(PARAMETERS) bit rates from PCD to PICC Length                   */
#define RFAL_ISODEP_SPARAM_TAG_PCD2PICC_LEN     (2U)     /*!< S(PARAMETERS) bit rates from PICC to PCD Length                   */
#define RFAL_ISODEP_SPARAM_TAG_BRACK_LEN        (0U)     /*!< S(PARAMETERS) tag Bit rates Acknowledgement Length                */

#define RFAL_ISODEP_ATS_TA_DPL_212              (0x01U)  /*!< ATS TA DSI 212 kbps support bit mask                              */
#define RFAL_ISODEP_ATS_TA_DPL_424              (0x02U)  /*!< ATS TA DSI 424 kbps support bit mask                              */
#define RFAL_ISODEP_ATS_TA_DPL_848              (0x04U)  /*!< ATS TA DSI 848 kbps support bit mask                              */
#define RFAL_ISODEP_ATS_TA_DLP_212              (0x10U)  /*!< ATS TA DSI 212 kbps support bit mask                              */
#define RFAL_ISODEP_ATS_TA_DLP_424              (0x20U)  /*!< ATS TA DRI 424 kbps support bit mask                              */
#define RFAL_ISODEP_ATS_TA_DLP_848              (0x40U)  /*!< ATS TA DRI 848 kbps support bit mask                              */
#define RFAL_ISODEP_ATS_TA_SAME_D               (0x80U)  /*!< ATS TA same bit both directions bit mask                          */
#define RFAL_ISODEP_ATS_TB_FWI_MASK             (0xF0U)  /*!< Mask bits for FWI (Frame Waiting Integer) in TB byte              */
#define RFAL_ISODEP_ATS_TB_SFGI_MASK            (0x0FU)  /*!< Mask bits for SFGI (Start-Up Frame Guard Integer) in TB byte      */

#define RFAL_ISODEP_ATS_T0_TA_PRESENCE_MASK     (0x10U)  /*!< Mask bit for TA presence                                          */
#define RFAL_ISODEP_ATS_T0_TB_PRESENCE_MASK     (0x20U)  /*!< Mask bit for TB presence                                          */
#define RFAL_ISODEP_ATS_T0_TC_PRESENCE_MASK     (0x40U)  /*!< Mask bit for TC presence                                          */
#define RFAL_ISODEP_ATS_T0_FSCI_MASK            (0x0FU)  /*!< Mask bit for FSCI presence                                        */
#define RFAL_ISODEP_ATS_T0_OFFSET               (0x01U)  /*!< Offset of T0 in ATS Response                                      */


#define RFAL_ISODEP_MAX_I_RETRYS                (2U)     /*!< Number of retries for a I-Block  Digital 1.1   15.2.5.4                            */
#define RFAL_ISODEP_MAX_R_RETRYS                (3U)     /*!< Number of retries for a R-Block  Digital 1.1 A8 - nRETRY ACK/NAK:  [2,5]           */
#define RFAL_ISODEP_MAX_S_RETRYS                (3U)     /*!< Number of retries for a S-Block  Digital 1.1 A8 - nRETRY DESELECT: [0,5] WTX[2,5]  */
#define RFAL_ISODEP_RATS_RETRIES                (1U)     /*!< RATS retries upon fail           Digital 1.1  A.6 - [0,1]                          */
 

/*! Frame Size for Proximity Card Integer definitions                                                               */
typedef enum
{
      RFAL_ISODEP_FSXI_16   =  0,  /*!< Frame Size for Proximity Card Integer with 16 bytes                         */
      RFAL_ISODEP_FSXI_24   =  1,  /*!< Frame Size for Proximity Card Integer with 24 bytes                         */
      RFAL_ISODEP_FSXI_32   =  2,  /*!< Frame Size for Proximity Card Integer with 32 bytes                         */
      RFAL_ISODEP_FSXI_40   =  3,  /*!< Frame Size for Proximity Card Integer with 40 bytes                         */
      RFAL_ISODEP_FSXI_48   =  4,  /*!< Frame Size for Proximity Card Integer with 48 bytes                         */
      RFAL_ISODEP_FSXI_64   =  5,  /*!< Frame Size for Proximity Card Integer with 64 bytes                         */
      RFAL_ISODEP_FSXI_96   =  6,  /*!< Frame Size for Proximity Card Integer with 96 bytes                         */
      RFAL_ISODEP_FSXI_128  =  7,  /*!< Frame Size for Proximity Card Integer with 128 bytes                        */
      RFAL_ISODEP_FSXI_256  =  8,  /*!< Frame Size for Proximity Card Integer with 256 bytes                        */
      RFAL_ISODEP_FSXI_512  =  9,  /*!< Frame Size for Proximity Card Integer with 512 bytes   ISO14443-3 Amd2 2012 */
      RFAL_ISODEP_FSXI_1024 = 10,  /*!< Frame Size for Proximity Card Integer with 1024 bytes  ISO14443-3 Amd2 2012 */
      RFAL_ISODEP_FSXI_2048 = 11,  /*!< Frame Size for Proximity Card Integer with 2048 bytes  ISO14443-3 Amd2 2012 */
      RFAL_ISODEP_FSXI_4096 = 12   /*!< Frame Size for Proximity Card Integer with 4096 bytes  ISO14443-3 Amd2 2012 */
} rfalIsoDepFSxI;

/*! Frame Size for Proximity Card  definitions                                                             */
typedef enum
{
    RFAL_ISODEP_FSX_16   = 16,    /*!< Frame Size for Proximity Card with 16 bytes                         */
    RFAL_ISODEP_FSX_24   = 24,    /*!< Frame Size for Proximity Card with 24 bytes                         */
    RFAL_ISODEP_FSX_32   = 32,    /*!< Frame Size for Proximity Card with 32 bytes                         */
    RFAL_ISODEP_FSX_40   = 40,    /*!< Frame Size for Proximity Card with 40 bytes                         */
    RFAL_ISODEP_FSX_48   = 48,    /*!< Frame Size for Proximity Card with 48 bytes                         */
    RFAL_ISODEP_FSX_64   = 64,    /*!< Frame Size for Proximity Card with 64 bytes                         */
    RFAL_ISODEP_FSX_96   = 96,    /*!< Frame Size for Proximity Card with 96 bytes                         */
    RFAL_ISODEP_FSX_128  = 128,   /*!< Frame Size for Proximity Card with 128 bytes                        */
    RFAL_ISODEP_FSX_256  = 256,   /*!< Frame Size for Proximity Card with 256 bytes                        */
    RFAL_ISODEP_FSX_512  = 512,   /*!< Frame Size for Proximity Card with 512 bytes   ISO14443-3 Amd2 2012 */
    RFAL_ISODEP_FSX_1024 = 1024,  /*!< Frame Size for Proximity Card with 1024 bytes  ISO14443-3 Amd2 2012 */
    RFAL_ISODEP_FSX_2048 = 2048,  /*!< Frame Size for Proximity Card with 2048 bytes  ISO14443-3 Amd2 2012 */
    RFAL_ISODEP_FSX_4096 = 4096,  /*!< Frame Size for Proximity Card with 4096 bytes  ISO14443-3 Amd2 2012 */
} rfalIsoDepFSx;


/*! RATS format  Digital 1.1 13.6.1                                                               */
typedef struct
{
    uint8_t      CMD;                               /*!< RATS command byte: 0xE0                  */
    uint8_t      PARAM;                             /*!< Param indicating FSDI and DID            */
} rfalIsoDepRats;


/*! ATS response format  Digital 1.1 13.6.2                                                       */
typedef struct
{
  uint8_t        TL;                                /*!< Length Byte, including TL byte itself    */
  uint8_t        T0;                                /*!< Format Byte T0 indicating if TA, TB, TC  */
  uint8_t        TA;                                /*!< Interface Byte TA(1)                     */
  uint8_t        TB;                                /*!< Interface Byte TB(1)                     */
  uint8_t        TC;                                /*!< Interface Byte TC(1)                     */
  uint8_t        HB[RFAL_ISODEP_ATS_HB_MAX_LEN];    /*!< Historical Bytes                         */
} rfalIsoDepAts;


/*! PPS Request format (Protocol and Parameter Selection) ISO14443-4  5.3                         */
typedef struct
{
    uint8_t      PPSS;                              /*!< Start Byte: [ 1101b | CID[4b] ]          */
    uint8_t      PPS0;                              /*!< Parameter 0:[ 000b | PPS1[1n] | 0001b ]  */
    uint8_t      PPS1;                              /*!< Parameter 1:[ 0000b | DSI[2b] | DRI[2b] ]*/    
} rfalIsoDepPpsReq;


/*! PPS Response format (Protocol and Parameter Selection) ISO14443-4  5.4                        */
typedef struct
{
    uint8_t      PPSS;                              /*!< Start Byte:  [ 1101b | CID[4b] ]         */
} rfalIsoDepPpsRes;


/*! ATTRIB Command Format  Digital 1.1  15.6.1 */
typedef struct
{
    uint8_t         cmd;                                   /*!< ATTRIB_REQ command byte           */
    uint8_t         nfcid0[RFAL_NFCB_NFCID0_LEN];          /*!< NFCID0 of the card to be selected */
    struct{
            uint8_t PARAM1;                                /*!< PARAM1 of ATTRIB command          */
            uint8_t PARAM2;                                /*!< PARAM2 of ATTRIB command          */
            uint8_t PARAM3;                                /*!< PARAM3 of ATTRIB command          */
            uint8_t PARAM4;                                /*!< PARAM4 of ATTRIB command          */
    }Param;                                                /*!< Parameter of ATTRIB command       */
    uint8_t         HLInfo[RFAL_ISODEP_ATTRIB_HLINFO_LEN]; /*!< Higher Layer Information          */
} rfalIsoDepAttribCmd;


/*! ATTRIB Response Format  Digital 1.1  15.6.2 */
typedef struct
{
    uint8_t         mbliDid;                               /*!< Contains MBLI and DID             */
    uint8_t         HLInfo[RFAL_ISODEP_ATTRIB_HLINFO_LEN]; /*!< Higher Layer Information          */
} rfalIsoDepAttribRes;

/*! S(Parameters) Command Format  ISO14443-4 (2016) Table 4 */
typedef struct
{
    uint8_t         tag;                                      /*!< S(PARAMETERS) Tag field        */
    uint8_t         length;                                   /*!< S(PARAMETERS) Length field     */
    uint8_t         value[RFAL_ISODEP_SPARAM_VALUES_MAX_LEN]; /*!< S(PARAMETERS) Value field      */
} rfalIsoDepSParameter;


/*! Activation info as Poller and Listener for NFC-A and NFC-B                                    */
typedef union {/*  PRQA S 0750 # MISRA 19.2 - Both members of the union will not be used concurrently, device is only of type A or B at a time. Thus no problem can occur. */

    /*! NFC-A information                                                                         */
    union {/*  PRQA S 0750 # MISRA 19.2 - Both members of the union will not be used concurrently, device is only PCD or PICC at a time. Thus no problem can occur. */
        struct {
            rfalIsoDepAts        ATS;               /*!< ATS response            (Poller mode)    */
            uint8_t              ATSLen;            /*!< ATS response length     (Poller mode)    */
            }Listener;
        struct {
            rfalIsoDepRats      RATS;               /*!< RATS request          (Listener mode)    */
        }Poller;
    }A;
    
    /*! NFC-B information                                                                         */
    union {/*  PRQA S 0750 # MISRA 19.2 - Both members of the union will not be used concurrently, device is only PCD or PICC at a time. Thus no problem can occur. */
        struct{
            rfalIsoDepAttribRes  ATTRIB_RES;        /*!< ATTRIB_RES              (Poller mode)    */
            uint8_t              ATTRIB_RESLen;     /*!< ATTRIB_RES length       (Poller mode)    */
        }Listener;
        struct{
            rfalIsoDepAttribCmd  ATTRIB;            /*!< ATTRIB request        (Listener mode)    */
            uint8_t              ATTRIBLen;         /*!< ATTRIB request length (Listener mode)    */
        }Poller;
    }B;
}rfalIsoDepActivation;


/*! ISO-DEP device Info */
typedef struct {
    uint8_t            FWI;             /*!< Frame Waiting Integer                                */
    uint32_t           FWT;             /*!< Frame Waiting Time (1/fc)                            */
    uint32_t           dFWT;            /*!< Delta Frame Waiting Time (1/fc)                      */
    uint32_t           SFGI;            /*!< Start-up Frame Guard time Integer                    */
    uint32_t           SFGT;            /*!< Start-up Frame Guard Time (ms)                       */
    uint8_t            FSxI;            /*!< Frame Size Device/Card Integer (FSDI or FSCI)        */
    uint16_t           FSx;             /*!< Frame Size Device/Card (FSD or FSC)                  */
    uint32_t           MBL;             /*!< Maximum Buffer Length (optional for NFC-B)           */
    rfalBitRate        DSI;             /*!< Bit Rate coding from Listener (PICC) to Poller (PCD) */
    rfalBitRate        DRI;             /*!< Bit Rate coding from Poller (PCD) to Listener (PICC) */
    uint8_t            DID;             /*!< Device ID                                            */
    uint8_t            NAD;             /*!< Node ADdress                                         */
    bool               supDID;          /*!< DID supported flag                                   */
    bool               supNAD;          /*!< NAD supported flag                                   */
    bool               supAdFt;         /*!< Advanced Features supported flag                     */
} rfalIsoDepInfo;


/*! ISO-DEP Device structure */
typedef struct {
    rfalIsoDepActivation    activation; /*!< Activation Info                                      */
    rfalIsoDepInfo          info;       /*!< ISO-DEP (ISO14443-4) device Info                     */
} rfalIsoDepDevice;


/*! ATTRIB Response parameters */
typedef struct
{
    uint8_t  mbli;                                     /*!< MBLI                                     */
    uint8_t  HLInfo[RFAL_ISODEP_ATTRIB_HLINFO_LEN];    /*!< Hi Layer Information                     */
    uint8_t  HLInfoLen;                                /*!< Hi Layer Information Length              */
} rfalIsoDepAttribResParam;


/*! ATS Response parameter */
typedef struct
{
    uint8_t     fsci;                                  /*!< Frame Size of Proximity Card Integer     */
    uint8_t     fwi;                                   /*!< Frame Waiting Time Integer               */
    uint8_t     sfgi;                                  /*!< Start-Up Frame Guard Time Integer        */
    bool        didSupport;                            /*!< DID Supported                            */
    uint8_t     ta;                                    /*!< Max supported bitrate both direction     */
    uint8_t     *hb;                                   /*!< Historical Bytes data                    */
    uint8_t     hbLen;                                 /*!< Historical Bytes Length                  */
} rfalIsoDepAtsParam;


/*! Structure of APDU Buffer format from caller */
typedef struct
{
    uint8_t  prologue[RFAL_ISODEP_PROLOGUE_SIZE];      /*!< Prologue/SoD buffer                      */
    uint8_t  apdu[RFAL_FEATURE_ISO_DEP_APDU_MAX_LEN];  /*!< APDU/Payload buffer                      */
} rfalIsoDepApduBufFormat;


/*! Listen Activation Parameters Structure */
typedef struct
{
    rfalIsoDepBufFormat  *rxBuf;                       /*!< Receive Buffer struct reference          */
    uint16_t             *rxLen;                       /*!< Received INF data length in Bytes        */
    bool                 *isRxChaining;                /*!< Received data is not complete            */
    rfalIsoDepDevice     *isoDepDev;                   /*!< ISO-DEP device info                      */
} rfalIsoDepListenActvParam;





/*! Structure of parameters used on ISO DEP APDU Transceive */
typedef struct
{
    rfalIsoDepApduBufFormat  *txBuf;                   /*!< Transmit Buffer struct reference         */
    uint16_t                 txBufLen;                 /*!< Transmit Buffer INF field length in Bytes*/
    rfalIsoDepApduBufFormat  *rxBuf;                   /*!< Receive Buffer struct reference in Bytes */
    uint16_t                 *rxLen;                   /*!< Received INF data length in Bytes        */
    rfalIsoDepBufFormat      *tmpBuf;                  /*!< Temp buffer for Rx I-Blocks (internal)   */
    uint32_t                 FWT;                      /*!< FWT to be used (ignored in Listen Mode)  */
    uint32_t                 dFWT;                     /*!< Delta FWT to be used                     */
    uint16_t                 FSx;                      /*!< Other device Frame Size (FSD or FSC)     */
    uint16_t                 ourFSx;                   /*!< Our device Frame Size (FSD or FSC)       */
    uint8_t                  DID;                      /*!< Device ID (RFAL_ISODEP_NO_DID if no DID) */
} rfalIsoDepApduTxRxParam;
/*
 ******************************************************************************
 * MACROS
 ******************************************************************************
 */

#define isoDep_PCBisIBlock( pcb )       ( ((pcb) & (ISODEP_PCB_xBLOCK_MASK | ISODEP_PCB_IB_VALID_MASK)) == (ISODEP_PCB_IBLOCK | ISODEP_PCB_IB_VALID_VAL)) /*!< Checks if pcb is a I-Block */
#define isoDep_PCBisRBlock( pcb )       ( ((pcb) & (ISODEP_PCB_xBLOCK_MASK | ISODEP_PCB_RB_VALID_MASK)) == (ISODEP_PCB_RBLOCK | ISODEP_PCB_RB_VALID_VAL)) /*!< Checks if pcb is a R-Block */
#define isoDep_PCBisSBlock( pcb )       ( ((pcb) & (ISODEP_PCB_xBLOCK_MASK | ISODEP_PCB_SB_VALID_MASK)) == (ISODEP_PCB_SBLOCK | ISODEP_PCB_SB_VALID_VAL)) /*!< Checks if pcb is a S-Block */

#define isoDep_PCBisChaining( pcb )     ( ((pcb) & ISODEP_PCB_CHAINING_BIT) == ISODEP_PCB_CHAINING_BIT) /*!< Checks if pcb is indicating chaining */

#define isoDep_PCBisDeselect( pcb )     ( ((pcb) & ISODEP_PCB_Sxx_MASK) == ISODEP_PCB_DESELECT)         /*!< Checks if pcb is indicating DESELECT */
#define isoDep_PCBisWTX( pcb )          ( ((pcb) & ISODEP_PCB_Sxx_MASK) == ISODEP_PCB_WTX)              /*!< Checks if pcb is indicating WTX      */

#define isoDep_PCBisACK( pcb )          ( ((pcb) & ISODEP_PCB_Rx_MASK) == ISODEP_PCB_ACK)               /*!< Checks if pcb is indicating ACK      */
#define isoDep_PCBisNAK( pcb )          ( ((pcb) & ISODEP_PCB_Rx_MASK) == ISODEP_PCB_NAK)               /*!< Checks if pcb is indicating ACK      */

#define isoDep_PCBhasDID( pcb )         ( ((pcb) & ISODEP_PCB_DID_BIT) == ISODEP_PCB_DID_BIT)           /*!< Checks if pcb is indicating DID      */
#define isoDep_PCBhasNAD( pcb )         ( ((pcb) & ISODEP_PCB_NAD_BIT) == ISODEP_PCB_NAD_BIT)           /*!< Checks if pcb is indicating NAD      */


#define isoDep_PCBisIChaining( pcb )    ( isoDep_PCBisIBlock(pcb) && isoDep_PCBisChaining(pcb) )       /*!< Checks if pcb is I-Block indicating chaining*/

#define isoDep_PCBisSDeselect( pcb )    ( isoDep_PCBisSBlock(pcb) && isoDep_PCBisDeselect(pcb) )       /*!< Checks if pcb is S-Block indicating DESELECT*/
#define isoDep_PCBisSWTX( pcb )         ( isoDep_PCBisSBlock(pcb) && isoDep_PCBisWTX(pcb) )            /*!< Checks if pcb is S-Block indicating WTX     */

#define isoDep_PCBisRACK( pcb )         ( isoDep_PCBisRBlock(pcb) && isoDep_PCBisACK(pcb) )            /*!< Checks if pcb is R-Block indicating ACK     */
#define isoDep_PCBisRNAK( pcb )         ( isoDep_PCBisRBlock(pcb) && isoDep_PCBisNAK(pcb) )            /*!< Checks if pcb is R-Block indicating NAK     */


#define isoDep_PCBIBlock( bn )          ( (uint8_t)( 0x00U | ISODEP_PCB_IBLOCK | ISODEP_PCB_B2_BIT | ((bn) & ISODEP_PCB_BN_MASK) ))  /*!< Returns an I-Block with the given block number (bn)                     */  
#define isoDep_PCBIBlockChaining( bn )  ( (uint8_t)(isoDep_PCBIBlock(bn) | ISODEP_PCB_CHAINING_BIT))                              /*!< Returns an I-Block with the given block number (bn) indicating chaining */

#define isoDep_PCBRBlock( bn )          ( (uint8_t)( 0x00U | ISODEP_PCB_RBLOCK | ISODEP_PCB_B6_BIT | ISODEP_PCB_B2_BIT | ((bn) & ISODEP_PCB_BN_MASK) ) ) /*!< Returns an R-Block with the given block number (bn)                */
#define isoDep_PCBRACK( bn )            ( (uint8_t)( isoDep_PCBRBlock( bn ) | ISODEP_PCB_ACK ) )                                                      /*!< Returns an R-Block with the given block number (bn) indicating ACK */
#define isoDep_PCBRNAK( bn )            ( (uint8_t)( isoDep_PCBRBlock( bn ) | ISODEP_PCB_NAK ) )                                                      /*!< Returns an R-Block with the given block number (bn) indicating NAK */


#define isoDep_GetBN( pcb )             ( (uint8_t) ((pcb) & ISODEP_PCB_BN_MASK   ) )                    /*!< Returns the block number (bn) from the given pcb */
#define isoDep_GetWTXM( inf )           ( (uint8_t) ((inf) & ISODEP_SWTX_WTXM_MASK) )                    /*!< Returns the WTX value from the given inf byte    */
#define isoDep_isWTXMValid( wtxm )      (((wtxm) >= ISODEP_WTXM_MIN) && ((wtxm) <= ISODEP_WTXM_MAX))     /*!< Checks if the given wtxm is valid                */

#define isoDep_WTXMListenerMax( fwt )   ( MIN( (uint8_t)(ISODEP_FWT_LIS_MAX / (fwt)), ISODEP_WTXM_MAX) ) /*!< Calculates the Max WTXM value for the given fwt as a Listener    */

#define isoDepCalcdSGFT( s )            (384U  * ((uint32_t)1U << (s)))                                  /*!< Calculates the dSFGT with given SFGI  Digital 1.1  13.8.2.1 & A.6*/
#define isoDepCalcSGFT( s )             (4096U * ((uint32_t)1U << (s)))                                  /*!< Calculates the SFGT with given SFGI  Digital 1.1  13.8.2         */

#define isoDep_PCBNextBN( bn )          (((uint8_t)(bn)^0x01U) & ISODEP_PCB_BN_MASK)                     /*!< Returns the value of the next block number based on bn     */
#define isoDep_PCBPrevBN( bn )          isoDep_PCBNextBN(bn)                                             /*!< Returns the value of the previous block number based on bn */
#define isoDep_ToggleBN( bn )           ((bn) = (((bn)^0x01U) & ISODEP_PCB_BN_MASK) )                    /*!< Toggles the block number value of the given bn             */

#define isoDep_WTXAdjust( v )           ((v) - ((v)>>3))                                                 /*!< Adjust WTX timer value to a percentage of the total, current 88% */


/*! ISO 14443-4 7.5.6.2 & Digital 1.1 - 15.2.6.2  The CE SHALL NOT attempt error recovery and remains in Rx mode upon Transmission or a Protocol Error */
#define isoDepReEnableRx( rxB, rxBL, rxL )              rfalTransceiveBlockingTx( NULL, 0, rxB, rxBL, rxL, RFAL_TXRX_FLAGS_DEFAULT, RFAL_FWT_NONE )

#define isoDepTimerStart( timer, time_ms ) (timer) = platformTimerCreate((uint16_t)(time_ms))            /*!< Configures and starts the WTX timer  */
#define isoDepTimerisExpired( timer )      timerIsExpired( timer )                               /*!< Checks WTX timer has expired         */

/*! Internal structure to be used in handling of S(PARAMETRS) only */
typedef struct
{    
    uint8_t               pcb;       /*!< PCB byte                      */
    rfalIsoDepSParameter  sParam;    /*!< S(PARAMETERS)                 */
} rfalIsoDepControlMsgSParam;

/*! Enumeration of the possible control message types */
typedef enum
{    
    ISODEP_R_ACK,                    /*!< R-ACK  Acknowledge            */
    ISODEP_R_NAK,                    /*!< R-NACK Negative acknowledge   */
    ISODEP_S_WTX,                    /*!< S-WTX  Waiting Time Extension */
    ISODEP_S_DSL                     /*!< S-DSL  Deselect               */
} rfalIsoDepControlMsg;

/*! Enumeration of the IsoDep roles */
typedef enum
{
    ISODEP_ROLE_PCD,                /*!< Perform as Reader/PCD          */
    ISODEP_ROLE_PICC                /*!< Perform as Card/PICC           */
} rfalIsoDepRole;

/*! ISO-DEP layer states */
typedef enum
{
    ISODEP_ST_IDLE,                 /*!< Idle State                     */
    ISODEP_ST_PCD_TX,               /*!< PCD Transmission State         */
    ISODEP_ST_PCD_RX,               /*!< PCD Reception State            */
    ISODEP_ST_PCD_WAIT_DSL,         /*!< PCD Wait for DSL response      */
        
    ISODEP_ST_PICC_ACT_ATS,         /*!< PICC has replied to RATS (ATS) */
    ISODEP_ST_PICC_ACT_ATTRIB,      /*!< PICC has replied to ATTRIB     */
    ISODEP_ST_PICC_RX,              /*!< PICC REception State           */
    ISODEP_ST_PICC_SWTX,            /*!< PICC Waiting Time eXtension    */
    ISODEP_ST_PICC_TX,              /*!< PICC Transmission State        */
} rfalIsoDepState;




/*! Holds all ISO-DEP data(counters, buffers, ID, timeouts, frame size)         */
typedef struct{
  rfalIsoDepState state;         /*!< ISO-DEP module state                      */
  rfalIsoDepRole  role;          /*!< Current ISO-DEP role                      */
  
  uint8_t         blockNumber;   /*!< Current block number                      */
  uint8_t         did;           /*!< Current DID                               */
  uint8_t         nad;           /*!< Current DID                               */
  uint8_t         cntIRetrys;    /*!< I-Block retry counter                     */
  uint8_t         cntRRetrys;    /*!< R-Block retry counter                     */
  uint8_t         cntSRetrys;    /*!< S-Block retry counter                     */
  uint32_t        fwt;           /*!< Current FWT (Frame Waiting Time)          */
  uint32_t        dFwt;          /*!< Current delta FWT                         */
  uint16_t        fsx;           /*!< Current FSx FSC or FSD (max Frame size)   */
  bool            isTxChaining;  /*!< Flag for chaining on Tx                   */
  bool            isRxChaining;  /*!< Flag for chaining on Rx                   */
  uint8_t*        txBuf;         /*!< Tx buffer pointer                         */
  uint8_t*        rxBuf;         /*!< Rx buffer pointer                         */
  uint16_t        txBufLen;      /*!< Tx buffer length                          */
  uint16_t        rxBufLen;      /*!< Rx buffer length                          */
  uint8_t         txBufInfPos;   /*!< Start of payload in txBuf                 */
  uint8_t         rxBufInfPos;   /*!< Start of payload in rxBuf                 */
  
  
  uint16_t        ourFsx;        /*!< Our current FSx FSC or FSD (Frame size)   */
  uint8_t         lastPCB;       /*!< Last PCB sent                             */
  uint8_t         lastWTXM;      /*!< Last WTXM sent                            */
  uint8_t         atsTA;         /*!< TA on ATS                                 */
  uint8_t         hdrLen;        /*!< Current ISO-DEP length                    */
  rfalBitRate     txBR;          /*!< Current Tx Bit Rate                       */
  rfalBitRate     rxBR;          /*!< Current Rx Bit Rate                       */
  uint16_t        *rxLen;        /*!< Output parameter ptr to Rx length         */
  bool            *rxChaining;   /*!< Output parameter ptr to Rx chaining flag  */  
  uint32_t        WTXTimer;      /*!< Timer used for WTX                        */
  bool            lastDID00;     /*!< Last PCD block had DID flag (for DID = 0) */
  
  bool            isTxPending;   /*!< Flag pending Block while waiting WTX Ack  */
  bool            isWait4WTX;    /*!< Flag for waiting WTX Ack                  */
  
  uint32_t        SFGTTimer;     /*!< Timer used for SFGT                       */
  
  uint8_t         maxRetriesI;   /*!< Number of retries for a I-Block           */
  uint8_t         maxRetriesS;   /*!< Number of retries for a S-Block           */
  uint8_t         maxRetriesR;   /*!< Number of retries for a R-Block           */
  uint8_t         maxRetriesRATS;/*!< Number of retries for RATS                */
  
  rfalComplianceMode compMode;   /*!< Compliance mode                           */
  
  uint8_t         ctrlRxBuf[ISODEP_CONTROLMSG_BUF_LEN];  /*!< Control msg buf   */
  uint16_t        ctrlRxLen;  /*!< Control msg rcvd len (used only for DSL)     */
  
  
  rfalIsoDepListenActvParam actvParam;  /*!< Listen Activation context          */
  
  rfalIsoDepApduTxRxParam APDUParam;        /*!< APDU TxRx params               */
  uint16_t                APDUTxPos;        /*!< APDU Tx position               */
  uint16_t                APDURxPos;        /*!< APDU Rx position               */
  bool                    isAPDURxChaining; /*!< APDU Transceive chaining flag  */
  
}rfalIsoDep;
/* CLASS */
class rfal_rfst25r3911b : public rfal_rf {
   public:
    rfal_rfst25r3911b(SPIClass *spi,int cs_pin, int int_pin, uint32_t spi_speed = 5000000);
    ReturnCode rfalInitialize(); 
    ReturnCode rfalCalibrate();
    ReturnCode rfalAdjustRegulators( uint16_t* result_mV);
    void st25r3911CalibrateAntenna(uint8_t* result);
    void rfalAnalogConfigInitialize();
    void CheckInterrupts(); 
    void rfalWorker(); 
    ReturnCode rfalIsoDepDeselect();
    ReturnCode rfalTransceiveBlockingRx( void );  
    ReturnCode rfalTransceiveBlockingTx( uint8_t* txBuf, uint16_t txBufLen, uint8_t* rxBuf, uint16_t rxBufLen, uint16_t* actLen, uint32_t flags, uint32_t fwt );      
    ReturnCode rfalGetTransceiveStatus();    
    ReturnCode rfalStartTransceive( const rfalTransceiveContext *ctx );
    ReturnCode rfalIsoDepStartTransceive( rfalIsoDepTxRxParam param );
    ReturnCode rfalIsoDepGetTransceiveStatus( void );
    
    
  private:
     /* FUNCTION */
    ReturnCode rfalRunTransceiveWorker();
    bool rfalIsTransceiveInTx();
    bool rfalIsTransceiveInRx();
    void rfalTransceiveTx();
    void rfalTransceiveRx();
    bool rfalIsGTExpired();
    void rfalErrorHandling();
    void rfalFIFOStatusUpdate();
    uint8_t rfalFIFOGetNumIncompleteBits( void );
    bool rfalFIFOStatusIsMissingPar( void );
    bool rfalFIFOStatusIsIncompleteByte( void );
    uint8_t rfalFIFOStatusGetNumBytes( void );
    ReturnCode rfalRunListenModeWorker( void );
    ReturnCode rfalListenSetState( rfalLmState newSt );
    void rfalRunWakeUpModeWorker( void );
    ReturnCode rfalTransceiveRunBlockingTx( void );
    
    
    /* INTERRUPT */
    void st25r3911Isr();
    void st25r3911InitInterrupts();
    void st25r3911CheckForReceivedInterrupts();
    void st25r3911DisableInterrupts(uint32_t mask);
    void st25r3911ModifyInterrupts(uint32_t clr_mask, uint32_t set_mask);
    void st25r3911ClearInterrupts();
    uint32_t st25r3911GetInterrupt(uint32_t mask);
    void st25r3911EnableInterrupts(uint32_t mask);
    uint32_t st25r3911WaitForInterruptsTimed(uint32_t mask, uint16_t tmo);
    void rfalPrepareTransceive();
    
    /* COM */
    void st25r3911WriteRegister(uint8_t reg, uint8_t value);
    void st25r3911WriteMultipleRegisters(uint8_t reg, const uint8_t* values, uint8_t len);
    void st25r3911ReadRegister(uint8_t reg, uint8_t* value);
    void st25r3911ReadMultipleRegisters(uint8_t reg, uint8_t* values, uint8_t length);
    void st25r3911ModifyRegister(uint8_t reg, uint8_t clr_mask, uint8_t set_mask);
    void st25r3911Initialize();
    void st25r3911ExecuteCommand(uint8_t cmd);
    void st25r3911ExecuteCommands(const uint8_t *cmds, uint8_t length);
    void st25r3911OscOn();
    bool st25r3911CheckReg( uint8_t reg, uint8_t mask, uint8_t value );
    uint16_t st25r3911MeasureVoltage(uint8_t mpsv);
    uint8_t st25r3911MeasurePowerSupply( uint8_t mpsv );
    void st25r3911ChangeRegisterBits(uint8_t reg, uint8_t valueMask, uint8_t value);
    ReturnCode st25r3911ExecuteCommandAndGetResult(uint8_t cmd, uint8_t resreg, uint8_t sleeptime, uint8_t* result);
    void st25r3911TxRxOff();
    void st25r3911ClrRegisterBits( uint8_t reg, uint8_t clr_mask );
    bool st25r3911CheckChipID( uint8_t *rev );
    void st25r3911WriteTestRegister(uint8_t reg, uint8_t value);
    void st25r3911SetRegisterBits( uint8_t reg, uint8_t set_mask );      
    void st25r3911ReadTestRegister(uint8_t reg, uint8_t* value);
    void st25r3911ChangeTestRegisterBits( uint8_t reg, uint8_t valueMask, uint8_t value );
    void rfalFIFOStatusClear( void );
     ReturnCode rfalChipChangeRegBits( uint16_t reg, uint8_t valueMask, uint8_t value );
     ReturnCode rfalChipChangeTestRegBits( uint16_t reg, uint8_t valueMask, uint8_t value );
     void st25r3911SetNumTxBits( uint32_t nBits );
    void st25r3911WriteFifo(const uint8_t* values, uint8_t length);    
    void st25r3911ReadFifo(uint8_t* buf, uint8_t length);
     void st25r3911StartGPTimer_8fcs(uint16_t gpt_8fcs, uint8_t trigger_source);
     void rfalCleanupTransceive(  );
     ReturnCode st25r3911SetNoResponseTime_64fcs(uint32_t nrt_64fcs);
     
     /* CRC */
     uint16_t rfalCrcCalculateCcitt(uint16_t preloadValue, const uint8_t* buf, uint16_t length);
     uint16_t rfalCrcUpdateCcitt(uint16_t crcSeed, uint8_t dataByte);
     
    /* ANALOG CONFIG */
    ReturnCode rfalSetAnalogConfig( rfalAnalogConfigId configId );
    rfalAnalogConfigNum rfalAnalogConfigSearch( rfalAnalogConfigId configId, uint16_t *configOffset );

    /* RFAL_ISO */
    ReturnCode iso15693VCDCode(uint8_t* buffer, uint16_t length, bool sendCrc, bool sendFlags, bool picopassMode,
                   uint16_t *subbit_total_length, uint16_t *offset,
                   uint8_t* outbuf, uint16_t outBufSize, uint16_t* actOutBufSize);
    ReturnCode iso15693VICCDecode(const uint8_t *inBuf,
                      uint16_t inBufLen,
                      uint8_t* outBuf,
                      uint16_t outBufLen,
                      uint16_t* outBufPos,
                      uint16_t* bitsBeforeCol,
                      uint16_t ignoreBits,
                      bool picopassMode );

     /* RFAL_ISO_DEP */
     ReturnCode isoDepDataExchangePCD( uint16_t *outActRxLen, bool *outIsChaining );     
     ReturnCode isoDepTx( uint8_t pcb, const uint8_t* txBuf, uint8_t *infBuf, uint16_t infLen, uint32_t fwt );
     ReturnCode isoDepHandleControlMsg( rfalIsoDepControlMsg controlMsg, uint8_t param );
     
    /* VARIABLE */
    /* Connection */
    SPIClass *dev_spi;
    int cs_pin;
    int int_pin;    
    uint32_t spi_speed;

    /* Interrupt*/
    t_st25r3911Interrupt st25r3911interrupt;

    /* Analog Config */
    rfalAnalogConfigMgmt   gRfalAnalogConfigMgmt;  /*!< Analog Configuration LUT management */

    /* Structure RFAL */
    rfal gRFAL;


    /* Structure ISO*/
    iso15693PhyConfig_t iso15693PhyConfig;
};

#endif
