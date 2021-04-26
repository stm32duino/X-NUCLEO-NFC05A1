#ifndef RFAL_ANALOG_CONFIG_H
#define RFAL_ANALOG_CONFIG_H


#define RFAL_ANALOG_CONFIG_LUT_SIZE                 (87U)     /*!< Maximum number of Configuration IDs in the Loop Up Table     */
#define RFAL_ANALOG_CONFIG_LUT_NOT_FOUND            (0xFFU)   /*!< Index value indicating no Configuration IDs found            */

#define RFAL_ANALOG_CONFIG_TBL_SIZE                 (1024U)   /*!< Maximum number of Register-Mask-Value in the Setting List    */


#define RFAL_ANALOG_CONFIG_POLL_LISTEN_MODE_MASK    (0x8000U) /*!< Mask bit of Poll Mode in Analog Configuration ID             */
#define RFAL_ANALOG_CONFIG_TECH_MASK                (0x7F00U) /*!< Mask bits for Technology in Analog Configuration ID          */
#define RFAL_ANALOG_CONFIG_BITRATE_MASK             (0x00F0U) /*!< Mask bits for Bit rate in Analog Configuration ID            */
#define RFAL_ANALOG_CONFIG_DIRECTION_MASK           (0x0003U) /*!< Mask bits for Direction in Analog Configuration ID           */
#define RFAL_ANALOG_CONFIG_CHIP_SPECIFIC_MASK       (0x00FFU) /*!< Mask bits for Chip Specific Technology                       */

#define RFAL_ANALOG_CONFIG_POLL_LISTEN_MODE_SHIFT   (15U)     /*!< Shift value of Poll Mode in Analog Configuration ID          */
#define RFAL_ANALOG_CONFIG_TECH_SHIFT               (8U)      /*!< Shift value for Technology in Analog Configuration ID        */
#define RFAL_ANALOG_CONFIG_BITRATE_SHIFT            (4U)      /*!< Shift value for Technology in Analog Configuration ID        */
#define RFAL_ANALOG_CONFIG_DIRECTION_SHIFT          (0U)      /*!< Shift value for Direction in Analog Configuration ID         */

#define RFAL_ANALOG_CONFIG_POLL                     (0x0000U) /*!< Poll Mode bit setting in Analog Configuration ID             */
#define RFAL_ANALOG_CONFIG_LISTEN                   (0x8000U) /*!< Listen Mode bit setting in Analog Configuration ID           */

#define RFAL_ANALOG_CONFIG_TECH_CHIP                (0x0000U) /*!< Chip-Specific bit setting in Analog Configuration ID         */
#define RFAL_ANALOG_CONFIG_TECH_NFCA                (0x0100U) /*!< NFC-A Technology bits setting in Analog Configuration ID     */
#define RFAL_ANALOG_CONFIG_TECH_NFCB                (0x0200U) /*!< NFC-B Technology bits setting in Analog Configuration ID     */
#define RFAL_ANALOG_CONFIG_TECH_NFCF                (0x0400U) /*!< NFC-F Technology bits setting in Analog Configuration ID     */
#define RFAL_ANALOG_CONFIG_TECH_AP2P                (0x0800U) /*!< AP2P Technology bits setting in Analog Configuration ID      */
#define RFAL_ANALOG_CONFIG_TECH_NFCV                (0x1000U) /*!< NFC-V Technology bits setting in Analog Configuration ID     */
#define RFAL_ANALOG_CONFIG_TECH_RFU                 (0x2000U) /*!< RFU for Technology bits */

#define RFAL_ANALOG_CONFIG_BITRATE_COMMON           (0x0000U) /*!< Common settings for all bit rates in Analog Configuration ID */
#define RFAL_ANALOG_CONFIG_BITRATE_106              (0x0010U) /*!< 106kbits/s settings in Analog Configuration ID               */
#define RFAL_ANALOG_CONFIG_BITRATE_212              (0x0020U) /*!< 212kbits/s settings in Analog Configuration ID               */
#define RFAL_ANALOG_CONFIG_BITRATE_424              (0x0030U) /*!< 424kbits/s settings in Analog Configuration ID               */
#define RFAL_ANALOG_CONFIG_BITRATE_848              (0x0040U) /*!< 848kbits/s settings in Analog Configuration ID               */
#define RFAL_ANALOG_CONFIG_BITRATE_1695             (0x0050U) /*!< 1695kbits/s settings in Analog Configuration ID              */
#define RFAL_ANALOG_CONFIG_BITRATE_3390             (0x0060U) /*!< 3390kbits/s settings in Analog Configuration ID              */
#define RFAL_ANALOG_CONFIG_BITRATE_6780             (0x0070U) /*!< 6780kbits/s settings in Analog Configuration ID              */
#define RFAL_ANALOG_CONFIG_BITRATE_1OF4             (0x00C0U) /*!< 1 out of 4 for NFC-V setting in Analog Configuration ID      */
#define RFAL_ANALOG_CONFIG_BITRATE_1OF256           (0x00D0U) /*!< 1 out of 256 for NFC-V setting in Analog Configuration ID    */

#define RFAL_ANALOG_CONFIG_NO_DIRECTION             (0x0000U) /*!< No direction setting in Analog Conf ID (Chip Specific only)  */
#define RFAL_ANALOG_CONFIG_TX                       (0x0001U) /*!< Transmission bit setting in Analog Configuration ID          */
#define RFAL_ANALOG_CONFIG_RX                       (0x0002U) /*!< Reception bit setting in Analog Configuration ID             */
#define RFAL_ANALOG_CONFIG_ANTICOL                  (0x0003U)  /*!< Anticollision setting in Analog Configuration ID             */

#define RFAL_ANALOG_CONFIG_CHIP_INIT                (0x0000U)  /*!< Chip-Specific event: Startup;Reset;Initialize                */
#define RFAL_ANALOG_CONFIG_CHIP_DEINIT              (0x0001U)  /*!< Chip-Specific event: Deinitialize                            */
#define RFAL_ANALOG_CONFIG_CHIP_FIELD_ON            (0x0002U)  /*!< Chip-Specific event: Field On                                */
#define RFAL_ANALOG_CONFIG_CHIP_FIELD_OFF           (0x0003U)  /*!< Chip-Specific event: Field Off                               */
#define RFAL_ANALOG_CONFIG_CHIP_WAKEUP_ON           (0x0004U)  /*!< Chip-Specific event: Wake-up On                              */
#define RFAL_ANALOG_CONFIG_CHIP_WAKEUP_OFF          (0x0005U)  /*!< Chip-Specific event: Wake-up Off                             */
#define RFAL_ANALOG_CONFIG_CHIP_LISTEN_ON           (0x0006U)  /*!< Chip-Specific event: Listen On                               */
#define RFAL_ANALOG_CONFIG_CHIP_LISTEN_OFF          (0x0007U)  /*!< Chip-Specific event: Listen Off                              */
#define RFAL_ANALOG_CONFIG_CHIP_POLL_COMMON         (0x0008U)  /*!< Chip-Specific event: Poll common                             */
#define RFAL_ANALOG_CONFIG_CHIP_LISTEN_COMMON       (0x0009U)  /*!< Chip-Specific event: Listen common                           */

#define RFAL_ANALOG_CONFIG_UPDATE_LAST              (0x00U)   /*!< Value indicating Last configuration set during update        */
#define RFAL_ANALOG_CONFIG_UPDATE_MORE              (0x01U)   /*!< Value indicating More configuration set coming during update */

#define RFAL_ANALOG_CONFIG_ID_GET_POLL_LISTEN(id)   (RFAL_ANALOG_CONFIG_POLL_LISTEN_MODE_MASK & (id)) /*!< Check if id indicates Listen mode   */

#define RFAL_ANALOG_CONFIG_ID_GET_TECH(id)          (RFAL_ANALOG_CONFIG_TECH_MASK & (id))      /*!< Get the technology of Configuration ID     */
#define RFAL_ANALOG_CONFIG_ID_IS_CHIP(id)           (RFAL_ANALOG_CONFIG_TECH_MASK & (id))      /*!< Check if ID indicates Chip-specific        */
#define RFAL_ANALOG_CONFIG_ID_IS_NFCA(id)           (RFAL_ANALOG_CONFIG_TECH_NFCA & (id))      /*!< Check if ID indicates NFC-A                */
#define RFAL_ANALOG_CONFIG_ID_IS_NFCB(id)           (RFAL_ANALOG_CONFIG_TECH_NFCB & (id))      /*!< Check if ID indicates NFC-B                */
#define RFAL_ANALOG_CONFIG_ID_IS_NFCF(id)           (RFAL_ANALOG_CONFIG_TECH_NFCF & (id))      /*!< Check if ID indicates NFC-F                */
#define RFAL_ANALOG_CONFIG_ID_IS_AP2P(id)           (RFAL_ANALOG_CONFIG_TECH_AP2P & (id))      /*!< Check if ID indicates AP2P                 */
#define RFAL_ANALOG_CONFIG_ID_IS_NFCV(id)           (RFAL_ANALOG_CONFIG_TECH_NFCV & (id))      /*!< Check if ID indicates NFC-V                */

#define RFAL_ANALOG_CONFIG_ID_GET_BITRATE(id)       (RFAL_ANALOG_CONFIG_BITRATE_MASK & (id))   /*!< Get Bitrate of Configuration ID            */
#define RFAL_ANALOG_CONFIG_ID_IS_COMMON(id)         (RFAL_ANALOG_CONFIG_BITRATE_MASK & (id))   /*!< Check if ID indicates common bitrate       */
#define RFAL_ANALOG_CONFIG_ID_IS_106(id)            (RFAL_ANALOG_CONFIG_BITRATE_106 & (id))    /*!< Check if ID indicates 106kbits/s           */
#define RFAL_ANALOG_CONFIG_ID_IS_212(id)            (RFAL_ANALOG_CONFIG_BITRATE_212 & (id))    /*!< Check if ID indicates 212kbits/s           */
#define RFAL_ANALOG_CONFIG_ID_IS_424(id)            (RFAL_ANALOG_CONFIG_BITRATE_424 & (id))    /*!< Check if ID indicates 424kbits/s           */
#define RFAL_ANALOG_CONFIG_ID_IS_848(id)            (RFAL_ANALOG_CONFIG_BITRATE_848 & (id))    /*!< Check if ID indicates 848kbits/s           */
#define RFAL_ANALOG_CONFIG_ID_IS_1695(id)           (RFAL_ANALOG_CONFIG_BITRATE_1695 & (id))   /*!< Check if ID indicates 1695kbits/s          */
#define RFAL_ANALOG_CONFIG_ID_IS_3390(id)           (RFAL_ANALOG_CONFIG_BITRATE_3390 & (id))   /*!< Check if ID indicates 3390kbits/s          */
#define RFAL_ANALOG_CONFIG_ID_IS_6780(id)           (RFAL_ANALOG_CONFIG_BITRATE_6780 & (id))   /*!< Check if ID indicates 6780kbits/s          */
#define RFAL_ANALOG_CONFIG_ID_IS_1OF4(id)           (RFAL_ANALOG_CONFIG_BITRATE_1OF4 & (id))   /*!< Check if ID indicates 1 out of 4 bitrate   */
#define RFAL_ANALOG_CONFIG_ID_IS_1OF256(id)         (RFAL_ANALOG_CONFIG_BITRATE_1OF256 & (id)) /*!< Check if ID indicates 1 out of 256 bitrate */

#define RFAL_ANALOG_CONFIG_ID_GET_DIRECTION(id)     (RFAL_ANALOG_CONFIG_DIRECTION_MASK & (id)) /*!< Get Direction of Configuration ID          */
#define RFAL_ANALOG_CONFIG_ID_IS_TX(id)             (RFAL_ANALOG_CONFIG_TX & (id))             /*!< Check if id indicates TX                   */
#define RFAL_ANALOG_CONFIG_ID_IS_RX(id)             (RFAL_ANALOG_CONFIG_RX & (id))             /*!< Check if id indicates RX                   */

#define RFAL_ANALOG_CONFIG_CONFIG_NUM(x)            (sizeof(x)/sizeof((x)[0]))                 /*!< Get Analog Config number                   */

/*! Set Analog Config ID value by: Mode, Technology, Bitrate and Direction      */
#define RFAL_ANALOG_CONFIG_ID_SET(mode, tech, br, direction)    \
    (  RFAL_ANALOG_CONFIG_ID_GET_POLL_LISTEN(mode) \
     | RFAL_ANALOG_CONFIG_ID_GET_TECH(tech) \
     | RFAL_ANALOG_CONFIG_ID_GET_BITRATE(br) \
     | RFAL_ANALOG_CONFIG_ID_GET_DIRECTION(direction) \
    )


/* TYPES */

typedef uint8_t  rfalAnalogConfigMode;       /*!< Polling or Listening Mode of Configuration                    */
typedef uint8_t  rfalAnalogConfigTech;       /*!< Technology of Configuration                                   */
typedef uint8_t  rfalAnalogConfigBitrate;    /*!< Bitrate of Configuration                                      */
typedef uint8_t  rfalAnalogConfigDirection;  /*!< Transmit/Receive direction of Configuration                   */

typedef uint8_t  rfalAnalogConfigRegAddr[2]; /*!< Register Address to ST Chip                                   */
typedef uint8_t  rfalAnalogConfigRegMask;    /*!< Register Mask Value                                           */
typedef uint8_t  rfalAnalogConfigRegVal;     /*!< Register Value                                                */

typedef uint16_t rfalAnalogConfigId;         /*!< Analog Configuration ID                                       */
typedef uint16_t rfalAnalogConfigOffset;     /*!< Analog Configuration offset address in the table              */
typedef uint8_t  rfalAnalogConfigNum;        /*!< Number of Analog settings for the respective Configuration ID */


/*! Struct that contain the Register-Mask-Value set. Make sure that the whole structure size is even and unaligned! */
typedef struct {
    rfalAnalogConfigRegAddr addr;  /*!< Register Address    */
    rfalAnalogConfigRegMask mask;  /*!< Register Mask Value */
    rfalAnalogConfigRegVal  val;   /*!< Register Value      */
} rfalAnalogConfigRegAddrMaskVal;


/*! Struct that represents the Analog Configs */
typedef struct {
    uint8_t                        id[sizeof(rfalAnalogConfigId)]; /*!< Configuration ID                   */
    rfalAnalogConfigNum            num;                            /*!< Number of Config Sets to follow    */
    rfalAnalogConfigRegAddrMaskVal regSet[];                       /*!< Register-Mask-Value sets           */ /*  PRQA S 1060 # MISRA 18.7 - Flexible Array Members are the only meaningful way of denoting a variable length input buffer which follows a fixed header structure. */
} rfalAnalogConfig;

#define RFAL_TEST_REG         0x0080U      /*!< Test Register indicator  */    

/*! Struct for Analog Config Look Up Table Update */
typedef struct {
    const uint8_t *currentAnalogConfigTbl; /*!< Reference to start of current Analog Configuration      */
    uint16_t configTblSize;          /*!< Total size of Analog Configuration                      */
    bool    ready;                  /*!< Indicate if Look Up Table is complete and ready for use */
} rfalAnalogConfigMgmt;

/* TLB */
/*! Macro for Configuration Setting with only one register-mask-value set: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1] */
#define MODE_ENTRY_1_REG(MODE, R0, M0, V0)              \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU), 1, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0)
/*! Macro for Configuration Setting with only two register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1] */
#define MODE_ENTRY_2_REG(MODE, R0, M0, V0, R1, M1, V1)  \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU), 2, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0) \
                                 , (uint8_t)((R1) >> 8), (uint8_t)((R1) & 0xFFU), (uint8_t)(M1), (uint8_t)(V1)

/*! Macro for Configuration Setting with only three register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_3_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2)  \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU), 3, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0) \
                                 , (uint8_t)((R1) >> 8), (uint8_t)((R1) & 0xFFU), (uint8_t)(M1), (uint8_t)(V1) \
                                 , (uint8_t)((R2) >> 8), (uint8_t)((R2) & 0xFFU), (uint8_t)(M2), (uint8_t)(V2) \

/*! Macro for Configuration Setting with only four register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_4_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3)  \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU), 4, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0) \
                                 , (uint8_t)((R1) >> 8), (uint8_t)((R1) & 0xFFU), (uint8_t)(M1), (uint8_t)(V1) \
                                 , (uint8_t)((R2) >> 8), (uint8_t)((R2) & 0xFFU), (uint8_t)(M2), (uint8_t)(V2) \
                                 , (uint8_t)((R3) >> 8), (uint8_t)((R3) & 0xFFU), (uint8_t)(M3), (uint8_t)(V3) \

/*! Macro for Configuration Setting with only five register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_5_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4)  \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU), 5, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0) \
                                 , (uint8_t)((R1) >> 8), (uint8_t)((R1) & 0xFFU), (uint8_t)(M1), (uint8_t)(V1) \
                                 , (uint8_t)((R2) >> 8), (uint8_t)((R2) & 0xFFU), (uint8_t)(M2), (uint8_t)(V2) \
                                 , (uint8_t)((R3) >> 8), (uint8_t)((R3) & 0xFFU), (uint8_t)(M3), (uint8_t)(V3) \
                                 , (uint8_t)((R4) >> 8), (uint8_t)((R4) & 0xFFU), (uint8_t)(M4), (uint8_t)(V4) \

/*! Macro for Configuration Setting with only six register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_6_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5)  \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU), 6, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0) \
                                 , (uint8_t)((R1) >> 8), (uint8_t)((R1) & 0xFFU), (uint8_t)(M1), (uint8_t)(V1) \
                                 , (uint8_t)((R2) >> 8), (uint8_t)((R2) & 0xFFU), (uint8_t)(M2), (uint8_t)(V2) \
                                 , (uint8_t)((R3) >> 8), (uint8_t)((R3) & 0xFFU), (uint8_t)(M3), (uint8_t)(V3) \
                                 , (uint8_t)((R4) >> 8), (uint8_t)((R4) & 0xFFU), (uint8_t)(M4), (uint8_t)(V4) \
                                 , (uint8_t)((R5) >> 8), (uint8_t)((R5) & 0xFFU), (uint8_t)(M5), (uint8_t)(V5) \
                                 
/*! Macro for Configuration Setting with only seven register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_7_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6)  \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU), 7, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0) \
                                 , (uint8_t)((R1) >> 8), (uint8_t)((R1) & 0xFFU), (uint8_t)(M1), (uint8_t)(V1) \
                                 , (uint8_t)((R2) >> 8), (uint8_t)((R2) & 0xFFU), (uint8_t)(M2), (uint8_t)(V2) \
                                 , (uint8_t)((R3) >> 8), (uint8_t)((R3) & 0xFFU), (uint8_t)(M3), (uint8_t)(V3) \
                                 , (uint8_t)((R4) >> 8), (uint8_t)((R4) & 0xFFU), (uint8_t)(M4), (uint8_t)(V4) \
                                 , (uint8_t)((R5) >> 8), (uint8_t)((R5) & 0xFFU), (uint8_t)(M5), (uint8_t)(V5) \
                                 , (uint8_t)((R6) >> 8), (uint8_t)((R6) & 0xFFU), (uint8_t)(M6), (uint8_t)(V6) \

/*! Macro for Configuration Setting with only eight register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_8_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6, R7, M7, V7)  \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU), 8, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0) \
                                 , (uint8_t)((R1) >> 8), (uint8_t)((R1) & 0xFFU), (uint8_t)(M1), (uint8_t)(V1) \
                                 , (uint8_t)((R2) >> 8), (uint8_t)((R2) & 0xFFU), (uint8_t)(M2), (uint8_t)(V2) \
                                 , (uint8_t)((R3) >> 8), (uint8_t)((R3) & 0xFFU), (uint8_t)(M3), (uint8_t)(V3) \
                                 , (uint8_t)((R4) >> 8), (uint8_t)((R4) & 0xFFU), (uint8_t)(M4), (uint8_t)(V4) \
                                 , (uint8_t)((R5) >> 8), (uint8_t)((R5) & 0xFFU), (uint8_t)(M5), (uint8_t)(V5) \
                                 , (uint8_t)((R6) >> 8), (uint8_t)((R6) & 0xFFU), (uint8_t)(M6), (uint8_t)(V6) \
                                 , (uint8_t)((R7) >> 8), (uint8_t)((R7) & 0xFFU), (uint8_t)(M7), (uint8_t)(V7) \

/*! Macro for Configuration Setting with only nine register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_9_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6, R7, M7, V7, R8, M8, V8)  \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU), 9, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0) \
                                 , (uint8_t)((R1) >> 8), (uint8_t)((R1) & 0xFFU), (uint8_t)(M1), (uint8_t)(V1) \
                                 , (uint8_t)((R2) >> 8), (uint8_t)((R2) & 0xFFU), (uint8_t)(M2), (uint8_t)(V2) \
                                 , (uint8_t)((R3) >> 8), (uint8_t)((R3) & 0xFFU), (uint8_t)(M3), (uint8_t)(V3) \
                                 , (uint8_t)((R4) >> 8), (uint8_t)((R4) & 0xFFU), (uint8_t)(M4), (uint8_t)(V4) \
                                 , (uint8_t)((R5) >> 8), (uint8_t)((R5) & 0xFFU), (uint8_t)(M5), (uint8_t)(V5) \
                                 , (uint8_t)((R6) >> 8), (uint8_t)((R6) & 0xFFU), (uint8_t)(M6), (uint8_t)(V6) \
                                 , (uint8_t)((R7) >> 8), (uint8_t)((R7) & 0xFFU), (uint8_t)(M7), (uint8_t)(V7) \
                                 , (uint8_t)((R8) >> 8), (uint8_t)((R8) & 0xFFU), (uint8_t)(M8), (uint8_t)(V8) \

/*! Macro for Configuration Setting with only ten register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_10_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6, R7, M7, V7, R8, M8, V8, R9, M9, V9)  \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU),10, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0) \
                                 , (uint8_t)((R1) >> 8), (uint8_t)((R1) & 0xFFU), (uint8_t)(M1), (uint8_t)(V1) \
                                 , (uint8_t)((R2) >> 8), (uint8_t)((R2) & 0xFFU), (uint8_t)(M2), (uint8_t)(V2) \
                                 , (uint8_t)((R3) >> 8), (uint8_t)((R3) & 0xFFU), (uint8_t)(M3), (uint8_t)(V3) \
                                 , (uint8_t)((R4) >> 8), (uint8_t)((R4) & 0xFFU), (uint8_t)(M4), (uint8_t)(V4) \
                                 , (uint8_t)((R5) >> 8), (uint8_t)((R5) & 0xFFU), (uint8_t)(M5), (uint8_t)(V5) \
                                 , (uint8_t)((R6) >> 8), (uint8_t)((R6) & 0xFFU), (uint8_t)(M6), (uint8_t)(V6) \
                                 , (uint8_t)((R7) >> 8), (uint8_t)((R7) & 0xFFU), (uint8_t)(M7), (uint8_t)(V7) \
                                 , (uint8_t)((R8) >> 8), (uint8_t)((R8) & 0xFFU), (uint8_t)(M8), (uint8_t)(V8) \
                                 , (uint8_t)((R9) >> 8), (uint8_t)((R9) & 0xFFU), (uint8_t)(M9), (uint8_t)(V9) \

/*! Macro for Configuration Setting with eleven register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_11_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6, R7, M7, V7, R8, M8, V8, R9, M9, V9, R10, M10, V10)  \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU),11, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0) \
                                 , (uint8_t)((R1) >> 8), (uint8_t)((R1) & 0xFFU), (uint8_t)(M1), (uint8_t)(V1) \
                                 , (uint8_t)((R2) >> 8), (uint8_t)((R2) & 0xFFU), (uint8_t)(M2), (uint8_t)(V2) \
                                 , (uint8_t)((R3) >> 8), (uint8_t)((R3) & 0xFFU), (uint8_t)(M3), (uint8_t)(V3) \
                                 , (uint8_t)((R4) >> 8), (uint8_t)((R4) & 0xFFU), (uint8_t)(M4), (uint8_t)(V4) \
                                 , (uint8_t)((R5) >> 8), (uint8_t)((R5) & 0xFFU), (uint8_t)(M5), (uint8_t)(V5) \
                                 , (uint8_t)((R6) >> 8), (uint8_t)((R6) & 0xFFU), (uint8_t)(M6), (uint8_t)(V6) \
                                 , (uint8_t)((R7) >> 8), (uint8_t)((R7) & 0xFFU), (uint8_t)(M7), (uint8_t)(V7) \
                                 , (uint8_t)((R8) >> 8), (uint8_t)((R8) & 0xFFU), (uint8_t)(M8), (uint8_t)(V8) \
                                 , (uint8_t)((R9) >> 8), (uint8_t)((R9) & 0xFFU), (uint8_t)(M9), (uint8_t)(V9) \
                                 , (uint8_t)((R10) >> 8), (uint8_t)((R10) & 0xFFU), (uint8_t)(M10), (uint8_t)(V10) \

/*! Macro for Configuration Setting with twelve register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_12_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6, R7, M7, V7, R8, M8, V8, R9, M9, V9, R10, M10, V10, R11, M11, V11)  \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU),12, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0) \
                                 , (uint8_t)((R1) >> 8), (uint8_t)((R1) & 0xFFU), (uint8_t)(M1), (uint8_t)(V1) \
                                 , (uint8_t)((R2) >> 8), (uint8_t)((R2) & 0xFFU), (uint8_t)(M2), (uint8_t)(V2) \
                                 , (uint8_t)((R3) >> 8), (uint8_t)((R3) & 0xFFU), (uint8_t)(M3), (uint8_t)(V3) \
                                 , (uint8_t)((R4) >> 8), (uint8_t)((R4) & 0xFFU), (uint8_t)(M4), (uint8_t)(V4) \
                                 , (uint8_t)((R5) >> 8), (uint8_t)((R5) & 0xFFU), (uint8_t)(M5), (uint8_t)(V5) \
                                 , (uint8_t)((R6) >> 8), (uint8_t)((R6) & 0xFFU), (uint8_t)(M6), (uint8_t)(V6) \
                                 , (uint8_t)((R7) >> 8), (uint8_t)((R7) & 0xFFU), (uint8_t)(M7), (uint8_t)(V7) \
                                 , (uint8_t)((R8) >> 8), (uint8_t)((R8) & 0xFFU), (uint8_t)(M8), (uint8_t)(V8) \
                                 , (uint8_t)((R9) >> 8), (uint8_t)((R9) & 0xFFU), (uint8_t)(M9), (uint8_t)(V9) \
                                 , (uint8_t)((R10) >> 8), (uint8_t)((R10) & 0xFFU), (uint8_t)(M10), (uint8_t)(V10) \
                                 , (uint8_t)((R11) >> 8), (uint8_t)((R11) & 0xFFU), (uint8_t)(M11), (uint8_t)(V11) \

/*! Macro for Configuration Setting with thirteen register-mask-value sets: 
 *  - Configuration ID[2], Number of Register sets to follow[1], Register[2], Mask[1], Value[1], Register[2], Mask[1], Value[1], Register[2]... */
#define MODE_ENTRY_13_REG(MODE, R0, M0, V0, R1, M1, V1, R2, M2, V2, R3, M3, V3, R4, M4, V4, R5, M5, V5, R6, M6, V6, R7, M7, V7, R8, M8, V8, R9, M9, V9, R10, M10, V10, R11, M11, V11, R12, M12, V12)  \
    (uint8_t)((MODE) >> 8), (uint8_t)((MODE) & 0xFFU),13, (uint8_t)((R0) >> 8), (uint8_t)((R0) & 0xFFU), (uint8_t)(M0), (uint8_t)(V0) \
                                 , (uint8_t)((R1) >> 8), (uint8_t)((R1) & 0xFFU), (uint8_t)(M1), (uint8_t)(V1) \
                                 , (uint8_t)((R2) >> 8), (uint8_t)((R2) & 0xFFU), (uint8_t)(M2), (uint8_t)(V2) \
                                 , (uint8_t)((R3) >> 8), (uint8_t)((R3) & 0xFFU), (uint8_t)(M3), (uint8_t)(V3) \
                                 , (uint8_t)((R4) >> 8), (uint8_t)((R4) & 0xFFU), (uint8_t)(M4), (uint8_t)(V4) \
                                 , (uint8_t)((R5) >> 8), (uint8_t)((R5) & 0xFFU), (uint8_t)(M5), (uint8_t)(V5) \
                                 , (uint8_t)((R6) >> 8), (uint8_t)((R6) & 0xFFU), (uint8_t)(M6), (uint8_t)(V6) \
                                 , (uint8_t)((R7) >> 8), (uint8_t)((R7) & 0xFFU), (uint8_t)(M7), (uint8_t)(V7) \
                                 , (uint8_t)((R8) >> 8), (uint8_t)((R8) & 0xFFU), (uint8_t)(M8), (uint8_t)(V8) \
                                 , (uint8_t)((R9) >> 8), (uint8_t)((R9) & 0xFFU), (uint8_t)(M9), (uint8_t)(V9) \
                                 , (uint8_t)((R10) >> 8), (uint8_t)((R10) & 0xFFU), (uint8_t)(M10), (uint8_t)(V10) \
                                 , (uint8_t)((R11) >> 8), (uint8_t)((R11) & 0xFFU), (uint8_t)(M11), (uint8_t)(V11) \
                                 , (uint8_t)((R12) >> 8), (uint8_t)((R12) & 0xFFU), (uint8_t)(M12), (uint8_t)(V12) \
 
/* Setting for approximately 14%: */
#define AM_MOD_DRIVER_LEVEL_DEFAULT 0xb9 

/*  PRQA S 3406 1 # MISRA 8.6 - Externally generated table included by the library */   /*  PRQA S 1514 1 # MISRA 8.9 - Externally generated table included by the library */
 const uint8_t rfalAnalogConfigDefaultSettings[] = {
      //****** Default Analog Configuration for Chip-Specific Reset. ******/
      MODE_ENTRY_10_REG( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_INIT)
                      , ST25R3911_REG_OP_CONTROL,             0x30, 0x10 /* default to AM */
                      , ST25R3911_REG_IO_CONF1,               0x06, 0x06 /* MCUCLK: HF clk off */
                      , ST25R3911_REG_IO_CONF1,               (ST25R3911_REG_IO_CONF1_mask_out_cl | ST25R3911_REG_IO_CONF1_lf_clk_off), 0x07 /* MCUCLK: LF clk off */
                      , ST25R3911_REG_IO_CONF2,               0x18, 0x18 /* pull downs */
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_pm, 0x1U<<ST25R3911_REG_RX_CONF4_shift_rg2_pm /* increase digitizer window for PM */
                      , ST25R3911_REG_ANT_CAL_TARGET,         0xFF, 0x80 /* 90degrees */
                      , ST25R3911_REG_ANT_CAL_CONTROL,        0xF8, 0x00 /* trim value from calibrate antenna */
                      , ST25R3911_REG_AM_MOD_DEPTH_CONTROL,   ST25R3911_REG_AM_MOD_DEPTH_CONTROL_am_s, ST25R3911_REG_AM_MOD_DEPTH_CONTROL_am_s /* AM modulated level is defined by RFO AM Modulated Level Def Reg, fixed setting, no automatic adjustment */
                      , ST25R3911_REG_FIELD_THRESHOLD,        ST25R3911_REG_FIELD_THRESHOLD_mask_trg, ST25R3911_REG_FIELD_THRESHOLD_trg_75mV
                      , ST25R3911_REG_FIELD_THRESHOLD,        ST25R3911_REG_FIELD_THRESHOLD_mask_rfe, ST25R3911_REG_FIELD_THRESHOLD_rfe_75mV
                      )
                      
      //****** Default Analog Configuration for Poll NFC-A Tx. ******/
    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_RFO_AM_ON_LEVEL, 0xff, 0xf0 /* Used for 848 TX: very high AM to keep wave shapes */
                      )
                      
    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am,        0x00        /* OOK */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am,        0x00        /* OOK */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am,        0x00        /* OOK */
                      )

    , MODE_ENTRY_2_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_848 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,  ST25R3911_REG_AUX_tr_am,  ST25R3911_REG_AUX_tr_am       /* AM! */
                      , ST25R3911_REG_RFO_AM_ON_LEVEL, 0xff, 0xf0 /* Used for 848 TX: very high AM to keep wave shapes */
                      )

      //****** Default Analog Configuration for Poll NFC-A Rx. ******/
    , MODE_ENTRY_3_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF3, 0xff, 0x18
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x2U<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_rx_tol, 0x00                                               /* rx_tol Off */
                      )
                                                                                  
    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1, 0x7f, 0x00
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1, 0x7f, 0x04
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1, 0x7f, 0x22
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCA | RFAL_ANALOG_CONFIG_BITRATE_848 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1, 0x7f, 0x22
                      )

      //****** Default Analog Configuration for Poll NFC-B Tx. ******/
    , MODE_ENTRY_2_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, ST25R3911_REG_AUX_tr_am   /* AM */
                      , ST25R3911_REG_RFO_AM_ON_LEVEL,        0xff, AM_MOD_DRIVER_LEVEL_DEFAULT                  /* Fixed driver for AM level: ~14% */
                      )

      //****** Default Analog Configuration for Poll NFC-B Rx. ******/
    , MODE_ENTRY_3_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF3, 0xff, 0x18
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x1U<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_rx_tol, ST25R3911_REG_AUX_rx_tol                           /* rx_tol On as default */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x04
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x04
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x22
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_848 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x22
                      )
                      
    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_1695 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x6c
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCB | RFAL_ANALOG_CONFIG_BITRATE_3390 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x6c
                      )
                      
      //****** Default Analog Configuration for Poll NFC-F Tx. ******/
    , MODE_ENTRY_2_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, ST25R3911_REG_AUX_tr_am /* AM */
                      , ST25R3911_REG_RFO_AM_ON_LEVEL,        0xff, AM_MOD_DRIVER_LEVEL_DEFAULT /* Fixed driver for AM level: ~14% */
                      )

      //****** Default Analog Configuration for Poll NFC-F Common bitrate Rx. ******/
    , MODE_ENTRY_3_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF3, 0xff, 0x18
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x1U<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_rx_tol, ST25R3911_REG_AUX_rx_tol                           /* rx_tol On as default */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x13   /* dev. from data sheet: lp 300kKz */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCF | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x0b   /* dev. from data sheet: lp 600kHz */
                      )
                      
    //****** Default Analog Configuration for Poll NFC-V Common bitrate Tx ******/
    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX, ST25R3911_REG_AUX_tr_am, 0x00
                      )

    //****** Default Analog Configuration for Poll NFC-V Common bitrate Rx ******/
    , MODE_ENTRY_4_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_NFCV | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1, 0x7f, 0x0c /* use filter settings from table 9: "Recommended for 424/484 kHz sub-carrier" */
                      , ST25R3911_REG_RX_CONF3, 0xff, 0x18
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x1U<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_rx_tol, ST25R3911_REG_AUX_rx_tol                           /* rx_tol On as default */
                      )

      //****** Default Analog Configuration for Poll AP2P Common bitrate Tx. ******/
    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_RFO_AM_ON_LEVEL,        0xff, AM_MOD_DRIVER_LEVEL_DEFAULT /* Fixed driver for AM level: ~14% */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, 0x00 /* OOK */
                      )

    , MODE_ENTRY_2_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, ST25R3911_REG_AUX_tr_am /* AM */
                      , ST25R3911_REG_RFO_AM_ON_LEVEL,        0xff, AM_MOD_DRIVER_LEVEL_DEFAULT /* Fixed driver for AM level: ~14% */
                      )

    , MODE_ENTRY_2_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_TX)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, ST25R3911_REG_AUX_tr_am /* AM */
                      , ST25R3911_REG_RFO_AM_ON_LEVEL,        0xff, AM_MOD_DRIVER_LEVEL_DEFAULT /* Fixed driver for AM level: ~14% */
                      )

      //****** Default Analog Configuration for Poll AP2P Common bitrate Rx. ******/
      
    , MODE_ENTRY_4_REG( (RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_LISTEN_ON)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x45
                      , ST25R3911_REG_RX_CONF3,              (ST25R3911_REG_RX_CONF3_lim | ST25R3911_REG_RX_CONF3_rg_nfc), (ST25R3911_REG_RX_CONF3_lim | ST25R3911_REG_RX_CONF3_rg_nfc)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_rx_tol, ST25R3911_REG_AUX_rx_tol                            /* rx_tol On as default */
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x1U<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                      )
                      
    , MODE_ENTRY_4_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF1,               0x7f, 0x45
                      , ST25R3911_REG_RX_CONF3,              (ST25R3911_REG_RX_CONF3_lim | ST25R3911_REG_RX_CONF3_rg_nfc), (ST25R3911_REG_RX_CONF3_lim | ST25R3911_REG_RX_CONF3_rg_nfc)
                      , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_rx_tol, ST25R3911_REG_AUX_rx_tol                            /* rx_tol On as default */
                      , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x1U<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF3,               ST25R3911_REG_RX_CONF3_mask_rg1_am, 0xc0
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF3,               ST25R3911_REG_RX_CONF3_mask_rg1_am, 0x00
                      )

    , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_POLL | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_RX)
                      , ST25R3911_REG_RX_CONF3,               ST25R3911_REG_RX_CONF3_mask_rg1_am, 0x00
                      )

      //****** Default Analog Configuration for Listen AP2P Common bitrate Tx. ******/
      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_TX)
                        , ST25R3911_REG_RFO_AM_ON_LEVEL,        0xff, AM_MOD_DRIVER_LEVEL_DEFAULT /* Fixed driver for AM level: ~14% */
                        )

      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_TX)
                        , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, 0x00 /* OOK */
                        )

      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_TX)
                        , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, ST25R3911_REG_AUX_tr_am /* AM */
                        )

      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_TX)
                        , ST25R3911_REG_AUX,                    ST25R3911_REG_AUX_tr_am, ST25R3911_REG_AUX_tr_am /* AM */
                        )

        //****** Default Analog Configuration for Listen AP2P Common bitrate Rx. ******/
      , MODE_ENTRY_3_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_COMMON | RFAL_ANALOG_CONFIG_RX)
                        , ST25R3911_REG_RX_CONF1,               0x7f, 0x45
                        , ST25R3911_REG_RX_CONF3,              (ST25R3911_REG_RX_CONF3_lim | ST25R3911_REG_RX_CONF3_rg_nfc), (ST25R3911_REG_RX_CONF3_lim | ST25R3911_REG_RX_CONF3_rg_nfc)
                        , ST25R3911_REG_RX_CONF4,               ST25R3911_REG_RX_CONF4_mask_rg2_am, 0x1U<<ST25R3911_REG_RX_CONF4_shift_rg2_am /* increase digitizer window for AM */
                        )

      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_106 | RFAL_ANALOG_CONFIG_RX)
                        , ST25R3911_REG_RX_CONF3,               ST25R3911_REG_RX_CONF3_mask_rg1_am, 0xc0
                        )

      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_212 | RFAL_ANALOG_CONFIG_RX)
                        , ST25R3911_REG_RX_CONF3,               ST25R3911_REG_RX_CONF3_mask_rg1_am, 0x00
                        )

      , MODE_ENTRY_1_REG( (RFAL_ANALOG_CONFIG_LISTEN | RFAL_ANALOG_CONFIG_TECH_AP2P | RFAL_ANALOG_CONFIG_BITRATE_424 | RFAL_ANALOG_CONFIG_RX)
                        , ST25R3911_REG_RX_CONF3,               ST25R3911_REG_RX_CONF3_mask_rg1_am, 0x00
                        )
};
#endif
