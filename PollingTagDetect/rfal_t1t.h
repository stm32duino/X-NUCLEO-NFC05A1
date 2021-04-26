

/*
 ******************************************************************************
 * GLOBAL DEFINES
 ******************************************************************************
 */
#define RFAL_T1T_UID_LEN               4   /*!< T1T UID length of cascade level 1 only tag  */
#define RFAL_T1T_HR_LENGTH             2   /*!< T1T HR(Header ROM) length                   */

#define RFAL_T1T_HR0_NDEF_MASK      0xF0   /*!< T1T HR0 NDEF capability mask  T1T 1.2 2.2.2 */
#define RFAL_T1T_HR0_NDEF_SUPPORT   0x10   /*!< T1T HR0 NDEF capable value    T1T 1.2 2.2.2 */


/*! NFC-A T1T (Topaz) command set */
typedef enum
{
    RFAL_T1T_CMD_RID      = 0x78,          /*!< T1T Read UID                                */
    RFAL_T1T_CMD_RALL     = 0x00,          /*!< T1T Read All                                */
    RFAL_T1T_CMD_READ     = 0x01,          /*!< T1T Read                                    */
    RFAL_T1T_CMD_WRITE_E  = 0x53,          /*!< T1T Write with erase (single byte)          */
    RFAL_T1T_CMD_WRITE_NE = 0x1A           /*!< T1T Write with no erase (single byte)       */
} rfalT1Tcmds;


/*
******************************************************************************
* GLOBAL TYPES
******************************************************************************
*/


/*! NFC-A T1T (Topaz) RID_RES  Digital 1.1  10.6.2 & Table 50 */
typedef struct
{
    uint8_t hr0;                           /*!< T1T Header ROM: HR0                         */
    uint8_t hr1;                           /*!< T1T Header ROM: HR1                         */
    uint8_t uid[RFAL_T1T_UID_LEN];         /*!< T1T UID                                     */
} rfalT1TRidRes;
