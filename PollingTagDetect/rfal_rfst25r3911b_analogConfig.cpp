#include "rfal_rfst25r3911b.h"

ReturnCode rfal_rfst25r3911b::rfalSetAnalogConfig( rfalAnalogConfigId configId )
{
    ReturnCode retCode = ERR_NONE;
    rfalAnalogConfigOffset configOffset = 0;
    rfalAnalogConfigNum numConfigSet;
    rfalAnalogConfigRegAddrMaskVal *configTbl;
    
    rfalAnalogConfigNum i;
    
    if (true != gRfalAnalogConfigMgmt.ready)
    {
        return ERR_REQUEST;
    }
    
    /* Search LUT for the specific Configuration ID. */
    while(true)
    {
        numConfigSet = rfalAnalogConfigSearch(configId, &configOffset);
        if( RFAL_ANALOG_CONFIG_LUT_NOT_FOUND == numConfigSet )
        {
            break;
        }
          
        configTbl = (rfalAnalogConfigRegAddrMaskVal *)( (uint32_t)gRfalAnalogConfigMgmt.currentAnalogConfigTbl + (uint32_t)configOffset); 
        /* Increment the offset to the next index to search from. */
        configOffset += (uint16_t)(numConfigSet * sizeof(rfalAnalogConfigRegAddrMaskVal)); 
        
        if ((gRfalAnalogConfigMgmt.configTblSize + 1U) < configOffset)
        {   /* Error check make sure that the we do not access outside the configuration Table Size */
            return ERR_NOMEM;
        }
        
        for ( i = 0; i < numConfigSet; i++)
        {
            if( (GETU16(configTbl[i].addr) & RFAL_TEST_REG) != 0U )
            {
                EXIT_ON_ERR(retCode, rfalChipChangeTestRegBits( (GETU16(configTbl[i].addr) & ~RFAL_TEST_REG), configTbl[i].mask, configTbl[i].val) );
            }
            else
            {
                EXIT_ON_ERR(retCode, rfalChipChangeRegBits( GETU16(configTbl[i].addr), configTbl[i].mask, configTbl[i].val) );
            }
        }
        
    } /* while(found Analog Config Id) */
    
    return retCode;
    
} /* rfalSetAnalogConfig() */

void rfal_rfst25r3911b::rfalAnalogConfigInitialize( void )
{
    /* Use default Analog configuration settings in Flash by default. */

    /* Check whether the Default Analog settings are to be used or custom ones */
    #ifdef RFAL_ANALOG_CONFIG_CUSTOM
        gRfalAnalogConfigMgmt.currentAnalogConfigTbl = (const uint8_t *)&rfalAnalogConfigCustomSettings;
        gRfalAnalogConfigMgmt.configTblSize          = rfalAnalogConfigCustomSettingsLength;
    #else  
        gRfalAnalogConfigMgmt.currentAnalogConfigTbl = (const uint8_t *)&rfalAnalogConfigDefaultSettings;
        gRfalAnalogConfigMgmt.configTblSize          = sizeof(rfalAnalogConfigDefaultSettings);
    #endif
  
  gRfalAnalogConfigMgmt.ready = true;
} /* rfalAnalogConfigInitialize() */

rfalAnalogConfigNum rfal_rfst25r3911b::rfalAnalogConfigSearch( rfalAnalogConfigId configId, uint16_t *configOffset )
{
    rfalAnalogConfigId foundConfigId;
    rfalAnalogConfigId configIdMaskVal;
    const uint8_t *configTbl;
    const uint8_t *currentConfigTbl;
    uint16_t i;
    
    currentConfigTbl = gRfalAnalogConfigMgmt.currentAnalogConfigTbl;
    configIdMaskVal  = ((RFAL_ANALOG_CONFIG_POLL_LISTEN_MODE_MASK | RFAL_ANALOG_CONFIG_BITRATE_MASK) 
                       |((RFAL_ANALOG_CONFIG_TECH_CHIP == RFAL_ANALOG_CONFIG_ID_GET_TECH(configId)) ? (RFAL_ANALOG_CONFIG_TECH_MASK | RFAL_ANALOG_CONFIG_CHIP_SPECIFIC_MASK) : configId)
                       |((RFAL_ANALOG_CONFIG_NO_DIRECTION == RFAL_ANALOG_CONFIG_ID_GET_DIRECTION(configId)) ? RFAL_ANALOG_CONFIG_DIRECTION_MASK : configId)
                       );
   
    i = *configOffset;
    while (i < gRfalAnalogConfigMgmt.configTblSize)
    {
        configTbl = &currentConfigTbl[i];
        foundConfigId = GETU16(configTbl);
        if (configId == (foundConfigId & configIdMaskVal))
        {
            *configOffset = (uint16_t)(i + sizeof(rfalAnalogConfigId) + sizeof(rfalAnalogConfigNum));
            return configTbl[sizeof(rfalAnalogConfigId)];
        }
        
        /* If Config Id does not match, increment to next Configuration Id */
        i += (uint16_t)( sizeof(rfalAnalogConfigId) + sizeof(rfalAnalogConfigNum) 
                        + (configTbl[sizeof(rfalAnalogConfigId)] * sizeof(rfalAnalogConfigRegAddrMaskVal) )
                        );
    } /* for */
    
    return RFAL_ANALOG_CONFIG_LUT_NOT_FOUND; 
}
