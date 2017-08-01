
#include "sx1278-rf-cfg.h"
#include "sx1278-const.h"

#ifdef USE_SX1278_RADIO

/*!
 * SX1278 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

extern sx1278_rf_cfg_t lora_settings;


void SX1278LoRaSetDefaults( void )
{
    // REMARK: See SX1276 datasheet for modified default values.

    SX1276Read( REG_LR_VERSION, &SX1276LR->RegVersion );
}

void SX1278LoRaSetRFFrequency( uint32_t freq )
{
    lora_settings.RFFrequency = freq;

    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    psx1278_reg_lora->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    psx1278_reg_lora->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    psx1278_reg_lora->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    burst_write( REG_LR_FRFMSB, &psx1278_reg_lora->RegFrfMsb, 3 );
}

uint32_t SX1278LoRaGetRFFrequency( void )
{
    single_read_procBuffer( REG_LR_FRFMSB, &psx1278_reg_lora->RegFrfMsb, 3 );
    lora_settings.RFFrequency = ( ( uint32_t )psx1278_reg_lora->RegFrfMsb << 16 ) | ( ( uint32_t )psx1278_reg_lora->RegFrfMid << 8 ) | ( ( uint32_t )psx1278_reg_lora->RegFrfLsb );
    lora_settings.RFFrequency = ( uint32_t )( ( double )lora_settings.RFFrequency * ( double )FREQ_STEP );

    return lora_settings.RFFrequency;
}

void SX1278LoRaSetRFPower( int8_t power )
{
    single_read_proc( REG_LR_PACONFIG, &psx1278_reg_lora->RegPaConfig );
    single_read_proc( REG_LR_PADAC, &psx1278_reg_lora->RegPaDac );

    if ( ( psx1278_reg_lora->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if ( ( psx1278_reg_lora->RegPaDac & 0x87 ) == 0x87 )
        {
            if ( power < 5 )
            {
                power = 5;
            }
            if ( power > 20 )
            {
                power = 20;
            }
            psx1278_reg_lora->RegPaConfig = ( psx1278_reg_lora->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            psx1278_reg_lora->RegPaConfig = ( psx1278_reg_lora->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if ( power < 2 )
            {
                power = 2;
            }
            if ( power > 17 )
            {
                power = 17;
            }
            psx1278_reg_lora->RegPaConfig = ( psx1278_reg_lora->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            psx1278_reg_lora->RegPaConfig = ( psx1278_reg_lora->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if ( power < -1 )
        {
            power = -1;
        }
        if ( power > 14 )
        {
            power = 14;
        }
        psx1278_reg_lora->RegPaConfig = ( psx1278_reg_lora->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
        psx1278_reg_lora->RegPaConfig = ( psx1278_reg_lora->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    burst_write_byte( REG_LR_PACONFIG, psx1278_reg_lora->RegPaConfig );
    lora_settings.Power = power;
}

int8_t SX1278LoRaGetRFPower( void )
{
    single_read_proc( REG_LR_PACONFIG, &psx1278_reg_lora->RegPaConfig );
    single_read_proc( REG_LR_PADAC, &psx1278_reg_lora->RegPaDac );

    if ( ( psx1278_reg_lora->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if ( ( psx1278_reg_lora->RegPaDac & 0x07 ) == 0x07 )
        {
            lora_settings.Power = 5 + ( psx1278_reg_lora->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
        else
        {
            lora_settings.Power = 2 + ( psx1278_reg_lora->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
    }
    else
    {
        lora_settings.Power = -1 + ( psx1278_reg_lora->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
    }
    return lora_settings.Power;
}

void SX1278LoRaSetSignalBandwidth( uint8_t bw )
{
    single_read_proc( REG_LR_MODEMCONFIG1, &psx1278_reg_lora->RegModemConfig1 );
    psx1278_reg_lora->RegModemConfig1 = ( psx1278_reg_lora->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
    burst_write_byte( REG_LR_MODEMCONFIG1, psx1278_reg_lora->RegModemConfig1 );
    lora_settings.SignalBw = bw;
}

uint8_t SX1278LoRaGetSignalBandwidth( void )
{
    single_read_proc( REG_LR_MODEMCONFIG1, &psx1278_reg_lora->RegModemConfig1 );
    lora_settings.SignalBw = ( psx1278_reg_lora->RegModemConfig1 & ~RFLR_MODEMCONFIG1_BW_MASK ) >> 4;
    return lora_settings.SignalBw;
}

void SX1278LoRaSetSpreadingFactor( uint8_t factor )
{

    if ( factor > 12 )
    {
        factor = 12;
    }
    else if ( factor < 6 )
    {
        factor = 6;
    }

    if ( factor == 6 )
    {
        SX1278LoRaSetNbTrigPeaks( 5 );
    }
    else
    {
        SX1278LoRaSetNbTrigPeaks( 3 );
    }

    single_read_proc( REG_LR_MODEMCONFIG2, &psx1278_reg_lora->RegModemConfig2 );
    psx1278_reg_lora->RegModemConfig2 = ( psx1278_reg_lora->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
    burst_write_byte( REG_LR_MODEMCONFIG2, psx1278_reg_lora->RegModemConfig2 );
    lora_settings.SpreadingFactor = factor;
}

uint8_t SX1278LoRaGetSpreadingFactor( void )
{
    single_read_proc( REG_LR_MODEMCONFIG2, &psx1278_reg_lora->RegModemConfig2 );
    lora_settings.SpreadingFactor = ( psx1278_reg_lora->RegModemConfig2 & ~RFLR_MODEMCONFIG2_SF_MASK ) >> 4;
    return lora_settings.SpreadingFactor;
}

void SX1278LoRaSetErrorCoding( uint8_t value )
{
    single_read_proc( REG_LR_MODEMCONFIG1, &psx1278_reg_lora->RegModemConfig1 );
    psx1278_reg_lora->RegModemConfig1 = ( psx1278_reg_lora->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
    burst_write_byte( REG_LR_MODEMCONFIG1, psx1278_reg_lora->RegModemConfig1 );
    lora_settings.ErrorCoding = value;
}

uint8_t SX1278LoRaGetErrorCoding( void )
{
    single_read_proc( REG_LR_MODEMCONFIG1, &psx1278_reg_lora->RegModemConfig1 );
    lora_settings.ErrorCoding = ( psx1278_reg_lora->RegModemConfig1 & ~RFLR_MODEMCONFIG1_CODINGRATE_MASK ) >> 1;
    return lora_settings.ErrorCoding;
}

void SX1278LoRaSetPacketCrcOn( bool enable )
{
    single_read_proc( REG_LR_MODEMCONFIG2, &psx1278_reg_lora->RegModemConfig2 );
    psx1278_reg_lora->RegModemConfig2 = ( psx1278_reg_lora->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
    burst_write_byte( REG_LR_MODEMCONFIG2, psx1278_reg_lora->RegModemConfig2 );
    lora_settings.CrcOn = enable;
}

void SX1278LoRaSetPreambleLength( uint16_t value )
{
    single_read_procBuffer( REG_LR_PREAMBLEMSB, &psx1278_reg_lora->RegPreambleMsb, 2 );

    psx1278_reg_lora->RegPreambleMsb = ( value >> 8 ) & 0x00FF;
    psx1278_reg_lora->RegPreambleLsb = value & 0xFF;
    burst_write( REG_LR_PREAMBLEMSB, &psx1278_reg_lora->RegPreambleMsb, 2 );
}

uint16_t SX1278LoRaGetPreambleLength( void )
{
    single_read_procBuffer( REG_LR_PREAMBLEMSB, &psx1278_reg_lora->RegPreambleMsb, 2 );
    return ( ( psx1278_reg_lora->RegPreambleMsb & 0x00FF ) << 8 ) | psx1278_reg_lora->RegPreambleLsb;
}

bool SX1278LoRaGetPacketCrcOn( void )
{
    single_read_proc( REG_LR_MODEMCONFIG2, &psx1278_reg_lora->RegModemConfig2 );
    lora_settings.CrcOn = ( psx1278_reg_lora->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON ) >> 1;
    return lora_settings.CrcOn;
}

void SX1278LoRaSetImplicitHeaderOn( bool enable )
{
    single_read_proc( REG_LR_MODEMCONFIG1, &psx1278_reg_lora->RegModemConfig1 );
    psx1278_reg_lora->RegModemConfig1 = ( psx1278_reg_lora->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
    burst_write_byte( REG_LR_MODEMCONFIG1, psx1278_reg_lora->RegModemConfig1 );
    lora_settings.ImplicitHeaderOn = enable;
}

bool SX1278LoRaGetImplicitHeaderOn( void )
{
    single_read_proc( REG_LR_MODEMCONFIG1, &psx1278_reg_lora->RegModemConfig1 );
    lora_settings.ImplicitHeaderOn = ( psx1278_reg_lora->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_ON );
    return lora_settings.ImplicitHeaderOn;
}

void SX1278LoRaSetRxSingleOn( bool enable )
{
    lora_settings.RxSingleOn = enable;
}

bool SX1278LoRaGetRxSingleOn( void )
{
    return lora_settings.RxSingleOn;
}

void SX1278LoRaSetFreqHopOn( bool enable )
{
    lora_settings.FreqHopOn = enable;
}

bool SX1278LoRaGetFreqHopOn( void )
{
    return lora_settings.FreqHopOn;
}

void SX1278LoRaSetHopPeriod( uint8_t value )
{
    psx1278_reg_lora->RegHopPeriod = value;
    burst_write_byte( REG_LR_HOPPERIOD, psx1278_reg_lora->RegHopPeriod );
    lora_settings.HopPeriod = value;
}

uint8_t SX1278LoRaGetHopPeriod( void )
{
    single_read_proc( REG_LR_HOPPERIOD, &psx1278_reg_lora->RegHopPeriod );
    lora_settings.HopPeriod = psx1278_reg_lora->RegHopPeriod;
    return lora_settings.HopPeriod;
}

void SX1278LoRaSetTxPacketTimeout( uint32_t value )
{
    lora_settings.TxPacketTimeout = value;
}

uint32_t SX1278LoRaGetTxPacketTimeout( void )
{
    return lora_settings.TxPacketTimeout;
}

void SX1278LoRaSetRxPacketTimeout( uint32_t value )
{
    lora_settings.RxPacketTimeout = value;
}

uint32_t SX1278LoRaGetRxPacketTimeout( void )
{
    return lora_settings.RxPacketTimeout;
}

void SX1278LoRaSetPayloadLength( uint8_t value )
{
    psx1278_reg_lora->RegPayloadLength = value;
    burst_write_byte( REG_LR_PAYLOADLENGTH, psx1278_reg_lora->RegPayloadLength );
    lora_settings.PayloadLength = value;
}

uint8_t SX1278LoRaGetPayloadLength( void )
{
    single_read_proc( REG_LR_PAYLOADLENGTH, &psx1278_reg_lora->RegPayloadLength );
    lora_settings.PayloadLength = psx1278_reg_lora->RegPayloadLength;
    return lora_settings.PayloadLength;
}

void SX1278LoRaSetPa20dBm( bool enale )
{
    single_read_proc( REG_LR_PADAC, &psx1278_reg_lora->RegPaDac );
    single_read_proc( REG_LR_PACONFIG, &psx1278_reg_lora->RegPaConfig );

    if ( ( psx1278_reg_lora->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if ( enale == true )
        {
            psx1278_reg_lora->RegPaDac = 0x87;
        }
    }
    else
    {
        psx1278_reg_lora->RegPaDac = 0x84;
    }
    burst_write_byte( REG_LR_PADAC, psx1278_reg_lora->RegPaDac );
}

bool SX1278LoRaGetPa20dBm( void )
{
    single_read_proc( REG_LR_PADAC, &psx1278_reg_lora->RegPaDac );

    return ( ( psx1278_reg_lora->RegPaDac & 0x07 ) == 0x07 ) ? true : false;
}

void SX1278LoRaSetPAOutput( uint8_t outputPin )
{
    single_read_proc( REG_LR_PACONFIG, &psx1278_reg_lora->RegPaConfig );
    psx1278_reg_lora->RegPaConfig = (psx1278_reg_lora->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | outputPin;
    burst_write_byte( REG_LR_PACONFIG, psx1278_reg_lora->RegPaConfig );
}

uint8_t SX1278LoRaGetPAOutput( void )
{
    single_read_proc( REG_LR_PACONFIG, &psx1278_reg_lora->RegPaConfig );
    return psx1278_reg_lora->RegPaConfig & ~RFLR_PACONFIG_PASELECT_MASK;
}

void SX1278LoRaSetPaRamp( uint8_t value )
{
    single_read_proc( REG_LR_PARAMP, &psx1278_reg_lora->RegPaRamp );
    psx1278_reg_lora->RegPaRamp = ( psx1278_reg_lora->RegPaRamp & RFLR_PARAMP_MASK ) | ( value & ~RFLR_PARAMP_MASK );
    burst_write_byte( REG_LR_PARAMP, psx1278_reg_lora->RegPaRamp );
}

uint8_t SX1278LoRaGetPaRamp( void )
{
    single_read_proc( REG_LR_PARAMP, &psx1278_reg_lora->RegPaRamp );
    return psx1278_reg_lora->RegPaRamp & ~RFLR_PARAMP_MASK;
}

void SX1278LoRaSetSymbTimeout( uint16_t value )
{
    single_read_procBuffer( REG_LR_MODEMCONFIG2, &psx1278_reg_lora->RegModemConfig2, 2 );

    psx1278_reg_lora->RegModemConfig2 = ( psx1278_reg_lora->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    psx1278_reg_lora->RegSymbTimeoutLsb = value & 0xFF;
    burst_write( REG_LR_MODEMCONFIG2, &psx1278_reg_lora->RegModemConfig2, 2 );
}

uint16_t SX1278LoRaGetSymbTimeout( void )
{
    single_read_procBuffer( REG_LR_MODEMCONFIG2, &psx1278_reg_lora->RegModemConfig2, 2 );
    return ( ( psx1278_reg_lora->RegModemConfig2 & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) << 8 ) | psx1278_reg_lora->RegSymbTimeoutLsb;
}

void SX1278LoRaSetLowDatarateOptimize( bool enable )
{
    single_read_proc( REG_LR_MODEMCONFIG3, &psx1278_reg_lora->RegModemConfig3 );
    psx1278_reg_lora->RegModemConfig3 = ( psx1278_reg_lora->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) | ( enable << 3 );
    burst_write_byte( REG_LR_MODEMCONFIG3, psx1278_reg_lora->RegModemConfig3 );
}

bool SX1278LoRaGetLowDatarateOptimize( void )
{
    single_read_proc( REG_LR_MODEMCONFIG3, &psx1278_reg_lora->RegModemConfig3 );
    return ( ( psx1278_reg_lora->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON ) >> 3 );
}

void SX1278LoRaSetNbTrigPeaks( uint8_t value )
{
    single_read_proc( 0x31, &psx1278_reg_lora->RegDetectOptimize );
    psx1278_reg_lora->RegDetectOptimize = ( psx1278_reg_lora->RegDetectOptimize & 0xF8 ) | value;
    burst_write_byte( 0x31, psx1278_reg_lora->RegDetectOptimize );
}

uint8_t SX1278LoRaGetNbTrigPeaks( void )
{
    single_read_proc( 0x31, &psx1278_reg_lora->RegDetectOptimize );
    return ( psx1278_reg_lora->RegDetectOptimize & 0x07 );
}

#endif // USE_SX1278_RADIO
