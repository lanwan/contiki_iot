
#ifndef SX1278_LORA_MUSIC_H
#define SX1278_LORA_MUSIC_H

void SX1278LoRaSetDefaults( void );

void SX1278LoRaSetRFFrequency( uint32_t freq );
uint32_t SX1278LoRaGetRFFrequency( void );

void SX1278LoRaSetRFPower( int8_t power );
int8_t SX1278LoRaGetRFPower( void );

void SX1278LoRaSetSignalBandwidth( uint8_t bw );
uint8_t SX1278LoRaGetSignalBandwidth( void );

void SX1278LoRaSetSpreadingFactor( uint8_t factor );
uint8_t SX1278LoRaGetSpreadingFactor( void );

void SX1278LoRaSetErrorCoding( uint8_t value );
uint8_t SX1278LoRaGetErrorCoding( void );

void SX1278LoRaSetPacketCrcOn( bool enable );
bool SX1278LoRaGetPacketCrcOn( void );

void SX1278LoRaSetPreambleLength( uint16_t value );
uint16_t SX1278LoRaGetPreambleLength( void );

void SX1278LoRaSetImplicitHeaderOn( bool enable );
bool SX1278LoRaGetImplicitHeaderOn( void );

void SX1278LoRaSetRxSingleOn( bool enable );
bool SX1278LoRaGetRxSingleOn( void );

void SX1278LoRaSetFreqHopOn( bool enable );
bool SX1278LoRaGetFreqHopOn( void );

void SX1278LoRaSetHopPeriod( uint8_t value );
uint8_t SX1278LoRaGetHopPeriod( void );

void SX1278LoRaSetTxPacketTimeout( uint32_t value );
uint32_t SX1278LoRaGetTxPacketTimeout( void );

void SX1278LoRaSetRxPacketTimeout( uint32_t value );
uint32_t SX1278LoRaGetRxPacketTimeout( void );

void SX1278LoRaSetPayloadLength( uint8_t value );
uint8_t SX1278LoRaGetPayloadLength( void );

void SX1278LoRaSetPa20dBm( bool enale );
bool SX1278LoRaGetPa20dBm( void );

void SX1278LoRaSetPAOutput( uint8_t outputPin );
uint8_t SX1278LoRaGetPAOutput( void );

void SX1278LoRaSetPaRamp( uint8_t value );
uint8_t SX1278LoRaGetPaRamp( void );

void SX1278LoRaSetSymbTimeout( uint16_t value );
uint16_t SX1278LoRaGetSymbTimeout( void );

void SX1278LoRaSetLowDatarateOptimize( bool enable );
bool SX1278LoRaGetLowDatarateOptimize( void );

void SX1278LoRaSetNbTrigPeaks( uint8_t value );
uint8_t SX1278LoRaGetNbTrigPeaks( void );


#endif /* SX1278_LORA_MUSIC_H */