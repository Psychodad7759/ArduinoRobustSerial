#pragma once

class RobustSerial
{
public:

  class Interface
  {
  public:
    
    // This method will be called in the case serial data is available
    virtual void  onSerialData(       byte      bySerialNo,
                                      uint8_t   byType,
                                      byte      *pData,
                                      uint16_t  size,
                                      bool      bFirst) = 0;

    virtual void  onSerialDataFinish( byte      bySerialNo,
                                      uint8_t   byType) = 0;
  };
  
                RobustSerial( HardwareSerial  &serial,
                              byte            bySerialNo,
                              Interface       *pInterface);
              
                ~RobustSerial();

                /// Send one byte (Note: call sendFinish() to finish the data transfer)
          bool  send(         uint8_t         byType,
                              const byte      byData,
                              uint16_t        dwPackageSize = 0);

                /// Send some data (Note: call sendFinish() to finish the data transfer)
          bool  send(         uint8_t         byType,
                              String          &sData,
                              uint16_t        dwPackageSize = 0);

                /// Send some data (Note: call sendFinish() to finish the data transfer)
          bool  send(         uint8_t         byType,
                              byte            *pData,
                              uint16_t        dwSize,
                              uint16_t        dwPackageSize = 0);

          bool  sendFinish(   uint8_t         byType);

                /// Fetch available data
  void          fetch();
  
private:

  enum ePacket
  {
    PACKET_UNDEFINED      = 0x0,
    PACKET_DATA           = 0x1,
    PACKET_DATA_FINISH    = 0x2,
    PACKET_BUFFER_FETCHED = 0x3,
    PACKET_BUFFER_ERROR   = 0x4,
  };

  enum eMode
  {
    MODE_IDLE,
    MODE_FETCHING,
  };

  static  uint16_t const  nBufferSizeMax = 200;
          eMode           mode;
 
  bool  waitForData(uint16_t size);

  bool  waitForFetched();

  void  emptySerial();
  
  HardwareSerial  &serial;
  byte            bySerialNo;
  Interface       *pInterface;
  byte            pbyBuffer[nBufferSizeMax];
  uint16_t        nBufferSize;
};
