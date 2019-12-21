#include "Arduino.h"
#include "RobustSerial.h"

//#define _DEBUG
//#define _DEBUG_SEND_PACKAGE
//#define _DEBUG_SEND_ERROR

RobustSerial::RobustSerial( HardwareSerial  &serial,
                            byte            bySerialNo,
                            Interface       *pInterface) :  serial(     serial),
                                                            bySerialNo( bySerialNo),
                                                            pInterface( pInterface),
                                                            nBufferSize(0),
                                                            mode(       MODE_IDLE)
{
}
              
RobustSerial::~RobustSerial()
{
}

bool RobustSerial::send(uint8_t     byType,
                        const byte  byData,
                        uint16_t    dwPackageSize)
{
  return this->send(byType, (byte*)&byData, 1, dwPackageSize);
}

bool RobustSerial::send(uint8_t   byType,
                        String    &sData,
                        uint16_t  dwPackageSize)
{
  return this->send(byType, (byte*)sData.c_str(), sData.length() + 1, dwPackageSize);
}
                      
bool RobustSerial::send(uint8_t   byType,
                        byte      *pData,
                        uint16_t  dwSize,
                        uint16_t  dwPackageSize)
{
  if (this->mode!=MODE_IDLE)
  {
    return false;
  }

  if (dwPackageSize==0)
  {
    dwPackageSize = nBufferSizeMax;
  }
  
  uint16_t  nDataLeft(  dwSize),
            size(       0);

  //#ifdef _DEBUG
  //Serial.println("send()");
  //Serial.print("byType: ");Serial.println(byType);
  //Serial.print("dwSize: ");Serial.println(dwSize);
  //#endif

  while(nDataLeft > 0)
  {
    byte      *pDataWrite(    NULL);
    uint16_t  dwDataWriteSize(0);

    if (this->nBufferSize > 0)
    {
      /// Use the buffer
      while(this->nBufferSize < dwPackageSize && nDataLeft > 0)
      {
        this->pbyBuffer[this->nBufferSize++] = *pData;

        ++pData;
        --nDataLeft;
      }

      if (this->nBufferSize >= dwPackageSize)
      {
        /// The buffer is ready for sending!
        
        pDataWrite      = this->pbyBuffer;
        dwDataWriteSize = this->nBufferSize;

        this->nBufferSize = 0;
      }
    }
    else
    {
      if (nDataLeft > dwPackageSize)
      {
        // Data is bigger than the buffer, so just directly write out the data!

        pDataWrite      = pData;
        dwDataWriteSize = dwPackageSize;

        pData     += dwPackageSize;
        nDataLeft -= dwPackageSize;
      }
      else
      {
        /// Put the rest to the buffer!
        while(this->nBufferSize < dwPackageSize && nDataLeft > 0)
        {
          this->pbyBuffer[this->nBufferSize++] = *pData;
  
          ++pData;
          --nDataLeft;
        }
      }
    }
    
    if (pDataWrite)
    {
      #if defined(_DEBUG) || defined(_DEBUG_SEND_PACKAGE)
      Serial.println("->send data");
      Serial.print("dwDataWriteSize: ");Serial.println(dwDataWriteSize);
      #endif
  
      this->serial.write((byte)PACKET_DATA);
      this->serial.write((byte)byType);
      this->serial.write((byte*)&dwDataWriteSize, sizeof(uint16_t));
      this->serial.write((byte*)pDataWrite, dwDataWriteSize);
      this->serial.flush();

      /*
      uint16_t dwWriteSize(0);

      do
      {
        dwWriteSize = dwDataWriteSize > 16 ? 16 : dwDataWriteSize;
        
        this->serial.write((byte*)pDataWrite, dwWriteSize);
        this->serial.flush();

        dwDataWriteSize -= 16;
        pDataWrite      += 16;
      } 
      while(dwWriteSize == 16);
      */
  
      // Wait for receiver fetched data
      if (!this->waitForFetched())
      {
        #if defined(_DEBUG) || defined(_DEBUG_SEND_ERROR)
        Serial.println("Error: Other side not fetched data!");
        #endif
        
        return false;
      }
    }

    if (nDataLeft>0)
    {
      #ifdef _DEBUG
      Serial.print("nDataLeft: ");Serial.println(nDataLeft);
      #endif
    }    
  }
  
  return true;
}

bool RobustSerial::sendFinish(uint8_t byType)
{
  if (this->mode!=MODE_IDLE)
  {
    return false;
  }
  
  #ifdef _DEBUG
  Serial.println("sendFinish()");
  Serial.print("byType: ");Serial.println(byType);
  #endif
  
  if (this->nBufferSize > 0)
  {
    #if defined(_DEBUG) || defined(_DEBUG_SEND_PACKAGE)
    Serial.println("->send data");
    Serial.print("nBufferSize: ");Serial.println(this->nBufferSize);
    #endif
    
    this->serial.write((byte)PACKET_DATA);
    this->serial.write((byte)byType);
    this->serial.write((byte*)&this->nBufferSize, sizeof(uint16_t));
    this->serial.write((byte*)this->pbyBuffer, this->nBufferSize);
    this->serial.flush();

    this->nBufferSize = 0;

    // Wait for receiver fetched data
    if (!this->waitForFetched())
    {
      #if defined(_DEBUG) || defined(_DEBUG_SEND_ERROR)
      Serial.println("Error: Other side not fetched data!");
      #endif
        
      return false;
    }
    
    #ifdef _DEBUG
    Serial.println("OK");
    #endif
  }

  #ifdef _DEBUG
  Serial.println("Send the finish package");Serial.println();
  #endif

  /// Send the finish package
  this->serial.write((byte)PACKET_DATA_FINISH);
  this->serial.write((byte)byType);
  this->serial.flush();

  // Wait for receiver fetched data
  if (!this->waitForFetched())
  {
    #ifdef _DEBUG
    Serial.println("Error: Other side not fetched data!");
    #endif
    
    return false;
  }

  return true;
}
                
void RobustSerial::fetch()
{
  byte      byPacketData( 0),
            byType(       0);
  uint16_t  size(         0);
  byte      pData[this->nBufferSizeMax];
  bool      bFirst(       true);
  
  while(this->serial.available() || this->mode==MODE_FETCHING)
  {
    /// In fetching mode, wait for the next data!
    if (this->mode==MODE_FETCHING &&
        !this->waitForData(1))
    {
      break;
    }
    
    #ifdef _DEBUG
    Serial.println("fetch()");
    #endif
    
    byPacketData = this->serial.read();

    switch(byPacketData)
    {
      case PACKET_DATA:
      { 
        this->mode = MODE_FETCHING;
        
        if (!this->waitForData(sizeof(uint8_t)))
        {
          #ifdef _DEBUG
          Serial.println("Error missing data");
          #endif
          
          /// Signalize that an error occured
          this->serial.write(PACKET_BUFFER_ERROR);
          this->serial.flush();
        
          continue;
        }
        
        /// Get the data type
        byType = this->serial.read();

        #ifdef _DEBUG
        Serial.println("PACKET_DATA");
        Serial.print("byType: ");Serial.println(byType);
        #endif

        if (!this->waitForData(sizeof(uint16_t)))
        {
          #ifdef _DEBUG
          Serial.println("Error missing data");
          #endif
          
          /// Signalize that an error occured
          this->serial.write(PACKET_BUFFER_ERROR);
          this->serial.flush();
        
          continue;
        }

        /// Get the data size
        this->serial.readBytes((byte*)&size,  sizeof(uint16_t));

        #ifdef _DEBUG
        Serial.print("size: ");Serial.println(size);
        #endif

        if (size > this->nBufferSizeMax)
        {
          /// Signalize that an error occured
          this->serial.write(PACKET_BUFFER_ERROR);
          this->serial.flush();
          
          this->emptySerial();
          return;
        }


        /*
        if (!this->waitForData(size))
        {
          #ifdef _DEBUG
          Serial.println("Error missing data");
          #endif
          
          /// Signalize that an error occured
          this->serial.write(PACKET_BUFFER_ERROR);
          this->serial.flush();
        
          continue;
        }
        
        /// Read all package data
        this->serial.readBytes(pData, size);
        */
        
        /// Read all package data
        for (int i = 0, j = 0; i < size; ++i)
        {
          /// Reset counter
          j = 200;
          
          while(!this->serial.available() && j > 0)
          {
            --j;
            
            delay(10);
          }

          if (j==0)
          {
            #ifdef _DEBUG
            Serial.println("Error missing data");
            #endif
            
            /// Signalize that an error occured
            this->serial.write(PACKET_BUFFER_ERROR);
            this->serial.flush();
          
            return;
          }
                    
          pData[i] = this->serial.read();          
        }        

        #ifdef _DEBUG
        Serial.println("read data ok");
        #endif

        /// Signalize that the buffer has been read
        this->serial.write(PACKET_BUFFER_FETCHED);
        this->serial.flush();
        
        this->pInterface->onSerialData(this->bySerialNo, byType, pData, size, bFirst);

        bFirst = false;
      }
      break;

      case PACKET_DATA_FINISH:
      {
        if (!this->waitForData(sizeof(uint8_t))) return;
        
        /// Get the data type
        byType = this->serial.read();

        #ifdef _DEBUG
        Serial.println("PACKET_DATA_FINISH");
        Serial.print("byType: ");Serial.println(byType);
        #endif

        /// Signalize that the buffer has been read
        this->serial.write((byte)PACKET_BUFFER_FETCHED);
        this->serial.flush();

        /// Data fetching is done!
        this->mode  = MODE_IDLE;
        bFirst      = true;
        
        this->pInterface->onSerialDataFinish(this->bySerialNo, byType);
      }
      break;

      case PACKET_BUFFER_FETCHED:
      #ifdef _DEBUG
      Serial.println("Got other side buffer fetched to late!");
      #endif
      break;

      case PACKET_BUFFER_ERROR:
      #ifdef _DEBUG
      Serial.println("An buffer fetch error occurred!");
      #endif
      break;

      default:
      #ifdef _DEBUG
      Serial.println("ERROR");
      Serial.print("byPacketData: ");Serial.println(byPacketData);
      #endif
      break;
    }
  }
}

bool RobustSerial::waitForData(uint16_t size)
{  
  for (int i = 0, j = 0, k = 0; i < 100; ++i)
  {
    k = this->serial.available();
    
    if (k != j)
    {
      if (k >= size)
      {
        /// The data is now available!
        return true;  
      }
      
      j = k;
      i = 0;
    }
        
    delay(10);
  }

  return false;
}
  
bool RobustSerial::waitForFetched()
{  
  for (int i = 0, j = 0; i < 500; ++i)
  {
    j = this->serial.available();
    
    if (j >= 1)
    {
      byte byData(this->serial.read());

      if (byData==PACKET_BUFFER_FETCHED)
      {
        return true;
      }

      #ifdef _DEBUG
      Serial.println("waitForFetched()");
      Serial.println("Got wrong byte!");
      Serial.print("byData: ");Serial.println(byData);
      #endif
      
      return false;
    }
    
    delay(10);
  }

  return false;
}

void RobustSerial::emptySerial()
{
  while(this->serial.available() > 0)
  {
    this->serial.read();
  }

  this->serial.write((byte)PACKET_BUFFER_FETCHED);
  this->serial.flush();
}
