/***************************************************************************
** This file is part of the generic algorithm library Wiselib.           **
** Copyright (C) 2012 by the Wisebed (www.wisebed.eu) project.           **
**                                                                       **
** The Wiselib is free software: you can redistribute it and/or modify   **
** it under the terms of the GNU Lesser General Public License as        **
** published by the Free Software Foundation, either version 3 of the    **
** License, or (at your option) any later version.                       **
**                                                                       **
** The Wiselib is distributed in the hope that it will be useful,        **
** but WITHOUT ANY WARRANTY; without even the implied warranty of        **
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
** GNU Lesser General Public License for more details.                   **
**                                                                       **
** You should have received a copy of the GNU Lesser General Public      **
** License along with the Wiselib.                                       **
** If not, see <http://www.gnu.org/licenses/>.                           **
***************************************************************************/

#ifndef _ARDUINO_BLE_RADIO_H
#define _ARDUINO_BLE_RADIO_H

#ifdef ARDUINO_USE_BLE_RADIO

#include <Arduino.h>
#include "arduino_types.h"
#include "util/base_classes/extended_radio_base.h"
#include <SoftwareSerial.h>
#include <string.h>

//---------------------------------------------------------------------------------
// connection; change if you want to use other pins (adjustable by jumpers)
#define RX_PIN 2
#define TX_PIN 3
//---------------------------------------------------------------------------------

#define BAUD_RATE           9600
#define READ_TIMEOUT_MILLIS 500

#define ACK                 0x40
#define NCK                 0x23
#define DEVICE_FOUND        0x02

#define CMD_TEST                     0x01
#define CMD_SET_ADVERTISEMENT_DATA   0x02
#define CMD_SET_NAME                 0x03
#define CMD_GET_FIRMWARE_INFO_STRING 0x04
#define CMD_START_DEVICE             0x05
#define CMD_STOP_DEVICE              0x06

namespace wiselib
{

   /**
    *  \brief Arduino Implementation of \ref extended_adio_concept "Extended Radio concept".
    *  \ingroup extended_radio_concept
    *  \ingroup android_facets
    * This code is desgined for this shield:
    * http://imall.iteadstudio.com/development-platform/arduino/shields/im130704001.html
    * And this firmware:
    * https://github.com/bjoerke/HM-10-Firmware
    *
    * IMPORTANT: There are no dedicated nodes in the network
    *            You can just broadcast to all nodes which will send advertisement data
    */
   template<typename OsModel_P>
   class ArduinoBleRadio : public ExtendedRadioBase<OsModel_P, uint64_t, uint8_t, uint8_t>  //OsModel, NodeId, Size, BlockData
   {
   public:
      typedef OsModel_P OsModel;

      typedef ArduinoBleRadio<OsModel> self_type;
      typedef self_type* self_pointer_t;

      typedef uint64_t node_id_t;    //6 byte MAC address (2 upper bytes are not used)
      typedef uint8_t block_data_t;  //Data type used for raw data in message sending process. 
      typedef uint8_t size_t;        //Unsigned integer that represents length information. 

      typedef BaseExtendedData<OsModel> ExtendedData; //extended data: link quality      

      enum ErrorCodes
      {
         SUCCESS = OsModel::SUCCESS,
         ERR_UNSPEC = OsModel::ERR_UNSPEC,
         ERR_NOTIMPL = OsModel::ERR_NOTIMPL
      };

      enum SpecialNodeIds
      {
         NULL_NODE_ID           = 0x1000000000000000,
         BROADCAST_ADDRESS      = 0x2000000000000000,
      };

      enum Restrictions
      {
         MAX_MESSAGE_LENGTH = 31 ///< Maximal number of bytes in payload
      };

      /**
       * (Constructor)
       */
      ArduinoBleRadio() : shield_(RX_PIN, TX_PIN)
      {
         shield_.begin(BAUD_RATE);
         delay(100); while(shield_.available() > 0) shield_.read();  //empty serial buffer
         if(! test()) Serial.println("Shield not ready!");
         stop_device();  //in case it is already running
         print_firmware_info_string();
         set_device_name("wiselib");
      }

      /**
       * Turn on radio.
       * Messages can be sent and received.
       * \return error code
       */
      int enable_radio()
      {
         //timer_->template set_timer<ArduinoBluetoothRadio<OsModel_P> , &ArduinoBluetoothRadio<OsModel_P>::read_recv_packet > ( POLL_INTERVAL, this , ( void* )timer_ );
         return start_device() ? SUCCESS : ERR_UNSPEC;
      }

      /**
       * Turn off radio.
       * \return error code
       */
      int disable_radio()
      {
         return start_device() ? SUCCESS : ERR_UNSPEC;
      }

      int send(node_id_t id, size_t len, block_data_t* data )
      {
         if(id == BROADCAST_ADDRESS)
         {
            return set_advertisement_data(len, data) ? SUCCESS : ERR_UNSPEC;
         }
         else
         {
            return ERR_NOTIMPL;   //only broadcasting of advertisement data is supported
         }
      }

      /**
       * Must be called frequently; Waits for incomming 'Device Found' responses
       * on serial line.
       */
      void poll()
      {
         uint8_t opcode, length;
         if(shield_.available() > 0)
         {
            read_data(&opcode);
            switch(opcode)
            {
               case DEVICE_FOUND:
               {
                  if(read_data(&length))
                  {
                     for(int i=0; i<length; i++)
                     {
                        if(! read_data(& ( ( (uint8_t*) &device_info_)[i] ) ) ) return;
                     }
                     extended_data_.set_link_metric( ((uint16_t) device_info_.rssi)+(2<<31));  //TODO Firmware passes a SIGNED int, link metric is defined as UNSINGED int
                     ExtendedRadioBase<OsModel_P, node_id_t, size_t, block_data_t>
                        ::notify_receivers(NULL_NODE_ID, (size_t) length-7, (block_data_t*) device_info_.advData, extended_data_);
                  }
                  break;
               }
               default:
                  break; //unknown opcode
            }
         }
      }
                
               

   private:
      SoftwareSerial shield_;

      struct deviceInfo
      {
         uint8_t address[6];
         uint8_t rssi;
         uint8_t advData[31];
      }__attribute__((packed));
      struct deviceInfo device_info_;
       
      ExtendedData extended_data_;

      /**
       * Sends data to shield
       * \param data data to send
       */
      void write_data(uint8_t data)
      {
         shield_.write(data);
      }

      /**
       * Reads data from shield
       * \param data data to fill
       * \return true on success; false on timeout
       */
      bool read_data(uint8_t* data)
      {
         int timeout = READ_TIMEOUT_MILLIS;
         while(timeout != 0)
         {
            if(shield_.available() > 0)
            {
               *data = shield_.read();
               return true;
            }
            delay(1);
            timeout--;
        }
        return false;
      }

      /**
       * Checks if shield is available and ready
       * \return true if ready
       */
      bool test()
      {
         uint8_t rsp;
         for(int i=0; i<100; i++)
         {
            write_data(CMD_TEST);
            if(read_data(&rsp))
            {
               if(rsp == ACK) return true;
            }
         }
         return false;
      }

      /**
       * Prints the firmware info string
       */
      void print_firmware_info_string()
      {
         uint8_t chr, length;
         write_data(CMD_GET_FIRMWARE_INFO_STRING);
         if(!read_data(&length)) return;
         for(int i=0; i<length; i++)
         {
            if(!read_data(&chr)) return;
            Serial.write(chr);
         }
         Serial.println();
      }
         
      /**
       * Starts the device
       * \return true on success
       */
      bool start_device()
      {
         uint8_t rsp;
         write_data(CMD_START_DEVICE);
         write_data(0x00);  //reserved for future use
         write_data(0x00);  //reserved for future use
         return (read_data(&rsp) && rsp == ACK);
      }

      /**
       * Stops the device
       * \return true on success
       */
      bool stop_device()
      {
         uint8_t rsp;
         write_data(CMD_STOP_DEVICE);
         return (read_data(&rsp) && rsp == ACK);
      }

      /**
       * Sets the device's name
       * \return true on success
       */
      bool set_device_name(char* name)
      {
         uint8_t rsp;
         int length = strlen(name);
         write_data(CMD_SET_NAME);
         write_data(length);
         for(int i=0; i<length; i++)
         {
            write_data(name[i]);
         }
         return (read_data(&rsp) && rsp == ACK);
      }

      /**
       * Sets the advertisement data
       * \return true on success
       */
      bool set_advertisement_data(uint8_t length, uint8_t* adv_data)
      {
         uint8_t rsp;
         write_data(CMD_SET_ADVERTISEMENT_DATA);
         write_data(length);
         for(int i=0; i<length; i++)
         {
            write_data(adv_data[i]);
         }
         return (read_data(&rsp) && rsp == ACK);
      }


   };
}

#undef RX_PIN 2
#undef TX_PIN 3
#undef BAUD_RATE           9600
#undef READ_TIMEOUT_MILLIS 500
#undef ACK                 0x40
#undef NCK                 0x23
#undef DEVICE_FOUND        0x02
#undef CMD_TEST                     0x01
#undef CMD_SET_ADVERTISEMENT_DATA   0x02
#undef CMD_SET_NAME                 0x03
#undef CMD_GET_FIRMWARE_INFO_STRING 0x04
#undef CMD_START_DEVICE             0x05
#undef CMD_STOP_DEVICE              0x06


#endif
#endif

