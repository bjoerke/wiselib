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

#ifndef ARDUINO_IBEACON_RADIO_H
#define ARDUINO_IBEACON_RADIO_H

#if ARDUINO_USE_IBEACON

#include "arduino_types.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <string.h>

//---------------------------------------------------------------------------------
// connection; change if you want to use other pins
#define RX_PIN 2
#define TX_PIN 3
//---------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// ibeacon properties
#define NAME "wiseBeacon"  //12 characters max
#define UUID "10000000-00C0FFEE-DEADBEEF-00000001"
#define MAJOR "0003"
#define MINOR "0023"
//--------------------------------------------------------------------------------

#define TIMEOUT_MILLIS 500  //millis to wait between send and receive actions
#define FIRMWARE_VERSION "HMSoft V526"

#define   BAUD_9600 9600
#define  BAUD_19200 19200
#define  BAUD_38400 38400
#define  BAUD_57600 57600
#define BAUD_115200 115200   //this baud rate seems to be too high for SoftwareSerial (tested with ArduinoUno)


namespace wiselib
{

   /**
    * depricated!!!! (assumes old firmware!)
    * Implements the RadioModel for use with iBeacons
    * IMPORTANT: There are no dedicated nodes in the network
    *            You can just broadcast to all nodes (which means that the device acts as iBeacon
    *            and advertises its UUID,Major#,Minor#)
    */
   template<typename OsModel_P>
   class ArduinoIBeaconRadio
   {
   public:

      typedef OsModel_P OsModel;

      typedef uint8_t  node_id_t;   //TODO: maybe it is better to change this to char* later for MAC adresses
      typedef uint8_t  block_data_t;
      typedef uint8_t  size_t;
      typedef uint8_t  message_id_t;

      enum ErrorCodes
      {
         SUCCESS = OsModel::SUCCESS,
         ERR_UNSPEC = OsModel::ERR_UNSPEC,
         ERR_NOTIMPL = OsModel::ERR_NOTIMPL
      };
      enum SpecialNodeIds
      {
         NULL_NODE_ID           = 0,
         BROADCAST_ADDRESS      = 1,
      };
      enum Restrictions
      {
         MAX_MESSAGE_LENGTH = 32 ///< Maximal number of bytes in payload
      };

      /**
       * (Constructor)
       */
      ArduinoIBeaconRadio(): chip_(RX_PIN, TX_PIN)
      {
         if(setup())
            Serial.println("BLE setup done");
         else
             Serial.print("BLE cannot be initialized.\nHave you set up the jumpers correctly?\n");
      }

      /**
       * (Destructor)
       * Disables iBeacon broadcasting
       */
      ~ArduinoIBeaconRadio()
      {
         disable_radio();
      }

      /**
       * Enables radio so that data can be send and received
       * NOTE: broadcasting of ibeacon advertisment is started in send()
       */
      int enable_radio()
      {
         ;
      }

      /**
       * Disables radio including advertisement of ibeacon data
       */
      int disable_radio()
      {
         return send_command_and_check("AT+IBEA0", "OK+Set:0") ? SUCCESS : ERR_UNSPEC; 
      }

      /**
       * Returns the nodes id
       */
      node_id_t id()
      {
         return NULL_NODE_ID;  //since only broadcasting is supported, this is no problem
      }

      int send(node_id_t id, size_t len, block_data_t* data )
      {
         if(id != BROADCAST_ADDRESS)
         {
            Serial.println("iBeacons only support broadcasting (=advertising)");
            return ERR_NOTIMPL;
         }
         send_command_and_check("AT+ROLE0", "OK+Set:0", 3);  //set role to peripheral
         send_command_and_check("AT+IBEA1", "OK+Set:1", 3);  //enable iBeacon advertising
      }

   private:
      SoftwareSerial chip_;

      /**
       * Sends data to BLE shield
       * \param cmd command to send
       * \return String containing the chip's answer or NULL on timeout
       */
      char* send_command(char* cmd)
      {
         static char response_[255];  //TODO how long is the maximum response string?
         while(chip_.available()>0) chip_.read();  //empty serial
         // send comand
         chip_.write(cmd);

         // wait until response arrives or timeout occurs
         int timeout = TIMEOUT_MILLIS/20;
         while(timeout>0 && chip_.available()==0)
         {
            delay(20);
            timeout--;
         }
         if(timeout==0) return NULL;

         // read response
         int i=0;
         while(chip_.available()>0)
         {
            response_[i]=chip_.read();
            i++;
            if(!chip_.available() > 0) delay(2);  // wait a bit if chip is not fast enough
         }
         response_[i]='\0';
         return response_;
      }

      /**
       * Sends a command and checks if reponse is as expected
       * \param command command to send
       * \param expected_response expected response
       * \param num_tries number of times to try this comand (default is to send this command just once)
       * \return true if response arrives and is as expected
       */
      bool send_command_and_check(char* command, char* expected_response, int num_tries = 1)
      {
         for(int i=0; i<num_tries; i++)
         {
            char* resp = send_command(command);
            if (resp != NULL && strcmp(resp, expected_response) == 0) return true;
         }
         return false;
      }

      /**
       * Sends test commands ("AT") to shield
       * \param max_tries maximum number of tries
       * \return true if returns "OK"
       */
      bool test(int max_tries)
      {
         for(int cur_try=0; cur_try<max_tries; cur_try++)
         {
            if(send_command_and_check("AT", "OK")) return true;
         }
         return false;  //took more tries than allowed
      }

      /**
       * Sets the shield's baud rate
       * \param baud one of the BAUD_ constants (default is BAUD_9600)
       * \return true if done successfull
       */
      bool set_baud(int baud)
      {
         char baud_char;
         switch(baud)
         {
            case   9600: baud_char='0'; break;
            case  19200: baud_char='1'; break;
            case  38400: baud_char='2'; break;
            case  57600: baud_char='3'; break;
            case 115200: baud_char='4'; break;  //does not seem to work
            default: return false;
         }
         char cmd[] = "AT+BAUD%";
         char expected_resp[] = "OK+Set:%";
         cmd[7] = baud_char;  //replace % with the baud rate character
         expected_resp[7] = baud_char;
         char* resp = send_command(cmd);
         if(resp != NULL && strcmp(resp, expected_resp) == 0)
         {
            chip_.begin(baud);
         }
      }

      /**
       * Queries help information
       * \return help info
       */
      const char* get_help_info()
      {
         char* resp = send_command("AT+HELP?");
         return (resp == NULL) ? "n/a" : resp;
      }

      /**
       * Queries software version
       * \return version
       */
      const char* get_version()
      {
         char* resp = send_command("AT+VER??");
         if(resp == NULL)
         {
            resp = send_command("AT+VERS");
         }
         return (resp == NULL) ? "n/a" : resp;
      }

      /**
       * Resets the shield to default settings:
       * - baud = BAUD_9600, parity = NONE, stop_bit = ONE
       * - mode = TRANSMISSION, work_type = IMMEDIATELY, notify = OFF
       * - module_name = "HMSoft", pin_code = "000000", bond_mode = NO_PIN
       * \return true if done successfully
       */
      bool renew()
      {
         return send_command_and_check("AT+RENEW", "OK+RENEW");
      }

      /**
       * Sets the bluetooth module's name
       * \param name name to set (not more than 12 characters)
       * \return true on success
       */
      bool set_name(char* name)
      {
         char command[] =  "AT+NAME%%%%%%%%%%%%";  //12 characters max
         char response[] = "OK+Set:%%%%%%%%%%%%";
         for(int i=0; i<12; i++)
         {
            command[7+i] = name[i];
            response[7+i] = name[i];
         }
         if(!send_command_and_check(command, response, 2)) return false;
         return true;
      }

      /**
       * Resets the shield and wait until it is ready again
       * \return true if done successfully
       */
      bool reset()
      {
         if(!send_command_and_check("AT+RESET","OK+RESET", 3)) return false;
         delay(10);
         return test(5);
      }

      /**
       * Changes the ibeacon's uuid
       * \param uuid uuid to set (format: #-#-#-# where # is an 8 digit hex number)
       */
      bool set_ibeacon_uuid(const char* uuid)
      {
         char command[]  = "AT+IBE#%%%%%%%%";
         char response[] = "OK+Set:0x%%%%%%%%";
         for(int part=0; part<4; part++)
         {
            command[6] = part + '0';  //replace # with '0', '1', '2' or '3'
            for(int i=0; i<8; i++)  //replace % with part of uuid
            {
               command[i+7]=uuid[i+part*9];
               response[i+9]=uuid[i+part*9];
            }
            if(!send_command_and_check(command, response, 2)) return false;
         }
         return true;
      }

      /**
       * Sets the major ID
       * \param minor String with ID (4 digit hex number)
       * \return true on success
       */
      bool set_ibeacon_major(const char* major)
      {
         char command[] =  "AT+MARJ0x%%%%";
         char response[] = "OK+Set:0x%%%%";
         for(int i=0; i<4; i++)
         {
            command[9+i] = major[i];
            response[9+i] = major[i];
         }
         return send_command_and_check(command, response);
      }

      /**
       * Sets the minor ID
       * \param minor String with ID (4 digit hex number)
       * \return true on success
       */
      bool set_ibeacon_minor(const char* minor)
      {
         char command[] =  "AT+MINO0x%%%%";
         char response[] = "OK+Set:0x%%%%";
         for(int i=0; i<4; i++)
         {
            command[9+i] = minor[i];
            response[9+i] = minor[i];
         }
         return send_command_and_check(command, response);
      }

      /**
       * Init shield
       * \return true on success
       */
      bool setup()
      {
         chip_.begin(9600);
         if(!test(15) || !renew()) return false;
         Serial.print("Help at:"); Serial.println(get_help_info());
         const char* version = get_version();
         Serial.print("Version:"); Serial.println(version);         
         if(strcmp(version, FIRMWARE_VERSION) != 0)
         {
            Serial.print("WARNING: invalid firmware ('"); Serial.print(FIRMWARE_VERSION); Serial.println("' expected)");
            return false;
         }
         if(!send_command_and_check("AT+FILT0", "OK+Set:0")) return false;  //disable filtering
         if(!send_command_and_check("AT+ROLE1", "OK+Set:1")) return false;  //set to central role
         if(!set_name(NAME) || !set_ibeacon_uuid(UUID) || !set_ibeacon_major(MAJOR) || !set_ibeacon_minor(MINOR) ) return false;
         if(!reset()) return false;
         return true;
      }
   };

}

#undef RX_PIN
#undef TX_PIN
#undef NAME
#undef TIMEOUT_MILLIS
#undef FIRMWARE_VERSION
#undef BAUD_9600
#undef BAUD_19200
#undef BAUD_38400
#undef BAUD_57600
#undef BAUD_115200 
#undef UUID
#undef MAJOR
#undef MINOR

#endif
#endif
