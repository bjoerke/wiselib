/***************************************************************************
 ** This file is part of the generic algorithm library Wiselib.           **
 ** Copyright (C) 2008,2009 by the Wisebed (www.wisebed.eu) project.      **
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

#ifndef __DISTANCE_ESTIMATION_IBEACON__
#define __DISTANCE_ESTIMATION_IBEACON__

#include <math.h>

namespace wiselib
{

   /** 
    * \brief 
    * \tparam
    * \ingroup
    */
   template<typename OsModel_P,
            typename ExtendedRadio_P,
            typename Arithmetic_P = double>
   class IBeaconDistanceEstimation
   {

   public:

      typedef typename ExtendedRadio_P::size_t size_t;
      typedef typename ExtendedRadio_P::node_id_t node_id_t;
      typedef typename ExtendedRadio_P::block_data_t block_data_t;
      typedef typename ExtendedRadio_P::ExtendedData ExtendedData;

      typedef struct ibeacon_advertising_data
      {
         uint8_t header[9];
         uint8_t uuid[16];
         uint8_t major_hi;
         uint8_t major_lo;
         uint8_t minor_hi;
         uint8_t minor_lo;
         uint8_t tx_power;
      }__attribute__((packed)) ibeacon_advertising_data_t;

      /**
       * \brief calculates distance to node based on received data
       * \param node_id       node which has sent this data
       * \param len           length of data
       * \param data          the data
       * \param extended_data extended data of node
       * \return estimated distance to node or -1 if it cannot be calculated
       */
      Arithmetic_P estimate_distance(node_id_t node, size_t len, block_data_t* data, const ExtendedData& extended_data)
      {
         if(is_ibeacon_adv_data(len, data))
         {
            ibeacon_advertising_data_t* iad = (ibeacon_advertising_data_t*) data;
            uint16_t major = ( (uint16_t) iad->major_hi) << 8 | iad->major_lo;
            uint16_t minor = ( (uint16_t) iad->minor_hi) << 8 | iad->minor_lo;
            int tx_power = iad->tx_power - 256; //2s complement
            Arithmetic_P rssi = extended_data.link_metric();
            if(rssi == 0) return -1;

            //estimate distance
            Arithmetic_P ratio = rssi / tx_power;
//            if(ratio < 1.0)  return pow(ratio, 10);
//            else             return (0.89976)*pow(ratio,7.7095) + 0.111; 
return ratio*ratio*ratio*ratio*ratio*ratio*ratio*ratio*ratio*ratio;
         }
         else
         {
            return -1;
         }
      }

   private:

      bool is_ibeacon_adv_data(size_t len, uint8_t* data)
      {
          if(len < sizeof(ibeacon_advertising_data_t)) return false;
          return(data[0]==0x02 && data[1]==0x01 && data[2]==0x06 && data[3]==0x1A && data[4]==0xFF &&
                 data[5]==0x4C && data[6]==0x00 && data[7]==0x02 && data[8]==0x15);
      }

   };

}

#endif

