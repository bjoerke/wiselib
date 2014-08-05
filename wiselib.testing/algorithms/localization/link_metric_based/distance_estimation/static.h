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

#ifndef __DISTANCE_ESTIMATION_STATIC__
#define __DISTANCE_ESTIMATION_STATIC__

#include <math.h>

namespace wiselib
{

   /** 
    * \brief Only for testing - always returns a one meter distance
    */
   template<typename OsModel_P,
            typename ExtendedRadio_P,
            typename Arithmetic_P>
   class StaticDistanceEstimation
   {

   public:

      typedef typename ExtendedRadio_P::size_t size_t;
      typedef typename ExtendedRadio_P::node_id_t node_id_t;
      typedef typename ExtendedRadio_P::block_data_t block_data_t;
      typedef typename ExtendedRadio_P::ExtendedData ExtendedData;

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
         return pow(10.1,0.2);
      }

   };

}

#endif

