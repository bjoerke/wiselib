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

#ifndef __ALGORITHMS_LOCALIZATION_LINK_METRIC_BASED_H
#define __ALGORITHMS_LOCALIZATION_LINK_METRIC_BASED_H

#include "algorithms/localization/distance_based/math/vec.h"
#include "util/delegates/delegate.hpp"
#include "util/pstl/vector_static.h"

namespace wiselib
{

   /** 
    * \brief Calculates the node's position based on link metrics
    *
    * This algorithm calculates the node's position based on link metrics
    * of links to anchor nodes. This is done as follows:
    *  - you have to pass a module which estimates the distance to a node
    *    based on link metric and data received from this node. This module only
    *    needs one method:
    *     Arithmetic_P estimate_distance(node_id_t node, size_t len,
    *            block_data_t* data, const ExtendedData& extended_data);
    *    This method is called by this class on data reception to build a map of
    *    anchornode-distance pairs.
    *  - if at least 3 node-distance pairs are available trilateration is used to
    *    calculate the node's position.
    *  - the position is updated frequently so that this algorithm is suitable for
    *    a moving node.
    *
    * \tparam OsModel_P           Os or plattform which is used
    * \tparam ExtendedRadio_P     Radio which is used to receive data from anchor nodes. Its extended data
    *                             is the link metric to use for distance estimation
    * \tparam DistanceEstimator_P Module which estimates the distance between node and anchor-node
    * \tparam Arithmetic_P        Data type to use for calculations; determines precision
    * \tparam MAX_CALLBACKS       Maximum number of callbacks
    * \tparam Debug_p             Debug output
    *
    * \ingroup basic_algorithm
    * \ingroup position_concept
    */
   template<typename OsModel_P,
            typename ExtendedRadio_P,
            typename DistanceEstimator_P,
            typename Arithmatic_P = double,
            int MAX_CALLBACKS = 10,
            typename Debug_P = typename OsModel_P::Debug>
   class LinkMetricBasedLocalization
   {

   public:

      typedef OsModel_P OsModel;
      typedef ExtendedRadio_P ExtendedRadio;
      typedef Debug_P Debug;
      typedef Arithmatic_P Arithmetic;
      typedef DistanceEstimator_P DistanceEstimator;

      typedef LinkMetricBasedLocalization<OsModel, ExtendedRadio, DistanceEstimator, Arithmetic, MAX_CALLBACKS, Debug> self_type;

      typedef typename ExtendedRadio::size_t size_t;
      typedef typename ExtendedRadio::node_id_t node_id_t;
      typedef typename ExtendedRadio::block_data_t block_data_t;
      typedef typename ExtendedRadio::ExtendedData ExtendedData;

      enum State
      {
         READY,
         NO_VALUE,
         INACTIVE,
         OK,
         FAILED
      };
      enum ErrorCodes
      {
         SUCCESS = OsModel::SUCCESS,
         ERR_UNSPEC = OsModel::ERR_UNSPEC,
         ERR_NOMEM = OsModel::ERR_NOMEM,
         ERR_BUSY = OsModel::ERR_BUSY,
         ERR_NOTIMPL = OsModel::ERR_NOTIMPL,
         ERR_NETDOWN = OsModel::ERR_NETDOWN,
         ERR_HOSTUNREACH = OsModel::ERR_HOSTUNREACH
      };

      //position type
      typedef Vec<Arithmetic> value_t;

      //callback registration
      typedef delegate1<void, int> state_delegate_t;
      typedef vector_static<OsModel, state_delegate_t, MAX_CALLBACKS> CallbackVector;
      typedef typename CallbackVector::iterator CallbackVectorIterator;

      /**
       * Constructor
       */
      LinkMetricBasedLocalization()
      {
         ;
      }

      /**
       * Initializes the algorithm
       */
      int init(ExtendedRadio* radio, Debug* debug)
      {
         radio_ = radio;
         debug_ = debug;
         init();
      }

      /**
       * Reset algorithm.
       * Known position (if so) is lost, state is set back to NO_VALUE. 
       */
      int init()
      {
         idx_ = radio_->template reg_recv_callback<self_type, &self_type::on_receive>(this);
         radio_->enable_radio();
         set_state(NO_VALUE);
      }


      /**
       * Turn off algorithm.
       * State is set to INACTIVE.
       */
      int destruct( void )
      {
         radio_->disable_radio();
         set_state(INACTIVE);
      }

      /**
       * Return current position
       */
      value_t operator() () 
      {
         ;
      }

      /**
       * Return current state - can either be READY, NO_VALUE, or INACTIVE. 
       */
      int state()
      {
         return state_;
      }

      /**
       * Register state changed callback function - method signature must be void RCV_METHOD_NAME(int state).
       * \return callback ID or -1 on error
       */ 
      template<class T, void (T::*TMethod)(int)>
      int register_state_callback(T* obj_pnt)
      {
         if(callbacks_.empty())
         {
            callbacks_.assign( MAX_CALLBACKS, state_delegate_t() );
         }
         for ( unsigned int i = 0; i < callbacks_.size(); ++i )
         {
            if ( callbacks_.at(i) == state_delegate_t() )
            {
               callbacks_.at(i) = state_delegate_t::template from_method<T, TMethod>( obj_pnt );
               return i;
            }
         }
         return -1;
      }

      /**
       * Unregister state-changed callback. 
       */
      int unregister_state_callback(int idx) 	
      {
         callbacks_.at(idx) = state_delegate_t();
         return SUCCESS;
      }


   private:

      ExtendedRadio* radio_;
      Debug* debug_;
      int idx_;
      int state_;
      CallbackVector callbacks_;
      DistanceEstimator distance_estimator_;

      void set_state(int new_state)
      {
         state_ = new_state;
         //notify callbacks
         for( CallbackVectorIterator it = callbacks_.begin();
              it != callbacks_.end();
              it++ )
         {
            if ( *it != state_delegate_t() )
               (*it)(new_state);
         }
      }

      /**
       * Callback on data reception
       */
      void on_receive(node_id_t node, size_t len, block_data_t* data, const ExtendedData& extended_data)
      {
         Arithmetic distance = distance_estimator_.estimate_distance(node, len, data, extended_data);
         if(distance != -1)
            debug_->debug("%f", distance);
      }


   };
}// namespace wiselib
#endif
