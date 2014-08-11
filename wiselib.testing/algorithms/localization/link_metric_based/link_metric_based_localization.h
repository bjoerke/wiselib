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

#include "algorithms/localization/link_metric_based/coordinate3d.h"
#include "algorithms/localization/link_metric_based/trilateration.h"
#include "util/delegates/delegate.hpp"
#include "util/pstl/vector_static.h"
#include "util/pstl/map_static_vector.h"
#include <util/pstl/pair.h>

//distance values are calculated as follows:
//  dist = dist_new * SMOOTHING_FACTOR + dist_old * (1-SMOOTHING_FACTOR)
#define SMOOTHING_FACTOR 0.9

//a node is deleted if no data has been received since TIME_TO_LIVE seconds
#define TIME_TO_LIVE   10

#define DEBUG

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
    *  - if at least 4 node-distance pairs are available trilateration is used to
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
    * \tparam MAX_ANCHORS         Maximum number of anchors used
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
            int MAX_ANCHORS = 10,
            typename Clock_P = typename OsModel_P::Clock,
            typename Debug_P = typename OsModel_P::Debug>
   class LinkMetricBasedLocalization
   {

   public:

      typedef OsModel_P OsModel;
      typedef ExtendedRadio_P ExtendedRadio;
      typedef Debug_P Debug;
      typedef Arithmatic_P Arithmetic;
      typedef DistanceEstimator_P DistanceEstimator;
      typedef Clock_P Clock; 

      typedef LinkMetricBasedLocalization<OsModel, ExtendedRadio, DistanceEstimator, Arithmetic, MAX_CALLBACKS, MAX_ANCHORS, Clock, Debug> self_type;

      typedef typename ExtendedRadio::size_t size_t;
      typedef typename ExtendedRadio::node_id_t node_id_t;
      typedef typename ExtendedRadio::block_data_t block_data_t;
      typedef typename ExtendedRadio::ExtendedData ExtendedData;

      typedef typename Clock::time_t    time_t;
      typedef typename Clock::seconds_t seconds_t;

      //callback registration
      typedef delegate1<void, int> state_delegate_t;
      typedef vector_static<OsModel, state_delegate_t, MAX_CALLBACKS> CallbackVector;
      typedef typename CallbackVector::iterator CallbackVectorIterator;

      //stores information about neighbor nodes
      typedef struct node_info
      {
         node_id_t  node_id;
         Arithmetic distance;
         seconds_t  last_update_time;
      }node_info_t;
      
      //stores information about anchors
      typedef MapStaticVector<OsModel, node_id_t, coordinate3d<Arithmetic>, MAX_ANCHORS> AnchorsMap;

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
      typedef coordinate3d<Arithmetic> value_t;

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
      int init(ExtendedRadio* radio, Debug* debug, Clock* clock)
      {
         radio_ = radio;
         debug_ = debug;
         clock_ = clock;
         idx_ = -1;
         return init();
      }

      /**
       * Reset algorithm.
       * Known position (if so) is lost, state is set back to NO_VALUE. 
       */
      int init()
      {
         anchors_.clear();
         delete_node_infos();
         if(idx_ == -1)  idx_ = radio_->template reg_recv_callback<self_type, &self_type::on_receive>(this);
         radio_->enable_radio();
         set_state(NO_VALUE);
         return SUCCESS;
      }


      /**
       * Turn off algorithm.
       * State is set to INACTIVE.
       */
      int destruct( void )
      {
         radio_->disable_radio();
         radio_->unreg_recv_callback(idx_); 
         idx_=-1;
         set_state(INACTIVE);
         return SUCCESS;
      }

      /**
       * Return current position
       */
      value_t operator() () 
      {
         return position_;
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

      /**
       * Adds a new anchor node which will be used for localization
       * \param node_id id of anchor
       * \param position the 3d position of this node
       */
      void add_anchor(node_id_t node_id, coordinate3d<Arithmetic> position)
      {
         pair<node_id_t, coordinate3d<Arithmetic> > p(node_id, position);
         if(!anchors_.contains(node_id))
         {
            anchors_.insert(p);
         }
      }


   private:

      CallbackVector callbacks_;
      ExtendedRadio* radio_;
      Debug* debug_;
      Clock* clock_;

      int idx_;
      int state_;
      DistanceEstimator distance_estimator_;

      value_t position_;

      node_info_t neighbors_[MAX_ANCHORS];
      int num_neighbors_;
      AnchorsMap anchors_;

      /**
       * Changes state and notifies all listeners
       */
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
      void on_receive(node_id_t node_id, size_t len, block_data_t* data, const ExtendedData& extended_data)
      {
         Arithmetic distance = distance_estimator_.estimate_distance(node_id, len, data, extended_data);
         if(distance != -1)
         {
            update_node_infos(node_id, distance);
            update_position();
#ifdef DEBUG
            int rssi = extended_data.link_metric();
            if(sizeof(int)==4)      debug_->debug("Found: %08X%08X @ %.1f (%d)", (uint32_t) (node_id>>32), (uint32_t) (node_id), distance, rssi);  //For Android
            else if(sizeof(int)==2) debug_->debug("Found: %04X%04X%04X%04X @ %d (%d)", (uint16_t) (node_id>>48), (uint16_t) (node_id>>32), (uint16_t) (node_id>>16), (uint16_t) (node_id>>0), (int) (distance*100), rssi); //For Arduino
#endif
         }
      }

      /**
       * Updaates this node's position after a change of node info data
       */
      //TODO: search for 4 nearest neighbors only!
      void update_position()
      {
         coordinate3d<Arithmetic>* anchor_positions[4];
         Arithmetic anchor_distances[4];
         int num_anchors=0;
         for(int i=0; i<MAX_ANCHORS; i++)
         {
            node_id_t node_id = neighbors_[i].node_id;
            if(node_id != ExtendedRadio::NULL_NODE_ID)
            {
               //search for corresponding anchor (if any)
               typename AnchorsMap::iterator it = anchors_.find(node_id);
               if(it != anchors_.end())
               {
                  //corresponding anchor found
                  anchor_positions[num_anchors] = &it->second;
                  anchor_distances[num_anchors] = neighbors_[i].distance;
                  num_anchors++;
               }
               //if enough anchors (i.e. 4) then start trilatertion
               if(num_anchors == 4)
               {
#ifdef DEBUG
                  for(int j=0; j<4; j++) debug_->debug("Anchor %d at %.1f, %.1f, %.1f is %.1f meters away",j, anchor_positions[j]->x, anchor_positions[j]->y, anchor_positions[j]->z, anchor_distances[j]);
#endif
                  trilateration3d<Arithmetic>(anchor_positions, anchor_distances, &position_);
                  set_state(READY);
                  return;
               }
            }
         }
         //not enough anchors found
         return;
      }


      /**
       * Deletes all node infos
       */
      void delete_node_infos()
      {
         for(int i=0; i<MAX_ANCHORS; i++)
         {
            neighbors_[i].node_id = ExtendedRadio::NULL_NODE_ID;
         }
         num_neighbors_ = 0;
      }

      /**
       * Updates node info or adds a new one if it does not exist.
       * Also deletes old nodes.
       * \param node_id id of node to update info for
       * \param distance new estimated distance of this node
       */
      void update_node_infos(node_id_t node_id, Arithmetic distance)
      {
         node_info_t* free_info = NULL;
         bool node_id_found = false;
         seconds_t cur_time = clock_->seconds(clock_->time());

         for(int i=0; i<MAX_ANCHORS; i++)
         {
            //empty entry?
            if(neighbors_[i].node_id == ExtendedRadio::NULL_NODE_ID)
            {
               if(!node_id_found && free_info == NULL)
               {
                  free_info = &neighbors_[i];
               }
            }
            else  //entry is not empty
            {
               if(neighbors_[i].node_id == node_id)
               {
                  neighbors_[i].distance = distance * SMOOTHING_FACTOR +  neighbors_[i].distance * (1-SMOOTHING_FACTOR);
                  neighbors_[i].last_update_time = cur_time;
                  node_id_found = true;
               }
               else
               {
                  //check if entry is too old -> node is out of reach
                  if(cur_time - neighbors_[i].last_update_time > TIME_TO_LIVE)
                  {
                     //delete entry
#ifdef DEBUG
                     node_id_t node_id = neighbors_[i].node_id;
                     if(sizeof(int)==4)      debug_->debug("Deleted: %08X%08X", (uint32_t) (node_id>>32), (uint32_t) (node_id));  //For Android
                     else if(sizeof(int)==2) debug_->debug("Deleted: %04X%04X%04X%04X", (uint16_t) (node_id>>48), (uint16_t) (node_id>>32), (uint16_t) (node_id>>16), (uint16_t) (node_id>>0)); //For Arduino
#endif
                    neighbors_[i].node_id = ExtendedRadio::NULL_NODE_ID;
                    if(!node_id_found)
                    {
                       free_info = &neighbors_[i];
                    }
                    num_neighbors_--;
                 }
              }
            }
         }
         //insert this new info
         if(!node_id_found)
         {
            if(free_info != NULL)
            {
               free_info->node_id = node_id;
               free_info->distance = distance;
               free_info->last_update_time = cur_time;
               num_neighbors_--;
            }
            else
            {
               //To avoid a full list increase the MAX_ANCHORS parameter!
               debug_->debug("List of nearby neighbors is full!");
            }
         }
      }


   };
}// namespace wiselib

#undef SMOOTHING_FACTOR
#ifdef DEBUG
#undef DEBUG
#endif
#undef TIME_TO_LIVE

#endif
