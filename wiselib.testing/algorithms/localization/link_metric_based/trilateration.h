/***************************************************************************
 ** This file is part of the generic algorithm library Wiselib->           **
 ** Copyright (C) 2008,2009 by the Wisebed (www->wisebed->eu) project->      **
 **                                                                       **
 ** The Wiselib is free software: you can redistribute it and/or modify   **
 ** it under the terms of the GNU Lesser General Public License as        **
 ** published by the Free Software Foundation, either version 3 of the    **
 ** License, or (at your option) any later version->                       **
 **                                                                       **
 ** The Wiselib is distributed in the hope that it will be useful,        **
 ** but WITHOUT ANY WARRANTY; without even the implied warranty of        **
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE->  See the         **
 ** GNU Lesser General Public License for more details->                   **
 **                                                                       **
 ** You should have received a copy of the GNU Lesser General Public      **
 ** License along with the Wiselib->                                       **
 ** If not, see <http://www->gnu->org/licenses/>->                           **
 ***************************************************************************/

#ifndef __ALGORITHMS_LOCALIZATION_LINK_METRIC_BASED_TRILATERATION_H
#define __ALGORITHMS_LOCALIZATION_LINK_METRIC_BASED_TRILATERATION_H

#include "algorithms/localization/link_metric_based/coordinate3d.h"
#define Q(x) (x*x)

namespace wiselib
{

   template<typename Arithmetic>
   Arithmetic det(
      Arithmetic a, Arithmetic b, Arithmetic c,
      Arithmetic d, Arithmetic e, Arithmetic f,
      Arithmetic g, Arithmetic h, Arithmetic i)
   {
      return (a*e*i)+(b*f*g)+(c*d*h)-(g*e*c)-(h*f*a)-(i*d*b);
   }

   template<typename Arithmetic>
   void trilateration3d(coordinate3d<Arithmetic>** anchors, Arithmetic* distances, coordinate3d<Arithmetic>* outpos)
   {
      Arithmetic alpha = (Q(distances[0])-Q(distances[1])) -
                         (Q(anchors[0]->x)-Q(anchors[1]->x)) -
                         (Q(anchors[0]->y)-Q(anchors[1]->y)) -
                         (Q(anchors[0]->z)-Q(anchors[1]->z));
      Arithmetic beta  = (Q(distances[0])-Q(distances[2])) -
                         (Q(anchors[0]->x)-Q(anchors[2]->x)) -
                         (Q(anchors[0]->y)-Q(anchors[2]->y)) -
                         (Q(anchors[0]->z)-Q(anchors[2]->z));
      Arithmetic gamma = (Q(distances[0])-Q(distances[3])) -
                         (Q(anchors[0]->x)-Q(anchors[3]->x)) -
                         (Q(anchors[0]->y)-Q(anchors[3]->y)) -
                         (Q(anchors[0]->z)-Q(anchors[3]->z));

      Arithmetic x10 = anchors[1]->x-anchors[0]->x;
      Arithmetic x20 = anchors[2]->x-anchors[0]->x;
      Arithmetic x30 = anchors[3]->x-anchors[0]->x;

      Arithmetic y10 = anchors[1]->y-anchors[0]->y;
      Arithmetic y20 = anchors[2]->y-anchors[0]->y;
      Arithmetic y30 = anchors[3]->y-anchors[0]->y;

      Arithmetic z10 = anchors[1]->z-anchors[0]->z;
      Arithmetic z20 = anchors[2]->z-anchors[0]->z;
      Arithmetic z30 = anchors[3]->z-anchors[0]->z;

      Arithmetic divisor = 8*det<Arithmetic>(
        x10, y10, z10,
        x20, y20, z20,
        x30, y30, z30);

      outpos->x = det(
      alpha, 2*y10, 2*z10,
      beta,  2*y20, 2*z20,
      gamma, 2*y30, 2*z30) /divisor;

      outpos->y = det(
      2*x10, alpha, 2*z10,
      2*x20, beta,  2*z20,
      2*x30, gamma, 2*z30) /divisor;

      outpos->z = det(
      2*x10, 2*y10, alpha,
      2*x20, 2*y20, beta,
      2*x30, 2*y30, gamma) /divisor;
   }

}

#undef Q
#endif
