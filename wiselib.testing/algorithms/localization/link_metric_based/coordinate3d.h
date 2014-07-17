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

#ifndef __ALGORITHMS_LOCALIZATION_LINK_METRIC_BASED_COORD3D_H
#define __ALGORITHMS_LOCALIZATION_LINK_METRIC_BASED_COORD3D_H

namespace wiselib
{
   template<typename Arithmetic>
   class coordinate3d
   {
      public:
         Arithmetic x, y, z;
         coordinate3d() : x(0), y(0), z(0) {}
         coordinate3d(Arithmetic x, Arithmetic y, Arithmetic z): x(x), y(y), z(z) {}

   };

}

#endif
