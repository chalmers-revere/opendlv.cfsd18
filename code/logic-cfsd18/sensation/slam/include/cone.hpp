/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef OPENDLV_LOGIC_CFSD18_SENSATION_CONE_HPP
#define OPENDLV_LOGIC_CFSD18_SENSATION_CONE_HPP

#include <iostream>
#include <cmath>
#include <vector>


namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace sensation {

class Cone{
  public:
    Cone(double x, double y,int property,int id);
    ~Cone();
    
    double getX();
    double getY();
    int getProperty();
    int getId();
    
    void setX(double x);
    void setY(double y);
    void setProperty(int property);
    void setId(int id);

    //g2o::pointxy_vertex coneToVertex();
    //void vertexToCone(vertex);
  private:
    double m_x;
    double m_y;
    int m_property;
    int m_id;

};

} //sensation
} //cfsd18
} //logic
} //opendlv

#endif
