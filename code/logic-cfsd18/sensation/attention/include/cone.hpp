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

#ifndef CONE_HPP
#define CONE_HPP

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace sensation {

class Cone{
  public:
    Cone(double x, double y, double z);
    ~Cone() = default;
    



    double getX();
    double getY();
    double getZ();
    void setX(double x);
    void setY(double y);
    void setZ(double z);
    void addHit();
    int getHits();
    void addMiss();
    int getMisses();
    bool isThisMe(double x, double y);
    bool shouldBeInFrame();
    bool shouldBeRemoved();
    void setValidState(bool state);
    bool isValid();

  private:
    double m_x;
    double m_y;
    double m_z;
    int m_hits;
    int m_missHit;
    bool m_isValid;
    const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;

};

}
}
}
}
#endif

