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

#include "cone.hpp"

namespace opendlv {
namespace logic {
namespace cfsd18 {
namespace sensation {

Cone::Cone(double x, double y,double z):
  m_x()
, m_y()
, m_z()
, m_hits()
, m_missHit()
, m_isValid()
{
  m_x = x;
  m_y = y;
  m_z = z;
  m_hits = 0;
  m_missHit = 0;
  m_isValid = true;
}

double Cone::getX(){
  return m_x;
}

double Cone::getY(){
  return m_y;
}

double Cone::getZ(){

  return m_z;
}

void Cone::setX(double x){
  std::cout << "new x: " << x << " old x: " << m_x << std::endl;
  m_x = x;
}

void Cone::setY(double y){

  std::cout << "new y: " << y << " old y: " << m_y << std::endl;
  m_y = y;
}
void Cone::setZ(double z){

  m_z = z;
}
void Cone::addHit(){

  m_hits++;
  m_missHit = 0;

}
int Cone::getHits(){

  return m_hits;
}
void Cone::addMiss(){

  m_missHit++;
}

int Cone::getMisses(){

  return m_missHit;
}

bool Cone::isThisMe(double x, double y){

  //double diffX = std::abs(m_x - x);
  //double diffY = std::abs(m_y - y);
  double distance = std::sqrt( (m_x - x)*(m_x - x) + (m_y - y)*(m_y - y) );
  if(distance < 1){return true;}else{return false;}

}

bool Cone::shouldBeInFrame(){

  if(m_hits >= 3 && m_y > 2 && m_missHit < 4 && m_isValid){return true;}else{return false;}
}

bool Cone::shouldBeRemoved(){

  if(m_missHit >= 4 || m_y < 2 ){return true;}else{return false;}

}

void Cone::setValidState(bool state){

  m_isValid = state;

}

bool Cone::isValid(){

  return m_isValid;
}

}
}
}
}