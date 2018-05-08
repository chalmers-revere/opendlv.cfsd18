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

//#include <iostream>
#include <cstdlib>

#include <opendavinci/odcore/data/TimeStamp.h>
#include <opendavinci/odcore/strings/StringToolbox.h>
#include <opendavinci/odcore/wrapper/Eigen.h>

#include "blackbox.hpp"

namespace opendlv {
namespace sim {
namespace cfsd18 {
namespace cognition {

BlackBox::BlackBox(int32_t const &a_argc, char **a_argv) :
  DataTriggeredConferenceClientModule(a_argc, a_argv, "sim-cfsd18-cognition-blackbox")
, m_lastTimeStamp()
, m_coneCollector()
, m_coneMutex()
, m_newFrame()
, m_timeDiffMilliseconds()
, m_lastTypeId()
, m_surfaceId()
, m_net(0)
{
    m_coneCollector = Eigen::MatrixXd::Zero(4,100);
    m_newFrame = true;
    m_timeDiffMilliseconds = 150;
    m_lastTypeId = -1;
    m_surfaceId = rand();
}

BlackBox::~BlackBox()
{
}

void BlackBox::setUp()
{
  char curword[20];
  int id;
  int pop_size = 1;

  //Read in the start Genome
  std::string const filename = "cfsdstartgenes";
  std::string const HOME = "/opt/opendlv.data/";
  std::string infile = HOME + filename;

  std::ifstream iFile(infile,ios::in);
  iFile>>curword;
  iFile>>id;
  cout<<"Reading in Genome id "<<id<<endl;
  NEAT::Genome* start_genome = new NEAT::Genome(id,iFile);
  iFile.close();
  NEAT::Population* pop = new NEAT::Population(start_genome,pop_size);
  pop->verify();

  std::vector<NEAT::Organism*> orgs = pop->organisms;
  NEAT::Organism* org = orgs[0];
  m_net = org->net;
}

void BlackBox::tearDown()
{
}


void BlackBox::nextContainer(odcore::data::Container &a_container)
{
/*
    if (a_container.getDataType() == opendlv::logic::perception::ObjectDirection::ID()) {
        auto coneDirection = a_container.getData<opendlv::logic::perception::ObjectDirection>();
	    uint32_t objectId = coneDirection.getObjectId();

        CheckContainer(objectId);
        m_coneCollector(0,objectId) = coneDirection.getAzimuthAngle();
	    m_coneCollector(1,objectId) = coneDirection.getZenithAngle();
	    coneNum++;

	}

	if(a_container.getDataType() == opendlv::logic::perception::ObjectDistance::ID()){
        auto coneDistance = a_container.getData<opendlv::logic::perception::ObjectDistance>();
        uint32_t objectId = coneDistance.getObjectId();

        //CheckContainer(objectId);
        m_coneCollector(2,objectId) = coneDistance.getDistance();
	    m_coneCollector(3,objectId) = 0;
	}
*/

if(a_container.getDataType() == opendlv::logic::perception::Object::ID()){
    std::cout << "RECIEVED AN OBJECT!" << std::endl;
    m_lastTimeStamp = a_container.getSampleTimeStamp();
    auto coneObject = a_container.getData<opendlv::logic::perception::Object>();
    int conesInFrame = coneObject.getObjectId();

    //std::cout << "FRAME: " << m_newFrame << std::endl;
    //Check last timestamp if they are from same message
    //std::cout << "Message Recieved " << std::endl;
    if (m_newFrame){
       m_newFrame = false;
       std::thread coneCollector(&BlackBox::initializeCollection, this, conesInFrame);
       coneCollector.detach();
       //initializeCollection();
    }

  }


if (a_container.getDataType() == opendlv::logic::perception::ObjectDirection::ID()) {
    //std::cout << "Recieved Direction" << std::endl;
    //Retrive data and timestamp
    m_lastTimeStamp = a_container.getSampleTimeStamp();
    auto coneDirection = a_container.getData<opendlv::logic::perception::ObjectDirection>();
    uint32_t objectId = coneDirection.getObjectId();
  //bool newFrameDir = false;
    {
      odcore::base::Lock lockCone(m_coneMutex);
      //std::cout << "Message Recieved " << std::endl;
      m_coneCollector(0,objectId) = coneDirection.getAzimuthAngle();
      m_coneCollector(1,objectId) = coneDirection.getZenithAngle();

	//std::cout << "FRAME BEFORE LOCAL: " << m_newFrame << std::endl;
    //newFrameDir = m_newFrame;
    }

	//std::cout << "FRAME: " << m_newFrame << std::endl;
   /* if (newFrameDir){

      std::thread coneCollector (&DetectConeLane::initializeCollection,this);
      coneCollector.detach();

    }
   */
}

else if(a_container.getDataType() == opendlv::logic::perception::ObjectDistance::ID()){

    m_lastTimeStamp = a_container.getSampleTimeStamp();
    auto coneDistance = a_container.getData<opendlv::logic::perception::ObjectDistance>();
    uint32_t objectId = coneDistance.getObjectId();
  //bool newFrameDist = false;
    {
      odcore::base::Lock lockCone(m_coneMutex);
      m_coneCollector(2,objectId) = coneDistance.getDistance();

	//std::cout << "FRAME BEFORE LOCAL: " << m_newFrame << std::endl;
    //newFrameDist = m_newFrame;
    }

    //std::cout << "FRAME: " << m_newFrame << std::endl;
    //Check last timestamp if they are from same message
    //std::cout << "Message Recieved " << std::endl;
   /* if (newFrameDist){
       std::thread coneCollector(&DetectConeLane::initializeCollection, this);
       coneCollector.detach();
       //initializeCollection();
    }
   */
  }

  else if(a_container.getDataType() == opendlv::logic::perception::ObjectType::ID()){

    //std::cout << "Recieved Type" << std::endl;
    m_lastTimeStamp = a_container.getSampleTimeStamp();
    auto coneType = a_container.getData<opendlv::logic::perception::ObjectType>();
    int objectId = coneType.getObjectId();
  //bool newFrameType =false;
    {
      odcore::base::Lock lockCone(m_coneMutex);
      m_lastTypeId = (m_lastTypeId<objectId)?(objectId):(m_lastTypeId);
      auto type = coneType.getType();
      m_coneCollector(3,objectId) = type;
    //newFrameType = m_newFrame;
    }

    //std::cout << "FRAME: " << m_newFrame << std::endl;
    //Check last timestamp if they are from same message
    //std::cout << "Message Recieved " << std::endl;
   /* if (newFrameType){
      std::thread coneCollector (&DetectConeLane::initializeCollection,this); //just sleep instead maybe since this is unclear how it works
      coneCollector.detach();
      //initializeCollection();

    }
   */
}

}

void BlackBox::initializeCollection(int conesInFrame){
  //std::this_thread::sleep_for(std::chrono::duration 1s); //std::chrono::milliseconds(m_timeDiffMilliseconds)
  bool sleep = true;
  //auto start = std::chrono::system_clock::now();
  //std::cout << "m_lastTypeId1: " << m_lastTypeId<< std::endl;
  //std::cout << "conesInFrame1: " << conesInFrame<< std::endl;
  while(sleep) // Can probably be rewritten nicer
  {
  /*  auto now = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
    if ( elapsed.count() > m_timeDiffMilliseconds*1000 )
        sleep = false;
  */
    if (m_lastTypeId >= conesInFrame-1)
        sleep = false;
  } // End of while
  //std::cout << "m_lastTypeId2: " << m_lastTypeId<< std::endl;
  //std::cout << "conesInFrame2: " << conesInFrame<< std::endl;
  Eigen::MatrixXd extractedCones;
  int nLeft = 0;
  int nRight = 0;
  int nSmall = 0;
  int nBig = 0;

  {
    odcore::base::Lock lockCone(m_coneMutex);
    extractedCones = m_coneCollector.leftCols(m_lastTypeId+1);
    for (int i = 0; i < extractedCones.cols(); i++) {
      int type = extractedCones(3,i);
      if(type == 1){ nLeft++; }
      else if(type == 2){ nRight++; }
      else if(type == 3){ nSmall++; }
      else if(type == 4){ nBig++; }
      else
      {
        std::cout << "WARNING! Object " << i << " has invalid cone type: " << type << std::endl;
      } // End of else
    } // End of for

    std::cout << "members: " << nLeft << " " << nRight << " " << nSmall << " " << nBig << std::endl;
    m_coneCollector = Eigen::MatrixXd::Zero(4,100);
    m_lastTypeId = -1;
    m_newFrame = true;
  }

  //Initialize for next collection
  //std::cout << "Collection done " << extractedCones.rows() << " " << extractedCones.cols() << std::endl;
//std::cout << "extractedCones: " << extractedCones.transpose() << std::endl;
  if(extractedCones.cols() > 0){
    //std::cout << "Extracted Cones " << std::endl;
    //std::cout << extractedCones << std::endl;

    BlackBox::sortIntoSideArrays(extractedCones, nLeft, nRight, nSmall, nBig);
  } // End of if
} // End of initializeCollection


void BlackBox::sortIntoSideArrays(MatrixXd extractedCones, int nLeft, int nRight, int nSmall, int nBig)
{
  int coneNum = extractedCones.cols();
  //Convert to cartesian
  Eigen::MatrixXd cone;
  Eigen::MatrixXd coneLocal = Eigen::MatrixXd::Zero(2,coneNum);

  for(int p = 0; p < coneNum; p++)
  {
    cone = BlackBox::Spherical2Cartesian(extractedCones(0,p), extractedCones(1,p), extractedCones(2,p));
    coneLocal.col(p) = cone;
  } // End of for
std::cout << "ConeLocal: " << coneLocal.transpose() << std::endl;

  Eigen::MatrixXd coneLeft = Eigen::MatrixXd::Zero(2,nLeft);
  Eigen::MatrixXd coneRight = Eigen::MatrixXd::Zero(2,nRight);
  Eigen::MatrixXd coneSmall = Eigen::MatrixXd::Zero(2,nSmall);
  Eigen::MatrixXd coneBig = Eigen::MatrixXd::Zero(2,nBig);
  int a = 0;
  int b = 0;
  int c = 0;
  int d = 0;
  int type;

  for(int k = 0; k < coneNum; k++){
    type = static_cast<int>(extractedCones(3,k));
    if(type == 1)
    {
      coneLeft.col(a) = coneLocal.col(k);
      a++;
    }
    else if(type == 2)
    {
      coneRight.col(b) = coneLocal.col(k);
      b++;
    }
    else if(type == 3)
    {
      coneSmall.col(c) = coneLocal.col(k);
      c++;
    }
    else if(type == 4)
    {
      coneBig.col(d) = coneLocal.col(k);
      d++;
    } // End of else
  } // End of for


  ArrayXXf location(1,2);
  location << -3,0;

  MatrixXf coneLeft_f = coneLeft.cast <float> ();
  MatrixXf coneRight_f = coneRight.cast <float> ();
  ArrayXXf sideLeft = coneLeft_f.transpose().array();
  ArrayXXf sideRight = coneRight_f.transpose().array();

  BlackBox::generateSurfaces(sideLeft, sideRight, location);
} // End of sortIntoSideArrays


void BlackBox::generateSurfaces(ArrayXXf sideLeft, ArrayXXf sideRight, ArrayXXf location){
  
std::cout << "sideLeft: " << sideLeft.rows() << std::endl;
std::cout << "sideRight: " << sideRight.rows() << std::endl;
std::cout << "location: " << location.cols() << std::endl;
  
  std::vector<float> in (24);  //Input loading array
  double out1;
  double out2;
  vector<NEAT::NNode*>::iterator out_iter;

  /*-- setup the input layer based on the four iputs --*/
  //setup_input(net,x,x_dot,theta,theta_dot);
  // TODO: Insert input values
  in[0]=1;  //Bias
  in[1]=1;
  in[2]=1;
  in[3]=1;
  in[4]=1;
  in[5]=1;
  in[6]=1;
  in[7]=1;
  in[8]=1;
  in[9]=1;
  in[10]=1;
  in[11]=1;
  in[12]=1;
  in[13]=1;
  in[14]=1;
  in[15]=1;
  in[16]=1;
  in[17]=1;
  in[18]=1;
  in[19]=1;
  in[20]=1;
  in[21]=1;
  in[22]=1;
  in[23]=1;
  m_net->load_sensors(in);

  //activate_net(net);   /*-- activate the network based on the input --*/
  //Activate the net
  //If it loops, exit returning only fitness of 1 step
  if(m_net->activate())
  {
    std::cout << "NET ACTIVATED" << std::endl;
    out_iter=m_net->outputs.begin();
    out1=(*out_iter)->activation;
    ++out_iter;
    out2=(*out_iter)->activation;
  }
  std::cout << "OUTS: " << out1 << " and " << out2 << std::endl;
  // TODO: Send kinematicstate message

} // End of generateSurfaces


// copy from perception-detectcone
Eigen::MatrixXd BlackBox::Spherical2Cartesian(double azimuth, double zenimuth, double distance)
{
  double xData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*sin(azimuth * static_cast<double>(DEG2RAD));
  double yData = distance * cos(zenimuth * static_cast<double>(DEG2RAD))*cos(azimuth * static_cast<double>(DEG2RAD));
  Eigen::MatrixXd recievedPoint = MatrixXd::Zero(2,1);
  recievedPoint << xData,
                   yData;
  return recievedPoint;
} // End of Spherical2Cartesian





}
}
}
}
