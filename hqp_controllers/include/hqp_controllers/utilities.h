#ifndef UTILITIES_H
#define UTILITIES_H

#include <sstream>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Geometry>

namespace hqp_controllers {
//----------------------------------------------------------
inline void printKDLFrame(std::ostringstream& out, const KDL::Frame& frame)
{
  out<<"FRAME:"<<std::endl;
  out<<"Origin: "<<frame.p(0)<<" "<<frame.p(1)<<" "<<frame.p(2)<<std::endl;
  KDL::Vector e1 = frame.M.UnitX();  KDL::Vector e2 = frame.M.UnitY();  KDL::Vector e3 = frame.M.UnitZ();
  Eigen::Matrix3d rot;
  rot<<e1(0), e2(0), e3(0), e1(1), e2(1), e3(1), e1(2), e2(2), e3(2);
  out<<rot<<std::endl;
}
//----------------------------------------------------------
inline void printKDLChain(std::ostringstream& out, const KDL::Chain& chain)
{
    out<<"CHAIN:"<<std::endl;
    out<<"Nr. of joints: "<<chain.getNrOfJoints()<<" | nr. of segments: "<<chain.getNrOfSegments()<<std::endl;
        for(unsigned int i=0; i<chain.getNrOfSegments();i++)
        {
            KDL::Segment seg = chain.getSegment(i);
            KDL::Joint jnt = seg.getJoint();
            out<<"Segment name: "<<seg.getName()<<std::endl;
            out<<"Joint name: "<<jnt.getName()<<std::endl;
            out<<"Joint type: "<<jnt.getTypeName()<<std::endl;
            out<<"Frame to tip: "<<std::endl;
            printKDLFrame(out,seg.getFrameToTip());
            out<<std::endl;
        }
}
//----------------------------------------------------------
}//end namespace hqp_controllers

#endif // UTILITIES_H
