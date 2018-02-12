#include <stdio.h>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/proximity/CollisionDetector.hpp>

#include <rw/invkin/JacobianIKSolver.hpp>

#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Rotation3DVector.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/loaders.hpp>

#include <iostream>


using namespace std;





int main(int argc, char** argv )
{
    rw::models::WorkCell::Ptr workcell_ = rw::loaders::XMLRWLoader::load("../Mini-Picker/Mini-Picker_v2_DHJoints.wc.xml");

    auto currentState = workcell_->getDefaultState();
    auto device_ = workcell_->findDevice("UR5");
    
    
    rw::math::Q current_configuration = device_->getQ(currentState);

    device_->setQ(current_configuration,currentState);

    auto wTb=device_->worldTbase(currentState);
    auto bTw = inverse(wTb);

    // Update the transform om "marker_f".
//    rw::kinematics::MovableFrame* marker_frame=(rw::kinematics::MovableFrame*)workcell_->findFrame("marker_f");
    rw::kinematics::FixedFrame* marker_frame=(rw::kinematics::FixedFrame*)workcell_->findFrame("marker_f");
    cout << "First: " << marker_frame->getTransform(currentState) << endl;
    auto hej=marker_frame->getParent(currentState);
    cout << *hej << endl;
//    rw::math::Vector3D<double> tcpPmarker(req.tcpPmarker[0],req.tcpPmarker[1],req.tcpPmarker[2]);
//    rw::math::EAA<double> tcpRvecmarker(req.tcpRmarker[0],req.tcpRmarker[1],req.tcpRmarker[2]);
    rw::math::Vector3D<double> tcpPmarker(0.032,-0.112,0.067);
    rw::math::RPY<double> tcpRvecmarker(M_PI,-M_PI/2,0);
//    marker_frame->setTransform(rw::math::Transform3D<double>(tcpPmarker,tcpRvecmarker.toRotation3D()),currentState);
    marker_frame->setTransform(rw::math::Transform3D<double>(tcpPmarker,tcpRvecmarker.toRotation3D()));
    cout << hej->fTf(marker_frame,currentState)<<endl;
    cout << "Second: " << marker_frame->getTransform(currentState) << endl;
//        rw::kinematics::Frame* tool_frame=(rw::kinematics::Frame*)marker_frame;


    rw::invkin::JacobianIKSolver findQ(device_,marker_frame,currentState);

    auto bTtool=(device_->baseTframe(marker_frame,currentState));


    rw::math::Vector3D<double> desired_position_world(0.0,0.0,1.0);
    auto desired_position_base=bTw*desired_position_world;

//             auto current_rotation=bTtool.R();

//    auto desired_rotation_world_vec=rw::math::EAA<double>(0.0,0.0,0.0);
    auto desired_rotation_world_vec=rw::math::RPY<double>(0.0,M_PI/2,M_PI/4);
    auto desired_rotation_world=desired_rotation_world_vec.toRotation3D();
    auto desired_rotation_base=bTw.R()*desired_rotation_world;

    rw::math::Transform3D<double> desired_transform(desired_position_base,desired_rotation_base);

    auto end_configuration = findQ.solve(desired_transform,currentState);
    cout << end_configuration[0] << endl;
    cout << end_configuration.size() << endl;
    rw::trajectory::QPath path;

//    if(end_configuration.size() > 0)
//    {
//
//
//        bool valid_path = false;
//        valid_path = planner_->query(current_configuration, end_configuration[0], path);
//        if (!valid_path)
//        {
//            cout << "Desired position in base frame: " << desired_position_base << endl;
//            cout << "Desired rotation (and current): " << desired_rotation_base << endl;
//            ROS_ERROR_STREAM("Could not find a path from '" << current_configuration << "' to '" << end_configuration[0] << "'.");
//            throw std::runtime_error("No valid path found.");
//
//            res.ok=0;
//        }
//        else
//            res.ok=1;

    
    
    
    
}
