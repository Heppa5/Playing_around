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
#include <rw/models.hpp>
#include <fstream>
#include <iostream>


using namespace std;
using namespace rw;




int main(int argc, char** argv )
{
     rw::models::WorkCell::Ptr workcell_orig_ = rw::loaders::XMLRWLoader::load("../Mini-Picker_orig/Mini-Picker_v2_DHJoints.wc.xml");
     rw::models::WorkCell::Ptr workcell_changed_ = rw::loaders::XMLRWLoader::load("../Mini-Picker_changed/Mini-Picker_v2_DHJoints.wc.xml");

     // f
     auto currentState_orig_ = workcell_orig_->getDefaultState();
     auto device_orig_ = workcell_orig_->findDevice("UR5");

     auto currentState_changed_ = workcell_changed_->getDefaultState();
     auto device_changed_ = workcell_changed_->findDevice("UR5");

     rw::math::Q current_configuration = device_orig_->getQ(currentState_orig_);
     auto joint_limits=device_orig_->getBounds();
 //    cout << joint_limits.first << endl;
 //    cout << joint_limits.second << endl;
     device_orig_->setQ(current_configuration,currentState_orig_);

     auto wTb=device_orig_->worldTbase(currentState_orig_);
     auto bTw = inverse(wTb);

     std::random_device generator;

     ofstream myfile;
     myfile.open ("simulated_data.txt");



 //    rw::math::RPY wRc(30.1*M_PI/180,-1.75*M_PI/180,-10.9345*M_PI/180);
 //    rw::math::Transform3D wTc_changed(rw::math::Vector3D(1.0900163,-1.1597528,0.092517919),wRc.toRotation3D());

     for(int i=0; i<500 ;i++)
     {
         rw::math::Q random(6,0,0,0,0,0,0);
         for(int j=0; j<6 ;j++)
         {
             std::uniform_real_distribution<double> distribution(joint_limits.first[j],joint_limits.second[j]);
             random[j]=distribution(generator);
         }

         // wTmarker with forward kinematics is given from orignal cell
         device_orig_->setQ(random,currentState_orig_);
         auto tcp = workcell_orig_->findFrame("UR5.TCP");
         auto world=workcell_orig_->findFrame("WORLD");
         auto wTtcp=world->fTf(tcp,currentState_orig_);

         // Getting camera data from changed workcell
         device_changed_->setQ(random,currentState_changed_);
         auto marker = workcell_changed_->findFrame("marker_f");
         auto cam = workcell_changed_->findFrame("cam");
         auto cTmarker=cam->fTf(marker,currentState_changed_);

         // check data integrity
         auto world_changed= workcell_changed_->findFrame("WORLD");
         auto wTc=world_changed->fTf(cam,currentState_changed_);
         auto wTmarker_changed=wTc*cTmarker;

         auto marker_orig = workcell_orig_->findFrame("marker_f");
         auto tcpTmarker=tcp->fTf(marker_orig,currentState_orig_);
         auto wTmarker_orig=wTtcp*tcpTmarker;
         rw::math::EAA<double> wRmarker_changed(wTmarker_changed.R());
         rw::math::EAA<double> wRmarker_orig(wTmarker_orig.R());
         cout << wTmarker_changed.P()[0] << " , " << wTmarker_changed.P()[1] << " , " << wTmarker_changed.P()[2] << " , "  << wRmarker_changed[0] << " , " << wRmarker_changed[1] << " , " << wRmarker_changed[2] << endl;
         cout << wTmarker_orig.P()[0] << " , " << wTmarker_orig.P()[1] << " , " << wTmarker_orig.P()[2] << " , "  << wRmarker_orig[0] << " , " << wRmarker_orig[1] << " , " << wRmarker_orig[2] << endl;

         // print data:
         rw::math::EAA<double> cRmarker(cTmarker.R());
         rw::math::EAA<double> wRtcp(wTtcp.R());
         myfile << cTmarker.P()[0] << " , " << cTmarker.P()[1] << " , " << cTmarker.P()[2] << " , "  << cRmarker[0] << " , " << cRmarker[1] << " , " << cRmarker[2] << endl;
         myfile << wTtcp.P()[0] << " , " << wTtcp.P()[1] << " , " << wTtcp.P()[2] << " , "  << wRtcp[0] << " , " << wRtcp[1] << " , " << wRtcp[2] << endl;
 //        auto marker=workcell_orig_->findFrame("marker_f");
 //        auto cam=workcell_orig_->findFrame("cam");
 //        auto cTmarker=cam->fTf(marker,currentState_orig_);
 //        rw::math::Vector3D<double> markerPorego(0,0,0);
 //        auto cPorego=cTmarker*markerPorego;
 //        rw::math::EAA<double> rotation(cTmarker.R());
 //        cout << cPorego[0] << "," << cPorego[1] << "," << cPorego[2] << "," << rotation[0] << "," << rotation[1] << "," << rotation[2] << endl;
 //        cout << random[0] << "," << random[1] << "," << random[2] << "," << random[3] << "," << random[4] << "," << random[5] << endl;
     }
     myfile.close();
     cout << "The original tranforms:"<< endl;
     auto world = workcell_orig_->findFrame("WORLD");
     auto cam = workcell_orig_->findFrame("cam");
     auto wTc=world->fTf(cam,currentState_orig_);

     auto tcp = workcell_orig_->findFrame("UR5.TCP");
     auto marker = workcell_orig_->findFrame("marker_f");
     auto tcpTmarker=tcp->fTf(marker,currentState_orig_);
     cout << wTc << endl;
     cout << tcpTmarker << endl;


     cout << "Changed transforms: " << endl;
     world = workcell_changed_->findFrame("WORLD");
     cam = workcell_changed_->findFrame("cam");
     wTc=world->fTf(cam,currentState_changed_);

     tcp = workcell_changed_->findFrame("UR5.TCP");
     marker = workcell_changed_->findFrame("marker_f");
     tcpTmarker=tcp->fTf(marker,currentState_changed_);
     cout << wTc << endl;
     cout << tcpTmarker << endl;
    

//
//    0.85251993, 0.10858911, -0.50973761, 1.0800163,
//    0.55342996, -0.10745841, 0.82497406, -1.1997528,
//    0.033956207, -0.98651284, -0.15509245, 0.062517919,

//    rw::math::Rotation3D<double> wow(0.85251993, 0.10858911, -0.50973761,0.55342996, -0.10745841, 0.82497406,0.033956207, -0.98651284, -0.15509245);
//    rw::math::RPY<double> hej(wow);
//    cout << hej(0)*180/M_PI << " " << hej(1)*180/M_PI << " " << hej(2)*180/M_PI << endl;

//    auto world=workcell_->findFrame("WORLD");
//    auto cam=workcell_->findFrame("cam");
//   auto base=workcell_->findFrame("UR5.Base");
//
//   cout << base->fTf(cam,currentState) << endl;
//   cout << rw::math::RPY<double>(base->fTf(cam,currentState).R());
//    auto wTc=world->fTf(cam,currentState);
//    wTb=device_->worldTbase(currentState);
//    bTw = inverse(wTb);
//    cout << "W w-ftf" <<world->fTf(cam,currentState) << endl;
////     cout << "W" << endl;
////     cout << "W c-ftf" <<cam->fTf(world,currentState) << endl;
//    cout << "W" <<bTw*wTc << endl;
    
    


//    // Update the transform om "marker_f".
////    rw::kinematics::MovableFrame* marker_frame=(rw::kinematics::MovableFrame*)workcell_->findFrame("marker_f");
//    rw::kinematics::FixedFrame* marker_frame=(rw::kinematics::FixedFrame*)workcell_->findFrame("marker_f");
//    cout << "First: " << marker_frame->getTransform(currentState) << endl;
//    auto hej=marker_frame->getParent(currentState);
//    cout << *hej << endl;
////    rw::math::Vector3D<double> tcpPmarker(req.tcpPmarker[0],req.tcpPmarker[1],req.tcpPmarker[2]);
////    rw::math::EAA<double> tcpRvecmarker(req.tcpRmarker[0],req.tcpRmarker[1],req.tcpRmarker[2]);
//    rw::math::Vector3D<double> tcpPmarker(0.032,-0.112,0.067);
//    rw::math::RPY<double> tcpRvecmarker(M_PI,-M_PI/2,0);
////    marker_frame->setTransform(rw::math::Transform3D<double>(tcpPmarker,tcpRvecmarker.toRotation3D()),currentState);
//    marker_frame->setTransform(rw::math::Transform3D<double>(tcpPmarker,tcpRvecmarker.toRotation3D()));
//    cout << hej->fTf(marker_frame,currentState)<<endl;
//    cout << "Second: " << marker_frame->getTransform(currentState) << endl;
////        rw::kinematics::Frame* tool_frame=(rw::kinematics::Frame*)marker_frame;
//
//
//    rw::invkin::JacobianIKSolver findQ(device_,marker_frame,currentState);
//
//    auto bTtool=(device_->baseTframe(marker_frame,currentState));
//
//
//    rw::math::Vector3D<double> desired_position_world(0.0,0.0,1.0);
//    auto desired_position_base=bTw*desired_position_world;
//
////             auto current_rotation=bTtool.R();
//
////    auto desired_rotation_world_vec=rw::math::EAA<double>(0.0,0.0,0.0);
//    auto desired_rotation_world_vec=rw::math::RPY<double>(0.0,M_PI/2,M_PI/4);
//    auto desired_rotation_world=desired_rotation_world_vec.toRotation3D();
//    auto desired_rotation_base=bTw.R()*desired_rotation_world;
//
//    rw::math::Transform3D<double> desired_transform(desired_position_base,desired_rotation_base);
//
//    auto end_configuration = findQ.solve(desired_transform,currentState);
//    cout << end_configuration[0] << endl;
//    cout << end_configuration.size() << endl;
//    rw::trajectory::QPath path;
//
//    rw::math::EAA<double> wow1(-1.5849442482,-1.91942083836,0.339741319418);
//    rw::math::RPY<double> wow2(wow1.toRotation3D());
//    cout << wow2 << endl;
//
//    // marker 1
////     x: -0.0247044805437
////     y: -0.125604972243
////     z: 0.963011920452
////   orientation:
////     x: -1.5849442482
////     y: -1.91942083836
////     z: 0.339741319418
//
//
//    rw::math::EAA<double> wow3(1.71568334103,-0.845532774925,0.641229689121);
//    rw::math::RPY<double> wow4(wow3.toRotation3D());
//    cout << wow4 << endl;
//
//    // marker 2
////    position:
////     x: -0.0751321911812
////     y: 0.225212737918
////     z: 1.07765138149
////   orientation:
////     x: 1.71568334103
////     y: -0.845532774925
////     z: 0.641229689121
//
//
//
////    if(end_configuration.size() > 0)
////    {
////
////
////        bool valid_path = false;
////        valid_path = planner_->query(current_configuration, end_configuration[0], path);
////        if (!valid_path)
////        {
////            cout << "Desired position in base frame: " << desired_position_base << endl;
////            cout << "Desired rotation (and current): " << desired_rotation_base << endl;
////            ROS_ERROR_STREAM("Could not find a path from '" << current_configuration << "' to '" << end_configuration[0] << "'.");
////            throw std::runtime_error("No valid path found.");
////
////            res.ok=0;
////        }
////        else
////            res.ok=1;

    
    
    
    
}
