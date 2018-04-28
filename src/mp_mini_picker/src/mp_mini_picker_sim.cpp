
#include "mp_mini_picker/currentQ.h"
#include "mp_mini_picker/moveToQ.h"
#include "mp_mini_picker/moveToPointTcp.h"
#include "mp_mini_picker/moveToPoseTcp.h"
#include "mp_mini_picker/moveToPointMarker.h"
#include "mp_mini_picker/moveToPoseMarker.h"
#include "mp_mini_picker/changeTcpTMarker.h"

//#include <caros/serial_device_si_proxy.h>
//#include <caros/common_robwork.h>

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

#include <ros/ros.h>

#include <string>
#include <stdexcept>

#include <std_msgs/String.h>
// #include <message_package/currrentToolPosition.h>
#include <rw/loaders/WorkCellFactory.hpp>
#include "message_package/Q.h"
#include <message_package/currentToolPosition2.h>

#include <geometry_msgs/PoseStamped.h>

using namespace ros;
using namespace std;

class UrTest
{
    public:
    ros::Publisher _pub_new_Q;
    ros::Subscriber _sub_current_Q;
        UrTest()
        {
            initWorkCell();
            initDevice();
            initPathPlannerWithCollisionDetector();
            
            
//             robot_move = nodehandle_.advertise<message_package::currrentToolPosition>("/robot/moved", 1);

            robot_move = nodehandle_.advertise<message_package::currentToolPosition2>("/robot/moved", 1);
            robot_move_marker = nodehandle_.advertise<message_package::currentToolPosition2>("/robot/moved_marker", 1);
            _pub_new_Q = nodehandle_.advertise<message_package::Q>("/robot_sim/new_Q",10);

            cout << "Number of subscribers to new Q: " << _pub_new_Q.getNumSubscribers() << endl;
            _sub_current_Q = nodehandle_.subscribe("/robot_sim/Q",1,&UrTest::update_Q,this);
            currentQ=rw::math::Q(6,0,0,0,0,0,0);


        }

        virtual ~UrTest()
        {
            /* Empty */
        }
        rw::math::Q currentQ;
        bool currentQ_initialized=false;
        void update_Q(const message_package::Q::ConstPtr msg)
        {
            for(int i=0; i<6 ; i++)
            {
                currentQ[i]=msg->robot_configuration[i];
            }
            if(currentQ_initialized==false)
            {
                cout << currentQ << endl;
            }
            currentQ_initialized=true;
        }
       
        
//         ros::std_msgs::Float64MultiArray get_current_Q()
        bool get_current_Q(mp_mini_picker::currentQ::Request  &req,
                   mp_mini_picker::currentQ::Response &res)
        {
//             ROS_INFO("Got service call for current Q");
            if ( req.getQ == 1) 
            {
                auto current_Q=getCurrentJointConfiguration();
                for(int i = 0 ; i < 6 ; i++) // 6 joints 
                {
                    res.Q[i]=current_Q[i];
                }
            }
        }

        bool move_to_Q(mp_mini_picker::moveToQ::Request  &req,
                   mp_mini_picker::moveToQ::Response &res)
        {
            ROS_INFO("ASKED TO MOVE TO Q");
            rw::math::Q start_configuration = getCurrentJointConfiguration();
            rw::math::Q end_configuration = rw::math::Q(start_configuration.size());
            
            // Fill up desired configurations
            for(int i=0 ; i < 6 ; i++ ) 
                end_configuration[i] = req.Q[i];
            
            rw::trajectory::QPath path;
            
            
            bool valid_path = false;
            ROS_ASSERT(planner_);
            valid_path = planner_->query(start_configuration, end_configuration, path);
            if (!valid_path)
            {
                ROS_ERROR_STREAM("Could not find a path from '" << start_configuration << "' to '" << end_configuration << "'.");
                throw std::runtime_error("No valid path found.");
                
                res.ok=0;
            }
            else
                res.ok=1;
            
            for (int i = 0; i<path.size() ; i++)
            {   
                rw::math::Q p = path[i];
                bool ret = true;
                message_package::Q msg;
                for(int j=0; j<6 ;j++)
                    msg.robot_configuration[j]=p[j];
                _pub_new_Q.publish(msg);
                
                if (!ret)
                {
//                  return_status = false;
                    ROS_ERROR_STREAM("The serial device didn't acknowledge the movePtp command.");
                }
                //ROS_INFO("Should have send a move");
//                 
            }
        }
        
    
    bool change_end_transformation(mp_mini_picker::changeTcpTMarker::Request  &req,
                             mp_mini_picker::changeTcpTMarker::Response &res)
    {
        got_tcpTmarker=true;
        rw::math::Vector3D<double> tcpPmarker(req.tcpPmarker[0],req.tcpPmarker[1],req.tcpPmarker[2]);
        rw::math::EAA<double> tcpRvecmarker(req.tcpRmarker[0],req.tcpRmarker[1],req.tcpRmarker[2]);
        tcpTmarker=rw::math::Transform3D<double>(tcpPmarker,tcpRvecmarker.toRotation3D());
        res.ok=true;
        cout << tcpTmarker << endl;
    }
    
    bool move_to_pose_marker(mp_mini_picker::moveToPoseMarker::Request  &req,
                             mp_mini_picker::moveToPoseMarker::Response &res)
    {
        rw::math::Q current_configuration = getCurrentJointConfiguration();

        device_->setQ(current_configuration,currentState);

        auto wTb=device_->worldTbase(currentState);
        auto bTw = inverse(wTb);

        // Update the transform om "marker_f".
        rw::kinematics::FixedFrame* marker_frame=(rw::kinematics::FixedFrame*)workcell_->findFrame("marker_f");
//         cout << "First: " << marker_frame->getTransform(currentState) << endl;
        rw::math::Vector3D<double> tcpPmarker(req.tcpPmarker[0],req.tcpPmarker[1],req.tcpPmarker[2]);
        rw::math::EAA<double> tcpRvecmarker(req.tcpRmarker[0],req.tcpRmarker[1],req.tcpRmarker[2]);
        marker_frame->setTransform(rw::math::Transform3D<double>(tcpPmarker,tcpRvecmarker.toRotation3D()));
//         cout << "Second: " << marker_frame->getTransform(currentState) << endl;
//        rw::kinematics::Frame* tool_frame=(rw::kinematics::Frame*)marker_frame;


        rw::invkin::JacobianIKSolver findQ(device_,marker_frame,currentState);

        auto bTtool=(device_->baseTframe(marker_frame,currentState));


        rw::math::Vector3D<double> desired_position_world(req.pose[0],req.pose[1],req.pose[2]);
        auto desired_position_base=bTw*desired_position_world;

//             auto current_rotation=bTtool.R();

        auto desired_rotation_world_vec=rw::math::EAA<double>(req.pose[3],req.pose[4],req.pose[5]);
        auto desired_rotation_world=desired_rotation_world_vec.toRotation3D();
        rw::math::RPY<double> test(desired_rotation_world);
//         cout << "Desired rotation in world: " <<test << endl;
        auto desired_rotation_base=bTw.R()*desired_rotation_world;

        rw::math::Transform3D<double> desired_transform(desired_position_base,desired_rotation_base);

        auto end_configuration = findQ.solve(desired_transform,currentState);
        rw::trajectory::QPath path;

        if(end_configuration.size() > 0)
        {
            res.ok=end_configuration.size();

            bool valid_path = false;
            ROS_ASSERT(planner_);
            valid_path = planner_->query(current_configuration, end_configuration[0], path);
            if (!valid_path)
            {
                cout << "Desired position in base frame: " << desired_position_base << endl;
                cout << "Desired rotation (and current): " << desired_rotation_base << endl;
                ROS_ERROR_STREAM("Could not find a path from '" << current_configuration << "' to '" << end_configuration[0] << "'.");
                throw std::runtime_error("No valid path found.");

                res.ok=2;
            }
            else
                res.ok=1;

            for (int i = 0; i<path.size() ; i++)
            {
                rw::math::Q p = path[i];
                bool ret = true;

                message_package::Q msg;
                for(int j=0; j<6 ;j++)
                    msg.robot_configuration[j]=p[j];
                _pub_new_Q.publish(msg);

                if (!ret)
                {
                    ROS_ERROR_STREAM("The serial device didn't acknowledge the movePtp command.");
                }
            }
        }
        else
        {
            ROS_INFO("No solution to JacobianIKSolver");
            res.ok = 3;
        }
    }

    bool move_to_point_marker(mp_mini_picker::moveToPointMarker::Request  &req,
                              mp_mini_picker::moveToPointMarker::Response &res)
    {
        rw::math::Q current_configuration = getCurrentJointConfiguration();


//             rw::kinematics::State state = workcell_->getDefaultState();

        device_->setQ(current_configuration,currentState);


        auto wTb=device_->worldTbase(currentState);
        auto bTw = inverse(wTb);

        // Update the transform om "marker_f".
        rw::kinematics::FixedFrame* marker_frame=(rw::kinematics::FixedFrame*)workcell_->findFrame("marker_f");
        cout << "First: " << marker_frame->getTransform(currentState) << endl;
        rw::math::Vector3D<double> tcpPmarker(req.tcpPmarker[0],req.tcpPmarker[1],req.tcpPmarker[2]);
        rw::math::EAA<double> tcpRvecmarker(req.tcpRmarker[0],req.tcpRmarker[1],req.tcpRmarker[2]);
        marker_frame->setTransform(rw::math::Transform3D<double>(tcpPmarker,tcpRvecmarker.toRotation3D()));
        cout << "Second: " << marker_frame->getTransform(currentState) << endl;

        rw::invkin::JacobianIKSolver findQ(device_,marker_frame,currentState);

        auto bTtcp=(device_->baseTframe(marker_frame,currentState));


        rw::math::Vector3D<double> desired_position(req.point[0],req.point[1],req.point[2]);
        auto Wdesired_position=bTw*desired_position;

        auto current_rotation=bTtcp.R();
        rw::math::Transform3D<double> desired_transform(Wdesired_position,current_rotation);



        auto end_configuration = findQ.solve(desired_transform,currentState);


        rw::trajectory::QPath path;

        if(end_configuration.size() > 0)
        {
            res.ok=end_configuration.size();

            bool valid_path = false;
            ROS_ASSERT(planner_);
            valid_path = planner_->query(current_configuration, end_configuration[0], path);
            if (!valid_path)
            {
                cout << "Desired position in base frame: " << Wdesired_position << endl;
                cout << "Desired rotation (and current): " << current_rotation << endl;
                ROS_ERROR_STREAM("Could not find a path from '" << current_configuration << "' to '" << end_configuration[0] << "'.");
                throw std::runtime_error("No valid path found.");

                res.ok=0;
            }
            else
                res.ok=1;

            for (int i = 0; i<path.size() ; i++)
            {
                rw::math::Q p = path[i];
                bool ret = true;

                message_package::Q msg;
                for(int j=0; j<6 ;j++)
                    msg.robot_configuration[j]=p[j];
                _pub_new_Q.publish(msg);

                if (!ret)
                {
                    ROS_ERROR_STREAM("The serial device didn't acknowledge the movePtp command.");
                }
            }
        }
        else
        {
            ROS_INFO("No solution to JacobianIKSolver");
            res.ok = 0;
        }
    }
        
        bool move_to_pose_tcp(mp_mini_picker::moveToPoseTcp::Request  &req,
                   mp_mini_picker::moveToPoseTcp::Response &res)
        {
            cout << "Move to pose TCP" << endl;
            rw::math::Q current_configuration = getCurrentJointConfiguration();

            device_->setQ(current_configuration,currentState);

            auto wTb=device_->worldTbase(currentState);
            auto bTw = inverse(wTb);
            auto tool_frame=workcell_->findFrame("UR5.TCP");
            rw::invkin::JacobianIKSolver findQ(device_,tool_frame,currentState);

            auto bTtool=(device_->baseTframe(tool_frame,currentState));

            
            rw::math::Vector3D<double> desired_position_world(req.pose[0],req.pose[1],req.pose[2]);
            auto desired_position_base=bTw*desired_position_world;
            
//             auto current_rotation=bTtool.R();
            
            auto desired_rotation_world_vec=rw::math::EAA<double>(req.pose[3],req.pose[4],req.pose[5]);
            auto desired_rotation_world=desired_rotation_world_vec.toRotation3D();
            auto desired_rotation_base=bTw.R()*desired_rotation_world;
            
            rw::math::Transform3D<double> desired_transform(desired_position_base,desired_rotation_base);

            auto end_configuration = findQ.solve(desired_transform,currentState);
            
            
            rw::trajectory::QPath path;
            
            if(end_configuration.size() > 0)
            {
                cout << "Found a solution " << endl;
                res.ok=end_configuration.size();

                bool valid_path = false;
                ROS_ASSERT(planner_);
                valid_path = planner_->query(current_configuration, end_configuration[0], path);
                if (!valid_path)
                {
                    cout << "Desired position in base frame: " << desired_position_base << endl;
                    cout << "Desired rotation (and current): " << desired_rotation_base << endl;
                    ROS_ERROR_STREAM("Could not find a path from '" << current_configuration << "' to '" << end_configuration[0] << "'.");
                    throw std::runtime_error("No valid path found.");
                    
                    res.ok=0;
                }
                else
                    res.ok=1;
                
                for (int i = 0; i<path.size() ; i++)
                {
                    rw::math::Q p = path[i];
                    bool ret = true;

                    message_package::Q msg;
                    for(int j=0; j<6 ;j++)
                        msg.robot_configuration[j]=p[j];
                    _pub_new_Q.publish(msg);
                    
                    if (!ret)
                    {
                        ROS_ERROR_STREAM("The serial device didn't acknowledge the movePtp command.");
                    }                
                }
            }
            else
            {
                ROS_INFO("No solution to JacobianIKSolver");
                res.ok = 0;
            }    
        }
        
        bool move_to_point_tcp(mp_mini_picker::moveToPointTcp::Request  &req,
                   mp_mini_picker::moveToPointTcp::Response &res)
        {
            rw::math::Q current_configuration = getCurrentJointConfiguration();
            
            
//             rw::kinematics::State state = workcell_->getDefaultState();
            
            device_->setQ(current_configuration,currentState);
            
            
            auto wTb=device_->worldTbase(currentState);
            auto bTw = inverse(wTb);
            auto tool_frame=workcell_->findFrame("UR5.TCP");
            
            rw::invkin::JacobianIKSolver findQ(device_,tool_frame,currentState);

            auto bTtool=(device_->baseTframe(tool_frame,currentState));

            
            rw::math::Vector3D<double> desired_position(req.point[0],req.point[1],req.point[2]);
            auto Wdesired_position=bTw*desired_position;
            
            auto current_rotation=bTtool.R();
            rw::math::Transform3D<double> desired_transform(Wdesired_position,current_rotation);
            

            
            auto end_configuration = findQ.solve(desired_transform,currentState);
            
            
            rw::trajectory::QPath path;
            
            if(end_configuration.size() > 0)
            {
                res.ok=end_configuration.size();

                bool valid_path = false;
                ROS_ASSERT(planner_);
                valid_path = planner_->query(current_configuration, end_configuration[0], path);
                if (!valid_path)
                {
                    cout << "Desired position in base frame: " << Wdesired_position << endl;
                    cout << "Desired rotation (and current): " << current_rotation << endl;
                    ROS_ERROR_STREAM("Could not find a path from '" << current_configuration << "' to '" << end_configuration[0] << "'.");
                    throw std::runtime_error("No valid path found.");
                    
                    res.ok=0;
                }
                else
                    res.ok=1;
                
                for (int i = 0; i<path.size() ; i++)
                {
                    rw::math::Q p = path[i];
                    bool ret = true;
                    message_package::Q msg;
                    for(int j=0; j<6 ;j++)
                        msg.robot_configuration[j]=p[j];
                    _pub_new_Q.publish(msg);
                    
                    if (!ret)
                    {
                        ROS_ERROR_STREAM("The serial device didn't acknowledge the movePtp command.");
                    }                
                }
            }
            else
            {
                ROS_INFO("No solution to JacobianIKSolver");
                res.ok = 0;
            }    
        }
        
        void publish_position()
        {
//            auto current_configuration=sdsip_.getQ();
            rw::math::Q current_configuration=getCurrentJointConfiguration();
//             rw::kinematics::State state = workcell_->getDefaultState();
            device_->setQ(current_configuration,currentState);
             auto wTb=device_->worldTbase(currentState);
            
//             auto bTe=device_->baseTend(currentState);
//             auto tcp_frame=workcell_->findFrame("UR5.Joint5");
            
//            auto tcp_frame=workcell_->findFrame("marker_f");
            auto tcp_frame=workcell_->findFrame("UR5.TCP");
            auto bTe=device_->baseTframe(tcp_frame,currentState);
            auto bMarker= wTb*bTe*rw::math::Vector3D<double>(0,0,0);
            
            
            auto R_WtoE=(wTb.R())*(bTe.R());

            auto Rvec_WtoE=rw::math::EAA<double>(R_WtoE);
//             auto Rvec_WtoE=rw::math::EAA<double>(R_WtoE);

            // Get Transformation matrix from w to tcp
            auto wTtcp=wTb*bTe;
            auto Rvec_wTtcp=rw::math::EAA<double>(wTtcp.R());
            auto tvec_wTtcp=wTtcp.P();

            message_package::currentToolPosition2 msg;
            msg.wTtcp.pose.position.x= tvec_wTtcp[0];
            msg.wTtcp.pose.position.y= tvec_wTtcp[1];
            msg.wTtcp.pose.position.z= tvec_wTtcp[2];
            msg.wTtcp.pose.orientation.x=Rvec_wTtcp[0];
            msg.wTtcp.pose.orientation.y=Rvec_wTtcp[1];
            msg.wTtcp.pose.orientation.z=Rvec_wTtcp[2];

            msg.tcp.pose.position.x= bMarker[0];
            msg.tcp.pose.position.y= bMarker[1];
            msg.tcp.pose.position.z= bMarker[2];
            msg.tcp.pose.orientation.x=Rvec_WtoE[0];
            msg.tcp.pose.orientation.y=Rvec_WtoE[1];
            msg.tcp.pose.orientation.z=Rvec_WtoE[2];
            msg.tcp.header.stamp = ros::Time::now();
//            geometry_msgs::PoseStamped msg;
//            msg.pose.position.x= bMarker[0];
//            msg.pose.position.y= bMarker[1];
//            msg.pose.position.z= bMarker[2];
//            msg.pose.orientation.x=Rvec_WtoE[0];
//            msg.pose.orientation.y=Rvec_WtoE[1];
//            msg.pose.orientation.z=Rvec_WtoE[2];
//            msg.header.stamp = ros::Time::now();
            
            
            for(int i = 0 ; i < 6 ; i++) // 6 joints 
            {
                msg.Q[i]=current_configuration[i];
            }
//             msg.woopa=10;
            
            robot_move.publish(msg);
            
            

        }
        
        void publish_marker_position()
        {
            if(got_tcpTmarker==true)
            {
                auto current_configuration=getCurrentJointConfiguration();
//                 rw::math::Q current_configuration(6,0,0,0,0,0,0);
    //             rw::kinematics::State state = workcell_->getDefaultState();
                device_->setQ(current_configuration,currentState);
                auto wTb=device_->worldTbase(currentState);
                
    //             auto bTe=device_->baseTend(currentState);
    //             auto tcp_frame=workcell_->findFrame("UR5.Joint5");
                
    //            auto tcp_frame=workcell_->findFrame("marker_f");
                auto tcp_frame=workcell_->findFrame("UR5.TCP");
                auto bTe=device_->baseTframe(tcp_frame,currentState);
                auto bMarker= wTb*bTe*tcpTmarker*rw::math::Vector3D<double>(0,0,0);
                
                
                auto R_WtoE=(wTb.R())*(bTe.R())*(tcpTmarker.R());

                auto Rvec_WtoE=rw::math::EAA<double>(R_WtoE);
    //             auto Rvec_WtoE=rw::math::EAA<double>(R_WtoE);

                // Get Transformation matrix from w to tcp
                auto wTmarker=wTb*bTe*tcpTmarker;
                auto Rvec_wTtcp=rw::math::EAA<double>(wTmarker.R());
                auto tvec_wTtcp=wTmarker.P();

                message_package::currentToolPosition2 msg;
                msg.wTtcp.pose.position.x= tvec_wTtcp[0];
                msg.wTtcp.pose.position.y= tvec_wTtcp[1];
                msg.wTtcp.pose.position.z= tvec_wTtcp[2];
                msg.wTtcp.pose.orientation.x=Rvec_wTtcp[0];
                msg.wTtcp.pose.orientation.y=Rvec_wTtcp[1];
                msg.wTtcp.pose.orientation.z=Rvec_wTtcp[2];

                msg.tcp.pose.position.x= bMarker[0];
                msg.tcp.pose.position.y= bMarker[1];
                msg.tcp.pose.position.z= bMarker[2];
                msg.tcp.pose.orientation.x=Rvec_WtoE[0];
                msg.tcp.pose.orientation.y=Rvec_WtoE[1];
                msg.tcp.pose.orientation.z=Rvec_WtoE[2];
                msg.tcp.header.stamp = ros::Time::now();
    //            geometry_msgs::PoseStamped msg;
    //            msg.pose.position.x= bMarker[0];
    //            msg.pose.position.y= bMarker[1];
    //            msg.pose.position.z= bMarker[2];
    //            msg.pose.orientation.x=Rvec_WtoE[0];
    //            msg.pose.orientation.y=Rvec_WtoE[1];
    //            msg.pose.orientation.z=Rvec_WtoE[2];
    //            msg.header.stamp = ros::Time::now();
                
                
                for(int i = 0 ; i < 6 ; i++) // 6 joints 
                {
                    msg.Q[i]=current_configuration[i];
                }
    //             msg.woopa=10;
                
                robot_move_marker.publish(msg);
            }
            
            

        }
    protected:
        

        
        
        void initWorkCell()
        {

            workcell_ =  rw::loaders::WorkCellLoader::Factory::load("/home/jepod13/Jesper/UR5/Mini-Picker/Mini-Picker_v2_DHJoints.wc.xml");
            if (workcell_ == NULL)
            {
                ROS_ERROR("No workcell was loaded - exiting...");
                throw std::runtime_error("Not able to obtain a workcell.");
            }
            currentState = workcell_->getDefaultState();
        }

        void initDevice()
        {
            std::string device_name="UR5";

            ROS_DEBUG_STREAM("Looking for the device '" << device_name << "' in the workcell.");
            device_ = workcell_->findDevice(device_name);
            if (device_ == NULL)
            {
                ROS_FATAL_STREAM("Unable to find device " << device_name << " in the loaded workcell");
                throw std::runtime_error("Not able to find the device within the workcell.");
            }
        }

        void initPathPlannerWithCollisionDetector()
        {
//             rw::kinematics::State state = workcell_->getDefaultState();
            /* Collision detector */
            auto detector = rw::common::ownedPtr( new rw::proximity::CollisionDetector(workcell_,
                                                                                       rwlibs::proximitystrategies::ProximityStrategyPQP::make()
                                                                                      )
                                                );
            /* PlannerConstraint that uses the collision detector to verify that the _start_ and _end_ configurations are
                * collision free and that the edge(s) between those is/are also collision free. */
            const rw::pathplanning::PlannerConstraint planner_constraint = rw::pathplanning::PlannerConstraint::make(detector,
                                                        device_, 
                                                        currentState
                                                                );

            /* Just using a really simple path planner (straight line in the configuration space) */
            // planner_ = rw::pathplanning::QToQPlanner::make(planner_constraint);
            planner_ = rwlibs::pathplanners::RRTQToQPlanner::makeConnect(planner_constraint,
                                                                         rw::pathplanning::QSampler::makeUniform(device_),
                                                                         rw::math::MetricFactory::makeEuclidean<rw::math::Q>(), 0.1
                                                                        );
        }

        rw::math::Q getCurrentJointConfiguration()
        {
            if(currentQ_initialized==false)
            {
                rw::math::Q current_configuration(6,0,0,0,0,0,0);
                return current_configuration;
            }
            else
            {
                return currentQ;
            }
//            return sdsip_.getQ();
        }

        

    protected:
        ros::NodeHandle nodehandle_;


        rw::models::WorkCell::Ptr workcell_;
        rw::models::Device::Ptr device_;
        rw::pathplanning::QToQPlanner::Ptr planner_;
        
        rw::kinematics::State  currentState;
        
        ros::Publisher robot_move;
        
        ros::Publisher robot_move_marker;
        
        rw::math::Transform3D<double> tcpTmarker;
        bool got_tcpTmarker=false;
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "caros_universalrobot");
    ros::NodeHandle n;
        
    const double q_change = -0.2;

    UrTest ur_test;
    
    rw::math::RPY<double> wow(M_PI,-M_PI/2.0,0.0);
    rw::math::Rotation3D<double> wow2(-0.55818027, 0.13887417, -0.81698585,0.80561882, 0.16013092, -0.56890559,0.067801453, -0.9773857, -0.19605455);
    rw::math::RPY<double> wow3(wow2);
    cout << "Correct rotation for marker vs. tool:\n" << wow.toRotation3D() << endl;
    cout <<"WATHC HERE: " << wow3 << endl;
    
//     cout << rw::math::RPY(rw::math::Rotation3D(0.268154, 0.801153, 0.535021, 0.358975, -0.59846, 0.716228, 0.893997, -0, -0.448074)) << endl;

//    rw::math::Rotation3D<double> hej(-0.73812979,-0.057609037, -0.6721946,0.67215085, -0.14862442, -0.72534406,-0.058118165, -0.98721433, 0.14842606);
//    rw::math::RPY<double> wow(hej);
//    cout << wow << endl;
    
    ros::ServiceServer service = n.advertiseService("/robot/GetCurrentQ", &UrTest::get_current_Q, &ur_test);
    ros::ServiceServer service2 = n.advertiseService("/robot/MoveToQ", &UrTest::move_to_Q, &ur_test);
    ros::ServiceServer service3 = n.advertiseService("/robot/MoveToPointTcp", &UrTest::move_to_point_tcp, &ur_test);
    ros::ServiceServer service4 = n.advertiseService("/robot/MoveToPoseTcp", &UrTest::move_to_pose_tcp, &ur_test);
    ros::ServiceServer service5 = n.advertiseService("/robot/MoveToPointMarker", &UrTest::move_to_point_marker, &ur_test);
    ros::ServiceServer service6 = n.advertiseService("/robot/MoveToPoseMarker", &UrTest::move_to_pose_marker, &ur_test);
    ros::ServiceServer service7 = n.advertiseService("/robot/tcpTmarker", &UrTest::change_end_transformation, &ur_test);
    
    ROS_INFO("We are about to spin");
    //ros::spin();
    ros::Time last_publish=ros::Time::now();
    ros::Duration time_between_publishing(0.14285); // camera has framerate of 7 hz
    ros::Duration time_difference;
    while(true)
    {
        ros::Time current_time=ros::Time::now();
        time_difference=current_time-last_publish;
        if(time_difference>=time_between_publishing)
        {
//             ROS_INFO("I'm inside");
            last_publish=ros::Time::now();
            ur_test.publish_position();
            ur_test.publish_marker_position();
        }
        
        ros::spinOnce();
    }

    return 0;
}
