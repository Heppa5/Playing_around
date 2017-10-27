
#include "mp_mini_picker/currentQ.h"
#include "mp_mini_picker/moveToQ.h"

#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>

#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/proximity/CollisionDetector.hpp>


#include <rw/math/Vector3D.hpp>

#include <ros/ros.h>

#include <string>
#include <stdexcept>

#include <std_msgs/String.h>


#include <geometry_msgs/PoseStamped.h>

using namespace ros;
using namespace std;

class UrTest
{
    public:
        UrTest() : nodehandle_("~"), sdsip_(nodehandle_, "caros_universalrobot")
        {
            initWorkCell();
            initDevice();
            initPathPlannerWithCollisionDetector();
            
            robot_move = nodehandle_.advertise<geometry_msgs::PoseStamped>("/robot/moved", 1);
            
        }

        virtual ~UrTest()
        {
            /* Empty */
        }

       
        
//         ros::std_msgs::Float64MultiArray get_current_Q()
        bool get_current_Q(mp_mini_picker::currentQ::Request  &req,
                   mp_mini_picker::currentQ::Response &res)
        {
            ROS_INFO("Got service call for current Q");
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
//             cout << path.size() << endl;
            
            bool stop_succeeded = sdsip_.stop();
            if(!stop_succeeded)
                ROS_WARN("Did not succeed in sending stop command");
            
            for (int i = 0; i<path.size() ; i++)
            {
                rw::math::Q p = path[i];
                bool ret = false;
                ret = sdsip_.movePtp(p);
                
                if (!ret)
                {
//                  return_status = false;
                    ROS_ERROR_STREAM("The serial device didn't acknowledge the movePtp command.");
                }
//                 
            }
            
//             rw::math::Q current_configuration = getCurrentJointConfiguration();
//             rw::kinematics::State state = workcell_->getDefaultState();
//             
//             
//             
//             device_->setQ(current_configuration,state);
// //             auto wTb=device_->worldTbase(state);
//             
// //             auto bTe=device_->baseTend(state);
// //             auto tcp_frame=workcell_->findFrame("UR5.Joint5");
//             auto tcp_frame=workcell_->findFrame("tool");
//             auto bTe=device_->baseTframe(tcp_frame,state);
//             auto bMarker= bTe*rw::math::Vector3D<double>(0,0,0);
//             
//             cout << bMarker << endl;
//             
//             geometry_msgs::Pose msg;
//             msg.position.x= bMarker[0];
//             msg.position.y= bMarker[1];
//             msg.position.z= bMarker[2];
// 
//             robot_move.publish(msg);

            
        }
        
        void publish_position()
        {
            auto current_configuration=sdsip_.getQ();
            rw::kinematics::State state = workcell_->getDefaultState();
            device_->setQ(current_configuration,state);
             auto wTb=device_->worldTbase(state);
            
//             auto bTe=device_->baseTend(state);
//             auto tcp_frame=workcell_->findFrame("UR5.Joint5");
            auto tcp_frame=workcell_->findFrame("tool");
            auto bTe=device_->baseTframe(tcp_frame,state);
            auto bMarker= wTb*bTe*rw::math::Vector3D<double>(0,0,0);
            
//             cout << bMarker << endl;
            
            geometry_msgs::PoseStamped msg;
            msg.pose.position.x= bMarker[0];
            msg.pose.position.y= bMarker[1];
            msg.pose.position.z= bMarker[2];
            msg.header.stamp = ros::Time::now();
            robot_move.publish(msg);

        }
    protected:
        
        void initWorkCell()
        {
            workcell_ = caros::getWorkCell();
            if (workcell_ == NULL)
            {
                ROS_ERROR("No workcell was loaded - exiting...");
                throw std::runtime_error("Not able to obtain a workcell.");
            }
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
            rw::kinematics::State state = workcell_->getDefaultState();
            /* Collision detector */
            auto detector = rw::common::ownedPtr( new rw::proximity::CollisionDetector(workcell_,
                                                                                       rwlibs::proximitystrategies::ProximityStrategyPQP::make()
                                                                                      )
                                                );
            /* PlannerConstraint that uses the collision detector to verify that the _start_ and _end_ configurations are
                * collision free and that the edge(s) between those is/are also collision free. */
            const rw::pathplanning::PlannerConstraint planner_constraint = rw::pathplanning::PlannerConstraint::make(detector,
                                                                                                                     device_, 
                                                                                                                     state
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
            /* Make sure to get and operate on fresh data from the serial device
                * It's assumed that the serial device is not movinghttps://e-learn.sdu.dk/bbcswebdav/pid-4929056-dt-content-rid-7854943_2/courses/EK-SSP-U1-1-E17/Agenda%20STATSIG%20lecture%2007.pdf
                * ^- That could be asserted / verified using sdsip.isMoving()
                * However other sources could invoke services on the UR that causes it to move...
                */
//             ros::Time current_timestamp = ros::Time::now();
//             ros::Time obtained_timestamp = sdsip_.getTimeStamp();
//             ROS_INFO("Current timestamp: %s", current_timestamp);
//             ROS_INFO("Obtained timestamp: %s", obtained_timestamp);
//             if(!(sdsip_.isMoving())
//             {
//                 while (current_timestamp > obtained_timestamp )
//                 {
//                     ROS_INFO("Not getting fresh data from serial device - waiting 100 ms");
//                     ros::Duration(0.1).sleep();  // In seconds
//                     ros::spinOnce();
//                     obtained_timestamp = sdsip_.getTimeStamp();
//                 }
//             }

            return sdsip_.getQ();
        }

        

    protected:
        ros::NodeHandle nodehandle_;
        caros::SerialDeviceSIProxy sdsip_;

        rw::models::WorkCell::Ptr workcell_;
        rw::models::Device::Ptr device_;
        rw::pathplanning::QToQPlanner::Ptr planner_;
        
        ros::Publisher robot_move;
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "caros_universalrobot");
    ros::NodeHandle n;
        
    const double q_change = -0.2;

    UrTest ur_test;
    
    
    ros::ServiceServer service = n.advertiseService("/robot/GetCurrentQ", &UrTest::get_current_Q, &ur_test);
    ros::ServiceServer service2 = n.advertiseService("/robot/MoveToQ", &UrTest::move_to_Q, &ur_test);
    
    
    
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
        }
        
        ros::spinOnce();
    }

    return 0;
}
