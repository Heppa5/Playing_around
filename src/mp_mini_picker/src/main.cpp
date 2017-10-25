
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


#include <geometry_msgs/Pose.h>

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
            ROS_INFO("I'm Here");
            robot_move = nodehandle_.advertise<geometry_msgs::Pose>("/robot/moved", 1);
            ROS_INFO("I'm Here2");
            
        }

        virtual ~UrTest()
        {
            /* Empty */
        }

        bool testMovePtp(const double q_change)
        {
            if (!doTestMovePtp(q_change))
            {
                std::cout<<"Check 3"<<std::endl;
                return false;
            }
            ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
            ros::Duration(5).sleep();  // In seconds

            if (!doTestMovePtp(-q_change))
            {
                std::cout<<"Check 4"<<std::endl;
                return false;
            }
            ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
            ros::Duration(5).sleep();  // In seconds

            return true;
        }

        bool testMoveServoQ(const double q_change)
        {
            if (!doTestMoveServoQ(q_change))
            {
                return false;
            }
            ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
            ros::Duration(5).sleep();  // In seconds

            if (!doTestMoveServoQ(-q_change))
            {
                return false;
            }
            ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
            ros::Duration(5).sleep();  // In seconds

            return true;
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
            
            
//             vector<double> vec1 = { 1.1, 2., 3.1};
//             std_msgs::Float64MultiArray msg;
// 
//             // set up dimensions
//             msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
//             msg.layout.dim[0].size = vec1.size();
//             msg.layout.dim[0].stride = 1;
//             msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1
// 
//             // copy in the data
//             msg.data.clear();
//             msg.data.insert(msg.data.end(), vec1.begin(), vec1.end());
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
            
            rw::math::Q current_configuration = getCurrentJointConfiguration();
            rw::kinematics::State state = workcell_->getDefaultState();
            
            
            
            device_->setQ(current_configuration,state);
//             auto wTb=device_->worldTbase(state);
            
//             auto bTe=device_->baseTend(state);
//             auto tcp_frame=workcell_->findFrame("UR5.Joint5");
            auto tcp_frame=workcell_->findFrame("tool");
            auto bTe=device_->baseTframe(tcp_frame,state);
            auto bMarker= bTe*rw::math::Vector3D<double>(0,0,0);
            
            cout << bMarker << endl;
            
            geometry_msgs::Pose msg;
            msg.position.x= bMarker[0];
            msg.position.y= bMarker[1];
            msg.position.z= bMarker[2];

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
                * It's assumed that the serial device is not moving
                * ^- That could be asserted / verified using sdsip.isMoving()
                * However other sources could invoke services on the UR that causes it to move...
                */
            ros::Time current_timestamp = ros::Time::now();
            ros::Time obtained_timestamp = sdsip_.getTimeStamp();
            while (current_timestamp > obtained_timestamp)
            {
                ros::Duration(0.1).sleep();  // In seconds
                ros::spinOnce();
                obtained_timestamp = sdsip_.getTimeStamp();
            }

            return sdsip_.getQ();
        }

        rw::trajectory::QPath getQPath(const double q_change)
        {
            rw::math::Q start_configuration = getCurrentJointConfiguration();
            rw::math::Q end_configuration = start_configuration + rw::math::Q(start_configuration.size(), q_change);
            rw::trajectory::QPath path;
            bool valid_path = false;
            ROS_ASSERT(planner_);
            valid_path = planner_->query(start_configuration, end_configuration, path);
            if (!valid_path)
            {
                ROS_ERROR_STREAM("Could not find a path from '" << start_configuration << "' to '" << end_configuration << "'.");
                throw std::runtime_error("No valid path found.");
            }

            return path;
        }

        rw::trajectory::QPath linearInterpolatedPath(const rw::math::Q& start, const rw::math::Q& end,
                                                    const double total_duration = 10.0, const double duration_step = 1.0)
        {
            ROS_ASSERT(duration_step > 0);
            ROS_ASSERT(duration_step < total_duration);

            rw::trajectory::QLinearInterpolator interpolator(start, end, total_duration);

            rw::trajectory::QPath path;

            path.push_back(start);
            for (double t = duration_step; t <= (total_duration - duration_step); t += duration_step)
            {
                path.push_back(interpolator.x(t));
            }
            path.push_back(end);

            return path;
        }

        bool doTestMovePtp(const double q_change)
        {
            bool return_status = true;
            std::cout<<"Check 5"<<std::endl;
            cout << "no joke" << endl;
            rw::trajectory::QPath path = getQPath(q_change);
            std::cout<<"Check 6"<<std::endl;
            for (const rw::math::Q& p : path)
            {
                ROS_INFO_STREAM("Ask to movePtp to '" << p << "'.");
                bool ret = false;
                std::cout<<"Check 7"<<std::endl;
                ret = sdsip_.movePtp(p);
                std::cout<<"Check 8"<<std::endl;
                if (!ret)
                {
                std::cout<<"Check 9"<<std::endl;
                return_status = false;
                ROS_ERROR_STREAM("The serial device didn't acknowledge the movePtp command.");
                }
            }

            return return_status;
        }

        bool doTestMoveServoQ(const double q_change)
        {
            bool return_status = true;
            rw::trajectory::QPath path = getQPath(q_change);

            ROS_ASSERT(path.size() == 2);
            rw::math::Q start_configuration = path.at(0);
            rw::math::Q end_configuration = path.at(1);

            // replace the path with an interpolated path
            path = linearInterpolatedPath(start_configuration, end_configuration);

            for (const rw::math::Q& p : path)
            {
                ROS_INFO_STREAM("Ask to moveServoQ to '" << p << "'.");
                bool ret = false;
                ret = sdsip_.moveServoQ(p);
                if (!ret)
                {
                return_status = false;
                ROS_ERROR_STREAM("The serial device didn't acknowledge the moveServoQ command.");
                }
            }

            return return_status;
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
    
    
    
//     bool ret = false;
//     //   ret = ur_test.testMovePtp(q_change);
// 
//     if (!ret)
//     {
//         ROS_ERROR_STREAM("Could not properly do the testMovePtp");
//         return 1;
//     }
    
    
    //ros::Publisher current_Q_pub = n.advertise<std_msgs::Float64MultiArray>("current_Q", 1000);
    
    ros::ServiceServer service = n.advertiseService("GetCurrentQ", &UrTest::get_current_Q, &ur_test);
    ros::ServiceServer service2 = n.advertiseService("MoveToQ", &UrTest::move_to_Q, &ur_test);
    
    
    
    ROS_INFO("We are about to spin");
    ros::spin();

    return 0;
}
