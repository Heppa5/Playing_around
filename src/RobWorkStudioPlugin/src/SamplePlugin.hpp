#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// #define BOOST_BIND_NO_PLACEHOLDERS

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>

// Qt
#include <QTimer>

// ROS
#include <ros/ros.h>
#include <ros/time.h>

#include "ui_SamplePlugin.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <message_package/Q.h>

#include "foo.h"

using namespace std;



class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:

    SamplePlugin();
    virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

    void new_Q(const message_package::Q::Ptr msg);
    void attach_marker1(const std_msgs::Bool msg);
    rw::math::Q calc_new_Q();
private slots:
    void btnPressed();
    void timer();
  
    void stateChangedListener(const rw::kinematics::State& state);

private:

    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

    QTimer* _timer;

    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rw::models::Device::Ptr _device;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;
    ros::Publisher _pub_current_Q,_pub_marker1,_pub_marker2;
    ros::Subscriber _sub_new_Q,_sub_attach_marker1;
    rw::math::Q current_goal;
    int promille_correction=10;
    bool current_goal_initialized=false;
    ros::NodeHandle nh_;
    int timer_time_ms=21,update_time=100, update_time_counter=0;

};

#endif /*RINGONHOOKPLUGIN_HPP_*/
