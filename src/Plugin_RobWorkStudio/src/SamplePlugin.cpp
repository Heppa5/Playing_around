#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>


// ROS
#include <ros/ros.h>
#include <ros/time.h>
//#include <std_msgs>
#include <geometry_msgs/PoseStamped.h>

#include <Plugin_RobWorkStudio/suctionCup.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <message_package/Q.h>

#include <functional>
#include <rw/kinematics/MovableFrame.hpp>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;

using namespace cv;

using namespace std;

Foo hej4((int)(2));

class BeforeMain
{
    BeforeMain(){
        cout << "HEr er jeg" << endl;
//        ros::init();
    }
};
BeforeMain hej();

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    cout << "IN constructor" << endl;

	setupUi(this);
//    int argc=1;
//    std::vector<char*> argv;
////    argv.push_back(nullptr);
//    argv.push_back("home/jepod13/catkin_ws/devel/lib/libRobWorkStudioPlugin.so");
//    cout << "Wow " << endl;
//    ros::init(argc, argv.data(), "ROS_ROBWORK_STUDIO_PLUGIN");
//    cout << "Wow3 " << endl;
//    ros::NodeHandle nh_;
	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;



    _pub_current_Q = nh_.advertise<message_package::Q>("/robot_sim/Q", 1);
    cout << "Number of subcribers to current Q: " << _pub_current_Q.getNumSubscribers() << endl;
    _pub_marker1 = nh_.advertise<geometry_msgs::PoseStamped>("/kalman_filter/marker1", 1);
    _pub_marker2 = nh_.advertise<geometry_msgs::PoseStamped>("/kalman_filter/marker2", 1);

//    ros::ServiceServer service = nh_.advertiseService("/caros_universalrobot/set_io", &SamplePlugin::set_suction,this);
    service = nh_.advertiseService("/caros_universalrobot/set_io", &SamplePlugin::set_suction,this);



    _sub_new_Q = nh_.subscribe("/robot_sim/new_Q", 10, &SamplePlugin::new_Q, this);
    _sub_attach_marker1 = nh_.subscribe("/robot_sim/attach_marker1", 100, &SamplePlugin::attach_marker1, this);
    _timer->start(timer_time_ms);
    cout << "WOW#" << endl;
    cout <<"Number of Publishers for new Q: " << _sub_new_Q.getNumPublishers() << endl;
    cout <<"Topis for new Q: " <<_sub_new_Q.getTopic() << endl;
    cout << "Out of constructor" << endl;


}

bool SamplePlugin::set_suction(Plugin_RobWorkStudio::suctionCup::Request &req, Plugin_RobWorkStudio::suctionCup::Response &res)
{
    cout << "Value is: " << req.value << endl;
    if(req.value==1)
    {
        cout << "&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;

        cout << "I was attached" << endl;
        auto box_frame=_wc->findFrame("box1");
        auto suctionCup_frame=_wc->findFrame("SuctionCup");
//        box_frame->attachTo(suctionCup_frame,_state);
    }
    else{
        cout << "&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
        cout << "I was unattached" << endl;
        auto box_frame=_wc->findFrame("box1");
        auto world_frame=_wc->findFrame("WORLD");
//        box_frame->attachTo(world_frame,_state);
    }
    res.executed=1;

}

void SamplePlugin::new_Q(const message_package::Q::Ptr msg)
{
    rw::math::Q Next_location(6,0,0,0,0,0,0);
    cout << "Got new Q" << endl;
    for(int i=0; i<6 ; i++)
    {
        Next_location[i]=msg->robot_configuration[i];
    }
    current_goal=Next_location;
    current_goal_initialized=true;
}

void SamplePlugin::attach_marker1(const std_msgs::Bool msg)
{
    // NEED to update
}

SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize() {
	log().info() << "INITALIZE" << "\n";
	getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);
        
	// Auto load workcell
//	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/jepod13/Downloads/FinalProject/PA10WorkCell/ScenePA10RoVi1.wc.xml");
	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/jepod13/Jesper/UR5/Mini-Picker/Mini-Picker_v2_DHJoints.wc.xml");
//    SamplePlugin::open(wc);
    getRobWorkStudio()->setWorkCell(wc);


	// Load Lena image
	Mat im, image;
// 	im = imread("/home/jepod13/Downloads/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
// 	cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
// 	if(! image.data ) {
// 		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
// 	}
// 	QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
// 	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}

void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();
    _device = _wc->findDevice("UR5");

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
	// Add the texture render to this workcell if there is a frame for texture
//	Frame* textureFrame = _wc->findFrame("MarkerTexture");
//	if (textureFrame != NULL) {
//		getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
//	}
	// Add the background render to this workcell if there is a frame for texture
//	Frame* bgFrame = _wc->findFrame("Background");
//	if (bgFrame != NULL) {
//		getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
//	}

	// Create a GLFrameGrabber if there is a camera frame with a Camera property set
	Frame* cameraFrame = _wc->findFrame("CameraSim");
	if (cameraFrame != NULL) {
		if (cameraFrame->getPropertyMap().has("cam")) {
			// Read the dimensions and field of view
			double fovy;
			int width,height;
//			std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
			std::string camParam = cameraFrame->getPropertyMap().get<std::string>("cam");
			std::istringstream iss (camParam, std::istringstream::in);
			iss >> fovy >> width >> height;
			// Create a frame grabber
			_framegrabber = new GLFrameGrabber(width,height,fovy);
			SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
			_framegrabber->init(gldrawer);
		}
	}
    }
}


void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}


void SamplePlugin::btnPressed() {
    QObject *obj = sender();
	if(obj==_btn0){
		log().info() << "Button 0\n";
		// Set a new texture (one pixel = 1 mm)
//		Image::Ptr image;
//		image = ImageLoader::Factory::load("/home/jepod13/Downloads/FinalProject/SamplePluginPA10/markers/Marker1.ppm");
//		_textureRender->setImage(*image);
//		image = ImageLoader::Factory::load("/home/jepod13/Downloads/FinalProject/SamplePluginPA10/backgrounds/color1.ppm");
//		_bgRender->setImage(*image);
//		getRobWorkStudio()->updateAndRepaint();
	} else if(obj==_btn1){
		log().info() << "Reset\n";
        reset_values();
		// Toggle the timer on and off
//		if (!_timer->isActive())
//		    _timer->start(100); // run 10 Hz
//		else
//			_timer->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
        promille_correction=_spinBox->value();
	}
}

rw::math::Q SamplePlugin::calc_new_Q()
{
    if(current_goal_initialized==true) {
        auto current = _device->getQ(_state);
        rw::math::Q result(6,0,0,0,0,0,0);
        bool really_close = true;
        for (int i = 0; i < 6; i++) {

            result(i) = current(i) + (current_goal(i) - current(i)) * ((double) promille_correction / 1000.0);
//            cout << "Joint " << i << " = " << result(i) << endl;
            if ((current_goal(i) - current(i)) * ((double) promille_correction / 1000.0) > 0.005) {
                really_close = false;
            }
        }
//        if (really_close == true) {
//            return current_goal;
//        }
        return result;
    }else {
        return _device->getQ(_state);
    }
}

void SamplePlugin::reset_values(){
    rw::math::Q start(6,0, -1.57, -1.57, -1.57, 1.57, 0);
    current_goal=start;
//    <RPY> 0 0 0 </RPY> <Pos> 0.535 -0.235 -0.625 </Pos>
    rw::math::RPY<> hej(0,0,0);
    rw::math::Vector3D<> wow(0.535,-0.235,-0.625);
    rw::math::Transform3D<> wow2(wow,hej);
    rw::kinematics::MovableFrame* box1_frame=(rw::kinematics::MovableFrame*)_wc->findFrame("box1");
    auto world_frame=_wc->findFrame("WORLD");
    box1_frame->attachTo(world_frame,_state);
    box1_frame->setTransform(wow2,_state);
    _device->setQ(start, _state);
    getRobWorkStudio()->setState(_state);

}

void SamplePlugin::timer() {
//    cout << "WTF_timer" << endl;
//	if (_framegrabber != NULL) {
//		// Get the image as a RW image
//		Frame* cameraFrame = _wc->findFrame("CameraSim");
//		_framegrabber->grab(cameraFrame, _state);
//		const Image& image = _framegrabber->getImage();
//
//		// Convert to OpenCV image
//		Mat im = toOpenCVImage(image);
//		Mat imflip;
//		cv::flip(im, imflip, 0);
//
//		// Show in QLabel
//		QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
//		QPixmap p = QPixmap::fromImage(img);
//		unsigned int maxW = 400;
//		unsigned int maxH = 800;
//		_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
//	}
    if(update_time_counter%update_time==0)
    {
        if (_device != nullptr) {
            auto current_Q = _device->getQ(_state);
            message_package::Q msg;
            for (int i = 0; i < 6; i++) {
                msg.robot_configuration[i] = current_Q[i];
            }
            _pub_current_Q.publish(msg);
            //        cout <<"Device is not Null Pointer" << endl;
        } else {
            cout << "We need a device pointer" << endl;
        }
        auto cam_frame = _wc->findFrame("cam");
        auto marker1_frame = _wc->findFrame("marker_f");
        auto marker2_frame = _wc->findFrame("marker2_f");
        if (marker1_frame != nullptr && cam_frame != nullptr) {
            geometry_msgs::PoseStamped msg;
            auto cTm1 = cam_frame->fTf(marker1_frame, _state);
            msg.pose.position.x = cTm1.P()(0);
            msg.pose.position.y = cTm1.P()(1);
            msg.pose.position.z = cTm1.P()(2);
            rw::math::EAA<> hej(cTm1.R());
            msg.pose.orientation.x = hej(0);
            msg.pose.orientation.y = hej(1);
            msg.pose.orientation.z = hej(2);
            _pub_marker1.publish(msg);
        } else {
            cout << "We need a pointer cam/marker1" << endl;
        }
        if (marker2_frame != nullptr && cam_frame != nullptr) {
            geometry_msgs::PoseStamped msg;
            auto cTm2 = cam_frame->fTf(marker2_frame, _state);
            msg.pose.position.x = cTm2.P()(0);
            msg.pose.position.y = cTm2.P()(1);
            msg.pose.position.z = cTm2.P()(2);
            rw::math::EAA<> hej(cTm2.R());
            msg.pose.orientation.x = hej(0);
            msg.pose.orientation.y = hej(1);
            msg.pose.orientation.z = hej(2);
            _pub_marker2.publish(msg);
        } else {
            cout << "We need a pointer cam/marker2" << endl;
        }


        // LAst thing to do
//        cout << "CALC NEW Q " << endl;
        auto new_temp_Q = calc_new_Q();
//        cout << "Calculated a Q" << endl;
        _device->setQ(new_temp_Q, _state);
        getRobWorkStudio()->setState(_state);
        //    cout << "WTF_OUT"<< endl;
    }
    ros::spinOnce();
    update_time_counter=update_time_counter+timer_time_ms;
}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}
