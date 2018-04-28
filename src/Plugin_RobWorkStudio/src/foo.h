//
// Created by jepod13 on 3/23/18.
//

#ifndef ROBWORKSTUDIOPLUGIN_FOO_H
#define ROBWORKSTUDIOPLUGIN_FOO_H
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
using namespace std;
class Foo
{
public:
   Foo(int wow)
   {
       std::cout << "###################################WOWOWOWOWOWOWOW!" << std::endl;
       int argc=1;
       std::vector<char*> argv;
//    argv.push_back(nullptr);
       argv.push_back("home/jepod13/catkin_ws/devel/lib/libPlugin_RobWorkStudio.so");
       cout << "Wow " << endl;
       ros::init(argc, argv.data(), "ROS_ROBWORK_STUDIO_PLUGIN");
       cout << "Wow3 " << endl;
   }

 // other stuff
};
#endif //ROBWORKSTUDIOPLUGIN_FOO_H
