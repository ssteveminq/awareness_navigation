#include <despot/simple_tui.h>
#include "pathfollowing.h"
#include "ros/ros.h"
#include <ros/package.h>

using namespace despot;

class TUI: public SimpleTUI {
public:
  TUI() {
  }

  DSPOMDP* InitializeModel(option::Option* options) {
   
   DSPOMDP* model =  new Navigation(10,10);

      return model;
  }

  void InitializeDefaultParameters() {
     Globals::config.num_scenarios = 100;
  }
};

int main(int argc, char* argv[]) {
  
 ros::init(argc, argv, "navi");
  
  // ros::Rate loop_rate(20);
  // while (ros::ok())
  // {
  //   // TUI().run(argc, argv);
  //    ros::spinOnce();
  //    loop_rate.sleep();  
  // }

  // ros::spin();

  // return 0;

  return TUI().run(argc, argv);


}


