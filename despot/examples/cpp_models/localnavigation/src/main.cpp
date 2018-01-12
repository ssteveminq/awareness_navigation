#include <despot/simple_tui.h>
#include "localnavigation.h"
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
  return TUI().run(argc, argv);
}


