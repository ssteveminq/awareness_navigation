#include <despot/simple_tui.h>
#include "navigation.h"
#include "ros/ros.h"
#include <ros/package.h>

using namespace despot;

class TUI: public SimpleTUI {
public:
  TUI() {
  }

  DSPOMDP* InitializeModel(option::Option* options) {
   
    DSPOMDP* model = !options[E_PARAMS_FILE] ?
      new Navigation() : new Navigation(options[E_PARAMS_FILE].arg);
    return model;
  }

  void InitializeDefaultParameters() {
  }
};

int main(int argc, char* argv[]) {
  
 //ros::init(argc, argv, "navi");
  return TUI().run(argc, argv);
}


