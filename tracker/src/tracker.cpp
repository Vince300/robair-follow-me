#include <ros/ros.h>
#include "Application.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "suivi");

    Application app;
    app.run();
    return 0;
}
