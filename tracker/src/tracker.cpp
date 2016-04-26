#include <ros/ros.h>
#include "Application.hpp"

int main(int argc, char const *argv[]) {
    ros::init(argc, argv, "suivi");

    Application app;
    app.run();
    return 0;
}
