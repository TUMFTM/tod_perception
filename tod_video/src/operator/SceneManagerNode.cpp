// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include <QApplication>
#include "SceneManager.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorSceneManager");
    ros::NodeHandle nodeHandle;
    QApplication app(argc, argv);
    SceneManager sceneManager(nodeHandle, app);
    sceneManager.run();
    return 0;
}
