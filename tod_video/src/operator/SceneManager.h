// Copyright 2020 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <QMainWindow>
#include <QPushButton>
#include <QCheckBox>
#include <thread>
#include <string>
#include <vector>
#include <memory>
#include "tod_msgs/Status.h"
#include "tod_core/CameraParameters.h"
#include "tod_video/ClientConfig.h"
#include "tod_video/VideoConfig.h"
#include "tod_video/BitrateConfig.h"

namespace Ui { class MainWindow; }

class SceneManager : public QMainWindow {
    Q_OBJECT

public:
    explicit SceneManager(ros::NodeHandle &nodeHandle, QApplication &app,
                          QWidget *parent = 0);
    ~SceneManager();
    void run();


private:
    struct Streamable;


private slots:
    void slot_toggle_checkbox_clicked(std::shared_ptr<Streamable> streamable);
    void slot_select_button_clicked(std::shared_ptr<Streamable> streamable);
    void slot_bitrate_changed();
    void slot_scaling_changed();
    void slot_cropping_changed();
    void slot_bitrate_sum_changed();


private:
    ros::NodeHandle &_nh;
    std::string _nn{""};
    ros::Subscriber _subStatusMsg, _subGps;
    ros::Publisher _pubBrPred;
    std::unique_ptr<tod_core::CameraParameters> _camParams;
    sensor_msgs::NavSatFixConstPtr _gpsMsg{nullptr};
    std::vector<std::shared_ptr<Streamable>> _streamables;
    std::shared_ptr<Streamable> _selectedStreamable{nullptr};
    QApplication &_app;
    Ui::MainWindow *_ui;
    uint8_t _vidRateControlMode{tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE};
    int _bitrateSum{0};
    bool _connected{false};
    bool _adaptVideoParamsEnabled{false};

    void callback_status_msg(const tod_msgs::Status &msg);
    void change_select_button_color(std::shared_ptr<Streamable> streamable, bool select);
    void set_reconfigure_fields_of_selected_stream();
    void reset_reconfigure_fields();
    void get_config_of_all_streams();
    template <typename T>
    T request_server_reconfigure(const T &desCfg, const std::string &name);
    void request_clients_reconfigure_and_update_gui(
        const tod_video::ClientConfig &config, std::shared_ptr<Streamable> streamable);

    struct Streamable {
        std::string name;
        bool unavailable{false}, streamOnConnect{true};
        QPushButton *button;
        QCheckBox *checkBox;
        tod_video::VideoConfig config;
        Streamable(const std::string &vehicleName, const std::string &operatorName, const bool myStreamOnConnect,
                   QPushButton *myButton, QCheckBox *myCheckBox)
            : name(operatorName), streamOnConnect(myStreamOnConnect), button(myButton), checkBox(myCheckBox) {
            config.camera_name = operatorName;
            config.bitrate = config.width = config.height = config.offset_width = config.offset_height = 0;
            config.scaling = tod_video::Video_1p000;
        }
    };
};
