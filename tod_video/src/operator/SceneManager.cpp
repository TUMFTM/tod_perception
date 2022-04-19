// Copyright 2020 Andreas Schimpe
#include "SceneManager.h"
#include <ui_SceneManager.h>
#include <iostream>

SceneManager::SceneManager(ros::NodeHandle &nodeHandle, QApplication &app, QWidget *parent) :
    QMainWindow(parent),
    _nh(nodeHandle),
    _nn{ros::this_node::getName()},
    _camParams{std::make_unique<tod_core::CameraParameters>(_nh)},
    _app(app) {
    if (!_nh.getParam(_nn + "/adaptVideoParamsEnabled", _adaptVideoParamsEnabled))
        ROS_ERROR("%s: Could not get param adaptVideoParamsEnabled", _nn.c_str());

    _subStatusMsg = _nh.subscribe("/Operator/Manager/status_msg", 5,
                                          &SceneManager::callback_status_msg, this);

    if (_adaptVideoParamsEnabled) {
        _subGps = _nh.subscribe<sensor_msgs::NavSatFix>(
            "/Operator/VehicleBridge/gps/fix", 5, [&](const auto &msg) {
                _gpsMsg = msg;
            });
        _pubBrPred = _nh.advertise<geometry_msgs::PointStamped>("bitrate_desired_on_gps", 5);
    }

    struct NameAndStreamOnConnect { std::string vehicle_name; std::string operator_name; bool streamOnConnect{true}; };
    std::vector<NameAndStreamOnConnect> cams;
    for (const auto& cam : _camParams->get_sensors()) {
        auto& camToInitialize = cams.emplace_back();
        camToInitialize.vehicle_name = cam.vehicle_name;
        camToInitialize.operator_name = cam.operator_name;
        camToInitialize.streamOnConnect = cam.stream_on_connect;
        // support for camera names with dots: name with DOT, uri with .
        std::string str2find = "DOT";
        std::size_t found;
        while ((found = camToInitialize.vehicle_name.find(str2find)) != std::string::npos) {
            camToInitialize.vehicle_name.replace(found, str2find.length(), ".");
        }
        while ((found = camToInitialize.operator_name.find(str2find)) != std::string::npos) {
            camToInitialize.operator_name.replace(found, str2find.length(), ".");
        }
    }

    // init gui
    _ui = new Ui::MainWindow;
    _ui->setupUi(this);
    // create checkbox and push button for every camera to stream
    const int horOffs(40), verOffs(100), verStep(40), buttonHeight(30);
    for (int i=0; i < cams.size(); ++i) {
        const NameAndStreamOnConnect& cam = cams.at(i);
        _streamables.emplace_back(std::make_shared<Streamable>(
            cam.vehicle_name, cam.operator_name, cam.streamOnConnect, new QPushButton(cam.operator_name.c_str(), this),
            new QCheckBox(cam.operator_name.c_str(), this)));
        // push button
        _streamables.at(i)->button->setGeometry(horOffs, verOffs+i*verStep, 200, buttonHeight);
        _streamables.at(i)->button->setText(cam.operator_name.c_str());
        _streamables.at(i)->button->setToolTip("Select camera to reconfigure");
        _streamables.at(i)->button->setFont(QFont("Ubuntu", 12, 60, false));
        _streamables.at(i)->button->setPalette(QPalette(QColor(Qt::lightGray)));
        connect(_streamables.at(i)->button, &QPushButton::clicked, [=]() {
            slot_select_button_clicked(_streamables.at(i));
        });
        // checkbox
        _streamables.at(i)->checkBox->setGeometry(horOffs-30, verOffs + i*verStep, 20, buttonHeight);
        _streamables.at(i)->checkBox->setToolTip("Toggle Camera On and Off");
        connect(_streamables.at(i)->checkBox, &QPushButton::clicked, [=]() {
            slot_toggle_checkbox_clicked(_streamables.at(i));
        });
    }

    if (_adaptVideoParamsEnabled) {
        // reconfigure line edits
        connect(_ui->bitrateLineEdit, &QLineEdit::returnPressed, [=]() { slot_bitrate_changed(); });
        connect(_ui->scalingComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
                [=](int index){ slot_scaling_changed(); });
        connect(_ui->widthLineEdit, &QLineEdit::returnPressed, [=]() { slot_cropping_changed(); });
        connect(_ui->heightLineEdit, &QLineEdit::returnPressed, [=]() { slot_cropping_changed(); });
        connect(_ui->widthOffsetLineEdit, &QLineEdit::returnPressed, [=]() { slot_cropping_changed(); });
        connect(_ui->heightOffsetLineEdit, &QLineEdit::returnPressed, [=]() { slot_cropping_changed(); });
        connect(_ui->bitrateSumLineEdit, &QLineEdit::returnPressed, [=]() { slot_bitrate_sum_changed(); });
        _ui->scalingComboBox->addItem(tod_video::Video_1p000);
        //    _ui->scalingComboBox->addItem(tod_video::Video_0p875);
        _ui->scalingComboBox->addItem(tod_video::Video_0p750);
        //    _ui->scalingComboBox->addItem(tod_video::Video_0p675);
        _ui->scalingComboBox->addItem(tod_video::Video_0p500);
        //    _ui->scalingComboBox->addItem(tod_video::Video_0p375);
        _ui->scalingComboBox->addItem(tod_video::Video_0p250);
        _ui->scalingComboBox->addItem(tod_video::Video_0p125);
    }
    _ui->infoLineEdit->setText("");
    show(); // window
}

SceneManager::~SceneManager() {
    delete _ui;
    QApplication::quit();
}

void SceneManager::run() {
    std::thread rosThread([this]() {
        ros::Rate r(20);
        while (ros::ok()) {
            ros::spinOnce();
            if (_adaptVideoParamsEnabled) {
                if ( _gpsMsg && (_vidRateControlMode == tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_COLLECTIVE
                                || _vidRateControlMode == tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE)
                    && _connected) {
                    // publish current desired total bitrate on gps in video rate control modes single and collective
                    geometry_msgs::PointStamped predMsg;
                    predMsg.header.frame_id = _gpsMsg->header.frame_id;
                    predMsg.header.stamp = ros::Time::now();
                    predMsg.point.x = _gpsMsg->longitude;
                    predMsg.point.y = _gpsMsg->latitude;
                    predMsg.point.z = _bitrateSum;
                    _pubBrPred.publish(predMsg);
                }
            }
            r.sleep();
        }
        _app.closeAllWindows();
    });
    _app.exec(); // blocking
    rosThread.join();
}

void SceneManager::callback_status_msg(const tod_msgs::Status &msg) {
    bool prevConnected = _connected;
    uint8_t prevVidRateControlMode = _vidRateControlMode;
    _connected = (msg.tod_status != tod_msgs::Status::TOD_STATUS_IDLE);
    _vidRateControlMode = msg.operator_video_rate_mode;
    if (_connected && !prevConnected) {
        // on connect
        ros::Duration(2.0).sleep();
        get_config_of_all_streams();
        _ui->infoLineEdit->setText("Connected to vehicle.");
        for (auto& streamable : _streamables) {
            if (!streamable->streamOnConnect) {
                // pause
                tod_video::ClientConfig cfg;
                cfg.camera_name = streamable->name;
                cfg.pause = true;
                request_clients_reconfigure_and_update_gui(cfg, streamable);
                streamable->checkBox->setCheckState(Qt::CheckState::Unchecked);
            }
        }
    }
    if ((prevVidRateControlMode != tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE)
        && (_vidRateControlMode == tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE)) {
        // on entering mode Single
        get_config_of_all_streams();
        _ui->infoLineEdit->setText("Entered single manual mode.");
        if (_selectedStreamable) {
            change_select_button_color(_selectedStreamable, false);
            _selectedStreamable.reset();
        }
    }
    if ((prevVidRateControlMode != tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_AUTOMATIC)
        && (_vidRateControlMode == tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_AUTOMATIC)) {
        // on entering mode Automatic
        _bitrateSum = 0;
        _ui->bitrateSumLineEdit->setText(std::to_string(_bitrateSum).c_str());
        if (_selectedStreamable) {
            change_select_button_color(_selectedStreamable, false);
            _selectedStreamable.reset();
        }
        reset_reconfigure_fields();
        _ui->infoLineEdit->setText("Entered Automatic mode");
    }
    if (!_connected && prevConnected) {
        // on disconnect
        for (auto streamable : _streamables) {
            streamable->config.paused = true;
            streamable->checkBox->setCheckState(Qt::CheckState::Unchecked);
        }
        if (_selectedStreamable) {
            change_select_button_color(_selectedStreamable, false);
            _selectedStreamable.reset();
        }
        reset_reconfigure_fields();
        _ui->bitrateSumLineEdit->setText("0");
        _ui->infoLineEdit->setText("Disconnected from vehicle.");
    }
    if ((prevVidRateControlMode != tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_COLLECTIVE)
        && (_vidRateControlMode == tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_COLLECTIVE)) {
        // on entering mode Collective
        get_config_of_all_streams();
        reset_reconfigure_fields();
        _ui->infoLineEdit->setText("Entered Collective");
    }
}

void SceneManager::slot_toggle_checkbox_clicked(std::shared_ptr<Streamable> streamable) {
    if (!_connected) {
        _ui->infoLineEdit->setText("Cannot turn streams on or off when not connected to vehicle");
        streamable->checkBox->setCheckState(Qt::CheckState::Unchecked);
        return;
    }
    if (streamable->unavailable) {
        _ui->infoLineEdit->setText("Cannot turn stream on as it is not available");
        streamable->checkBox->setCheckState(Qt::CheckState::Unchecked);
        return;
    }
    if (_vidRateControlMode != tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE) {
        _ui->infoLineEdit->setText("Cannot turn stream on or off when not in single manual control mode");
        if (streamable->config.paused) streamable->checkBox->setCheckState(Qt::CheckState::Unchecked);
        else streamable->checkBox->setCheckState(Qt::CheckState::Checked);
        return;
    }
    // send reconfigure request to toggle rtsp client
    tod_video::ClientConfig config;
    config.camera_name = streamable->name;
    config.pause = !streamable->config.paused;
    request_clients_reconfigure_and_update_gui(config, streamable);
}

void SceneManager::slot_select_button_clicked(std::shared_ptr<Streamable> streamable) {
    if (!_connected) {
        _ui->infoLineEdit->setText("Cannot select a stream when not connected to vehicle");
        return;
    }
    if (_selectedStreamable) {
        // deselect
        if (streamable->name == _selectedStreamable->name) {
            _ui->infoLineEdit->setText(std::string("Deselected " + _selectedStreamable->name).c_str());
            change_select_button_color(_selectedStreamable, false);
            _selectedStreamable.reset();
            reset_reconfigure_fields();
            return;
        }
    }
    if (streamable->unavailable) {
        _ui->infoLineEdit->setText("Cannot select stream on as it is not available");
        return;
    }
    if (streamable->config.paused) {
        _ui->infoLineEdit->setText("Cannot select stream if it is paused");
        return;
    }
    if (_vidRateControlMode != tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE) {
        _ui->infoLineEdit->setText("Cannot select stream when not in single manual control mode");
        return;
    }
    if (!_adaptVideoParamsEnabled) {
        _ui->infoLineEdit->setText("Cannot select stream as network reconfigure is disabled");
        return;
    }
    // select, and deselect old selection if necessary
    if (_selectedStreamable) change_select_button_color(_selectedStreamable, false);
    _selectedStreamable = streamable;
    change_select_button_color(_selectedStreamable, true);
    set_reconfigure_fields_of_selected_stream();
    _ui->infoLineEdit->setText(std::string("Selected " + streamable->name + " for reconfigure").c_str());
}

void SceneManager::slot_bitrate_changed() {
    if (!_selectedStreamable) {
        _ui->infoLineEdit->setText("No stream selected - cannot change bitrate");
        reset_reconfigure_fields();
        return;
    }
    _bitrateSum -= _selectedStreamable->config.bitrate;
    tod_video::VideoConfig desCfg = _selectedStreamable->config;
    desCfg.bitrate = _ui->bitrateLineEdit->text().toInt();
    _selectedStreamable->config = request_server_reconfigure(desCfg, "VideoConfigSend");
    set_reconfigure_fields_of_selected_stream();
    _bitrateSum += _selectedStreamable->config.bitrate;
    _ui->bitrateSumLineEdit->setText(std::to_string(_bitrateSum).c_str());
    _ui->infoLineEdit->setText(std::string("Changed bitrate of " + _selectedStreamable->name).c_str());
}

void SceneManager::slot_scaling_changed() {
    if (!_selectedStreamable) {
        _ui->infoLineEdit->setText("No stream selected - cannot scale");
        reset_reconfigure_fields();
        return;
    }
    tod_video::VideoConfig desCfg = _selectedStreamable->config;
    desCfg.scaling = std::string(_ui->scalingComboBox->currentText().toUtf8());
    if (desCfg.scaling == _selectedStreamable->config.scaling) return;
    _selectedStreamable->config = request_server_reconfigure(desCfg, "VideoConfigSend");
    set_reconfigure_fields_of_selected_stream();
    _ui->infoLineEdit->setText(std::string("Scaled " + _selectedStreamable->name).c_str());
}

void SceneManager::slot_cropping_changed() {
    if (!_selectedStreamable) {
        _ui->infoLineEdit->setText("No stream selected - cannot crop");
        reset_reconfigure_fields();
        return;
    }
    tod_video::VideoConfig desCfg = _selectedStreamable->config;
    desCfg.width = _ui->widthLineEdit->text().toInt();
    desCfg.height = _ui->heightLineEdit->text().toInt();
    desCfg.offset_width = _ui->widthOffsetLineEdit->text().toInt();
    desCfg.offset_height = _ui->heightOffsetLineEdit->text().toInt();
    _selectedStreamable->config = request_server_reconfigure(desCfg, "VideoConfigSend");
    set_reconfigure_fields_of_selected_stream();
    _ui->infoLineEdit->setText(std::string("Cropped " + _selectedStreamable->name).c_str());
}

void SceneManager::slot_bitrate_sum_changed() {
    if (!_connected) {
        _ui->infoLineEdit->setText("Cannot change bitrate sum when not connected to vehicle");
        _ui->bitrateSumLineEdit->setText(std::to_string(_bitrateSum).c_str());
        return;
    }
    if (_vidRateControlMode != tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_COLLECTIVE) {
        _ui->infoLineEdit->setText("Cannot set bitrate sum, when not in collective manual mode");
        _ui->bitrateSumLineEdit->setText(std::to_string(_bitrateSum).c_str());
        return;
    }
    tod_video::BitrateConfig desCfg;
    desCfg.bitrate = _ui->bitrateSumLineEdit->text().toInt();
    tod_video::BitrateConfig actCfg = request_server_reconfigure(desCfg, "BitrateConfigSend");
    _bitrateSum = actCfg.bitrate;
    _ui->bitrateSumLineEdit->setText(std::to_string(_bitrateSum).c_str());
    std::string info{"Set bitrate sum to " + std::to_string(_bitrateSum)};
    _ui->infoLineEdit->setText(info.c_str());
}

void SceneManager::change_select_button_color(std::shared_ptr<Streamable> streamable, bool select) {
    QPalette pal = streamable->button->palette();
    if (select) pal.setColor(QPalette::Button, QColor(Qt::darkGreen));
    else pal.setColor(QPalette::Button, QColor(Qt::lightGray));
    streamable->button->setPalette(pal);
    streamable->button->update();
}

void SceneManager::set_reconfigure_fields_of_selected_stream() {
    _ui->bitrateLineEdit->setText(std::to_string(_selectedStreamable->config.bitrate).c_str());
    _ui->widthLineEdit->setText(std::to_string(_selectedStreamable->config.width).c_str());
    _ui->heightLineEdit->setText(std::to_string(_selectedStreamable->config.height).c_str());
    _ui->widthOffsetLineEdit->setText(std::to_string(_selectedStreamable->config.offset_width).c_str());
    _ui->heightOffsetLineEdit->setText(std::to_string(_selectedStreamable->config.offset_height).c_str());
    _ui->scalingComboBox->setCurrentText(_selectedStreamable->config.scaling.c_str());
}

void SceneManager::reset_reconfigure_fields() {
    _ui->bitrateLineEdit->setText("0");
    _ui->widthLineEdit->setText("0");
    _ui->heightLineEdit->setText("0");
    _ui->widthOffsetLineEdit->setText("0");
    _ui->heightOffsetLineEdit->setText("0");
    _ui->scalingComboBox->setCurrentIndex(0);
}

void SceneManager::get_config_of_all_streams() {
    if (_adaptVideoParamsEnabled) {
        _bitrateSum = 0;
        for (auto streamable : _streamables) {
            // request video config from rtsp server
            tod_video::VideoConfig cfg = streamable->config;
            cfg.width = cfg.height = -1; // server replies with current config on this
            streamable->config = request_server_reconfigure(cfg, "VideoConfigSend");
            if (streamable->config.paused) {
                streamable->checkBox->setCheckState(Qt::CheckState::Unchecked);
                streamable->unavailable = true;
            } else {
                streamable->checkBox->setCheckState(Qt::CheckState::Checked);
                _bitrateSum += streamable->config.bitrate;
                streamable->unavailable = false;
            }
        }
        _ui->bitrateSumLineEdit->setText(std::to_string(_bitrateSum).c_str());
    } else {
        // if network reconfigure disabled -> assume all videos are streaming when connected
        for (auto streamable : _streamables) {
            streamable->config.paused = false;
            streamable->checkBox->setCheckState(Qt::CheckState::Checked);
            _ui->bitrateSumLineEdit->setText("-1");
        }
    }
}

template <typename T>
T SceneManager::request_server_reconfigure(const T &desCfg, const std::string &name) {
    dynamic_reconfigure::ReconfigureRequest req;
    dynamic_reconfigure::ReconfigureResponse resp;
    desCfg.__toMessage__(req.config);
    if (!ros::service::call("/Operator/Video/" + name + "/set_parameters", req, resp))
        ROS_ERROR("%s: ROS service call to %s failed.", _nn.c_str(), name.c_str());
    T actCfg;
    actCfg.__fromMessage__(resp.config);
    return actCfg;
}

void SceneManager::request_clients_reconfigure_and_update_gui(
    const tod_video::ClientConfig &config, std::shared_ptr<Streamable> streamable) {
    dynamic_reconfigure::ReconfigureRequest req;
    dynamic_reconfigure::ReconfigureResponse resp;
    config.__toMessage__(req.config);
    if (!ros::service::call("/Operator/Video/RtspClients/set_parameters", req, resp))
        ROS_ERROR("%s: failed to call service", _nn.c_str());
    else
        streamable->config.paused = config.pause;
    std::string mode{""};
    if (streamable->config.paused) {
        mode = "Stopped";
        _bitrateSum -= streamable->config.bitrate;
    } else {
        mode = "Started";
        _bitrateSum += streamable->config.bitrate;
    }
    _ui->bitrateSumLineEdit->setText(std::to_string(_bitrateSum).c_str());
    _ui->infoLineEdit->setText(std::string(mode + " streaming " + streamable->name).c_str());
}
