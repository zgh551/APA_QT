#-------------------------------------------------
#
# Project created by QtCreator 2019-08-30T14:39:34
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = APA_Assistant
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
CONFIG += c++11
CONFIG += resources_big
CONFIG += precompile_header
#PRECOMPILED_HEADER = stable.h

SOURCES += \
    Common/Configure/Configs/vehilce_config.cpp \
    Common/Filter/digital_filter.cpp \
    Common/Filter/digital_filter_coefficients.cpp \
    Common/Math/algebraic_geometry.cpp \
    Common/Math/crc_compute.cpp \
    Common/Math/curve_fitting.cpp \
    Common/Math/fresnel.cpp \
    Common/Math/huogh.cpp \
    Common/Math/interpolation.cpp \
    Common/Math/linear_quadratic_regulator.cpp \
    Common/Math/math_utils.cpp \
    Common/Math/solve_equation.cpp \
    Common/Math/vector_2d.cpp \
    Common/Math/vehicle_body.cpp \
    Common/Utils/Src/link_list.cpp \
    Common/Utils/Src/node.cpp \
    Common/VehicleState/GeometricTrack/geometric_track.cpp \
    Common/VehicleState/Interface/vehicle_state.cpp \
    Control/Common/pid.cpp \
    Control/Common/trajectory_analyzer.cpp \
    Control/Interface/controller.cpp \
    Control/LatControl/lat_control.cpp \
    Control/LatControl/lat_control_lqr.cpp \
    Control/LonControl/lon_control.cpp \
    Interaction/CANBUS/BoRui/bo_rui_controller.cpp \
    Interaction/CANBUS/BoRui/bo_rui_message.cpp \
    Interaction/CANBUS/ChangAn/chang_an_controller.cpp \
    Interaction/CANBUS/ChangAn/chang_an_message.cpp \
    Interaction/CANBUS/DongFengE70/dong_feng_e70_message.cpp \
    Interaction/CANBUS/Interface/message_manager.cpp \
    Interaction/CANBUS/Interface/vehicle_controller.cpp \
    Interaction/HMI/Terminal.cpp \
    Interaction/HMI/simulation.cpp \
    Interaction/Ultrasonic/Ultrasonic.cpp \
    Percaption/Interface/percaption.cpp \
    Percaption/UltrasonicPercaption/ultrasonic_obstacle_percption.cpp \
    Planning/Common/configuration.cpp \
    Planning/Common/steering_common.cpp \
    Planning/Curvature/curvature.cpp \
    Planning/HC_CC_StateSpace/hc00_reeds_shepp_state_space.cpp \
    Planning/HC_CC_StateSpace/hc0pm_reeds_shepp_state_space.cpp \
    Planning/HC_CC_StateSpace/hc_reeds_shepp_state_space.cpp \
    Planning/HC_CC_StateSpace/hcpm0_reeds_shepp_state_space.cpp \
    Planning/HC_CC_StateSpace/hcpmpm_reeds_shepp_state_space.cpp \
    Planning/Interface/planning.cpp \
    Planning/Interface/hc_cc_state_space.cpp \
    Planning/ParallelParking/parallel_planning.cpp \
    Planning/Path/hc_cc_circle.cpp \
    Planning/Path/hc_cc_rs_path.cpp \
    Planning/Path/path.cpp \
    Planning/Path/path_line.cpp \
    Planning/VerticalParking/vertical_planning.cpp \
    QCustomPlot/qcustomplot.cpp \
    QCustomPlot/axistag.cpp \
    main.cpp \
    mainwindow.cpp \

#    Planning/Interface/sh2_state_space.cpp \
#    Planning/OMPL_Path/ompl_obstacle.cpp \
#    Planning/OMPL_Path/ompl_planner.cpp \
#    Planning/OMPL_Path/ompl_space.cpp \

HEADERS += \
    Common/Configure/Configs/system_config.h \
    Common/Configure/Configs/vehilce_config.h \
    Common/Configure/Data/bo_rui_configure.h \
    Common/Configure/Data/chang_an_configure.h \
    Common/Configure/Data/common_configure.h \
    Common/Configure/Data/dong_feng_configure.h \
    Common/Filter/digital_filter.h \
    Common/Filter/digital_filter_coefficients.h \
    Common/Math/algebraic_geometry.h \
    Common/Math/crc_compute.h \
    Common/Math/curve_fitting.h \
    Common/Math/fresnel.h \
    Common/Math/huogh.h \
    Common/Math/interpolation.h \
    Common/Math/linear_quadratic_regulator.h \
    Common/Math/math_utils.h \
    Common/Math/solve_equation.h \
    Common/Math/vector_2d.h \
    Common/Math/vehicle_body.h \
    Common/Utils/Inc/link_list.h \
    Common/Utils/Inc/node.h \
    Common/Utils/Inc/property.h \
    Common/VehicleState/GeometricTrack/geometric_track.h \
    Common/VehicleState/Interface/vehicle_state.h \
    Control/Common/pid.h \
    Control/Common/trajectory_analyzer.h \
    Control/Interface/controller.h \
    Control/LatControl/lat_control.h \
    Control/LatControl/lat_control_lqr.h \
    Control/LonControl/lon_control.h \
    Interaction/CANBUS/BoRui/bo_rui_controller.h \
    Interaction/CANBUS/BoRui/bo_rui_message.h \
    Interaction/CANBUS/ChangAn/chang_an_controller.h \
    Interaction/CANBUS/ChangAn/chang_an_message.h \
    Interaction/CANBUS/DongFengE70/dong_feng_e70_controller.h \
    Interaction/CANBUS/DongFengE70/dong_feng_e70_message.h \
    Interaction/CANBUS/Interface/message_manager.h \
    Interaction/CANBUS/Interface/vehicle_controller.h \
    Interaction/HMI/Terminal.h \
    Interaction/HMI/simulation.h \
    Interaction/Ultrasonic/Ultrasonic.h \
    Percaption/Interface/percaption.h \
    Percaption/UltrasonicPercaption/ultrasonic_obstacle_percption.h \
    Planning/Common/configuration.h \
    Planning/Common/steering_common.h \
    Planning/Curvature/curvature.h \
    Planning/HC_CC_StateSpace/hc00_reeds_shepp_state_space.h \
    Planning/HC_CC_StateSpace/hc0pm_reeds_shepp_state_space.h \
    Planning/HC_CC_StateSpace/hc_reeds_shepp_state_space.h \
    Planning/HC_CC_StateSpace/hcpm0_reeds_shepp_state_space.h \
    Planning/HC_CC_StateSpace/hcpmpm_reeds_shepp_state_space.h \
    Planning/Interface/planning.h \
    Planning/Interface/hc_cc_state_space.h \
    Planning/ParallelParking/parallel_planning.h \
    Planning/Path/hc_cc_circle.h \
    Planning/Path/hc_cc_rs_path.h \
    Planning/Path/path.h \
    Planning/Path/path_line.h \
    Planning/VerticalParking/vertical_planning.h \
    QCustomPlot/axistag.h \
    QCustomPlot/qcustomplot.h \
    mainwindow.h \
    stable.h \

#    Planning/Interface/sh2_state_space.h \
#    Planning/OMPL_Path/ompl_obstacle.h \
#    Planning/OMPL_Path/ompl_planner.h \
#    Planning/OMPL_Path/ompl_space.h \

#LIBS += -L$$PWD/WinZlgCan/ -lControlCAN

INCLUDEPATH += /usr/include/eigen3
#INCLUDEPATH += C:\Boost\include\boost-1_73

#DEFINES += BOOST_USE_LIB

FORMS += \
        mainwindow.ui

RESOURCES += \
    icon.qrc

DISTFILES +=
