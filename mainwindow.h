#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vector>
/**
 * @brief 绘图库
 */
#include "QCustomPlot/qcustomplot.h"
#include "QCustomPlot/axistag.h"

/**
 * @brief Eigen矩阵库
 */
#include <Eigen/Dense>


//#include "ompl/base/spaces/DubinsStateSpace.h"
//#include "ompl/base/spaces/ReedsSheppStateSpace.h"
//#include "ompl/base/ScopedState.h"
//#include "ompl/geometric/SimpleSetup.h"
//#include <boost/program_options.hpp>

/**
 * @brief ZLG CAN 驱动
 */
//#include "WinZlgCan/win_zlg_can.h"
//#include "WinZlgCan/can_rev_work_thread.h"

/**
 * @brief 交互
 */
#include "Interaction/HMI/Terminal.h"
#include "Interaction/HMI/simulation.h"
#include "Interaction/Ultrasonic/Ultrasonic.h"

/**
 * @brief 规划
 */
#include "Planning/ParallelParking/parallel_planning.h"
#include "Planning/VerticalParking/vertical_planning.h"
#include "Planning/HC_CC_StateSpace/hc_reeds_shepp_state_space.h"
//#include "Planning/OMPL_Path/ompl_planner.h"
/**
 * @brief 控制
 */
#include "Control/LatControl/lat_control.h"
#include "Control/LatControl/lat_control_lqr.h"
#include "Control/Common/trajectory_analyzer.h"

/**
 * @brief 车辆配置
 */
#ifdef BORUI
#include "Interaction/CANBUS/BoRui/bo_rui_message.h"
#include "Interaction/CANBUS/BoRui/bo_rui_controller.h"
#endif

/**
 * @brief 车位边界配置
 */
//#define BOUNDARY_LEFT  (-18.0)
//#define BOUNDARY_RIGHT ( 18.0)
//#define BOUNDARY_TOP   ( 12.0)
//#define BOUNDARY_DOWN  (-12.0)

//#define BOUNDARY_X_SIZE     (36)
//#define BOUNDARY_Y_SIZE     (24)

namespace Ui {

class MainWindow;

}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    /********************************************************************************/
    /********************************* UI function **********************************/
    /********************************************************************************/
    void ControlUI(void);
    void DetectUI(void);
    void G1_PlanUI(void);
    void HC_PlanUI(void);
    void G2_PlanUI(void);
    void TrackUI(void);

    // init function
    void Init(void);

    void DetectVehicleModule(Vector2d p,float yaw);
    void PathVehicleModule(Vector2d p,float yaw);

    void VehicleModuleShow(Vector2d p,float yaw,QCPCurve *vehicle_center,QCPCurve *vehicle_modle,QCustomPlot *plot);
    /**
     * @brief 显示矢量箭头
     * @param x :the x axis location
     * @param y :the y axis location
     * @param yaw :the yaw angle od the vectoe arrow
     * @param arrow :the plot object
     */
    void VectorArrowShow(State base, State *head, QCPItemLine *arrow);

    /**
     * @brief 显示路径并通过颜色区别路径切换
     * @param p :the path point
     */
    void HC_CC_PathShow(vector<State> &p);

    /**
     * @brief 画圆
     * @param x :the x axis location with the centern of circle
     * @param y :the y axis location with the centern of circle
     * @param radius :the radius of the circle
     * @param e :the ellipse object
     */
    void CircleShow(double x, double y, double radius, QCPItemEllipse *e);

    /**
     * @brief 显示规划的几何圆
     * @param c :the circle location
     */
    void HC_CC_CircleShow(HC_CC_RS_Path *c);

    void TargetPathShow(TrackLinkList *list);
    void TargetPathShow(std::vector<TargetTrack> *vec);
    void SteeringAngleShow(float angle);

    //文件解析
    void FileDataInit(void);
    void AnalyzeOneLine(const QByteArray &baLine);


    // ompl planner test
//    void ompl_motion_planner(State start, State end);
//    void ompl_path_show(std::vector<ompl::base::State *> state,
//                        const ob::SpaceInformationPtr& si);

    void DetectTask(void);
    void PlanTask(void);
    void TrackTask(void);
    Ui::MainWindow *ui;
    // plot variable
    /* Control UI*/
    QGridLayout *gControlLayout;
    QCustomPlot *mControlPlot;
    QPointer<QCPGraph> mControlGraph1;
    QPointer<QCPGraph> mControlGraph2;
    AxisTag *mControlTag1;
    AxisTag *mControlTag2;

    /* Detect UI*/
    QGridLayout *gDetectLayout;
    QCustomPlot *mDetectPlot;
    QCPCurve *mDetectVehicleModuleCurve;
    QCPCurve *mDetectVehicleCenterCurve;
    QCPCurve *mDetectEdgePoint;
    QCPCurve *mDetectValidEdgePoint;
    QCPCurve *mDetectRearEdgeTrianglePosition;

    QCPCurve *mDetectLeftEdgeGroundPosition;
    QCPCurve *mDetectRightEdgeGroundPosition;

    QCPCurve *mDetectLeftEdgeFitLine;
    QCPCurve *mDetectRightEdgeFitLine;

    QLabel *label_FrontObstacle_Text;
    QLabel *label_FrontObstacleDistance_Value;
    QLabel *label_FrontObstacleRegion_Value;
    QLabel *label_FrontObstacleStatus_Value;
    QLabel *label_RearObstacle_Text;
    QLabel *label_RearObstacleDistance_Value;
    QLabel *label_RearObstacleRegion_Value;
    QLabel *label_RearObstacleStatus_Value;

    QString obstacle_region[5] = {"左侧", "左中","中间","右中", "右侧"};
    QString obstacle_status[5] = {"正常", "盲区", "超探", "噪声", "无效" };

    QRadioButton *radio_right_enter_location;
    QRadioButton *radio_left_enter_location;
    QRadioButton *radio_center_enter_location;

    QPushButton *button_file_select;
    QFile *detect_file;
    QPushButton *button_start_calculate;

    /* Planning UI*/
    /**
     * @brief G1 Ui elements
     */

    QRadioButton *radio_parallel;
    QRadioButton *radio_vertical;

    QLineEdit *text_VehicleInitPointX;
    QLineEdit *text_VehicleInitPointY;
    QLineEdit *text_VehicleInitPointYaw;

    QLineEdit *text_ParkingLength;
    QLineEdit *text_ParkingWidth;

    QLabel *label_G1_VehiceTrackX_Value;
    QLabel *label_G1_VehiceTrackY_Value;
    QLabel *label_G1_VehiceTrackYaw_Value;

    QPushButton *gParkingInformationConfirmG1;

    // 用于车辆模型的绘制
    QCPCurve *mPathVehicleModuleCurveG1;
    QCPCurve *mPathParkingCurveG1;
    QCPCurve *mPathVehicleCenterCurveG1;

    QCPItemEllipse *mPathLeftCircle;
    QCPItemEllipse *mPathRightCircle;

    QVector<double> ParkingPointX,ParkingPointY;

    QGridLayout *gG1_PlanLayout;
    QCustomPlot *mG1_PathPlot;

    /**
     * @brief HC Ui elements
     */

    QLineEdit *HC_VehicleStartPointX;
    QLineEdit *HC_VehicleStartPointY;
    QLineEdit *HC_VehicleStartPointYaw;
    QLineEdit *HC_VehicleStartPointKappa;

    QLineEdit *HC_VehicleEndPointX;
    QLineEdit *HC_VehicleEndPointY;
    QLineEdit *HC_VehicleEndPointYaw;
    QLineEdit *HC_VehicleEndPointKappa;
    /**
     * @brief 规划路径，根据曲率不同显示不同的颜色
     */
    QCPCurve *mPathPlanningStraightLine;
    QCPCurve *mPathPlanningClothoid;
    QCPCurve *mPathPlanningCircle;

    /**
     * @brief Arrow for start and goal position
     */
    QCPItemLine *mStartArrow;
    QCPItemLine *mEndArrow;

    /**
     * @brief Circle for HC CC RS
     */
    QCPItemEllipse *mStartCircle;
    QCPItemEllipse *mEndCircle;
    QCPItemEllipse *mMiddleCircle1;
    QCPItemEllipse *mMiddleCircle2;

    QPushButton *gParkingInformationConfirmHC;

    QGridLayout *gHC_PlanLayout;
    QCustomPlot *mHC_PathPlot;

//    QPointer<QCPGraph> mPathVehicleGraph;
//    QPointer<QCPGraph> mPathVehicleModuleDownGraph;

    /**
     * @brief G2 UI elements
     */
    QLineEdit *text_VehicleStartPointX;
    QLineEdit *text_VehicleStartPointY;
    QLineEdit *text_VehicleStartPointYaw;
    QLineEdit *text_VehicleStartPointKappa;

    QLineEdit *text_VehicleEndPointX;
    QLineEdit *text_VehicleEndPointY;
    QLineEdit *text_VehicleEndPointYaw;
    QLineEdit *text_VehicleEndPointKappa;

    QLabel *label_G2_VehiceTrackX_Value;
    QLabel *label_G2_VehiceTrackY_Value;
    QLabel *label_G2_VehiceTrackYaw_Value;

    QRadioButton *radio_obstacle_configure;
    QRadioButton *radio_start_gaol_configure;

    QPushButton *gParkingInformationConfirmG2;

    QCPCurve *mPathVehicleModuleCurveG2;
    QCPCurve *mPathParkingCurveG2;

    QGridLayout *gG2_PlanLayout;
    QCustomPlot *mG2_PathPlot;

    /**
     * @brief 描述圆和圆之间的切点
     */
    QCPCurve *mTangentCirclePoint;
    QCPColorMap *SpaceInformationMap;

    /**
     * @brief mTrackPlot: 跟踪模块
     */
    QCustomPlot *mTrackPlot;
    QCustomPlot *mTrackSinglePlot;

    QCPCurve *mTrackVehicleModuleCurve;
    QCPCurve *mTrackVehicleCenterCurve;
    QCPCurve *mTrackParkingCurve;
    QCPCurve *mTrackTargetCurve;

    QPointer<QCPGraph> mGraph_SteeringAngle;
    QPointer<QCPGraph> mGraph_Track;
    AxisTag *mTag_SteeringAngle;
    AxisTag *mTag_Track;

    QGridLayout *gPlotLayout;
    QGridLayout *gTrackLayout;

    QRadioButton *radio_sin_curvature;
    QRadioButton *radio_double_line;
    QRadioButton *radio_circle_curvature;

    QLabel *label_TrackUI_VehiceTrackX_Value;
    QLabel *label_TrackUI_VehiceTrackY_Value;
    QLabel *label_TrackUI_VehiceTrackYaw_Value;

    QPushButton *button_patn_generate;
    QPushButton *button_track_start;
    // timer 20ms Task
    QTimer mDataTimer20ms;
    QPushButton *button_timer_control;

    QListWidget *list_function;
    QStackedWidget *stack_Widget;
    QTabWidget *gPlanTab;
    /* CAN Configure */
    QPushButton *button_CanConnect;
    QPushButton *button_CanOpen;
    QPushButton *button_CanClose;
//    WinZlgCan mWinZlgCan;
//    CanRevWorkThread mCanRevWorkThread;

    // 跟踪
    GeometricTrack mGeometricTrack;
    // 配置
    VehilceConfig mVehilceConfig;
    //HMI
    Terminal *mTerminal;
    Simulation *mSimulation;
    BoRuiMessage *mBoRuiMessage;
    BoRuiController *mBoRuiController;
    // 感知
    Percaption mPercaption;
    UltrasonicObstaclePercption mUltrasonicObstaclePercption;
    // 规划
    ParallelPlanning *mParallelPlanning;
    VerticalPlanning *mVerticalPlanning;
    Curvature        *mCurvature;
    HC_ReedsSheppStateSpace *mHC_ReedsSheppStateSpace;
//    OMPL_Planner *m_OMPL_Planner;
    /**
     * @brief 箭头显示相关参数
     */
    State mBaseState[2];
    State mHeadState[2];
    int m_base_state_index, m_head_state_index;

    /**
     * @brief 障碍物显示和配置相关参数
     */
//    uint8_t ObstacleArray[BOUNDARY_X_SIZE][BOUNDARY_Y_SIZE];
    uint8_t selected_grid_colour;

    // 控制
    LatControl *mLatControl;
    LatControl_LQR *m_LatControl_LQR;

    TrackLinkList *_target_curvature_data_sets;
    std::vector<TargetTrack> *_target_curvature_vectors;
    TrajectoryAnalyzer *m_TrajectoryAnalyzer;

    // Detect Module
    Ultrasonic mUltrasonic;
    QList<Ultrasonic_Packet> LRU_List[12];
    QList<ObstacleLocationPacket> LRU_PositionList[12];
    QList<ObstacleLocationPacket> ObstacleBody_List[4];
    ObstacleLocationPacket temp_obstacle_body;

    Vector2d vehicle_last_position[4];
    ParkingEdgeBufferLocationPacket _ultrasonic_data_buffer[4];

    QList<GeometricTrack> VehicleTrackList;
    uint8_t NewFileUpdateFlag;
    int32_t time_step_cnt;
    Vector2d FrontLeftPoint,FrontRightPoint,RearLeftPoint,RearRightPoint;


private slots:
    // 功能选择激活函数图片选择的槽函数
    void sProcessItemActiveState(QListWidgetItem *current, QListWidgetItem *previous);

    // 50ms Task
    void sTimer20msTask(void);
    void sTimer20ms_Control(void);

    // CAN 配置相关的槽函数
    void sCAN_Connect(void);
    void sCAN_Open(void);
    void sCAN_Close(void);

    // 障碍物感知信息
//    void sDisplayPercaption(Percaption *p);

    // 感知区域的文件数据载入
    void sPercaptionDataFileSelect(void);

    // 感知检测相关计算
    void sCalculateDetect(void);

    //规划
    // click check buttom
    void sParallelPlanSelect();
    void sVerticalPlanSelect();

    void sParkingConfirmG1();
    void sParkingConfirmHC();
    void sParkingConfirmG2();

    void sPathCirclePoint(uint8_t id,Circle *c);

    void sPathGenarate(void);

    // 鼠标拖拽功能
    /**
     * @brief 鼠标按下事件处理
     * @param event
     */
    void sMousePressEvent(QMouseEvent *event);

    /**
     * @brief 鼠标释放事件处理
     * @param event
     */
    void sMouseReleaseEvent(QMouseEvent *event);

    /**
     * @brief 鼠标移动事件处理
     * @param event
     */
    void sMouseMoveEvent(QMouseEvent *event);
    // 跟踪模块
    void sTrackStart(void);
};

#endif // MAINWINDOW_H
