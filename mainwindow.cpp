#include "mainwindow.h"
#include "ui_mainwindow.h"

using Eigen::MatrixXd;
using namespace math;
using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    Init();
    ui->setupUi(this);
    // 重新配置窗体大小
    this->resize(1500,845);

    /****************** Control UI ******************/
    ControlUI();
    /****************** Detect UI ******************/
    DetectUI();
    /****************** plan UI ******************/
    G1_PlanUI();
    HC_PlanUI();
    G2_PlanUI();
    /****************** Track UI ******************/
    TrackUI();

    /********************** 申请功能图标列表 ****************************/
    list_function = new QListWidget();
    /* 车辆控制 */
    QListWidgetItem *control_item = new QListWidgetItem(QIcon(":/Icon/unactive_car.png"),tr("控制"));
    list_function->addItem(control_item);
    /* 检测模块 */
    QListWidgetItem *detect_item = new QListWidgetItem(QIcon(":/Icon/unactive_detect.png"),tr("检测"));
    list_function->addItem(detect_item);
    /* 路径规划模块 */
    QListWidgetItem *plan_item = new QListWidgetItem(QIcon(":/Icon/unactive_path.png"),tr("规划"));
    list_function->addItem(plan_item);
    /* 路径跟踪模块 */
    QListWidgetItem *track_item = new QListWidgetItem(QIcon(":/Icon/unactive_track.png"),tr("跟踪"));
    list_function->addItem(track_item);

    /*整体列表配置*/
    list_function->setViewMode(QListWidget::IconMode);//设置显示模式为图片模式
    list_function->setDragEnabled(false);//控件不允许拖动
    list_function->setFlow(QListWidget::TopToBottom);//设置元素排列方式从上往下
    list_function->setResizeMode(QListWidget::Adjust);//
    list_function->setMovement(QListWidget::Static);

    list_function->setIconSize(QSize(60,60));
    list_function->setMinimumHeight(100);
    list_function->setFixedWidth(70);

    list_function->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);//关闭水平滚动条
    list_function->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);//关闭垂直滚动条

    /****************** 申请各个功能模块的部件 ******************/
    QWidget *pControlWidget = new QWidget();
    QWidget *pDetectWidget  = new QWidget();
    QWidget *pG1_PlanWidget = new QWidget();
    QWidget *pHC_PlanWidget = new QWidget();
    QWidget *pG2_PlanWidget = new QWidget();
    QWidget *pTrackWidget   = new QWidget();

    gPlanTab = new QTabWidget(); //Plan Tab

    pControlWidget->setLayout(gControlLayout);
    pDetectWidget->setLayout(gDetectLayout);
    pG1_PlanWidget->setLayout(gG1_PlanLayout);
    pHC_PlanWidget->setLayout(gHC_PlanLayout);
    pG2_PlanWidget->setLayout(gG2_PlanLayout);
    pTrackWidget->setLayout(gTrackLayout);

    gPlanTab->addTab(pG1_PlanWidget, "G1 Plan");
    gPlanTab->addTab(pHC_PlanWidget, "HC Path");
    gPlanTab->addTab(pG2_PlanWidget, "G2 Plan");

    stack_Widget = new QStackedWidget();
    stack_Widget->addWidget(pControlWidget);
    stack_Widget->addWidget(pDetectWidget);
    stack_Widget->addWidget(gPlanTab);
    stack_Widget->addWidget(pTrackWidget);

    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(list_function,0,0);
    pMainLayout->addWidget(stack_Widget,0,1);
    // 设置第0列与第1列的比例为1：9
    pMainLayout->setColumnStretch(0,1);
    pMainLayout->setColumnStretch(1,9);
    pMainLayout->setColumnMinimumWidth(0,70);

    QWidget *PMainWidget = new QWidget();
    PMainWidget->setLayout(pMainLayout);

    setCentralWidget(PMainWidget);
    /****************** single connect slot *********************/
    /*** List Widget ***/
    // list 选择变动与 stack widget 界面更新的连接
    connect(list_function,SIGNAL(currentRowChanged(int)),stack_Widget,SLOT(setCurrentIndex(int)));
    // list 的元素变化时相应图片激活状态切换
    connect(list_function,SIGNAL(currentItemChanged(QListWidgetItem*,QListWidgetItem*)),this,SLOT(sProcessItemActiveState(QListWidgetItem*,QListWidgetItem*)));

    connect(&mDataTimer20ms,SIGNAL(timeout()),this,SLOT(sTimer20msTask()));
    connect(button_timer_control,SIGNAL(clicked()),this,SLOT(sTimer20ms_Control()));

    /*** CAN Configure single connect ***/
    // CAN Connect single and the Slot
    connect(button_CanConnect,SIGNAL(clicked()),this,SLOT(sCAN_Connect()));
    // CAN Open single and the Slot
    connect(button_CanOpen,SIGNAL(clicked()),this,SLOT(sCAN_Open()));
    // CAN Close single and the Slot
    connect(button_CanClose,SIGNAL(clicked()),this,SLOT(sCAN_Close()));

//    connect(&mCanRevWorkThread,SIGNAL(SendPercaptionMessage(Percaption*)),this,SLOT(sDisplayPercaption(Percaption*)));

    /*** 感知检测 ***/
    connect(button_file_select,SIGNAL(clicked()),this,SLOT(sPercaptionDataFileSelect()));
    connect(button_start_calculate,SIGNAL(clicked()),this,SLOT(sCalculateDetect()));

    /*** 路径规划 ***/
    connect(radio_parallel,SIGNAL(clicked()),this,SLOT(sParallelPlanSelect()));
    connect(radio_vertical,SIGNAL(clicked()),this,SLOT(sVerticalPlanSelect()));

    connect(gParkingInformationConfirmG1,SIGNAL(clicked()),this,SLOT(sParkingConfirmG1()));
    connect(gParkingInformationConfirmHC,SIGNAL(clicked()),this,SLOT(sParkingConfirmHC()));
    connect(gParkingInformationConfirmG2,SIGNAL(clicked()),this,SLOT(sParkingConfirmG2()));

    connect(mParallelPlanning,SIGNAL(sCircleCenterPoint(uint8_t,Circle*)),this,SLOT(sPathCirclePoint(uint8_t,Circle*)));
    connect(mVerticalPlanning,SIGNAL(sCircleCenterPoint(uint8_t,Circle*)),this,SLOT(sPathCirclePoint(uint8_t,Circle*)));

    connect(mHC_PathPlot, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(sMousePressEvent(QMouseEvent*)));
    connect(mHC_PathPlot, SIGNAL(mouseRelease(QMouseEvent*)), this, SLOT(sMouseReleaseEvent(QMouseEvent*)));
    connect(mHC_PathPlot, SIGNAL(mouseMove(QMouseEvent*)), this, SLOT(sMouseMoveEvent(QMouseEvent*)));
    /*** 轨迹跟踪 ***/
    connect(button_patn_generate,SIGNAL(clicked()),this,SLOT(sPathGenarate()));
    connect(button_track_start,SIGNAL(clicked()),this,SLOT(sTrackStart()));

    // 定时器20ms
    mDataTimer20ms.start(20);
}

MainWindow::~MainWindow()
{
//    mCanRevWorkThread.quit();
//    mWinZlgCan.CanClose();
    delete ui;
}

/**
 * @brief MainWindow::ControlUI for vehicle control
 */
void MainWindow::ControlUI(void)
{
    /* CAN 配置 Group */
    button_CanConnect = new QPushButton(tr("连接"));
    button_CanOpen    = new QPushButton(tr("打开"));
    button_CanClose   = new QPushButton(tr("复位"));

    QVBoxLayout *gCanButtonLayout = new QVBoxLayout();
    gCanButtonLayout->addWidget(button_CanConnect);
    gCanButtonLayout->addWidget(button_CanOpen);
    gCanButtonLayout->addWidget(button_CanClose);
    gCanButtonLayout->setMargin(3);

    QGroupBox *gCanGroup = new QGroupBox();
    gCanGroup->setTitle("CAN Configure");
    gCanGroup->setFixedHeight(120);
    gCanGroup->setLayout(gCanButtonLayout);

    /* 车辆状态显示 Group */
    QLabel *label_VCU_Text = new QLabel();
    label_VCU_Text->setText("VCU:");
    QLabel *label_VCU_Value = new QLabel();
    label_VCU_Value->setText("0");

    QLabel *label_ESC_Text = new QLabel();
    label_ESC_Text->setText("ESC:");
    QLabel *label_ESC_Value = new QLabel();
    label_ESC_Value->setText("0");

    QLabel *label_EPS_Text = new QLabel();
    label_EPS_Text->setText("EPS:");
    QLabel *label_EPS_Value = new QLabel();
    label_EPS_Value->setText("0");

    QGridLayout *gVehicleStatusLayout = new QGridLayout();
    gVehicleStatusLayout->addWidget(label_VCU_Text,0,0);
    gVehicleStatusLayout->addWidget(label_VCU_Value,0,1);
    gVehicleStatusLayout->addWidget(label_ESC_Text,1,0);
    gVehicleStatusLayout->addWidget(label_ESC_Value,1,1);
    gVehicleStatusLayout->addWidget(label_EPS_Text,2,0);
    gVehicleStatusLayout->addWidget(label_EPS_Value,2,1);
    gVehicleStatusLayout->setColumnMinimumWidth(0,30);

    QGroupBox *gVehicleStatusGroup = new QGroupBox();
    gVehicleStatusGroup->setTitle("Status");
    gVehicleStatusGroup->setFixedHeight(100);
    gVehicleStatusGroup->setLayout(gVehicleStatusLayout);

    gVehicleStatusGroup->setStyleSheet("QGroupBox{color:green;\
                                                  font-weight:20px;\
                                                  font:bold;\
                                                 }");

//    gVehicleStatusGroup->setStyleSheet(QString("QGroupBox{subcontrol-origin: margin;subcontrol-position: top center;padding: 0 3px;background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,stop: 0 #FF0ECE, stop: 1 #FFFFFF);}"));
    /* 车辆转向显示 Group */
    QLabel *label_Angle_Text = new QLabel();
    label_Angle_Text->setText("角度:");
    QLabel *label_Angle_Value = new QLabel();
    label_Angle_Value->setText("0");

    QLabel *label_AngularVelocity_Text = new QLabel();
    label_AngularVelocity_Text->setText("角速度:");
    QLabel *label_AngularVelocity_Value = new QLabel();
    label_AngularVelocity_Value->setText("0");

    QGridLayout *gVehicleEPSLayout = new QGridLayout();
    gVehicleEPSLayout->addWidget(label_Angle_Text,0,0);
    gVehicleEPSLayout->addWidget(label_Angle_Value,0,1);
    gVehicleEPSLayout->addWidget(label_AngularVelocity_Text,1,0);
    gVehicleEPSLayout->addWidget(label_AngularVelocity_Value,1,1);

    QGroupBox *gVehicleEPSGroup = new QGroupBox();
    gVehicleEPSGroup->setTitle("EPS");
    gVehicleEPSGroup->setFixedHeight(90);
    gVehicleEPSGroup->setLayout(gVehicleEPSLayout);

    /* 车辆速度显示 Group */
    QLabel *label_VehicleVelocity_Text = new QLabel();
    label_VehicleVelocity_Text->setText("车速:");
    QLabel *label_VehicleVelocity_Value = new QLabel();
    label_VehicleVelocity_Value->setText("0");

    QLabel *label_RearLeftWheelVelocity_Text = new QLabel();
    label_RearLeftWheelVelocity_Text->setText("左轮速:");
    QLabel *label_RearLeftWheelVelocity_Value = new QLabel();
    label_RearLeftWheelVelocity_Value->setText("0");

    QLabel *label_RearRightWheelVelocity_Text = new QLabel();
    label_RearRightWheelVelocity_Text->setText("右轮速:");
    QLabel *label_RearRightWheelVelocity_Value = new QLabel();
    label_RearRightWheelVelocity_Value->setText("0");

    QLabel *label_RearLeftWheelPulse_Text = new QLabel();
    label_RearLeftWheelPulse_Text->setText("左脉冲:");
    QLabel *label_RearLeftWheelPulse_Value = new QLabel();
    label_RearLeftWheelPulse_Value->setText("0");

    QLabel *label_RearRightWheelPulse_Text = new QLabel();
    label_RearRightWheelPulse_Text->setText("右脉冲:");
    QLabel *label_RearRightWheelPulse_Value = new QLabel();
    label_RearRightWheelPulse_Value->setText("0");

    QGridLayout *gVehicleVelocityLayout = new QGridLayout();
    gVehicleVelocityLayout->addWidget(label_VehicleVelocity_Text,0,0);
    gVehicleVelocityLayout->addWidget(label_VehicleVelocity_Value,0,1);
    gVehicleVelocityLayout->addWidget(label_RearLeftWheelVelocity_Text,1,0);
    gVehicleVelocityLayout->addWidget(label_RearLeftWheelVelocity_Value,1,1);
    gVehicleVelocityLayout->addWidget(label_RearRightWheelVelocity_Text,2,0);
    gVehicleVelocityLayout->addWidget(label_RearRightWheelVelocity_Value,2,1);
    gVehicleVelocityLayout->addWidget(label_RearLeftWheelPulse_Text,3,0);
    gVehicleVelocityLayout->addWidget(label_RearLeftWheelPulse_Value,3,1);
    gVehicleVelocityLayout->addWidget(label_RearRightWheelPulse_Text,4,0);
    gVehicleVelocityLayout->addWidget(label_RearRightWheelPulse_Value,4,1);

    QGroupBox *gVehicleVelocityGroup = new QGroupBox();
    gVehicleVelocityGroup->setTitle("Velocity");
    gVehicleVelocityGroup->setFixedHeight(160);
    gVehicleVelocityGroup->setLayout(gVehicleVelocityLayout);

    /* vehilce motion group */
    QLabel *label_Lon_Text = new QLabel();
    label_Lon_Text->setText("Lon:");
    QLabel *label_Lon_Value = new QLabel();
    label_Lon_Value->setText("0");

    QLabel *label_Lat_Text = new QLabel();
    label_Lat_Text->setText("Lat:");
    QLabel *label_Lat_Value = new QLabel();
    label_Lat_Value->setText("0");

    QLabel *label_Yaw_Text = new QLabel();
    label_Yaw_Text->setText("Yaw:");
    QLabel *label_Yaw_Value = new QLabel();
    label_Yaw_Value->setText("0");

    QGridLayout *gVehicleMotionLayout = new QGridLayout();
    gVehicleMotionLayout->addWidget(label_Lon_Text,0,0);
    gVehicleMotionLayout->addWidget(label_Lon_Value,0,1);
    gVehicleMotionLayout->addWidget(label_Lat_Text,1,0);
    gVehicleMotionLayout->addWidget(label_Lat_Value,1,1);
    gVehicleMotionLayout->addWidget(label_Yaw_Text,2,0);
    gVehicleMotionLayout->addWidget(label_Yaw_Value,2,1);

    QGroupBox *gVehicleMotionGroup = new QGroupBox();
    gVehicleMotionGroup->setTitle("Motion");
    gVehicleMotionGroup->setFixedHeight(100);
    gVehicleMotionGroup->setLayout(gVehicleMotionLayout);

    /* Control Input Group */
    QLabel *label_TargetSteering_Text = new QLabel();
    label_TargetSteering_Text->setText("方向盘:");
    QLabel *label_TargetSteering_Value = new QLabel();
    label_TargetSteering_Value->setText("0");

    QLabel *label_TargetVelocity_Text = new QLabel();
    label_TargetVelocity_Text->setText("速度:");
    QLabel *label_TargetVelocity_Value = new QLabel();
    label_TargetVelocity_Value->setText("0");

    QLabel *label_TargetGear_Text = new QLabel();
    label_TargetGear_Text->setText("挡位:");
    QLabel *label_TargetGear_Value = new QLabel();
    label_TargetGear_Value->setText("0");

    QLabel *label_TargetTorque_Text = new QLabel();
    label_TargetTorque_Text->setText("扭矩:");
    QLabel *label_TargetTorque_Value = new QLabel();
    label_TargetTorque_Value->setText("0");

    QLabel *label_TargetAcc_Text = new QLabel();
    label_TargetAcc_Text->setText("加速度:");
    QLabel *label_TargetAcc_Value = new QLabel();
    label_TargetAcc_Value->setText("0");

    QGridLayout *gVehicleControlInputLayout = new QGridLayout();
    gVehicleControlInputLayout->addWidget(label_TargetSteering_Text,0,0);
    gVehicleControlInputLayout->addWidget(label_TargetSteering_Value,0,1);
    gVehicleControlInputLayout->addWidget(label_TargetVelocity_Text,1,0);
    gVehicleControlInputLayout->addWidget(label_TargetVelocity_Value,1,1);
    gVehicleControlInputLayout->addWidget(label_TargetGear_Text,2,0);
    gVehicleControlInputLayout->addWidget(label_TargetGear_Value,2,1);
    gVehicleControlInputLayout->addWidget(label_TargetTorque_Text,3,0);
    gVehicleControlInputLayout->addWidget(label_TargetTorque_Value,3,1);
    gVehicleControlInputLayout->addWidget(label_TargetAcc_Text,4,0);
    gVehicleControlInputLayout->addWidget(label_TargetAcc_Value,4,1);

    QGroupBox *gVehicleControlInputGroup = new QGroupBox();
    gVehicleControlInputGroup->setTitle("Control");
    gVehicleControlInputGroup->setFixedHeight(160);
    gVehicleControlInputGroup->setLayout(gVehicleControlInputLayout);

    /********************* Control IO Layout *********************/
    QGridLayout *gControl_IO_Layout = new QGridLayout();
    gControl_IO_Layout->addWidget(gCanGroup,0,0);
    gControl_IO_Layout->addWidget(gVehicleStatusGroup,1,0);
    gControl_IO_Layout->addWidget(gVehicleEPSGroup,2,0);
    gControl_IO_Layout->addWidget(gVehicleVelocityGroup,3,0);
    gControl_IO_Layout->addWidget(gVehicleMotionGroup,4,0);
    gControl_IO_Layout->addWidget(gVehicleControlInputGroup,5,0);
//    gControl_IO_Layout->addWidget(button_timer_control,6,0);
    gControl_IO_Layout->setRowStretch(0,1);
    gControl_IO_Layout->setRowStretch(1,1);
    gControl_IO_Layout->setRowStretch(2,1);
    gControl_IO_Layout->setRowStretch(3,1);
    gControl_IO_Layout->setRowStretch(4,1);
    gControl_IO_Layout->setRowStretch(5,1);
    gControl_IO_Layout->setRowStretch(6,1);
//    gControl_IO_Layout->setRowStretch(7,1);
    // Control Plot 元素
    mControlPlot = new QCustomPlot();

    // configure plot to have two right axes:
    mControlPlot->yAxis->setTickLabels(false);
    connect(mControlPlot->yAxis2, SIGNAL(rangeChanged(QCPRange)), mControlPlot->yAxis, SLOT(setRange(QCPRange))); // left axis only mirrors inner right axis
    mControlPlot->yAxis2->setVisible(true);
    mControlPlot->axisRect()->addAxis(QCPAxis::atRight);
    mControlPlot->axisRect()->axis(QCPAxis::atRight, 0)->setPadding(30); // add some padding to have space for tags
    mControlPlot->axisRect()->axis(QCPAxis::atRight, 1)->setPadding(30); // add some padding to have space for tags

    mControlPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    // create graphs:
    mControlGraph1 = mControlPlot->addGraph(mControlPlot->xAxis, mControlPlot->axisRect()->axis(QCPAxis::atRight, 0));
    mControlGraph2 = mControlPlot->addGraph(mControlPlot->xAxis, mControlPlot->axisRect()->axis(QCPAxis::atRight, 1));
    mControlGraph1->setPen(QPen(QColor(250, 120, 0)));
    mControlGraph2->setPen(QPen(QColor(0, 180, 60)));

    // create tags with newly introduced AxisTag class (see axistag.h/.cpp):
    mControlTag1 = new AxisTag(mControlGraph1->valueAxis());
    mControlTag1->setPen(mControlGraph1->pen());
    mControlTag2 = new AxisTag(mControlGraph2->valueAxis());
    mControlTag2->setPen(mControlGraph2->pen());

    gControlLayout = new QGridLayout();
    gControlLayout->addLayout(gControl_IO_Layout, 0, 0);
    gControlLayout->addWidget(mControlPlot, 0, 1);
    gControlLayout->setColumnStretch(0,1);
    gControlLayout->setColumnStretch(1,10);
    gControlLayout->setColumnMinimumWidth(0,100);
}

/**
 * @brief MainWindow::DetectUI for ultrasonic detect
 */
void MainWindow::DetectUI(void)
{
    // 超声避障距离显示组件
    label_FrontObstacle_Text = new QLabel();
    label_FrontObstacle_Text->setText("前:");
    label_FrontObstacleDistance_Value = new QLabel();
    label_FrontObstacleDistance_Value->setText("0");
    label_FrontObstacleRegion_Value = new QLabel();
    label_FrontObstacleRegion_Value->setText("中间");
    label_FrontObstacleStatus_Value = new QLabel();
    label_FrontObstacleStatus_Value->setText("正常");

    label_RearObstacle_Text = new QLabel();
    label_RearObstacle_Text->setText("后:");
    label_RearObstacleDistance_Value = new QLabel();
    label_RearObstacleDistance_Value->setText("0");
    label_RearObstacleRegion_Value = new QLabel();
    label_RearObstacleRegion_Value->setText("中间");
    label_RearObstacleStatus_Value = new QLabel();
    label_RearObstacleStatus_Value->setText("正常");

    QGridLayout *gObstacleDistanceLayout = new QGridLayout();
    gObstacleDistanceLayout->addWidget(label_FrontObstacle_Text,0,0);
    gObstacleDistanceLayout->addWidget(label_FrontObstacleDistance_Value,0,1);
    gObstacleDistanceLayout->addWidget(label_FrontObstacleRegion_Value,0,2);
    gObstacleDistanceLayout->addWidget(label_FrontObstacleStatus_Value,0,3);
    gObstacleDistanceLayout->addWidget(label_RearObstacle_Text,1,0);
    gObstacleDistanceLayout->addWidget(label_RearObstacleDistance_Value,1,1);
    gObstacleDistanceLayout->addWidget(label_RearObstacleRegion_Value,1,2);
    gObstacleDistanceLayout->addWidget(label_RearObstacleStatus_Value,1,3);

    gObstacleDistanceLayout->setColumnStretch(0,1);
    gObstacleDistanceLayout->setColumnStretch(1,3);
    gObstacleDistanceLayout->setColumnStretch(2,2);
    gObstacleDistanceLayout->setColumnStretch(3,2);

    QGroupBox *gObstacleDistance = new QGroupBox();
    gObstacleDistance->setTitle("避障距离");
    gObstacleDistance->setFixedHeight(90);
    gObstacleDistance->setLayout(gObstacleDistanceLayout);

    // 定位模式选择组件
    radio_right_enter_location = new QRadioButton();
    radio_right_enter_location->setText("右侧入库");
    radio_left_enter_location = new QRadioButton();
    radio_left_enter_location->setText("左侧入库");
    radio_center_enter_location = new QRadioButton();
    radio_center_enter_location->setText("进库定位");

    QGridLayout *gLocationModeLayout = new QGridLayout();
    gLocationModeLayout->addWidget(radio_right_enter_location,0,0);
    gLocationModeLayout->addWidget(radio_left_enter_location,1,0);
    gLocationModeLayout->addWidget(radio_center_enter_location,2,0);

    gLocationModeLayout->setRowStretch(0,1);
    gLocationModeLayout->setRowStretch(1,1);
    gLocationModeLayout->setRowStretch(2,1);

    QGroupBox *gObstacleLocationMode = new QGroupBox();
    gObstacleLocationMode->setTitle("定位模式选择");
    gObstacleLocationMode->setFixedHeight(150);
    gObstacleLocationMode->setLayout(gLocationModeLayout);

    button_file_select = new QPushButton();
    button_file_select->setText("打开文件");

    button_start_calculate = new QPushButton();
    button_start_calculate->setText("开始计算");

    QGridLayout *gDetect_Show_Layout = new QGridLayout();
    gDetect_Show_Layout->addWidget(gObstacleDistance,0,0);
    gDetect_Show_Layout->addWidget(gObstacleLocationMode,1,0);
    gDetect_Show_Layout->addWidget(button_file_select,2,0);
    gDetect_Show_Layout->addWidget(button_start_calculate,3,0);

    gDetect_Show_Layout->setRowStretch(0,1);
    gDetect_Show_Layout->setRowStretch(1,1);
    gDetect_Show_Layout->setRowStretch(2,1);
    gDetect_Show_Layout->setRowStretch(3,1);
    gDetect_Show_Layout->setRowStretch(4,1);
    // 感知检测图形绘制 begin
    mDetectPlot = new QCustomPlot();
    mDetectPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    mDetectPlot->legend->setVisible(true);
    mDetectPlot->legend->setFont(QFont("Helvetica", 9));
//    mDetectPlot->legend->setRowSpacing(-3);
    mDetectPlot->xAxis->setLabel("x");
    mDetectPlot->yAxis->setLabel("y");
//    mDetectPlot->xAxis->setRange(BOUNDARY_LEFT,BOUNDARY_RIGHT);
//    mDetectPlot->yAxis->setRange(BOUNDARY_DOWN,BOUNDARY_TOP);

    mDetectVehicleModuleCurve = new QCPCurve(mDetectPlot->xAxis,mDetectPlot->yAxis);
    mDetectVehicleModuleCurve->setName("车辆模型");
    mDetectVehicleModuleCurve->setPen(QPen(Qt::red,3));
    mDetectVehicleModuleCurve->setBrush(QBrush(QColor(190,35,155,30)));

    mDetectVehicleCenterCurve = new QCPCurve(mDetectPlot->xAxis,mDetectPlot->yAxis);
    mDetectVehicleCenterCurve->setName("后轴中心");
    mDetectVehicleCenterCurve->setPen(QPen(Qt::gray,3));
    mDetectVehicleCenterCurve->setLineStyle(QCPCurve::lsNone);
    mDetectVehicleCenterCurve->setScatterStyle(QCPScatterStyle::ssStar);

    mDetectEdgePoint = new QCPCurve(mDetectPlot->xAxis,mDetectPlot->yAxis);
    mDetectEdgePoint->setName("障碍物边沿");
    mDetectEdgePoint->setPen(QPen(Qt::blue,3));
    mDetectEdgePoint->setLineStyle(QCPCurve::lsNone);
    mDetectEdgePoint->setScatterStyle(QCPScatterStyle::ssStar);

    mDetectValidEdgePoint = new QCPCurve(mDetectPlot->xAxis,mDetectPlot->yAxis);
    mDetectValidEdgePoint->setName("有效库位边沿点");
    mDetectValidEdgePoint->setPen(QPen(Qt::red,10));
    mDetectValidEdgePoint->setLineStyle(QCPCurve::lsNone);
    mDetectValidEdgePoint->setScatterStyle(QCPScatterStyle::ssCross);

    mDetectRearEdgeTrianglePosition = new QCPCurve(mDetectPlot->xAxis,mDetectPlot->yAxis);
    mDetectRearEdgeTrianglePosition->setName("后边沿定位");
    mDetectRearEdgeTrianglePosition->setPen(QPen(Qt::darkYellow,2));
    mDetectRearEdgeTrianglePosition->setLineStyle(QCPCurve::lsNone);
    mDetectRearEdgeTrianglePosition->setScatterStyle(QCPScatterStyle::ssStar);

    mDetectLeftEdgeGroundPosition = new QCPCurve(mDetectPlot->xAxis,mDetectPlot->yAxis);
    mDetectLeftEdgeGroundPosition->setName("左边沿");
    mDetectLeftEdgeGroundPosition->setPen(QPen(Qt::darkCyan,2));
    mDetectLeftEdgeGroundPosition->setLineStyle(QCPCurve::lsNone);
    mDetectLeftEdgeGroundPosition->setScatterStyle(QCPScatterStyle::ssStar);

    mDetectRightEdgeGroundPosition = new QCPCurve(mDetectPlot->xAxis,mDetectPlot->yAxis);
    mDetectRightEdgeGroundPosition->setName("右边沿");
    mDetectRightEdgeGroundPosition->setPen(QPen(Qt::darkRed,2));
    mDetectRightEdgeGroundPosition->setLineStyle(QCPCurve::lsNone);
    mDetectRightEdgeGroundPosition->setScatterStyle(QCPScatterStyle::ssStar);


    mDetectLeftEdgeFitLine = new QCPCurve(mDetectPlot->xAxis,mDetectPlot->yAxis);
    mDetectLeftEdgeFitLine->setName("左边沿拟合");
    mDetectLeftEdgeFitLine->setPen(QPen(Qt::darkGreen,2));
    mDetectLeftEdgeFitLine->setLineStyle(QCPCurve::lsLine);
    mDetectLeftEdgeFitLine->setScatterStyle(QCPScatterStyle::ssPlus);

    mDetectRightEdgeFitLine = new QCPCurve(mDetectPlot->xAxis,mDetectPlot->yAxis);
    mDetectRightEdgeFitLine->setName("右边沿拟合");
    mDetectRightEdgeFitLine->setPen(QPen(Qt::darkYellow,2));
    mDetectRightEdgeFitLine->setLineStyle(QCPCurve::lsLine);
    mDetectRightEdgeFitLine->setScatterStyle(QCPScatterStyle::ssPlus);

    // 感知检测图形绘制 end

    gDetectLayout = new QGridLayout();
    gDetectLayout->addLayout(gDetect_Show_Layout, 0,0);
    gDetectLayout->addWidget(mDetectPlot, 0, 1);
    gDetectLayout->setColumnStretch(0,1);
    gDetectLayout->setColumnStretch(1,9);
}

/**
 * @brief MainWindow::G1_PlanUI for one generation planning
 */
void MainWindow::G1_PlanUI(void)
{
    // Plan模式选择组件
    radio_parallel = new QRadioButton();
    radio_parallel->setText("Parallel Plan");
    radio_vertical = new QRadioButton();
    radio_vertical->setText("Vertical Plan");

    QGridLayout *gPlanTypeLayout = new QGridLayout();
    gPlanTypeLayout->addWidget(radio_parallel,0,0);
    gPlanTypeLayout->addWidget(radio_vertical,1,0);

    gPlanTypeLayout->setRowStretch(0,1);
    gPlanTypeLayout->setRowStretch(1,1);

    QGroupBox *gPlanType_Group = new QGroupBox();
    gPlanType_Group->setTitle("Planning Type");
    gPlanType_Group->setFixedHeight(100);
    gPlanType_Group->setLayout(gPlanTypeLayout);

    // 车辆初始位置 begin
    QLabel *label_VehicleInitPointX_Text = new QLabel();
    label_VehicleInitPointX_Text->setText("X:");
    QLabel *label_VehicleInitPointY_Text = new QLabel();
    label_VehicleInitPointY_Text->setText("Y:");
    QLabel *label_VehicleInitPointYaw_Text = new QLabel();
    label_VehicleInitPointYaw_Text->setText("Yaw:");

    text_VehicleInitPointX = new QLineEdit();
    text_VehicleInitPointX->setText("8");
    text_VehicleInitPointY = new QLineEdit();
    text_VehicleInitPointY->setText("3");
    text_VehicleInitPointYaw = new QLineEdit();
    text_VehicleInitPointYaw->setText("0");

    QGridLayout *gVehicleInitPosition_Layout = new QGridLayout();
    gVehicleInitPosition_Layout->addWidget(label_VehicleInitPointX_Text,0,0);
    gVehicleInitPosition_Layout->addWidget(label_VehicleInitPointY_Text,1,0);
    gVehicleInitPosition_Layout->addWidget(label_VehicleInitPointYaw_Text,2,0);

    gVehicleInitPosition_Layout->addWidget(text_VehicleInitPointX,0,1);
    gVehicleInitPosition_Layout->addWidget(text_VehicleInitPointY,1,1);
    gVehicleInitPosition_Layout->addWidget(text_VehicleInitPointYaw,2,1);

    QGroupBox *gVehicleInitPosition_Group = new QGroupBox();
    gVehicleInitPosition_Group->setTitle("车辆初始位置");
    gVehicleInitPosition_Group->setFixedHeight(120);
    gVehicleInitPosition_Group->setLayout(gVehicleInitPosition_Layout);
    // 车辆初始位置 end

    // 车位信息 begin
    QLabel *label_ParkingLength_Text = new QLabel();
    label_ParkingLength_Text->setText("长:");
    QLabel *label_ParkingWidth_Text = new QLabel();
    label_ParkingWidth_Text->setText("宽:");

    text_ParkingLength = new QLineEdit();
    text_ParkingLength->setText("0.0");
    text_ParkingWidth = new QLineEdit();
    text_ParkingWidth->setText("0.0");

    QGridLayout *gVehicleParking_Layout = new QGridLayout();
    gVehicleParking_Layout->addWidget(label_ParkingLength_Text,0,0);
    gVehicleParking_Layout->addWidget(label_ParkingWidth_Text,1,0);
    gVehicleParking_Layout->addWidget(text_ParkingLength,0,1);
    gVehicleParking_Layout->addWidget(text_ParkingWidth,1,1);
    gVehicleParking_Layout->setColumnStretch(0,1);
    gVehicleParking_Layout->setColumnStretch(1,3);

    QGroupBox *gVehicleParking_Group = new QGroupBox();
    gVehicleParking_Group->setTitle("车位信息");
    gVehicleParking_Group->setFixedHeight(100);
    gVehicleParking_Group->setLayout(gVehicleParking_Layout);
    // 车位信息 end

    gParkingInformationConfirmG1 = new QPushButton();
    gParkingInformationConfirmG1->setText("库位信息确认");

    // 实时车辆位置跟踪 begin
    QLabel *label_VehiceTrackX_Text = new QLabel();
    label_VehiceTrackX_Text->setText("X:");
    QLabel *label_VehiceTrackY_Text = new QLabel();
    label_VehiceTrackY_Text->setText("Y:");
    QLabel *label_VehiceTrackYaw_Text = new QLabel();
    label_VehiceTrackYaw_Text->setText("Yaw:");

    label_G1_VehiceTrackX_Value = new QLabel();
    label_G1_VehiceTrackX_Value->setText("0");
    label_G1_VehiceTrackY_Value = new QLabel();
    label_G1_VehiceTrackY_Value->setText("0");
    label_G1_VehiceTrackYaw_Value = new QLabel();
    label_G1_VehiceTrackYaw_Value->setText("0");//单位 度

    QLabel *label_VehiceTrackX_Unit = new QLabel();
    label_VehiceTrackX_Unit->setText("m");
    QLabel *label_VehiceTrackY_Unit = new QLabel();
    label_VehiceTrackY_Unit->setText("m");
    QLabel *label_VehiceTrackYaw_Unit = new QLabel();
    label_VehiceTrackYaw_Unit->setText("°");

    QGridLayout *gVehicleTrack_Layout = new QGridLayout();
    gVehicleTrack_Layout->addWidget(label_VehiceTrackX_Text,0,0);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackY_Text,1,0);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackYaw_Text,2,0);

    gVehicleTrack_Layout->addWidget(label_G1_VehiceTrackX_Value,0,1);
    gVehicleTrack_Layout->addWidget(label_G1_VehiceTrackY_Value,1,1);
    gVehicleTrack_Layout->addWidget(label_G1_VehiceTrackYaw_Value,2,1);

    gVehicleTrack_Layout->addWidget(label_VehiceTrackX_Unit,0,2);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackY_Unit,1,2);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackYaw_Unit,2,2);

    gVehicleTrack_Layout->setColumnStretch(0,2);
    gVehicleTrack_Layout->setColumnStretch(1,5);
    gVehicleTrack_Layout->setColumnStretch(1,1);

    QGroupBox *gVehicleTrack_Group = new QGroupBox();
    gVehicleTrack_Group->setTitle("实时跟踪");
    gVehicleTrack_Group->setFixedHeight(120);
    gVehicleTrack_Group->setLayout(gVehicleTrack_Layout);
    // 实时车辆位置跟踪 end

    QGridLayout *gPath_IO_Layout = new QGridLayout();
    gPath_IO_Layout->addWidget(gPlanType_Group, 0, 0);
    gPath_IO_Layout->addWidget(gVehicleInitPosition_Group, 1, 0);
    gPath_IO_Layout->addWidget(gVehicleParking_Group, 2, 0);
    gPath_IO_Layout->addWidget(gVehicleTrack_Group, 3, 0);
    gPath_IO_Layout->addWidget(gParkingInformationConfirmG1, 4, 0);
    gPath_IO_Layout->setColumnMinimumWidth(0,200);
    gPath_IO_Layout->setRowStretch(0,1);
    gPath_IO_Layout->setRowStretch(1,1);
    gPath_IO_Layout->setRowStretch(2,1);
    gPath_IO_Layout->setRowStretch(3,1);
    gPath_IO_Layout->setRowStretch(4,1);
    gPath_IO_Layout->setRowStretch(5,1);

    mG1_PathPlot = new QCustomPlot();
    mG1_PathPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    mG1_PathPlot->legend->setVisible(true);
    mG1_PathPlot->legend->setFont(QFont("Helvetica", 9));
    mG1_PathPlot->legend->setRowSpacing(-3);
    mG1_PathPlot->xAxis->setLabel("x");
    mG1_PathPlot->yAxis->setLabel("y");
    mG1_PathPlot->xAxis->setRange(-5,10);
    mG1_PathPlot->yAxis->setRange(-5,10);

    mPathVehicleModuleCurveG1 = new QCPCurve(mG1_PathPlot->xAxis,mG1_PathPlot->yAxis);
    mPathVehicleModuleCurveG1->setName("车辆模型");
    mPathVehicleModuleCurveG1->setPen(QPen(Qt::red,3));
    mPathVehicleModuleCurveG1->setBrush(QBrush(QColor(90,35,255,20)));

    mPathParkingCurveG1 = new QCPCurve(mG1_PathPlot->xAxis,mG1_PathPlot->yAxis);
    mPathParkingCurveG1->setName("库位");
    mPathParkingCurveG1->setPen(QPen(Qt::green,4));
    mPathParkingCurveG1->setBrush(QBrush(QColor(90,255,240,80)));

    mPathVehicleCenterCurveG1 = new QCPCurve(mG1_PathPlot->xAxis,mG1_PathPlot->yAxis);
    mPathVehicleCenterCurveG1->setName("后轴中心");

    mPathLeftCircle = new QCPItemEllipse(mG1_PathPlot);
    mPathLeftCircle->setPen(QPen(Qt::yellow,1));
    mPathLeftCircle->setBrush(QBrush(QColor(90,125,140,20)));

    mPathRightCircle = new QCPItemEllipse(mG1_PathPlot);
    mPathRightCircle->setPen(QPen(Qt::black,1));
    mPathRightCircle->setBrush(QBrush(QColor(20,25,140,20)));

    ParkingPointX.resize(9);
    ParkingPointY.resize(9);

    gG1_PlanLayout = new QGridLayout();
    gG1_PlanLayout->addLayout(gPath_IO_Layout, 0, 0);
    gG1_PlanLayout->addWidget(mG1_PathPlot, 0, 1);
    gG1_PlanLayout->setColumnMinimumWidth(0,180);
    gG1_PlanLayout->setColumnStretch(0,1);
    gG1_PlanLayout->setColumnStretch(1,6);
}

void MainWindow::HC_PlanUI(void)
{
    // 车辆初始位置 begin
    QLabel *label_VehicleStartPointX_Text = new QLabel();
    label_VehicleStartPointX_Text->setText("X:");
    QLabel *label_VehicleStartPointY_Text = new QLabel();
    label_VehicleStartPointY_Text->setText("Y:");
    QLabel *label_VehicleStartPointYaw_Text = new QLabel();
    label_VehicleStartPointYaw_Text->setText("Yaw:");
    QLabel *label_VehicleStartPointKappa_Text = new QLabel();
    label_VehicleStartPointKappa_Text->setText("Kappa:");

    HC_VehicleStartPointX = new QLineEdit();
    HC_VehicleStartPointX->setText("-6");
    HC_VehicleStartPointY = new QLineEdit();
    HC_VehicleStartPointY->setText("3");
    HC_VehicleStartPointYaw = new QLineEdit();
    HC_VehicleStartPointYaw->setText("0");
    HC_VehicleStartPointKappa = new QLineEdit();
    HC_VehicleStartPointKappa->setText("0");

    QGridLayout *gVehicleStartPosition_Layout = new QGridLayout();
    gVehicleStartPosition_Layout->addWidget(label_VehicleStartPointX_Text,0,0);
    gVehicleStartPosition_Layout->addWidget(label_VehicleStartPointY_Text,1,0);
    gVehicleStartPosition_Layout->addWidget(label_VehicleStartPointYaw_Text,2,0);
    gVehicleStartPosition_Layout->addWidget(label_VehicleStartPointKappa_Text,3,0);

    gVehicleStartPosition_Layout->addWidget(HC_VehicleStartPointX,0,1);
    gVehicleStartPosition_Layout->addWidget(HC_VehicleStartPointY,1,1);
    gVehicleStartPosition_Layout->addWidget(HC_VehicleStartPointYaw,2,1);
    gVehicleStartPosition_Layout->addWidget(HC_VehicleStartPointKappa,3,1);

    QGroupBox *gVehicleStartPosition_Group = new QGroupBox();
    gVehicleStartPosition_Group->setTitle("车辆初始位置");
    gVehicleStartPosition_Group->setFixedHeight(180);
    gVehicleStartPosition_Group->setLayout(gVehicleStartPosition_Layout);
    // 车辆初始位置 end

    // 车辆最终位置 begin
    QLabel *label_VehicleEndPointX_Text = new QLabel();
    label_VehicleEndPointX_Text->setText("X:");
    QLabel *label_VehicleEndPointY_Text = new QLabel();
    label_VehicleEndPointY_Text->setText("Y:");
    QLabel *label_VehicleEndPointYaw_Text = new QLabel();
    label_VehicleEndPointYaw_Text->setText("Yaw:");
    QLabel *label_VehicleEndPointKappa_Text = new QLabel();
    label_VehicleEndPointKappa_Text->setText("Kappa:");

    HC_VehicleEndPointX = new QLineEdit();
    HC_VehicleEndPointX->setText("6");
    HC_VehicleEndPointY = new QLineEdit();
    HC_VehicleEndPointY->setText("-2");
    HC_VehicleEndPointYaw = new QLineEdit();
    HC_VehicleEndPointYaw->setText("0");
    HC_VehicleEndPointKappa = new QLineEdit();
    HC_VehicleEndPointKappa->setText("0");

    QGridLayout *gVehicleEndPosition_Layout = new QGridLayout();
    gVehicleEndPosition_Layout->addWidget(label_VehicleEndPointX_Text,0,0);
    gVehicleEndPosition_Layout->addWidget(label_VehicleEndPointY_Text,1,0);
    gVehicleEndPosition_Layout->addWidget(label_VehicleEndPointYaw_Text,2,0);
    gVehicleEndPosition_Layout->addWidget(label_VehicleEndPointKappa_Text,3,0);

    gVehicleEndPosition_Layout->addWidget(HC_VehicleEndPointX,0,1);
    gVehicleEndPosition_Layout->addWidget(HC_VehicleEndPointY,1,1);
    gVehicleEndPosition_Layout->addWidget(HC_VehicleEndPointYaw,2,1);
    gVehicleEndPosition_Layout->addWidget(HC_VehicleEndPointKappa,3,1);

    QGroupBox *gVehicleEndPosition_Group = new QGroupBox();
    gVehicleEndPosition_Group->setTitle("车辆目标位置");
    gVehicleEndPosition_Group->setFixedHeight(180);
    gVehicleEndPosition_Group->setLayout(gVehicleEndPosition_Layout);
    // 车辆最终位置 end

    gParkingInformationConfirmHC = new QPushButton();
    gParkingInformationConfirmHC->setText("信息确认");

    QGridLayout *gHC_Path_IO_Layout = new QGridLayout();
    gHC_Path_IO_Layout->addWidget(gVehicleStartPosition_Group,0,0);
    gHC_Path_IO_Layout->addWidget(gVehicleEndPosition_Group,1,0);
    gHC_Path_IO_Layout->addWidget(gParkingInformationConfirmHC,2,0);
    gHC_Path_IO_Layout->setColumnMinimumWidth(0,200);
    gHC_Path_IO_Layout->setRowStretch(0,1);
    gHC_Path_IO_Layout->setRowStretch(1,1);
    gHC_Path_IO_Layout->setRowStretch(2,1);
    gHC_Path_IO_Layout->setRowStretch(3,1);

    /**
     * Plot Configure
     */
    mHC_PathPlot = new QCustomPlot();
    mHC_PathPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    mHC_PathPlot->legend->setVisible(true);
    mHC_PathPlot->legend->setFont(QFont("Helvetica", 9));
    mHC_PathPlot->legend->setRowSpacing(-3);
    mHC_PathPlot->xAxis->setLabel("x");
    mHC_PathPlot->yAxis->setLabel("y");
    mHC_PathPlot->xAxis->setRange(-5,10);
    mHC_PathPlot->yAxis->setRange(-5,10);

    mStartArrow = new QCPItemLine(mHC_PathPlot);
    mStartArrow->setHead(QCPLineEnding::esSpikeArrow);
    mStartArrow->setPen(QPen(Qt::darkRed,5));

    mEndArrow = new QCPItemLine(mHC_PathPlot);
    mEndArrow->setHead(QCPLineEnding::esSpikeArrow);
    mEndArrow->setPen(QPen(Qt::darkGreen,5));

    QPen RedDotPen, GreenDotPen, blueDotPen, BlackDotPen;
    RedDotPen.setColor(Qt::red);
    RedDotPen.setStyle(Qt::DotLine);
    RedDotPen.setWidthF(5);

    GreenDotPen.setColor(Qt::green);
    GreenDotPen.setStyle(Qt::DotLine);
    GreenDotPen.setWidthF(5);

    blueDotPen.setColor(QColor(30, 40, 255, 150));
    blueDotPen.setStyle(Qt::DotLine);
    blueDotPen.setWidthF(5);

    BlackDotPen.setColor(Qt::black);
    BlackDotPen.setStyle(Qt::DotLine);
    BlackDotPen.setWidthF(10);

    mPathPlanningStraightLine = new QCPCurve(mHC_PathPlot->xAxis,mHC_PathPlot->yAxis);
    mPathPlanningStraightLine->setName("直线段");
    mPathPlanningStraightLine->setPen(RedDotPen);
    mPathPlanningStraightLine->setLineStyle(QCPCurve::LineStyle::lsNone);
    mPathPlanningStraightLine->setScatterStyle(QCPScatterStyle::ssDot);

    mPathPlanningClothoid = new QCPCurve(mHC_PathPlot->xAxis,mHC_PathPlot->yAxis);
    mPathPlanningClothoid->setName("回旋段");
    mPathPlanningClothoid->setPen(GreenDotPen);
    mPathPlanningClothoid->setLineStyle(QCPCurve::LineStyle::lsNone);
    mPathPlanningClothoid->setScatterStyle(QCPScatterStyle::ssDot);

    mPathPlanningCircle = new QCPCurve(mHC_PathPlot->xAxis,mHC_PathPlot->yAxis);
    mPathPlanningCircle->setName("圆弧段");
    mPathPlanningCircle->setPen(blueDotPen);
    mPathPlanningCircle->setLineStyle(QCPCurve::LineStyle::lsNone);
    mPathPlanningCircle->setScatterStyle(QCPScatterStyle::ssDot);

    mTangentCirclePoint = new QCPCurve(mHC_PathPlot->xAxis,mHC_PathPlot->yAxis);
    mTangentCirclePoint->setName("切点");
    mTangentCirclePoint->setPen(BlackDotPen);
    mTangentCirclePoint->setLineStyle(QCPCurve::LineStyle::lsNone);
    mTangentCirclePoint->setScatterStyle(QCPScatterStyle::ssDot);

    mStartCircle = new QCPItemEllipse(mHC_PathPlot);
    mStartCircle->setPen(QPen(Qt::red,1));
    mStartCircle->setBrush(QBrush(QColor(90,125,140,20)));

    mEndCircle = new QCPItemEllipse(mHC_PathPlot);
    mEndCircle->setPen(QPen(Qt::green,1));
    mEndCircle->setBrush(QBrush(QColor(60,125,140,20)));

    mMiddleCircle1 = new QCPItemEllipse(mHC_PathPlot);
    mMiddleCircle1->setPen(QPen(Qt::darkBlue,1));
    mMiddleCircle1->setBrush(QBrush(QColor(30,125,140,20)));

    mMiddleCircle2 = new QCPItemEllipse(mHC_PathPlot);
    mMiddleCircle2->setPen(QPen(Qt::darkYellow,1));
    mMiddleCircle2->setBrush(QBrush(QColor(10,125,100,20)));

    gHC_PlanLayout = new QGridLayout();
    gHC_PlanLayout->addLayout(gHC_Path_IO_Layout, 0, 0);
    gHC_PlanLayout->addWidget(mHC_PathPlot, 0, 1);
    gHC_PlanLayout->setColumnMinimumWidth(0,180);
    gHC_PlanLayout->setColumnStretch(0,1);
    gHC_PlanLayout->setColumnStretch(1,6);
}

void MainWindow::G2_PlanUI(void)
{
    // 车辆初始位置 begin
    QLabel *label_VehicleStartPointX_Text = new QLabel();
    label_VehicleStartPointX_Text->setText("X:");
    QLabel *label_VehicleStartPointY_Text = new QLabel();
    label_VehicleStartPointY_Text->setText("Y:");
    QLabel *label_VehicleStartPointYaw_Text = new QLabel();
    label_VehicleStartPointYaw_Text->setText("Yaw:");
    QLabel *label_VehicleStartPointKappa_Text = new QLabel();
    label_VehicleStartPointKappa_Text->setText("Kappa:");

    text_VehicleStartPointX = new QLineEdit();
    text_VehicleStartPointX->setText("-6");
    text_VehicleStartPointY = new QLineEdit();
    text_VehicleStartPointY->setText("3");
    text_VehicleStartPointYaw = new QLineEdit();
    text_VehicleStartPointYaw->setText("0");
    text_VehicleStartPointKappa = new QLineEdit();
    text_VehicleStartPointKappa->setText("0");

    QGridLayout *gVehicleStartPosition_Layout = new QGridLayout();
    gVehicleStartPosition_Layout->addWidget(label_VehicleStartPointX_Text,0,0);
    gVehicleStartPosition_Layout->addWidget(label_VehicleStartPointY_Text,1,0);
    gVehicleStartPosition_Layout->addWidget(label_VehicleStartPointYaw_Text,2,0);
    gVehicleStartPosition_Layout->addWidget(label_VehicleStartPointKappa_Text,3,0);

    gVehicleStartPosition_Layout->addWidget(text_VehicleStartPointX,0,1);
    gVehicleStartPosition_Layout->addWidget(text_VehicleStartPointY,1,1);
    gVehicleStartPosition_Layout->addWidget(text_VehicleStartPointYaw,2,1);
    gVehicleStartPosition_Layout->addWidget(text_VehicleStartPointKappa,3,1);

    QGroupBox *gVehicleStartPosition_Group = new QGroupBox();
    gVehicleStartPosition_Group->setTitle("车辆初始位置");
    gVehicleStartPosition_Group->setFixedHeight(180);
    gVehicleStartPosition_Group->setLayout(gVehicleStartPosition_Layout);
    // 车辆初始位置 end

    // 车辆最终位置 begin
    QLabel *label_VehicleEndPointX_Text = new QLabel();
    label_VehicleEndPointX_Text->setText("X:");
    QLabel *label_VehicleEndPointY_Text = new QLabel();
    label_VehicleEndPointY_Text->setText("Y:");
    QLabel *label_VehicleEndPointYaw_Text = new QLabel();
    label_VehicleEndPointYaw_Text->setText("Yaw:");
    QLabel *label_VehicleEndPointKappa_Text = new QLabel();
    label_VehicleEndPointKappa_Text->setText("Kappa:");

    text_VehicleEndPointX = new QLineEdit();
    text_VehicleEndPointX->setText("6");
    text_VehicleEndPointY = new QLineEdit();
    text_VehicleEndPointY->setText("-2");
    text_VehicleEndPointYaw = new QLineEdit();
    text_VehicleEndPointYaw->setText("0");
    text_VehicleEndPointKappa = new QLineEdit();
    text_VehicleEndPointKappa->setText("0");

    QGridLayout *gVehicleEndPosition_Layout = new QGridLayout();
    gVehicleEndPosition_Layout->addWidget(label_VehicleEndPointX_Text,0,0);
    gVehicleEndPosition_Layout->addWidget(label_VehicleEndPointY_Text,1,0);
    gVehicleEndPosition_Layout->addWidget(label_VehicleEndPointYaw_Text,2,0);
    gVehicleEndPosition_Layout->addWidget(label_VehicleEndPointKappa_Text,3,0);

    gVehicleEndPosition_Layout->addWidget(text_VehicleEndPointX,0,1);
    gVehicleEndPosition_Layout->addWidget(text_VehicleEndPointY,1,1);
    gVehicleEndPosition_Layout->addWidget(text_VehicleEndPointYaw,2,1);
    gVehicleEndPosition_Layout->addWidget(text_VehicleEndPointKappa,3,1);

    QGroupBox *gVehicleEndPosition_Group = new QGroupBox();
    gVehicleEndPosition_Group->setTitle("车辆目标位置");
    gVehicleEndPosition_Group->setFixedHeight(180);
    gVehicleEndPosition_Group->setLayout(gVehicleEndPosition_Layout);
    // 车辆最终位置 end

    gParkingInformationConfirmG2 = new QPushButton();
    gParkingInformationConfirmG2->setText("信息确认");

    // 实时车辆位置跟踪 begin
    QLabel *label_VehiceTrackX_Text = new QLabel();
    label_VehiceTrackX_Text->setText("X:");
    QLabel *label_VehiceTrackY_Text = new QLabel();
    label_VehiceTrackY_Text->setText("Y:");
    QLabel *label_VehiceTrackYaw_Text = new QLabel();
    label_VehiceTrackYaw_Text->setText("Yaw:");

    label_G2_VehiceTrackX_Value = new QLabel();
    label_G2_VehiceTrackX_Value->setText("0");
    label_G2_VehiceTrackY_Value = new QLabel();
    label_G2_VehiceTrackY_Value->setText("0");
    label_G2_VehiceTrackYaw_Value = new QLabel();
    label_G2_VehiceTrackYaw_Value->setText("0");//单位 度

    QLabel *label_VehiceTrackX_Unit = new QLabel();
    label_VehiceTrackX_Unit->setText("m");
    QLabel *label_VehiceTrackY_Unit = new QLabel();
    label_VehiceTrackY_Unit->setText("m");
    QLabel *label_VehiceTrackYaw_Unit = new QLabel();
    label_VehiceTrackYaw_Unit->setText("°");

    QGridLayout *gVehicleTrack_Layout = new QGridLayout();
    gVehicleTrack_Layout->addWidget(label_VehiceTrackX_Text,0,0);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackY_Text,1,0);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackYaw_Text,2,0);

    gVehicleTrack_Layout->addWidget(label_G2_VehiceTrackX_Value,0,1);
    gVehicleTrack_Layout->addWidget(label_G2_VehiceTrackY_Value,1,1);
    gVehicleTrack_Layout->addWidget(label_G2_VehiceTrackYaw_Value,2,1);

    gVehicleTrack_Layout->addWidget(label_VehiceTrackX_Unit,0,2);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackY_Unit,1,2);
    gVehicleTrack_Layout->addWidget(label_VehiceTrackYaw_Unit,2,2);

    gVehicleTrack_Layout->setColumnStretch(0,2);
    gVehicleTrack_Layout->setColumnStretch(1,5);
    gVehicleTrack_Layout->setColumnStretch(2,1);

    QGroupBox *gVehicleTrack_Group = new QGroupBox();
    gVehicleTrack_Group->setTitle("实时跟踪");
    gVehicleTrack_Group->setFixedHeight(120);
    gVehicleTrack_Group->setLayout(gVehicleTrack_Layout);
    // 实时车辆位置跟踪 end

    // 规划相关配置 begin
    radio_obstacle_configure = new QRadioButton();
    radio_obstacle_configure->setText("障碍物配置");
    radio_obstacle_configure->setChecked(true);
    radio_start_gaol_configure = new QRadioButton();
    radio_start_gaol_configure->setText("起始目标点配置");

    QGridLayout *gPlannerConfigureTypeLayout = new QGridLayout();
    gPlannerConfigureTypeLayout->addWidget(radio_obstacle_configure,0,0);
    gPlannerConfigureTypeLayout->addWidget(radio_start_gaol_configure,1,0);
    gPlannerConfigureTypeLayout->setRowStretch(0,1);
    gPlannerConfigureTypeLayout->setRowStretch(1,1);

    QGroupBox *gGroupConfigureType = new QGroupBox();
    gGroupConfigureType->setTitle("配置类型");
    gGroupConfigureType->setFixedHeight(100);
    gGroupConfigureType->setLayout(gPlannerConfigureTypeLayout);
    // 规划相关配置 end

    QGridLayout *gPath_IO_Layout = new QGridLayout();
    gPath_IO_Layout->addWidget(gVehicleStartPosition_Group,0,0);
    gPath_IO_Layout->addWidget(gVehicleEndPosition_Group,1,0);
    gPath_IO_Layout->addWidget(gParkingInformationConfirmG2,2,0);
    gPath_IO_Layout->addWidget(gVehicleTrack_Group,3,0);
    gPath_IO_Layout->addWidget(gGroupConfigureType,4,0);
    gPath_IO_Layout->setColumnMinimumWidth(0,200);
    gPath_IO_Layout->setRowStretch(0,1);
    gPath_IO_Layout->setRowStretch(1,1);
    gPath_IO_Layout->setRowStretch(2,1);
    gPath_IO_Layout->setRowStretch(3,1);
    gPath_IO_Layout->setRowStretch(4,1);
    gPath_IO_Layout->setRowStretch(5,1);

    mG2_PathPlot = new QCustomPlot();
    mG2_PathPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    mG2_PathPlot->legend->setVisible(true);
    mG2_PathPlot->legend->setFont(QFont("Helvetica", 9));
    mG2_PathPlot->legend->setRowSpacing(-3);
    mG2_PathPlot->xAxis->setLabel("x");
    mG2_PathPlot->yAxis->setLabel("y");

    mPathVehicleModuleCurveG2 = new QCPCurve(mG2_PathPlot->xAxis,mG2_PathPlot->yAxis);
    mPathVehicleModuleCurveG2->setName("车辆模型");
    mPathVehicleModuleCurveG2->setPen(QPen(Qt::red,3));
    mPathVehicleModuleCurveG2->setBrush(QBrush(QColor(90,35,255,20)));

    mPathParkingCurveG2 = new QCPCurve(mG2_PathPlot->xAxis,mG2_PathPlot->yAxis);
    mPathParkingCurveG2->setName("库位");
    mPathParkingCurveG2->setPen(QPen(Qt::green,4));
    mPathParkingCurveG2->setBrush(QBrush(QColor(90,255,240,80)));

    gG2_PlanLayout = new QGridLayout();
    gG2_PlanLayout->addLayout(gPath_IO_Layout, 0, 0);
    gG2_PlanLayout->addWidget(mG2_PathPlot, 0, 1);
    gG2_PlanLayout->setColumnMinimumWidth(0,200);
    gG2_PlanLayout->setColumnStretch(0,1);
    gG2_PlanLayout->setColumnStretch(1,9);
}
/**
 * @brief MainWindow::TrackUI 跟踪UI配置函数
 */
void MainWindow::TrackUI(void)
{
    // 实时车辆位置跟踪 begin
    QLabel *label_TrackUI_VehiceTrackX_Text = new QLabel();
    label_TrackUI_VehiceTrackX_Text->setText("X:");
    QLabel *label_TrackUI_VehiceTrackY_Text = new QLabel();
    label_TrackUI_VehiceTrackY_Text->setText("Y:");
    QLabel *label_TrackUI_VehiceTrackYaw_Text = new QLabel();
    label_TrackUI_VehiceTrackYaw_Text->setText("Yaw:");

    label_TrackUI_VehiceTrackX_Value = new QLabel();
    label_TrackUI_VehiceTrackX_Value->setText("0");
    label_TrackUI_VehiceTrackY_Value = new QLabel();
    label_TrackUI_VehiceTrackY_Value->setText("0");
    label_TrackUI_VehiceTrackYaw_Value = new QLabel();
    label_TrackUI_VehiceTrackYaw_Value->setText("0");//单位 度

    QLabel *label_TrackUI_VehiceTrackX_Unit = new QLabel();
    label_TrackUI_VehiceTrackX_Unit->setText("m");
    QLabel *label_TrackUI_VehiceTrackY_Unit = new QLabel();
    label_TrackUI_VehiceTrackY_Unit->setText("m");
    QLabel *label_TrackUI_VehiceTrackYaw_Unit = new QLabel();
    label_TrackUI_VehiceTrackYaw_Unit->setText("°");

    QGridLayout *gTrackUI_VehicleTrack_Layout = new QGridLayout();
    gTrackUI_VehicleTrack_Layout->addWidget(label_TrackUI_VehiceTrackX_Text,0,0);
    gTrackUI_VehicleTrack_Layout->addWidget(label_TrackUI_VehiceTrackY_Text,1,0);
    gTrackUI_VehicleTrack_Layout->addWidget(label_TrackUI_VehiceTrackYaw_Text,2,0);

    gTrackUI_VehicleTrack_Layout->addWidget(label_TrackUI_VehiceTrackX_Value,0,1);
    gTrackUI_VehicleTrack_Layout->addWidget(label_TrackUI_VehiceTrackY_Value,1,1);
    gTrackUI_VehicleTrack_Layout->addWidget(label_TrackUI_VehiceTrackYaw_Value,2,1);

    gTrackUI_VehicleTrack_Layout->addWidget(label_TrackUI_VehiceTrackX_Unit,0,2);
    gTrackUI_VehicleTrack_Layout->addWidget(label_TrackUI_VehiceTrackY_Unit,1,2);
    gTrackUI_VehicleTrack_Layout->addWidget(label_TrackUI_VehiceTrackYaw_Unit,2,2);

    gTrackUI_VehicleTrack_Layout->setColumnStretch(0,2);
    gTrackUI_VehicleTrack_Layout->setColumnStretch(1,5);
    gTrackUI_VehicleTrack_Layout->setColumnStretch(1,1);

    QGroupBox *gTrackUI_VehicleTrack_Group = new QGroupBox();
    gTrackUI_VehicleTrack_Group->setTitle("实时跟踪");
    gTrackUI_VehicleTrack_Group->setFixedHeight(120);
    gTrackUI_VehicleTrack_Group->setLayout(gTrackUI_VehicleTrack_Layout);
    // 实时车辆位置跟踪 end

    // 定位模式选择组件
    radio_sin_curvature = new QRadioButton();
    radio_sin_curvature->setText("三角函数曲线");
    radio_double_line = new QRadioButton();
    radio_double_line->setText("伯努利双纽线");
    radio_circle_curvature = new QRadioButton();
    radio_circle_curvature->setText("圆弧8字曲线");

    QGridLayout *gCurvatureTypeLayout = new QGridLayout();
    gCurvatureTypeLayout->addWidget(radio_sin_curvature,0,0);
    gCurvatureTypeLayout->addWidget(radio_double_line,1,0);
    gCurvatureTypeLayout->addWidget(radio_circle_curvature,2,0);

    gCurvatureTypeLayout->setRowStretch(0,1);
    gCurvatureTypeLayout->setRowStretch(1,1);
    gCurvatureTypeLayout->setRowStretch(2,1);

    QGroupBox *gGroupCurvatureType = new QGroupBox();
    gGroupCurvatureType->setTitle("曲线类型");
    gGroupCurvatureType->setFixedHeight(150);
    gGroupCurvatureType->setLayout(gCurvatureTypeLayout);

    button_patn_generate = new QPushButton();
    button_patn_generate->setText("路径生成");

    button_track_start = new QPushButton();
    button_track_start->setText("开始跟踪");

    button_timer_control = new QPushButton();
    button_timer_control->setText("开始");

    QGridLayout *gPathGenerate_Layout = new QGridLayout();
    gPathGenerate_Layout->addWidget(button_patn_generate,0,0);
    gPathGenerate_Layout->addWidget(button_track_start,1,0);
    gPathGenerate_Layout->addWidget(button_timer_control,2,0);
    gPathGenerate_Layout->setRowStretch(0,1);
    gPathGenerate_Layout->setRowStretch(1,1);
    gPathGenerate_Layout->setRowStretch(2,1);

    QGroupBox *gPathGenerate_Group = new QGroupBox();
    gPathGenerate_Group->setTitle("路径生成");
    gPathGenerate_Group->setFixedHeight(180);
    gPathGenerate_Group->setLayout(gPathGenerate_Layout);

    QGridLayout *gTrack_IO_Layout = new QGridLayout();
    gTrack_IO_Layout->addWidget(gTrackUI_VehicleTrack_Group,0,0);
    gTrack_IO_Layout->addWidget(gGroupCurvatureType,1,0);
    gTrack_IO_Layout->addWidget(gPathGenerate_Group,2,0);
    gTrack_IO_Layout->setRowStretch(0,1);
    gTrack_IO_Layout->setRowStretch(1,1);
    gTrack_IO_Layout->setRowStretch(2,1);
    gTrack_IO_Layout->setRowStretch(3,1);

    // 绘图界面配置
    mTrackPlot = new QCustomPlot();
    mTrackPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    mTrackPlot->legend->setVisible(true);
    mTrackPlot->legend->setFont(QFont("Helvetica", 9));
    mTrackPlot->legend->setRowSpacing(-3);
    mTrackPlot->xAxis->setLabel("x");
    mTrackPlot->yAxis->setLabel("y");
//    mTrackPlot->xAxis->setRange(BOUNDARY_LEFT,BOUNDARY_RIGHT);
//    mTrackPlot->yAxis->setRange(BOUNDARY_DOWN,BOUNDARY_TOP);

    mTrackVehicleModuleCurve = new QCPCurve(mTrackPlot->xAxis,mTrackPlot->yAxis);
    mTrackVehicleModuleCurve->setName("车辆模型");
    mTrackVehicleModuleCurve->setPen(QPen(Qt::red,3));
    mTrackVehicleModuleCurve->setBrush(QBrush(QColor(90,35,255,20)));

    mTrackParkingCurve = new QCPCurve(mTrackPlot->xAxis,mTrackPlot->yAxis);
    mTrackParkingCurve->setName("库位");
    mTrackParkingCurve->setPen(QPen(Qt::green,4));
    mTrackParkingCurve->setBrush(QBrush(QColor(90,255,240,80)));

    mTrackTargetCurve = new QCPCurve(mTrackPlot->xAxis,mTrackPlot->yAxis);
    mTrackTargetCurve->setName("目标路径");
    mTrackTargetCurve->setPen(QPen(Qt::blue,3));

    mTrackVehicleCenterCurve = new QCPCurve(mTrackPlot->xAxis,mTrackPlot->yAxis);
    mTrackVehicleCenterCurve->setName("后轴中心");
    mTrackVehicleCenterCurve->setPen(QPen(Qt::darkRed,3));

    mTrackSinglePlot = new QCustomPlot();
    mTrackSinglePlot->yAxis->setLabel("steering angle(deg/s)");

    mTrackSinglePlot->yAxis->setTickLabels(false);
    connect(mTrackSinglePlot->yAxis2, SIGNAL(rangeChanged(QCPRange)), mTrackSinglePlot->yAxis, SLOT(setRange(QCPRange))); // left axis only mirrors inner right axis
    mTrackSinglePlot->yAxis2->setVisible(true);
    mTrackSinglePlot->axisRect()->addAxis(QCPAxis::atRight);
    mTrackSinglePlot->axisRect()->axis(QCPAxis::atRight, 0)->setPadding(30); // add some padding to have space for tags
//    mTrackSinglePlot->axisRect()->axis(QCPAxis::atRight, 1)->setPadding(30); // add some padding to have space for tags

    mTrackSinglePlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    // create graphs:
    mGraph_SteeringAngle = mTrackSinglePlot->addGraph(mTrackSinglePlot->xAxis, mTrackSinglePlot->axisRect()->axis(QCPAxis::atRight, 0));
    mGraph_SteeringAngle->setPen(QPen(QColor(250, 120, 0)));

//    mGraph_Track = mTrackSinglePlot->addGraph(mTrackSinglePlot->xAxis, mTrackSinglePlot->axisRect()->axis(QCPAxis::atRight, 1));
//    mGraph_Track->setPen(QPen(QColor(0, 180, 60)));

    // create tags with newly introduced AxisTag class (see axistag.h/.cpp):
    mTag_SteeringAngle = new AxisTag(mGraph_SteeringAngle->valueAxis());
    mTag_SteeringAngle->setPen(mGraph_SteeringAngle->pen());
//    mTag_Track = new AxisTag(mGraph_Track->valueAxis());
//    mTag_Track->setPen(mGraph_Track->pen());

    //Plot Ui layout Configure
    gPlotLayout = new QGridLayout();
    gPlotLayout->addWidget(mTrackPlot, 0, 0);
    gPlotLayout->addWidget(mTrackSinglePlot, 1, 0);
    gPlotLayout->setRowStretch(0,2);
    gPlotLayout->setRowStretch(1,1);

    gTrackLayout = new QGridLayout();
    gTrackLayout->addLayout(gTrack_IO_Layout, 0, 0);
    gTrackLayout->addLayout(gPlotLayout, 0, 1);


    gTrackLayout->setColumnMinimumWidth(0,200);
    gTrackLayout->setColumnStretch(0,1);
    gTrackLayout->setColumnStretch(1,9);

    SteeringAngleShow(520);
    SteeringAngleShow(-520);
}

void MainWindow::Init()
{
     mTerminal = new Terminal();
     mSimulation = new Simulation();


//    Percaption mPercaption;
//    GeometricTrack mGeometricTrack;
//    VehilceConfig mVehilceConfig;

     mBoRuiMessage = new BoRuiMessage();
     mBoRuiController = new BoRuiController();

    NewFileUpdateFlag = 0;
    time_step_cnt     = 0;

    // Path
    mParallelPlanning = new ParallelPlanning();
    mVerticalPlanning = new VerticalPlanning();

    mParallelPlanning->Init();
    mVerticalPlanning->Init();

    mHC_ReedsSheppStateSpace = new HC_ReedsSheppStateSpace(0.2, 0.215, 0.02);

//    m_OMPL_Planner = new OMPL_Planner();

    m_base_state_index = -1;
    m_head_state_index = -1;
    selected_grid_colour = 0xA5;
    // Track
    mLatControl = new LatControl();
    mLatControl->Init();

    mCurvature = new Curvature();
    mCurvature->Init();

    m_LatControl_LQR = new LatControl_LQR();
    m_LatControl_LQR->Init(&mVehilceConfig);

    _target_curvature_data_sets = new TrackLinkList();

    m_TrajectoryAnalyzer = new TrajectoryAnalyzer();

    _target_curvature_vectors = new std::vector<TargetTrack>;
}

/****** Function ******/
/**
 * @brief 显示车辆模型和后轴中心
 * @param p:车辆位置
 * @param yaw:车辆偏航角
 * @param vehicle_center:图形绘制车辆外形曲线
 * @param vehicle_modle: 图形绘制车辆后轴曲线
 * @param plot: 图形绘制面板
 * @return None
 */
void MainWindow::VehicleModuleShow(Vector2d p,float yaw,QCPCurve *vehicle_center,QCPCurve *vehicle_modle,QCustomPlot *plot)
{
    FrontLeftPoint  = p + Vector2d(mVehilceConfig.getFrontLeftDiagonal().Length,0.0).rotate(mVehilceConfig.getFrontLeftDiagonal().Angle + yaw);
    FrontRightPoint = p + Vector2d(mVehilceConfig.getFrontRightDiagonal().Length,0.0).rotate(mVehilceConfig.getFrontRightDiagonal().Angle + yaw);
    RearLeftPoint   = p + Vector2d(-mVehilceConfig.getRearLeftDiagonal().Length,0.0).rotate(mVehilceConfig.getRearLeftDiagonal().Angle + yaw);
    RearRightPoint  = p + Vector2d(-mVehilceConfig.getRearRightDiagonal().Length,0.0).rotate(mVehilceConfig.getRearRightDiagonal().Angle + yaw);

    vehicle_center->addData(static_cast<double>(p.getX()),static_cast<double>(p.getY()));

    QVector<double> VehiclePointX(5),VehiclePointY(5);
    VehiclePointX[0] = static_cast<double>(RearRightPoint.getX());
    VehiclePointX[1] = static_cast<double>(RearLeftPoint.getX());
    VehiclePointX[2] = static_cast<double>(FrontLeftPoint.getX());
    VehiclePointX[3] = static_cast<double>(FrontRightPoint.getX());
    VehiclePointX[4] = static_cast<double>(RearRightPoint.getX());

    VehiclePointY[0] = static_cast<double>(RearRightPoint.getY());
    VehiclePointY[1] = static_cast<double>(RearLeftPoint.getY());
    VehiclePointY[2] = static_cast<double>(FrontLeftPoint.getY());
    VehiclePointY[3] = static_cast<double>(FrontRightPoint.getY());
    VehiclePointY[4] = static_cast<double>(RearRightPoint.getY());
    vehicle_modle->setData(VehiclePointX,VehiclePointY);

    plot->replot();
}

/**
 * @brief 显示矢量箭头
 * @param x :the x axis location
 * @param y :the y axis location
 * @param yaw :the yaw angle od the vectoe arrow
 * @param arrow :the plot object
 */
void MainWindow::VectorArrowShow(State base, State *head, QCPItemLine *arrow)
{
    Vector2d arrow_poit[2];
    Vector2d temp_arrow;
    arrow_poit[0].setX(base.x);
    arrow_poit[0].setY(base.y);
    arrow_poit[1].setX(2.0);
    arrow_poit[1].setY(0.0);
    temp_arrow = arrow_poit[0] + arrow_poit[1].rotate(base.psi);

    head->x = temp_arrow.getX();
    head->y = temp_arrow.getY();
    head->psi = base.psi;

    arrow->start->setCoords(arrow_poit[0].getX(), arrow_poit[0].getY());
    arrow->end->setCoords(temp_arrow.getX(), temp_arrow.getY());
}

void MainWindow::HC_CC_PathShow(vector<State> &p)
{
    QVector<double> clothoid_path_x, clothoid_path_y;
    QVector<double> circle_path_x, circle_path_y;
    QVector<double> straight_line_path_x, straight_line_path_y;
    for (auto &path_state : p)
    {
        if(fabs(path_state.kappa) < math::getEpsilon())
        {
            straight_line_path_x.push_back(path_state.x);
            straight_line_path_y.push_back(path_state.y);
        }
        else if( fabs(fabs(path_state.kappa) - 0.2) < math::getEpsilon() )
        {
            circle_path_x.push_back(path_state.x);
            circle_path_y.push_back(path_state.y);
        }
        else
        {
            clothoid_path_x.push_back(path_state.x);
            clothoid_path_y.push_back(path_state.y);
        }
    }
    mPathPlanningStraightLine->setData(straight_line_path_x, straight_line_path_y);
    mPathPlanningCircle->setData(circle_path_x, circle_path_y);
    mPathPlanningClothoid->setData(clothoid_path_x, clothoid_path_y);
}

/**
 * @brief 画圆
 * @param x :the x axis location with the centern of circle
 * @param y :the y axis location with the centern of circle
 * @param radius :the radius of the circle
 * @param e :the ellipse object
 */
void MainWindow::CircleShow(double x, double y, double radius, QCPItemEllipse *e)
{
    e->topLeft->setCoords    ( x - radius, y + radius);
    e->bottomRight->setCoords( x + radius, y - radius);
}
/**
 * @brief 显示规划的几何圆
 * @param c :the circle location
 */
void MainWindow::HC_CC_CircleShow(HC_CC_RS_Path *c)
{
    CircleShow(c->getCircleStart()->getCenter_x(),
               c->getCircleStart()->getCenter_y(),
               1 / c->getKappa(), mStartCircle);

    CircleShow(c->getCircleEnd()->getCenter_x(),
               c->getCircleEnd()->getCenter_y(),
               1 / c->getKappa(), mEndCircle);


    if(c->getCi1() != nullptr)
    {
        CircleShow(c->getCi1()->getCenter_x(),
                   c->getCi1()->getCenter_y(),
                   1 / c->getKappa(), mMiddleCircle1);
        mMiddleCircle1->setVisible(true);
    }
    else
    {
        mMiddleCircle1->setVisible(false);
    }

    if(c->getCi2() != nullptr)
    {
        CircleShow(c->getCi2()->getCenter_x(),
                   c->getCi2()->getCenter_y(),
                   1 / c->getKappa(), mMiddleCircle2);
        mMiddleCircle2->setVisible(true);
    }
    else
    {
        mMiddleCircle2->setVisible(false);
    }

    QVector<double> q_x, q_y;
    q_x.push_back(c->getCircleStart()->getStart().getX());
    q_y.push_back(c->getCircleStart()->getStart().getY());
    if(c->getQi1() != nullptr)
    {
        q_x.push_back(c->getQi1()->getX());
        q_y.push_back(c->getQi1()->getY());
    }
    if(c->getQi2() != nullptr)
    {
        q_x.push_back(c->getQi2()->getX());
        q_y.push_back(c->getQi2()->getY());
    }
    if(c->getQi3() != nullptr)
    {
        q_x.push_back(c->getQi3()->getX());
        q_y.push_back(c->getQi3()->getY());
    }
    if(c->getQi4() != nullptr)
    {
        q_x.push_back(c->getQi4()->getX());
        q_y.push_back(c->getQi4()->getY());
    }
    q_x.push_back(c->getCircleEnd()->getStart().getX());
    q_y.push_back(c->getCircleEnd()->getStart().getY());
    mTangentCirclePoint->setData(q_x, q_y);
}
/**
 * @brief MainWindow::TargetPathShow 显示目标曲线
 * @param list：包含目标曲线的数据集
 */
void MainWindow::TargetPathShow(TrackLinkList *list)
{
    uint16_t i;
    Node<TargetTrack>* _track_node;
    _track_node = list->getHeadNode();
    QVector<double> VehiclePointX,VehiclePointY;
    i = 0;
    while(_track_node->next != NULL)
    {
        VehiclePointX.append(_track_node->data.point.getX());
        VehiclePointY.append(_track_node->data.point.getY());
        _track_node = _track_node->next;
        i++;
    }
    VehiclePointX.append(_track_node->data.point.getX());
    VehiclePointY.append(_track_node->data.point.getY());
    mTrackTargetCurve->setData(VehiclePointX,VehiclePointY);
}

void MainWindow::TargetPathShow(std::vector<TargetTrack> *vec)
{
    uint16_t i;
    QVector<double> VehiclePointX,VehiclePointY;
    i = 0;
    for(std::vector<TargetTrack>::iterator it = vec->begin();it != vec->end();it++)
    {
        VehiclePointX.append(it->point.getX());
        VehiclePointY.append(it->point.getY());
        i++;
    }
    mTrackTargetCurve->setData(VehiclePointX,VehiclePointY);
}
/**
 * @brief MainWindow::SteeringAngleShow 显示方向盘转角信息
 * @param angle：需显示角度值
 */
void MainWindow::SteeringAngleShow(float angle)
{
    mGraph_SteeringAngle->addData(mGraph_SteeringAngle->dataCount(),static_cast<double>(angle));
    mTrackSinglePlot->xAxis->rescale();
    mGraph_SteeringAngle->rescaleValueAxis(false,true);
    mTrackSinglePlot->xAxis->setRange(mTrackSinglePlot->xAxis->range().upper,100,Qt::AlignRight);
    double graphSteeringAngleValue = mGraph_SteeringAngle->dataMainValue(mGraph_SteeringAngle->dataCount()-1);
    mTag_SteeringAngle->updatePosition(graphSteeringAngleValue);
    mTag_SteeringAngle->setText(QString::number(graphSteeringAngleValue,'f',2));

    mTrackSinglePlot->replot();
}

/**
 * @brief MainWindow::FileDataInit 注入文件数据的缓存区初始化
 * @param  None
 * @return None
 */
void MainWindow::FileDataInit(void)
{
    uint8_t i;

    VehicleTrackList.clear();

    for(i=0;i < 12;i++)
    {
       LRU_List[i].clear();
       LRU_PositionList[i].clear();
    }
    for(i=0;i<4;i++)
    {
        ObstacleBody_List[i].clear();
        _ultrasonic_data_buffer[i].Position.setX(0.0);
        _ultrasonic_data_buffer[i].Position.setY(0.0);
        _ultrasonic_data_buffer[i].UltrasonicData.Distance1 = 0.0f;
        _ultrasonic_data_buffer[i].UltrasonicData.Distance2 = 0.0f;
        _ultrasonic_data_buffer[i].UltrasonicData.Level = 0.0f;
        _ultrasonic_data_buffer[i].UltrasonicData.Width = 0.0f;
        _ultrasonic_data_buffer[i].UltrasonicData.status = 0;
        _ultrasonic_data_buffer[i].UltrasonicData.Time_Tx = 0;
        _ultrasonic_data_buffer[i].UltrasonicData.Time_Ms = 0.0f;
    }
    time_step_cnt = 0;

    mDetectVehicleCenterCurve->data().data()->clear();
    mDetectEdgePoint->data().data()->clear();
    mDetectValidEdgePoint->data().data()->clear();
    mDetectRearEdgeTrianglePosition->data().data()->clear();

    mDetectLeftEdgeGroundPosition->data().data()->clear();
    mDetectRightEdgeGroundPosition->data().data()->clear();

    mDetectLeftEdgeFitLine->data().data()->clear();
    mDetectRightEdgeFitLine->data().data()->clear();
}

/**
 * @brief MainWindow::AnalyzeOneLine 分析一行数据
 * @param baLine：待解析的一行数据
 * @return None
 */
void MainWindow::AnalyzeOneLine(const QByteArray &baLine)
{
    QList<QByteArray> detect_list = baLine.split(' ');

    ObstacleLocationPacket temp_position;

    temp_position.Position.setX( detect_list[13].toDouble() );
    temp_position.Position.setY( detect_list[14].toDouble() );
    temp_position.Status     = static_cast<UltrasonicStatus>(detect_list[15].toUShort());
    LRU_PositionList[4].append(temp_position);

    temp_position.Position.setX( detect_list[16].toDouble() );
    temp_position.Position.setY( detect_list[17].toDouble() );
    temp_position.Status     = static_cast<UltrasonicStatus>(detect_list[18].toUShort());
    LRU_PositionList[5].append(temp_position);

    temp_position.Position.setX( detect_list[19].toDouble() );
    temp_position.Position.setY( detect_list[20].toDouble() );
    temp_position.Status     = static_cast<UltrasonicStatus>(detect_list[21].toUShort());
    LRU_PositionList[6].append(temp_position);

    temp_position.Position.setX( detect_list[22].toDouble() );
    temp_position.Position.setY( detect_list[23].toDouble() );
    temp_position.Status     = static_cast<UltrasonicStatus>(detect_list[24].toUShort());
    LRU_PositionList[7].append(temp_position);

    temp_position.Position.setX( detect_list[25].toDouble() );
    temp_position.Position.setY( detect_list[26].toDouble() );
    temp_position.Status     = static_cast<UltrasonicStatus>(detect_list[27].toUShort());
    LRU_PositionList[8].append(temp_position);

    temp_position.Position.setX( detect_list[28].toDouble() );
    temp_position.Position.setY( detect_list[29].toDouble() );
    temp_position.Status     = static_cast<UltrasonicStatus>(detect_list[30].toUShort());
    LRU_PositionList[9].append(temp_position);

    temp_position.Position.setX( detect_list[31].toDouble() );
    temp_position.Position.setY( detect_list[32].toDouble() );
    temp_position.Status     = static_cast<UltrasonicStatus>(detect_list[33].toUShort());
    LRU_PositionList[10].append(temp_position);

    temp_position.Position.setX( detect_list[34].toDouble() );
    temp_position.Position.setY( detect_list[35].toDouble() );
    temp_position.Status     = static_cast<UltrasonicStatus>(detect_list[36].toUShort());
    LRU_PositionList[11].append(temp_position);

    GeometricTrack *temp_track = new GeometricTrack();
    temp_track->getPosition().setX( detect_list[37].toDouble() );
    temp_track->getPosition().setY( detect_list[38].toDouble() );
    temp_track->Yaw = detect_list[39].toFloat();
    VehicleTrackList.append(*temp_track);

    Ultrasonic_Packet LRU_temp;
    LRU_temp.Distance1 = detect_list[40].toFloat();
    LRU_temp.Distance2 = detect_list[41].toFloat();
    LRU_temp.Level     = detect_list[42].toFloat();
    LRU_temp.Width     = detect_list[43].toFloat();
    LRU_temp.status    = static_cast<uint8_t>(detect_list[44].toUShort());
    LRU_List[8].append(LRU_temp);

    LRU_temp.Distance1 = detect_list[45].toFloat();
    LRU_temp.Distance2 = detect_list[46].toFloat();
    LRU_temp.Level     = detect_list[47].toFloat();
    LRU_temp.Width     = detect_list[48].toFloat();
    LRU_temp.status    = static_cast<uint8_t>(detect_list[49].toUShort());
    LRU_List[9].append(LRU_temp);

    LRU_temp.Distance1 = detect_list[50].toFloat();
    LRU_temp.Distance2 = detect_list[51].toFloat();
    LRU_temp.Level     = detect_list[52].toFloat();
    LRU_temp.Width     = detect_list[53].toFloat();
    LRU_temp.status    = static_cast<uint8_t>(detect_list[54].toUShort());
    LRU_List[10].append(LRU_temp);

    LRU_temp.Distance1 = detect_list[55].toFloat();
    LRU_temp.Distance2 = detect_list[56].toFloat();
    LRU_temp.Level     = detect_list[57].toFloat();
    LRU_temp.Width     = detect_list[58].toFloat();
    LRU_temp.status    = static_cast<uint8_t>(detect_list[59].toUShort());
    LRU_List[11].append(LRU_temp);

    LRU_List[4].append(LRU_temp);
    LRU_List[5].append(LRU_temp);
    LRU_List[6].append(LRU_temp);
    LRU_List[7].append(LRU_temp);
}

/**
 * @brief MainWindow::DetectTask 超声感知检测任务
 */
void MainWindow::DetectTask(void)
{
    int32_t len;
    uint16_t i;
    if(0xA5 == NewFileUpdateFlag)
    {
        len = VehicleTrackList.length();

        for(i=4;i<12;i++)
        {
            mUltrasonicObstaclePercption.DataPushStateMachine(LRU_List[i][time_step_cnt],LRU_PositionList[i][time_step_cnt],i);

            switch(i)
            {
                case 4:
                case 5:
                case 6:
                case 7:
                    if(Normal == LRU_PositionList[i][time_step_cnt].Status)
                    {
                       mDetectRearEdgeTrianglePosition->addData(static_cast<double>(LRU_PositionList[i][time_step_cnt].Position.getX()),static_cast<double>(LRU_PositionList[i][time_step_cnt].Position.getY()));
                    }
                    break;

                case 10:
                    if(Normal == LRU_PositionList[i][time_step_cnt].Status)
                    {
                        mDetectLeftEdgeGroundPosition->addData(static_cast<double>(LRU_PositionList[i][time_step_cnt].Position.getX()),static_cast<double>(LRU_PositionList[i][time_step_cnt].Position.getY()));
                    }
                    break;

                case 11:
                    if(Normal == LRU_PositionList[i][time_step_cnt].Status)
                    {
                        mDetectRightEdgeGroundPosition->addData(static_cast<double>(LRU_PositionList[i][time_step_cnt].Position.getX()),static_cast<double>(LRU_PositionList[i][time_step_cnt].Position.getY()));
                    }
                    break;
                default:
                    break;
            }
        }
        // 车辆模型绘制
//        DetectVehicleModule(VehicleTrackList[time_step_cnt].getPosition(),VehicleTrackList[time_step_cnt].getYaw());
        VehicleModuleShow(VehicleTrackList[time_step_cnt].getPosition(),VehicleTrackList[time_step_cnt].getYaw(),mDetectVehicleCenterCurve,mDetectVehicleModuleCurve,mDetectPlot);
        time_step_cnt++;
        if(radio_right_enter_location->isChecked()){
            qDebug() << "the front edge list length:" << mUltrasonicObstaclePercption.getFrontEdgeListLength();
            qDebug() << "the rear edge list length:" << mUltrasonicObstaclePercption.getRearEdgeListLength();
        }
        if(radio_left_enter_location->isChecked()){
            qDebug() << "the front edge list length:" << mUltrasonicObstaclePercption.getFrontEdgeListLength();
            qDebug() << "the rear edge list length:" << mUltrasonicObstaclePercption.getRearEdgeListLength();
        }
        if(radio_center_enter_location->isChecked()){
            qDebug() << "the left list length:" << mUltrasonicObstaclePercption.getLeftEdgeListLength();
            qDebug() << "the right list length:" << mUltrasonicObstaclePercption.getRightEdgeListLength();
        }

        if(time_step_cnt >= len)
        {
            NewFileUpdateFlag = 0x5A;
            if(radio_right_enter_location->isChecked()){mUltrasonicObstaclePercption.Command = RIGHT_PARKING_STOP_CMD;}
            if(radio_left_enter_location->isChecked()){mUltrasonicObstaclePercption.Command = LEFT_PARKING_STOP_CMD;}
            if(radio_center_enter_location->isChecked()){mUltrasonicObstaclePercption.Command = ENTER_PARKING_STOP_CMD;}
        }
    }
    else if(0x5A == NewFileUpdateFlag)
    {
        for(i=4;i<12;i++)
        {
            mUltrasonicObstaclePercption.DataPushStateMachine(LRU_List[i][time_step_cnt-1],LRU_PositionList[i][time_step_cnt-1],i);
        }
    }

    if(SUCCESS == mUltrasonicObstaclePercption.ParkingCalculateStateMachine())
    {
        Vector2d line_point;
        if(radio_center_enter_location->isChecked()){
            qDebug() << "Left fit line variance:" << mUltrasonicObstaclePercption.getLeftFitLinePacket().variance;
            qDebug() << "Right fit line variance:" << mUltrasonicObstaclePercption.getRightFitLinePacket().variance;

            mDetectLeftEdgeFitLine->data().data()->clear();
            mDetectRightEdgeFitLine->data().data()->clear();

            // left line fit
            if(0xAA == mUltrasonicObstaclePercption.getLeftFitLinePacket().valid_flag)
            {
                line_point.setX(VehicleTrackList[0].getPosition().getX());
                line_point.setY(tan(mUltrasonicObstaclePercption.getLeftFitLinePacket().angle) * line_point.getX() +
                                     mUltrasonicObstaclePercption.getLeftFitLinePacket().offset);
                mDetectLeftEdgeFitLine->addData(static_cast<double>(line_point.getX()) ,static_cast<double>(line_point.getY()));

                line_point.setX(VehicleTrackList[time_step_cnt-1].getPosition().getX());
                line_point.setY(tan(mUltrasonicObstaclePercption.getLeftFitLinePacket().angle) * line_point.getX() +
                                     mUltrasonicObstaclePercption.getLeftFitLinePacket().offset);
                mDetectLeftEdgeFitLine->addData(static_cast<double>(line_point.getX()) ,static_cast<double>(line_point.getY()));
            }
            // right line fit
            if(0xAA == mUltrasonicObstaclePercption.getRightFitLinePacket().valid_flag)
            {
                line_point.setX(VehicleTrackList[0].getPosition().getX());
                line_point.setY(tan(mUltrasonicObstaclePercption.getRightFitLinePacket().angle) * line_point.getX() +
                                     mUltrasonicObstaclePercption.getRightFitLinePacket().offset);
                mDetectRightEdgeFitLine->addData(static_cast<double>(line_point.getX()) ,static_cast<double>(line_point.getY()));

                line_point.setX(VehicleTrackList[time_step_cnt-1].getPosition().getX());
                line_point.setY(tan(mUltrasonicObstaclePercption.getRightFitLinePacket().angle) * line_point.getX() +
                                     mUltrasonicObstaclePercption.getRightFitLinePacket().offset);
                mDetectRightEdgeFitLine->addData(static_cast<double>(line_point.getX()) ,static_cast<double>(line_point.getY()));
            }
        }
        else if(radio_right_enter_location->isChecked())
        {
            // right line fit
            if(0xAA == mUltrasonicObstaclePercption.getFrontEdgeFitLinePacket().valid_flag)
            {
                qDebug() << "Front Edge fit line variance:" << mUltrasonicObstaclePercption.getFrontEdgeFitLinePacket().variance;
                line_point.setX(VehicleTrackList[0].getPosition().getX());
                line_point.setY(tan(mUltrasonicObstaclePercption.getFrontEdgeFitLinePacket().angle) * line_point.getX() +
                                     mUltrasonicObstaclePercption.getFrontEdgeFitLinePacket().offset);
                mDetectRightEdgeFitLine->addData(static_cast<double>(line_point.getX()) ,static_cast<double>(line_point.getY()));

                line_point.setX(VehicleTrackList[time_step_cnt-1].getPosition().getX());
                line_point.setY(tan(mUltrasonicObstaclePercption.getFrontEdgeFitLinePacket().angle) * line_point.getX() +
                                     mUltrasonicObstaclePercption.getFrontEdgeFitLinePacket().offset);
                mDetectRightEdgeFitLine->addData(static_cast<double>(line_point.getX()) ,static_cast<double>(line_point.getY()));
            }
            if(mUltrasonicObstaclePercption.getValidParkingEdgePosition().FrontOutSide.valid_flag == 0xA5)
            {
                mDetectValidEdgePoint->addData(static_cast<double>(mUltrasonicObstaclePercption.getValidParkingEdgePosition().FrontOutSide.position.getX()) ,
                                               static_cast<double>(mUltrasonicObstaclePercption.getValidParkingEdgePosition().FrontOutSide.position.getY()));
            }
            if(mUltrasonicObstaclePercption.getValidParkingEdgePosition().RearOutSide.valid_flag == 0xA5)
            {
                mDetectValidEdgePoint->addData(static_cast<double>(mUltrasonicObstaclePercption.getValidParkingEdgePosition().RearOutSide.position.getX()) ,
                                               static_cast<double>(mUltrasonicObstaclePercption.getValidParkingEdgePosition().RearOutSide.position.getY()));
            }
        }
        mDetectPlot->replot();
    }
}

/**
 * @brief MainWindow::PlanTask 规划任务
 */
void MainWindow::PlanTask(void)
{
    switch (gPlanTab->currentIndex())
    {
    case 0:
        /****** 第一代规划算法(几何规划) ******/
        if (radio_parallel->isChecked())
        {
            // 平行泊车控制
            mParallelPlanning->Work(&mPercaption);
            mParallelPlanning->Control(mBoRuiController,mBoRuiMessage,&mGeometricTrack,&mPercaption);
        }
        else if (radio_vertical->isChecked())
        {
            // 垂直泊车控制
            mVerticalPlanning->Work(&mPercaption,&mGeometricTrack);
            mVerticalPlanning->Control(mBoRuiController,mBoRuiMessage,&mGeometricTrack,&mPercaption);
        }
        // 仿真信号更新
        mSimulation->Update(mBoRuiController,mBoRuiMessage);

        // 车辆位置更新
        mGeometricTrack.VelocityUpdate(mBoRuiMessage,0.02f);

        label_G1_VehiceTrackX_Value->setText(QString::number(static_cast<double>(mGeometricTrack.getPosition().getX()),'f',2));
        label_G1_VehiceTrackY_Value->setText(QString::number(static_cast<double>(mGeometricTrack.getPosition().getY()),'f',2));
        label_G1_VehiceTrackYaw_Value->setText(QString::number(static_cast<double>(mGeometricTrack.getYaw()*57.3f),'f',2));

        VehicleModuleShow(mGeometricTrack.getPosition(),mGeometricTrack.getYaw(),mPathVehicleCenterCurveG1,mPathVehicleModuleCurveG1,mG1_PathPlot);
        mG1_PathPlot->replot();
        break;

    case 1:
        mHC_PathPlot->replot();
        break;

    case 2:
        /****** 第二代规划算法(基于搜索) ******/
        mG2_PathPlot->replot();
        break;

    default:
        break;
    }
}

/**
 * @brief MainWindow::TrackTask 跟踪任务
 */
void MainWindow::TrackTask(void)
{
    TargetTrack temp_node;
    TargetTrack end_node;
    if(_target_curvature_vectors->size() > 0)
    {

        mLatControl->Work(mBoRuiMessage,mBoRuiController,&mGeometricTrack,m_TrajectoryAnalyzer);

//        m_LatControl_LQR->Work(mBoRuiMessage,&mGeometricTrack,*m_TrajectoryAnalyzer,mBoRuiController);
    }
    // 仿真信号更新
    mSimulation->Update(mBoRuiController,mBoRuiMessage);

    // 车辆位置更新
    mGeometricTrack.VelocityUpdate(mBoRuiMessage,0.02f);

    label_TrackUI_VehiceTrackX_Value->setText(QString::number(static_cast<double>(mGeometricTrack.getPosition().getX()),'f',2));
    label_TrackUI_VehiceTrackY_Value->setText(QString::number(static_cast<double>(mGeometricTrack.getPosition().getY()),'f',2));
    label_TrackUI_VehiceTrackYaw_Value->setText(QString::number(static_cast<double>(mGeometricTrack.getYaw()*57.3f),'f',2));
    SteeringAngleShow(mBoRuiMessage->getSteeringAngle());
    VehicleModuleShow(mGeometricTrack.getPosition(),mGeometricTrack.getYaw(),mTrackVehicleCenterCurve,mTrackVehicleModuleCurve,mTrackPlot);
}
/****** SLOT ******/
/**
 * @brief MainWindow::sTimer20msTask 定时任务20ms，调度各个子任务
 */
void MainWindow::sTimer20msTask(void)
{
    if(0 == stack_Widget->currentIndex()){

    }
    else if(1 == stack_Widget->currentIndex()){
        DetectTask();
    }
    else if(2 == stack_Widget->currentIndex()){
        PlanTask();
    }
    else if(3 == stack_Widget->currentIndex()){
        TrackTask();
    }
    else{
        qDebug() << "sTimer20msTask: over the index";
    }
}

// 点击按钮
void MainWindow::sTimer20ms_Control(void)
{
    if(mDataTimer20ms.isActive())
    {
        mDataTimer20ms.stop();
        button_timer_control->setText("开始");
    }
    else
    {
        mDataTimer20ms.start(20);
        button_timer_control->setText("结束");
    }
}

void MainWindow::sProcessItemActiveState(QListWidgetItem *current, QListWidgetItem *previous)
{
    if(!previous)
    {
        current->setIcon(QIcon(":/Icon/active_car.png"));
    }
    else
    {
        switch (this->list_function->row(current)) {
            case 0:
                current->setIcon(QIcon(":/Icon/active_car.png"));
            break;

            case 1:
                current->setIcon(QIcon(":/Icon/active_detect.png"));
                break;

            case 2:
                current->setIcon(QIcon(":/Icon/active_path.png"));
                break;

            case 3:
                current->setIcon(QIcon(":/Icon/active_track.png"));
                break;

            default:
                current->setIcon(QIcon(":/Icon/active_car.png"));
                break;
        }
        switch (this->list_function->row(previous)) {
            case 0:
                previous->setIcon(QIcon(":/Icon/unactive_car.png"));
            break;

            case 1:
                previous->setIcon(QIcon(":/Icon/unactive_detect.png"));
                break;

            case 2:
                previous->setIcon(QIcon(":/Icon/unactive_path.png"));
                break;

            case 3:
                previous->setIcon(QIcon(":/Icon/unactive_track.png"));
                break;

            default:
                previous->setIcon(QIcon(":/Icon/unactive_car.png"));
                break;
        }
    }
}

void MainWindow::sCAN_Connect(void)
{
//    if(1 == mWinZlgCan.getConnectStatus())
//    {
//        mWinZlgCan.CanClose();
//        mCanRevWorkThread.quit();
//        this->button_CanOpen->setEnabled(false);
//        this->button_CanClose->setEnabled(false);
//        this->button_CanConnect->setText("连接");
//    }
//    else
//    {
//        mWinZlgCan.CanConnect();
//        if(1 == mWinZlgCan.getConnectStatus())
//        {
//            this->button_CanOpen->setEnabled(true);
//            this->button_CanClose->setEnabled(false);
//            this->button_CanConnect->setText("断开");
//        }
//    }
}

void MainWindow::sCAN_Open(void)
{
//    if((0 == mWinZlgCan.CanOpen(0)) && (0 == mWinZlgCan.CanOpen(1)) && (0 == mWinZlgCan.CanOpen(2)))
//    {
//        mCanRevWorkThread.start();
//        mWinZlgCan.setOpenStatus(1);
//        this->button_CanOpen->setEnabled(false);
//        this->button_CanClose->setEnabled(true);
//    }
//    else
//    {
//        QMessageBox::information(this, "错误", "CAN设备启动失败");
//        mWinZlgCan.setOpenStatus(0);
//        this->button_CanClose->setEnabled(false);
//        this->button_CanOpen->setEnabled(false);
//    }
}

void MainWindow::sCAN_Close(void)
{
//    mWinZlgCan.CanReset(0);
//    mWinZlgCan.CanReset(1);
//    mWinZlgCan.CanReset(2);
//    mWinZlgCan.setOpenStatus(0);
//    mCanRevWorkThread.quit();
//    this->button_CanOpen->setEnabled(true);
//    this->button_CanClose->setEnabled(false);
}

//void MainWindow::sDisplayPercaption(Percaption *p)
//{
//    label_FrontObstacleDistance_Value->setText(QString::number(static_cast<double>(p->getFrontObstacleDistance().distance)));
//    label_FrontObstacleRegion_Value->setText(obstacle_region[p->getFrontObstacleDistance().region]);
//    label_FrontObstacleStatus_Value->setText(obstacle_status[p->getRearObstacleDistance().status]);

//    label_RearObstacleDistance_Value->setText(QString::number(static_cast<double>(p->getRearObstacleDistance().distance)));
//    label_RearObstacleRegion_Value->setText(obstacle_region[p->getRearObstacleDistance().region]);
//    label_RearObstacleStatus_Value->setText(obstacle_status[p->getRearObstacleDistance().status]);

    // calculate and add a new data point to each graph:
//    mDetectGraph1->addData(mDetectGraph1->dataCount(), qSin(mDetectGraph1->dataCount()/50.0)+qSin(mDetectGraph1->dataCount()/50.0/0.3843)*0.25);
//    mDetectGraph2->addData(mDetectGraph2->dataCount(), qCos(mDetectGraph2->dataCount()/50.0)+qSin(mDetectGraph2->dataCount()/50.0/0.4364)*0.15);
//    mDetectGraph1->addData(mDetectGraph1->dataCount(), static_cast<double>(p->getFrontObstacleDistance().distance));
//    mDetectGraph2->addData(mDetectGraph2->dataCount(), static_cast<double>(p->getRearObstacleDistance().distance));

//    // make key axis range scroll with the data:
//    mDetectPlot->xAxis->rescale();
//    mDetectGraph1->rescaleValueAxis(false, true);
//    mDetectGraph2->rescaleValueAxis(false, true);
//    mDetectPlot->xAxis->setRange(mDetectPlot->xAxis->range().upper, 100, Qt::AlignRight);

//    // update the vertical axis tag positions and texts to match the rightmost data point of the graphs:
//    double graph1Value = mDetectGraph1->dataMainValue(mDetectGraph1->dataCount()-1);
//    double graph2Value = mDetectGraph2->dataMainValue(mDetectGraph2->dataCount()-1);
//    mDetectTag1->updatePosition(graph1Value);
//    mDetectTag2->updatePosition(graph2Value);
//    mDetectTag1->setText(QString::number(graph1Value, 'f', 2));
//    mDetectTag2->setText(QString::number(graph2Value, 'f', 2));

//    mDetectPlot->replot();
//}

// 注入文件选择
void MainWindow::sPercaptionDataFileSelect(void)
{
    QString file_name;
    file_name = QFileDialog::getOpenFileName(this,tr("数据注入文件"),"",tr("text(*.txt)"));

    if(!file_name.isNull())
    {
        detect_file = new QFile(file_name);
        if(detect_file->exists())
        {
            if(!detect_file->open(QIODevice::ReadOnly))
            {
                QMessageBox::warning(this,tr("打开错误"),tr("打开文件错误：") + detect_file->errorString());
                return;
            }
            else//打开成功
            {
                FileDataInit();
                while(!detect_file->atEnd())
                {
                    QByteArray baLine = detect_file->readLine();
                    AnalyzeOneLine(baLine);
                }
                NewFileUpdateFlag = 0xA5;
                time_step_cnt     = 0;
                if(radio_right_enter_location->isChecked()){mUltrasonicObstaclePercption.Command = RIGHT_PARKING_START_CMD;}
                if(radio_left_enter_location->isChecked()){mUltrasonicObstaclePercption.Command = LEFT_PARKING_START_CMD;}
                if(radio_center_enter_location->isChecked()){mUltrasonicObstaclePercption.Command = ENTER_PARKING_START_CMD;}

                detect_file->close();
            }
        }
        else
        {
            qDebug() << "sPercaptionDataFileSelect: File do not exit！";
            return;
        }
    }
    else
    {
        qDebug("sPercaptionDataFileSelect: Cancel");
    }
}

/**
 * @brief MainWindow::sCalculateDetect: 执行感知检测的相关计算
 */
void MainWindow::sCalculateDetect(void)
{
    mUltrasonicObstaclePercption.Command = 0x57;
}

/**
 * @brief sParallelPlanSelect: radio buttom check the plan type
 */
void MainWindow::sParallelPlanSelect()
{
    text_ParkingLength->setText("6");
    text_ParkingWidth->setText("2.4");

    mPercaption.PositionX = text_VehicleInitPointX->text().toFloat();
    mPercaption.PositionY = text_VehicleInitPointY->text().toFloat();
    mPercaption.AttitudeYaw = text_VehicleInitPointYaw->text().toFloat();

    mPercaption.ParkingLength = text_ParkingLength->text().toFloat();
    mPercaption.ParkingWidth  = text_ParkingWidth->text().toFloat();

    mGeometricTrack.Init(mPercaption.PositionX,mPercaption.PositionY,mPercaption.AttitudeYaw);

    ParkingPointX[1] = 0;
    ParkingPointX[2] = 0;
    ParkingPointX[3] = static_cast<double>(mPercaption.ParkingLength);
    ParkingPointX[4] = static_cast<double>(mPercaption.ParkingLength);

    ParkingPointY[1] = 0;
    ParkingPointY[2] = static_cast<double>(-mPercaption.ParkingWidth);
    ParkingPointY[3] = static_cast<double>(-mPercaption.ParkingWidth);
    ParkingPointY[4] = 0;

    mPathParkingCurveG1->setData(ParkingPointX,ParkingPointY);

    QVector<double> ParkingTrackPointX(1),ParkingTrackPointY(1);
    ParkingTrackPointX[0] = static_cast<double>(mGeometricTrack.getPosition().getX());
    ParkingTrackPointY[0] = static_cast<double>(mGeometricTrack.getPosition().getY());
    mPathVehicleCenterCurveG1->setData(ParkingTrackPointX,ParkingTrackPointY);
}

/**
 * @brief MainWindow::sVerticalPlanSelect: radio buttom check the plan type
 */
void MainWindow::sVerticalPlanSelect()
{
    text_ParkingLength->setText("2.4");
    text_ParkingWidth->setText("5.5");

    mPercaption.PositionX = text_VehicleInitPointX->text().toFloat();
    mPercaption.PositionY = text_VehicleInitPointY->text().toFloat();
    mPercaption.AttitudeYaw = text_VehicleInitPointYaw->text().toFloat();

    mPercaption.ParkingLength = text_ParkingLength->text().toFloat();
    mPercaption.ParkingWidth  = text_ParkingWidth->text().toFloat();

    mGeometricTrack.Init(mPercaption.PositionX,mPercaption.PositionY,mPercaption.AttitudeYaw);

    ParkingPointX[1] = 0;
    ParkingPointX[2] = 0;
    ParkingPointX[3] = static_cast<double>(mPercaption.ParkingLength);
    ParkingPointX[4] = static_cast<double>(mPercaption.ParkingLength);

    ParkingPointY[1] = 0;
    ParkingPointY[2] = static_cast<double>(-mPercaption.ParkingWidth);
    ParkingPointY[3] = static_cast<double>(-mPercaption.ParkingWidth);
    ParkingPointY[4] = 0;

    mPathParkingCurveG1->setData(ParkingPointX,ParkingPointY);

    QVector<double> ParkingTrackPointX(1),ParkingTrackPointY(1);
    ParkingTrackPointX[0] = static_cast<double>(mGeometricTrack.getPosition().getX());
    ParkingTrackPointY[0] = static_cast<double>(mGeometricTrack.getPosition().getY());
    mPathVehicleCenterCurveG1->setData(ParkingTrackPointX,ParkingTrackPointY);
}

/**
 * @brief MainWindow::sParkingConfirm : 库位确认按钮
 */
void MainWindow::sParkingConfirmG1()
{
    mPercaption.PositionX = text_VehicleInitPointX->text().toFloat();
    mPercaption.PositionY = text_VehicleInitPointY->text().toFloat();
    mPercaption.AttitudeYaw = text_VehicleInitPointYaw->text().toFloat();

    mPercaption.ParkingLength = text_ParkingLength->text().toFloat();
    mPercaption.ParkingWidth  = text_ParkingWidth->text().toFloat();

    mGeometricTrack.Init(mPercaption.PositionX,mPercaption.PositionY,mPercaption.AttitudeYaw);

    ParkingPointX[1] = 0;
    ParkingPointX[2] = 0;
    ParkingPointX[3] = static_cast<double>(mPercaption.ParkingLength);
    ParkingPointX[4] = static_cast<double>(mPercaption.ParkingLength);

    ParkingPointY[1] = 0;
    ParkingPointY[2] = static_cast<double>(-mPercaption.ParkingWidth);
    ParkingPointY[3] = static_cast<double>(-mPercaption.ParkingWidth);
    ParkingPointY[4] = 0;

    mPathParkingCurveG1->setData(ParkingPointX,ParkingPointY);

    QVector<double> ParkingTrackPointX(1),ParkingTrackPointY(1);
    ParkingTrackPointX[0] = static_cast<double>(mGeometricTrack.getPosition().getX());
    ParkingTrackPointY[0] = static_cast<double>(mGeometricTrack.getPosition().getY());
    mPathVehicleCenterCurveG1->setData(ParkingTrackPointX,ParkingTrackPointY);

    if (radio_parallel->isChecked()) // parallel planning init and start commond
    {
        mParallelPlanning->Init();
        mParallelPlanning->Command = 0x60;
    }
    else if (radio_vertical->isChecked())
    {
        mVerticalPlanning->Init();
        mVerticalPlanning->Command = 0x60;
    }
}

/**
 * @brief MainWindow::sParkingConfirmHC for hc path generate
 */
void MainWindow::sParkingConfirmHC()
{
    mBaseState[0].x       = HC_VehicleStartPointX->text().toDouble();
    mBaseState[0].y       = HC_VehicleStartPointY->text().toDouble();
    mBaseState[0].psi     = HC_VehicleStartPointYaw->text().toDouble();
    mBaseState[0].kappa   = HC_VehicleStartPointKappa->text().toDouble();
    mBaseState[0].d       = 0.2;

    mBaseState[1].x     = HC_VehicleEndPointX->text().toDouble();
    mBaseState[1].y     = HC_VehicleEndPointY->text().toDouble();
    mBaseState[1].psi   = HC_VehicleEndPointYaw->text().toDouble();
    mBaseState[1].kappa = HC_VehicleEndPointKappa->text().toDouble();
    mBaseState[1].d     = 0.2;

    VectorArrowShow(mBaseState[0], &mHeadState[0], mStartArrow);
    VectorArrowShow(mBaseState[1], &mHeadState[1], mEndArrow);

    vector<State> path_points = mHC_ReedsSheppStateSpace->getPath(mBaseState[0], mBaseState[1]);
    HC_CC_PathShow(path_points);

    HC_CC_RS_Path* circle_path = mHC_ReedsSheppStateSpace->getCirclePath(mBaseState[0], mBaseState[1]);
    HC_CC_CircleShow(circle_path);
}

void MainWindow::sParkingConfirmG2()
{   
    mBaseState[0].x       = text_VehicleStartPointX->text().toDouble();
    mBaseState[0].y       = text_VehicleStartPointY->text().toDouble();
    mBaseState[0].psi     = text_VehicleStartPointYaw->text().toDouble();
    mBaseState[0].kappa   = text_VehicleStartPointKappa->text().toDouble();
    mBaseState[0].d       = 1.0;

    mBaseState[1].x     = text_VehicleEndPointX->text().toDouble();
    mBaseState[1].y     = text_VehicleEndPointY->text().toDouble();
    mBaseState[1].psi   = text_VehicleEndPointYaw->text().toDouble();
    mBaseState[1].kappa = text_VehicleEndPointKappa->text().toDouble();
    mBaseState[1].d     = 1.0;
}

//id:
// 0 -> right circle
// 1 -> left  circle
void MainWindow::sPathCirclePoint(uint8_t id,Circle *c)
{
    switch(id)
    {
        case 0:
        mPathRightCircle->topLeft->setCoords(c->Center.getX() - c->Radius,c->Center.getY() + c->Radius);
        mPathRightCircle->bottomRight->setCoords(c->Center.getX() + c->Radius,c->Center.getY() - c->Radius);
            break;

        case 1:
        mPathLeftCircle->topLeft->setCoords(c->Center.getX() - c->Radius,c->Center.getY() + c->Radius);
        mPathLeftCircle->bottomRight->setCoords(c->Center.getX() + c->Radius,c->Center.getY() - c->Radius);
            break;

        default:
            break;
    }
}

/**
 * @brief MainWindow::sPathGenarate 路径生成按钮事件
 */
void MainWindow::sPathGenarate(void)
{
    uint16_t curvature_type=0;

    if(radio_sin_curvature->isChecked())
    {
        curvature_type = 1;
    }
    else if(radio_double_line->isChecked())
    {
        curvature_type = 2;
    }
    else if(radio_circle_curvature->isChecked())
    {
        curvature_type = 3;
    }
    else
    {
        curvature_type = 0;
    }
//    _target_curvature_data_sets->Delete();
//    mCurvature->GenerateCurvatureSets(_target_curvature_data_sets,curvature_type);
//    m_TrajectoryAnalyzer->Init(_target_curvature_data_sets);
//    TargetPathShow(_target_curvature_data_sets);

    mCurvature->GenerateCurvaturePointSets(_target_curvature_vectors,curvature_type);
    m_TrajectoryAnalyzer->Init(_target_curvature_vectors);
//    m_TrajectoryAnalyzer->TrajectoryTransformToCOM(m_LatControl_LQR->getLr());
    TargetPathShow(_target_curvature_vectors);
}

// 鼠标拖拽功能
/**
 * @brief 鼠标按下事件处理
 * @param event
 */
void MainWindow::sMousePressEvent(QMouseEvent *event)
{
    int x_pos = event->pos().x();
    int y_pos = event->pos().y();

    double x_val = mHC_PathPlot->xAxis->pixelToCoord(x_pos);
    double y_val = mHC_PathPlot->yAxis->pixelToCoord(y_pos);

    for (int var = 0; var < 2; ++var)
    {
        if(fabs(mBaseState[var].x - x_val) < 0.3 &&
           fabs(mBaseState[var].y - y_val) < 0.3 )
        {
            m_base_state_index = var;
            break;
        }
    }
    for (int var = 0; var < 2; ++var)
    {
        if(fabs(mHeadState[var].x - x_val) < 0.3 &&
           fabs(mHeadState[var].y - y_val) < 0.3 )
        {
            m_head_state_index = var;
            break;
        }
    }
}

/**
 * @brief 鼠标释放事件处理
 * @param event
 */
void MainWindow::sMouseReleaseEvent(QMouseEvent *event)
{
    mHC_PathPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    int x_pos = event->pos().x();
    int y_pos = event->pos().y();

    double x_val = mHC_PathPlot->xAxis->pixelToCoord(x_pos);
    double y_val = mHC_PathPlot->yAxis->pixelToCoord(y_pos);

    if(m_base_state_index >= 0 && m_base_state_index < 2)
    {
        mBaseState[m_base_state_index].x = x_val;
        mBaseState[m_base_state_index].y = y_val;
    }
    if(m_head_state_index >= 0 && m_head_state_index < 2)
    {
        Vector2d base, head;
        base.setX(mBaseState[m_head_state_index].x);
        base.setY(mBaseState[m_head_state_index].y);
        head.setX(x_val);
        head.setY(y_val);
        mBaseState[m_head_state_index].psi = (head - base).Angle();
    }

    VectorArrowShow(mBaseState[0], &mHeadState[0], mStartArrow);
    VectorArrowShow(mBaseState[1], &mHeadState[1], mEndArrow);

    m_base_state_index = -1;
    m_head_state_index = -1;
}

/**
 * @brief 鼠标移动事件处理
 * @param event
 */
void MainWindow::sMouseMoveEvent(QMouseEvent *event)
{
    mHC_PathPlot->setInteractions(QCP::iRangeZoom);

    int x_pos = event->pos().x();
    int y_pos = event->pos().y();

    double x_val = mHC_PathPlot->xAxis->pixelToCoord(x_pos);
    double y_val = mHC_PathPlot->yAxis->pixelToCoord(y_pos);

    if(-1 == m_base_state_index && -1 == m_head_state_index)
    {
        return;
    }
    if(m_base_state_index >= 0 && m_base_state_index < 2)
    {
        mBaseState[m_base_state_index].x = x_val;
        mBaseState[m_base_state_index].y = y_val;
    }
    if(m_head_state_index >= 0 && m_head_state_index < 2)
    {
        Vector2d base, head;
        base.setX(mBaseState[m_head_state_index].x);
        base.setY(mBaseState[m_head_state_index].y);
        head.setX(x_val);
        head.setY(y_val);
        mBaseState[m_head_state_index].psi = (head - base).Angle();
    }

    VectorArrowShow(mBaseState[0], &mHeadState[0], mStartArrow);
    VectorArrowShow(mBaseState[1], &mHeadState[1], mEndArrow);

    vector<State> path_points = mHC_ReedsSheppStateSpace->getPath(mBaseState[0], mBaseState[1]);
    HC_CC_PathShow(path_points);

    HC_CC_RS_Path* circle_path = mHC_ReedsSheppStateSpace->getCirclePath(mBaseState[0], mBaseState[1]);
    HC_CC_CircleShow(circle_path);
}

void MainWindow::sTrackStart(void)
{
    mBoRuiController->setDistance(10);
    mBoRuiController->setVelocity(0.2f);
    mBoRuiController->setAPAEnable(1);
    mBoRuiController->setGear(Drive);

    QVector<double> PointX(1),PointY(1);
    PointX[0] = 0;
    PointY[0] = 0;
    mTrackVehicleCenterCurve->setData(PointX,PointY);

    mGeometricTrack.Init();
    mTrackPlot->replot();
}
