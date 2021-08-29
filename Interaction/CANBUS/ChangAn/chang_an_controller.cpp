/*****************************************************************************/
/* FILE NAME: chang_an_controller.cpp           COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: chang an vehicle control class     					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      December 28 2018    Initial Version                  */
/*****************************************************************************/

#include <./Interaction/CANBUS/ChangAn/chang_an_controller.h>

ChangAnController::ChangAnController() {
	Init();
}

ChangAnController::~ChangAnController() {

}

void ChangAnController::Init()
{
	_steerig_angle_active_control_state = WaitSteeringAngleControlSingleState;
	_rolling_counter_torque_AEB 		= 0;
	_rolling_counter_brake_ACC 			= 0;
	_rolling_counter_steering_control 	= 0;
	_rolling_counter_gear_control 		= 0;

	/* ACC */
	_current_target_acceleration_ACC 			= 0;
	_current_target_acceleration_enable_single 	= 0;

	/* AEB */
	_current_target_deceleration_AEB 			= 0;
	_current_target_deceleration_enable_single 	= 0;

	/* Torque */
	_current_torque = 0;
	_current_torque_enable_single = 0;

	/* SteeringAngle */
	_steering_angle_set 			= 0;

	_current_steering_angle_target 					= 0;
	_current_steering_angle_target_active_single 	= 0;

	/* Gear */
	_current_gear 				= 0;
	_current_gear_enable_single = 0;
	_current_gear_valid_single 	= 0;
}

void ChangAnController::Start()
{
	GearEnable 	        = 1;
	SteeringEnable      = 1;

	AccelerationEnable  = 1;
	DecelerationEnable  = 1;
	TorqueEnable 	    = 1;
	VelocityEnable      = 1;
}

void ChangAnController::Stop()
{
	GearEnable 	        = 0;
	SteeringEnable      = 0;

	AccelerationEnable  = 0;
	DecelerationEnable  = 0;
	TorqueEnable 	    = 0;
	VelocityEnable      = 0;
}

//void ChangAnController::Update(ControlCommand cmd)
//{
//	GearEnable 	        = cmd.ControlEnable.B.GearEnable;
//	if( (0 == SteeringEnable) || (0 == cmd.ControlEnable.B.SteeringEnable) )
//	{
//		SteeringEnable  = cmd.ControlEnable.B.SteeringEnable;
//	}
//	AccelerationEnable  = cmd.ControlEnable.B.AccelerationEnable;
//	DecelerationEnable  = cmd.ControlEnable.B.DecelerationEnable;
//	TorqueEnable 	    = cmd.ControlEnable.B.TorqueEnable;
//	VelocityEnable      = cmd.ControlEnable.B.VelocityEnable;


//	Gear			  = cmd.Gear;
//	SteeringAngle 	  = cmd.SteeringAngle;
//	SteeringAngleRate = cmd.SteeringAngleRate;

//	if(0 == VelocityEnable)
//	{
//		Acceleration 	  = cmd.Acceleration;
//	}
//	Deceleration 	  = cmd.Deceleration;
//	Torque 			  = cmd.Torque;
//	Velocity          = cmd.Velocity;
//}

void ChangAnController::Update(APAControlCommand cmd)
{

}

void ChangAnController::Push(void)
{
	VehicleContorlNew();
//	VehicleContorl();
}

void ChangAnController::VehicleContorlStep1()
{
//	CAN_Packet m_CAN_Packet;
//	m_CAN_Packet.id = 0x6fe;
//	m_CAN_Packet.length = 8;
//	/// data buffer
//	/* ACC */
//	_current_target_acceleration_ACC = (uint8_t)((Acceleration + 5.0)*20);
//	_current_target_acceleration_enable_single = AccelerationEnable;
//	/* AEB */
//	_current_target_deceleration_AEB = (uint16_t)((Deceleration + 16.0) * 2000);
//	_current_target_deceleration_enable_single = DecelerationEnable;
//	/* Torque */
//	_current_torque = (uint16_t)(Torque * 10.23);
//	_current_torque_enable_single = TorqueEnable;
//	/* Steering Angle */
//	_current_steering_angle_target = (int16_t)(_steering_angle_set * 10);
//	_current_steering_angle_target_active_single = SteeringEnable;

//	/// Data Mapping
//	m_CAN_Packet.data[0] = _current_target_acceleration_ACC;
//	m_CAN_Packet.data[1] = (uint8_t)((_current_target_deceleration_AEB >> 8) & 0xFF);
//	m_CAN_Packet.data[2] = (uint8_t)((_current_target_deceleration_AEB     ) & 0xFF);
//	m_CAN_Packet.data[3] = (uint8_t)( _rolling_counter_torque_AEB & 0x0F);
//	m_CAN_Packet.data[3] = _current_target_acceleration_enable_single ?
//						   (uint8_t) ( m_CAN_Packet.data[3] | 0x80 ) :
//						   (uint8_t) ( m_CAN_Packet.data[3] & 0x7F ) ;
//	m_CAN_Packet.data[3] = _current_target_deceleration_enable_single ?
//						   (uint8_t) ( m_CAN_Packet.data[3] | 0x40 ) :
//						   (uint8_t) ( m_CAN_Packet.data[3] & 0xBF ) ;
//	m_CAN_Packet.data[4] = (uint8_t)((_current_torque >> 2) & 0xFF);
//	m_CAN_Packet.data[5] = (uint8_t)((_current_torque << 6) & 0xC0);
//	m_CAN_Packet.data[5] = _current_torque_enable_single              ?
//						   (uint8_t) ( m_CAN_Packet.data[5] | 0x20 ) :
//						   (uint8_t) ( m_CAN_Packet.data[5] & 0xDF ) ;
//	m_CAN_Packet.data[5] = (uint8_t) ((m_CAN_Packet.data[5] & 0xFC ) |
//									  ( _current_steering_angle_target_active_single & 0x03));
//	m_CAN_Packet.data[6] = (uint8_t)((_current_steering_angle_target >> 8) & 0xFF);
//	m_CAN_Packet.data[7] = (uint8_t)((_current_steering_angle_target     ) & 0xFF);
//	CAN1_TransmitMsg(m_CAN_Packet);
}

void ChangAnController::VehicleContorlStep2()
{
//	uint8_t i;
//	CAN_Packet m_CAN_Packet;
//	m_CAN_Packet.id = 0x6FF;
//	m_CAN_Packet.length = 8;
//	/// data buffer
//	_current_gear = Gear;
//	_current_gear_enable_single = GearEnable;
//	_current_gear_valid_single  = GearEnable;

//	/// data mapping
//	for(i=0;i<4;i++){m_CAN_Packet.data[i] = 0;}
//	m_CAN_Packet.data[4] = (uint8_t)(_rolling_counter_brake_ACC & 0x0f);
//	m_CAN_Packet.data[5] = (uint8_t)((_current_gear << 4 ) & 0x70);
//	m_CAN_Packet.data[5] = _current_gear_enable_single          ?
//						   (uint8_t) ( m_CAN_Packet.data[5] | 0x80 ) :
//						   (uint8_t) ( m_CAN_Packet.data[5] & 0x7F ) ;
//	m_CAN_Packet.data[5] = _current_gear_valid_single          ?
//						   (uint8_t) ( m_CAN_Packet.data[5] | 0x08 ) :
//						   (uint8_t) ( m_CAN_Packet.data[5] & 0xF7 ) ;

//	m_CAN_Packet.data[6] = (uint8_t)(
//								(
//									( _rolling_counter_brake_ACC                 & 0x0f )
//								+ 	((_current_target_acceleration_enable_single & 0x01 ) << 4)
//								+ 	  _current_target_acceleration_ACC
//								)^0xFF
//							);
//	m_CAN_Packet.data[7] = (uint8_t)(
//								(
//									( _rolling_counter_torque_AEB   & 0x0f)
//								+	((_current_torque_enable_single & 0x01) << 2)
//								+	((_current_torque & 0x001F) << 3)
//								+	((_current_torque & 0x03E0) >> 5)
//								+	((_current_target_deceleration_enable_single & 0x01) << 7)
//								+	((_current_target_deceleration_AEB >> 8) & 0xFF)
//								+	((_current_target_deceleration_AEB     ) & 0xFF)
//								) ^ 0xFF
//							);
//	CAN1_TransmitMsg(m_CAN_Packet);
}

void ChangAnController::VehicleContorlStep3()
{
//	uint8_t i;
//	CAN_Packet m_CAN_Packet;
//	m_CAN_Packet.id = 0x6E9;
//	m_CAN_Packet.length = 8;

//	/// data mapping
//	m_CAN_Packet.data[0] = 0;
//	m_CAN_Packet.data[1] = (uint8_t)(_rolling_counter_steering_control & 0x0f);
//	m_CAN_Packet.data[2] = (uint8_t)	(
//											(
//												(_rolling_counter_steering_control & 0x0f)
//											+	((_current_steering_angle_target >> 8) & 0xFF)
//											+	((_current_steering_angle_target     ) & 0xFF)
//											+	_current_steering_angle_target_active_single
//											) ^ 0xFF
//										);
//	m_CAN_Packet.data[3] = (uint8_t)	(
//											(
//												(((_current_gear_enable_single & 0x01) << 2)
//												^(((_current_gear_valid_single & 0x01) << 3) + _current_gear)
//												^(_rolling_counter_gear_control & 0x0F)) << 4
//											)|(_rolling_counter_gear_control & 0x0F)
//										);
//	for( i = 4 ;i < 8 ; i++ ){m_CAN_Packet.data[i] = 0;}
//	CAN1_TransmitMsg(m_CAN_Packet);
}

void ChangAnController::VehicleContorlNew()
{
//	CAN_Packet m_CAN_Packet;
//	m_CAN_Packet.id = 0x111;
//	m_CAN_Packet.length = 8;
//	/// data buffer
//	/* ACC */
//	_current_target_acceleration_ACC = (uint8_t)((Acceleration + 5.0)*20);
//	_current_target_acceleration_enable_single = AccelerationEnable;
//	/* AEB */
//	_current_target_deceleration_AEB = (uint16_t)((Deceleration + 16.0) * 2000);
//	_current_target_deceleration_enable_single = DecelerationEnable;
//	/* Torque */
//	_current_torque = (uint16_t)(Torque * 10.22);
//	_current_torque_enable_single = TorqueEnable;
//	/* Steering Angle */
//	_current_steering_angle_target = (int16_t)(_steering_angle_set * 10);
//	_current_steering_angle_target_active_single = SteeringEnable;

//	/// Data Mapping
//	m_CAN_Packet.data[0] = _current_target_acceleration_ACC;
//	m_CAN_Packet.data[1] = (uint8_t)((_current_target_deceleration_AEB >> 8) & 0xFF);
//	m_CAN_Packet.data[2] = (uint8_t)((_current_target_deceleration_AEB     ) & 0xFF);
//	m_CAN_Packet.data[3] = (uint8_t)( _rolling_counter_torque_AEB & 0x0F);
//	m_CAN_Packet.data[3] = _current_target_acceleration_enable_single ?
//						   (uint8_t) ( m_CAN_Packet.data[3] | 0x80 ) :
//						   (uint8_t) ( m_CAN_Packet.data[3] & 0x7F ) ;
//	m_CAN_Packet.data[3] = _current_target_deceleration_enable_single ?
//						   (uint8_t) ( m_CAN_Packet.data[3] | 0x40 ) :
//						   (uint8_t) ( m_CAN_Packet.data[3] & 0xBF ) ;
//	m_CAN_Packet.data[4] = (uint8_t)((_current_torque >> 2) & 0xFF);
//	m_CAN_Packet.data[5] = (uint8_t)((_current_torque << 6) & 0xC0);
//	m_CAN_Packet.data[5] = _current_torque_enable_single              ?
//						   (uint8_t) ( m_CAN_Packet.data[5] | 0x20 ) :
//						   (uint8_t) ( m_CAN_Packet.data[5] & 0xDF ) ;

//	m_CAN_Packet.data[5] = (uint8_t) ((m_CAN_Packet.data[5] & 0xFC ) |
//									  ( _current_steering_angle_target_active_single & 0x03));
//	m_CAN_Packet.data[6] = (uint8_t)((_current_steering_angle_target >> 8) & 0xFF);
//	m_CAN_Packet.data[7] = (uint8_t)((_current_steering_angle_target     ) & 0xFF);
//	CAN0_TransmitMsg(m_CAN_Packet);


//	m_CAN_Packet.id = 0x112;
//	m_CAN_Packet.length = 8;
//	/// data buffer
//	_current_gear = Gear;
//	_current_gear_enable_single = GearEnable;
////	_current_gear_valid_single = GearEnable;
//	/// data mapping
//	m_CAN_Packet.data[0] = 0;
//	m_CAN_Packet.data[1] = 0;
//	m_CAN_Packet.data[2] = 0;
//	m_CAN_Packet.data[3] = 0;
//	m_CAN_Packet.data[4] = 0;

//	m_CAN_Packet.data[5] = (uint8_t)((_current_gear << 4 ) & 0x70);
//	m_CAN_Packet.data[5] = _current_gear_enable_single          ?
//						   (uint8_t) ( m_CAN_Packet.data[5] | 0x80 ) :
//						   (uint8_t) ( m_CAN_Packet.data[5] & 0x7F ) ;
//	m_CAN_Packet.data[5] = _current_gear_valid_single          ?
//						   (uint8_t) ( m_CAN_Packet.data[5] | 0x08 ) :
//						   (uint8_t) ( m_CAN_Packet.data[5] & 0xF7 ) ;

//	m_CAN_Packet.data[6] = 0;
//	m_CAN_Packet.data[7] = 0;
//	CAN0_TransmitMsg(m_CAN_Packet);
}

void ChangAnController::VehicleContorl()
{
	VehicleContorlStep1();
	VehicleContorlStep2();
	VehicleContorlStep3();
	_rolling_counter_torque_AEB++;
	_rolling_counter_brake_ACC++;
	_rolling_counter_steering_control++;
	_rolling_counter_gear_control++;
}

void ChangAnController::SteeringAngleControl(float dt)
{
    float da = SteeringAngleRate * dt;
    float left_target_angle = SteeringAngle - da;
    float right_target_angle = SteeringAngle + da;

    if(_steering_angle_set < left_target_angle)
    {
    	_steering_angle_set += da;
//    	_steering_angle_set = steer_angle + da;
    }
    else if(_steering_angle_set > right_target_angle)
    {
    	_steering_angle_set -= da;
//    	_steering_angle_set = steer_angle - da;
    }
    else
    {
    	_steering_angle_set = SteeringAngle;
    }
}

void ChangAnController::SteeringAngleControlStateMachine(uint8_t fd)
{
	switch(_steerig_angle_active_control_state)
	{
		case WaitSteeringAngleControlSingleState:
			if(1 == SteeringEnable)
			{
				_steerig_angle_active_control_state = WaitFeedbackSingleState;
			}
			break;

		case WaitFeedbackSingleState:
			if(fd)
			{
				SteeringEnable = 2;
				_steerig_angle_active_control_state = WaitExistState;
			}
			break;

		case WaitExistState:
			if( (!fd) || ( 0 == SteeringEnable ))
			{
				_steerig_angle_active_control_state = WaitSteeringAngleControlSingleState;
			}
			break;

		default:
			_steerig_angle_active_control_state = WaitSteeringAngleControlSingleState;
			break;
	}
}

void ChangAnController::GearControlStateMachine(uint8_t fd)
{
	switch(_gear_active_control_state)
	{
	case WaitGearControlState:
		if(GearEnable == 1)
		{
			_gear_active_control_state = WaitGearFeedbackSingleState;
		}
		break;

	case WaitGearFeedbackSingleState:
		if(fd == 1)
		{
			_current_gear_valid_single = 1;
			_gear_active_control_state = WaitGearExistState;
		}
		break;

	case WaitGearExistState:
		if((GearEnable == 0) || (!fd))
		{
			_gear_active_control_state = WaitGearControlState;
		}
		break;

	default:

		break;

	}
}

void ChangAnController::EnableControl()
{
	if(APAEnable)
	{
		if(0 == SteeringEnable)
		{
			SteeringEnable  = 1;
		}
		GearEnable 	        = 1;
		AccelerationEnable  = 1;
//		DecelerationEnable  = 1;
		TorqueEnable 	    = 1;
		VelocityEnable      = 1;
	}
	else
	{
		Stop();
	}
}
