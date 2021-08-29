/*
 * controller.h
 *
 *  Created on: 2019年1月3日
 *      Author: Henry Zhu
 */

#ifndef INTERFACE_CONTROLLER_H_
#define INTERFACE_CONTROLLER_H_

#include <QMainWindow>
#include "Interaction/CANBUS/Interface/vehicle_controller.h"
#include "Interaction/CANBUS/Interface/message_manager.h"
#include "Control/Common/pid.h"

class Controller {
public:
	Controller();
	virtual ~Controller();

	virtual void Init() = 0;

	virtual void Proc(MessageManager *msg,VehicleController *ctl,PID *pid) = 0;
};

#endif /* INTERFACE_CONTROLLER_H_ */
