/*
 * bo_rui_message.h
 *
 *  Created on: 2019年3月15日
 *      Author: Henry Zhu
 */

#ifndef CANBUS_BORUI_BO_RUI_MESSAGE_H_
#define CANBUS_BORUI_BO_RUI_MESSAGE_H_

#include <QMainWindow>
#include "./Common/Utils/Inc/property.h"
#include "./Interaction/CANBUS/Interface/message_manager.h"
#include "./Common/Math/crc_compute.h"
#include "Common/Configure/Configs/vehilce_config.h"


class BoRuiMessage  : public MessageManager
{
public:
	BoRuiMessage();
	virtual ~BoRuiMessage();

	void Init() override;
    void Parse(const uint32_t id,const uint8_t *data,const uint32_t lenght) override;

private:

};

#endif /* CANBUS_BORUI_BO_RUI_MESSAGE_H_ */
