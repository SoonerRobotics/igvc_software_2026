/*
 * CanSparkMax.cpp
 *
 *  Created on: Oct 13, 2025
 *      Author: m
 */

#include "CanSparkMax.h"
#include "main.h"
#include <cstring>   // for C++
#include "usb_device.h"
#include "usbd_cdc_if.h"
extern CAN_HandleTypeDef hcan2;
uint8_t enablemsg[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

extern CanSparkMax* s_spark_registry[64];
CanSparkMax::CanSparkMax(uint8_t id, bool reversed)
    : deviceID(id), isReversed(reversed) {
	uint16_t period_ms = 50;
	uint8_t data[8] = { (uint8_t)(period_ms & 0xFF), (uint8_t)(period_ms >> 8), 0,0,0,0,0,0 };
	if (id == 8) {
		enablemsg[1] = enablemsg[1] + 1;
	}
	else {
	enablemsg[0] = enablemsg[0] + (1 << id);
	}
	s_spark_registry[deviceID] = this;
	sendSparkMsg(2, 6, deviceID, 2, data);

	    // position feedback (API_CLASS=6, API_INDEX=34)
	sendSparkMsg(3, 6, deviceID, 2, data);

	    // absolute position feedback (API_CLASS=6, API_INDEX=37)
	sendSparkMsg(5, 6, deviceID, 2, data);

}

bool CanSparkMax::sendSparkMsg(uint8_t  api_index, uint8_t  api_class, uint8_t id, uint8_t  dlc, uint8_t data[8]) {
	uint32_t can_id =  (DEVICE_TYPE <<24) | (MANUFACTURER_CODE << 16) | (api_class << 10) | (api_index <<6) | (deviceID);
	CAN_TxHeaderTypeDef tx;
	tx.IDE = CAN_ID_EXT;
	tx.RTR = CAN_RTR_DATA;
	tx.ExtId = can_id;
	tx.DLC = dlc;
	uint32_t mailbox;
	uint8_t can_data[8] = {0};           // always 8 bytes
	for (int i = 0; i < dlc; ++i) {
	    can_data[i] = data[i];
	}
	if (HAL_CAN_AddTxMessage(&hcan2,&tx,can_data, &mailbox) != HAL_OK) {
		return false;
	}
	return true;
}
bool CanSparkMax::sendHeartbeat() {

	return CanSparkMax::sendSparkMsg(NONRIO_HEARTBEAT_API_INDEX,NONRIO_HEARTBEAT_API_CLASS,0,8,enablemsg);
}
bool CanSparkMax::setPosition(float value) {
	uint8_t buf[8] = {0};
	floatToData(value,buf);
	return CanSparkMax::sendSparkMsg(POSITION_API_INDEX,POSITION_API_CLASS,deviceID,8,buf);
}
bool CanSparkMax::setVelocity(float value){
	uint8_t buf[8] = {0};
	floatToData((value),buf);
	return CanSparkMax::sendSparkMsg(VELOCITY_API_INDEX,VELOCITY_API_CLASS,deviceID,8,buf);

}
void CanSparkMax::floatToData(float f, uint8_t* data)
{
	std::memcpy(data, &f, sizeof(float));
}

void CanSparkMax::handleFeedback(uint8_t api_class,
                                uint8_t api_index,
                                const uint8_t data[8])
{
    float f;
    std::memcpy(&f, data, sizeof(float));

    char msg[96];
    int len = 0;

    if (api_class == ABSOLUTE_ENCODER_FEEDBACK_API_CLASS && api_index == 5) {
        absolute_position_ = f;

        len = snprintf(msg, sizeof(msg),
                       "Spark %u | ABS_POS = %.6f\r\n",
                       (unsigned)deviceID,
                       (double)absolute_position_);
    }
    else if (api_class == DRIVE_ENCODER_FEEDBACK_API_CLASS && api_index == 2) {
        drive_position_ = f;

        len = snprintf(msg, sizeof(msg),
                       "Spark %u | DRIVE_POS = %.6f\r\n",
                       (unsigned)deviceID,
                       (double)drive_position_);
    }
    else if (api_class == ENCODER_API_CLASS && api_index == 1) {
        rpm_ = f;

        len = snprintf(msg, sizeof(msg),
                       "Spark %u | RPM = %.3f\r\n",
                       (unsigned)deviceID,
                       (double)rpm_);
    }

    //if (len > 0 && deviceID == 2) {
    //    (void)CDC_Transmit_FS((uint8_t*)msg, (uint16_t)len);
    //}
}


float CanSparkMax::getAbsolutePosition()
{
    return absolute_position_;
}

float CanSparkMax::getDrivePosition()
{
    return drive_position_;
}

float CanSparkMax::getRPM()
{
    return rpm_;
}
