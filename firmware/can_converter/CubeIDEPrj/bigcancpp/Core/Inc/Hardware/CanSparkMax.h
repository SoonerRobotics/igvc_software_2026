 /*
 * CanSparkMax.h
 *
 *  Created on: Oct 13, 2025
 *      Author: m
 */
#include "main.h"
#ifndef SRC_HARDWARE_CANSPARKMAX_H_
#define SRC_HARDWARE_CANSPARKMAX_H_

#define DEVICE_TYPE 2
#define MANUFACTURER_CODE 5
#define ENABLE_API_CLASS 11
#define ENABLE_API_INDEX 0


#define NONRIO_HEARTBEAT_API_CLASS 11
#define NONRIO_HEARTBEAT_API_INDEX 2
#define NONRIO_HEARTBEAT_DATA {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00}

#define ABSOLUTE_ENCODER_FEEDBACK_API_CLASS 6
#define ABSOLUTE_ENCODER_FEEDBACK_API_INDEX 37

#define DRIVE_ENCODER_FEEDBACK_API_CLASS 6
#define DRIVE_ENCODER_FEEDBACK_API_INDEX 33

#define PERCENT_OUTPUT_API_CLASS 0
#define PERCENT_OUTPUT_API_INDEX 2

#define ENCODER_API_CLASS 6
#define ENCODER_API_INDEX 2

#define POSITION_API_CLASS 3
#define POSITION_API_INDEX 2

#define VELOCITY_API_CLASS 1
#define VELOCITY_API_INDEX 2

#define PARAMETER_API_CLASS 48
#define PARAMETER_API_INDEX 0


class CanSparkMax
{
public:
	CanSparkMax(uint8_t id, bool reversed);
	bool setPosition(float value);
	bool setVelocity(float value);
	float getAbsolutePosition();
	float getDrivePosition();
	float getRPM();
	float encoderToRadians(float encoder_reading);
	bool sendHeartbeat();
    void handleFeedback(uint8_t api_class, uint8_t api_index, const uint8_t data[8]);

private:
	bool sendSparkMsg(uint8_t  api_index, uint8_t  api_class,uint8_t id, uint8_t  dlc, uint8_t data[8]);
	void floatToData(float f, uint8_t* data);
	uint8_t deviceID;
	bool isReversed;
	float absolute_position_ = 0.0f;
	float drive_position_ = 0.0f;
	float rpm_ = 0.0f;
};




#endif /* SRC_HARDWARE_CANSPARKMAX_H_ */


