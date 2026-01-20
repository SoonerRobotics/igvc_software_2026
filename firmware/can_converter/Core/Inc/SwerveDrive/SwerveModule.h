/*
 * SwerveModule.h
 *
 *  Created on: Nov 19, 2025
 *      Author: m
 */

#ifndef SRC_SWERVEDRIVE_SWERVEMODULE_H_
#define SRC_SWERVEDRIVE_SWERVEMODULE_H_

#include "Hardware/CanSparkMax.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

typedef struct
{
	double x_pos;
	double y_pos;
	uint8_t drive_motor_id;
	uint8_t angle_motor_id;
	bool is_drive_motor_reversed;
	bool is_angle_motor_reversed;

} SwerveModuleConfig;
typedef struct {
	double x_vel;
	double y_vel;
} SwerveModuleState;

class SwerveModule {

public:
	SwerveModule(SwerveModuleConfig config);
	SwerveModuleState updateState(SwerveModuleState desired_state);
	float getDriveDelta();
    double getXPos() const { return config_.x_pos; }
    double getYPos() const { return config_.y_pos; }
    double getCurrentAngleRad() const { return current_angle_motor_position; }
    void debugPrint();
public:
	SwerveModuleConfig config_;
	CanSparkMax drive_motor_;
	CanSparkMax angle_motor_;
	double driveMotorGearRatio = 16.0f;
	double steerMotorGearRatio = 25.0f;
	double wheel_radius = .1016f;
	double drive_motor_conversion_factor_;
	double angle_motor_conversion_factor_;
	double last_set_angle_;
	double drive_motor_last_position;
	double current_angle_motor_position;
	double last_drive_motor_delta;

};




#endif /* SRC_SWERVEDRIVE_SWERVEMODULE_H_ */
