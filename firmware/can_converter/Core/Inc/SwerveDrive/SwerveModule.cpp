/*
 * SwerveModule.cpp
 *
 *  Created on: Nov 19, 2025
 *      Author: m
 */
#include "SwerveModule.h"
#include <cmath>

float driveMotorGearRatio = 1.0f / ((10.0f / 60) * (24.0f / 60));
float wheelRadius = 0.1016f;
float driveMotorConversion = wheelRadius * (driveMotorGearRatio / (2 * wheelRadius * M_PI));

float radiansA(double degrees) {
    return degrees * M_PI / 180.0f;
}

SwerveModule::SwerveModule(SwerveModuleConfig config)
    : config_(config),
      drive_motor_(config.drive_motor_id,  config.is_drive_motor_reversed),
      angle_motor_(config.angle_motor_id,  config.is_angle_motor_reversed),
      last_set_angle_(0.0),
	  drive_motor_last_position(0.0),
	  current_angle_motor_position(0.0),
	  last_drive_motor_delta(0.0)
{

}
float SwerveModule::getDriveDelta() {
	if (drive_motor_last_position == 0.0) {
		drive_motor_last_position = drive_motor_.getDrivePosition() / driveMotorConversion;
		return 0.0;
	}

	float current_drive_motor_position = drive_motor_.getDrivePosition() / driveMotorConversion;
	last_drive_motor_delta = (current_drive_motor_position - drive_motor_last_position);
	drive_motor_last_position = current_drive_motor_position;
	current_angle_motor_position = angle_motor_.getAbsolutePosition();
	return last_drive_motor_delta;
}


void SwerveModule::updateState(SwerveModuleState desired_state) {

	double desired_drive_speed = sqrt(desired_state.delta_x * desired_state.delta_x + desired_state.delta_y * desired_state.delta_y);
	double desired_angle = atan2(desired_state.delta_y, desired_state.delta_x);

	double angle_error = desired_angle - last_set_angle_;
	if (angle_error > M_PI) {
		angle_error = (2*M_PI) - angle_error;
	}
	else if (angle_error < -M_PI) {
		angle_error = (2*M_PI) + angle_error;
	}
	if (std::fabs(angle_error) > radiansA(90)) {
		desired_drive_speed *= -1;
		desired_angle += radiansA(180);
	}
	if (desired_angle > M_PI) {
		desired_angle = desired_angle - (2 * M_PI);
	}
	else if (desired_angle < -M_PI) {
		desired_angle = desired_angle + (2 * M_PI);
	}

	if (std::fabs(desired_drive_speed) > 0.3) {
		last_set_angle_ = desired_angle;
		angle_motor_.setPosition((desired_angle)/(2*M_PI));
	}
//	desired_drive_speed *= cos(desired_angle - angle_motor_.getAngle());
	drive_motor_.setVelocity(desired_drive_speed * 42.0f); //no clue about 42 zemlin gap
}

void SwerveModule::debugPrint()
{
    char msg[96];
    int len = snprintf(
        msg,
        sizeof(msg),
		"d=%.3f",
        getDriveDelta()
    );
    CDC_Transmit_FS((uint8_t*)msg, len);
}




