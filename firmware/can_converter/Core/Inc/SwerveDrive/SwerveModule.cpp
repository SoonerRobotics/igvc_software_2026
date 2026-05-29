/*
 * SwerveModule.cpp
 *
 *  Created on: Nov 19, 2025
 *      Author: m
 */
#include "SwerveModule.h"
#include <cmath>

// Constnats
static constexpr float kGearRatio = 14.2f; // motor rotations per wheel rotation
static constexpr float kWheelRadiusMeters = 6.0f * 0.0254f; // 6 inch wheel radius in meters
static constexpr float kWheelCircumference = M_PI * kWheelRadiusMeters; // wheel circumference

// Calculations
static constexpr float kMpsToRpm = (kGearRatio * 60.0f) / kWheelCircumference; // conversion factor from m/s to RPM
static constexpr float kRpmToMeters = kWheelCircumference / kGearRatio;

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
    float current_pos_rotations = drive_motor_.getDrivePosition(); // raw motor rotations

    if (drive_motor_last_position == 0.0f) {
        drive_motor_last_position = current_pos_rotations;
        return 0.0f;
    }

    float delta_rotations = current_pos_rotations - drive_motor_last_position;
    drive_motor_last_position = current_pos_rotations;
    current_angle_motor_position = angle_motor_.getAngle();
	
    return delta_rotations * kRpmToMeters;
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

	if (std::fabs(desired_drive_speed) > 0.05) {
		last_set_angle_ = desired_angle;
		angle_motor_.setPosition((desired_angle)/(2*M_PI));
	}

	desired_drive_speed *= cos(desired_angle - getCurrentAngleRad());
	drive_motor_.setVelocity(desired_drive_speed * kMpsToRpm);
}





