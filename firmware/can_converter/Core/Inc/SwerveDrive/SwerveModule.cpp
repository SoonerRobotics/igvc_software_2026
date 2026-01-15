/*
 * SwerveModule.cpp
 *
 *  Created on: Nov 19, 2025
 *      Author: m
 */
#include "SwerveModule.h"
#include <cmath>


SwerveModule::SwerveModule(SwerveModuleConfig config)
    : config_(config),
      drive_motor_(config.drive_motor_id,  config.is_drive_motor_reversed),
      angle_motor_(config.angle_motor_id,  config.is_angle_motor_reversed),
      drive_motor_conversion_factor_(driveMotorGearRatio/(2*wheel_radius *M_PI)),
      angle_motor_conversion_factor_(steerMotorGearRatio),
      last_set_angle_(0.0),
	  drive_motor_last_position(0.0),
	  current_angle_motor_position(0.0),
	  last_drive_motor_delta(0.0)
{

}
float SwerveModule::getDriveDelta() {
	if (drive_motor_last_position == 0.0) {
		drive_motor_last_position = drive_motor_.getDrivePosition()/drive_motor_conversion_factor_;
		return 0.0;
	}
	float current_drive_motor_position = drive_motor_.getDrivePosition()/drive_motor_conversion_factor_;
	last_drive_motor_delta = (current_drive_motor_position - drive_motor_last_position);
	drive_motor_last_position = current_drive_motor_position;
	current_angle_motor_position = angle_motor_.getAbsolutePosition() * 2 * M_PI;
	return last_drive_motor_delta;
}


SwerveModuleState SwerveModule::updateState(SwerveModuleState desired_state) {

	double desired_drive_speed = sqrt(desired_state.x_vel * desired_state.x_vel + desired_state.y_vel * desired_state.y_vel);
	double desired_angle = atan2(desired_state.y_vel, desired_state.x_vel);

	double angle_error = desired_angle - last_set_angle_;
	if (angle_error > M_PI) {
		angle_error = (2*M_PI) - angle_error;
	}
	else if (angle_error < -M_PI) {
		angle_error = (2*M_PI) + angle_error;
	}
	if (std::fabs(angle_error) > (M_PI/2.0)) {
		desired_drive_speed = -desired_drive_speed;
		desired_angle += M_PI;
	}
	if (desired_angle > M_PI) {
		desired_angle = desired_angle -(2*M_PI);
	}
	else if (desired_angle < -M_PI) {
		desired_angle = desired_angle +(2*M_PI);
	}

	if (std::fabs(desired_drive_speed) > 0.3) {
		last_set_angle_ = desired_angle;
		angle_motor_.setPosition((desired_angle)/(2*M_PI));
	}
	desired_drive_speed = desired_drive_speed * cos(desired_angle - angle_motor_.getAbsolutePosition()*2*M_PI);

	drive_motor_.setVelocity(desired_drive_speed*42.0f); //no clue about 42 zemlin gap


	double measured_drive_speed = drive_motor_.getRPM()*2*M_PI*wheel_radius/60.0;
    double measured_angle = angle_motor_.getAbsolutePosition() *2*M_PI* angle_motor_conversion_factor_;
    SwerveModuleState measured_state;
    measured_state.x_vel = measured_drive_speed * cos(measured_angle);
    measured_state.y_vel = measured_drive_speed * sin(measured_angle);
    return measured_state;

}




