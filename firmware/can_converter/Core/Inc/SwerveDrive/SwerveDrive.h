/*
 * SwerveDrive.h
 *
 *  Created on: Nov 19, 2025
 *      Author: m
 */

#ifndef INC_SWERVEDRIVE_SWERVEDRIVE_H_
#define INC_SWERVEDRIVE_SWERVEDRIVE_H_
#include "SwerveModule.h"
#include "arm_math.h"

typedef struct
{
    SwerveModule* front_left;
    SwerveModule* front_right;
    SwerveModule* back_left;
    SwerveModule* back_right;

} SwerveDriveConfig;

typedef struct
{
    double delta_x;
    double delta_y;
    double delta_theta;
} SwerveDriveState;

class SwerveDrive
{
public:
	SwerveDrive(SwerveDriveConfig& config);
	SwerveDriveState updateState(SwerveDriveState& state);
public:
	SwerveDriveConfig config_;
	SwerveModule front_left_module_;
	SwerveModule front_right_module_;
	SwerveModule back_left_module_;
	SwerveModule back_right_module_;
	void debug_print();
    float drive_kinematics_data_[8][3];
    arm_matrix_instance_f32 drive_kinematics_mat_;

    // Input state u (3×1): [vx, vy, w]
    float input_vel_[3];
    arm_matrix_instance_f32 input_vel_mat_;
    float wheel_velocities_[8];
    arm_matrix_instance_f32 wheel_velocities_mat_;
    float drive_kinematics_inv_data_[3 * 8];
    arm_matrix_instance_f32 drive_kinematics_inv_mat_;
    float wheel_meas_data_[8];
    arm_matrix_instance_f32 wheel_meas_mat_;
    float vel_est_data_[3];
    arm_matrix_instance_f32 vel_est_mat_;
};



#endif /* INC_SWERVEDRIVE_SWERVEDRIVE_H_ */
