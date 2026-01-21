/*
 * SwerveDrive.cpp
 *
 *  Created on: Nov 19, 2025
 *      Author: m
 */

#include "SwerveDrive.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <math.h>
SwerveDrive::SwerveDrive(SwerveDriveConfig& config)
	: config_(config),
	  front_left_module_(*config.front_left),
	  front_right_module_(*config.front_right),
	  back_left_module_(*config.back_left),
	  back_right_module_(*config.back_right)
{
	arm_mat_init_f32(&drive_kinematics_mat_,8,3, &drive_kinematics_data_[0][0]);
	arm_mat_init_f32(&input_vel_mat_,3,1,input_vel_);
	arm_mat_init_f32(&wheel_velocities_mat_, 8, 1, wheel_velocities_);
    arm_mat_init_f32(&drive_kinematics_inv_mat_, 3, 8, drive_kinematics_inv_data_);
    arm_mat_init_f32(&wheel_meas_mat_, 8, 1, wheel_meas_data_);
    arm_mat_init_f32(&vel_est_mat_, 3, 1, vel_est_data_);
    auto fill_rows = [this](int row_index, const SwerveModule& m)
     {
         float x = static_cast<float>(m.getXPos());
         float y = static_cast<float>(m.getYPos());

         // row_index is even: vx row
         drive_kinematics_data_[row_index + 0][0] = 1.0f;
         drive_kinematics_data_[row_index + 0][1] = 0.0f;
         drive_kinematics_data_[row_index + 0][2] = -y;

         // row_index+1: vy row
         drive_kinematics_data_[row_index + 1][0] = 0.0f;
         drive_kinematics_data_[row_index + 1][1] = 1.0f;
         drive_kinematics_data_[row_index + 1][2] = x;
     };
    fill_rows(0, front_left_module_);
    fill_rows(2, front_right_module_);
    fill_rows(4, back_left_module_);
    fill_rows(6, back_right_module_);


    // Compute Pinv
    float At_data[3 * 8];
    float AtA_data[3 * 3];
    float AtA_inv_data[3 * 3];

    arm_matrix_instance_f32 At_mat;
    arm_matrix_instance_f32 AtA_mat;
    arm_matrix_instance_f32 AtA_inv_mat;

    arm_mat_init_f32(&At_mat,     3, 8, At_data);
    arm_mat_init_f32(&AtA_mat,    3, 3, AtA_data);
    arm_mat_init_f32(&AtA_inv_mat,3, 3, AtA_inv_data);

    // At = A^T
    arm_mat_trans_f32(&drive_kinematics_mat_, &At_mat);

    // AtA = At * A   (3x8 * 8x3 = 3x3)
    arm_mat_mult_f32(&At_mat, &drive_kinematics_mat_, &AtA_mat);

    // (AtA)^-1
    arm_status status = arm_mat_inverse_f32(&AtA_mat, &AtA_inv_mat);
    if (status == ARM_MATH_SUCCESS)
    {
        // drive_kinematics_inv_mat_ = (AtA)^-1 * At   (3x3 * 3x8 = 3x8)
        arm_mat_mult_f32(&AtA_inv_mat, &At_mat, &drive_kinematics_inv_mat_);
    }
    else
    {
    	while(1);
    }
}
SwerveDriveState SwerveDrive::updateState(SwerveDriveState& state)
{
    // 1) Pack robot motion into input_vel_ (treating x_vel, y_vel, angular_vel as deltas or velocities per step)
    input_vel_[0] = static_cast<float>(state.delta_x);        // vx or Δx
    input_vel_[1] = static_cast<float>(state.delta_y);        // vy or Δy
    input_vel_[2] = static_cast<float>(state.delta_theta);  // w  or Δθ



    // 2) Forward kinematics: wheel_velocities_ = A * input_vel_
    //    This gives per-module [vx_i, vy_i] in robot frame
    arm_mat_mult_f32(&drive_kinematics_mat_,
                     &input_vel_mat_,
                     &wheel_velocities_mat_);

    // 3) Send commands to each module (vx, vy for that wheel)
    SwerveModuleState fl_cmd;
    fl_cmd.delta_x = static_cast<double>(wheel_velocities_[0]);
    fl_cmd.delta_y = static_cast<double>(wheel_velocities_[1]);

    SwerveModuleState fr_cmd;
    fr_cmd.delta_x = static_cast<double>(wheel_velocities_[2]);
    fr_cmd.delta_y = static_cast<double>(wheel_velocities_[3]);

    SwerveModuleState bl_cmd;
    bl_cmd.delta_x = static_cast<double>(wheel_velocities_[4]);
    bl_cmd.delta_y = static_cast<double>(wheel_velocities_[5]);

    SwerveModuleState br_cmd;
    br_cmd.delta_x = static_cast<double>(wheel_velocities_[6]);
    br_cmd.delta_y = static_cast<double>(wheel_velocities_[7]);

    // Update each module (let them handle motor commands, internal state, etc.)
    front_left_module_.updateState(fl_cmd);
    front_right_module_.updateState(fr_cmd);
    back_left_module_.updateState(bl_cmd);
    back_right_module_.updateState(br_cmd);

    // 4) Read back what actually happened from each module:
    //
    // We assume:
    //  - getDriveDelta() returns distance (meters) this step
    //  - getCurrentAngleRad() returns steering angle (radians)
    //
    // Then project each wheel's motion into robot frame: vx_i = d * cos(theta), vy_i = d * sin(theta)

    // Front left
    {
        float d   = front_left_module_.getDriveDelta();
        float ang = static_cast<float>(front_left_module_.getCurrentAngleRad());
        wheel_meas_data_[0] = d * cosf(ang); // vx_fl
        wheel_meas_data_[1] = d * sinf(ang); // vy_fl
    }

    // Front right
    {
        float d   = front_right_module_.getDriveDelta();
        float ang = static_cast<float>(front_right_module_.getCurrentAngleRad());
        wheel_meas_data_[2] = d * cosf(ang); // vx_fr
        wheel_meas_data_[3] = d * sinf(ang); // vy_fr
    }

    // Back left
    {
        float d   = back_left_module_.getDriveDelta();
        float ang = static_cast<float>(back_left_module_.getCurrentAngleRad());
        wheel_meas_data_[4] = d * cosf(ang); // vx_bl
        wheel_meas_data_[5] = d * sinf(ang); // vy_bl
    }

    // Back right
    {
        float d   = back_right_module_.getDriveDelta();
        float ang = static_cast<float>(back_right_module_.getCurrentAngleRad());
        wheel_meas_data_[6] = d * cosf(ang); // vx_br
        wheel_meas_data_[7] = d * sinf(ang); // vy_br
    }

    // 5) Inverse kinematics (odometry): vel_est_ = A^+ * wheel_meas_
    arm_mat_mult_f32(&drive_kinematics_inv_mat_,
                     &wheel_meas_mat_,
                     &vel_est_mat_);

    // 6) Copy estimated robot motion back into a SwerveDriveState (convert float -> double)
    SwerveDriveState measured{};
    measured.delta_x       = static_cast<double>(vel_est_data_[0]); // Δx
    measured.delta_y       = static_cast<double>(vel_est_data_[1]); // Δy
    measured.delta_theta = static_cast<double>(vel_est_data_[2]); // Δθ

    return measured;
}
void SwerveDrive::debug_print()
{
    front_left_module_.debugPrint();
    front_right_module_.debugPrint();
    back_left_module_.debugPrint();
    back_right_module_.debugPrint();
}

