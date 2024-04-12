#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/motor.h>

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Get and enable devices.
 WbDeviceTag imu = wb_robot_get_device("inertial unit");
 wb_inertial_unit_enable(imu, timestep);
 WbDeviceTag gps = wb_robot_get_device("gps");
 wb_gps_enable(gps, timestep);
 WbDeviceTag compass = wb_robot_get_device("compass");
 wb_compass_enable(compass, timestep);
 WbDeviceTag gyro = wb_robot_get_device("gyro");
 wb_gyro_enable(gyro, timestep);
 wb_keyboard_enable(timestep);
 

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag propeller1 = wb_robot_get_device("motor_1");//blue
  WbDeviceTag propeller2 = wb_robot_get_device("motor_2");//yellow
  WbDeviceTag propeller3 = wb_robot_get_device("motor_3");//green
  WbDeviceTag propeller4 = wb_robot_get_device("motor_4"); //red
  WbDeviceTag motors[4] = {propeller1, propeller2, propeller3, propeller4};;
  int m;
  for (m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 2.0);
  }

  // Display the welcome message.
  printf("Start the drone...\n");

  // Wait one second.
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }

  // Display manual control message.
  printf("You can control the drone with your computer keyboard:\n");
  printf("- 'up': move forward.\n");
  printf("- 'down': move backward.\n");
  printf("- 'right': turn right.\n");
  printf("- 'left': turn left.\n");
  printf("- 'shift + up': increase the target altitude.\n");
  printf("- 'shift + down': decrease the target altitude.\n");
  printf("- 'shift + right': strafe right.\n");
  printf("- 'shift + left': strafe left.\n");

  // Constants, empirically found.
  const double k_vertical_thrust = 10;  // with this thrust, the drone lifts.
  const double k_vertical_offset = 0.5;   // Vertical offset where the robot actually targets to stabilize itself.
  const double k_vertical_p = 1.0;        // P constant of the vertical PID.
  const double k_roll_p = 5.0;           // P constant of the roll PID.
  const double k_pitch_p = 3.0;          // P constant of the pitch PID.

  // Variables.
  double target_altitude = 1.0;  // The target altitude. Can be changed by the user.

  // Main loop
  while (wb_robot_step(timestep) != -1) {
   // const double time = wb_robot_get_time();  // in seconds.

    // Retrieve robot position using the sensors.
   const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
   const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
   const double altitude = wb_gps_get_values(gps)[2];
   const double roll_velocity = wb_gyro_get_values(gyro)[0];
   const double pitch_velocity = wb_gyro_get_values(gyro)[1];


    // Transform the keyboard input to disturbances on the stabilization algorithm.
    double roll_disturbance = 0.0;
    double pitch_disturbance = 0.0;
    double yaw_disturbance = 0.0;
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_RIGHT:
          pitch_disturbance = -3.5;
          break;
        case WB_KEYBOARD_LEFT:
          pitch_disturbance = 3.5;
          break;
        case WB_KEYBOARD_UP:
          yaw_disturbance = -5.3;
          break;
        case WB_KEYBOARD_DOWN:
          yaw_disturbance = 5.3;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
          roll_disturbance = -1.0;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
          roll_disturbance = 1.0;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
          target_altitude += 0.05;
          printf("target altitude: %f [m]\n", target_altitude);
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
          target_altitude -= 0.05;
          printf("target altitude: %f [m]\n", target_altitude);
          break;
      }
      key = wb_keyboard_get_key();
    }

    // Compute the roll, pitch, yaw and vertical inputs.
    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
    const double yaw_input = yaw_disturbance;
    const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

    // Actuate the motors taking into consideration all the computed inputs.
    const double propeller1_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double propeller2_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    const double propeller3_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double propeller4_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    wb_motor_set_velocity(propeller1, propeller1_input);
    wb_motor_set_velocity(propeller2, -propeller2_input);
    wb_motor_set_velocity(propeller3, -propeller3_input);
    wb_motor_set_velocity(propeller4, propeller4_input);
  };

 wb_robot_cleanup();

  return EXIT_SUCCESS;
}