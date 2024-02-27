#include <webots/robot.h>
#include <webots/motor.h>

int main() {
  wb_robot_init();

  // Pobierz uchwyt do silnika (śmigła)
  WbDeviceTag propeller1 = wb_robot_get_device("motor_1");//blue
  WbDeviceTag propeller2 = wb_robot_get_device("motor_2");//yellow
  WbDeviceTag propeller3 = wb_robot_get_device("motor_3");//green
  WbDeviceTag propeller4 = wb_robot_get_device("motor_4"); //red
  
  // Ustaw silnik w tryb prędkości
  wb_motor_set_position(propeller1, INFINITY);
  wb_motor_set_position(propeller2, INFINITY);
  wb_motor_set_position(propeller3, INFINITY);
  wb_motor_set_position(propeller4, INFINITY);



  // Ustaw prędkość obrotową śmigła
  
  wb_motor_set_velocity(propeller1, 8.0);
  wb_motor_set_velocity(propeller2, 8.0);
  wb_motor_set_velocity(propeller3, 8.0);
  wb_motor_set_velocity(propeller4, 8.0);  // Ustaw prędkość na 10 radianów na sekundę


  int i = 0;
  // Pętla główna
  while (wb_robot_step(64) != -1) {
    // Możesz dodać dodatkową logikę sterowania tutaj
  }

  wb_robot_cleanup();
  return 0;
}
