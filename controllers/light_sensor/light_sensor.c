/*
 * Copyright 1996-2018 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  An example of a controller using a light sensor device.
 */

#include <stdio.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <math.h>
#include <webots/supervisor.h>

#define MAX_SPEED 10
#define SPEED 6
#define TIME_STEP 64
#define FAST_ROTATE 1.0
#define SLOW_ROTATE 0.3
#define RADIUS 0.045

static double initializer(){
   printf("Starting initialization \n");
   //setting device tags
   WbDeviceTag ls1, ls0, ls2, left_motor, right_motor;
   
   //get handlers for devices
   left_motor = wb_robot_get_device("left wheel motor");
   right_motor = wb_robot_get_device("right wheel motor");
   ls0 = wb_robot_get_device("ls0");
   ls1 = wb_robot_get_device("ls1");
   ls2 = wb_robot_get_device("ls2");
  
   double left_speed = -FAST_ROTATE;
   double right_speed = FAST_ROTATE;
   double max = -1.0;
   double previous = -1.0;
   printf("All set! max is: %f\n", max);
   int found = 0;
   wb_motor_set_velocity(left_motor, left_speed);
   wb_motor_set_velocity(right_motor, right_speed);
   while(wb_robot_step(TIME_STEP) != 1){
     double reading = wb_light_sensor_get_value(ls1);
     //printf("reading before round is: %f\n", reading);
     reading = (int)(reading * 1000 + .5);
     reading = (float)reading/1000;
     printf("reading is: %f & max is: %f & previous is: %f\n",reading, max, previous);
     if(reading == max && found == 1)
       break;
     else if(reading > max){
      // printf("New max!\n");
       max=reading;
     }
     else if (reading < max && reading < previous && found == 0){
       //printf("Passed max. Rotating.....\n");
       found = 1;
       left_speed *= SLOW_ROTATE;
       right_speed *= -SLOW_ROTATE;
       wb_motor_set_velocity(left_motor, left_speed);
       wb_motor_set_velocity(right_motor, right_speed);
     }
      else if (reading == previous)
        printf("compiler is happy\n");
      else if (reading < previous && found ==1){
       //printf("Rotating.....\n");
       left_speed *= -1;
       right_speed *= -1;
       wb_motor_set_velocity(left_motor, left_speed);
       wb_motor_set_velocity(right_motor, right_speed);
     }
     previous = reading;
   }
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  const double ls0_value = wb_light_sensor_get_value(ls0);
  const double ls1_value = wb_light_sensor_get_value(ls1);
  const double ls2_value = wb_light_sensor_get_value(ls2);
  double w = (-2 * pow(RADIUS,2))/( (2/ls0_value) - (1/ls1_value) - (1/ls2_value));
  return w;
}

int main() {
  WbDeviceTag ls0, ls1, ls2, left_motor, right_motor;

  wb_robot_init();

  /* get a handler to the distance sensors. */
  ls0 = wb_robot_get_device("ls0");
  ls1 = wb_robot_get_device("ls1");
  ls2 = wb_robot_get_device("ls2");

  wb_light_sensor_enable(ls0, TIME_STEP);
  wb_light_sensor_enable(ls1, TIME_STEP);
  wb_light_sensor_enable(ls2, TIME_STEP);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  

  
  while (wb_robot_step(TIME_STEP) != 1) {
  
    /* read sensor values */
    const double ls0_value = wb_light_sensor_get_value(ls0);
    const double ls1_value = wb_light_sensor_get_value(ls1);
    const double ls2_value = wb_light_sensor_get_value(ls2);

    double w = (-2 * pow(RADIUS,2))/( (2/ls0_value) - (1/ls1_value) - (1/ls2_value));
    printf("w is: %f\n", w);
    double Watt = w * 4 * M_PI;
    printf("Watt are aprox.: %f\n", Watt);
    //double rdistance = sqrt(5/(4*M_PI*ls1_value));
    //double ldistance = sqrt(5/(4*M_PI*ls0_value));
    //printf("Distance from right is: %f\n", rdistance);
    //printf("Distance from left is: %f\n", ldistance);
    
    //double robot_distance = sqrt((pow(rdistance, 2) + pow(ldistance, 2) - 2* pow(0.045,2))/2);
    //double robot_angle = asin((pow(robot_distance,2) + pow(0.045,2) - pow(ldistance, 2))/(robot_distance * 0.045));
    
    //printf("Robot distance is: %f\n", robot_distance);
   // printf("Angle with beacon is: %f\n", robot_angle);
    
    //printf("Left light sensor: %f\n", ls0_value);
    //printf("Right light sensor: %f\n", ls1_value);

    //double left_speed = -1.0;
    //left_speed = (left_speed < MAX_SPEED) ? left_speed : MAX_SPEED;
    //double right_speed = (1024 - ls1_value) / 100.0;
    //double right_speed = 1.0;

    /* Set the motor speeds. */
   //wb_motor_set_velocity(left_motor, left_speed);
   //wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}