/*
 * Copyright 1996-2020 Cyberbotics Ltd.
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
 * Description:   Autonoumous vehicle controller example
 */

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// to be used as array indices
enum { X, Y, Z };

#define TIME_STEP 4
#define TIME_STEP_REC 4
#define UNKNOWN 99999.99

// Line following PID
#define KP 0.25
#define KI 0.006
#define KD 2

bool PID_need_reset = false;

// Size of the yellow line angle filter
#define FILTER_SIZE 3

// enabe various 'features'
bool enable_collision_avoidance = false;
bool enable_display = false;
bool has_gps = false;
bool has_camera = false;

// speedometer
WbDeviceTag display;
int display_width = 0;
int display_height = 0;
WbImageRef speedometer_image = NULL;

// GPS
WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// misc variables
double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool autodrive = true;

void print_help() {
  printf("You can drive this car!\n");
  printf("Select the 3D window and then use the cursor keys to:\n");
  printf("[LEFT]/[RIGHT] - steer\n");
  printf("[UP]/[DOWN] - accelerate/slow down\n");
}

// set target speed
void set_speed(double kmh) {
  // max speed
  if (kmh > 250.0)
    kmh = 250.0;

  speed = kmh;

  printf("setting speed to %g km/h\n", kmh);
  wbu_driver_set_cruising_speed(kmh);
}

// positive: turn right, negative: turn left
void set_steering_angle(double wheel_angle) {
  // limit the difference with previous steering_angle
  if (wheel_angle - steering_angle > 0.1)
    wheel_angle = steering_angle + 0.1;
  if (wheel_angle - steering_angle < -0.1)
    wheel_angle = steering_angle - 0.1;
  steering_angle = wheel_angle;
  // limit range of the steering angle
  if (wheel_angle > 0.5)
    wheel_angle = 0.5;
  else if (wheel_angle < -0.5)
    wheel_angle = -0.5;
  wbu_driver_set_steering_angle(wheel_angle);
}

void change_manual_steer_angle(int inc) {
  double new_manual_steering = manual_steering + inc;
  if (new_manual_steering <= 25.0 && new_manual_steering >= -25.0) {
    manual_steering = new_manual_steering;
    set_steering_angle(manual_steering * 0.02);
  }

  if (manual_steering == 0)
    printf("going straight\n");
  else
    printf("turning %.2f rad (%s)\n", steering_angle, steering_angle < 0 ? "left" : "right");
}

void check_keyboard() {
  int key = wb_keyboard_get_key();
  switch (key) {
    case 'W':
      set_speed(speed + 0.3);
      break;
    case 'S':
      set_speed(speed - 0.3);
      break;
    case 'D':
      change_manual_steer_angle(+1);
      break;
    case 'A':
      change_manual_steer_angle(-1);
      break;
    // case 'A':
      // set_autodrive(true);
      // break;
  }
}


void update_display() {
  const double NEEDLE_LENGTH = 50.0;

  // display background
  wb_display_image_paste(display, speedometer_image, 0, 0, false);

  // draw speedometer needle
  double current_speed = wbu_driver_get_current_speed();
  if (isnan(current_speed))
    current_speed = 0.0;
  double alpha = current_speed / 260.0 * 3.72 - 0.27;
  int x = -NEEDLE_LENGTH * cos(alpha);
  int y = -NEEDLE_LENGTH * sin(alpha);
  wb_display_draw_line(display, 100, 95, 100 + x, 95 + y);

  // draw text
  char txt[64];
  sprintf(txt, "GPS coords: %.1f %.1f", gps_coords[X], gps_coords[Z]);
  wb_display_draw_text(display, txt, 10, 130);
  sprintf(txt, "GPS speed:  %.1f", gps_speed);
  wb_display_draw_text(display, txt, 10, 140);
}

int main(int argc, char **argv) {

  wbu_driver_init();
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP_REC);
  wb_receiver_set_channel(receiver, WB_CHANNEL_BROADCAST);
 

   WbNodeRef robot_node = wb_supervisor_node_get_from_def("TeslaModel3");
   WbFieldRef trans_vehicle = wb_supervisor_node_get_field(robot_node, "translation");
   WbFieldRef rot_vehicle = wb_supervisor_node_get_field(robot_node, "rotation");


  // start engine
  wbu_driver_set_hazard_flashers(true);
  wbu_driver_set_dipped_beams(true);
  wbu_driver_set_antifog_lights(true);
  wbu_driver_set_wiper_mode(SLOW);
  print_help();
  // allow to switch to manual control
  wb_keyboard_enable(TIME_STEP);

  // main loop
  while (wbu_driver_step(8) != -1) {
    // get user input
    check_keyboard();
    static int i = 0;
    
    continue;  // for running
                 
    // for data collection
    while (wb_receiver_get_queue_length(receiver) > 0) {    
      double *message = (double *)wb_receiver_get_data(receiver);
      printf("received: %f , %f\n",message[0], message[1]);
      wb_receiver_next_packet(receiver);
      
      continue;
      
      // reset the robot
      double INITIAL_TRANS[3] = {0.0};
      INITIAL_TRANS[0] = 0.0;
      INITIAL_TRANS[1] = 0.165;
      INITIAL_TRANS[2] = 0.0;
      double step_mov = 2.5;
      int step = 2.5;
      
      double INITIAL_ROT[4] = {0.0};  //[x,y,z,r] where y = 1
      INITIAL_ROT[1] = 1.0; 
      double rot_step = M_PI/4;
      
      double receiver_msg0 = message[0];
      double receiver_msg1 = message[1];
      
      for(int i = -step; i <= step; i++ )
      {
          INITIAL_TRANS[0] =  receiver_msg0 + step_mov*i;
          printf("INITIAL_TRANS[0] %f , message[0] %f \n", INITIAL_TRANS[0], message[0]);
          
          wb_supervisor_field_set_sf_vec3f(trans_vehicle, INITIAL_TRANS);
          for(int j = -step; j <= step; j++)
          {
             INITIAL_TRANS[2] =  receiver_msg1 + step_mov*j;
             wb_supervisor_field_set_sf_vec3f(trans_vehicle, INITIAL_TRANS);
             
             for(float r = 0; r <= M_PI; r +=rot_step)
             {
                  INITIAL_ROT[3] += r;
                  wb_supervisor_field_set_sf_rotation(rot_vehicle, INITIAL_ROT);
                  wb_robot_step(32);
                  wb_receiver_next_packet(receiver);
             }
          }         
      }
 
     
    }
    
    ++i;
  }

  wbu_driver_cleanup();

  return 0;  // ignored
}
