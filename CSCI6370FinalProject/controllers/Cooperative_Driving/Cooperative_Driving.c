//Oscar Layton Gonzalez
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/device.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>


// referencing array items through indexing
enum {X, Y, Z};

#define TIME_STEP 40
#define UNKNOWN 999999999.99

// Line following PID: KP, KI, KD
#define KP 1.85
#define KI .00001
#define KD 13.08

bool PID_reset = false;

// Size of the white line angle filter
#define FILTER_SIZE_WHITE 3

// other features
bool Avoid_collision = false;

bool has_gps = false;
bool has_camera = false;

// camera specs
WbDeviceTag camera;
int camera_w = 5;
int camera_h = 3;
double camera_fov = -1.0;

// SICK laser specs
WbDeviceTag sick;
int sick_w = -1;
double sick_range = -1.0;
double sick_fov = -1.0;

// GPS
WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// variables
double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool autodrive = true;

void print_help() {
  printf("You can drive the car now!\n");
  printf("Select the 3D window and then use the cursor keys to:\n");
  printf("[LEFT]/[RIGHT] - steer control\n");
  printf("[UP]/[DOWN] - accelerate/decelerate and reverse\n");
}

void set_autonomousdriving(bool onoff) {
  if (autodrive == onoff)
    return;
  autodrive = onoff;
  switch (autodrive) {
    case false:
      printf("switching to manual drive...\n");
      printf("hit [A] to return to auto-drive.\n");
      break;
    case true:
      if (has_camera)
        printf("switching to auto-drive...\n");
      else
        printf("impossible to switch auto-drive on without a camera\n");
      break;
  }
}

// set target speed function
void set_speed(double kmh) {
  // max speed
  if (kmh > 100.0)
    kmh = 100.0;

  speed = kmh;

  printf("setting speed to %g km/h\n", kmh);
  wbu_driver_set_cruising_speed(kmh);
}

// positive = turn right, negative = turn left
void set_steering_angle(double wheel_angle) {
  // limit the difference with previous steering_angle
  if (wheel_angle - steering_angle > 0.1)
    wheel_angle = steering_angle + 0.1;
  if (wheel_angle - steering_angle < -0.1)
    wheel_angle = steering_angle - 0.1;
  steering_angle = wheel_angle;
  // limit a range of the steering angle
  if (wheel_angle > 0.5)
    wheel_angle = 0.5;
  else if (wheel_angle < -0.5)
    wheel_angle = -0.5;
  wbu_driver_set_steering_angle(wheel_angle);
}

void change_manual_steer_angle(int inc) {
  set_autonomousdriving(false);

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
    case WB_KEYBOARD_UP:
      set_speed(speed + 5.0);
      break;
    case WB_KEYBOARD_DOWN:
      set_speed(speed - 5.0);
      break;
    case WB_KEYBOARD_RIGHT:
      change_manual_steer_angle(+1);
      break;
    case WB_KEYBOARD_LEFT:
      change_manual_steer_angle(-1);
      break;
    case 'A':
      set_autonomousdriving(true);
      break;
  }
}


void broadcast_message(char *message) {
  
}


// compute red green blue (rgb) difference
int color_difference(const unsigned char a[3], const unsigned char b[3]) {
  int i, diff = 0;
  for (i = 0; i < 3; i++) {
    int d = a[i] - b[i];
    diff += d > 0 ? d : -d;
  }
  return diff;
}

// returns approximate angle of white road line
// or UNKNOWN value set if no pixel of white line is visible
double camera_image_process(const unsigned char *image) {
  int num_pixels = camera_h * camera_w;  // number of pixels in the image
  const unsigned char REF[3] = {255, 255, 255};    // white lines (RGB format)
  int sumx = 0;                                   // summed x position of pixels
  int pixel_count = 0;                            // white pixels count

  const unsigned char *pixel = image;
  int x;
  for (x = 0; x < num_pixels; x++, pixel += 4) {
    if (color_difference(pixel, REF) < 70) {
      sumx += x % camera_w;
      pixel_count++;  // count the white pixels
    }
  }

  // for if no pixels were detected
  if (pixel_count == 0)
    return UNKNOWN;

  return ((double)sumx / pixel_count / camera_w - 0.71) * camera_fov;
}

// filter angle of the white line
double filter_angle(double new_value) {
  static bool first_call = true;
  static double old_value[FILTER_SIZE_WHITE];
  int i;

  if (first_call || new_value == UNKNOWN) {  // reset all the old values to 0.0
    first_call = false;
    for (i = 0; i < FILTER_SIZE_WHITE; ++i)
      old_value[i] = 0.0;
  } else {  // shift older values
    for (i = 0; i < FILTER_SIZE_WHITE - 1; ++i)
      old_value[i] = old_value[i + 1];
  }

  if (new_value == UNKNOWN)
    return UNKNOWN;
  else {
    old_value[FILTER_SIZE_WHITE - 1] = new_value;
    double sum = 0.0;
    for (i = 0; i < FILTER_SIZE_WHITE; ++i)
      sum += old_value[i];
    return (double)sum / FILTER_SIZE_WHITE;
  }
}

// returns angle calculated of obstacle
// or UNKNOWN value set if no obstacle was detected
#define RIGHT_TURN 1
#define LEFT_TURN -1
double sick_data_process(const float *sick_data, double *obstacle_dist, double *obstacle_angle) {
  const int HALF_AREA = 50;  // check 20 degrees wide middle area
  int sumx = 0;
  int collision_count = 0;
  int x;
  *obstacle_dist = 0.0;
  for (x = sick_w / 2 - HALF_AREA; x < sick_w / 2 + HALF_AREA; x++) {
    float range = sick_data[x];
    if (range < 10.0) {
      sumx += x;
      collision_count++;
      *obstacle_dist += range;
    }
  }

  // if no obstacle was detected...
  if (collision_count == 0) {
    *obstacle_angle = UNKNOWN;
    return UNKNOWN;
  }

  *obstacle_dist = *obstacle_dist / collision_count;
  *obstacle_angle = ((double)sumx / collision_count / sick_w - 0.5) * sick_fov;

  // Determine which way to turn to avoid the obstacle
  double turn_direction = UNKNOWN;
  if (*obstacle_angle < 0) {
    turn_direction = RIGHT_TURN;
  } else if (*obstacle_angle > 0) {
    turn_direction = LEFT_TURN;
  }

  // Broadcast obstacle distance, angle, and turn direction to other cars
  char message[256];
  sprintf(message, "obstacle %f %f %f", *obstacle_dist, *obstacle_angle, turn_direction);
  broadcast_message(message);

  return *obstacle_angle;
}

double applyPID(double white_line_angle) {
  static double oldValue = 0.0;
  static double integral = 0.0;

  if (PID_reset) {
    oldValue = white_line_angle;
    integral = 0.0;
    PID_reset = false;
  }

  // anti-windup mechanism
  if (signbit(white_line_angle) != signbit(oldValue))
    integral = 0.0;

  double diff = white_line_angle - oldValue;

  // limit
  if (integral < 30 && integral > -30)
    integral += white_line_angle;

  oldValue = white_line_angle;
  return KP * white_line_angle + KI * integral + KD * diff;
}

int main(int argc, char **argv) {
  wbu_driver_init();

  // check if there is a SICK
  int j = 0;
  for (j = 0; j < wb_robot_get_number_of_devices(); ++j) {
    WbDeviceTag device = wb_robot_get_device_by_index(j);
    const char *name = wb_device_get_name(device);
    if (strcmp(name, "Sick LMS 291") == 0)
      Avoid_collision = true;
    else if (strcmp(name, "gps") == 0)
      has_gps = true;
    else if (strcmp(name, "camera") == 0)
      has_camera = true;
  }

  // camera device
  if (has_camera) {
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, TIME_STEP);
    camera_w = wb_camera_get_width(camera);
    camera_h = wb_camera_get_height(camera);
    camera_fov = wb_camera_get_fov(camera);
  }

  // SICK sensor
  if (Avoid_collision) {
    sick = wb_robot_get_device("Sick LMS 291");
    wb_lidar_enable(sick, TIME_STEP);
    sick_w = wb_lidar_get_horizontal_resolution(sick);
    sick_range = wb_lidar_get_max_range(sick);
    sick_fov = wb_lidar_get_fov(sick);
  }

  // initialize gps
  if (has_gps) {
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
  }

  // start engine
  if (has_camera)
    set_speed(10.0);  // km/h
  wbu_driver_set_hazard_flashers(true);
  wbu_driver_set_dipped_beams(true);
  wbu_driver_set_antifog_lights(true);
  wbu_driver_set_wiper_mode(SLOW);

  print_help();

  // allow to switch to manual control
  wb_keyboard_enable(TIME_STEP);

  // main loop
  while (wbu_driver_step() != -1) {
    // get user input
    check_keyboard();
    static int i = 0;

    // updates sensors only every TIME_STEP milliseconds
    if (i % (int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0) {
      // read sensors
      const unsigned char *camera_image = NULL;
      const float *sick_data = NULL;
      if (has_camera)
        camera_image = wb_camera_get_image(camera);
      if (Avoid_collision)
        sick_data = wb_lidar_get_range_image(sick);

      if (autodrive && has_camera) {
        double white_line_angle = filter_angle(camera_image_process(camera_image));
        double obstacle_dist;
        double obstacle_angle;
        if (Avoid_collision)
          obstacle_angle = sick_data_process(sick_data, &obstacle_dist, &obstacle_angle);
        else {
          obstacle_angle = UNKNOWN;
          obstacle_dist = 0;
        }

        // avoid obstacles and follow white line
        if (Avoid_collision && obstacle_angle != UNKNOWN) {
          // an obstacle has been detected
          wbu_driver_set_brake_intensity(0.0);
          // compute the steering angle required to avoid the obstacle
          double obstacle_steering = steering_angle;
          if (obstacle_angle > 0.0 && obstacle_angle < 0.4)
            obstacle_steering = steering_angle + (obstacle_angle - 0.25) / obstacle_dist;
          else if (obstacle_angle > -0.4)
            obstacle_steering = steering_angle + (obstacle_angle + 0.25) / obstacle_dist;
          double steer = steering_angle;
          // if we see the line we determine the best steering angle to both avoid obstacle and follow the line
          if (white_line_angle != UNKNOWN) {
            const double line_following_steering = applyPID(white_line_angle);
            if (obstacle_steering > 0 && line_following_steering > 0)
              steer = obstacle_steering > line_following_steering ? obstacle_steering : line_following_steering;
            else if (obstacle_steering < 0 && line_following_steering < 0)
              steer = obstacle_steering < line_following_steering ? obstacle_steering : line_following_steering;
          } else
            PID_reset = true;
          // apply the computed required angle
          set_steering_angle(steer);
        } else if (white_line_angle != UNKNOWN) {
          // no obstacle has been detected, simply follow the line
          wbu_driver_set_brake_intensity(0.0);
          set_steering_angle(applyPID(white_line_angle));
        } else {
          // no obstacle has been detected but we lost the line => we brake and hope to find the line again
          wbu_driver_set_brake_intensity(0.4);
          PID_reset = true;
        }
      }

    }

    ++i;
  }

  wbu_driver_cleanup();

  return 0;  // ignored
}