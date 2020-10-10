#include "struct.h"
#include "function.h"
extern int dist;
extern int start_flag;
extern int speed;
extern int mode;
extern int dist_left;
extern int outbreak_flag;
extern int outbreak_count;
extern int max_outbreak_thrshold;
extern int min_outbreak_thrshold;
extern int angle;
extern int backward_right_flag;
extern int parking_flag;
extern int interval;
extern int temp;
extern float ab[2];
extern float steer;
extern int return_flag;
extern bool check_yellow_line;

extern int finish_flag;
extern int traffic_light_flag;

void tunnel_run(){}
int get_distance(int channel)                       
{
  int data, volt, dist;
  data = DistanceSensor(channel);
  volt = data * 5000 / 4095;
  // volt = data_transform(data, 0, 4095, 0, 5000);
  dist = (27.61 / (volt - 0.1696))*1000;

  return dist;
}
void mode_outbreak()
{
  if(outbreak_flag == 0 && outbreak_count > max_outbreak_thrshold) {
    outbreak_flag = 1;
    DesireSpeed_Write(0);
  }
  else if(outbreak_flag == 1 && outbreak_count < min_outbreak_thrshold) {
    printf("restart!!!\n");
    outbreak_flag = 2;
    usleep(1000000);
    speed = 120;
    DesireSpeed_Write(speed);
  }
  else if(outbreak_flag == 2){
    CameraYServoControl_Write(1700);
    CarLight_Write(0x01);
    mode++;
  }
}
void return_lane_horizontal_right()
{
  SteeringServoControl_Write(2000);
  usleep(300000);
  DesireSpeed_Write(60);

  while(1){
    dist = get_distance(2);
    if(dist > 40){
      break;
    }
  }

  SteeringServoControl_Write(1000);
  usleep(300000);

  while(1){
    dist = get_distance(4);

    if(dist > 200){
      DesireSpeed_Write(0);
      usleep(300000);
      break;
    }
  }
}
void return_lane_vertical_right()
{
  angle = 1520;
  DesireSpeed_Write(80);

  while(1) {
    SteeringServoControl_Write(angle);
    dist = get_distance(4);

    if(dist < 20){
      angle = 1520;
      continue;
    }

    else if(dist > 170){
      DesireSpeed_Write(0);
      usleep(300000);
      break;
    }

    else if(dist > 30){
      angle = 1000;
      continue;
    }
  }
}
void go_backward_right()
{
  printf("go backward\n");
  SteeringServoControl_Write(1000);
  DesireSpeed_Write(-100);

  while(1) {
    dist = get_distance(5);

    if(dist < 70 && backward_right_flag == 0){
      backward_right_flag = 1;
    }
    else if(dist < 50 &&backward_right_flag == 2){
      DesireSpeed_Write(0);
      usleep(300000);
      parking_flag = 4;
      break;
    }
    else if(dist > 100 && backward_right_flag == 1){
        backward_right_flag = 2;
    }
  }
}

void horizontal_parking_right()
{
  dist = get_distance(3);
  interval = data_transform(dist , 0 , 30 , 0 , 400);
  angle = 1520 + interval;

  DesireSpeed_Write(120);
  SteeringServoControl_Write(angle);

  while(1){
    dist = get_distance(5);
    if(dist  > 100) {
      DesireSpeed_Write(0);
      break;
    }
  }
  SteeringServoControl_Write(2000);
  DesireSpeed_Write(-100);

  while(1){
    dist = get_distance(4);

    if(dist < 8 ) {
      DesireSpeed_Write(0);
      break;
    }

  }
  Alarm_Write(ON);
  usleep(1000000);
  Alarm_Write(OFF);
}
void vertical_parking()
{
  SteeringServoControl_Write(1520);
  DesireSpeed_Write(-100);

  while(1){
    dist = get_distance(4);
    if(dist < 10){
      DesireSpeed_Write(0);
      break;
    }

  }
  Alarm_Write(ON);
  usleep(300000);
  Alarm_Write(OFF);
}
void left_rotate(){
  if(check_yellow_line){

    DesireSpeed_Write(-50);

    if(ab[0] < -0.11) ab[0] = -0.11;
    else if(ab[0] > 0.11) ab[0] = 0.11;

    temp = data_transformF(ab[0], -0.11, 0.11,  1500000.0, 400000.0);
    usleep((int) temp);
    DesireSpeed_Write(50);

    steer = data_transformF(ab[0], -0.11, 0.11, 500.0, 400.0);

    SteeringServoControl_Write((int) steer + 1500);

    usleep(2000000);


    while(1){
      dist = get_distance(3);

      if(return_flag == 0 && dist < 40) return_flag = 1;
      else if(return_flag == 1 && dist > 50){
         break;
      }
    }
    traffic_light_flag = 3;
    SteeringServoControl_Write(1520);
  }
}

void right_rotate(){
  if(check_yellow_line){

    DesireSpeed_Write(-50);

    if(ab[0] < -0.11)  ab[0] = -0.11;
    else if(ab[0] > 0.11) ab[0] = 0.11;

    temp = data_transformF(ab[0], -0.11, 0.11,  400000.0, 1500000.0);

    usleep((int) temp);
    DesireSpeed_Write(50);

    steer = data_transformF(ab[0], -0.11, 0.11, 400.0, 500.0);

    SteeringServoControl_Write( 1500 - (int)steer);

    usleep(3000000);


    while(1){
      dist = get_distance(5);

      if(return_flag == 0 && dist < 40) return_flag = 1;
      else if(return_flag == 1 && dist > 50){
         break;
      }
    }
    traffic_light_flag = 3;
    SteeringServoControl_Write(1520);
  }
}
void mode_traffic_light(){
  if(traffic_light_flag < 1){
  }
  else if(traffic_light_flag == 1){
    speed = 50;
    DesireSpeed_Write(speed);
    left_rotate();
  }
  else if(traffic_light_flag == 2){
    speed = 50;
    DesireSpeed_Write(speed);
    right_rotate();
  }
  else if(traffic_light_flag == 3){
    if(is_stop()){
      printf("LineSensor_Read() = STOP! \n");
      traffic_light_flag = 4;

      SteeringServoControl_Write(1520);
      finish_flag = 1;
      mode = -1;
    }
  }
}

