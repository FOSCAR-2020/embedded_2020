#include "function.h"
extern int data;
extern char sensor;
extern int i, j;
extern char byte;
extern int angle;
extern int temp_angle;

bool is_stop()
{
  int flag = 0;
  sensor = LineSensor_Read();

  for(i = 0; i < 8; i++){
    if((i % 4) == 0) printf(" ");
    if((sensor & byte)) printf("1");
    else{
      printf("0");
      if(i != 0) flag++;
    }
    sensor = sensor << 1;
  }

  if(flag >= 3){
    printf("LineSensor_Read() = STOP! \n");
    return true;
  }
  return false;
}
void driving_write_steer()
{
  if(temp_angle != angle){
    angle = temp_angle;
    SteeringServoControl_Write(angle);
  }
}
int data_transform(int x, int in_min, int in_max, int out_min, int out_max) // 적외선 센서 데이터 변환 함수
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float data_transformF(float x, float in_min, float in_max, float out_min, float out_max) // 적외선 센서 데이터 변환 함수
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
