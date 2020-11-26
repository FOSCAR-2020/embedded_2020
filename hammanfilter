#include <stdio.h>
#include <random>
#define PREMEM 1
#define INIT_VALUE 50
#define SENSOR_NUM 7
typedef struct f{
     int distanceFillter[SENSOR_NUM];//i는 1번부터 6번까지 초음파필터
     
}FillteredDistanceMem;
void init_FillteredDistanceMem(FillteredDistanceMem* PointDist){
    for(int i=0;i<SENSOR_NUM;i++){
            PointDist->distanceFillter[i]=INIT_VALUE;
    }
}
int get_distance(int n){
    
     std::random_device rd;
     std::mt19937 gen(rd());
     std::uniform_int_distribution<int> dis(0, 100);
    
    return dis(gen);
}
int get_filltered_distance(FillteredDistanceMem* PointDist,int n){
    double sum=(0.9)*get_distance(n)+(0.1)*PointDist->distanceFillter[n];
    PointDist->distanceFillter[n]=(int)sum;
    return  (int)sum;
}
void fillter_fill(FillteredDistanceMem* PointDist ){
    for(int i=1;i<SENSOR_NUM;i++){
        PointDist->distanceFillter[i]=get_distance(i);
    }
}
int main()
{
   FillteredDistanceMem dist;
   init_FillteredDistanceMem(&dist);
   fillter_fill(&dist);
    for(int i=0;i<100;i++){
        for(int j=1;j<SENSOR_NUM;j++)
            printf("sensor num%d:%d \n",j,get_filltered_distance(&dist,j));
        
    }

    return 0;
}
