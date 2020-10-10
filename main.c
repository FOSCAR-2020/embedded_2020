#include "function.h"
#include "struct.h"

volatile int mode = 1;

volatile bool cameraOnOff = 0;
volatile bool driveOnOff = 0;


unsigned char status;
short speed;
unsigned char gain;
int position, posInit, posDes, posRead;
short cameraY;
int channel;

int data;
char sensor;
int i, j;
char byte = 0x80;

volatile float four_point[16] = {};
volatile float ab[2] = {};

volatile int angle = 1500;
volatile int temp_angle;


volatile int start_flag = 0;
volatile int overpass_flag = 0;
volatile int outbreak_flag = 0;

volatile int parking_flag = 0;

volatile int rotary_flag = 0;

volatile int tunnel_flag = 0;

volatile int passing_lane_flag = 0;

volatile int traffic_light_flag = 0;
volatile int finish_flag = 0;

volatile float slope[2] = {};
volatile int centerP[6] = {};



int data, volt, dist, interval;
int data_left, data_right, volt_left, volt_right, dist_left, dist_right;


bool start_condition();
void mode_start();

bool overpass_finish();
void mode_overpass();


volatile int outbreak_count = 0; //돌발 표지판 이진화 픽셀값 Count

const int max_outbreak_thrshold = 3000;
const int min_outbreak_thrshold = 500;


bool parking_finish = 0;
int backward_right_flag = 0;
int cntDist = 0, cntDist2 = 0;
int wall = 0;

volatile int curve_count = 0;

bool is_parking_area();

bool parking_start();
void mode_parking();


volatile int rotary_enter_count = 0;
volatile int stop_line_count = 0;
volatile int stop_line_flag = 0;
volatile int control_Speed_flag = 0;

int direction;


const int max_rotary_threshold = 1000;
const int min_rotary_threshold = 40;

volatile int rotary_ready_flag = 0;
volatile int rotary_finish_flag = 0;


const int max_stop_line_threshold = 4000;
const int min_stop_line_threshold = 30;

bool rotary_finish();
void checkAnotherCar();
void mode_rotary();

const int start_tunnel_threshold = 30;
const int finish_tunnel_threshold = 150;

void tunnel_run();
void mode_tunnel();

volatile bool is_return = false;

int is_passing_lane();
void go_left();
void go_right();
void return_from_left_lane();
void return_from_right_lane();
void mode_passing_lane();

volatile bool check_yellow_line = false;
volatile int return_flag = 0;
float temp = 0;
float steer = 0;
typedef enum {
    DUMP_NONE,
    DUMP_CMD,
    DUMP_READY,
    DUMP_WRITE_TO_FILE,
    DUMP_DONE
}DumpState;

typedef struct _DumpMsg{
    long type;
    int  state_msg;
}DumpMsg;

struct thr_data {
    struct display *disp;
    struct v4l2 *v4l2;
    struct vpe *vpe;
    struct buffer **input_bufs;

    DumpState dump_state;
    unsigned char dump_img_data[VPE_OUTPUT_IMG_SIZE];

    int msgq_id;
    bool bfull_screen;
    bool bstream_start;
    pthread_t threads[3];
};

static int allocate_input_buffers(struct thr_data *data)
{
    int i;
    struct vpe *vpe = data->vpe;

    data->input_bufs = calloc(NUMBUF, sizeof(*data->input_bufs));
    for(i = 0; i < NUMBUF; i++) {
        data->input_bufs[i] = alloc_buffer(vpe->disp, vpe->src.fourcc, vpe->src.width, vpe->src.height, false);
    }
    if (!data->input_bufs)
        ERROR("allocating shared buffer failed\n");

    for (i = 0; i < NUMBUF; i++) {
        /** Get DMABUF fd for corresponding buffer object */
        vpe->input_buf_dmafd[i] = omap_bo_dmabuf(data->input_bufs[i]->bo[0]);
        data->input_bufs[i]->fd[0] = vpe->input_buf_dmafd[i];
    }
    return 0;
}

static void free_input_buffers(struct buffer **buffer, uint32_t n, bool bmultiplanar)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        if (buffer[i]) {
            close(buffer[i]->fd[0]);
            omap_bo_del(buffer[i]->bo[0]);
            if(bmultiplanar){
                close(buffer[i]->fd[1]);
                omap_bo_del(buffer[i]->bo[1]);
            }
        }
    }
    free(buffer);
}

static void draw_operatingtime(struct display *disp, uint32_t time)
{
    FrameBuffer tmpFrame;
    unsigned char* pbuf[4];
    char strtime[128];

    memset(strtime, 0, sizeof(strtime));

    sprintf(strtime, "%03d(ms)", time);

    if(get_framebuf(disp->overlay_p_bo, pbuf) == 0) {
        tmpFrame.buf = pbuf[0];
        tmpFrame.format = draw_get_pixel_foramt(disp->overlay_p_bo->fourcc);//FORMAT_RGB888; //alloc_overlay_plane() -- FOURCC('R','G','2','4');
        tmpFrame.stride = disp->overlay_p_bo->pitches[0];//tmpFrame.width*3;

        drawString(&tmpFrame, strtime, TIME_TEXT_X, TIME_TEXT_Y, 0, TIME_TEXT_COLOR);
    }
}


static void suddenMission(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        outbreak_count = outbreak(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}


static void drive(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);
        if(mode == 4 && curve_count < 3){
          temp_angle = line_detector(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, slope, 8);

        }
        else if(mode == 7 && passing_lane_flag == 0){
          temp_angle = line_detector(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, slope, 4);
        }
        else if(mode == 7 && passing_lane_flag == 4 || passing_lane_flag == 5){
          temp_angle = line_detector(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, slope, 5);
        }
        else if(mode == 7 && passing_lane_flag == 7){
          temp_angle = line_detector(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, slope, 4);
          if(temp_angle == 1520){
            temp_angle = 1700;
          }
        }
        else if(mode == 7 && passing_lane_flag == 6){
          temp_angle = line_detector(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, slope, 3);
          if(temp_angle == 1520){
            temp_angle = 1300;
          }
        }
        else{
          temp_angle = line_detector(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, slope, 1);
          if(mode == 5 && rotary_flag == 0 && stop_line_flag == 1 && temp_angle >= 1520){
            temp_angle = 1300;
          }
        }

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}

static void rotary_enter(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        rotary_enter_count = enter_the_rotary(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}

static void passing_lane_decision(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        passing_lane_flag = passing_lane_check(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, four_point, ab);

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}

static void is_yellow_line(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);


        if(mode == 7){
          is_return = is_yellow_horizental(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, 1, ab);
        }
        else{
          check_yellow_line = is_yellow_horizental(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, 2, ab);
        }


        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}

static void is_stop_line(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        stop_line_count = stop_line_detector(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}

static void trafficLightMission(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        traffic_light_flag = traffic_light(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, centerP);

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}



void * capture_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    struct v4l2 *v4l2 = data->v4l2;
    struct vpe *vpe = data->vpe;
    struct buffer *capt;
    bool isFirst = true;
    int index;
    int count = 0;
    int i;
    v4l2_reqbufs(v4l2, NUMBUF);

    // init vpe input
    vpe_input_init(vpe);

    // allocate vpe input buffer
    allocate_input_buffers(data);

    if(vpe->dst.coplanar)
        vpe->disp->multiplanar = true;
    else
        vpe->disp->multiplanar = false;
    printf("disp multiplanar:%d \n", vpe->disp->multiplanar);
    vpe_output_init(vpe);
    vpe_output_fullscreen(vpe, data->bfull_screen);

    for (i = 0; i < NUMBUF; i++)
        v4l2_qbuf(v4l2,vpe->input_buf_dmafd[i], i);

    for (i = 0; i < NUMBUF; i++)
        vpe_output_qbuf(vpe, i);

    v4l2_streamon(v4l2);
    vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

    vpe->field = V4L2_FIELD_ANY;

    while(1) {
        if(cameraOnOff == 0)
          continue;

        index = v4l2_dqbuf(v4l2, &vpe->field);
        vpe_input_qbuf(vpe, index);

        if (isFirst) {
            vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
            isFirst = false;
            MSG("streaming started...");
            data->bstream_start = true;
        }

        index = vpe_output_dqbuf(vpe);
        capt = vpe->disp_bufs[index];

        switch(mode) {
          case -1 :
            driveOnOff = 0;
          case 1 :  // 출발 및 도로주행
            break;
          case 2 :  // 고가도로 구간
            break;
          case 3 :  // 우선정지 장애물
            suddenMission(vpe->disp, capt);
            driveOnOff = 1;
            break;
          case 4 :  // 주차
            driveOnOff = 1;
            if(parking_flag == 3 || parking_flag == 4) driveOnOff = 0;

            break;
          case 5 :  // 회전 교차로
            driveOnOff = 1;
            if(rotary_flag == 0 && stop_line_flag == 0){
              is_stop_line(vpe->disp, capt);
            }
            else if(rotary_flag == 1) {   // 정지선 인식 후 다른 차 대기하는 경우
              driveOnOff = 0;
              rotary_enter(vpe->disp, capt);
              break;
            }
            break;
          case 6 :  // 터널
            driveOnOff = 1;
            break;
          case 7 :  // 차로 추월
            driveOnOff = 1;
            cameraOnOff = 1;

            if(passing_lane_flag == 0){
              rotary_enter(vpe->disp, capt);
            }
            else if(passing_lane_flag == 1){
              driveOnOff = 0;
              passing_lane_decision(vpe->disp, capt);
            }
            else if(passing_lane_flag == 4 || passing_lane_flag == 5){
              is_yellow_line(vpe->disp, capt);
              driveOnOff = 1;
            }
          break;
          case 8 : // 신호등
            driveOnOff = 0;
            if(traffic_light_flag < 1){
              trafficLightMission(vpe->disp, capt);
            }
            else if(traffic_light_flag == 1 || traffic_light_flag == 2 && !check_yellow_line){
              is_yellow_line(vpe->disp, capt);
            }
            break;
        }
        if(driveOnOff)
          drive(vpe->disp, capt);

        if (disp_post_vid_buffer(vpe->disp, capt, 0, 0, vpe->dst.width, vpe->dst.height)) {
            ERROR("Post buffer failed");
            return NULL;
        }
        update_overlay_disp(vpe->disp);

        if(data->dump_state == DUMP_READY) {
            DumpMsg dumpmsg;
            unsigned char* pbuf[4];

            if(get_framebuf(capt, pbuf) == 0) {
                switch(capt->fourcc) {
                    case FOURCC('Y','U','Y','V'):
                    case FOURCC('B','G','R','3'):
                        memcpy(data->dump_img_data, pbuf[0], VPE_OUTPUT_IMG_SIZE);
                        break;
                    case FOURCC('N','V','1','2'):
                        memcpy(data->dump_img_data, pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H); // y data
                        memcpy(data->dump_img_data+VPE_OUTPUT_W*VPE_OUTPUT_H, pbuf[1], VPE_OUTPUT_W*VPE_OUTPUT_H/2); // uv data
                        break;
                    default :
                        MSG("DUMP.. not yet support format : %.4s\n", (char*)&capt->fourcc);
                        break;
                }
            } else {
                MSG("dump capture buf fail !");
            }

            dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
            dumpmsg.state_msg = DUMP_WRITE_TO_FILE;
            data->dump_state = DUMP_WRITE_TO_FILE;
            if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), 0)) {
                MSG("state:%d, msg send fail\n", dumpmsg.state_msg);
            }
        }

        vpe_output_qbuf(vpe, index);
        index = vpe_input_dqbuf(vpe);
        v4l2_qbuf(v4l2, vpe->input_buf_dmafd[index], index);

    }

    MSG("Ok!");
    return NULL;
}

static struct thr_data* pexam_data = NULL;


void signal_handler(int sig)
{
    if(sig == SIGINT) {
        pthread_cancel(pexam_data->threads[0]);
        pthread_cancel(pexam_data->threads[1]);
        pthread_cancel(pexam_data->threads[2]);

        msgctl(pexam_data->msgq_id, IPC_RMID, 0);

        v4l2_streamoff(pexam_data->v4l2);
        vpe_stream_off(pexam_data->vpe->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
        vpe_stream_off(pexam_data->vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

        disp_free_buffers(pexam_data->vpe->disp, NUMBUF);
        free_input_buffers(pexam_data->input_bufs, NUMBUF, false);
        free_overlay_plane(pexam_data->vpe->disp);

        disp_close(pexam_data->vpe->disp);
        vpe_close(pexam_data->vpe);
        v4l2_close(pexam_data->v4l2);

        printf("-- 6_camera_opencv_disp example End --\n");
    }
}
bool start_condition()
{
  dist = get_distance(1);

  if(start_flag == 0 && dist < 10) start_flag = 1;
  else if(start_flag == 1 && dist > 20) start_flag = 2;
  else if(start_flag == 2) return true;

  return false;
}

void mode_start()
{
  if(start_condition()){
    speed = 100;
    DesireSpeed_Write(speed);
    mode = mode + 2;
  }
}
bool overpass_finish()
{
  dist_left = get_distance(6);

  if(dist_left > OVERPASS_MAX)
    return true;
  return false;
}

void mode_overpass()
{
  if(overpass_finish()) overpass_flag = 1; // 고가도로 종료 조건
  if(overpass_flag == 0)      // 고가도로 시작 전 + 주행 중
    tunnel_run();
  else if(overpass_flag == 1) {
    cameraOnOff = 1;
    DesireSpeed_Write(0);
    SteeringServoControl_Write(1520);
    usleep(500000);
    DesireSpeed_Write(100);
    mode++;
  }
}

bool is_parking_area_right()
{
  while(1) {
    dist = get_distance(3);
    wall = get_distance(5);
    if(dist < 40 && wall < 80)  {
      cntDist++;
      if(cntDist >= 1){
        cntDist = 0;
        return 1;
      }
    }
    else cntDist = 0;
    if(dist > 50){
      cntDist2++;
      if(cntDist2 >= 1){
        cntDist2 = 0;
        return 0;
      }
    }
    else cntDist2 = 0;
  }
}
bool parking_start_right()
{
  dist = get_distance(5);
  if(dist > 20){
    return 0;
  }
  return 1;
}
bool is_parking_finish_right()
{
  dist = get_distance(3);
  int dist1 = get_distance(2);
  if(dist > 70 && dist1 > 70) return true;
  return false;
}

void mode_parking(){



  if(curve_count == 0 && angle < 1150){
    curve_count++;
    return;
  }
  else if(curve_count == 1 && angle > 1850){
    curve_count++;
    return;
  }
  else if(curve_count == 2 && angle < 1150){
    curve_count++;
    CameraYServoControl_Write(1630);
    DesireSpeed_Write(80);
    CarLight_Write(ALL_OFF);
    return;
  }
  else if(curve_count < 3){
    return;
  }

  if(parking_flag == 0 && is_parking_area_right()) {
    parking_flag = 1;
    DesireSpeed_Write(80);
  }
  else if(parking_flag == 1 && !is_parking_area_right()) {
    parking_flag = 2;
  }
  else if(parking_flag == 2 && is_parking_area_right()) {
    parking_flag = 3;
  }
  else if(parking_flag == 3) {
    go_backward_right();
  }
  else if(parking_flag == 4) {
    if(!parking_start_right()) {
      horizontal_parking_right();
      return_lane_horizontal_right();
    }
    else {
      vertical_parking();
      return_lane_vertical_right();
    }

    parking_flag = 5;

    if (parking_finish){
      DesireSpeed_Write(40);
    }
    else{
      DesireSpeed_Write(80);
    }

    while(!is_parking_finish_right()) {
      driving_write_steer();
    }
    if(!parking_finish) {
      parking_finish = 1;
      parking_flag = 0;
      backward_right_flag = 0;
    }
    else {
      parking_flag = -1;
      mode++;
    }
  }
}
bool rotary_finish()
{
  if(stop_line_count > max_stop_line_threshold && rotary_finish_flag == 0){
    rotary_finish_flag = 1;
  }
  else if(stop_line_count < min_stop_line_threshold && rotary_finish_flag == 1){ //차가 지나간 후에 주행 시작
    printf("rotary_finish!!!");
    return true;
  }
  return false;


}

bool isAnotherCar()
{
  dist = get_distance(4);

  if(dist <= MIN_DIST) {
    return true;
  }

  else if(dist >= MAX_DIST) {
    return false;
  }
}

clock_t start1, end1;
volatile int check_time_flag = 0;

void mode_rotary()
{
  if(rotary_flag == 0){

    if(stop_line_flag == 0 && stop_line_count > 300){

      stop_line_flag = 1;
      return;
    }


    if(is_stop()) {
      printf("LineSensor_Read() = STOP! \n");
      speed = 0;
      DesireSpeed_Write(speed);
      rotary_flag = 1;
    }
  }
  else if(rotary_flag == 1){
    if(rotary_enter_count > max_rotary_threshold && rotary_ready_flag == 0) rotary_ready_flag = 1;  //차가 완전히 지나가기 전
    else if(rotary_enter_count < min_rotary_threshold && rotary_ready_flag == 1){   //차가 지나간 후에 주행 시작
      start1 = clock();
      CameraYServoControl_Write(1680);
      rotary_flag = 2;
      speed = 40;
      DesireSpeed_Write(speed);
    }

  }
  else if(rotary_flag == 2){

    if(check_time_flag == 0){

      end1 = clock();
      float res1 = (float) (end1 - start1) / CLOCKS_PER_SEC;

      if(res1 > 6.0){
        DesireSpeed_Write(0);
        check_time_flag = 1;
      }
    }
    if(isAnotherCar()) {
      speed = 160;
      DesireSpeed_Write(speed);
      rotary_flag = 3;
    }
  }
  else if(rotary_flag == 3) {
    if(!isAnotherCar()) {
      mode++;
      rotary_flag = -1;
      stop_line_flag = 0;
    }
  }
}
void mode_tunnel()
{

  dist_left = get_distance(6);
  int dist_back_left = get_distance(5);
  if(control_Speed_flag == 0 && temp_angle > 1950){
    control_Speed_flag = 1;
    return;
  }
  else if(control_Speed_flag == 1 && temp_angle > 1450 && temp_angle < 1550){
    DesireSpeed_Write(160);
    control_Speed_flag = 2;
    CarLight_Write(0x01);
    CameraYServoControl_Write(1680);

  }
  else if((tunnel_flag == 0) && (dist_left < start_tunnel_threshold) && dist_back_left < start_tunnel_threshold) { // 터널 시작
    tunnel_flag = 1;
  speed = 160;
    DesireSpeed_Write(speed);
    printf("tunnel_start!!!!\n");


  }

  else if((tunnel_flag == 1) && (dist_left > finish_tunnel_threshold) && (dist_back_left > finish_tunnel_threshold)) { // 터널 탈출
    tunnel_flag = 2;
    printf("tunnel_end!!!\n");
    CarLight_Write(ALL_OFF);

  }

  else if(tunnel_flag == 1) {
    tunnel_run();
  }

  else if(tunnel_flag == 2) {
    CarLight_Write(0x00);
    tunnel_flag = -1;
    mode++;
    DesireSpeed_Write(-100);
    usleep(800000);
    DesireSpeed_Write(50);
    CameraYServoControl_Write(1630);
  }
}
int is_passing_lane()
{

  if(rotary_enter_count >  1200){
    	DesireSpeed_Write(-40);
    	usleep(600000);
    	return 1;
    }
    return 0;
}

void go_right()
{
  Winker_Write(RIGHT_ON);

	SteeringServoControl_Write(1100);
  usleep(2000000);
	passing_lane_flag = 4;
  // DesireSpeed_Write(70);

  Winker_Write(ALL_OFF);


}

void go_left()
{
  Winker_Write(LEFT_ON);


	SteeringServoControl_Write(2000);
  usleep(2000000);
  passing_lane_flag = 5;
  Winker_Write(ALL_OFF);


}

void return_from_right_lane(){

  if(is_return){
    DesireSpeed_Write(-100);
    usleep(500000);
    DesireSpeed_Write(50);
    SteeringServoControl_Write(2000);
    usleep(2000000);
    passing_lane_flag = 6;
  }
}

void return_from_left_lane()
{
  if(is_return){
    DesireSpeed_Write(-100);
    usleep(500000);
    DesireSpeed_Write(50);
    SteeringServoControl_Write(1000);
    usleep(2000000);
    passing_lane_flag = 7;
  }
}
void mode_passing_lane()
{

	if(passing_lane_flag == 0){
	  passing_lane_flag = is_passing_lane();
	}
	else if(passing_lane_flag == 1){
    printf("%lf, %lf\n", four_point[10], four_point[11]);

	}
	else if(passing_lane_flag == 2){
    printf("%lf, %lf\n", four_point[10], four_point[11]);

		CameraYServoControl_Write(1630);
		DesireSpeed_Write(50);
		go_right();
	}
	else if(passing_lane_flag == 3){
    printf("%lf, %lf\n", four_point[10], four_point[11]);

		CameraYServoControl_Write(1630);
		DesireSpeed_Write(50);

		go_left();
	}
  else if(passing_lane_flag == 4){
    return_from_right_lane();
  }
  else if(passing_lane_flag == 5){
    return_from_left_lane();
  }
  else if (passing_lane_flag == 6 || passing_lane_flag == 7){

    if(is_stop()) {
      printf("LineSensor_Read() = STOP! \n");
      speed = 0;
      DesireSpeed_Write(speed);
      Alarm_Write(ON);
      usleep(1000000);
      Alarm_Write(OFF);
      CameraYServoControl_Write(1630);
      SteeringServoControl_Write(1520);


      mode ++;
    }
  }
}
int main(int argc, char **argv)
{
  struct v4l2 *v4l2;
  struct vpe *vpe;
  struct thr_data tdata;
  int disp_argc = 3;
  char* disp_argv[] = {"dummy", "-s", "4:480x272", "\0"}; // ���� ���� ���� Ȯ�� �� ó��..
  int ret = 0;


  tdata.dump_state = DUMP_NONE;
  memset(tdata.dump_img_data, 0, sizeof(tdata.dump_img_data));

  // open vpe
  vpe = vpe_open();
  if(!vpe) {
    return 1;
  }

  // vpe input (v4l cameradata)
  vpe->src.width  = CAPTURE_IMG_W;
  vpe->src.height = CAPTURE_IMG_H;
  describeFormat(CAPTURE_IMG_FORMAT, &vpe->src);

  // vpe output (disp data)
  vpe->dst.width  = VPE_OUTPUT_W;
  vpe->dst.height = VPE_OUTPUT_H;
  describeFormat (VPE_OUTPUT_FORMAT, &vpe->dst);

  vpe->disp = disp_open(disp_argc, disp_argv);
  if (!vpe->disp) {
    ERROR("disp open error!");
    vpe_close(vpe);
    return 1;
  }

  set_z_order(vpe->disp, vpe->disp->overlay_p.id);
  set_global_alpha(vpe->disp, vpe->disp->overlay_p.id);
  set_pre_multiplied_alpha(vpe->disp, vpe->disp->overlay_p.id);
  alloc_overlay_plane(vpe->disp, OVERLAY_DISP_FORCC, 0, 0, OVERLAY_DISP_W, OVERLAY_DISP_H);

  //vpe->deint = 0;
  vpe->translen = 1;

  MSG ("Input(Camera) = %d x %d (%.4s)\nOutput(LCD) = %d x %d (%.4s)",
  vpe->src.width, vpe->src.height, (char*)&vpe->src.fourcc,
  vpe->dst.width, vpe->dst.height, (char*)&vpe->dst.fourcc);

  if (    vpe->src.height < 0 || vpe->src.width < 0 || vpe->src.fourcc < 0 || \
    vpe->dst.height < 0 || vpe->dst.width < 0 || vpe->dst.fourcc < 0) {
    ERROR("Invalid parameters\n");
  }

  v4l2 = v4l2_open(vpe->src.fourcc, vpe->src.width, vpe->src.height);
  if (!v4l2) {
    ERROR("v4l2 open error!");
    disp_close(vpe->disp);
    vpe_close(vpe);
    return 1;
  }

  tdata.disp = vpe->disp;
  tdata.v4l2 = v4l2;
  tdata.vpe = vpe;
  tdata.bfull_screen = true;
  tdata.bstream_start = false;

  if(-1 == (tdata.msgq_id = msgget((key_t)DUMP_MSGQ_KEY, IPC_CREAT | 0666))) {
    fprintf(stderr, "%s msg create fail!!!\n", __func__);
    return -1;
  }

  pexam_data = &tdata;

  ret = pthread_create(&tdata.threads[0], NULL, capture_thread, &tdata);
  if(ret) {
    MSG("Failed creating capture thread");
  }
  pthread_detach(tdata.threads[0]);
  CarControlInit();

  cameraY = 1630;
  SteeringServoControl_Write(angle);
  CameraXServoControl_Write(1500);
  CameraYServoControl_Write(cameraY);

  SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
  speed = 0; // speed set     --> speed must be set when using position controller
  DesireSpeed_Write(speed);

  //control on/off
  status = PositionControlOnOff_Read();
  printf("PositionControlOnOff_Read() = %d\n", status);
  PositionControlOnOff_Write(CONTROL);

  //speed controller gain set(PID제어)
  //P-gain
  gain = SpeedPIDProportional_Read();        // default value = 10, range : 1~50
  printf("SpeedPIDProportional_Read() = %d \n", gain);
  gain = 20;
  SpeedPIDProportional_Write(gain);

  //I-gain
  gain = SpeedPIDIntegral_Read();        // default value = 10, range : 1~50
  printf("SpeedPIDIntegral_Read() = %d \n", gain);
  gain = 20;
  SpeedPIDIntegral_Write(gain);

  //D-gain
  gain = SpeedPIDDifferential_Read();        // default value = 10, range : 1~50
  printf("SpeedPIDDefferential_Read() = %d \n", gain);
  gain = 20;
  SpeedPIDDifferential_Write(gain);

  PositionControlOnOff_Write(UNCONTROL);
  cameraOnOff = 1;
  while(1){

    switch(mode) {
      case -1:
        break;
      case 1 :
          mode_start();
          break;
      case 2 :
          mode_overpass();
          break;
      case 3 :
          mode_outbreak();
          break;
      case 4 :
          mode_parking();
          break;
      case 5 :
          mode_rotary();
          break;
      case 6 :
          mode_tunnel();
          break;
      case 7 :
          mode_passing_lane();
          break;
      case 8 :
          mode_traffic_light();
          break;
    }

    if(driveOnOff){
      driving_write_steer();
    }

    if(finish_flag == 1){
      printf("...finish...\n");
      SteeringServoControl_Write(1520);

      usleep(1500000);
      speed = 0;
      DesireSpeed_Write(speed);
      Alarm_Write(ON);
      usleep(1000000);
      Alarm_Write(OFF);
      break;
    }


  }
/* register signal handler for <CTRL>+C in order to clean up */
  if(signal(SIGINT, signal_handler) == SIG_ERR) {
    MSG("could not register signal handler");
    closelog();
    exit(EXIT_FAILURE);
  }

  pause();

  return ret;
}
