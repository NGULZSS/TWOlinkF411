#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "m_servo.h"
#include <STM32FreeRTOS.h>
#include "OneButton.h"
#include "TwoLinkKinematic.h"
#include "TFTshow.h" 
#include <Arduino.h>
#define TFT_GREY 0xBDF7
m_servo mservo(0); //引入舵机库
TFTshow TFTSHOWS;
HardwareSerial Serial2X(PA3, PA2);
int idlist[20]={0,1,2,4};           //使用set_angles()时需要初始化的ID数组
float anglelist[20]={20,10,230,10}; //使用set_angles()时需要初始化的ID数组
float ang;                          //调用get_state时 存放返回的角度值
int p_set;
extern double servo_rpara[10];
bool RunFlag=false;
float Aimq[2]={200.0,0};
TLKinematic TL;
#pragma region 线程创建
TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

TaskHandle_t ShowThread_Handler;
#define ShowThread_PRIO		3
void ShowThread(void *pvParameters);

TaskHandle_t ConturlThread_Handler;
#define ConturlThread_PRIO		2
void ConturlThread(void *pvParameters);

TaskHandle_t CommunicationThread_Handler;
#define CommunicationThread_PRIO		3
void CommunicationThread(void *pvParameters);
char InfoBuffer[1000];//信息存取
QueueHandle_t Speedqueue;
QueueHandle_t Positionqueue;
#pragma endregion

#pragma region 按键定义并初始化
#define KEY1 PC13 //按键1
#define KEY2 PC14 //按键2
#define KEY3 A4 //按键3
OneButton button1(KEY1, true);
OneButton button2(KEY2, true);
OneButton button3(KEY3, true);
void ClickRight();
void ClickLeft();
void ClickCenter();
#pragma endregion

 void setup(void) {
  pinMode(PA0,OUTPUT);
  p_set = 0;
  // tft.init();
  //       tft.setRotation(0);
  //       tft.fillScreen(TFT_BLACK);
  //       tft.setTextColor(TFT_GREEN, TFT_BLACK);
  button1.attachClick(ClickRight);
  button2.attachClick(ClickLeft);
  button3.attachClick(ClickCenter);
  TFTSHOWS.TFTINIT();
  TFTSHOWS.BasicParametersShow();
  delay(1000);
  mservo.set_angle(1,0,10);   //ID号121为广播模式
  xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )128,                   //任务堆栈大小
                (void*          )NULL,                  //传递给任务参数
                (UBaseType_t    )1,                     //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄            
    vTaskStartScheduler();          //¿ªÆôÈÎÎñµ÷¶È
 }

 void loop() {
  
  button1.tick();
  button2.tick();
  button3.tick();

 }


void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    Speedqueue = xQueueCreate( 50, sizeof( float ) );
    Positionqueue = xQueueCreate( 50, sizeof( float ) );
    //显示线程创建
    xTaskCreate((TaskFunction_t )ShowThread,             
                (const char*    )"ShowThread",           
                (uint16_t       )128,        
                (void*          )NULL,                  
                (UBaseType_t    )ShowThread_PRIO,        
                (TaskHandle_t*  )&ShowThread_Handler);   
    //控制线程创建
    xTaskCreate((TaskFunction_t )ConturlThread,     
                (const char*    )"ConturlThread",   
                (uint16_t       )128,
                (void*          )NULL,
                (UBaseType_t    )ConturlThread_PRIO,
                (TaskHandle_t*  )&ConturlThread_Handler); 
     //通信线程创建
    xTaskCreate((TaskFunction_t )CommunicationThread,     
                (const char*    )"CommunicationThread",   
                (uint16_t       )128,
                (void*          )NULL,
                (UBaseType_t    )CommunicationThread_PRIO,
                (TaskHandle_t*  )&CommunicationThread_Handler); 
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

//显示线程
void ShowThread(void *pvParameters)
{
  float getspeed;
  float getposition;
  
	while(1)
	{
      mservo.get_state(1,2,1);
      TFTSHOWS.EndTrackShow(mservo.Para.servo_rpara);  
        vTaskDelay(10);			//延时500ms
	}
}
//控制线程
void ConturlThread(void *pvParameters)
{
  
	while(1)
	{
     if(RunFlag==true)
     {
       TL.MoveJ(Aimq,130,400,mservo);
       //TL.MoveL(Aimq,180,400);
     }
     
     vTaskDelay(15);			//延时500ms

     //taskYIELD();任务切换函数
	}
}
//通信线程
void CommunicationThread(void *pvParameters)
{
  float Aimq[2]={100.0,0};
	while(1)
	{
   // HAL_UART_Transmit(&huart1,(char*)"WED",5,50);
    digitalWrite(PA0,LOW);
    vTaskDelay(200);	
    digitalWrite(PA0,HIGH);
    vTaskDelay(200);	
	}
	}

void ClickRight()
{
    //mservo.set_angle(1,90,100);
    RunFlag=true;
    digitalWrite(PA0,LOW);
    
}
void ClickLeft()
{
  static int as=10;
  as+=10;
  Aimq[0]+=20;
  Aimq[1]+=40;
   // mservo.set_angle(1,20+as,1); 
}
void ClickCenter()
{

  mservo.set_angle(1,20,1); 
}
