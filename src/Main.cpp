#include <Arduino.h>
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <STM32FreeRTOS.h>
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include "motors.h"
#include "hardware_cfg.h"
#include <SPI.h>
// ROS MSGS TYPES
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int64.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

//IMU
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
//PIXEL
#include "pixel.h"

/* DEFINES */
#define CLIENT_IP "192.168.1.177"
#define AGENT_IP "192.168.1.176"
#define AGENT_PORT 8888
#define NODE_NAME "stm32_node"


#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
      Serial.printf("o");          \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      Serial.printf("o");          \
    }                              \
  }

/* VARIABLES */

//ROS PUBLISHERS
rcl_publisher_t publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
//ROS SUBSCRIPTIONS
rcl_subscription_t subscriber;
rcl_subscription_t cmd_vel_subscriber;
//ROS MESSAGES
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist* twist_msg;
geometry_msgs__msg__Twist twist_msg_temp;
std_msgs__msg__String msgs;
uint8_t ros_msgs_cnt = 0;

//ROS
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//ETHERNET
IPAddress client_ip;
IPAddress agent_ip;
byte mac[] = {0x02, 0x47, 0x00, 0x00, 0x00, 0x01};


//MOTORS
MotorClass Motor1(M1_PWM_PIN, M1_PWM_TIM, M1_PWM_TIM_CH, M1_ILIM, M1A_IN, M1B_IN, M1_ENC_TIM, M1_ENC_A, M1_ENC_B, M1_DEFAULT_DIR);
MotorClass Motor2(M2_PWM_PIN, M2_PWM_TIM, M2_PWM_TIM_CH, M2_ILIM, M2A_IN, M2B_IN, M2_ENC_TIM, M2_ENC_A, M2_ENC_B, M2_DEFAULT_DIR);
MotorClass Motor3(M3_PWM_PIN, M3_PWM_TIM, M3_PWM_TIM_CH, M3_ILIM, M3A_IN, M3B_IN, M3_ENC_TIM, M3_ENC_A, M3_ENC_B, M3_DEFAULT_DIR);
MotorClass Motor4(M4_PWM_PIN, M4_PWM_TIM, M4_PWM_TIM_CH, M4_ILIM, M4A_IN, M4B_IN, M4_ENC_TIM, M4_ENC_A, M4_ENC_B, M4_DEFAULT_DIR);
MotorPidClass M1_PID(&Motor1);
MotorPidClass M2_PID(&Motor2);
MotorPidClass M3_PID(&Motor3);
MotorPidClass M4_PID(&Motor4);

//IMU

// double xPos = 0, yPos = 0, headingVel = 0;
// uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
// uint16_t PRINT_DELAY_MS = 500; // how often to print the data
// uint16_t printCount = 0; //counter to avoid printing every 10MS sample
// double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
// double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//PIXEL LED
SPIClass PixelSpi(PIXEL_MOSI, PB14, PIXEL_SCK);
PixelLedClass PixelStrip(PIXEL_LENGTH, &PixelSpi);
PixelStripSubsetClass RL_Light(&PixelStrip, 0, 4);
PixelStripSubsetClass FL_Light(&PixelStrip, 5, 8);
PixelStripSubsetClass FR_Light(&PixelStrip, 9, 12);
PixelStripSubsetClass RR_Light(&PixelStrip, 13, 17);
/* TASKS DECLARATION */

static void rclc_spin_task(void *p);
static void chatter_publisher_task(void *p);
static void runtime_stats_task(void *p);
static void m1_pid_handler_task(void *p);
static void m2_pid_handler_task(void *p);
static void m3_pid_handler_task(void *p);
static void m4_pid_handler_task(void *p);
static void setpoint_task(void *p);
static void pixel_led_task(void *p);

/* FUNCTIONS */

void error_loop() {
  while (1) {
    // Serial.printf("in error loop");
    digitalWrite(RD_LED, !digitalRead(RD_LED));
    delay(100);
  }
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__String *msgs = (const std_msgs__msg__String *)msgin;
  // Serial.printf("[%s]: I heard: [%s]\r\n", NODE_NAME,
  //               micro_ros_string_utilities_get_c_str(msgs->data));
}

void cmd_vel_callback(const void *msgin){
  twist_msg = (geometry_msgs__msg__Twist *)msgin;
  twist_msg_temp = *twist_msg;
  //Serial.println(uint32_t(twist_msg->linear.x));
}

char buffer[200];

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time); 
  //static std_msgs__msg__Int64 msg_int64;
  if (timer != NULL) {
    static int cnt = 0;
    int enc_mot1 = Motor1.EncValUpdate();
    int enc_mot3 = Motor3.EncValUpdate();
    sprintf(buffer, "M1 velocity: %d, M2 Velocity: %d, M3 Velocity: %d, M4 Velocity: %d", 
            M1_PID.Motor->GetVelocity(), M2_PID.Motor->GetVelocity(), M3_PID.Motor->GetVelocity(), M4_PID.Motor->GetVelocity());
    //sprintf(buffer, "Hello World: %d, sys_clk: %d", cnt++, xTaskGetTickCount());
    //Serial.printf("Publishing: %s\r\n", buffer);
    msgs.data = micro_ros_string_utilities_set(msgs.data, buffer);
    RCSOFTCHECK(rcl_publish(&publisher, &msgs, NULL));
    //msg_int64.data = 1;
    // RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
  }
}

/*==================== SETUP ========================*/

void setup() {
  portBASE_TYPE s1, s2, s3, s4, s5, s6, s7, s8;
  // Open serial communications and wait for port to open:
  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(460800);
  //Motors init
  M1_PID.SetSetpoint(0);
  M2_PID.SetSetpoint(0);
  M3_PID.SetSetpoint(0);
  M4_PID.SetSetpoint(0);
  //Pixel Led
  PixelStrip.SetStripColour(0xFF, 0x00, 0x00, 0x0F);
  PixelStrip.PixelStripMapSwap(13,17);
  PixelStrip.PixelStripMapSwap(14,16);
  //Hardware init
  pinMode(GRN_LED, OUTPUT);
  digitalWrite(GRN_LED, HIGH);
  pinMode(EN_LOC_5V, OUTPUT);
  digitalWrite(EN_LOC_5V, HIGH);
  pinMode(RD_LED, OUTPUT);
  digitalWrite(RD_LED, LOW);

  client_ip.fromString(CLIENT_IP);
  agent_ip.fromString(AGENT_IP);

  Serial.printf("Connecting to agent: \r\n");
  Serial.println(agent_ip);

  set_microros_native_ethernet_udp_transports(mac, client_ip, agent_ip,
                                              AGENT_PORT);

  delay(2000);
  pinMode(M2A_IN, OUTPUT_OPEN_DRAIN); // temporary after ethernet init
  PixelStrip.Init();


  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "chatter"));
  ros_msgs_cnt++;
  Serial.printf("Created 'chatter' publisher.\r\n");

  RCCHECK(rclc_publisher_init_default(
      &odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom/wheels"));
  ros_msgs_cnt++;
  Serial.printf("Created 'odom/wheels' publisher.\r\n");

  RCCHECK(rclc_publisher_init_default(
      &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "imu"));
  ros_msgs_cnt++;    
  Serial.printf("Created 'imu' publisher.\r\n");

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom/wheels"));
  ros_msgs_cnt++;
  Serial.printf("Created 'odom/wheels' subscriber\r\n");

  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));
  ros_msgs_cnt++;
  Serial.printf("Created 'cmd_vel' subscriber\r\n");
      

  // create timer,
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50),
                                  timer_callback));
  Serial.printf("Created timer\r\n");

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, ros_msgs_cnt, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msgs,
                                         &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &msgs,
                                        &cmd_vel_callback, ON_NEW_DATA));                                    
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // create spin task
  s1 = xTaskCreate(rclc_spin_task, "rclc_spin_task",
                   configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
                   NULL);

  // create spin task
  s2 = xTaskCreate(runtime_stats_task, "runtime_stats_task",
                   configMINIMAL_STACK_SIZE + 2000, NULL, tskIDLE_PRIORITY + 1,
                   NULL);

  s3 = xTaskCreate(m1_pid_handler_task, "m1_pid_handler_task",
                            configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
                            NULL);

  s4 = xTaskCreate(m2_pid_handler_task, "m2_pid_handler_task",
                            configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
                            NULL);

  s5 = xTaskCreate(m3_pid_handler_task, "m3_pid_handler_task",
                            configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
                            NULL);

  s6 = xTaskCreate(m4_pid_handler_task, "m4_pid_handler_task",
                            configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
                            NULL);
  s7 = xTaskCreate(setpoint_task, "setpoint_task",
                            configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
                            NULL);
  s8 = xTaskCreate(pixel_led_task, "pixel_led_task",
                          configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 1,
                          NULL);

  // check for creation errors
  if (s1 != pdPASS || s2 != pdPASS || s3 != pdPASS ||
      s4 != pdPASS || s5 != pdPASS || s6 != pdPASS ||
      s7 != pdPASS || s8 != pdPASS) {
    Serial.println(F("Creation problem"));
    while (1)
      ;
  }
  // start FreeRTOS
  vTaskStartScheduler();
}

/* RTOS TASKS */

static void rclc_spin_task(void *p) {
  UNUSED(p);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    RCSOFTCHECK(rclc_executor_spin(&executor));
    // RCSOFTCHECK(rclc_executor_spin_some(&executor, 2000));
    // RCSOFTCHECK(rclc_executor_spin_some(&executor,0));
    // Serial.printf("\r\nt: %d\r\n", xTaskGetTickCount());
    vTaskDelayUntil(&xLastWakeTime, 2);
  }
}

static void runtime_stats_task(void *p) {
  char buf[2000];
  Serial.printf("runtime stats task started\r\n");
  while (1) {
    vTaskGetRunTimeStats(buf);
    //Serial.printf("\r\n%s\r\n-------------", buf);
    vTaskDelay(2000);
  }
}

static void m1_pid_handler_task(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil(&xLastWakeTime, 1000/PID_FREQ);
    M1_PID.SetSetpoint(double(twist_msg_temp.linear.x));
    M1_PID.Handler();
    odom_msg.twist.twist.angular.z = M1_PID.Motor->GetVelocity();
  }
}

static void m2_pid_handler_task(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil(&xLastWakeTime, 1000/PID_FREQ);
    M2_PID.SetSetpoint(double(twist_msg_temp.linear.x));
    M2_PID.Handler();
  }
}

static void m3_pid_handler_task(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil(&xLastWakeTime, 1000/PID_FREQ);
    M3_PID.SetSetpoint(double(twist_msg_temp.linear.x));
    M3_PID.Handler();
  }
}

static void m4_pid_handler_task(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil(&xLastWakeTime, 1000/PID_FREQ);
    M4_PID.SetSetpoint(double(twist_msg_temp.linear.x));
    M4_PID.Handler();
  }
}

static void setpoint_task(void *p){
  int16_t Setpoint1 = 6000;
  int16_t Setpoint2 = -6000;
  int16_t DelayTime = 6000;
  while(1){
    // M1_PID.SetSetpoint(-Setpoint1);
    // M2_PID.SetSetpoint(Setpoint1);
    // M3_PID.SetSetpoint(-Setpoint1);
    // M4_PID.SetSetpoint(Setpoint1);
    // vTaskDelay(DelayTime);
    // M1_PID.SetSetpoint(-Setpoint2);
    // M2_PID.SetSetpoint(Setpoint2);
    // M3_PID.SetSetpoint(-Setpoint2);
    // M4_PID.SetSetpoint(Setpoint2);
    vTaskDelay(DelayTime);
  }
}

static void pixel_led_task(void *p){
  uint16_t DelayTime = 250;
  while(1){
    vTaskDelay(DelayTime);
    RL_Light.SetColour(0xFF, 0x00, 0x00);
    FL_Light.SetColour(0xFF, 0x00, 0x00);
    FR_Light.SetColour(0x00, 0x00, 0x00);
    RR_Light.SetColour(0x00, 0x00, 0x00);
    vTaskDelay(DelayTime);
    RL_Light.SetColour(0x00, 0x00, 0xFF);
    FL_Light.SetColour(0x00, 0x00, 0xFF);
    FR_Light.SetColour(0x00, 0x00, 0x00);
    RR_Light.SetColour(0x00, 0x00, 0x00);
    vTaskDelay(DelayTime);
    RL_Light.SetColour(0xFF, 0x00, 0x00);
    FL_Light.SetColour(0xFF, 0x00, 0x00);
    FR_Light.SetColour(0x00, 0x00, 0x00);
    RR_Light.SetColour(0x00, 0x00, 0x00);
    vTaskDelay(DelayTime);
    RL_Light.SetColour(0x00, 0x00, 0xFF);
    FL_Light.SetColour(0x00, 0x00, 0xFF);
    FR_Light.SetColour(0x00, 0x00, 0x00);
    RR_Light.SetColour(0x00, 0x00, 0x00);
    vTaskDelay(DelayTime);
    FR_Light.SetColour(0xFF, 0x00, 0x00);
    RR_Light.SetColour(0xFF, 0x00, 0x00);
    RL_Light.SetColour(0x00, 0x00, 0x00);
    FL_Light.SetColour(0x00, 0x00, 0x00);
    vTaskDelay(DelayTime);
    FR_Light.SetColour(0x00, 0x00, 0xFF);
    RR_Light.SetColour(0x00, 0x00, 0xFF);
    RL_Light.SetColour(0x00, 0x00, 0x00);
    FL_Light.SetColour(0x00, 0x00, 0x00);
    vTaskDelay(DelayTime);
    FR_Light.SetColour(0xFF, 0x00, 0x00);
    RR_Light.SetColour(0xFF, 0x00, 0x00);
    RL_Light.SetColour(0x00, 0x00, 0x00);
    FL_Light.SetColour(0x00, 0x00, 0x00);
    vTaskDelay(DelayTime);
    FR_Light.SetColour(0x00, 0x00, 0xFF);
    RR_Light.SetColour(0x00, 0x00, 0xFF);
    RL_Light.SetColour(0x00, 0x00, 0x00);
    FL_Light.SetColour(0x00, 0x00, 0x00);
  }
}

/*============== LOOP - IDDLE TASK ===============*/

void loop() {
  digitalWrite(GRN_LED, !digitalRead(GRN_LED));
  delay(1000);
}

/*=========== Runtime stats ====================*/

HardwareTimer stats_tim(TIM5);  // TIM5 - 32 bit

void vConfigureTimerForRunTimeStats(void) {
  //   m1_pwm.setPWM(1, M1_PWM, 1000, power);
  stats_tim.setPrescaleFactor(
      1680);  // Set prescaler to 2564 => timer frequency = 168MHz/1680 = 100000
              // Hz (from prediv'd by 1 clocksource of 168 MHz)
  stats_tim.setOverflow(0xffffffff);  // Set overflow to 32761 => timer
                                      // frequency = 65522 Hz / 32761 = 2 Hz
  stats_tim.refresh();                // Make register changes take effect
  stats_tim.resume();                 // Start
}

uint32_t vGetTimerValueForRunTimeStats(void) { return stats_tim.getCount(); }