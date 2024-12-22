#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <ESP32Encoder.h>
#include "pin_map.hpp"
// #include "motor_driver.hpp"
#include "motor_driver_mcpwm.hpp"
#include <agrobot_interfaces/msg/encoder_pulses.h>
#include <agrobot_interfaces/msg/motor_pw_ms.h>

// Macros
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

// Error handle loop
void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

// . Micro-ROS entities
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// . Publisher Encoder Pulses
rcl_publisher_t publisher_enc;
agrobot_interfaces__msg__EncoderPulses msg_encoder_pulses;
rclc_executor_t executor_pub;
rcl_timer_t timer;

//. Subscriber Motor PWM Values
rcl_subscription_t subscriber_pwm;
agrobot_interfaces__msg__MotorPWMs msg_motor_pwm;
rclc_executor_t executor_sub;

bool micro_ros_init_successful;

// State machine states
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

// MotorDriver motor1(MotorDriver1_DIR2, MotorDriver1_PWM2, 255, 25, 0, &encoder1,0);
// MotorDriver motor2(MotorDriver1_DIR1, MotorDriver1_PWM1, 255, 25, 1, &encoder2,0);
// MotorDriver motor3(MotorDriver2_DIR2, MotorDriver2_PWM2, 255, 25, 2, &encoder3,-20);
// MotorDriver motor4(MotorDriver2_DIR1, MotorDriver2_PWM1, 255, 25, 3, &encoder4,0);

void encoder_pulses_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {

    // Store the encoder pulses in the message
    msg_encoder_pulses.encoder_1_pulse = encoder1.getCount();
    msg_encoder_pulses.encoder_2_pulse = encoder2.getCount();
    msg_encoder_pulses.encoder_3_pulse = encoder3.getCount();
    msg_encoder_pulses.encoder_4_pulse = encoder4.getCount();

    // Publish the encoder pulses
    RCSOFTCHECK(rcl_publish(&publisher_enc, &msg_encoder_pulses, NULL));
  }
}

void motor_pwm_callback(const void *msgin)
{
  const agrobot_interfaces__msg__MotorPWMs *msg_motor_pwm = (const agrobot_interfaces__msg__MotorPWMs *)msgin;

  // motor1.runMotor(msg_motor_pwm->motor1pwm);
  // motor2.runMotor(msg_motor_pwm->motor2pwm);
  // motor3.runMotor(msg_motor_pwm->motor3pwm);
  // motor4.runMotor(msg_motor_pwm->motor4pwm);

  motorSetSpeed(1, msg_motor_pwm->motor1pwm);
  motorSetSpeed(2, msg_motor_pwm->motor2pwm);
  motorSetSpeed(3, msg_motor_pwm->motor3pwm);
  motorSetSpeed(4, msg_motor_pwm->motor4pwm);
  
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // Initialize and modify options ( Set DOMAIN_ID to 20)
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 20);

  // Create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // Create Node
  RCCHECK(rclc_node_init_default(&node, "agrobot_base_esp", "", &support));

  // Create Encoder Pulses Publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher_enc,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, EncoderPulses),
      "encoder_pulses"));

  // Create Motor PWM Subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber_pwm,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, MotorPWMs),
      "motor_pwm"));

  // Create Timer for Encoder Pulses
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      encoder_pulses_callback));

  // Create Executor
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_pwm, &msg_motor_pwm, motor_pwm_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{

  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher_enc, &node);
  rcl_timer_fini(&timer);
  rcl_subscription_fini(&subscriber_pwm, &node);
  rclc_executor_fini(&executor_pub);
  rclc_executor_fini(&executor_sub);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup_hardware()
{
  encoder1.attachFullQuad(ENC1_A, ENC1_B);
  encoder1.setCount(0);

  encoder2.attachFullQuad(ENC2_A, ENC2_B);
  encoder2.setCount(0);

  encoder3.attachFullQuad(ENC3_A, ENC3_B);
  encoder3.setCount(0);

  encoder4.attachFullQuad(ENC4_A, ENC4_B);
  encoder4.setCount(0);

  setupMCPWM();

  // motor1.setup();
  // motor2.setup();
  // motor3.setup();
  // motor4.setup();
}

void setup()
{

  // Configure WiFi transport

  IPAddress agent_ip(10, 42, 0, 1);
  size_t agent_port = 8888;

  char ssid[] = "adyansh_sakar";
  char psk[]= "12345678";

  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  // Serial.begin(115200);
  // set_microros_serial_transports(Serial);

  // Initialize the hardware
  setup_hardware();

  state = WAITING_AGENT;
}

void loop()
{
  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(100, state = (RMW_RET_OK == rmw_uros_ping_agent(1000, 3)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;

  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      destroy_entities();
    };
    break;

  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(1000, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(10));
      rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(10));
    }
    break;


  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;

  default:
    break;
  }
}

