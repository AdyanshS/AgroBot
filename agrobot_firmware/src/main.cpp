#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <ESP32Encoder.h>
#include "pin_map.hpp"
#include "motor_driver_mcpwm.hpp"

#include <std_msgs/msg/int32.h>
#include <agrobot_interfaces/msg/encoder_pulses.h>
#include <agrobot_interfaces/msg/motor_pw_ms.h>

#define MOTOR3_ERROR 9

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

//. Publisher Lift Encoder Pulses
rcl_publisher_t publisher_lift_enc;
std_msgs__msg__Int32 msg_lift_encoder;

//. Subscriber Motor PWM Values
rcl_subscription_t subscriber_pwm;
agrobot_interfaces__msg__MotorPWMs msg_motor_pwm;
rclc_executor_t executor_sub;

//. Subscriber Lift Motor PWM Values
rcl_subscription_t subscriber_lift_pwm;
std_msgs__msg__Int32 msg_lift_pwm;

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
// ESP32Encoder encoder5;

void testMotor(int motorDutyCycle, int duration, int cycles)
{
  // Array for encoder values
  long previousEncoderTicks[4] = {0, 0, 0, 0};
  long currentEncoderTicks[4] = {0, 0, 0, 0};
  long encoderDifference[4] = {0, 0, 0, 0};

  for (int i = 0; i < cycles; i++)
  {
    // Run motor clockwise
    motorSetSpeed(1, motorDutyCycle);
    motorSetSpeed(2, motorDutyCycle);
    motorSetSpeed(3, motorDutyCycle - 5);
    motorSetSpeed(4, motorDutyCycle);
    delay(duration);

    // Get current encoder ticks
    currentEncoderTicks[0] = encoder1.getCount();
    currentEncoderTicks[1] = encoder2.getCount();
    currentEncoderTicks[2] = encoder3.getCount();
    currentEncoderTicks[3] = encoder4.getCount();

    // Calculate encoder difference
    encoderDifference[0] = currentEncoderTicks[0] - previousEncoderTicks[0];
    encoderDifference[1] = currentEncoderTicks[1] - previousEncoderTicks[1];
    encoderDifference[2] = currentEncoderTicks[2] - previousEncoderTicks[2];
    encoderDifference[3] = currentEncoderTicks[3] - previousEncoderTicks[3];

    // Print encoder ticks and difference
    Serial.print("Cycle ");
    Serial.print(i + 1);
    Serial.print(": Encoder Ticks: ");
    Serial.print(currentEncoderTicks[0]);
    Serial.print(", ");
    Serial.print(currentEncoderTicks[1]);
    Serial.print(", ");
    Serial.print(currentEncoderTicks[2]);
    Serial.print(", ");
    Serial.print(currentEncoderTicks[3]);
    Serial.println();

    Serial.print("Difference: ");
    Serial.print(encoderDifference[0]);
    Serial.print(", ");
    Serial.print(encoderDifference[1]);
    Serial.print(", ");
    Serial.print(encoderDifference[2]);
    Serial.print(", ");
    Serial.print(encoderDifference[3]);
    Serial.println();

    // Update previous encoder ticks
    previousEncoderTicks[0] = currentEncoderTicks[0];
    previousEncoderTicks[1] = currentEncoderTicks[1];
    previousEncoderTicks[2] = currentEncoderTicks[2];
    previousEncoderTicks[3] = currentEncoderTicks[3];

    // Stop motor
    motorSetSpeed(1, 0);
    motorSetSpeed(2, 0);
    motorSetSpeed(3, 0);
    motorSetSpeed(4, 0);
    delay(duration);
  }

  // Run motor counterclockwise

  for (int i = 0; i < cycles; i++)
  {
    // Run motor clockwise

    motorSetSpeed(1, -motorDutyCycle);
    motorSetSpeed(2, -motorDutyCycle);
    motorSetSpeed(3, -motorDutyCycle + 5);
    motorSetSpeed(4, -motorDutyCycle);
    delay(duration);

    // Get current encoder ticks
    currentEncoderTicks[0] = encoder1.getCount();
    currentEncoderTicks[1] = encoder2.getCount();
    currentEncoderTicks[2] = encoder3.getCount();
    currentEncoderTicks[3] = encoder4.getCount();

    // Calculate encoder difference
    encoderDifference[0] = currentEncoderTicks[0] - previousEncoderTicks[0];
    encoderDifference[1] = currentEncoderTicks[1] - previousEncoderTicks[1];
    encoderDifference[2] = currentEncoderTicks[2] - previousEncoderTicks[2];
    encoderDifference[3] = currentEncoderTicks[3] - previousEncoderTicks[3];

    // Print encoder ticks and difference
    Serial.print("Cycle ");
    Serial.print(i + 1);
    Serial.print(": Encoder Ticks: ");
    Serial.print(currentEncoderTicks[0]);
    Serial.print(", ");
    Serial.print(currentEncoderTicks[1]);
    Serial.print(", ");
    Serial.print(currentEncoderTicks[2]);
    Serial.print(", ");
    Serial.print(currentEncoderTicks[3]);
    Serial.println();

    Serial.print("Difference: ");
    Serial.print(encoderDifference[0]);
    Serial.print(", ");
    Serial.print(encoderDifference[1]);
    Serial.print(", ");
    Serial.print(encoderDifference[2]);
    Serial.print(", ");
    Serial.print(encoderDifference[3]);
    Serial.println();

    // Update previous encoder ticks
    previousEncoderTicks[0] = currentEncoderTicks[0];
    previousEncoderTicks[1] = currentEncoderTicks[1];
    previousEncoderTicks[2] = currentEncoderTicks[2];
    previousEncoderTicks[3] = currentEncoderTicks[3];

    // Stop motor
    motorSetSpeed(1, 0);
    motorSetSpeed(2, 0);
    motorSetSpeed(3, 0);
    motorSetSpeed(4, 0);
    delay(duration);
  }
}

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

    // Publish the lift encoder pulses
    // msg_lift_encoder.data = encoder5.getCount();
    // RCSOFTCHECK(rcl_publish(&publisher_lift_enc, &msg_lift_encoder, NULL));
  }
}

void motor_pwm_callback(const void *msgin)
{
  const agrobot_interfaces__msg__MotorPWMs *msg_motor_pwm = (const agrobot_interfaces__msg__MotorPWMs *)msgin;

  // Introduce a small error for motor3
  int motor3_pwm = msg_motor_pwm->motor3pwm;
  // Serial.print("Motor 3 PWM: ");
  // Serial.println(motor3_pwm);

  // if (motor3_pwm != 0)
  // {
  //   // Apply full motor3 error
  //   if (abs(motor3_pwm - MOTOR3_ERROR > 0))
  //   {
  //     if (motor3_pwm > 0)
  //     {
  //       motor3_pwm = motor3_pwm - MOTOR3_ERROR;
  //     }
  //     else
  //     {
  //       motor3_pwm = motor3_pwm + MOTOR3_ERROR;
  //     }
  //   }
  //   // apply half motor 3 error
  //   else
  //   {
  //     if (motor3_pwm > 0)
  //     {
  //       motor3_pwm = motor3_pwm - MOTOR3_ERROR / 2;
  //     }
  //     else
  //     {
  //       motor3_pwm = motor3_pwm + MOTOR3_ERROR / 2;
  //     }
  //   }
  // }
  // Serial.print("Motor 3 PWM Corrected: ");
  // Serial.println(motor3_pwm);

  motorSetSpeed(1, msg_motor_pwm->motor1pwm);
  motorSetSpeed(2, msg_motor_pwm->motor2pwm);
  motorSetSpeed(3, motor3_pwm);
  motorSetSpeed(4, msg_motor_pwm->motor4pwm);
}

void lift_motor_callback(const void *msgin)
{
  const std_msgs__msg__Int32 *msg_lift_pwm = (const std_msgs__msg__Int32 *)msgin;
  motorSetSpeed(5, msg_lift_pwm->data);
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // Initialize and modify options ( Set DOMAIN_ID to 20)
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 20));

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

  // Create Lift Encoder Pulses Publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher_lift_enc,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "encoder_lift_motor"));

  // Create Motor PWM Subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber_pwm,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, MotorPWMs),
      "motor_pwm"));

  // Create Lift Motor PWM Subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber_lift_pwm,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "lift_motor_pwm"));

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
  // RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_lift_pwm, &msg_lift_pwm, lift_motor_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{

  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK(rcl_publisher_fini(&publisher_enc, &node));
  RCCHECK(rcl_publisher_fini(&publisher_lift_enc, &node));
  RCCHECK(rcl_timer_fini(&timer));
  RCCHECK(rcl_subscription_fini(&subscriber_pwm, &node));
  RCCHECK(rcl_subscription_fini(&subscriber_lift_pwm, &node));
  rclc_executor_fini(&executor_pub);
  rclc_executor_fini(&executor_sub);
  RCCHECK(rcl_node_fini(&node));
  rclc_support_fini(&support);
}

void setup_hardware()
{
  encoder1.attachFullQuad(ENC1_A, ENC1_B);
  encoder2.attachFullQuad(ENC2_A, ENC2_B);
  encoder3.attachFullQuad(ENC3_A, ENC3_B);
  encoder4.attachFullQuad(ENC4_A, ENC4_B);
  // encoder5.attachFullQuad(ENC5_A, ENC5_B);

  encoder1.setCount(0);
  encoder2.setCount(0);
  encoder3.setCount(0);
  encoder4.setCount(0);
  // encoder5.setCount(0);

  setupMCPWM();
}

void setup()
{

  // Configure WiFi transport

  IPAddress agent_ip(10, 42, 0, 1);
  // IPAddress agent_ip(192, 168, 0, 192);
  size_t agent_port = 8888;

  char ssid[] = "adyansh_sakar";
  char psk[] = "12345678";

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

//. FOR TESTING PURPOSES
// void loop()
// {
//   // testMotor(50, 2000, 3);

//   // motorSetSpeed(1, 50);
//   // delay(2000);
//   // motorSetSpeed(1, 0);
//   // delay(2000);

//   // motorSetSpeed(2, 50);
//   // delay(2000);
//   // motorSetSpeed(2, 0);
//   // delay(2000);

//   // motorSetSpeed(3, 50);
//   // delay(2000);
//   // motorSetSpeed(3, 0);
//   // delay(2000);

//   // motorSetSpeed(4, 50);
//   // delay(2000);
//   // motorSetSpeed(4, 0);
//   // delay(2000);

//   // motorSetSpeed(5, 50);
//   // delay(2000);
//   // motorSetSpeed(5, 0);
//   // delay(2000);

//   motorSetSpeed(1, 100);
//   motorSetSpeed(2, 100);
//   motorSetSpeed(3, 100);
//   motorSetSpeed(4, 100);
//   // motorSetSpeed(5, 50);

//   // Print encoder values
//   Serial.print("Encoder 1: ");
//   Serial.print(encoder1.getCount());
//   Serial.print(", Encoder 2: ");
//   Serial.print(encoder2.getCount());
//   Serial.print(", Encoder 3: ");
//   Serial.print(encoder3.getCount());
//   Serial.print(", Encoder 4: ");
//   Serial.println(encoder4.getCount());
//   Serial.print(", Encoder 5: ");
//   Serial.println(encoder5.getCount());
// }
