#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "pin_map.hpp"
#include "cobra_line_sensor.hpp"
#include "mcpwm_servo.hpp"
#include "sharp_ir_driver.hpp"
#include "ultrasonic_sensor.hpp"
#include "ezButton.h"

#include "agrobot_interfaces/msg/servo_angles.h"
#include "agrobot_interfaces/msg/limit_switch_states.h"
#include "agrobot_interfaces/msg/sensor_datas.h"

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

// . Publisher Sensor Data
rcl_publisher_t publisher_sensor;
agrobot_interfaces__msg__SensorDatas msg_sensor_data;
rclc_executor_t executor_pub;
rcl_timer_t timer;

//. Publisher Limit Switch Data
rcl_publisher_t publisher_limit_switch;
agrobot_interfaces__msg__LimitSwitchStates msg_limit_switch_states;

//. Subscriber Servo Motor Angles
rcl_subscription_t subscriber_servo_angles;
agrobot_interfaces__msg__ServoAngles msg_servo_angles;
rclc_executor_t executor_sub;

// Sensor Objects
CobraLineSensor lineSensor;
ezButton limitSwitch1(LimitSwitch1);
ezButton limitSwitch2(LimitSwitch2);
ezButton limitSwitch3(LimitSwitch3);

bool micro_ros_init_successful;

// State machine states
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void sensor_data_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // Update Sensor readings
    lineSensor.updateRawReadings();                 // Line Sensor
    double *distances = getUltrasonicDistancesCM(); // Ultrasonic

    // Store the sensor data in the message
    msg_sensor_data.cobra_ir_1 = lineSensor.getRawReading(0);
    msg_sensor_data.cobra_ir_2 = lineSensor.getRawReading(1);
    msg_sensor_data.cobra_ir_3 = lineSensor.getRawReading(2);
    msg_sensor_data.cobra_ir_4 = lineSensor.getRawReading(3);

    msg_sensor_data.sharp_ir_1 = getDistanceinCM(SharpIR1);
    msg_sensor_data.sharp_ir_2 = getDistanceinCM(SharpIR2);

    msg_sensor_data.ultrasonic_1 = distances[0];
    msg_sensor_data.ultrasonic_2 = distances[1];
    msg_sensor_data.ultrasonic_3 = distances[2];
    msg_sensor_data.ultrasonic_4 = distances[3];

    // Store the limit switch states in the message
    msg_limit_switch_states.limit_switch_1 = limitSwitch1.getState();
    msg_limit_switch_states.limit_switch_2 = limitSwitch2.getState();
    msg_limit_switch_states.limit_switch_3 = limitSwitch3.getState();

    // Publish the sensor data
    RCSOFTCHECK(rcl_publish(&publisher_sensor, &msg_sensor_data, NULL));

    // Publish the limit switch states
    RCSOFTCHECK(rcl_publish(&publisher_limit_switch, &msg_limit_switch_states, NULL));
  }
}

void servo_angle_callback(const void *msgin)
{
  const agrobot_interfaces__msg__ServoAngles *msg_servo_angles = (const agrobot_interfaces__msg__ServoAngles *)msgin;

  // Set the Servo Motor Angles
  setMotorAngle(servo1, msg_servo_angles->servo1_angle);
  setMotorAngle(servo2, msg_servo_angles->servo2_angle);
  setMotorAngle(servo3, msg_servo_angles->servo3_angle);
  setMotorAngle(servo4, msg_servo_angles->servo4_angle);
  setMotorAngle(servo5, msg_servo_angles->servo5_angle);
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
  RCCHECK(rclc_node_init_default(&node, "agrobot_sensor_esp32s3", "", &support));

  // Create Sensor Data Publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher_sensor,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, SensorDatas),
      "sensor_data"));

  // Create Limit Switch States Publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher_limit_switch,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, LimitSwitchStates),
      "limit_switch_states"));

  // Create Subscriber for Servo Motor Angles
  RCCHECK(rclc_subscription_init_default(
      &subscriber_servo_angles,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, ServoAngles),
      "servo_angles"));

  // Create Timer for Sensor Data and Limit Switch Data
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      sensor_data_callback));

  // Create Executor
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_servo_angles, &msg_servo_angles, servo_angle_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{

  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK(rcl_publisher_fini(&publisher_sensor, &node));
  RCCHECK(rcl_publisher_fini(&publisher_limit_switch, &node));
  RCCHECK(rcl_timer_fini(&timer));
  RCCHECK(rcl_subscription_fini(&subscriber_servo_angles, &node));
  rclc_executor_fini(&executor_pub);
  rclc_executor_fini(&executor_sub);
  RCCHECK(rcl_node_fini(&node));
  rclc_support_fini(&support);
}

void setup_hardware()
{
  setupMCPWMServo();         // Setup Servo Motors
  lineSensor.setup();        // Setup Cobra Line Sensors
  setupSharpIRsensor();      // Setup Sharp IR Sensors
  ultrasonic_sensor_setup(); // Setup Ultrasonic Sensors

  // Set the debounce time for the limit switches
  limitSwitch1.setDebounceTime(50);
  limitSwitch2.setDebounceTime(50);
  limitSwitch3.setDebounceTime(50);
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
      limitSwitch1.loop();
      limitSwitch2.loop();
      limitSwitch3.loop();
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
