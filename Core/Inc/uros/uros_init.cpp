/*
 * uros_init.cpp
 *
 *  Created on: Apr 9, 2025
 *      Author: stanly
 */


#include "uros_init.h"

rcl_publisher_t           pose_pub;
nav_msgs__msg__Odometry   pose_msg;
rcl_subscription_t        cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;
rcl_timer_t pose_pub_timer;


rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;
rclc_executor_t executor;

agent_status_t status = AGENT_WAITING;


int ping_fail_count = 0;
#define MAX_PING_FAIL_COUNT 5


extern UART_HandleTypeDef USARTx;

void uros_init(void) {
  // Initialize micro-ROS
  rmw_uros_set_custom_transport(
    true,
    (void *) &USARTx,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);
  
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();

  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
  printf("Error on default allocators (line %d)\n", __LINE__); 
  }
}

void uros_agent_status_check(void) {
  switch (status) {
    case AGENT_WAITING:
      handle_state_agent_waiting();
      break;
    case AGENT_AVAILABLE:
      handle_state_agent_available();
      break;
    case AGENT_CONNECTED:
      handle_state_agent_connected();
      break;
    case AGENT_TRYING:
      handle_state_agent_trying();
      break;
    case AGENT_DISCONNECTED:
      handle_state_agent_disconnected();
      break;
    default:
      break;
  }
}

void handle_state_agent_waiting(void) {
  status = (rmw_uros_ping_agent(100, 10) == RMW_RET_OK) ? AGENT_AVAILABLE : AGENT_WAITING;
}
void handle_state_agent_available(void) {
  uros_create_entities();
  status = AGENT_CONNECTED;
}
void handle_state_agent_connected(void) {
  if(rmw_uros_ping_agent(20, 5) == RMW_RET_OK){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    ping_fail_count = 0; // Reset ping fail count
  } else {
    ping_fail_count++;
    if(ping_fail_count >= MAX_PING_FAIL_COUNT){
      status = AGENT_TRYING;
    }
  }
}
void handle_state_agent_trying(void) {
  if(rmw_uros_ping_agent(50, 10) == RMW_RET_OK){
    status = AGENT_CONNECTED;
    ping_fail_count = 0; // Reset ping fail count
  } else {
    ping_fail_count++;
    if(ping_fail_count >= MAX_PING_FAIL_COUNT){
      status = AGENT_DISCONNECTED;
      ping_fail_count = 0;
    }
  }
}
void handle_state_agent_disconnected(void) {
  uros_destroy_entities();
  status = AGENT_WAITING;
}


void uros_create_entities(void) {
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, DOMAIN_ID);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator); // Initialize support structure

  rcl_init_options_fini(&init_options);
  
  rclc_node_init_default(&node, NODE_NAME, "", &support);                       // Initialize node

  rclc_publisher_init_default(                                                  // Initialize publisher for pose
    &pose_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "robot/pose");
  pose_msg.pose.pose.position.x = 0.0;
  pose_msg.pose.pose.position.y = 0.0;
  pose_msg.pose.pose.position.z = 0.0;
  pose_msg.pose.pose.orientation.x = 0.0;
  pose_msg.pose.pose.orientation.y = 0.0;
  pose_msg.pose.pose.orientation.z = 0.0;
  pose_msg.pose.pose.orientation.w = 1.0;

  rmw_uros_set_publisher_session_timeout(                                       // Set session timeout for publisher
    rcl_publisher_get_rmw_handle(&pose_pub),
    10);

  rclc_subscription_init_default(                                               // Initialize subscriber for command velocity
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "robot/cmd_vel");
  cmd_vel_msg.linear.x = 0.0;
  cmd_vel_msg.linear.y = 0.0;
  cmd_vel_msg.linear.z = 0.0;
  cmd_vel_msg.angular.x = 0.0;
  cmd_vel_msg.angular.y = 0.0;
  cmd_vel_msg.angular.z = 0.0;

  rclc_timer_init_default(&pose_pub_timer, &support, RCL_MS_TO_NS(100), pose_pub_timer_cb);

  
  rclc_executor_init(&executor, &support.context, 2, &allocator); // Create executor

  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_sub_cb, ON_NEW_DATA); // Add subscriber to executor
  rclc_executor_add_timer(&executor, &pose_pub_timer); // Add timer to executor
}
void uros_destroy_entities(void) {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // Destroy publisher
  rcl_publisher_fini(&pose_pub, &node);

  // Destroy subscriber
  rcl_subscription_fini(&cmd_vel_sub, &node);

  rcl_timer_fini(&pose_pub_timer);

  // Destroy executor
  rclc_executor_fini(&executor);

  // Destroy node
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void cmd_vel_sub_cb(const void* msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_vel_msg = *msg;
  pose_msg.pose.pose.position.x = cmd_vel_msg.linear.x*0.02;
  pose_msg.pose.pose.position.y = cmd_vel_msg.linear.y*0.02;
  pose_msg.pose.pose.position.z = cmd_vel_msg.linear.z*0.02;
  pose_msg.pose.pose.orientation.z = cmd_vel_msg.angular.z*0.02;
}

void pose_pub_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  rcl_publish(&pose_pub, &pose_msg, NULL);
}