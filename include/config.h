/**
 * @file config.c
 * @brief Configuration constants for the robot system.
 * @author Jonas Frei
 * @date 01.09.2026
 * @version 1.0
 *
 * @warning DO NOT add any other content besides #define statements to this file.
 * This file is intended to be included only in main.c to provide configuration constants.
 * DO NOT include this file in other source files besides main.c.
 *
 * @todo Replace these constants to match your specific robot.
 */

#pragma once

// Safety system configuration constants
#define SAFETY_SYSTEM_TASK_PERIOD_MS 1     /**< Safety system task period [ms] */
#define SAFETY_SYSTEM_EVENT_QUEUE_LENGTH 3 /**< Safety system event queue length [-] */

// I2C master configuration constants
#define I2C_MASTER_SCL_IO 33 /**< GPIO pin for I2C master SCL */
#define I2C_MASTER_SDA_IO 32 /**< GPIO pin for I2C master SDA */

// UPD interface configuration constants
#define UPD_INTERFACE_HOSTNAME "robot_9876"     /**< Hostname for the ESP32 on the network */
#define UPD_INTERFACE_UDP_PORT 3333          /**< UDP port for communication */
#define UPD_INTERFACE_TASK_PERIOD_MS 20      /**< Network task period [ms] */
#define UPD_INTERFACE_TELEMETRY_QUEUE_LEN 64 /**< Length of the telemetry queue [-] */

// Encoder driver configuration constants
#define ENCODER_DRIVER_ENC_A_PPR 16.0f /**< Encoder A pulses per revolution */
#define ENCODER_DRIVER_ENC_B_PPR 16.0f /**< Encoder B pulses per revolution */

// Motor driver configuration constants
#define MOTOR_DRIVER_BUS_VOLTAGE_UPDATE_PERIOD_MS 10000 /**< Bus voltage update period [ms] */
#define MOTOR_DRIVER_MIN_BUS_VOLTAGE 5.0f               /**< Minimum bus voltage [V] */
#define MOTOR_DRIVER_MOTOR_A_FREE_SPIN false            /**< Motor A free spin configuration */
#define MOTOR_DRIVER_MOTOR_B_FREE_SPIN false            /**< Motor B free spin configuration */

// Motor model configuration
#define MOTOR_MODEL_NR_OF_MOTORS 2 /**< Number of motors in the system */
#define MOTOR_L_TRANSMISSION_RATIO (3441.0f / 104.0f) /**< Transmission ratio for the left motor */ 
#define MOTOR_R_TRANSMISSION_RATIO (3441.0f / 104.0f) /**< Transmission ratio for the right motor */
#define MOTOR_L_ARMATURE_RESISTANCE 8.0f /**< Armature resistance for the left motor [Ohm] */
#define MOTOR_R_ARMATURE_RESISTANCE 8.0f /**< Armature resistance for the right motor [Ohm] */
#define MOTOR_L_MOTOR_CONSTANT 0.00844f /**< Motor constant for the left motor [Nm/A] */
#define MOTOR_R_MOTOR_CONSTANT 0.00844f /**< Motor constant for the right motor [Nm/A] */
#define MOTOR_L_MAX_TORQUE_INTERMITTENT 0.1f /**< Maximum intermittent torque for the left motor [Nm] */
#define MOTOR_R_MAX_TORQUE_INTERMITTENT 0.1f /**< Maximum intermittent torque for the right motor [Nm] */
#define MOTOR_L_MAX_VELOCITY (6700.0f/60.0f * 2.0f * 3.14159f) /**< Maximum velocity for the left motor [rad/s] */
#define MOTOR_R_MAX_VELOCITY (6700.0f/60.0f * 2.0f * 3.14159f) /**< Maximum velocity for the right motor [rad/s] */
#define MOTOR_L_INVERT_DIRECTION true /**< Invert direction for the left motor */
#define MOTOR_R_INVERT_DIRECTION false /**< Invert direction for the right motor */

// PD controller configuration constants
#define MOTOR_J 0.000000068f /**< Motor inertia [kg*m^2] */
#define MOTOR_J_RED (MOTOR_J * (MOTOR_L_TRANSMISSION_RATIO * MOTOR_L_TRANSMISSION_RATIO)) /**< Reduced motor inertia [kg*m^2] */

#define PD_CONTROLLER_D 0.7f
#define PD_CONTROLLER_OMEGA_0 (CONTROL_SYSTEM_TASK_FREQUENCY_HZ/(4.6f*PD_CONTROLLER_D))
#define PD_CONTROLLER_DT ((float)CONTROL_SYSTEM_TASK_PERIOD_MS / 1000.0f)
#define PD_CONTROLLER_KP (PD_CONTROLLER_OMEGA_0 * PD_CONTROLLER_OMEGA_0)
#define PD_CONTROLLER_KD (2.0f * PD_CONTROLLER_D * PD_CONTROLLER_OMEGA_0) /**< Derivative gain for the PD controller [1/s] */
#define PD_CONTROLLER_DIM 2 /**< Dimension of the input signal for the PD controller (e.g., 1 for scalar, 2 for vector) */

// Control system configuration constants
#define CONTROL_SYSTEM_TASK_PERIOD_MS 5                                               /**< Control system task period [ms] */
#define CONTROL_SYSTEM_TASK_PERIOD_S ((float)CONTROL_SYSTEM_TASK_PERIOD_MS / 1000.0f) /**< Control system task period [s] */
#define CONTROL_SYSTEM_TASK_FREQUENCY_HZ (1.0f / (float)CONTROL_SYSTEM_TASK_PERIOD_S) /**< Control system task frequency [Hz] */

// Sequencer configuration constants
#define SEQUENCER_TASK_PERIOD_MS 100 /**< Sequencer task period [ms] */
