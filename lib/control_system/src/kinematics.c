/**
 * @file kinematics.c
 * @brief Kinematics implementation for the robot system.
 * @author Jonas Frei
 * @date 01.09.2026
 * @version 1.0
 */

#include "kinematics.h"
#include "common_math.h"
#include <math.h>

static kinematics_config_t cfg = {0}; /**< Configuration for the kinematics module */

esp_err_t kinematics_init(const kinematics_config_t *config)
{
    // Store the configuration
    if (config == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    cfg = *config;

    // Validate the configuration parameters
    if (cfg.dt <= 0.0f ||
        cfg.wheel_distance <= 0.0f)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

void kinematics_calc_fw_kin_odom(const float qd[2], float xi[3], float xid[3])
{
    float xi_prev[3] = {xi[0], xi[1], xi[2]}; // Previous robot pose for derivative calculation
    float qd_robot[3] = {0.0f, 0.0f, 0.0f}; // Robotgeschwindigkeit im Roboterkoordinatensystem

    // R_psid_R
    qd_robot[0] = qd[0] * 0.5f + qd[1] * 0.5f; // vx
    qd_robot[1] = 0.0f;                        // vy
    qd_robot[2] = -qd[0]/cfg.wheel_distance + qd[1]/cfg.wheel_distance; // omega
    
    float phi = xi[2]; // Current orientation of the robot
    // G_psid_R + im gleichen Schritt Integrate 
    xi[0] += cosf(phi) * qd_robot[0] * cfg.dt; // x
    xi[1] += sinf(phi) * qd_robot[0] * cfg.dt; // y
    xi[2] += qd_robot[2] * cfg.dt; // phi

    // Calculate the robot velocities in the global frame using the derivative function
    derivative(xi, xi_prev, &cfg.dt, xid, 3); // Calculate the robot velocities in the global frame
}   

void kinematics_calc_inv_kin(const float xid_d[3], float qd_d[2])
{
    // Radius Wheels = 0.04 m
    qd_d[0] = (xid_d[0] - cfg.wheel_distance * xid_d[2] * 0.5f); // vx - omega * wheel_distance / 2
    qd_d[1] = (xid_d[0] + cfg.wheel_distance * xid_d[2] * 0.5f); // vx + omega * wheel_distance / 2

}
