/**
 * @file pi_controller.c
 * @brief PI controller implementation file.
 * @author Jonas Frei
 * @date 01.09.2026
 * @version 1.0
 */

#include "pi_controller.h"
#include "common_math.h"
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <math.h>

static pi_controller_config_t cfg = {0}; /**< Configuration for the PI velocity controller */

static float *e = NULL;                       /**< Pointer to store the integral terms for the PI velocity controller */
static EventGroupHandle_t event_group = NULL; /**< Event group for PI controller events */
#define INTEGRATOR_ENABLED_BIT BIT0           /**< Bit for integrator enabled event */

esp_err_t pi_controller_init(const pi_controller_config_t *config)
{
    // Store the configuration
    if (config == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    cfg = *config;

    // Validate the configuration parameters
    if (cfg.dt <= 0.0f ||
        cfg.kp < 0.0f ||
        cfg.ki < 0.0f ||
        cfg.integral_limit < 0.0f ||
        cfg.M == NULL ||
        cfg.dim <= 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    for (uint8_t i = 0; i < cfg.dim; i++)
    {
        if (cfg.M[i * cfg.dim + i] <= 0.0f) // Check diagonal elements of the mass matrix
        {
            return ESP_FAIL;
        }
    }

    // Create an event group for PI controller events
    event_group = xEventGroupCreate();
    if (event_group == NULL)
    {
        return ESP_FAIL;
    }

    // Allocate memory for the integral terms
    e = malloc(sizeof(float) * cfg.dim);
    if (e == NULL)
    {
        return ESP_FAIL;
    }

    // Initialize the integral terms to zero
    memset(e, 0, sizeof(float) * cfg.dim);

    return ESP_OK;
}

void pi_controller_cleanup(void)
{
    if (e != NULL)
    {
        free(e);
        e = NULL;
    }
}

void pi_controller_disable_integrator(void)
{
    // Clear the integrator enabled bit in the event group
    xEventGroupClearBits(event_group, INTEGRATOR_ENABLED_BIT);
}

void pi_controller_enable_integrator(void)
{
    // Set the integrator enabled bit in the event group
    xEventGroupSetBits(event_group, INTEGRATOR_ENABLED_BIT);
}

void pi_controller_reset_integrator(void)
{
    // Reset the integral terms to zero
    memset(e, 0, sizeof(float) * cfg.dim);
}


void pi_controller_calc_Q(const float *qd, const float *qd_d, float *Q)
{
    for (uint8_t i = 0; i < cfg.dim; i++)
    {
        float ed = qd_d[i] - qd[i];
        e[i] += (ed * cfg.dt); 

        if (xEventGroupGetBits(event_group) & INTEGRATOR_ENABLED_BIT) // Check if the integrator is enabled --> defined in safety_system.c
        {
            // Limit the integral term to prevent windup
            if (e[i] > cfg.integral_limit) {
                e[i] = cfg.integral_limit;
            } else if (e[i] < -cfg.integral_limit) {
                e[i] = -cfg.integral_limit;
            }
        }
        else
        {
            e[i] = 0.0f; // Reset the integral term if the integrator is disabled
        }

        float qdd = cfg.kp * ed + cfg.ki * e[i]; 
        Q[i] = cfg.M[i * cfg.dim + i] * qdd; 
    }
}
