/**
 * @file control_system.c
 * @brief Control system implementation.
 * @author Jonas Frei
 * @date 01.09.2026
 * @version 1.0
 */

#include "control_system.h"
#include "safety_system.h"
#include "udp_interface.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <math.h>

static const char *TAG = "Control System"; /**< Tag for logging purposes */

static control_system_config_t cfg = {0}; /**< Configuration for the control system */

static EventGroupHandle_t control_system_event_group = NULL; /**< Event group for control system events */
#define CONTROL_SYSTEM_INITIALIZED_BIT BIT0                  /**< Bit indicating control system is initialized in the event group */

esp_err_t control_system_init(const control_system_config_t *config)
{
    ESP_LOGI(TAG, "Initializing control system...");

    // Create control system event group if it doesn't exist
    if (control_system_event_group == NULL)
    {
        control_system_event_group = xEventGroupCreate();
        if (control_system_event_group == NULL)
        {
            ESP_LOGE(TAG, "Failed to create control system event group");
            return ESP_FAIL;
        }
    }
    else if (xEventGroupGetBits(control_system_event_group) & CONTROL_SYSTEM_INITIALIZED_BIT)
    {
        ESP_LOGW(TAG, "Control system already initialized.");
        return ESP_OK;
    }

    // Store the configuration
    if (config == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    cfg = *config;

    // Initialize all control system components
    ESP_ERROR_CHECK(encoder_driver_init(&cfg.encoder_config));
    ESP_ERROR_CHECK(motor_driver_init(&cfg.motor_driver_config));
    ESP_ERROR_CHECK(motor_model_init(&cfg.motor_model_config));
    ESP_ERROR_CHECK(pd_controller_init(&cfg.pd_controller_config));
    ESP_ERROR_CHECK(pi_controller_init(&cfg.pi_controller_config));
    ESP_ERROR_CHECK(kinematics_init(&cfg.kinematics_config));


    // Set the control system initialized bit in the event group
    xEventGroupSetBits(control_system_event_group, CONTROL_SYSTEM_INITIALIZED_BIT);
    ESP_LOGI(TAG, "Control system initialized.");
    return ESP_OK;
}
    
/*
//Hilfsfunktion für die Dreiecksfunktion / Periodische Dreiecksfunktion (normiert auf Periode 1, Wertebereich 0 bis 1)
static float triangle_function(float x)
{
    float wrapped = fmodf(x, 1.0f); // Wrap x to the range [0, 1)
    if (wrapped < 0.0f)
    {
        wrapped += 1.0f; // Ensure wrapped is non-negative
    }
    return 2.0f * fabsf(wrapped - 0.5f); // Triangle wave formula
}
*/

void control_system_task(void *arg)
{
    // Register the control system task with the safety system
    ESP_ERROR_CHECK(safety_system_register_task());

    // Wait for the control system to be initialized before starting the main loop
    xEventGroupWaitBits(control_system_event_group, CONTROL_SYSTEM_INITIALIZED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // Main control loop
    float shaft_angles[2] = {0.0f, 0.0f};
    //float voltages[2] = {0.0f, 0.0f};
    //float t = 0.0f; // Time variable for control calculations
    //float dt = (float)cfg.task_period_ms / 1000.0f; // Zeitschritt pro Regeltakt (z.b. 0.1s bei 100ms)
    float q[2] = {0.0f, 0.0f}; // Gelenkpositionen
    float qd[2] = {0.0f, 0.0f}; // Gelenkgeschwindigkeiten
    float U[2] = {0.0f, 0.0f}; // Motorspannungen
    float Q[2] = {0.0f, 0.0f}; // Soll Drehmoment
    //float q_soll[2] = {0.0f, 0.0f}; // Soll Gelenkpositionen
    float qd_soll[2] = {0.0f, 0.0f}; // Soll Gelenkgeschwindigkeiten
    //float max_qd = cfg.motor_model_config.params[1].max_velocity; // Maximale Geschwindigkeit für die Gelenke (rad/s)
    //float transmission_ratio = cfg.motor_model_config.params[1].transmission_ratio; // Übersetzungsverhältnis für die Gelenke
    float q_global[3] = {0.0f, 0.0f, 0.0f}; // Globale Position des Roboters (x, y, phi)
    //float qd_global[3] = {0.0f, 0.0f, 0.0f}; // Globale Geschwindigkeit des Roboters (vx, vy, omega)
    float xid_soll_robot[3] = {0.0f, 0.0f, 1.047f}; // Soll Geschwindigkeit des Roboters im Roboterkoordinatensystem (vx, vy, omega)
    TickType_t last_wake = xTaskGetTickCount();
    while (!safety_system_is_shutting_down())
    {
        /* Praktikum 1
        // Motorspannungen berechnen
        // U_L = sin(0.2*pi*t)
        voltages[0] = sinf(0.2f * (float)M_PI * t);

        // U_R = 2*triangle(0.5*t) - 1
        voltages[1] = 2.0f * triangle_function(0.5f * t) - 1.0f;

        motor_driver_set_voltage(voltages);

        // Get the angles from both encoders
        encoder_driver_get_angles(shaft_angles);
        
        // Add telemetry samples to the UDP interface
        udp_interface_add_control_sample(shaft_angles[0], shaft_angles[1]);
        
        // Update time variable for the next iteration
        t += dt;
        */
        
        /* Praktikum 2 Aufgabe 1
        encoder_driver_get_angles(shaft_angles);

        motor_model_calc_q(shaft_angles, q);
        motor_model_calc_qd(q, qd);
        motor_model_calc_voltage(Q, qd, U);
        motor_driver_set_voltage(U);

        udp_interface_add_control_sample(q[0], q[1]);
        ESP_LOGI(TAG, "q = [%.3f, %.3f] rad", q[0], q[1]);

        // Wait for the next cycle
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(cfg.task_period_ms));
        */

        /* Praktikum 2 Aufgabe 2
        encoder_driver_get_angles(shaft_angles);

        motor_model_calc_q(shaft_angles, q);
        motor_model_calc_qd(q, qd);

        q_soll[0] = q[0];
        q_soll[1] = q[0];

        pd_controller_calc_Q(q, q_soll, Q);
        motor_model_calc_voltage(Q, qd, U);
        motor_driver_set_voltage(U);

        udp_interface_add_control_sample(q[0], q[1]);
        ESP_LOGI(TAG, "Q = [%.3f, %.3f] Nm", Q[0], Q[1]);

        // Wait for the next cycle
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(cfg.task_period_ms));
        */
        /* Praktikum 3 Aufgabe 1
        encoder_driver_get_angles(shaft_angles);

        motor_model_calc_q(shaft_angles, q);
        motor_model_calc_qd(q, qd);

        q_soll[0] = q[1];
        q_soll[1] = q[0];

        pd_controller_calc_Q(q, q_soll, Q);
        motor_model_calc_voltage(Q, qd, U);
        motor_driver_set_voltage(U);

        udp_interface_add_control_sample(q[0], q[1]);
        ESP_LOGI(TAG, "Q = [%.3f, %.3f] Nm", Q[0], Q[1]);

        // Wait for the next cycle
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(cfg.task_period_ms));
        */
        /* Praktikum 3 Aufgabe 2
        encoder_driver_get_angles(shaft_angles);

        motor_model_calc_q(shaft_angles, q);
        motor_model_calc_qd(q, qd);

        qd_soll[1] = q[0] * max_qd /(2.0f * M_PI * transmission_ratio); // Set the desired velocity for the second joint based on the first joint's position
        qd_soll[0] = qd[0];


        pi_controller_calc_Q(qd, qd_soll, Q);
        motor_model_calc_voltage(Q, qd, U);
        motor_driver_set_voltage(U);


        udp_interface_add_control_sample(q[0], q[1]);
        ESP_LOGI(TAG, "Q = [%.3f, %.3f] Nm", q[0], qd[1]);

        // Wait for the next cycle
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(cfg.task_period_ms));
        */
        
        /* Praktikum 4 Aufgabe 1
        encoder_driver_get_angles(shaft_angles);

        motor_model_calc_q(shaft_angles, q);
        motor_model_calc_qd(q, qd);
        //motor_model_calc_voltage(Q, qd, U);
        //motor_driver_set_voltage(U);

        kinematics_calc_fw_kin_odom(qd, q_global, qd_global);


        udp_interface_add_control_sample(q_global[0], q_global[1]);
        ESP_LOGI(TAG, "q_global = [%.3f, %.3f, %.3f] m, m, rad", q_global[0], q_global[1], q_global[2]);
        //ESP_LOGI(TAG, "qd_global = [%.3f, %.3f, %.3f] m/s, m/s, rad/s", qd_global[0], qd_global[1], qd_global[2]);

        // Wait for the next cycle
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(cfg.task_period_ms));
        */
        
        encoder_driver_get_angles(shaft_angles);
        motor_model_calc_q(shaft_angles, q);
        motor_model_calc_qd(q, qd);

        kinematics_calc_inv_kin(xid_soll_robot, qd_soll);
        pi_controller_calc_Q(qd, qd_soll, Q);
        motor_model_calc_voltage(Q, qd, U);
        motor_driver_set_voltage(U);

        udp_interface_add_control_sample(q_global[0], q_global[1]);
        // LOGI nicht einschalten... sonst hackt der Motor
        //ESP_LOGI(TAG, "qd_soll = [%.3f, %.3f] m/s, m/s", q[0], q[1]);
        //ESP_LOGI(TAG, "qd_global = [%.3f, %.3f, %.3f] m/s, m/s, rad/s", qd_global[0], qd_global[1], qd_global[2]);

        // Wait for the next cycle
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(cfg.task_period_ms));

    }

    // Cleanup and exit the task
    xEventGroupClearBits(control_system_event_group, CONTROL_SYSTEM_INITIALIZED_BIT);
    ESP_ERROR_CHECK(safety_system_unregister_task());
    ESP_LOGI(TAG, "Control system task finished.");
    vTaskDelete(NULL);
}
