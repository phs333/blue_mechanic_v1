#include "motion.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "hardware.h"
#include "storage.h"
#include "can_bus.h"

static float get_deg_per_step(app_context_t *ctx, char axis);
static float normalize_angle_deg(float angle);
static esp_err_t compute_axis_deviation(app_context_t *ctx, char axis, float *deviation_deg);
static esp_err_t do_motion_move_axis_relative(app_context_t *ctx, char axis, int32_t requested_steps);
static esp_err_t do_motion_move_z_relative(app_context_t *ctx, int32_t requested_steps);

static float get_deg_per_step(app_context_t *ctx, char axis)
{
    size_t axis_index = 0;
    char axis_upper = (char)toupper((unsigned char)axis);
    if (axis_upper == 'X') {
        axis_index = AXIS_X_ID;
    } else if (axis_upper == 'Y') {
        axis_index = AXIS_Y_ID;
    } else {
        return 360.0f / (float)ctx->settings.steps_per_rev;
    }
    uint16_t msteps = ctx->settings.tmc_microsteps[axis_index];
    if (msteps == 0) {
        msteps = 16;
    }
    return 360.0f / ((float)ctx->settings.steps_per_rev * (float)msteps);
}


static esp_err_t do_motion_adjust_axis_to_home(app_context_t *ctx, char axis)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    char axis_upper = (char)toupper((unsigned char)axis);
    gpio_num_t dir_pin;
    bool invert;
    float target_deg;

    if (axis_upper == 'X') {
        dir_pin = DIR_X;
        invert = INVERTER_X;
        target_deg = ctx->settings.home_x_deg;
    } else if (axis_upper == 'Y') {
        dir_pin = DIR_Y;
        invert = INVERTER_Y;
        target_deg = ctx->settings.home_y_deg;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (int attempt = 0; attempt < 3; ++attempt) {
        float actual_deg = 0.0f;
        esp_err_t err = hardware_read_axis_encoder(axis_upper, &actual_deg);
        if (err != ESP_OK) {
            xSemaphoreGive(ctx->motion_mutex);
            return err;
        }

        float error_deg = normalize_angle_deg(target_deg - actual_deg);
        if (fabsf(error_deg) <= 0.3f) {
            break;
        }

        float deg_per_step = get_deg_per_step(ctx, axis_upper);
        int32_t steps_to_move = (int32_t)floorf(fabsf(error_deg) / deg_per_step);
        if (steps_to_move <= 0) {
            break;
        }

        gpio_set_level(dir_pin, ((error_deg > 0.0f) ^ invert) ? 1 : 0);
        hardware_step_pulse_rmt(axis_upper, steps_to_move, 800);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    xSemaphoreGive(ctx->motion_mutex);
    return ESP_OK;
}


static esp_err_t do_motion_home_z(app_context_t *ctx)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    ctx->state.z_bloqueado = false;
    ctx->state.em_homing_z = true;

    gpio_set_level(DIR_Z, Z_DIR_DOWN);
    int32_t search_steps = 0;
    while (!hardware_is_z_switch_pressed() && search_steps < Z_HOME_SEARCH_LIMIT_STEPS) {
        hardware_step_pulse(STEP_Z, 400);
        ++search_steps;
        if ((search_steps & 0xFFFU) == 0U) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    if (search_steps >= Z_HOME_SEARCH_LIMIT_STEPS && !hardware_is_z_switch_pressed()) {
        ctx->state.em_homing_z = false;
        ctx->state.z_bloqueado = true;
        xSemaphoreGive(ctx->motion_mutex);
        return ESP_ERR_TIMEOUT;
    }

    gpio_set_level(DIR_Z, Z_DIR_UP);
    int32_t release_steps = 0;
    while (hardware_is_z_switch_pressed() && release_steps < Z_HOME_RELEASE_LIMIT_STEPS) {
        hardware_step_pulse(STEP_Z, 800);
        ++release_steps;
        if ((release_steps & 0xFFFU) == 0U) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    if (release_steps >= Z_HOME_RELEASE_LIMIT_STEPS && hardware_is_z_switch_pressed()) {
        ctx->state.em_homing_z = false;
        ctx->state.z_bloqueado = true;
        xSemaphoreGive(ctx->motion_mutex);
        return ESP_ERR_TIMEOUT;
    }

    for (int32_t i = 0; i < PASSOS_ALIVIO_EXTRA_Z; ++i) {
        hardware_step_pulse(STEP_Z, 800);
    }

    ctx->state.atual_z = 0;
    ctx->state.em_homing_z = false;
    xSemaphoreGive(ctx->motion_mutex);
    return ESP_OK;
}



static esp_err_t do_motion_move_axis_force(app_context_t *ctx, char axis, int32_t requested_steps)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    char axis_upper = (char)toupper((unsigned char)axis);
    ESP_LOGI(APP_TAG, "MOVE_F %c steps=%d", axis_upper, (int)requested_steps);

    if (axis_upper == 'X' || axis_upper == 'Y') {
        gpio_num_t dir_pin;
        bool invert;

        if (axis_upper == 'X') {
            dir_pin = DIR_X;
            invert = INVERTER_X;
        } else {
            dir_pin = DIR_Y;
            invert = INVERTER_Y;
        }

        bool positive_motion = requested_steps > 0;
        if (invert) {
            positive_motion = !positive_motion;
        }

        if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGE(APP_TAG, "MOVE_F %c timeout no motion_mutex", axis_upper);
            return ESP_ERR_TIMEOUT;
        }

        gpio_set_level(dir_pin, positive_motion ? 1 : 0);
        esp_err_t rmt_err = hardware_step_pulse_rmt(axis_upper, (uint32_t)labs(requested_steps), ctx->state.move_delay_us);
        ESP_LOGI(APP_TAG, "MOVE_F %c RMT retornou %s", axis_upper, esp_err_to_name(rmt_err));

        xSemaphoreGive(ctx->motion_mutex);
        return rmt_err;
    }

    if (axis_upper == 'Z') {
        if (ctx->state.z_bloqueado) {
            return ESP_ERR_INVALID_STATE;
        }

        if (requested_steps == 0) {
            return ESP_OK;
        }

        if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }

        bool move_up = requested_steps > 0;
        int32_t steps = labs(requested_steps);
        gpio_set_level(DIR_Z, move_up ? Z_DIR_UP : Z_DIR_DOWN);

        for (int32_t i = 0; i < steps; ++i) {
            if (!move_up) {
                if (ctx->state.atual_z <= 0 || hardware_is_z_switch_pressed()) {
                    ctx->state.z_bloqueado = hardware_is_z_switch_pressed();
                    break;
                }
                ctx->state.atual_z--;
            } else {
                if (ctx->state.atual_z >= ctx->settings.max_passos_z) {
                    break;
                }
                ctx->state.atual_z++;
            }
            hardware_step_pulse(STEP_Z, ctx->state.move_delay_us);
            if ((i & 0xFFFU) == 0U) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        xSemaphoreGive(ctx->motion_mutex);
        return ESP_OK;
    }

    return ESP_ERR_INVALID_ARG;
}

static esp_err_t do_motion_move_axis(app_context_t *ctx, char axis, int32_t requested_steps)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "ctx nulo");

    char axis_upper = (char)toupper((unsigned char)axis);
    if (axis_upper == 'X' || axis_upper == 'Y') {
        return do_motion_move_axis_relative(ctx, axis_upper, requested_steps);
    }
    if (axis_upper == 'Z') {
        return do_motion_move_z_relative(ctx, requested_steps);
    }
    return ESP_ERR_INVALID_ARG;
}

static float normalize_angle_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static esp_err_t compute_axis_deviation(app_context_t *ctx, char axis, float *deviation_deg)
{
    float actual_deg = 0.0f;
    ESP_RETURN_ON_FALSE(deviation_deg != NULL, ESP_ERR_INVALID_ARG, APP_TAG, "deviation_deg nulo");
    ESP_RETURN_ON_ERROR(hardware_read_axis_encoder(axis, &actual_deg), APP_TAG, "Falha ao ler encoder");

    float home_deg = 0.0f;
    if (axis == 'X') {
        home_deg = ctx->settings.home_x_deg;
    } else if (axis == 'Y') {
        home_deg = ctx->settings.home_y_deg;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    *deviation_deg = normalize_angle_deg(actual_deg - home_deg);
    return ESP_OK;
}

static esp_err_t do_motion_move_axis_relative(app_context_t *ctx, char axis, int32_t requested_steps)
{
    gpio_num_t dir_pin;
    bool invert;
    float reduction;

    if (axis == 'X') {
        dir_pin = DIR_X;
        invert = INVERTER_X;
        reduction = REDUCAO_X;
    } else if (axis == 'Y') {
        dir_pin = DIR_Y;
        invert = INVERTER_Y;
        reduction = REDUCAO_Y;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    float current_deviation_deg = 0.0f;
    ESP_RETURN_ON_ERROR(compute_axis_deviation(ctx, axis, &current_deviation_deg), APP_TAG, "Falha ao calcular desvio");

    bool positive_motion = requested_steps > 0;
    if (invert) {
        positive_motion = !positive_motion;
    }

    float deg_per_step = get_deg_per_step(ctx, axis);
    float requested_move_deg = (positive_motion ? 1.0f : -1.0f) * fabsf((float)requested_steps) * deg_per_step;
    float target_deviation_deg = current_deviation_deg + requested_move_deg;
    float limit_deg = LIMITE_GRAUS_XY * reduction;

    if (target_deviation_deg > limit_deg) {
        target_deviation_deg = limit_deg;
    } else if (target_deviation_deg < -limit_deg) {
        target_deviation_deg = -limit_deg;
    }

    float permitted_move_deg = target_deviation_deg - current_deviation_deg;
    if (fabsf(permitted_move_deg) < deg_per_step) {
        return ESP_OK;
    }

    int32_t steps_to_execute = (int32_t)floorf(fabsf(permitted_move_deg) / deg_per_step);
    if (steps_to_execute <= 0) {
        return ESP_OK;
    }

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    gpio_set_level(dir_pin, ((permitted_move_deg > 0.0f) ^ invert) ? 1 : 0);
    esp_err_t rmt_err = hardware_step_pulse_rmt(axis, steps_to_execute, ctx->state.move_delay_us);

    xSemaphoreGive(ctx->motion_mutex);
    return rmt_err;
}

static esp_err_t do_motion_move_z_relative(app_context_t *ctx, int32_t requested_steps)
{
    if (ctx->state.z_bloqueado) {
        return ESP_ERR_INVALID_STATE;
    }

    if (requested_steps == 0) {
        return ESP_OK;
    }

    if (xSemaphoreTake(ctx->motion_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool move_up = requested_steps > 0;
    int32_t steps = labs(requested_steps);
    gpio_set_level(DIR_Z, move_up ? Z_DIR_UP : Z_DIR_DOWN);

    for (int32_t i = 0; i < steps; ++i) {
        if (!move_up) {
            if (ctx->state.atual_z <= 0 || hardware_is_z_switch_pressed()) {
                ctx->state.z_bloqueado = hardware_is_z_switch_pressed();
                break;
            }
            ctx->state.atual_z--;
        } else {
            if (ctx->state.atual_z >= ctx->settings.max_passos_z) {
                break;
            }
            ctx->state.atual_z++;
        }
        hardware_step_pulse(STEP_Z, ctx->state.move_delay_us);
        if ((i & 0xFFFU) == 0U) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    xSemaphoreGive(ctx->motion_mutex);
    return ESP_OK;
}

static void motion_task(void *arg)
{
    app_context_t *ctx = (app_context_t *)arg;
    motion_cmd_t cmd;

    while (true) {
        if (xQueueReceive(ctx->motion_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            esp_err_t err = ESP_OK;
            ESP_LOGI(APP_TAG, "motion_task recebeu cmd type=%d axis=%c steps=%d opcode=0x%02X", (int)cmd.type, cmd.axis, (int)cmd.steps, cmd.opcode);
            switch (cmd.type) {
            case MOTION_CMD_MOVE_REL:
                err = do_motion_move_axis(ctx, cmd.axis, cmd.steps);
                break;
            case MOTION_CMD_MOVE_FORCE:
                err = do_motion_move_axis_force(ctx, cmd.axis, cmd.steps);
                break;
            case MOTION_CMD_HOME:
                if (cmd.axis == 'Z') {
                    err = do_motion_home_z(ctx);
                } else {
                    err = do_motion_adjust_axis_to_home(ctx, cmd.axis);
                }
                break;
            }

            ESP_LOGI(APP_TAG, "motion_task cmd type=%d axis=%c finalizado err=%s", (int)cmd.type, cmd.axis, esp_err_to_name(err));

            bool can_online = false;
            if (xSemaphoreTake(ctx->state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                can_online = ctx->state.can_online;
                xSemaphoreGive(ctx->state_mutex);
            }

            if (can_online && cmd.opcode != 0) {
                (void)can_send_event(ctx, (err == ESP_OK) ? CAN_EVT_DONE : CAN_EVT_ERROR, cmd.opcode, (uint8_t)err);
            }
        }
    }
}

esp_err_t motion_init(app_context_t *ctx)
{
    ctx->motion_queue = xQueueCreate(10, sizeof(motion_cmd_t));
    if (ctx->motion_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    BaseType_t created = xTaskCreatePinnedToCore(motion_task, "motion_task", 4096, ctx, 8, NULL, 1);
    if (created != pdPASS) {
        vQueueDelete(ctx->motion_queue);
        ctx->motion_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t enqueue_motion_cmd(app_context_t *ctx, motion_cmd_t *cmd)
{
    if (ctx->motion_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(ctx->motion_queue, cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t motion_post_move_axis(app_context_t *ctx, char axis, int32_t steps, uint8_t sender_id, uint8_t opcode)
{
    motion_cmd_t cmd = {
        .type = MOTION_CMD_MOVE_REL,
        .axis = (char)toupper((unsigned char)axis),
        .steps = steps,
        .sender_node_id = sender_id,
        .opcode = opcode
    };
    return enqueue_motion_cmd(ctx, &cmd);
}

esp_err_t motion_post_move_axis_force(app_context_t *ctx, char axis, int32_t steps, uint8_t sender_id, uint8_t opcode)
{
    motion_cmd_t cmd = {
        .type = MOTION_CMD_MOVE_FORCE,
        .axis = (char)toupper((unsigned char)axis),
        .steps = steps,
        .sender_node_id = sender_id,
        .opcode = opcode
    };
    return enqueue_motion_cmd(ctx, &cmd);
}

esp_err_t motion_post_home_axis(app_context_t *ctx, char axis, uint8_t sender_id, uint8_t opcode)
{
    motion_cmd_t cmd = {
        .type = MOTION_CMD_HOME,
        .axis = (char)toupper((unsigned char)axis),
        .steps = 0,
        .sender_node_id = sender_id,
        .opcode = opcode
    };
    return enqueue_motion_cmd(ctx, &cmd);
}


