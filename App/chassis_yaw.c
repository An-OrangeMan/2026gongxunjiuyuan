#include "chassis_yaw.h"
#include "main.h"
#include <math.h>

fp32 chassis_yaw_target_deg;
fp32 chassis_yaw_error_deg;
fp32 chassis_yaw_rate_dps;
fp32 chassis_yaw_iout;
uint8_t CHASSIS_YAW_INPUT_RC = 1;
static uint8_t target_ready;
static uint8_t rate_ready;
static fp32 previous_yaw;
static uint32_t previous_feedback_ms;
static volatile fp32 feedback_yaw;
static volatile uint32_t feedback_ms;
static volatile uint8_t feedback_ready;
static uint32_t previous_control_ms;
static int16_t previous_rc;

static fp32 yaw_wrap(fp32 angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle >= 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

static uint8_t yaw_read(fp32 *yaw, uint32_t *stamp)
{
    uint32_t mask = __get_PRIMASK();
    uint32_t age;
    uint8_t ready;
    __disable_irq();
    *yaw = feedback_yaw;
    if (stamp != 0) *stamp = feedback_ms;
    age = HAL_GetTick() - feedback_ms;
    ready = feedback_ready;
    __set_PRIMASK(mask);
    return ready && age <= CHASSIS_YAW_FEEDBACK_MS;
}

static void yaw_update_rate(fp32 yaw, uint32_t stamp)
{
    uint32_t elapsed = stamp - previous_feedback_ms;
    if (!rate_ready || elapsed > CHASSIS_YAW_FEEDBACK_MS)
    {
        chassis_yaw_rate_dps = 0.0f;
        rate_ready = 1;
    }
    else if (elapsed == 0u) return; 
    else
    {
        fp32 dt = (fp32)elapsed * 0.001f;
        fp32 rate = yaw_wrap(yaw - previous_yaw) / dt;
        chassis_yaw_rate_dps += dt / (CHASSIS_YAW_RATE_FILTER_S + dt) *
            (rate - chassis_yaw_rate_dps);
    }
    previous_yaw = yaw;
    previous_feedback_ms = stamp;
}

static void yaw_set_target(fp32 target)
{
    chassis_yaw_target_deg = yaw_wrap(target);
    target_ready = 1;
}

void Chassis_Yaw_Init(void)
{
    target_ready = 0;
    rate_ready = 0;
    chassis_yaw_rate_dps = 0.0f;
    chassis_yaw_target_deg = 0.0f;
    chassis_yaw_error_deg = 0.0f;
    chassis_yaw_iout = 0.0f;
    previous_control_ms = HAL_GetTick();
    previous_rc = 0;
}

void Chassis_Yaw_Feedback(fp32 yaw_deg)
{
    if (!isfinite(yaw_deg)) return;
    feedback_yaw = yaw_deg;
    feedback_ms = HAL_GetTick();
    feedback_ready = 1;
}

uint8_t Chassis_Yaw_SetOffset(fp32 offset_deg)
{
    fp32 yaw;
    if (!isfinite(offset_deg) || !yaw_read(&yaw, 0)) return 0;
    yaw_set_target(yaw + yaw_wrap(offset_deg));
    chassis_yaw_iout = 0.0f; 
    return 1;
}

fp32 Chassis_Yaw_Control(int16_t rc_yaw)
{
    fp32 yaw, output, p_error, next_iout, next_output;
    uint32_t stamp;
    uint8_t turning = 0;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - previous_control_ms;
    fp32 dt = elapsed <= 100u ? (fp32)elapsed * 0.001f : 0.0f;
    previous_control_ms = now; // 无反馈时也更新时间，恢复后不补算丢失的时间
    if (!yaw_read(&yaw, &stamp))
    {
        chassis_yaw_error_deg = 0.0f;
        chassis_yaw_rate_dps = 0.0f;
        chassis_yaw_iout = 0.0f;
        rate_ready = 0;
        return 0.0f;
    }
    yaw_update_rate(yaw, stamp);
    
    if (!target_ready) yaw_set_target(yaw);

    if (CHASSIS_YAW_INPUT_RC)
    {
        if (rc_yaw > 660) rc_yaw = 660;
        if (rc_yaw < -660) rc_yaw = -660;
        if (rc_yaw >= -CHASSIS_YAW_RC_DEADBAND && rc_yaw <= CHASSIS_YAW_RC_DEADBAND)
            rc_yaw = 0;
        if (rc_yaw != 0)
        {
            fp32 lead;
            turning = 1;
            if (previous_rc == 0)
            {
                yaw_set_target(yaw);
                chassis_yaw_iout = 0.0f;
            }
            else if ((int32_t)rc_yaw * previous_rc < 0)
                chassis_yaw_iout = 0.0f; 
          
            yaw_set_target(chassis_yaw_target_deg +
                (fp32)rc_yaw / 660.0f * CHASSIS_YAW_RC_RATE_DPS * dt);
           
            lead = yaw_wrap(chassis_yaw_target_deg - yaw);
            if (lead > CHASSIS_YAW_RC_MAX_LEAD)
                yaw_set_target(yaw + CHASSIS_YAW_RC_MAX_LEAD);
            if (lead < -CHASSIS_YAW_RC_MAX_LEAD)
                yaw_set_target(yaw - CHASSIS_YAW_RC_MAX_LEAD);
        }
        else if (previous_rc != 0)
        {
            yaw_set_target(yaw);
            chassis_yaw_iout = 0.0f;
        }
        previous_rc = rc_yaw;
    }

    chassis_yaw_error_deg = yaw_wrap(chassis_yaw_target_deg - yaw);
    p_error = chassis_yaw_error_deg;
    if (!turning)
    {
        if (p_error > CHASSIS_YAW_ANGLE_DEADBAND)
            p_error -= CHASSIS_YAW_ANGLE_DEADBAND;
        else if (p_error < -CHASSIS_YAW_ANGLE_DEADBAND)
            p_error += CHASSIS_YAW_ANGLE_DEADBAND;
        else p_error = 0.0f;
    }
   
    output = CHASSIS_YAW_KP * p_error - CHASSIS_YAW_KD * chassis_yaw_rate_dps;

    if (p_error == 0.0f || p_error * chassis_yaw_iout < 0.0f)
        chassis_yaw_iout = 0.0f;
    next_iout = chassis_yaw_iout + CHASSIS_YAW_KI * p_error * dt;
    if (next_iout > CHASSIS_YAW_MAX_IOUT) next_iout = CHASSIS_YAW_MAX_IOUT;
    if (next_iout < -CHASSIS_YAW_MAX_IOUT) next_iout = -CHASSIS_YAW_MAX_IOUT;
    next_output = output + next_iout;
  
    if (fabsf(next_output) <= CHASSIS_YAW_MAX_WZ ||
        (next_output > CHASSIS_YAW_MAX_WZ && p_error < 0.0f) ||
        (next_output < -CHASSIS_YAW_MAX_WZ && p_error > 0.0f))
        chassis_yaw_iout = next_iout;
    output = CHASSIS_YAW_OUTPUT_SIGN * (output + chassis_yaw_iout);
    if (output > CHASSIS_YAW_MAX_WZ) output = CHASSIS_YAW_MAX_WZ;
    if (output < -CHASSIS_YAW_MAX_WZ) output = -CHASSIS_YAW_MAX_WZ;
    return output;
}
