#include "chassis.h"
#include "chassis_yaw.h"
#include "control.h"
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

volatile int16_t motor_speed_set[4];
volatile uint8_t motor_enabled;
void Motor_SetEnabled(uint8_t enabled)
{
    motor_enabled = enabled;
    if (!enabled) for (int i = 0; i < 4; i++) motor_speed_set[i] = 0;
}
static uint32_t now, irq_mask;
static RC_ctrl_t remote;
#if TEST_YAW_RC
static jmp_buf task_exit;
static unsigned int task_step;
#endif
uint32_t HAL_GetTick(void) { return now; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }
const RC_ctrl_t *get_remote_control_point(void) { return &remote; }

static void near(fp32 actual, fp32 expected)
{
    if (!isfinite(actual) || fabsf(actual - expected) > 0.001f)
    {
        fprintf(stderr, "actual=%f expected=%f\n", actual, expected);
        assert(0);
    }
}

static void start(fp32 yaw)
{
    Chassis_Yaw_Init();
    Chassis_Yaw_Feedback(yaw);
    near(Chassis_Yaw_Control(0), 0);
}

static void common_cases(void)
{
    Chassis_Yaw_Init();
    near(Chassis_Yaw_Control(0), 0);
    assert(!Chassis_Yaw_SetOffset(10)); /* No IMU frame yet. */
    start(30);
    Chassis_Yaw_Feedback(40);
    near(Chassis_Yaw_Control(0), 8 * CHASSIS_YAW_KP);
    near(chassis_yaw_target_deg, 30);
    Chassis_Yaw_Feedback(20);
    near(Chassis_Yaw_Control(0), -8 * CHASSIS_YAW_KP);
    near(chassis_yaw_target_deg, 30);
    Chassis_Yaw_Feedback(31.9f);
    near(Chassis_Yaw_Control(0), 0);
    Chassis_Yaw_Feedback(32.5f);
    near(Chassis_Yaw_Control(0), 0.5f * CHASSIS_YAW_KP);

    start(179);
    assert(Chassis_Yaw_SetOffset(10));
    near(chassis_yaw_target_deg, -171);
    near(Chassis_Yaw_Control(0), -8 * CHASSIS_YAW_KP);
    near(chassis_yaw_error_deg, 10);
    start(-179);
    assert(Chassis_Yaw_SetOffset(-10));
    near(chassis_yaw_target_deg, 171);
    near(Chassis_Yaw_Control(0), 8 * CHASSIS_YAW_KP);
    near(chassis_yaw_error_deg, -10);
    assert(!Chassis_Yaw_SetOffset(NAN));
    assert(!Chassis_Yaw_SetOffset(INFINITY));
    near(chassis_yaw_target_deg, 171);

    now += CHASSIS_YAW_FEEDBACK_MS + 1;
    near(Chassis_Yaw_Control(0), 0);
    assert(!Chassis_Yaw_SetOffset(30));
    Chassis_Yaw_Feedback(165);
    near(Chassis_Yaw_Control(0), -4 * CHASSIS_YAW_KP);
    near(chassis_yaw_target_deg, 171); /* Resume the same target. */
    now = 0xfffffff0u;
    start(30);
    now += 20;
    irq_mask = 1;
    near(Chassis_Yaw_Control(0), 0);
    assert(irq_mask == 1);
    irq_mask = 0;
    puts("PASS: initial lock, external disturbance, deadband, relative targets, wrap, stale feedback and tick rollover");
}

static void damping_cases(void)
{
    start(0);
    assert(Chassis_Yaw_SetOffset(10));
    near(Chassis_Yaw_Control(0), -8 * CHASSIS_YAW_KP);
    now += 100; /* 10 Hz angle feedback, not the 5 ms task period. */
    Chassis_Yaw_Feedback(2);
    near(Chassis_Yaw_Control(0), -6 * CHASSIS_YAW_KP - CHASSIS_YAW_KI * 0.6f + CHASSIS_YAW_KD * 13.333333f);
    near(chassis_yaw_rate_dps, 13.333333f);
    for (int i = 0; i < 19; i++)
    {
        now += 5;
        near(Chassis_Yaw_Control(0), -6 * CHASSIS_YAW_KP - CHASSIS_YAW_KI * (0.6f + (i + 1) * 0.03f) + CHASSIS_YAW_KD * 13.333333f); /* D remains unchanged between frames. */
    }
    assert(Chassis_Yaw_SetOffset(12));
    near(Chassis_Yaw_Control(0), -10 * CHASSIS_YAW_KP + CHASSIS_YAW_KD * 13.333333f); /* Target step changes only P. */
    near(chassis_yaw_rate_dps, 13.333333f);
    now += 5;
    Chassis_Yaw_Feedback(4);
    near(Chassis_Yaw_Control(0), -8 * CHASSIS_YAW_KP - CHASSIS_YAW_KI * 0.04f + CHASSIS_YAW_KD * 17.777778f);

    start(179);
    now += 100;
    Chassis_Yaw_Feedback(-179);
    near(Chassis_Yaw_Control(0), CHASSIS_YAW_KD * 13.333333f); /* Brake inside angle deadband; cross +180. */
    near(chassis_yaw_rate_dps, 13.333333f);
    start(-179);
    now += 100;
    Chassis_Yaw_Feedback(179);
    near(Chassis_Yaw_Control(0), -CHASSIS_YAW_KD * 13.333333f);
    near(chassis_yaw_rate_dps, -13.333333f);

    now += CHASSIS_YAW_FEEDBACK_MS + 1;
    near(Chassis_Yaw_Control(0), 0);
    near(chassis_yaw_rate_dps, 0);
    Chassis_Yaw_Feedback(160);
    Chassis_Yaw_Control(0);
    near(chassis_yaw_rate_dps, 0); /* Reconnect does not differentiate across the gap. */
    near(chassis_yaw_target_deg, -179);

    now = 0xfffffff0u;
    start(0);
    now += 100;
    Chassis_Yaw_Feedback(2);
    near(Chassis_Yaw_Control(0), CHASSIS_YAW_KD * 13.333333f); /* Timestamp rollover also works for D. */
    start(0);
    Chassis_Yaw_Feedback(2.01f);
    near(Chassis_Yaw_Control(0), 0.01f * CHASSIS_YAW_KP); /* No abrupt P jump at deadband edge. */
    puts("PASS: sampled/filtered damping, no target derivative kick, braking in deadband, wrap and recovery");
}

static void integral_cases(void)
{
    start(0);
    assert(Chassis_Yaw_SetOffset(10));
    for (int i = 0; i < 100; i++)
    {
        now += 10;
        Chassis_Yaw_Feedback(0);
        Chassis_Yaw_Control(0);
    }
    near(chassis_yaw_iout, CHASSIS_YAW_KI * 8);
    near(Chassis_Yaw_Control(0), -(8 * CHASSIS_YAW_KP + 8 * CHASSIS_YAW_KI));
    for (int i = 0; i < 1000; i++)
    {
        now += 10;
        Chassis_Yaw_Feedback(0);
        Chassis_Yaw_Control(0);
        assert(fabsf(chassis_yaw_iout) <= CHASSIS_YAW_MAX_IOUT);
    }
    near(chassis_yaw_iout, CHASSIS_YAW_MAX_IOUT);
    Chassis_Yaw_Feedback(10);
    Chassis_Yaw_Control(0);
    near(chassis_yaw_iout, 0); /* Arrival must not keep pushing with old I. */

    start(0);
    assert(Chassis_Yaw_SetOffset(10));
    for (int i = 0; i < 150; i++)
    {
        const uint32_t intervals[] = {2, 7, 11};
        now += intervals[i % 3];
        Chassis_Yaw_Feedback(0);
        Chassis_Yaw_Control(0);
    }
    near(chassis_yaw_iout, CHASSIS_YAW_KI * 8); /* Same one second with task jitter. */
    Chassis_Yaw_Feedback(20);
    Chassis_Yaw_Control(0);
    near(chassis_yaw_iout, 0); /* Reversed error discards the old-direction I. */

    start(0);
    assert(Chassis_Yaw_SetOffset(90));
    for (int i = 0; i < 100; i++)
    {
        now += 10;
        Chassis_Yaw_Feedback(0);
        near(Chassis_Yaw_Control(0), -CHASSIS_YAW_MAX_WZ);
    }
    near(chassis_yaw_iout, 0); /* Saturation must not accumulate further I. */
    assert(Chassis_Yaw_SetOffset(10));
    now += 100;
    Chassis_Yaw_Feedback(0);
    Chassis_Yaw_Control(0);
    assert(chassis_yaw_iout > 0);
    assert(Chassis_Yaw_SetOffset(-10));
    near(chassis_yaw_iout, 0); /* Each new host target clears I. */
    now += 100;
    Chassis_Yaw_Feedback(0);
    Chassis_Yaw_Control(0);
    assert(chassis_yaw_iout < 0);
    now += CHASSIS_YAW_FEEDBACK_MS + 1;
    near(Chassis_Yaw_Control(0), 0);
    near(chassis_yaw_iout, 0);

    now = 0xfffffff0u;
    start(0);
    assert(Chassis_Yaw_SetOffset(10));
    now += 20;
    Chassis_Yaw_Control(0);
    near(chassis_yaw_iout, CHASSIS_YAW_KI * 0.16f);
    now += 200;
    Chassis_Yaw_Feedback(0);
    Chassis_Yaw_Control(0);
    near(chassis_yaw_iout, CHASSIS_YAW_KI * 0.16f); /* No catch-up after task pauses. */
    start(0);
    for (int i = 0; i < 500; i++)
    {
        now += 5;
        Chassis_Yaw_Feedback(0);
        near(Chassis_Yaw_Control(0), 0);
        near(chassis_yaw_iout, 0); /* Centered, unchanged yaw never creates an I command. */
    }
    puts("PASS: timed integral, jitter, I limit, anti-windup, arrival/reversal/target/timeout resets and idle zero");
}

#if TEST_YAW_RC
static void rc_cases(void)
{
    start(30);
    for (int i = 0; i < 200; i++)
    {
        now += 5;
        Chassis_Yaw_Feedback(30);
        Chassis_Yaw_Control(330);
    }
    near(chassis_yaw_target_deg, 105); /* Half stick for one second: +75 degrees. */
    for (int i = 0; i < 200; i++)
    {
        now += 5;
        Chassis_Yaw_Feedback(45);
        Chassis_Yaw_Control(330);
    }
    near(chassis_yaw_target_deg, 135); /* Lead limited to 90 degrees from actual 45. */
    for (int i = 0; i < 150; i++)
    {
        const uint32_t intervals[3] = {4, 7, 9};
        now += intervals[i % 3];
        Chassis_Yaw_Feedback(60);
        Chassis_Yaw_Control(-330);
    }
    near(chassis_yaw_target_deg, 60); /* One second left, independent of jitter. */
    Chassis_Yaw_Feedback(52);
    near(Chassis_Yaw_Control(0), 0);
    near(chassis_yaw_target_deg, 52);
    Chassis_Yaw_Feedback(56);
    near(Chassis_Yaw_Control(20), 2 * CHASSIS_YAW_KP);
    near(Chassis_Yaw_Control(-20), 2 * CHASSIS_YAW_KP);
    near(chassis_yaw_target_deg, 52);
    for (int i = 0; i < 200; i++)
    {
        now += 5;
        Chassis_Yaw_Feedback(56);
        Chassis_Yaw_Control(-330);
    }
    near(chassis_yaw_target_deg, -19); /* Start the new turn at actual yaw 56. */

    start(170);
    for (int i = 0; i < 800; i++)
    {
        fp32 previous = chassis_yaw_target_deg;
        fp32 step;
        now += 10;
        Chassis_Yaw_Feedback(previous);
        Chassis_Yaw_Control(660);
        step = chassis_yaw_target_deg - previous;
        if (step < -180) step += 360;
        near(step, 1.5f);
    }
    assert(fabsf(chassis_yaw_target_deg + 70) < 0.02f);

    start(0);
    for (int i = 0; i < 1000; i++)
    {
        now += 10;
        Chassis_Yaw_Feedback(0); /* Blocked chassis must not reverse at 180. */
        assert(Chassis_Yaw_Control(660) <= 0);
    }
    near(chassis_yaw_target_deg, 90);
    assert(chassis_yaw_iout > 0);
    Chassis_Yaw_Control(0);
    near(chassis_yaw_iout, 0); /* Releasing the stick must clear turn accumulation. */
    near(chassis_yaw_target_deg, 0);
    start(0);
    now += 100;
    Chassis_Yaw_Control(660);
    near(chassis_yaw_target_deg, 15);
    now += 501;
    near(Chassis_Yaw_Control(660), 0);
    now += 5;
    Chassis_Yaw_Feedback(6);
    Chassis_Yaw_Control(660);
    near(chassis_yaw_target_deg, 15.75f); /* No catch-up for missing feedback. */
    now += 200;
    Chassis_Yaw_Feedback(6);
    Chassis_Yaw_Control(660);
    near(chassis_yaw_target_deg, 15.75f); /* No jump after a long task pause. */
    start(0);
    now += 20;
    near(Chassis_Yaw_Control(660), -3 * CHASSIS_YAW_KP - CHASSIS_YAW_KI * 0.06f); /* Active turn inside 2 degrees. */
    now += 20;
    near(Chassis_Yaw_Control(660), -6 * CHASSIS_YAW_KP - CHASSIS_YAW_KI * 0.18f);
    now += 20;
    Chassis_Yaw_Feedback(1.6f);
    assert(Chassis_Yaw_Control(660) < 0); /* Feedback re-entering the old deadband. */
    puts("PASS: held-stick integration, speed scaling, jitter, center hold, continuous wraps, lead limit and recovery");
}

static void wheels(int a, int b, int c, int d)
{
    near(motor_speed_set[0], a);
    near(motor_speed_set[1], b);
    near(motor_speed_set[2], c);
    near(motor_speed_set[3], d);
}

void vTaskDelay(uint32_t ticks)
{
    assert(ticks == CHASSIS_TASK_TIMEOUT);
    now += ticks;
    {
        unsigned int step = task_step++;
        if (step >= 4 && step < 204)
        {
            if (step == 4) assert(motor_speed_set[0] >= 0); /* D brakes preceding motion. */
            if (step == 203)
            {
                near(chassis_yaw_target_deg, 67.72727f); /* Right 100 held 1 s. */
                int speed = (int)fminf(22.72727f * CHASSIS_YAW_KP + chassis_yaw_iout, CHASSIS_YAW_MAX_WZ);
                wheels(-speed, speed, -speed, speed);
                remote.rc.ch[CHASSIS_YAW_CHANNEL] = 0;
                Chassis_Yaw_Feedback(52);
            }
            else Chassis_Yaw_Feedback(45);
            return;
        }
        switch (step)
        {
    case 0:
        wheels(-14, -14, -14, -14); /* Preserve current vx wheel mapping. */
        Chassis_Yaw_Feedback(60);
        break;
    case 1:
        wheels(-6, -22, -6, -22); /* Add yaw correction to forward vx. */
        remote.rc.ch[CHASSIS_X_CHANNEL] = -330;
        Chassis_Yaw_Feedback(30);
        break;
    case 2:
        wheels(13, 15, 13, 15); /* Reverse vx plus yaw damping. */
        remote.rc.ch[CHASSIS_X_CHANNEL] = 0;
        Chassis_Yaw_Feedback(45);
        break;
    case 3:
        wheels(8, -8, 8, -8); /* Hold heading even with all sticks centered. */
        remote.rc.ch[CHASSIS_YAW_CHANNEL] = 100;
        break;
    case 204:
        wheels(2, -2, 2, -2); /* Release clears the integral and retains damping. */
        near(chassis_yaw_target_deg, 52);
        Chassis_Yaw_Feedback(56);
        break;
    case 205:
        {
            int speed = (int)(188.42977f * CHASSIS_YAW_KD + 2 * CHASSIS_YAW_KP + 0.01f * CHASSIS_YAW_KI);
            wheels(speed, -speed, speed, -speed);
        }
        near(chassis_yaw_target_deg, 52);
        longjmp(task_exit, 1);
    default: assert(0);
        }
    }
}

static void task_case(void)
{
    memset(&remote, 0, sizeof(remote));
    remote.rc.ch[CHASSIS_X_CHANNEL] = 330;
    remote.rc.s[1] = 3;
    Chassis_Yaw_Feedback(30);
    task_step = 0;
    if (setjmp(task_exit) == 0) Chassis_task(NULL);
    assert(task_step == 206);
    puts("PASS: actual Chassis_task, preserved wheel formulas, forward/reverse vx, yaw input and release hold");
}

#else
void vTaskDelay(uint32_t ticks) { (void)ticks; }
static void host_cases(void)
{
    start(30);
    near(Chassis_Yaw_Control(660), 0); /* RC cannot change host targets. */
    assert(Chassis_Yaw_SetOffset(10));
    near(chassis_yaw_target_deg, 40);
    Chassis_Yaw_Feedback(35);
    near(Chassis_Yaw_Control(-660), -3 * CHASSIS_YAW_KP);
    near(chassis_yaw_target_deg, 40);
    assert(Chassis_Yaw_SetOffset(-20));
    near(chassis_yaw_target_deg, 15); /* Relative to the newest actual 35. */
    near(Chassis_Yaw_Control(0), fminf(18 * CHASSIS_YAW_KP, CHASSIS_YAW_MAX_WZ));
    Chassis_Yaw_Feedback(15);
    near(Chassis_Yaw_Control(660), 0);
    Chassis_Yaw_Feedback(20);
    near(Chassis_Yaw_Control(0), 3 * CHASSIS_YAW_KP);
    near(chassis_yaw_target_deg, 15);
    puts("PASS: host relative-angle entry and exclusive host control of targets");
}
#endif

int main(void)
{
    CHASSIS_YAW_INPUT_RC = TEST_YAW_RC;
    common_cases();
    damping_cases();
    integral_cases();
#if TEST_YAW_RC
    rc_cases();
    task_case();
#else
    host_cases();
#endif
    return 0;
}
