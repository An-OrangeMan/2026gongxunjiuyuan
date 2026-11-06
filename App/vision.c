#include "vision.h"
#include "control.h"
#include "chassis_yaw.h"
#include "servo.h"
#include "usart.h"
#include "string.h"
#include <math.h>

volatile uint8_t vision_active = 0;

/* ============ 1. CRC16 CCITT-FALSE ============ */
static uint16_t v_crc16(const uint8_t *d, uint16_t n)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < n; i++)
    {
        crc ^= (uint16_t)d[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* ============ 2. 接收解析状态机 ============ */
typedef enum { RX_H0, RX_H1, RX_CMD, RX_SEQ, RX_LEN, RX_DATA, RX_CRC_L, RX_CRC_H } rxst_t;
typedef struct { uint8_t cmd, seq, len, data[16], idx; uint16_t crc; } frame_t;

static frame_t g_w;                      /* 正在拼的帧 */
static frame_t g_r;                      /* 校验通过、待处理的帧 */
static rxst_t  g_st = RX_H0;
static uint32_t g_rx_ms = 0;
static volatile uint8_t g_ready = 0;

void Vision_FeedByte(uint8_t b)
{
    if (HAL_GetTick() - g_rx_ms > 100u) g_st = RX_H0;   /* 半帧超时重找帧头 */
    g_rx_ms = HAL_GetTick();

    switch (g_st)
    {
    case RX_H0:  if (b == V_HEAD0) g_st = RX_H1; break;
    case RX_H1:  g_st = (b == V_HEAD1) ? RX_CMD : (b == V_HEAD0) ? RX_H1 : RX_H0; break;
    case RX_CMD: g_w.cmd = b; g_st = RX_SEQ; break;
    case RX_SEQ: g_w.seq = b; g_st = RX_LEN; break;
    case RX_LEN:
        g_w.len = b; g_w.idx = 0;
        if (b > 16) { g_st = RX_H0; break; }             /* LEN 超限丢弃 */
        g_st = (b == 0) ? RX_CRC_L : RX_DATA;
        break;
    case RX_DATA:
        g_w.data[g_w.idx++] = b;
        if (g_w.idx >= g_w.len) g_st = RX_CRC_L;
        break;
    case RX_CRC_L: g_w.crc = b; g_st = RX_CRC_H; break;
    case RX_CRC_H:
    {
        uint8_t t[19];
        g_w.crc |= (uint16_t)b << 8;
        g_st = RX_H0;
        t[0] = g_w.cmd; t[1] = g_w.seq; t[2] = g_w.len;
        memcpy(&t[3], g_w.data, g_w.len);
        if (v_crc16(t, 3u + g_w.len) == g_w.crc && !g_ready)
        {
            memcpy(&g_r, &g_w, sizeof(frame_t));
            g_ready = 1;
        }
        break;
    }
    }
}

/* ============ 3. 发送 / 反馈 ============ */
static void v_send(uint8_t cmd, uint8_t seq, const uint8_t *data, uint8_t len)
{
    uint8_t buf[32]; uint16_t crc; uint8_t n = 0;
    buf[n++] = V_HEAD0; buf[n++] = V_HEAD1;
    buf[n++] = cmd; buf[n++] = seq; buf[n++] = len;
    for (uint8_t i = 0; i < len; i++) buf[n++] = data[i];
    crc = v_crc16(&buf[2], 3u + len);       /* 覆盖 CMD SEQ LEN DATA */
    buf[n++] = (uint8_t)(crc & 0xFF);       /* 低字节在前 */
    buf[n++] = (uint8_t)(crc >> 8);
    HAL_UART_Transmit(&huart7, buf, n, 100);
}

static void v_result(uint8_t orig_cmd, uint8_t seq, uint8_t status)
{
    uint8_t d[2] = { orig_cmd, status };
    v_send(V_CMD_RESULT, seq, d, 2);
}

/* ============ 4. 动作状态 ============ */
static uint8_t  g_act = 0;          /* 当前连续动作 V_CMD_MOVE/V_CMD_SPIN/0 */
static uint8_t  g_act_seq = 0;
static int8_t   g_act_dir = 0;      /* +1 / -1 */
static uint32_t g_act_fb = 0, g_act_ms = 0;

static uint8_t  g_stop_pend = 0, g_stop_seq = 0; uint32_t g_stop_ms = 0, g_stop_fb = 0;
static uint8_t  g_turn_pend = 0, g_turn_seq = 0; uint32_t g_turn_fb = 0;
static uint8_t  g_grip_pend = 0, g_grip_seq = 0; uint32_t g_grip_ms = 0, g_grip_fb = 0;


static uint8_t  g_team_ok = 0, g_start_ok = 0, g_my_seq = 0;
static uint32_t g_team_last = 0, g_start_last = 0;

static void v_abort_all(void)
{
    g_act = 0; g_act_dir = 0; g_turn_pend = 0;
    for (int i = 0; i < MOTOR_NUM; i++) motor_speed_set[i] = 0;
}

static void v_grip(uint8_t open_close)
{
    servo_target[0] = (open_close == 0x00) ? GRIPPER_OPEN_ANGLE : GRIPPER_CLOSE_ANGLE;
#if GRIPPER_MIRROR
    servo_target[1] = (uint8_t)(SERVO_MAX_ANGLE - servo_target[0]);
#else
    servo_target[1] = servo_target[0];
#endif
}

static void v_dispatch(const frame_t *f)
{
    switch (f->cmd)
    {
    case V_CMD_STOP:
        v_abort_all();
        g_stop_pend = 1; g_stop_seq = f->seq; g_stop_ms = HAL_GetTick(); g_stop_fb = 0;
        break;
    case V_CMD_MOVE:
        if (f->len >= 1) {
            g_act = V_CMD_MOVE; g_act_seq = f->seq;
            g_act_dir = (f->data[0] == 0x01) ? 1 : -1;   
            g_act_ms = HAL_GetTick(); g_act_fb = 0;
            g_turn_pend = 0;
        }
        break;
    case V_CMD_SPIN:
        if (f->len >= 1) {
            g_act = V_CMD_SPIN; g_act_seq = f->seq;
            g_act_dir = (f->data[0] == 0x00) ? 1 : -1;   
            g_act_ms = HAL_GetTick(); g_act_fb = 0;
            g_turn_pend = 0;
        }
        break;
    case V_CMD_TURN:
        if (f->len >= 2) {
            int16_t a = (int16_t)((uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8));
            fp32 deg = (fp32)a * 0.1f;                   
            if (deg >= -180.0f && deg <= 180.0f && deg != 0.0f) {
                CHASSIS_YAW_INPUT_RC = 0;
                if (Chassis_Yaw_SetOffset(V_TURN_SIGN * deg)) {   /* 有 IMU 反馈才接受 */
                    g_act = 0; g_act_dir = 0;
                    g_turn_pend = 1; g_turn_seq = f->seq; g_turn_fb = 0;
                }
            }
        }
        break;
    case V_CMD_GRIP:
        if (f->len >= 1) {
            v_grip(f->data[0]);                          /* 00 开 / 01 闭 */
            g_grip_pend = 1; g_grip_seq = f->seq; g_grip_ms = HAL_GetTick(); g_grip_fb = 0;
        }
        break;
    case V_CMD_RESULT:
        if (f->len >= 2 && f->data[1] == 0x01) {         /* 视觉回"接受" */
            if (f->data[0] == V_CMD_TEAM)  g_team_ok = 1;
            if (f->data[0] == V_CMD_START && g_team_ok) { g_start_ok = 1; vision_active = 1; }
        }
        break;
    }
}

static void v_handshake(void)
{
    uint32_t now = HAL_GetTick();
    if (!g_team_ok) {                                    /* 先发 TEAM，直到视觉接受 */
        if (now - g_team_last > 300u) {
            uint8_t d = V_TEAM_COLOR;
            v_send(V_CMD_TEAM, g_my_seq++, &d, 1);
            g_team_last = now;
        }
    } else if (!g_start_ok) {                            /* 再发 START */
        if (now - g_start_last > 300u) {
            uint8_t d = 0x01;
            v_send(V_CMD_START, g_my_seq++, &d, 1);
            g_start_last = now;
        }
    }
}

static void v_control(void)
{
    fp32 vx = 0, vy = 0, wz = 0, rate[4];

    if (!vision_active) return;


    if (g_act && HAL_GetTick() - g_act_ms > 500u) { g_act = 0; g_act_dir = 0; }

    if (g_act == V_CMD_MOVE) vx = (fp32)g_act_dir * V_MOVE_SPEED;
    else if (g_act == V_CMD_SPIN) wz = (fp32)g_act_dir * V_SPIN_SPEED;

    if (g_turn_pend) wz = Chassis_Yaw_Control(0);   /* 角度闭环，无 rc 输入 */

    chassis_move_forward_decomposition(rate, vx, vy, wz);
    for (int i = 0; i < MOTOR_NUM; i++) motor_speed_set[i] = (int16_t)rate[i];

    Motor_SetEnabled(1);
}

static void v_feedback(void)
{
    uint32_t now = HAL_GetTick();

    if (g_act == V_CMD_MOVE || g_act == V_CMD_SPIN) {    /* 连续：每 100ms 回 0 */
        if (now - g_act_fb >= 100u) { v_result(g_act, g_act_seq, V_STAT_RUNNING); g_act_fb = now; }
    }

    if (g_stop_pend) {                                   /* STOP：300ms 后回完成 */
        if (now - g_stop_ms > 300u) { v_result(V_CMD_STOP, g_stop_seq, V_STAT_DONE); g_stop_pend = 0; }
        else if (now - g_stop_fb >= 100u) { v_result(V_CMD_STOP, g_stop_seq, V_STAT_RUNNING); g_stop_fb = now; }
    }

    if (g_turn_pend) {                                   /* TURN：角度进死区回完成 */
        if (fabsf(chassis_yaw_error_deg) <= CHASSIS_YAW_ANGLE_DEADBAND) {
            v_result(V_CMD_TURN, g_turn_seq, V_STAT_DONE); g_turn_pend = 0;
        } else if (now - g_turn_fb >= 100u) { v_result(V_CMD_TURN, g_turn_seq, V_STAT_RUNNING); g_turn_fb = now; }
    }

    if (g_grip_pend) {                                   /* GRIP：800ms 后回完成 */
        if (now - g_grip_ms > 800u) { v_result(V_CMD_GRIP, g_grip_seq, V_STAT_DONE); g_grip_pend = 0; }
        else if (now - g_grip_fb >= 100u) { v_result(V_CMD_GRIP, g_grip_seq, V_STAT_RUNNING); g_grip_fb = now; }
    }
}

void Vision_task(void const *argument)
{
    (void)argument;
    __HAL_UART_ENABLE_IT(&huart7, UART_IT_RXNE);   

    for (;;)
    {
        frame_t f; uint8_t got = 0;
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (g_ready) { memcpy(&f, &g_r, sizeof(f)); g_ready = 0; got = 1; }
        __set_PRIMASK(mask);

        if (got) v_dispatch(&f);
        v_handshake();
        v_control();
        v_feedback();
        vTaskDelay(5);
    }
}
