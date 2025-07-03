/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-29     Rbb666       first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include "drv_gpio.h"

#define PWM_DEV_NAME    "pwm0"
#define PWM_CHANNEL_MOTOR1  2
#define PWM_CHANNEL_MOTOR2  3
#define MAX_SPEED       1000
#define PERIOD_NS       20000
#define DEFAULT_SPEED_MOTOR1   600
#define DEFAULT_SPEED_MOTOR2   800

/* 红外传感器配置 */
#define IR_SENSOR_PIN   GET_PIN(10, 0)      /* 红外传感器GPIO引脚 */
#define LED_PIN_0       GET_PIN(0, 0)       /* LED灯引脚 P0.0 */
#define LED_PIN_1       GET_PIN(0, 1)       /* LED灯引脚 P0.1 */

/* 障碍物状态 */
#define STATUS_SAFE     0   /* 安全距离 */
#define STATUS_5CM     1

/* 减速停车参数 */
#define DECELERATION_STEP 50  /* 每次减速的步长 */
#define DECELERATION_INTERVAL 50 /* 减速间隔(ms) */

static struct rt_device_pwm *pwm_dev;
static int current_speed1 = DEFAULT_SPEED_MOTOR1;
static int current_speed2 = DEFAULT_SPEED_MOTOR2;
static rt_base_t ir_sensor_pin;
static struct rt_semaphore ir_sem;
static volatile uint8_t obstacle_status = STATUS_SAFE; /* 当前障碍物状态 */
static volatile rt_bool_t is_decelerating = RT_FALSE;  /* 是否正在减速停车 */

/* 红外传感器中断服务函数 */
static void ir_sensor_isr(void *args)
{
    rt_sem_release(&ir_sem);
}

/* LED控制线程 */
static void led_control_thread(void *param)
{
    static uint32_t counter = 0;

    while (1) {
        counter++;

        switch (obstacle_status) {
            case STATUS_5CM:

                rt_pin_write(LED_PIN_0, (counter % 10) < 5 ? PIN_HIGH : PIN_LOW);
                rt_pin_write(LED_PIN_1, (counter % 10) < 5 ? PIN_HIGH : PIN_LOW);
                break;

            case STATUS_SAFE:
            default:
                /* 安全状态：关闭两个LED */
                rt_pin_write(LED_PIN_0, PIN_LOW);
                rt_pin_write(LED_PIN_1, PIN_LOW);
                break;
        }

        /* 每100ms执行一次 */
        rt_thread_mdelay(100);
    }
}

/* 减速停车函数 */
static void decelerate_to_stop(void)
{
    if (is_decelerating) return;

    is_decelerating = RT_TRUE;
    rt_kprintf("[Motor] Starting deceleration to stop...\n");

    // 获取当前速度
    int speed1 = current_speed1;
    int speed2 = current_speed2;

    // 逐步减速直到停止
    while (speed1 > 0 || speed2 > 0) {
        // 减速
        speed1 = (speed1 > DECELERATION_STEP) ? (speed1 - DECELERATION_STEP) : 0;
        speed2 = (speed2 > DECELERATION_STEP) ? (speed2 - DECELERATION_STEP) : 0;

        // 设置PWM输出
        rt_pwm_set(pwm_dev, PWM_CHANNEL_MOTOR1, PERIOD_NS, speed1 * 20);
        rt_pwm_set(pwm_dev, PWM_CHANNEL_MOTOR2, PERIOD_NS, speed2 * 20);

        rt_kprintf("[Motor] Decelerating: %d, %d\n", speed1, speed2);

        // 更新当前速度
        current_speed1 = speed1;
        current_speed2 = speed2;

        // 等待减速间隔
        rt_thread_mdelay(DECELERATION_INTERVAL);
    }

    rt_kprintf("[Motor] Fully stopped\n");
    is_decelerating = RT_FALSE;
}

/* 避障控制线程 */
static void obstacle_avoid_thread(void *param)
{
    rt_uint32_t detection_state;

    while (1) {
        /* 等待红外传感器中断 */
        if (rt_sem_take(&ir_sem, RT_WAITING_FOREVER) == RT_EOK) {
            detection_state = rt_pin_read(ir_sensor_pin);

            if (detection_state == PIN_HIGH) {
                if (obstacle_status != STATUS_5CM) {
                    obstacle_status = STATUS_5CM;
                    rt_kprintf("[IR] Obstacle at 15cm detected! Starting deceleration and blinking LEDs...\n");

                    // 启动减速停车
                    decelerate_to_stop();
                }
            }

                }
            }

        rt_thread_mdelay(10);
    }


static void auto_start_motor(void)
{
    // 分别设置两个电机的初始速度
    pwm_motor_control(DEFAULT_SPEED_MOTOR1, DEFAULT_SPEED_MOTOR2);
    rt_kprintf("[Motor] Auto started at speeds %d and %d\n",
              DEFAULT_SPEED_MOTOR1, DEFAULT_SPEED_MOTOR2);
}

int pwm_motor_init(void) {
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (!pwm_dev) {
        rt_kprintf("[Motor] Error: PWM device not found!\n");
        return -RT_ERROR;
    }

    /* 初始化PWM通道 */
    rt_pwm_set(pwm_dev, PWM_CHANNEL_MOTOR1, PERIOD_NS, 0);
    rt_pwm_enable(pwm_dev, PWM_CHANNEL_MOTOR1);
    rt_pwm_set(pwm_dev, PWM_CHANNEL_MOTOR2, PERIOD_NS, 0);
    rt_pwm_enable(pwm_dev, PWM_CHANNEL_MOTOR2);

    /* 初始化红外传感器 */
    ir_sensor_pin = IR_SENSOR_PIN;
    rt_pin_mode(ir_sensor_pin, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(ir_sensor_pin, PIN_IRQ_MODE_RISING_FALLING, ir_sensor_isr, RT_NULL);
    rt_pin_irq_enable(ir_sensor_pin, PIN_IRQ_ENABLE);

    /* 初始化LED引脚 */
    rt_pin_mode(LED_PIN_0, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_PIN_1, PIN_MODE_OUTPUT);
    rt_pin_write(LED_PIN_0, PIN_LOW);
    rt_pin_write(LED_PIN_1, PIN_LOW);

    /* 创建信号量 */
    rt_sem_init(&ir_sem, "ir_sem", 0, RT_IPC_FLAG_FIFO);

    /* 创建避障线程 */
    rt_thread_t avoid_thread = rt_thread_create("avoid",
                                        obstacle_avoid_thread,
                                        RT_NULL,
                                        1024,
                                        RT_THREAD_PRIORITY_MAX/2,
                                        10);
    if (avoid_thread) {
        rt_thread_startup(avoid_thread);
        rt_kprintf("[IR] Obstacle avoidance thread started\n");
    } else {
        rt_kprintf("[IR] Failed to create avoidance thread!\n");
    }

    /* 创建LED控制线程 */
    rt_thread_t led_thread = rt_thread_create("led_ctrl",
                                        led_control_thread,
                                        RT_NULL,
                                        512,
                                        RT_THREAD_PRIORITY_MAX/3,
                                        10);
    if (led_thread) {
        rt_thread_startup(led_thread);
        rt_kprintf("[LED] LED control thread started\n");
    } else {
        rt_kprintf("[LED] Failed to create LED control thread!\n");
    }

    rt_kprintf("[Motor] PWM & IR initialized (Ch%d, Ch%d)\n",
              PWM_CHANNEL_MOTOR1, PWM_CHANNEL_MOTOR2);

    auto_start_motor();
    return RT_EOK;
}

void pwm_motor_control(int speed1, int speed2) {
    // 手动实现限幅功能
    speed1 = (speed1 > MAX_SPEED) ? MAX_SPEED : ((speed1 < 0) ? 0 : speed1);
    speed2 = (speed2 > MAX_SPEED) ? MAX_SPEED : ((speed2 < 0) ? 0 : speed2);

    // 更新当前速度
    current_speed1 = speed1;
    current_speed2 = speed2;

    // 如果正在减速，取消减速状态
    is_decelerating = RT_FALSE;

    // 如果当前没有障碍物，恢复速度
    if (obstacle_status == STATUS_SAFE) {
        rt_pwm_set(pwm_dev, PWM_CHANNEL_MOTOR1, PERIOD_NS, speed1 * 20);
        rt_pwm_set(pwm_dev, PWM_CHANNEL_MOTOR2, PERIOD_NS, speed2 * 20);
    }

    // 打印设置的速度
    rt_kprintf("[Motor] Speeds set to: %d and %d\n", speed1, speed2);
}

static void motor_test(int argc, char **argv) {
    if (argc < 3) {
        rt_kprintf("Usage: motor_test <speed1 0-%d> <speed2 0-%d>\n",
                  MAX_SPEED, MAX_SPEED);
        return;
    }
    pwm_motor_control(atoi(argv[1]), atoi(argv[2]));
}

MSH_CMD_EXPORT(motor_test, Test motor control);
INIT_APP_EXPORT(pwm_motor_init);
