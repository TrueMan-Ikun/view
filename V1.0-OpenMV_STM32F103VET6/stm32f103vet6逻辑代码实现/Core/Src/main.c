/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "dfplayer.h"
#include "oled.h"
#include "search_control.h"
#include "system_voice.h"
#include "target_mode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
    uint8_t prev_search_mode;
    uint8_t prev_target_valid;
    uint8_t mode_found_prompt_played;
    uint8_t found_candidate_active;
    uint32_t found_candidate_tick;
    uint8_t lost_recovery_active;
    uint8_t lost_search_prompt_played;
    uint32_t mode_switch_search_only_until_tick;
    uint8_t mode_switch_wait_for_first_found;
} VoicePolicyState;

typedef enum
{
    GUIDE_ZONE_NONE = 0,
    GUIDE_ZONE_LEFT,
    GUIDE_ZONE_SLIGHT_LEFT,
    GUIDE_ZONE_FORWARD,
    GUIDE_ZONE_SLIGHT_RIGHT,
    GUIDE_ZONE_RIGHT
} GuideZone_t;

typedef enum
{
    GUIDE_FORWARD_LEVEL_NONE = 0,
    GUIDE_FORWARD_LEVEL_GO,
    GUIDE_FORWARD_LEVEL_CONTINUE,
    GUIDE_FORWARD_LEVEL_NEAR,
    GUIDE_FORWARD_LEVEL_ARRIVED
} GuideForwardLevel_t;

typedef enum
{
    GUIDE_COMMAND_IDLE = 0,
    GUIDE_COMMAND_WAIT_CENTER,
    GUIDE_COMMAND_DONE
} GuideCommandState_t;

typedef enum
{
    TARGET_PRESENCE_LOST = 0,
    TARGET_PRESENCE_CANDIDATE,
    TARGET_PRESENCE_CONFIRMED,
    TARGET_PRESENCE_HOLD
} TargetPresenceState_t;

typedef struct
{
    uint16_t continue_permille;
    uint16_t near_permille;
    uint16_t arrived_permille;
} GuideAreaThresholdConfig_t;

typedef struct
{
    float x;
    float v;
    float p00;
    float p01;
    float p10;
    float p11;
    float q_pos;
    float q_vel;
    float r;
} Kalman1D;

typedef enum
{
    BUZZER_ZONE_NONE = 0,
    BUZZER_ZONE_CENTER,
    BUZZER_ZONE_OUTER,
    BUZZER_ZONE_EDGE
} BuzzerZone_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/*
 * ======== Porting Config ========
 * 这一区域专门放“和硬件、协议、控制参数相关”的配置。
 *
 * 以后如果要修改：
 * 1. OpenMV 分辨率
 * 2. 舵机方向
 * 3. 舵机机械限位
 * 4. 跟随快慢
 * 5. 中心死区
 * 都优先改这里，不要直接去改下面主逻辑。
 *
 * 这样做的目的就是提高可移植性和后续维护性。
 */

/* =========================================================
 * 1. OpenMV 图像范围配置
 * =========================================================
 * 当前 OpenMV 端使用的是 QVGA(320x240)，并发送目标中心点坐标：
 * X: 0~319
 * Y: 0~239
 *
 * 如果以后改成 QQVGA / VGA / 其他分辨率，
 * 只需要改这里即可。
 */
#define IMAGE_MAX_X                 320.0f
#define IMAGE_MAX_Y                 240.0f

#define IMAGE_CENTER_X              (IMAGE_MAX_X * 0.5f)
#define IMAGE_CENTER_Y              (IMAGE_MAX_Y * 0.5f)

/* =========================================================
 * 2. 舵机通道配置
 * =========================================================
 * 当前使用 TIM2 的两个 PWM 通道：
 * X 轴（水平） -> TIM2_CH1
 * Y 轴（俯仰） -> TIM2_CH2
 *
 * 如果以后换定时器或者换引脚，
 * 优先改这里，并同步检查 CubeMX 对应 PWM 配置。
 */
#define SERVO_X_CHANNEL             TIM_CHANNEL_1
#define SERVO_Y_CHANNEL             TIM_CHANNEL_2

/* =========================================================
 * 3. 舵机角度与脉宽配置
 * =========================================================
 * SERVO_FULL_ANGLE_DEG：
 * 普通 180 度舵机通常保持 180.0f 即可。
 *
 * SERVO_PULSE_MIN_US / MAX_US：
 * 按舵机手册修改脉宽范围。
 * 常见值为 1000~2000us，也有些舵机是 500~2500us。
 */
#define SERVO_FULL_ANGLE_DEG        180.0f
#define SERVO_PULSE_MIN_US          1000.0f
#define SERVO_PULSE_MAX_US          2000.0f

/* =========================================================
 * 4. X / Y 两轴的机械安全角度范围
 * =========================================================
 * 这里的目的不是“让舵机能转满 180 度”，
 * 而是避免云台撞到机械极限。
 *
 * 如果后续发现某个方向快撞到结构件，
 * 直接改对应轴的 MIN / MAX 即可。
 */
#define SERVO_X_MIN_ANGLE           20.0f
#define SERVO_X_MAX_ANGLE           180.0f
#define SERVO_X_START_ANGLE         90.0f

#define SERVO_Y_MIN_ANGLE           5.0f
#define SERVO_Y_MAX_ANGLE           175.0f
#define SERVO_Y_START_ANGLE         90.0f

/* =========================================================
 * 5. 卡尔曼坐标滤波参数（仅坐标层）
 * =========================================================
 * 说明：
 * 1. 只用于 OpenMV 坐标去抖，不改搜索、语音、舵机输出层结构
 * 2. X / Y 各自使用一维“位置-速度”卡尔曼
 * 3. dt 优先按有效帧间隔估算，并做上下限保护
 */
#define KALMAN_FIXED_DT_S           0.02f
#define KALMAN_Q_POS_DEFAULT        0.02f
#define KALMAN_Q_VEL_DEFAULT        0.20f
#define KALMAN_R_MEAS_DEFAULT       5.0f   /* 降低：更信任实测坐标，快速目标响应更及时，原值11.0偏保守 */
#define KALMAN_P00_INIT             1.0f
#define KALMAN_P11_INIT             1.0f
#define KALMAN_DT_MIN_S             0.005f
#define KALMAN_DT_MAX_S             0.050f

/* UART2 接收队列参数（防止单缓冲被后续帧覆盖） */
#define UART2_RX_LINE_LEN           100U
#define UART2_RX_QUEUE_LEN          4U

/* OLED 刷新节流，降低 I2C 刷屏带来的主循环负担 */
#define OLED_REFRESH_INTERVAL_MS    120U

/* =========================================================
 * 6. 云台平滑输出参数
 * =========================================================
 * SERVO_SMOOTH_FACTOR：
 * 舵机角度平滑系数，范围 0~1
 * 越小：动作更柔和，但会更慢
 * 越大：动作更跟手，但可能更抖
 *
 * SERVO_DEAD_BAND：
 * 角度死区，单位“度”
 * 当前角度距离目标角度足够近时，直接认为到位，
 * 防止舵机在很小范围内不停修正。
 *
 * SERVO_UPDATE_DELAY_MS：
 * 控制循环更新周期。
 */
#define SERVO_SMOOTH_FACTOR         0.35f  /* 调高：云台跟随更快，原值0.30偏慢 */
#define SERVO_DEAD_BAND             1.0f
#define SERVO_UPDATE_DELAY_MS       8      /* 缩短：提升云台更新频率，原值12ms */


/* =========================================================
 * 7. 丢目标保持时间（预留）
 * =========================================================
 * 作用：
 * OpenMV 可能会出现短暂漏检。
 * 如果一丢目标就立刻停住，云台会显得很生硬。     
 * 当前版本搜索逻辑已改为“目标有效/无效”直接切换，
 * 本参数暂未参与主控制逻辑，保留用于后续扩展
 * 所以这里设置一个“短暂保持时间”。
 */
 //    #define TRACK_HOLD_MS               300U

/* =========================================================
 * 8. X 轴（水平）跟随参数
 * =========================================================
 * X_CENTER_MIN / MAX：
 * 图像中心附近的允许区间。
 * 当目标进入这个范围时，认为“基本居中”，
 * X 轴不再继续修正。
 *
 * X_TRACK_STEP_GAIN_DEG：
 * 每次根据误差修正的角度步进增益。
 * 越大：追得更快
 * 越小：更稳，但更慢
 *
 * X_TRACK_DIRECTION：
 * 如果发现目标在左边，舵机却往反方向转，
 * 只需要把 1.0f 和 -1.0f 对调，不用改下面逻辑。
 */
#define X_CENTER_MIN                150.0f
#define X_CENTER_MAX                170.0f
#define X_TRACK_STEP_GAIN_DEG       5.5f   /* 提高：更积极响应误差，原值4.5 */
#define X_TRACK_DIRECTION           (-1.0f)
#define X_TRACK_STEP_MAX_DEG        4.5f   /* 放开：允许更大单帧修正，原值3.2 */
#define X_CENTER_SLOW_FACTOR        0.7f
#define X_CENTER_STEP_MAX_DEG       0.8f
/* 恢复轻度步进低通平滑：1.0会导致突变过快使舵机机械齿轮回弹抖动，改为0.70起缓冲作用 */
#define X_STEP_SMOOTH_ALPHA         0.70f

/* =========================================================
 * 9. Y 轴（俯仰）跟随参数
 * =========================================================
 * 当前 QVGA 高度是 240，所以中心大约在 120。
 *
 * 如果发现：
 * 目标在画面上方，舵机却继续往下转
 * 或者
 * 目标在画面下方，舵机却继续往上转
 * 那么只改 Y_TRACK_DIRECTION 即可。
 */
#define Y_CENTER_MIN                110.0f
#define Y_CENTER_MAX                130.0f
#define Y_TRACK_STEP_GAIN_DEG       3.8f   /* 提高：更积极响应Y轴误差，原值3.2 */
#define Y_TRACK_DIRECTION           (1.0f)
#define Y_TRACK_STEP_MAX_DEG        1.5f   /* 放开：允许更大单帧修正，原值1.0 */
#define Y_CENTER_SLOW_FACTOR        0.7f
#define Y_CENTER_STEP_MAX_DEG       0.5f
/* 恢复轻度步进低通平滑，同X轴，防机械回弹 */
#define Y_STEP_SMOOTH_ALPHA         0.70f

/* 跟踪锁定：以中心点为基准，静止目标稳定后冻结微调，偏离后再解锁 */
#define TRACK_LOCK_CENTER_X_PIXELS       160.0f
#define TRACK_LOCK_CENTER_Y_PIXELS       120.0f
#define TRACK_LOCK_ENTER_X_ERR_BASE      5.0f
#define TRACK_LOCK_ENTER_Y_ERR_BASE      5.0f
#define TRACK_LOCK_ENTER_X_ERR_FAR       5.0f
#define TRACK_LOCK_ENTER_Y_ERR_FAR       5.0f
#define TRACK_LOCK_FAR_AREA_PERMILLE     12
#define TRACK_LOCK_EXIT_X_ERR_PIXELS    9.0f
#define TRACK_LOCK_EXIT_Y_ERR_PIXELS    8.0f
#define TRACK_LOCK_EXIT_STABLE_MS       180U
#define TRACK_LOCK_EXIT_CONFIRM_FRAMES  2U
#define TRACK_LOCK_STABLE_MS            320U

/* =========================================================
 * 10. 目标模式定义
 * =========================================================
 * 这里统一管理“当前识别哪个目标”。
 * 后续如果要增加类别，优先改这里。
 */

/* 当前总模式数 */

/* =========================================================
 * 11. 五个独立按钮引脚定义
 * =========================================================
 * 当前按键采用“上拉输入”方式：
 * 松开 = 高电平
 * 按下 = 低电平
 *
 * 后续如果换引脚，只改这里即可。
 */
#define KEY_PERSON_PORT             GPIOC
#define KEY_PERSON_PIN              GPIO_PIN_0

#define KEY_BOTTLE_PORT             GPIOC
#define KEY_BOTTLE_PIN              GPIO_PIN_1

#define KEY_PEN_PORT                GPIOE
#define KEY_PEN_PIN                 GPIO_PIN_13

#define KEY_STAIRS_PORT             GPIOE
#define KEY_STAIRS_PIN              GPIO_PIN_3

#define KEY_ELEVATOR_PORT           GPIOE
#define KEY_ELEVATOR_PIN            GPIO_PIN_2

#define KEY_EXTINGUISHER_PORT       GPIOE
#define KEY_EXTINGUISHER_PIN        GPIO_PIN_7

/* PE14 当前固定用作 DFPlayer 测试按键 */
#define KEY_VOICE_PORT              GPIOE
#define KEY_VOICE_PIN               GPIO_PIN_14

#define KEY_NONE_PORT               GPIOE
#define KEY_NONE_PIN                GPIO_PIN_15

/* =========================================================
 * 12. 按键消抖时间
 * =========================================================
 * 单位：ms
 * 如果后续发现按一下会触发两次，可以适当调大。
 */
#define KEY_DEBOUNCE_MS             180U

/* =========================================================
 * 13. DFPlayer 测试版配置
 * =========================================================
 * 当前只做最小可用测试：
 * 1. 上电后只初始化模块，不自动播放
 * 2. 通过 PE14 按键测试 MP3 文件的播放、暂停、继续
 * 3. 当前 0001.mp3 与“厕所模式”语音共用，便于先验证整条语音链路
 *
 * 后续如果要调整：
 * 1. 默认音量
 * 2. 模式与状态播报文件编号
 * 3. 测试按键使用的文件编号
 * 4. 模块上电等待时间
 * 优先修改这里即可。
 */
#define DFPLAYER_DEFAULT_VOLUME          15U
#define DFPLAYER_MODE_TOILET_MP3_INDEX   1U
#define DFPLAYER_MODE_EXIT_MP3_INDEX     2U
#define DFPLAYER_MODE_OFFICE_MP3_INDEX   3U
#define DFPLAYER_MODE_STAIRS_MP3_INDEX   4U
#define DFPLAYER_MODE_ELEVATOR_MP3_INDEX 5U
#define DFPLAYER_MODE_EXTINGUISHER_MP3_INDEX 6U
#define DFPLAYER_SEARCHING_MP3_INDEX     10U
#define DFPLAYER_TARGET_FOUND_MP3_INDEX  11U
#define DFPLAYER_TARGET_LOST_MP3_INDEX   12U
#define DFPLAYER_MODE_NONE_MP3_INDEX     13U
#define DFPLAYER_GUIDE_LEFT_MP3_INDEX        20U
#define DFPLAYER_GUIDE_RIGHT_MP3_INDEX       21U
#define DFPLAYER_GUIDE_SLIGHT_LEFT_MP3_INDEX 22U
#define DFPLAYER_GUIDE_SLIGHT_RIGHT_MP3_INDEX 23U
#define DFPLAYER_GUIDE_FACING_MP3_INDEX      24U
#define DFPLAYER_GUIDE_FORWARD_MP3_INDEX     25U
#define DFPLAYER_GUIDE_CONTINUE_MP3_INDEX    26U
#define DFPLAYER_GUIDE_NEAR_MP3_INDEX        27U
#define DFPLAYER_GUIDE_ARRIVED_MP3_INDEX     28U
#define DFPLAYER_TEST_MP3_INDEX          DFPLAYER_MODE_TOILET_MP3_INDEX
#define DFPLAYER_STARTUP_DELAY_MS   2500U
#define DFPLAYER_INIT_RETRY_DELAY_MS 200U
#define DFPLAYER_INIT_RETRY_COUNT   2U

/* =========================================================
 * DFPlayer 系统语音保护参数
 * =========================================================
 * VOICE_MODE_GUARD_MS   : 模式切换语音后的保护时间
 *                         防止“搜索/发现目标”语音抢播模式语音
 *
 * VOICE_STATUS_GUARD_MS : 状态语音之间的最小间隔
 *                         防止“正在搜索目标”和“已发现目标”连续抢播
 *
 * VOICE_FOUND_CONFIRM_MS: “已发现目标”语音的确认时间
 *                         只有目标连续稳定存在一段时间，才允许播报
 *
 * VOICE_TARGET_PROMPT_DELAY_MS:
 *                         “已发现目标”播出后，等待多久再播目标名称语音
 *                         当前适当留长，保证“已发现目标”能完整播完
 *                         这段时间内云台仍可继续做平滑校准
 *
 * VOICE_LOST_PROMPT_DELAY_MS:
 *                         “目标丢失”播出后的保护时间
 *                         当前按 0007.mp3 约 3 秒时长预留一点余量
 *
 * VOICE_MODE_SWITCH_SEARCH_ONLY_MS:
 *                         按键切换目标后的“只允许寻找”保护时间
 *                         这段时间内不播“已找到”和“目标丢失”
 */
#define VOICE_MODE_GUARD_MS         1800U
#define VOICE_STATUS_GUARD_MS       1200U
#define VOICE_FOUND_CONFIRM_MS      700U
#define VOICE_TARGET_PROMPT_DELAY_MS 3600U
#define VOICE_LOST_PROMPT_DELAY_MS  3200U
#define VOICE_MODE_SWITCH_SEARCH_ONLY_MS 1200U
#define SEARCH_DIRECTION_EPSILON_PIXELS 5.0f

/* =========================================================
 * 13.1 引导语音参数（基于云台角度）
 * =========================================================
 * GUIDE_CENTER_ANGLE_DEG:
 *   用户正前方对应的云台角度中心。
 *
 * GUIDE_OFFSET_FORWARD_DEG:
 *   目标与正前方偏差落在该范围内，提示“前进”。
 *
 * GUIDE_OFFSET_STRONG_DEG:
 *   超过该偏差，提示“左转/右转”；否则提示“微左/微右”。
 *
 * GUIDE_ZONE_STABLE_MS:
 *   方向分区稳定持续时间，避免边界抖动时频繁触发。
 *
 * GUIDE_BLOCK_AFTER_FOUND_MS:
 *   “已发现目标+目标名称”阶段完成前，暂不播引导语音。
 *
 * GUIDE_FORWARD_STAGE_*_MS:
 *   进入“正对目标”后，按时序触发“请向前走/继续向前/已接近目标/已到达目标附近”。
 */
#define GUIDE_CENTER_ANGLE_DEG           SERVO_X_START_ANGLE
#define GUIDE_OFFSET_FORWARD_DEG         12.0f
#define GUIDE_OFFSET_FORWARD_EXIT_DEG    16.0f
#define GUIDE_OFFSET_STRONG_DEG          28.0f
#define GUIDE_ZONE_STABLE_MS             450U
#define GUIDE_BLOCK_AFTER_FOUND_MS       (VOICE_TARGET_PROMPT_DELAY_MS + VOICE_STATUS_GUARD_MS)
#define GUIDE_DISTANCE_LEVEL_STABLE_MS   500U
#define GUIDE_AREA_FILTER_FACTOR         0.35f
#define GUIDE_RATIO_FILTER_FACTOR        0.30f
#define GUIDE_AREA_REF_MIN_PERMILLE      2U
#define GUIDE_REF_COLLECT_MS             800U
#define GUIDE_REF_MIN_SAMPLES            6U
#define GUIDE_AREA_CONTINUE_PERMILLE     10U
#define GUIDE_AREA_NEAR_PERMILLE         18U
#define GUIDE_AREA_ARRIVED_PERMILLE      28U
#define GUIDE_RATIO_CONTINUE_ENTER_PM    1400U
#define GUIDE_RATIO_CONTINUE_EXIT_PM     1250U
#define GUIDE_RATIO_NEAR_ENTER_PM        2200U
#define GUIDE_RATIO_NEAR_EXIT_PM         1950U
#define GUIDE_RATIO_ARRIVED_ENTER_PM     3000U
#define GUIDE_RATIO_ARRIVED_EXIT_PM      2700U

/* =========================================================
 * 14. 搜索模式参数（主动寻找目标）
 * =========================================================
 * 作用：
 * 当系统当前没有检测到目标时，
 * 云台自动执行左右扫描，以扩大视野范围，
 * 从而重新获取目标。
 *
 * 扫描方式：
 * 中心 → 左 → 右 → 左（往返扫描）
 *
 * SEARCH_X_LEFT_ANGLE / RIGHT：
 * 控制扫描的左右范围（不要太大，避免撞结构）
 *
 * SEARCH_STEP_DEG：
 * 每次扫描步进角度（越大越快，越小越平滑）
 *
 * SEARCH_UPDATE_MS：
 * 扫描更新周期（影响扫描速度）
 *
 * SEARCH_START_DELAY_MS：
 * 切换目标后，先回中等待一段时间再开始扫描，
 * 防止刚切换时画面还没稳定就开始乱扫。
 */
#define SEARCH_X_LEFT_ANGLE         20.0f
#define SEARCH_X_RIGHT_ANGLE        180.0f
#define SEARCH_Y_FIXED_ANGLE        90.0f
/* 将搜索更新周期与主循环(8ms)严格对齐，消除"快慢停"错峰步级跳频造成的卡顿感。步长等比例缩放(1.2*8/25 ≈ 0.4) */
#define SEARCH_STEP_DEG             0.40f
#define SEARCH_UPDATE_MS            8U
#define SEARCH_START_DELAY_MS       800U

/* =========================================================
 * 15. 自适应主动重捕获参数
 * =========================================================
 * 说明：
 * 1. 仅在“已经稳定跟踪过目标、随后又真正丢失”时启用
 * 2. 阶段 1 为局部快速搜索，阶段 2 为全范围扩展搜索
 * 3. 按键切换后的普通搜索仍沿用上面的基础搜索参数
 */

#define SEARCH_PHASE1_BASE_AMPLITUDE_DEG    10.0f
#define SEARCH_PHASE1_ERROR_GAIN_DEG        10.0f
#define SEARCH_PHASE1_TIME_GAIN_DEG_PER_S   4.0f
#define SEARCH_PHASE1_MIN_AMPLITUDE_DEG     8.0f
#define SEARCH_PHASE1_MAX_AMPLITUDE_DEG     26.0f
/* 同步缩放局部搜索的速度，更新周期18->8，步长1.8->0.8 */
#define SEARCH_PHASE1_STEP_DEG              0.80f
#define SEARCH_PHASE1_UPDATE_MS             8U
#define SEARCH_PHASE1_TIMEOUT_MS            1800U

#define SEARCH_PHASE2_STEP_BASE_DEG         1.2f
#define SEARCH_PHASE2_STEP_GAIN_DEG_PER_S   0.4f
#define SEARCH_PHASE2_STEP_MIN_DEG          1.2f
#define SEARCH_PHASE2_STEP_MAX_DEG          2.4f
#define SEARCH_PHASE2_LEFT_ANGLE            SEARCH_X_LEFT_ANGLE
#define SEARCH_PHASE2_RIGHT_ANGLE           SEARCH_X_RIGHT_ANGLE

/* =========================================================
 * 16. 目标存在性状态机参数（STM32 侧）
 * =========================================================
 * 状态：
 * LOST -> CANDIDATE -> CONFIRMED -> HOLD
 *
 * 说明：
 * 1. OpenMV 仍按原协议发送 X/Y/A 或 -1/-1
 * 2. STM32 通过状态机决定 target_valid，避免单帧抖动触发丢失
 * 3. 锁定状态下（target_track_locked=1）HOLD 保护时间更长
 */
#define TARGET_CANDIDATE_ENTER_FRAMES      2U
#define TARGET_CONFIRM_ENTER_FRAMES        3U   /* 降低：减少首次确认延迟约1帧，原值4U */
#define TARGET_CANDIDATE_MAX_MISS_FRAMES   1U
#define TARGET_HOLD_MAX_MISS_FRAMES_NORMAL 6U
#define TARGET_HOLD_MS_NORMAL              550U
#define TARGET_HOLD_MS_LOCKED              1100U
#define TARGET_CONFIRM_MIN_AREA_PERMILLE   2U

#define BUZZER_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_ON() HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET)
#define BUZZER_OFF() HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET)

#define BUZZER_RADAR_ON_CENTER  8U
#define BUZZER_RADAR_OFF_CENTER 150U
#define BUZZER_RADAR_ON_OUTER   10U
#define BUZZER_RADAR_OFF_OUTER  600U
#define BUZZER_RADAR_ON_EDGE    10U
#define BUZZER_RADAR_OFF_EDGE   1500U

#define GUIDE_REPROMPT_INTERVAL_MS 2500U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

    /* =========================================================
     * 语音策略状态（业务层）
     * =========================================================
     * 说明：
     * 1. 业务层只负责“什么时候应该提示”
     * 2. system_voice 层只负责“何时能播、如何排队与去重”
     * 3. 所有与“首次发现/模式切换保护/丢失恢复”相关的判定状态统一收口在这里
     */
    VoicePolicyState g_voice_policy_state = {0};
    uint32_t guide_enable_after_tick = 0;
    GuideZone_t guide_last_zone = GUIDE_ZONE_NONE;
    GuideZone_t guide_candidate_zone = GUIDE_ZONE_NONE;
    uint32_t guide_zone_candidate_tick = 0;
    uint32_t guide_forward_enter_tick = 0;
    uint8_t guide_prompt_facing_played = 0;
    GuideForwardLevel_t guide_forward_level_played = GUIDE_FORWARD_LEVEL_NONE;
    GuideForwardLevel_t guide_forward_level_candidate = GUIDE_FORWARD_LEVEL_NONE;
    uint32_t guide_forward_level_candidate_tick = 0U;
    GuideForwardLevel_t guide_forward_level_ratio_state = GUIDE_FORWARD_LEVEL_GO;
    int32_t guide_target_area_permille = -1;
    float guide_filtered_area_permille = 0.0f;
    uint8_t guide_area_ref_valid = 0U;
    float guide_area_ref_permille = 0.0f;
    float guide_ratio_filtered_permille = 1000.0f;
    uint32_t guide_area_ref_collect_start_tick = 0U;
    float guide_area_ref_collect_sum = 0.0f;
    uint16_t guide_area_ref_collect_samples = 0U;
    GuideCommandState_t guide_command_state = GUIDE_COMMAND_IDLE;
    uint32_t guide_last_voice_tick = 0U;

    float last_x_error = 0;
    
    uint8_t rx_data;                                      // USART2 每次接收字节
    char rx_line_build[UART2_RX_LINE_LEN];               // 中断侧当前拼接中的一帧
    char rx_buffer[UART2_RX_LINE_LEN];                   // 主循环侧取出的完整帧
    uint8_t rx_index = 0;                                // 中断侧当前接收位置
    char tx_buffer[100];                                 // OLED 打印用
    volatile uint8_t uart2_queue_write_idx = 0U;
    volatile uint8_t uart2_queue_read_idx = 0U;
    volatile uint8_t uart2_queue_count = 0U;
    char uart2_line_queue[UART2_RX_QUEUE_LEN][UART2_RX_LINE_LEN];

    uint32_t oled_last_refresh_tick = 0U;
    int32_t oled_last_disp_x = -32768;
    int32_t oled_last_disp_y = -32768;
    int32_t oled_last_disp_a = -32768;

    /* 蜂鸣器状态机变量 */
    BuzzerZone_t buzzer_last_zone = BUZZER_ZONE_NONE;
    uint8_t buzzer_is_on = 0U;
    uint32_t buzzer_state_tick = 0U;

    SearchControlConfig g_search_control_cfg =
    {
        .left_limit = SEARCH_X_LEFT_ANGLE,
        .right_limit = SEARCH_X_RIGHT_ANGLE,
        .fixed_y_angle = SEARCH_Y_FIXED_ANGLE,
        .normal_step_deg = SEARCH_STEP_DEG,
        .normal_update_ms = SEARCH_UPDATE_MS,
        .start_delay_ms = SEARCH_START_DELAY_MS,
        .direction_epsilon_pixels = SEARCH_DIRECTION_EPSILON_PIXELS,
        .phase1_base_amplitude_deg = SEARCH_PHASE1_BASE_AMPLITUDE_DEG,
        .phase1_error_gain_deg = SEARCH_PHASE1_ERROR_GAIN_DEG,
        .phase1_time_gain_deg_per_s = SEARCH_PHASE1_TIME_GAIN_DEG_PER_S,
        .phase1_min_amplitude_deg = SEARCH_PHASE1_MIN_AMPLITUDE_DEG,
        .phase1_max_amplitude_deg = SEARCH_PHASE1_MAX_AMPLITUDE_DEG,
        .phase1_step_deg = SEARCH_PHASE1_STEP_DEG,
        .phase1_update_ms = SEARCH_PHASE1_UPDATE_MS,
        .phase1_timeout_ms = SEARCH_PHASE1_TIMEOUT_MS,
        .phase2_step_base_deg = SEARCH_PHASE2_STEP_BASE_DEG,
        .phase2_step_gain_deg_per_s = SEARCH_PHASE2_STEP_GAIN_DEG_PER_S,
        .phase2_step_min_deg = SEARCH_PHASE2_STEP_MIN_DEG,
        .phase2_step_max_deg = SEARCH_PHASE2_STEP_MAX_DEG,
        .phase2_left_angle = SEARCH_PHASE2_LEFT_ANGLE,
        .phase2_right_angle = SEARCH_PHASE2_RIGHT_ANGLE
    };

    SearchControlState g_search_control_state;

    SystemVoiceConfig g_system_voice_cfg =
    {
        .target_prompt_indices =
        {
            DFPLAYER_MODE_TOILET_MP3_INDEX,
            DFPLAYER_MODE_EXIT_MP3_INDEX,
            DFPLAYER_MODE_OFFICE_MP3_INDEX,
            DFPLAYER_MODE_STAIRS_MP3_INDEX,
            DFPLAYER_MODE_ELEVATOR_MP3_INDEX,
            DFPLAYER_MODE_EXTINGUISHER_MP3_INDEX,
            DFPLAYER_MODE_NONE_MP3_INDEX
        },
        .searching_prompt_index = DFPLAYER_SEARCHING_MP3_INDEX,
        .target_found_prompt_index = DFPLAYER_TARGET_FOUND_MP3_INDEX,
        .target_lost_prompt_index = DFPLAYER_TARGET_LOST_MP3_INDEX,
        .mode_none_prompt_index = DFPLAYER_MODE_NONE_MP3_INDEX,
        .guide_left_prompt_index = DFPLAYER_GUIDE_LEFT_MP3_INDEX,
        .guide_right_prompt_index = DFPLAYER_GUIDE_RIGHT_MP3_INDEX,
        .guide_slight_left_prompt_index = DFPLAYER_GUIDE_SLIGHT_LEFT_MP3_INDEX,
        .guide_slight_right_prompt_index = DFPLAYER_GUIDE_SLIGHT_RIGHT_MP3_INDEX,
        .guide_facing_prompt_index = DFPLAYER_GUIDE_FACING_MP3_INDEX,
        .guide_forward_prompt_index = DFPLAYER_GUIDE_FORWARD_MP3_INDEX,
        .guide_continue_forward_prompt_index = DFPLAYER_GUIDE_CONTINUE_MP3_INDEX,
        .guide_near_target_prompt_index = DFPLAYER_GUIDE_NEAR_MP3_INDEX,
        .guide_arrived_prompt_index = DFPLAYER_GUIDE_ARRIVED_MP3_INDEX,
        .test_prompt_index = DFPLAYER_TEST_MP3_INDEX,
        .mode_guard_ms = VOICE_MODE_GUARD_MS,
        .status_guard_ms = VOICE_STATUS_GUARD_MS,
        .target_prompt_delay_ms = VOICE_TARGET_PROMPT_DELAY_MS,
        .lost_prompt_guard_ms = VOICE_LOST_PROMPT_DELAY_MS
    };

    SystemVoiceState g_system_voice_state;

		/* =========================================================
 * 目标模式相关变量
 * =========================================================
 * current_target_mode：当前识别模式
 * key_last_tick      ：上一次按键触发时间，用于消抖
 */
uint8_t current_target_mode = TARGET_MODE_TOILET;
uint32_t key_last_tick = 0;

/* =========================================================
 * 目标名称表
 * =========================================================
 * 这里主要用于 OLED 显示。
 * 顺序必须和 TARGET_MODE_xxx 保持一致。
 */
const char *target_mode_name[TARGET_MODE_COUNT] =
{
    "Toilet",
    "Exit",
    "Office",
    "Stairs",
    "Elevator",
    "Exting.",
    "None"
};

/*
 * 目标类别面积阈值表（单位：千分比）
 * 作用：
 * 1. 用目标框面积判断“继续向前 / 已接近目标 / 已到达目标附近”
 * 2. 支持按类别独立调参，后续你可根据实测直接改这一张表
 */
const GuideAreaThresholdConfig_t guide_area_threshold_table[TARGET_MODE_COUNT] =
{
    /* toilet */      {10U, 18U, 28U},
    /* exit */        {10U, 18U, 28U},
    /* office */      {10U, 18U, 28U},
    /* stairs */      {10U, 18U, 28U},
    /* elevator */    {10U, 18U, 28U},
    /* extinguisher*/ {10U, 18U, 28U},
    /* none */        {0U, 0U, 0U}
};
		
 /* =========================================================
  * 3. X 轴控制变量
  * =========================================================
  * filtered_x    : 滤波后的 X 坐标
   * x_error       : X 偏差（相对图像中心）
   * current_angle_x : 当前实际输出角度
   * target_angle_x  : 目标角度
   */
  float filtered_x = IMAGE_CENTER_X;
  float x_error = 0.0f;
  float current_angle_x = SERVO_X_START_ANGLE;
  float target_angle_x  = SERVO_X_START_ANGLE;
  float smooth_step_x = 0.0f;
  float smooth_step_y = 0.0f;
  Kalman1D kf_x;

  /* =========================================================
  * 4. Y 轴控制变量
  * ========================================================= */
  float filtered_y = IMAGE_CENTER_Y;
  float y_error = 0.0f;
  float current_angle_y = SERVO_Y_START_ANGLE;
  float target_angle_y  = SERVO_Y_START_ANGLE;
  Kalman1D kf_y;
  uint32_t kalman_last_update_tick = 0U;

  /* =========================================================
   * 5. 目标状态变量
   * =========================================================
   * last_target_tick : 最近一次收到有效目标的时间
   * target_valid     : 当前是否处于“检测到目标”的状态
   */
  uint32_t last_target_tick = 0;
  uint8_t target_valid = 0;
  uint8_t target_track_locked = 0U;
  uint32_t target_track_lock_candidate_tick = 0U;
  uint32_t target_track_unlock_candidate_tick = 0U;
  uint8_t target_track_unlock_candidate_frames = 0U;
  TargetPresenceState_t target_presence_state = TARGET_PRESENCE_LOST;
  uint8_t target_presence_hit_count = 0U;
  uint8_t target_presence_miss_count = 0U;
  uint32_t target_presence_hold_start_tick = 0U;
  uint8_t target_presence_hold_locked_snapshot = 0U;
 




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, float angle);
void Kalman1D_Init(Kalman1D *kf,
                   float init_pos,
                   float init_vel,
                   float q_pos,
                   float q_vel,
                   float r);
void Kalman1D_Predict(Kalman1D *kf, float dt_s);
float Kalman1D_Update(Kalman1D *kf, float measurement);
void Reset_TargetCoordinateFilters(void);
void Reset_TargetPresence_State(void);
void Update_TargetPresence_State(uint8_t frame_has_target, int32_t area_permille, uint32_t now_tick);
uint8_t Is_TargetPresence_Valid(void);

/* 用于做浮点数限幅，增强代码可读性和可移植性 */
float Limit_Float(float value, float min, float max);
uint8_t Parse_OpenMV_Message(const char *msg, int *x, int *y, int *area_permille);
uint8_t Try_Dequeue_OpenMV_Line(char *dst, uint16_t dst_len);
uint8_t Should_Refresh_OLED(uint32_t now_tick,
                            int32_t disp_x,
                            int32_t disp_y,
                            int32_t disp_a,
                            uint8_t force_refresh);

void Send_TargetMode_To_OpenMV(uint8_t mode);
void Update_TargetMode_Display(uint8_t mode);
void Handle_Buzzer_Update(void);
void Handle_GuideVoice_Update(void);
uint8_t Search_IsActive_FromModule(void);
uint8_t Search_IsLocalRecapture_FromModule(void);
void Start_NormalSearch_ByModule(uint32_t now_tick);
void Start_Recapture_ByModule(uint32_t now_tick);
void Stop_Search_ByModule(void);
void Reset_Search_ByModule(void);
GuideZone_t Get_GuideZone_FromPanAngle(float pan_angle_deg);
SystemVoiceEvent_t Get_GuideEvent_FromZone(GuideZone_t zone);
GuideForwardLevel_t Get_GuideForwardLevel_FromAreaByMode(uint8_t mode, int32_t area_permille);
GuideForwardLevel_t Get_GuideForwardLevel_FromRatio(float ratio_permille,
                                                    GuideForwardLevel_t prev_level);
SystemVoiceEvent_t Get_GuideForwardEvent_FromLevel(GuideForwardLevel_t level);
void Reset_GuideVoice_State(void);
void Handle_GuideVoice_Update(void);
void Reset_SystemVoice_State(uint8_t clear_found_prompt_flag);
void Handle_SystemVoice_Update(void);
void Handle_Voice_Test_Key(void);
void Key_Scan_And_Handle(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

/* =========================================================
   * 1. 启动两个 PWM 通道
   * =========================================================
   * X 轴：TIM2_CH1
   * Y 轴：TIM2_CH2
   *
   * 注意：
   * 这里只是启动 PWM 输出，
   * 前提是 CubeMX 中已经把对应通道配置为 PWM 模式。
   */
  HAL_TIM_PWM_Start(&htim2, SERVO_X_CHANNEL);
  HAL_TIM_PWM_Start(&htim2, SERVO_Y_CHANNEL);

  /* 让两个舵机先回到初始角度，防止上电后位置随机 */
  Servo_SetAngle(&htim2, SERVO_X_CHANNEL, current_angle_x);
  Servo_SetAngle(&htim2, SERVO_Y_CHANNEL, SERVO_Y_START_ANGLE);

  /* OLED 初始化 */
  OLED_Init();
  OLED_Refresh();

  /* 开启 USART2 接收中断，每次接收 1 字节 */
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);

  OLED_Clear();
  OLED_ShowString(0, 0, "N:");
  OLED_Refresh();
  /* =========================================================
   * DFPlayer 测试模块初始化
   * =========================================================
   * 1. 绑定 DFPlayer 使用的串口
   * 2. 等待模块和 TF 卡稳定上电
   * 3. 设置默认音量，但不自动播放
   */
  DFPlayer_Init(&huart1);
  HAL_Delay(DFPLAYER_STARTUP_DELAY_MS);
  for (uint8_t i = 0; i < DFPLAYER_INIT_RETRY_COUNT; i++)
  {
      DFPlayer_SetVolume(DFPLAYER_DEFAULT_VOLUME);
      HAL_Delay(DFPLAYER_INIT_RETRY_DELAY_MS);
  }

  SystemVoice_Init(&g_system_voice_state, &g_system_voice_cfg);
  Reset_TargetCoordinateFilters();
  Reset_TargetPresence_State();

  /* =========================================================
   * 2. 目标坐标变量
   * ========================================================= */
  int x = 0, y = 0;                     // 当前解析出的目标坐标
  int area_permille = -1;               // 目标框面积（千分比），-1 表示无效

 
  /* =========================================================
   * 6. 按键目标选择功能初始化
   * =========================================================
   * 上电默认先选择“厕所”
   * 并把当前模式显示到 OLED，同时发给 OpenMV。
   */
  Update_TargetMode_Display(current_target_mode);
  Send_TargetMode_To_OpenMV(current_target_mode);
	
  /* 切换目标后：云台回中，并进入搜索准备状态 */
  target_angle_x = current_angle_x;
  target_angle_y = SERVO_Y_START_ANGLE;
  Start_NormalSearch_ByModule(HAL_GetTick());

  /* 同步系统语音状态，避免上电后误触发搜索/发现播报 */
  g_voice_policy_state.prev_search_mode = Search_IsActive_FromModule();
  g_voice_policy_state.prev_target_valid = target_valid;
  g_voice_policy_state.mode_found_prompt_played = 0U;
  g_voice_policy_state.found_candidate_active = 0U;
  g_voice_policy_state.found_candidate_tick = 0U;
  g_voice_policy_state.lost_search_prompt_played = 0U;
  Reset_GuideVoice_State();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
  {
		    /* =========================================================
     * 0. 按键扫描
     * =========================================================
     * 每次循环先检测按钮是否被按下。
     * 如果按下，则切换当前识别目标，并通知 OpenMV。
     */
    Key_Scan_And_Handle();
		
    /* =========================================================
     * 1. 串口接收完成后，解析 OpenMV 发来的坐标
     * =========================================================
     * OpenMV 发送格式：
     * X:123,Y:45
     *
     * 如果没找到目标，则发送：
     * X:-1,Y:-1
     */
    {
        uint8_t has_latest_line = 0U;
        while (Try_Dequeue_OpenMV_Line(rx_buffer, (uint16_t)sizeof(rx_buffer)))
        {
            has_latest_line = 1U;
        }

        if (has_latest_line)
    {
        uint8_t parse_count = 0U;
        parse_count = Parse_OpenMV_Message(rx_buffer, &x, &y, &area_permille);

        if (parse_count >= 2)
        {
      /*
       * None 模式保护：
       * 1. 用户主动切到 None 后，主控不再根据 OpenMV 数据触发“丢失/发现”语音
       * 2. 防止 OpenMV 持续发送 X:-1,Y:-1 时累计触发“目标丢失”
       */
      if (current_target_mode == TARGET_MODE_NONE)
      {
          target_valid = 0;
          Reset_TargetPresence_State();
          guide_target_area_permille = -1;
          guide_filtered_area_permille = 0.0f;
      }
      else
      {
          uint32_t now_tick = HAL_GetTick();
          uint8_t frame_has_target = ((x != -1) && (y != -1)) ? 1U : 0U;
          uint8_t prev_target_valid = target_valid;

          /* 1) 先更新面积信息（供状态机与引导层使用） */
          if (frame_has_target && (area_permille >= 0))
          {
              if (guide_target_area_permille < 0)
              {
                  guide_filtered_area_permille = (float)area_permille;
              }
              else
              {
                  guide_filtered_area_permille =
                      (guide_filtered_area_permille * (1.0f - GUIDE_AREA_FILTER_FACTOR)) +
                      ((float)area_permille * GUIDE_AREA_FILTER_FACTOR);
              }

              guide_target_area_permille = (int32_t)(guide_filtered_area_permille + 0.5f);
          }
          else
          {
              guide_target_area_permille = -1;
              guide_filtered_area_permille = 0.0f;
          }

          /* 2) 目标存在性状态机决定 target_valid（而不是单帧直接决定） */
          Update_TargetPresence_State(frame_has_target, guide_target_area_permille, now_tick);
          target_valid = Is_TargetPresence_Valid();

          if ((!prev_target_valid) && target_valid)
          {
              /* 首次确认目标（或从 HOLD 快速恢复） */
              last_target_tick = now_tick;
              Stop_Search_ByModule();
          }
          else if (target_valid && frame_has_target)
          {
              /* 已确认目标且本帧有效，保持跟踪态 */
              last_target_tick = now_tick;
              Stop_Search_ByModule();
          }
          else if (prev_target_valid && (!target_valid))
          {
              /* 状态机判定“真的丢失”后才触发语音与搜索切换 */
              uint32_t lost_tick = now_tick;
              uint8_t mode_switch_guard_active =
                  ((int32_t)(g_voice_policy_state.mode_switch_search_only_until_tick - lost_tick) > 0) ? 1U : 0U;
              uint8_t mode_switch_pending_search =
                  g_voice_policy_state.mode_switch_wait_for_first_found ? 1U : 0U;

              g_voice_policy_state.found_candidate_active = 0U;
              g_voice_policy_state.found_candidate_tick = 0U;

              if (mode_switch_guard_active || mode_switch_pending_search)
              {
                  g_voice_policy_state.lost_recovery_active = 0U;
                  Start_NormalSearch_ByModule(lost_tick);
              }
              else if (!g_voice_policy_state.lost_recovery_active)
              {
                  g_voice_policy_state.lost_recovery_active = 1U;
                  SystemVoice_PostEvent(&g_system_voice_state,
                                        VOICE_EVT_TARGET_LOST,
                                        (TargetMode_t)current_target_mode,
                                        lost_tick);
                  guide_enable_after_tick = lost_tick + VOICE_LOST_PROMPT_DELAY_MS;
                  Start_Recapture_ByModule(lost_tick);
              }
          }

          /*
           * 3) 控制链只在“状态机确认有效 + 本帧有效”时运行
           *    HOLD 期间不更新新测量，等待短时恢复
           */
          if (frame_has_target && target_valid)
          {
          float kalman_dt_s = KALMAN_FIXED_DT_S;
          float step_x = 0.0f;
          float raw_step_x = 0.0f;
          float step_y = 0.0f;
          float lock_enter_x_thresh = TRACK_LOCK_ENTER_X_ERR_BASE;
          float lock_enter_y_thresh = TRACK_LOCK_ENTER_Y_ERR_BASE;
          float lock_x_error = 0.0f;
          float lock_y_error = 0.0f;
          float abs_lock_x_error = 0.0f;
          float abs_lock_y_error = 0.0f;
          uint8_t in_final_tune_zone = 0U;

          if (kalman_last_update_tick != 0U)
          {
              kalman_dt_s = (float)(now_tick - kalman_last_update_tick) / 1000.0f;
              kalman_dt_s = Limit_Float(kalman_dt_s, KALMAN_DT_MIN_S, KALMAN_DT_MAX_S);
          }
          kalman_last_update_tick = now_tick;

          /* ===== X轴控制（有效帧每帧都更新） ===== */
          Kalman1D_Predict(&kf_x, kalman_dt_s);
          filtered_x = Kalman1D_Update(&kf_x, (float)x);

          x_error = filtered_x - IMAGE_CENTER_X;
          last_x_error = x_error;

          /* ===== Y轴控制（有效帧每帧都更新） ===== */
          Kalman1D_Predict(&kf_y, kalman_dt_s);
          filtered_y = Kalman1D_Update(&kf_y, (float)y);

          y_error = filtered_y - IMAGE_CENTER_Y;

          in_final_tune_zone = ((filtered_x >= X_CENTER_MIN) && (filtered_x <= X_CENTER_MAX) &&
                                (filtered_y >= Y_CENTER_MIN) && (filtered_y <= Y_CENTER_MAX)) ? 1U : 0U;

          if ((guide_target_area_permille >= 0) &&
              (guide_target_area_permille < TRACK_LOCK_FAR_AREA_PERMILLE))
          {
              lock_enter_x_thresh = TRACK_LOCK_ENTER_X_ERR_FAR;
              lock_enter_y_thresh = TRACK_LOCK_ENTER_Y_ERR_FAR;
          }

          lock_x_error = filtered_x - TRACK_LOCK_CENTER_X_PIXELS;
          lock_y_error = filtered_y - TRACK_LOCK_CENTER_Y_PIXELS;
          abs_lock_x_error = (lock_x_error >= 0.0f) ? lock_x_error : (-lock_x_error);
          abs_lock_y_error = (lock_y_error >= 0.0f) ? lock_y_error : (-lock_y_error);

          if (target_track_locked)
          {
              if ((abs_lock_x_error > TRACK_LOCK_EXIT_X_ERR_PIXELS) ||
                  (abs_lock_y_error > TRACK_LOCK_EXIT_Y_ERR_PIXELS))
              {
                  if (target_track_unlock_candidate_tick == 0U)
                  {
                      target_track_unlock_candidate_tick = now_tick;
                      target_track_unlock_candidate_frames = 1U;
                  }
                  else
                  {
                      if (target_track_unlock_candidate_frames < 255U)
                      {
                          target_track_unlock_candidate_frames++;
                      }

                      if (((now_tick - target_track_unlock_candidate_tick) >= TRACK_LOCK_EXIT_STABLE_MS) ||
                          (target_track_unlock_candidate_frames >= TRACK_LOCK_EXIT_CONFIRM_FRAMES))
                      {
                          target_track_locked = 0U;
                          target_track_lock_candidate_tick = 0U;
                          target_track_unlock_candidate_tick = 0U;
                          target_track_unlock_candidate_frames = 0U;
                          smooth_step_x = 0.0f;
                          smooth_step_y = 0.0f;
                      }
                  }
              }
              else
              {
                  target_track_unlock_candidate_tick = 0U;
                  target_track_unlock_candidate_frames = 0U;
              }
          }
          else
          {
              target_track_unlock_candidate_tick = 0U;
              target_track_unlock_candidate_frames = 0U;

              /*
               * 锁定阈值仅用于“最后微调阶段”：
               * 只有进入中心区后才开始累计锁定候选时间。
               */
              if (in_final_tune_zone &&
                  (abs_lock_x_error <= lock_enter_x_thresh) &&
                  (abs_lock_y_error <= lock_enter_y_thresh))
              {
                  if (target_track_lock_candidate_tick == 0U)
                  {
                      target_track_lock_candidate_tick = now_tick;
                  }
                  else if ((now_tick - target_track_lock_candidate_tick) >= TRACK_LOCK_STABLE_MS)
                  {
                      target_track_locked = 1U;
                      target_track_unlock_candidate_tick = 0U;
                      target_track_unlock_candidate_frames = 0U;
                  }
              }
              else
              {
                  target_track_lock_candidate_tick = 0U;
              }
          }

          if (target_track_locked)
          {
              smooth_step_x = 0.0f;
              smooth_step_y = 0.0f;
              target_angle_x = current_angle_x;
              target_angle_y = current_angle_y;
          }
          else
          {
              raw_step_x = (x_error / IMAGE_CENTER_X)
                         * X_TRACK_STEP_GAIN_DEG
                         * X_TRACK_DIRECTION;

              if ((filtered_x >= X_CENTER_MIN) && (filtered_x <= X_CENTER_MAX))
              {
                  /* 中心区只减速，不冻结，提升反向穿越时的跟随性 */
                  raw_step_x *= X_CENTER_SLOW_FACTOR;
                  raw_step_x = Limit_Float(raw_step_x, -X_CENTER_STEP_MAX_DEG, X_CENTER_STEP_MAX_DEG);
              }
              else
              {
                  raw_step_x = Limit_Float(raw_step_x, -X_TRACK_STEP_MAX_DEG, X_TRACK_STEP_MAX_DEG);
              }

              /* 轻度平滑 step：smooth_step 往 raw_step 靠拢，避免台阶感又不过度变肉 */
              smooth_step_x = (smooth_step_x * (1.0f - X_STEP_SMOOTH_ALPHA)) +
                              (raw_step_x * X_STEP_SMOOTH_ALPHA);
              step_x = smooth_step_x;

              target_angle_x = current_angle_x + step_x;
              target_angle_x = Limit_Float(target_angle_x,
                                           SERVO_X_MIN_ANGLE,
                                           SERVO_X_MAX_ANGLE);

              {
                  float raw_step_y = (y_error / IMAGE_CENTER_Y)
                                   * Y_TRACK_STEP_GAIN_DEG
                                   * Y_TRACK_DIRECTION;

                  if ((filtered_y >= Y_CENTER_MIN) && (filtered_y <= Y_CENTER_MAX))
                  {
                      /* 中心区只减速，不冻结，与 X 轴策略统一 */
                      raw_step_y *= Y_CENTER_SLOW_FACTOR;
                      raw_step_y = Limit_Float(raw_step_y, -Y_CENTER_STEP_MAX_DEG, Y_CENTER_STEP_MAX_DEG);
                  }
                  else
                  {
                      raw_step_y = Limit_Float(raw_step_y, -Y_TRACK_STEP_MAX_DEG, Y_TRACK_STEP_MAX_DEG);
                  }

                  /* 轻度平滑 step：与 X 轴同机制，消除台阶感 */
                  smooth_step_y = (smooth_step_y * (1.0f - Y_STEP_SMOOTH_ALPHA)) +
                                  (raw_step_y * Y_STEP_SMOOTH_ALPHA);
                  step_y = smooth_step_y;

                  target_angle_y = current_angle_y + step_y;
                  target_angle_y = Limit_Float(target_angle_y,
                                               SERVO_Y_MIN_ANGLE,
                                               SERVO_Y_MAX_ANGLE);
              }
          }

          if (Should_Refresh_OLED(now_tick,
                                  (int32_t)filtered_x,
                                  (int32_t)filtered_y,
                                  (int32_t)guide_target_area_permille,
                                  0U))
          {
              OLED_ShowString(0, 0, "X:         ");
              OLED_ShowString(0, 2, "Y:         ");
              OLED_ShowString(0, 4, "A:         ");

              sprintf(tx_buffer, "%d", (int)filtered_x);
              OLED_ShowString(12, 0, tx_buffer);
              sprintf(tx_buffer, "%d", (int)filtered_y);
              OLED_ShowString(12, 2, tx_buffer);

              if (guide_target_area_permille >= 0)
              {
                  sprintf(tx_buffer, "%d", (int)guide_target_area_permille);
                  OLED_ShowString(12, 4, tx_buffer);
              }
              else
              {
                  OLED_ShowString(12, 4, "NA");
              }

              OLED_Refresh();
          }
          }
          else if ((!frame_has_target) && (!target_valid))
          {
              /* 仅在真正 LOST（而不是 HOLD）时显示 -1 */
              if (Should_Refresh_OLED(now_tick, -1, -1, -1, 0U))
              {
                  OLED_ShowString(0, 0, "X:-1       ");
                  OLED_ShowString(0, 2, "Y:-1       ");
                  OLED_ShowString(0, 4, "A:NA       ");
                  OLED_Refresh();
              }
          }
          else if ((!frame_has_target) && target_valid)
          {
              /* HOLD 期间无新帧：清空步进并冻结目标角，避免朝旧目标角继续蹭动 */
              smooth_step_x = 0.0f;
              smooth_step_y = 0.0f;
              target_angle_x = current_angle_x;
              target_angle_y = current_angle_y;
          }
      }
        }
    }
    }

    /* =========================================================
     * 5. 云台更新策略
     * =========================================================
     * 当前版本仅在 target_valid=1 时执行跟随；
     * target_valid=0 时进入静止或搜索流程。
     */
    if (!target_valid)
    {
        smooth_step_x = 0.0f;
        smooth_step_y = 0.0f;
        target_track_locked = 0U;
        target_track_lock_candidate_tick = 0U;
        target_track_unlock_candidate_tick = 0U;
        target_track_unlock_candidate_frames = 0U;
    }

    if (target_valid)
    {
        /* -----------------------------
         * 5.1 X 轴平滑更新
         * -----------------------------
         * 这里不是一步跳到 target_angle_x，
         * 而是平滑逼近，防止舵机动作过猛。
         */
        current_angle_x = current_angle_x * (1.0f - SERVO_SMOOTH_FACTOR)
                        + target_angle_x * SERVO_SMOOTH_FACTOR;

        /* 如果已经非常接近目标角度，就直接贴上去 */
        if ((current_angle_x > (target_angle_x - SERVO_DEAD_BAND)) &&
            (current_angle_x < (target_angle_x + SERVO_DEAD_BAND)))
        {
            current_angle_x = target_angle_x;
        }

        /* -----------------------------
         * 5.2 Y 轴平滑更新
         * ----------------------------- */
        current_angle_y = current_angle_y * (1.0f - SERVO_SMOOTH_FACTOR)
                        + target_angle_y * SERVO_SMOOTH_FACTOR;

        if ((current_angle_y > (target_angle_y - SERVO_DEAD_BAND)) &&
            (current_angle_y < (target_angle_y + SERVO_DEAD_BAND)))
        {
            current_angle_y = target_angle_y;
        }

        /* -----------------------------
         * 5.3 输出到两个舵机
         * ----------------------------- */
        Servo_SetAngle(&htim2, SERVO_X_CHANNEL, current_angle_x);
        Servo_SetAngle(&htim2, SERVO_Y_CHANNEL, current_angle_y);
    }
else
{
    /* =====================================================
     * NONE 模式：保持静止
     * ===================================================== */
    if (current_target_mode == TARGET_MODE_NONE)
    {
        target_angle_x = current_angle_x;
        target_angle_y = current_angle_y;

        Servo_SetAngle(&htim2, SERVO_X_CHANNEL, current_angle_x);
        Servo_SetAngle(&htim2, SERVO_Y_CHANNEL, current_angle_y);
    }
    else
    {
        /* =====================================================
         * 搜索模式（主动寻找目标）
         * ===================================================== */

        if (!target_valid && current_target_mode != TARGET_MODE_NONE)
        {
            if (!Search_IsActive_FromModule())
            {
                Start_NormalSearch_ByModule(HAL_GetTick());
            }
        }

        if (Search_IsActive_FromModule())
        {
            uint32_t now_tick = HAL_GetTick();
            if (SearchControl_Update(&g_search_control_state,
                                     &g_search_control_cfg,
                                     now_tick,
                                     target_valid,
                                     (TargetMode_t)current_target_mode,
                                     &target_angle_x,
                                     &target_angle_y))
            {
                /* 搜索模式同样使用指数平滑，避免直接跳角度产生台阶感 */
                current_angle_x = current_angle_x * (1.0f - SERVO_SMOOTH_FACTOR)
                                + target_angle_x * SERVO_SMOOTH_FACTOR;
                current_angle_y = current_angle_y * (1.0f - SERVO_SMOOTH_FACTOR)
                                + target_angle_y * SERVO_SMOOTH_FACTOR;

                Servo_SetAngle(&htim2, SERVO_X_CHANNEL, current_angle_x);
                Servo_SetAngle(&htim2, SERVO_Y_CHANNEL, current_angle_y);
            }
        }
    }
}

    /* 搜索/发现目标的自动语音放在状态更新完成后统一处理 */
    Handle_SystemVoice_Update();
    Handle_Buzzer_Update();
    Handle_GuideVoice_Update();

    HAL_Delay(SERVO_UPDATE_DELAY_MS);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PE2 PE3 PE7 PE13
                           PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : buzzer_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * 函数名：Parse_OpenMV_Message
 * 功能：解析 OpenMV 串口报文，兼容 XY 与 XYA 两种格式
 *
 * 支持格式示例：
 * 1. X:123,Y:45
 * 2. X:123,Y:45,A:18
 * 3. X:123, Y:45, A:18（带空格也可）
 *
 * 返回值：
 * 0 -> 无效报文
 * 2 -> 仅解析到 X、Y
 * 3 -> 解析到 X、Y、A
 */
uint8_t Parse_OpenMV_Message(const char *msg, int *x, int *y, int *area_permille)
{
    const char *x_tag;
    const char *y_tag;
    const char *a_tag;
    char *end_ptr;
    long parsed_value;

    if ((msg == NULL) || (x == NULL) || (y == NULL) || (area_permille == NULL))
    {
        return 0U;
    }

    *x = 0;
    *y = 0;
    *area_permille = -1;

    x_tag = strstr(msg, "X:");
    y_tag = strstr(msg, "Y:");

    if ((x_tag == NULL) || (y_tag == NULL))
    {
        return 0U;
    }

    parsed_value = strtol(x_tag + 2, &end_ptr, 10);
    if (end_ptr == (x_tag + 2))
    {
        return 0U;
    }
    *x = (int)parsed_value;

    parsed_value = strtol(y_tag + 2, &end_ptr, 10);
    if (end_ptr == (y_tag + 2))
    {
        return 0U;
    }
    *y = (int)parsed_value;

    a_tag = strstr(msg, "A:");
    if (a_tag != NULL)
    {
        parsed_value = strtol(a_tag + 2, &end_ptr, 10);
        if (end_ptr != (a_tag + 2))
        {
            *area_permille = (int)parsed_value;
            return 3U;
        }
    }

    return 2U;
}

/*
 * 函数名：Try_Dequeue_OpenMV_Line
 * 功能：从 UART2 接收队列取出一帧文本
 *
 * 返回值：
 * 0 -> 队列为空
 * 1 -> 取到一帧
 */
uint8_t Try_Dequeue_OpenMV_Line(char *dst, uint16_t dst_len)
{
    uint8_t read_idx;

    if ((dst == NULL) || (dst_len == 0U))
    {
        return 0U;
    }

    __disable_irq();
    if (uart2_queue_count == 0U)
    {
        __enable_irq();
        return 0U;
    }

    read_idx = uart2_queue_read_idx;
    uart2_queue_read_idx = (uint8_t)((uart2_queue_read_idx + 1U) % UART2_RX_QUEUE_LEN);
    uart2_queue_count--;
    __enable_irq();

    strncpy(dst, uart2_line_queue[read_idx], (size_t)dst_len - 1U);
    dst[dst_len - 1U] = '\0';
    return 1U;
}

/*
 * 函数名：Should_Refresh_OLED
 * 功能：OLED 刷新节流，降低频繁 I2C 刷屏带来的控制延迟
 */
uint8_t Should_Refresh_OLED(uint32_t now_tick,
                            int32_t disp_x,
                            int32_t disp_y,
                            int32_t disp_a,
                            uint8_t force_refresh)
{
    if (force_refresh)
    {
        oled_last_refresh_tick = now_tick;
        oled_last_disp_x = disp_x;
        oled_last_disp_y = disp_y;
        oled_last_disp_a = disp_a;
        return 1U;
    }

    if ((now_tick - oled_last_refresh_tick) < OLED_REFRESH_INTERVAL_MS)
    {
        return 0U;
    }

    oled_last_refresh_tick = now_tick;
    oled_last_disp_x = disp_x;
    oled_last_disp_y = disp_y;
    oled_last_disp_a = disp_a;
    return 1U;
}

/*
 * 函数名：Kalman1D_Init
 * 功能：初始化一维位置-速度卡尔曼滤波器
 */
void Kalman1D_Init(Kalman1D *kf,
                   float init_pos,
                   float init_vel,
                   float q_pos,
                   float q_vel,
                   float r)
{
    if (kf == NULL)
    {
        return;
    }

    kf->x = init_pos;
    kf->v = init_vel;
    kf->p00 = KALMAN_P00_INIT;
    kf->p01 = 0.0f;
    kf->p10 = 0.0f;
    kf->p11 = KALMAN_P11_INIT;
    kf->q_pos = q_pos;
    kf->q_vel = q_vel;
    kf->r = r;
}

/*
 * 函数名：Kalman1D_Predict
 * 功能：执行卡尔曼预测步骤
 */
void Kalman1D_Predict(Kalman1D *kf, float dt_s)
{
    float p00_old;
    float p01_old;
    float p10_old;
    float p11_old;

    if (kf == NULL)
    {
        return;
    }

    if (dt_s <= 0.0f)
    {
        dt_s = KALMAN_FIXED_DT_S;
    }

    kf->x += kf->v * dt_s;

    p00_old = kf->p00;
    p01_old = kf->p01;
    p10_old = kf->p10;
    p11_old = kf->p11;

    kf->p00 = p00_old + (dt_s * (p10_old + p01_old)) + (dt_s * dt_s * p11_old) + kf->q_pos;
    kf->p01 = p01_old + (dt_s * p11_old);
    kf->p10 = p10_old + (dt_s * p11_old);
    kf->p11 = p11_old + kf->q_vel;
}

/*
 * 函数名：Kalman1D_Update
 * 功能：执行卡尔曼测量更新步骤
 */
float Kalman1D_Update(Kalman1D *kf, float measurement)
{
    float p00_old;
    float p01_old;
    float p10_old;
    float p11_old;
    float residual;
    float s;
    float k0;
    float k1;

    if (kf == NULL)
    {
        return measurement;
    }

    p00_old = kf->p00;
    p01_old = kf->p01;
    p10_old = kf->p10;
    p11_old = kf->p11;

    residual = measurement - kf->x;
    s = p00_old + kf->r;

    if (s <= 0.0f)
    {
        s = 1e-6f;
    }

    k0 = p00_old / s;
    k1 = p10_old / s;

    kf->x += k0 * residual;
    kf->v += k1 * residual;

    kf->p00 = (1.0f - k0) * p00_old;
    kf->p01 = (1.0f - k0) * p01_old;
    kf->p10 = p10_old - (k1 * p00_old);
    kf->p11 = p11_old - (k1 * p01_old);

    return kf->x;
}

/*
 * 函数名：Reset_TargetCoordinateFilters
 * 功能：重置坐标滤波器到图像中心
 */
void Reset_TargetCoordinateFilters(void)
{
    Kalman1D_Init(&kf_x,
                  IMAGE_CENTER_X,
                  0.0f,
                  KALMAN_Q_POS_DEFAULT,
                  KALMAN_Q_VEL_DEFAULT,
                  KALMAN_R_MEAS_DEFAULT);
    Kalman1D_Init(&kf_y,
                  IMAGE_CENTER_Y,
                  0.0f,
                  KALMAN_Q_POS_DEFAULT,
                  KALMAN_Q_VEL_DEFAULT,
                  KALMAN_R_MEAS_DEFAULT);

    filtered_x = IMAGE_CENTER_X;
    filtered_y = IMAGE_CENTER_Y;
    smooth_step_x = 0.0f;
    kalman_last_update_tick = 0U;
    target_track_locked = 0U;
    target_track_lock_candidate_tick = 0U;
    target_track_unlock_candidate_tick = 0U;
    target_track_unlock_candidate_frames = 0U;
}

uint32_t Get_TargetPresence_HoldTimeoutMs(void)
{
    if (target_presence_hold_locked_snapshot)
    {
        return TARGET_HOLD_MS_LOCKED;
    }
    return TARGET_HOLD_MS_NORMAL;
}

void Reset_TargetPresence_State(void)
{
    target_presence_state = TARGET_PRESENCE_LOST;
    target_presence_hit_count = 0U;
    target_presence_miss_count = 0U;
    target_presence_hold_start_tick = 0U;
    target_presence_hold_locked_snapshot = 0U;
}

void Update_TargetPresence_State(uint8_t frame_has_target, int32_t area_permille, uint32_t now_tick)
{
    uint8_t area_ok = 1U;

    if ((area_permille >= 0) && ((uint32_t)area_permille < TARGET_CONFIRM_MIN_AREA_PERMILLE))
    {
        area_ok = 0U;
    }

    switch (target_presence_state)
    {
        case TARGET_PRESENCE_LOST:
            if (frame_has_target)
            {
                if (target_presence_hit_count < 255U)
                {
                    target_presence_hit_count++;
                }

                if (target_presence_hit_count >= TARGET_CANDIDATE_ENTER_FRAMES)
                {
                    target_presence_state = TARGET_PRESENCE_CANDIDATE;
                    target_presence_miss_count = 0U;
                }
            }
            else
            {
                target_presence_hit_count = 0U;
            }
            break;

        case TARGET_PRESENCE_CANDIDATE:
            if (frame_has_target)
            {
                target_presence_miss_count = 0U;
                if (target_presence_hit_count < 255U)
                {
                    target_presence_hit_count++;
                }

                if ((target_presence_hit_count >= TARGET_CONFIRM_ENTER_FRAMES) && area_ok)
                {
                    target_presence_state = TARGET_PRESENCE_CONFIRMED;
                    target_presence_hit_count = 0U;
                    target_presence_miss_count = 0U;
                }
            }
            else
            {
                if (target_presence_miss_count < 255U)
                {
                    target_presence_miss_count++;
                }
                target_presence_hit_count = 0U;

                if (target_presence_miss_count > TARGET_CANDIDATE_MAX_MISS_FRAMES)
                {
                    target_presence_state = TARGET_PRESENCE_LOST;
                    target_presence_hit_count = 0U;
                    target_presence_miss_count = 0U;
                }
            }
            break;

        case TARGET_PRESENCE_CONFIRMED:
            if (!frame_has_target)
            {
                target_presence_state = TARGET_PRESENCE_HOLD;
                target_presence_miss_count = 1U;
                target_presence_hold_start_tick = now_tick;
                target_presence_hold_locked_snapshot = target_track_locked ? 1U : 0U;
            }
            break;

        case TARGET_PRESENCE_HOLD:
            if (frame_has_target)
            {
                target_presence_state = TARGET_PRESENCE_CONFIRMED;
                target_presence_hit_count = 0U;
                target_presence_miss_count = 0U;
                target_presence_hold_start_tick = 0U;
                target_presence_hold_locked_snapshot = 0U;
            }
            else
            {
                uint32_t hold_timeout_ms = Get_TargetPresence_HoldTimeoutMs();
                uint8_t timeout_reached = ((now_tick - target_presence_hold_start_tick) >= hold_timeout_ms) ? 1U : 0U;
                uint8_t miss_limit_reached = 0U;

                if (target_presence_miss_count < 255U)
                {
                    target_presence_miss_count++;
                }

                /*
                 * HOLD 判丢策略：
                 * 1. 锁定快照下（进入 HOLD 时已锁定）采用“时间主导”，不让漏检帧数提前打穿
                 * 2. 非锁定下保留时间 + 漏检帧双条件
                 */
                if ((!target_presence_hold_locked_snapshot) &&
                    (target_presence_miss_count >= TARGET_HOLD_MAX_MISS_FRAMES_NORMAL))
                {
                    miss_limit_reached = 1U;
                }

                if (timeout_reached || miss_limit_reached)
                {
                    target_presence_state = TARGET_PRESENCE_LOST;
                    target_presence_hit_count = 0U;
                    target_presence_miss_count = 0U;
                    target_presence_hold_start_tick = 0U;
                    target_presence_hold_locked_snapshot = 0U;
                }
            }
            break;

        default:
            Reset_TargetPresence_State();
            break;
    }
}

uint8_t Is_TargetPresence_Valid(void)
{
    if ((target_presence_state == TARGET_PRESENCE_CONFIRMED) ||
        (target_presence_state == TARGET_PRESENCE_HOLD))
    {
        return 1U;
    }
    return 0U;
}

/*
 * 函数名：Send_TargetMode_To_OpenMV
 * 功能：把当前目标模式通过 USART2 发给 OpenMV
 *
 * 协议格式：
 * TARGET:0\r\n
 * TARGET:1\r\n
 * TARGET:2\r\n
 * TARGET:3\r\n
 * TARGET:4\r\n
 *
 * 说明：
 * 这里使用 USART2，是因为当前工程里 OpenMV 和 STM32 的通信
 * 就是走 huart2 这一条链路。
 */
void Send_TargetMode_To_OpenMV(uint8_t mode)
{
    char cmd[20];

    sprintf(cmd, "TARGET:%d\r\n", mode);
    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
}

/*
 * 函数名：Update_TargetMode_Display
 * 功能：在 OLED 上显示当前识别模式
 *
 * 说明：
 * 这里单独封装成函数，后续如果你想改显示样式，
 * 只需要改这里，不用到处改主逻辑。
 */
void Update_TargetMode_Display(uint8_t mode)
{
    OLED_ShowString(0, 6, "MODE:       ");
    OLED_ShowString(36, 6, (char *)target_mode_name[mode]);
    OLED_Refresh();
}

/*
 * 函数名：Search_IsActive_FromModule
 * 功能：判断“搜索是否激活”
 *
 * 说明：
 * 1. 搜索模块作为唯一状态源
 * 2. start_tick != 0 代表已经进入某种搜索流程（普通搜索或重捕获）
 */
uint8_t Search_IsActive_FromModule(void)
{
    return SearchControl_IsActive(&g_search_control_state);
}

/*
 * 函数名：Search_IsLocalRecapture_FromModule
 * 功能：判断当前是否处于局部重捕获阶段
 */
uint8_t Search_IsLocalRecapture_FromModule(void)
{
    return SearchControl_IsLocalRecapture(&g_search_control_state);
}

/*
 * 函数名：Start_NormalSearch_ByModule
 * 功能：通过搜索模块进入普通搜索
 */
void Start_NormalSearch_ByModule(uint32_t now_tick)
{
    SearchControl_StartModeSearch(&g_search_control_state,
                                  &g_search_control_cfg,
                                  now_tick,
                                  current_angle_x,
                                  last_x_error);
}

/*
 * 函数名：Start_Recapture_ByModule
 * 功能：通过搜索模块进入局部重捕获（phase1）
 */
void Start_Recapture_ByModule(uint32_t now_tick)
{
    SearchControl_StartRecapture(&g_search_control_state,
                                 &g_search_control_cfg,
                                 now_tick,
                                 current_angle_x,
                                 last_x_error);
}

/*
 * 函数名：Stop_Search_ByModule
 * 功能：目标恢复后退出搜索，由模块统一复位状态
 */
void Stop_Search_ByModule(void)
{
    SearchControl_OnTargetRecovered(&g_search_control_state,
                                    &g_search_control_cfg,
                                    current_angle_x);
}

/*
 * 函数名：Reset_Search_ByModule
 * 功能：清空搜索状态（例如 None 模式）
 */
void Reset_Search_ByModule(void)
{
    SearchControl_Reset(&g_search_control_state, &g_search_control_cfg, current_angle_x);
}

/*
 * 函数名：Get_GuideZone_FromPanAngle
 * 功能：根据云台水平角度判断用户与目标的相对方向分区
 */
GuideZone_t Get_GuideZone_FromPanAngle(float pan_angle_deg)
{
    float offset_deg = pan_angle_deg - GUIDE_CENTER_ANGLE_DEG;

    if (offset_deg <= (-GUIDE_OFFSET_STRONG_DEG))
    {
        return GUIDE_ZONE_RIGHT;
    }

    if (offset_deg <= (-GUIDE_OFFSET_FORWARD_DEG))
    {
        return GUIDE_ZONE_SLIGHT_RIGHT;
    }

    if (offset_deg < GUIDE_OFFSET_FORWARD_DEG)
    {
        return GUIDE_ZONE_FORWARD;
    }

    if (offset_deg < GUIDE_OFFSET_STRONG_DEG)
    {
        return GUIDE_ZONE_SLIGHT_LEFT;
    }

    return GUIDE_ZONE_LEFT;
}

/*
 * 函数名：Get_GuideEvent_FromZone
 * 功能：把方向分区映射到系统语音事件
 */
SystemVoiceEvent_t Get_GuideEvent_FromZone(GuideZone_t zone)
{
    switch (zone)
    {
        case GUIDE_ZONE_LEFT:
            return VOICE_EVT_GUIDE_LEFT;

        case GUIDE_ZONE_SLIGHT_LEFT:
            return VOICE_EVT_GUIDE_LEFT;

        case GUIDE_ZONE_FORWARD:
            return VOICE_EVT_GUIDE_FACING_TARGET;

        case GUIDE_ZONE_SLIGHT_RIGHT:
            return VOICE_EVT_GUIDE_RIGHT;

        case GUIDE_ZONE_RIGHT:
            return VOICE_EVT_GUIDE_RIGHT;

        default:
            return VOICE_EVT_NONE;
    }
}

/*
 * 函数名：Reset_GuideVoice_State
 * 功能：重置引导语音状态机（分区稳定与冷却）
 */
void Reset_GuideVoice_State(void)
{
    guide_enable_after_tick = 0U;
    guide_last_zone = GUIDE_ZONE_NONE;
    guide_candidate_zone = GUIDE_ZONE_NONE;
    guide_zone_candidate_tick = 0U;
    guide_forward_enter_tick = 0U;
    guide_prompt_facing_played = 0U;
    guide_forward_level_played = GUIDE_FORWARD_LEVEL_NONE;
    guide_forward_level_candidate = GUIDE_FORWARD_LEVEL_NONE;
    guide_forward_level_candidate_tick = 0U;
    guide_forward_level_ratio_state = GUIDE_FORWARD_LEVEL_GO;
    guide_target_area_permille = -1;
    guide_filtered_area_permille = 0.0f;
    guide_area_ref_valid = 0U;
    guide_area_ref_permille = 0.0f;
    guide_ratio_filtered_permille = 1000.0f;
    guide_area_ref_collect_start_tick = 0U;
    guide_area_ref_collect_sum = 0.0f;
    guide_area_ref_collect_samples = 0U;
    guide_command_state = GUIDE_COMMAND_IDLE;
}

GuideForwardLevel_t Get_GuideForwardLevel_FromAreaByMode(uint8_t mode, int32_t area_permille)
{
    GuideAreaThresholdConfig_t threshold_cfg;

    threshold_cfg.continue_permille = GUIDE_AREA_CONTINUE_PERMILLE;
    threshold_cfg.near_permille = GUIDE_AREA_NEAR_PERMILLE;
    threshold_cfg.arrived_permille = GUIDE_AREA_ARRIVED_PERMILLE;

    if ((uint32_t)mode < TARGET_MODE_COUNT)
    {
        threshold_cfg = guide_area_threshold_table[mode];
    }

    if (area_permille < 0)
    {
        return GUIDE_FORWARD_LEVEL_GO;
    }

    if ((uint32_t)area_permille >= threshold_cfg.arrived_permille)
    {
        return GUIDE_FORWARD_LEVEL_ARRIVED;
    }

    if ((uint32_t)area_permille >= threshold_cfg.near_permille)
    {
        return GUIDE_FORWARD_LEVEL_NEAR;
    }

    if ((uint32_t)area_permille >= threshold_cfg.continue_permille)
    {
        return GUIDE_FORWARD_LEVEL_CONTINUE;
    }

    return GUIDE_FORWARD_LEVEL_GO;
}

GuideForwardLevel_t Get_GuideForwardLevel_FromRatio(float ratio_permille,
                                                    GuideForwardLevel_t prev_level)
{
    GuideForwardLevel_t next_level = prev_level;

    switch (prev_level)
    {
        case GUIDE_FORWARD_LEVEL_ARRIVED:
            if (ratio_permille < (float)GUIDE_RATIO_ARRIVED_EXIT_PM)
            {
                next_level = GUIDE_FORWARD_LEVEL_NEAR;
            }
            break;

        case GUIDE_FORWARD_LEVEL_NEAR:
            if (ratio_permille >= (float)GUIDE_RATIO_ARRIVED_ENTER_PM)
            {
                next_level = GUIDE_FORWARD_LEVEL_ARRIVED;
            }
            else if (ratio_permille < (float)GUIDE_RATIO_NEAR_EXIT_PM)
            {
                next_level = GUIDE_FORWARD_LEVEL_CONTINUE;
            }
            break;

        case GUIDE_FORWARD_LEVEL_CONTINUE:
            if (ratio_permille >= (float)GUIDE_RATIO_NEAR_ENTER_PM)
            {
                next_level = GUIDE_FORWARD_LEVEL_NEAR;
            }
            else if (ratio_permille < (float)GUIDE_RATIO_CONTINUE_EXIT_PM)
            {
                next_level = GUIDE_FORWARD_LEVEL_GO;
            }
            break;

        case GUIDE_FORWARD_LEVEL_GO:
        default:
            if (ratio_permille >= (float)GUIDE_RATIO_CONTINUE_ENTER_PM)
            {
                next_level = GUIDE_FORWARD_LEVEL_CONTINUE;
            }
            break;
    }

    return next_level;
}

SystemVoiceEvent_t Get_GuideForwardEvent_FromLevel(GuideForwardLevel_t level)
{
    switch (level)
    {
        case GUIDE_FORWARD_LEVEL_GO:
            return VOICE_EVT_GUIDE_FORWARD;

        case GUIDE_FORWARD_LEVEL_CONTINUE:
            return VOICE_EVT_NONE;

        case GUIDE_FORWARD_LEVEL_NEAR:
            return VOICE_EVT_NONE;

        case GUIDE_FORWARD_LEVEL_ARRIVED:
            return VOICE_EVT_GUIDE_ARRIVED;

        default:
            return VOICE_EVT_NONE;
    }
}

/*
 * 函数名：Handle_GuideVoice_Update
 * 功能：在目标稳定跟踪阶段输出方向引导语音
 */
void Handle_GuideVoice_Update(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint32_t guard_until_tick = SystemVoice_GetGuardUntilTick(&g_system_voice_state);
    float offset_deg = 0.0f;
    float abs_offset_deg = 0.0f;
    GuideZone_t zone;
    SystemVoiceEvent_t guide_event;

    if ((current_target_mode == TARGET_MODE_NONE) ||
        Search_IsActive_FromModule() ||
        (!target_valid))
    {
        guide_candidate_zone = GUIDE_ZONE_NONE;
        guide_zone_candidate_tick = 0U;
        guide_last_zone = GUIDE_ZONE_NONE;
        guide_forward_enter_tick = 0U;
        guide_prompt_facing_played = 0U;
        guide_forward_level_played = GUIDE_FORWARD_LEVEL_NONE;
        guide_forward_level_candidate = GUIDE_FORWARD_LEVEL_NONE;
        guide_forward_level_candidate_tick = 0U;
        guide_forward_level_ratio_state = GUIDE_FORWARD_LEVEL_GO;
        guide_target_area_permille = -1;
        guide_filtered_area_permille = 0.0f;
        guide_area_ref_valid = 0U;
        guide_area_ref_permille = 0.0f;
        guide_ratio_filtered_permille = 1000.0f;
        guide_area_ref_collect_start_tick = 0U;
        guide_area_ref_collect_sum = 0.0f;
        guide_area_ref_collect_samples = 0U;
        guide_command_state = GUIDE_COMMAND_IDLE;
        return;
    }

    /*
     * 引导逻辑优化：
     * 1. 如果处于 HOLD 状态（目标短暂丢失），保持静默，不发出方向误导。
     * 2. 引入周期性重复提示（2.5s），防止用户一直偏离却听不到二次指令。
     */
    if (target_presence_state == TARGET_PRESENCE_HOLD)
    {
        return;
    }

    if (!g_voice_policy_state.mode_found_prompt_played ||
        g_voice_policy_state.lost_recovery_active ||
        g_voice_policy_state.mode_switch_wait_for_first_found)
    {
        return;
    }

    if ((int32_t)(guide_enable_after_tick - now_tick) > 0)
    {
        return;
    }

    if ((int32_t)(guard_until_tick - now_tick) > 0)
    {
        return;
    }

    /* 方向分区压缩成三态：左 / 中 / 右 */
    offset_deg = current_angle_x - GUIDE_CENTER_ANGLE_DEG;
    abs_offset_deg = (offset_deg >= 0.0f) ? offset_deg : (-offset_deg);
    zone = GUIDE_ZONE_FORWARD;

    if ((guide_last_zone == GUIDE_ZONE_FORWARD) && (abs_offset_deg <= GUIDE_OFFSET_FORWARD_EXIT_DEG))
    {
        zone = GUIDE_ZONE_FORWARD;
    }
    else if (offset_deg >= GUIDE_OFFSET_FORWARD_DEG)
    {
        zone = GUIDE_ZONE_LEFT;
    }
    else if (offset_deg <= (-GUIDE_OFFSET_FORWARD_DEG))
    {
        zone = GUIDE_ZONE_RIGHT;
    }

    if (zone != guide_candidate_zone)
    {
        guide_candidate_zone = zone;
        guide_zone_candidate_tick = now_tick;
        return;
    }

    if ((now_tick - guide_zone_candidate_tick) < GUIDE_ZONE_STABLE_MS)
    {
        return;
    }

    /*
     * 指令型引导策略（三态）：
     * 1. IDLE         : 允许发一次“向左/向右转”，或居中时直接“正前方”
     * 2. WAIT_CENTER  : 已发左右转指令，只等进入中间区，定期重复提示指令
     * 3. DONE         : 已播“正前方”，等待再次明显偏离后再开启下一轮
     */
    if ((guide_command_state == GUIDE_COMMAND_DONE) && (zone != GUIDE_ZONE_FORWARD))
    {
        guide_command_state = GUIDE_COMMAND_IDLE;
    }

    if (guide_command_state == GUIDE_COMMAND_WAIT_CENTER)
    {
        if (zone == GUIDE_ZONE_FORWARD)
        {
            SystemVoice_PostEvent(&g_system_voice_state,
                                  VOICE_EVT_GUIDE_FACING_TARGET,
                                  (TargetMode_t)current_target_mode,
                                  now_tick);
            SystemVoice_Update(&g_system_voice_state, now_tick);
            guide_command_state = GUIDE_COMMAND_DONE;
            guide_last_voice_tick = now_tick;
        }
        else
        {
            /* 如果用户迟迟未修正，每隔一段时间（如 2.5s）重新提示一次 */
            if ((now_tick - guide_last_voice_tick) >= GUIDE_REPROMPT_INTERVAL_MS)
            {
                guide_event = Get_GuideEvent_FromZone(zone);
                if (guide_event != VOICE_EVT_NONE)
                {
                    SystemVoice_PostEvent(&g_system_voice_state,
                                          guide_event,
                                          (TargetMode_t)current_target_mode,
                                          now_tick);
                    SystemVoice_Update(&g_system_voice_state, now_tick);
                    guide_last_voice_tick = now_tick;
                }
            }
        }
        return;
    }

    if (guide_command_state == GUIDE_COMMAND_IDLE)
    {
        if (zone == GUIDE_ZONE_FORWARD)
        {
            guide_event = VOICE_EVT_GUIDE_FACING_TARGET;
            guide_command_state = GUIDE_COMMAND_DONE;
        }
        else
        {
            guide_event = Get_GuideEvent_FromZone(zone);
            guide_command_state = GUIDE_COMMAND_WAIT_CENTER;
        }

        if (guide_event != VOICE_EVT_NONE)
        {
            SystemVoice_PostEvent(&g_system_voice_state,
                                  guide_event,
                                  (TargetMode_t)current_target_mode,
                                  now_tick);
            SystemVoice_Update(&g_system_voice_state, now_tick);
            guide_last_voice_tick = now_tick;
        }
    }
}

/*
 * 函数名：Key_Scan_And_Handle
 * 功能：扫描五个按钮，并根据按钮结果切换目标模式
 *
 * 当前映射关系：
 *PC0 -> Toilet
 *PC1 -> Exit
 *PE13 -> Office
 *PE3 -> Stairs
 *PE2 -> Elevator
 *PE7 -> Extinguisher
 *PE14 -> DFPlayer Test
 *PE15 -> None
 *
 * 说明：
 * 1. 当前按钮采用“上拉输入”
 * 2. 所以按下时读到的是 GPIO_PIN_RESET
 * 3. 用统一的消抖时间，防止一次按压触发多次
 */
void Key_Scan_And_Handle(void)
{
    enum
    {
        KEY_EVENT_NONE = 0,
        KEY_EVENT_VOICE,
        KEY_EVENT_TOILET,
        KEY_EVENT_EXIT,
        KEY_EVENT_OFFICE,
        KEY_EVENT_STAIRS,
        KEY_EVENT_ELEVATOR,
        KEY_EVENT_EXTINGUISHER,
        KEY_EVENT_MODE_NONE
    };

    static uint8_t key_scan_initialized = 0U;
    static uint8_t key_latched_voice = 0U;
    static uint8_t key_latched_toilet = 0U;
    static uint8_t key_latched_exit = 0U;
    static uint8_t key_latched_office = 0U;
    static uint8_t key_latched_stairs = 0U;
    static uint8_t key_latched_elevator = 0U;
    static uint8_t key_latched_extinguisher = 0U;
    static uint8_t key_latched_none = 0U;

    uint32_t now_tick = HAL_GetTick();
    uint8_t voice_pressed = (HAL_GPIO_ReadPin(KEY_VOICE_PORT, KEY_VOICE_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t toilet_pressed = (HAL_GPIO_ReadPin(KEY_PERSON_PORT, KEY_PERSON_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t exit_pressed = (HAL_GPIO_ReadPin(KEY_BOTTLE_PORT, KEY_BOTTLE_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t office_pressed = (HAL_GPIO_ReadPin(KEY_PEN_PORT, KEY_PEN_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t stairs_pressed = (HAL_GPIO_ReadPin(KEY_STAIRS_PORT, KEY_STAIRS_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t elevator_pressed = (HAL_GPIO_ReadPin(KEY_ELEVATOR_PORT, KEY_ELEVATOR_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t extinguisher_pressed = (HAL_GPIO_ReadPin(KEY_EXTINGUISHER_PORT, KEY_EXTINGUISHER_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t none_pressed = (HAL_GPIO_ReadPin(KEY_NONE_PORT, KEY_NONE_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t key_event = KEY_EVENT_NONE;
    uint8_t next_mode = TARGET_MODE_NONE;

    if (!key_scan_initialized)
    {
        key_scan_initialized = 1U;
        key_latched_voice = voice_pressed;
        key_latched_toilet = toilet_pressed;
        key_latched_exit = exit_pressed;
        key_latched_office = office_pressed;
        key_latched_stairs = stairs_pressed;
        key_latched_elevator = elevator_pressed;
        key_latched_extinguisher = extinguisher_pressed;
        key_latched_none = none_pressed;
        return;
    }

    /*
     * 独立按键锁存释放：
     * 某个按键回到高电平时，清除该按键锁存，允许下次再次触发。
     * 这样即使某个按键线路异常，也不会拖死其他按键。
     */
    if (!voice_pressed) key_latched_voice = 0U;
    if (!toilet_pressed) key_latched_toilet = 0U;
    if (!exit_pressed) key_latched_exit = 0U;
    if (!office_pressed) key_latched_office = 0U;
    if (!stairs_pressed) key_latched_stairs = 0U;
    if (!elevator_pressed) key_latched_elevator = 0U;
    if (!extinguisher_pressed) key_latched_extinguisher = 0U;
    if (!none_pressed) key_latched_none = 0U;

    if ((now_tick - key_last_tick) < KEY_DEBOUNCE_MS)
    {
        return;
    }

    if (voice_pressed && (!key_latched_voice))
    {
        key_latched_voice = 1U;
        key_event = KEY_EVENT_VOICE;
    }
    else if (toilet_pressed && (!key_latched_toilet))
    {
        key_latched_toilet = 1U;
        key_event = KEY_EVENT_TOILET;
    }
    else if (exit_pressed && (!key_latched_exit))
    {
        key_latched_exit = 1U;
        key_event = KEY_EVENT_EXIT;
    }
    else if (office_pressed && (!key_latched_office))
    {
        key_latched_office = 1U;
        key_event = KEY_EVENT_OFFICE;
    }
    else if (stairs_pressed && (!key_latched_stairs))
    {
        key_latched_stairs = 1U;
        key_event = KEY_EVENT_STAIRS;
    }
    else if (elevator_pressed && (!key_latched_elevator))
    {
        key_latched_elevator = 1U;
        key_event = KEY_EVENT_ELEVATOR;
    }
    else if (extinguisher_pressed && (!key_latched_extinguisher))
    {
        key_latched_extinguisher = 1U;
        key_event = KEY_EVENT_EXTINGUISHER;
    }
    else if (none_pressed && (!key_latched_none))
    {
        key_latched_none = 1U;
        key_event = KEY_EVENT_MODE_NONE;
    }

    if (key_event != KEY_EVENT_NONE)
    {
        key_last_tick = now_tick;

        if (key_event == KEY_EVENT_VOICE)
        {
            Handle_Voice_Test_Key();
            return;
        }

        switch (key_event)
        {
            case KEY_EVENT_TOILET:
                next_mode = TARGET_MODE_TOILET;
                break;

            case KEY_EVENT_EXIT:
                next_mode = TARGET_MODE_EXIT;
                break;

            case KEY_EVENT_OFFICE:
                next_mode = TARGET_MODE_OFFICE;
                break;

            case KEY_EVENT_STAIRS:
                next_mode = TARGET_MODE_STAIRS;
                break;

            case KEY_EVENT_ELEVATOR:
                next_mode = TARGET_MODE_ELEVATOR;
                break;

            case KEY_EVENT_EXTINGUISHER:
                next_mode = TARGET_MODE_EXTINGUISHER;
                break;

            case KEY_EVENT_MODE_NONE:
                next_mode = TARGET_MODE_NONE;
                break;

            default:
                break;
        }

        current_target_mode = next_mode;
        Update_TargetMode_Display(current_target_mode);
        Send_TargetMode_To_OpenMV(current_target_mode);

        if (current_target_mode == TARGET_MODE_NONE)
        {
            Reset_SystemVoice_State(0);
            Reset_TargetCoordinateFilters();
            g_voice_policy_state.mode_found_prompt_played = 1U;
            g_voice_policy_state.mode_switch_search_only_until_tick = 0U;
            g_voice_policy_state.mode_switch_wait_for_first_found = 0U;
            SystemVoice_PostEvent(&g_system_voice_state,
                                  VOICE_EVT_MODE_STOPPED,
                                  TARGET_MODE_NONE,
                                  now_tick);

            target_valid = 0;
            Reset_TargetPresence_State();
            Reset_Search_ByModule();
            g_voice_policy_state.prev_target_valid = 0U;
            g_voice_policy_state.prev_search_mode = Search_IsActive_FromModule();

            /* None 模式锁定当前位置，停止云台继续运动 */
            target_angle_x = current_angle_x;
            target_angle_y = current_angle_y;
            Servo_SetAngle(&htim2, SERVO_X_CHANNEL, current_angle_x);
            Servo_SetAngle(&htim2, SERVO_Y_CHANNEL, current_angle_y);
            return;
        }

        Reset_SystemVoice_State(1);
        Reset_TargetCoordinateFilters();
        g_voice_policy_state.mode_switch_search_only_until_tick =
            now_tick + VOICE_MODE_SWITCH_SEARCH_ONLY_MS;
        g_voice_policy_state.mode_switch_wait_for_first_found = 1U;

        /* 切换到任一目标模式后，统一进入搜索流程 */
        target_valid = 0;
        Reset_TargetPresence_State();
        Start_NormalSearch_ByModule(now_tick);

        g_voice_policy_state.prev_target_valid = 0U;
        g_voice_policy_state.prev_search_mode = Search_IsActive_FromModule();
    }
}

/*
 * 函数名：Reset_SystemVoice_State
 * 功能：重置系统语音相关状态
 *
 * 说明：
 * 1. 切换模式时清空待播语音，防止旧模式的状态语音延后插入
 * 2. 同时重置 PE14 测试按键状态，避免测试语音与系统语音互相干扰
 * 3. clear_found_prompt_flag = 1 时，允许新模式重新播报一次“已发现目标”
 */
void Reset_SystemVoice_State(uint8_t clear_found_prompt_flag)
{
    g_voice_policy_state.found_candidate_active = 0U;
    g_voice_policy_state.found_candidate_tick = 0U;
    g_voice_policy_state.lost_recovery_active = 0U;
    g_voice_policy_state.lost_search_prompt_played = 0U;
    g_voice_policy_state.mode_switch_search_only_until_tick = 0U;
    g_voice_policy_state.mode_switch_wait_for_first_found = 0U;

    SystemVoice_Reset(&g_system_voice_state, clear_found_prompt_flag);
    Reset_GuideVoice_State();

    if (clear_found_prompt_flag)
    {
        g_voice_policy_state.mode_found_prompt_played = 0U;
    }
}

/*
 * 函数名：Handle_SystemVoice_Update
 * 功能：根据业务状态边沿发布语音事件
 *
 * 说明：
 * 1. 本函数只负责“什么时候该提示”（业务层）
 * 2. system_voice 模块负责“现在能不能播、怎么排队和去重”（语音服务层）
 * 3. 搜索语音只在进入搜索状态时触发一次
 * 4. “已发现目标”需要稳定确认后触发，避免误检立即播报
 * 5. 正常首次发现：触发“已发现目标”；名称语音由 system_voice 继续排队
 * 6. 丢失后找回：只触发目标名称语音，不重复“已发现目标”
 */
void Handle_SystemVoice_Update(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint8_t search_active = Search_IsActive_FromModule();
    uint8_t mode_switch_guard_active =
        ((int32_t)(g_voice_policy_state.mode_switch_search_only_until_tick - now_tick) > 0) ? 1U : 0U;

    if (current_target_mode == TARGET_MODE_NONE)
    {
        /*
         * None 模式下不在这里反复 reset 语音服务层。
         * 原因：
         * 1. 切换到 None 时，按键分支已执行过 Reset_SystemVoice_State + MODE_STOPPED 事件
         * 2. 如果这里每轮都 reset，后续扩展为“入队后播放”的停止语音时会有被提前清空的风险
         */
        g_voice_policy_state.found_candidate_active = 0U;
        g_voice_policy_state.found_candidate_tick = 0U;
        g_voice_policy_state.lost_recovery_active = 0U;
        g_voice_policy_state.lost_search_prompt_played = 0U;
        g_voice_policy_state.mode_switch_search_only_until_tick = 0U;
        g_voice_policy_state.mode_switch_wait_for_first_found = 0U;

        g_voice_policy_state.prev_search_mode = search_active;
        g_voice_policy_state.prev_target_valid = target_valid;
        return;
    }

    if (!target_valid)
    {
        g_voice_policy_state.found_candidate_active = 0U;
        g_voice_policy_state.found_candidate_tick = 0U;
    }

    if ((!g_voice_policy_state.prev_search_mode) && search_active && !target_valid)
    {
        if (g_voice_policy_state.lost_recovery_active)
        {
            if (Search_IsLocalRecapture_FromModule() &&
                (!g_voice_policy_state.lost_search_prompt_played))
            {
                SystemVoice_PostEvent(&g_system_voice_state,
                                      VOICE_EVT_SEARCH_STARTED,
                                      (TargetMode_t)current_target_mode,
                                      now_tick);
                g_voice_policy_state.lost_search_prompt_played = 1U;
            }
        }
        else
        {
            SystemVoice_PostEvent(&g_system_voice_state,
                                  VOICE_EVT_SEARCH_STARTED,
                                  (TargetMode_t)current_target_mode,
                                  now_tick);
        }
    }

    if (mode_switch_guard_active)
    {
        g_voice_policy_state.found_candidate_active = 0U;
        g_voice_policy_state.found_candidate_tick = 0U;
    }
    else if ((!g_voice_policy_state.found_candidate_active) &&
             (((!g_voice_policy_state.prev_target_valid) && target_valid) ||
              (g_voice_policy_state.mode_switch_wait_for_first_found && target_valid)) &&
             (!g_voice_policy_state.mode_found_prompt_played ||
              g_voice_policy_state.lost_recovery_active))
    {
        g_voice_policy_state.found_candidate_active = 1U;
        g_voice_policy_state.found_candidate_tick = now_tick;
    }

    if (g_voice_policy_state.found_candidate_active &&
        target_valid &&
        (!g_voice_policy_state.mode_found_prompt_played ||
         g_voice_policy_state.lost_recovery_active))
    {
        if ((now_tick - g_voice_policy_state.found_candidate_tick) >= VOICE_FOUND_CONFIRM_MS)
        {
            g_voice_policy_state.found_candidate_active = 0U;
            g_voice_policy_state.found_candidate_tick = 0U;
            g_voice_policy_state.mode_switch_wait_for_first_found = 0U;

            if (g_voice_policy_state.lost_recovery_active)
            {
                SystemVoice_PostEvent(&g_system_voice_state,
                                      VOICE_EVT_TARGET_NAME,
                                      (TargetMode_t)current_target_mode,
                                      now_tick);
                guide_enable_after_tick = now_tick + VOICE_STATUS_GUARD_MS;
                g_voice_policy_state.lost_recovery_active = 0U;
            }
            else
            {
                SystemVoice_PostEvent(&g_system_voice_state,
                                      VOICE_EVT_TARGET_FOUND,
                                      (TargetMode_t)current_target_mode,
                                      now_tick);
                g_voice_policy_state.mode_found_prompt_played = 1U;
                guide_enable_after_tick = now_tick + GUIDE_BLOCK_AFTER_FOUND_MS;
            }
        }
    }

    SystemVoice_Update(&g_system_voice_state, now_tick);

    g_voice_policy_state.prev_search_mode = search_active;
    g_voice_policy_state.prev_target_valid = target_valid;
}

/*
 * 函数名：Handle_Voice_Test_Key
 * 功能：处理 PE14 语音测试按键的状态机
 *
 * 状态规则：
 * 1. 首次按下 -> 播放测试音频
 * 2. 第二次按下 -> 暂停
 * 3. 第三次按下 -> 继续
 *
 * 说明：
 * 1. 当前只做测试版，不与目标模式和跟随逻辑耦合。
 * 2. 测试音频默认复用厕所模式语音文件，便于单独验证 DFPlayer 链路。
 * 后续如果要扩展系统语音提示，优先从这里的业务调用入口开始整理即可。
 */
void Handle_Voice_Test_Key(void)
{
    SystemVoice_HandleTestKey(&g_system_voice_state);
}

float Limit_Float(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, float angle)
{
    float pulse;

    /* 统一做“逻辑角度 -> PWM 脉宽”映射。
     * 以后如果换舵机，优先修改配置区里的 SERVO_PULSE_MIN_US / SERVO_PULSE_MAX_US。
     */

    // 先限制在 0~180 度
    if (angle < 0.0f) angle = 0.0f;
    if (angle > SERVO_FULL_ANGLE_DEG) angle = SERVO_FULL_ANGLE_DEG;

    // 先用更保守的 1000~2000us 测试
    pulse = SERVO_PULSE_MIN_US +
            (angle / SERVO_FULL_ANGLE_DEG) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);

    /* 加上 0.5f 实现四舍五入，避免直接截断成 uint16 导致的单向（左侧）台阶量化误差叠加机械间隙造成的卡顿 */
    __HAL_TIM_SET_COMPARE(htim, channel, (uint16_t)(pulse + 0.5f));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (rx_data != '\r' && rx_data != '\n')
        {
            if (rx_index < (UART2_RX_LINE_LEN - 1U))
            {
                rx_line_build[rx_index++] = (char)rx_data;
            }
        }
        else
        {
            if (rx_index > 0)   // 防止连续收到 \r\n 时空触发
            {
                uint8_t write_idx = uart2_queue_write_idx;

                rx_line_build[rx_index] = '\0';
                strncpy(uart2_line_queue[write_idx], rx_line_build, UART2_RX_LINE_LEN - 1U);
                uart2_line_queue[write_idx][UART2_RX_LINE_LEN - 1U] = '\0';

                uart2_queue_write_idx = (uint8_t)((uart2_queue_write_idx + 1U) % UART2_RX_QUEUE_LEN);

                if (uart2_queue_count < UART2_RX_QUEUE_LEN)
                {
                    uart2_queue_count++;
                }
                else
                {
                    /* 队列满时丢弃最旧帧，保留最新数据 */
                    uart2_queue_read_idx = (uint8_t)((uart2_queue_read_idx + 1U) % UART2_RX_QUEUE_LEN);
                }

                rx_index = 0;
            }
        }

        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}


/*
 * 函数名：Handle_Buzzer_Update
 * 功能：基于当前相对角度执行“雷达式”连续节奏反馈（强反馈模式）
 */
void Handle_Buzzer_Update(void)
{
    uint32_t now_tick = HAL_GetTick();
    BuzzerZone_t current_zone = BUZZER_ZONE_NONE;
    float offset_deg;
    float abs_offset_deg;
    uint32_t on_ms = 0, off_ms = 0;
    uint8_t is_voice_busy = 0U;

    /* 0. 语音优先级检查：如果语音正在播报，大幅降低蜂鸣器频率，防止声音杂乱 */
    if (now_tick < SystemVoice_GetGuardUntilTick(&g_system_voice_state))
    {
        is_voice_busy = 1U;
    }

    /* 1. 过滤不响音的条件：无模式、搜索中、完全丢失目标 */
    if ((current_target_mode == TARGET_MODE_NONE) ||
        Search_IsActive_FromModule() ||
        (target_presence_state == TARGET_PRESENCE_LOST))
    {
        if (buzzer_is_on)
        {
            BUZZER_OFF();
            buzzer_is_on = 0U;
        }
        buzzer_last_zone = BUZZER_ZONE_NONE;
        return;
    }

    /* 1-B. HOLD 状态特殊反馈：表达“目标闪失，正在试图重合校准” */
    if (target_presence_state == TARGET_PRESENCE_HOLD)
    {
        on_ms = 10U;   /* 极短嘀嗒 */
        off_ms = 120U; /* 快速节奏 */
    }
    else
    {
        /* 2. 正常跟踪状态：判读当前处于哪个分区 */
        offset_deg = current_angle_x - GUIDE_CENTER_ANGLE_DEG;
        abs_offset_deg = (offset_deg >= 0.0f) ? offset_deg : (-offset_deg);

        if (abs_offset_deg < GUIDE_OFFSET_FORWARD_DEG)
        {
            current_zone = BUZZER_ZONE_CENTER; /* 锁定中心 */
            on_ms = BUZZER_RADAR_ON_CENTER;
            off_ms = BUZZER_RADAR_OFF_CENTER;
        }
        else if (abs_offset_deg < GUIDE_OFFSET_STRONG_DEG)
        {
            current_zone = BUZZER_ZONE_OUTER; /* 接近中心 */
            on_ms = BUZZER_RADAR_ON_OUTER;
            off_ms = BUZZER_RADAR_OFF_OUTER;
        }
        else
        {
            current_zone = BUZZER_ZONE_EDGE; /* 位于边缘 */
            on_ms = BUZZER_RADAR_ON_EDGE;
            off_ms = BUZZER_RADAR_OFF_EDGE;
        }
    }

    /* 3. 优先级抑制：如果语音忙，强制将间隔拉长到 1.5s 以上，保持静默压制 */
    if (is_voice_busy)
    {
        off_ms = 1500U;
    }

    /* 4. 执行非阻塞脉冲逻辑 */
    if (buzzer_is_on)
    {
        if ((now_tick - buzzer_state_tick) >= on_ms)
        {
            BUZZER_OFF();
            buzzer_is_on = 0U;
            buzzer_state_tick = now_tick;
        }
    }
    else
    {
        if ((now_tick - buzzer_state_tick) >= off_ms)
        {
            BUZZER_ON();
            buzzer_is_on = 1U;
            buzzer_state_tick = now_tick;
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
