#ifndef __SYSTEM_VOICE_H
#define __SYSTEM_VOICE_H

#include "target_mode.h"

typedef enum
{
    VOICE_EVT_NONE = 0,
    VOICE_EVT_MODE_STOPPED,
    VOICE_EVT_SEARCH_STARTED,
    VOICE_EVT_TARGET_FOUND,
    VOICE_EVT_TARGET_NAME,
    VOICE_EVT_TARGET_LOST,
    VOICE_EVT_GUIDE_LEFT,
    VOICE_EVT_GUIDE_RIGHT,
    VOICE_EVT_GUIDE_SLIGHT_LEFT,
    VOICE_EVT_GUIDE_SLIGHT_RIGHT,
    VOICE_EVT_GUIDE_FACING_TARGET,
    VOICE_EVT_GUIDE_FORWARD,
    VOICE_EVT_GUIDE_CONTINUE_FORWARD,
    VOICE_EVT_GUIDE_NEAR_TARGET,
    VOICE_EVT_GUIDE_ARRIVED
} SystemVoiceEvent_t;

typedef struct
{
    uint16_t target_prompt_indices[TARGET_MODE_COUNT];
    uint16_t searching_prompt_index;
    uint16_t target_found_prompt_index;
    uint16_t target_lost_prompt_index;
    uint16_t mode_none_prompt_index;
    uint16_t guide_left_prompt_index;
    uint16_t guide_right_prompt_index;
    uint16_t guide_slight_left_prompt_index;
    uint16_t guide_slight_right_prompt_index;
    uint16_t guide_facing_prompt_index;
    uint16_t guide_forward_prompt_index;
    uint16_t guide_continue_forward_prompt_index;
    uint16_t guide_near_target_prompt_index;
    uint16_t guide_arrived_prompt_index;
    uint16_t test_prompt_index;
    uint32_t mode_guard_ms;
    uint32_t status_guard_ms;
    uint32_t target_prompt_delay_ms;
    uint32_t lost_prompt_guard_ms;
} SystemVoiceConfig;

typedef struct
{
    uint16_t prompt_index;
    SystemVoiceEvent_t event;
    TargetMode_t mode;
    uint32_t due_tick;
} SystemVoicePendingItem;

#define SYSTEM_VOICE_QUEUE_SIZE 4U

typedef struct
{
    SystemVoiceConfig config;
    SystemVoicePendingItem queue[SYSTEM_VOICE_QUEUE_SIZE];
    uint8_t queue_count;
    uint32_t guard_until_tick;
    uint8_t test_started;
    uint8_t test_paused;
} SystemVoiceState;

void SystemVoice_Init(SystemVoiceState *state, const SystemVoiceConfig *config);
void SystemVoice_Reset(SystemVoiceState *state, uint8_t reset_found_cycle);
void SystemVoice_PostEvent(SystemVoiceState *state,
                           SystemVoiceEvent_t event,
                           TargetMode_t mode,
                           uint32_t now_tick);
void SystemVoice_Update(SystemVoiceState *state, uint32_t now_tick);
void SystemVoice_HandleTestKey(SystemVoiceState *state);
uint32_t SystemVoice_GetGuardUntilTick(const SystemVoiceState *state);

#endif /* __SYSTEM_VOICE_H */
