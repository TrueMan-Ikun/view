#include <string.h>
#include "dfplayer.h"
#include "system_voice.h"

static uint16_t SystemVoice_GetPromptIndex(const SystemVoiceState *state,
                                           SystemVoiceEvent_t event,
                                           TargetMode_t mode);
static uint32_t SystemVoice_GetGuardMs(const SystemVoiceState *state,
                                       SystemVoiceEvent_t event);
static uint8_t SystemVoice_IsGuideEvent(SystemVoiceEvent_t event);
static void SystemVoice_ClearQueue(SystemVoiceState *state);
static void SystemVoice_RemovePendingSearch(SystemVoiceState *state);
static void SystemVoice_RemovePendingGuide(SystemVoiceState *state);
static void SystemVoice_Enqueue(SystemVoiceState *state,
                                uint16_t prompt_index,
                                SystemVoiceEvent_t event,
                                TargetMode_t mode,
                                uint32_t due_tick);
static void SystemVoice_PlayNow(SystemVoiceState *state,
                                uint16_t prompt_index,
                                uint32_t guard_ms);

void SystemVoice_Init(SystemVoiceState *state, const SystemVoiceConfig *config)
{
    if ((state == NULL) || (config == NULL))
    {
        return;
    }

    memset(state, 0, sizeof(*state));
    memcpy(&state->config, config, sizeof(*config));
}

void SystemVoice_Reset(SystemVoiceState *state, uint8_t reset_found_cycle)
{
    if (state == NULL)
    {
        return;
    }

    (void)reset_found_cycle;
    SystemVoice_ClearQueue(state);
    state->guard_until_tick = 0U;
    state->test_started = 0U;
    state->test_paused = 0U;
}

void SystemVoice_PostEvent(SystemVoiceState *state,
                           SystemVoiceEvent_t event,
                           TargetMode_t mode,
                           uint32_t now_tick)
{
    uint16_t prompt_index;
    uint32_t due_tick;

    if (state == NULL)
    {
        return;
    }

    prompt_index = SystemVoice_GetPromptIndex(state, event, mode);

    if (prompt_index == 0U)
    {
        return;
    }

    due_tick = now_tick;
    if ((int32_t)(state->guard_until_tick - due_tick) > 0)
    {
        due_tick = state->guard_until_tick;
    }

    switch (event)
    {
        case VOICE_EVT_MODE_STOPPED:
            SystemVoice_ClearQueue(state);
            SystemVoice_PlayNow(state, prompt_index, state->config.mode_guard_ms);
            break;

        case VOICE_EVT_TARGET_LOST:
            SystemVoice_ClearQueue(state);
            SystemVoice_PlayNow(state, prompt_index, state->config.lost_prompt_guard_ms);
            break;

        case VOICE_EVT_SEARCH_STARTED:
            SystemVoice_Enqueue(state, prompt_index, event, mode, due_tick);
            break;

        case VOICE_EVT_TARGET_FOUND:
            SystemVoice_RemovePendingSearch(state);
            SystemVoice_Enqueue(state, prompt_index, event, mode, due_tick);
            prompt_index = SystemVoice_GetPromptIndex(state, VOICE_EVT_TARGET_NAME, mode);
            if (prompt_index != 0U)
            {
                SystemVoice_Enqueue(state,
                                    prompt_index,
                                    VOICE_EVT_TARGET_NAME,
                                    mode,
                                    due_tick + state->config.target_prompt_delay_ms);
            }
            break;

        case VOICE_EVT_TARGET_NAME:
            SystemVoice_RemovePendingSearch(state);
            SystemVoice_Enqueue(state, prompt_index, event, mode, due_tick);
            break;

        case VOICE_EVT_GUIDE_LEFT:
        case VOICE_EVT_GUIDE_RIGHT:
        case VOICE_EVT_GUIDE_SLIGHT_LEFT:
        case VOICE_EVT_GUIDE_SLIGHT_RIGHT:
        case VOICE_EVT_GUIDE_FACING_TARGET:
        case VOICE_EVT_GUIDE_FORWARD:
        case VOICE_EVT_GUIDE_CONTINUE_FORWARD:
        case VOICE_EVT_GUIDE_NEAR_TARGET:
        case VOICE_EVT_GUIDE_ARRIVED:
            SystemVoice_RemovePendingGuide(state);
            SystemVoice_Enqueue(state, prompt_index, event, mode, due_tick);
            break;

        default:
            break;
    }
}

void SystemVoice_Update(SystemVoiceState *state, uint32_t now_tick)
{
    SystemVoicePendingItem item;
    uint8_t i;

    if ((state == NULL) || (state->queue_count == 0U))
    {
        return;
    }

    if ((int32_t)(now_tick - state->queue[0].due_tick) < 0)
    {
        return;
    }

    item = state->queue[0];
    for (i = 1U; i < state->queue_count; i++)
    {
        state->queue[i - 1U] = state->queue[i];
    }
    state->queue_count--;

    SystemVoice_PlayNow(state,
                        item.prompt_index,
                        SystemVoice_GetGuardMs(state, item.event));
}

void SystemVoice_HandleTestKey(SystemVoiceState *state)
{
    if (state == NULL)
    {
        return;
    }

    if (state->config.test_prompt_index == 0U)
    {
        return;
    }

    if (!state->test_started)
    {
        DFPlayer_PlayMp3(state->config.test_prompt_index);
        state->test_started = 1U;
        state->test_paused = 0U;
        return;
    }

    if (!state->test_paused)
    {
        DFPlayer_Pause();
        state->test_paused = 1U;
    }
    else
    {
        DFPlayer_Resume();
        state->test_paused = 0U;
    }
}

uint32_t SystemVoice_GetGuardUntilTick(const SystemVoiceState *state)
{
    if (state == NULL)
    {
        return 0U;
    }

    return state->guard_until_tick;
}

static uint16_t SystemVoice_GetPromptIndex(const SystemVoiceState *state,
                                           SystemVoiceEvent_t event,
                                           TargetMode_t mode)
{
    if (state == NULL)
    {
        return 0U;
    }

    switch (event)
    {
        case VOICE_EVT_MODE_STOPPED:
            return state->config.mode_none_prompt_index;

        case VOICE_EVT_SEARCH_STARTED:
            return state->config.searching_prompt_index;

        case VOICE_EVT_TARGET_FOUND:
            return state->config.target_found_prompt_index;

        case VOICE_EVT_TARGET_NAME:
            if ((uint32_t)mode < TARGET_MODE_COUNT)
            {
                return state->config.target_prompt_indices[mode];
            }
            return 0U;

        case VOICE_EVT_TARGET_LOST:
            return state->config.target_lost_prompt_index;

        case VOICE_EVT_GUIDE_LEFT:
            return state->config.guide_left_prompt_index;

        case VOICE_EVT_GUIDE_RIGHT:
            return state->config.guide_right_prompt_index;

        case VOICE_EVT_GUIDE_SLIGHT_LEFT:
            return state->config.guide_slight_left_prompt_index;

        case VOICE_EVT_GUIDE_SLIGHT_RIGHT:
            return state->config.guide_slight_right_prompt_index;

        case VOICE_EVT_GUIDE_FACING_TARGET:
            return state->config.guide_facing_prompt_index;

        case VOICE_EVT_GUIDE_FORWARD:
            return state->config.guide_forward_prompt_index;

        case VOICE_EVT_GUIDE_CONTINUE_FORWARD:
            return state->config.guide_continue_forward_prompt_index;

        case VOICE_EVT_GUIDE_NEAR_TARGET:
            return state->config.guide_near_target_prompt_index;

        case VOICE_EVT_GUIDE_ARRIVED:
            return state->config.guide_arrived_prompt_index;

        default:
            return 0U;
    }
}

static uint32_t SystemVoice_GetGuardMs(const SystemVoiceState *state,
                                       SystemVoiceEvent_t event)
{
    if (state == NULL)
    {
        return 0U;
    }

    switch (event)
    {
        case VOICE_EVT_MODE_STOPPED:
            return state->config.mode_guard_ms;

        case VOICE_EVT_TARGET_LOST:
            return state->config.lost_prompt_guard_ms;

        case VOICE_EVT_GUIDE_LEFT:
        case VOICE_EVT_GUIDE_RIGHT:
        case VOICE_EVT_GUIDE_SLIGHT_LEFT:
        case VOICE_EVT_GUIDE_SLIGHT_RIGHT:
        case VOICE_EVT_GUIDE_FACING_TARGET:
        case VOICE_EVT_GUIDE_FORWARD:
        case VOICE_EVT_GUIDE_CONTINUE_FORWARD:
        case VOICE_EVT_GUIDE_NEAR_TARGET:
        case VOICE_EVT_GUIDE_ARRIVED:
            return 700U;

        default:
            return state->config.status_guard_ms;
    }
}

static uint8_t SystemVoice_IsGuideEvent(SystemVoiceEvent_t event)
{
    switch (event)
    {
        case VOICE_EVT_GUIDE_LEFT:
        case VOICE_EVT_GUIDE_RIGHT:
        case VOICE_EVT_GUIDE_SLIGHT_LEFT:
        case VOICE_EVT_GUIDE_SLIGHT_RIGHT:
        case VOICE_EVT_GUIDE_FACING_TARGET:
        case VOICE_EVT_GUIDE_FORWARD:
        case VOICE_EVT_GUIDE_CONTINUE_FORWARD:
        case VOICE_EVT_GUIDE_NEAR_TARGET:
        case VOICE_EVT_GUIDE_ARRIVED:
            return 1U;

        default:
            return 0U;
    }
}

static void SystemVoice_ClearQueue(SystemVoiceState *state)
{
    if (state == NULL)
    {
        return;
    }

    state->queue_count = 0U;
    memset(state->queue, 0, sizeof(state->queue));
}

static void SystemVoice_RemovePendingSearch(SystemVoiceState *state)
{
    uint8_t src;
    uint8_t dst = 0U;

    if (state == NULL)
    {
        return;
    }

    for (src = 0U; src < state->queue_count; src++)
    {
        if (state->queue[src].event == VOICE_EVT_SEARCH_STARTED)
        {
            continue;
        }

        if (dst != src)
        {
            state->queue[dst] = state->queue[src];
        }
        dst++;
    }

    state->queue_count = dst;
}

static void SystemVoice_RemovePendingGuide(SystemVoiceState *state)
{
    uint8_t src;
    uint8_t dst = 0U;

    if (state == NULL)
    {
        return;
    }

    for (src = 0U; src < state->queue_count; src++)
    {
        if (SystemVoice_IsGuideEvent(state->queue[src].event))
        {
            continue;
        }

        if (dst != src)
        {
            state->queue[dst] = state->queue[src];
        }
        dst++;
    }

    state->queue_count = dst;
}

static void SystemVoice_Enqueue(SystemVoiceState *state,
                                uint16_t prompt_index,
                                SystemVoiceEvent_t event,
                                TargetMode_t mode,
                                uint32_t due_tick)
{
    uint8_t index;

    if ((state == NULL) || (prompt_index == 0U))
    {
        return;
    }

    for (index = 0U; index < state->queue_count; index++)
    {
        if (state->queue[index].event == event)
        {
            state->queue[index].prompt_index = prompt_index;
            state->queue[index].mode = mode;
            state->queue[index].due_tick = due_tick;
            return;
        }
    }

    if (state->queue_count >= SYSTEM_VOICE_QUEUE_SIZE)
    {
        return;
    }

    state->queue[state->queue_count].prompt_index = prompt_index;
    state->queue[state->queue_count].event = event;
    state->queue[state->queue_count].mode = mode;
    state->queue[state->queue_count].due_tick = due_tick;
    state->queue_count++;
}

static void SystemVoice_PlayNow(SystemVoiceState *state,
                                uint16_t prompt_index,
                                uint32_t guard_ms)
{
    if ((state == NULL) || (prompt_index == 0U))
    {
        return;
    }

    DFPlayer_PlayMp3(prompt_index);
    state->test_started = 0U;
    state->test_paused = 0U;
    state->guard_until_tick = HAL_GetTick() + guard_ms;
}
