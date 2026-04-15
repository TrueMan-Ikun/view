#include "search_control.h"

static float SearchControl_LimitFloat(float value, float min, float max);
static void SearchControl_UpdateLocalWindow(SearchControlState *state,
                                            const SearchControlConfig *config,
                                            uint32_t now_tick,
                                            float last_x_error);

void SearchControl_Reset(SearchControlState *state, const SearchControlConfig *config, float current_angle_x)
{
    if ((state == NULL) || (config == NULL))
    {
        return;
    }

    state->phase = SEARCH_PHASE_NONE;
    state->angle_x = current_angle_x;
    state->direction = 1;
    state->last_tick = 0U;
    state->start_tick = 0U;
    state->phase_start_tick = 0U;
    state->center_x = current_angle_x;
    state->last_error_abs = 0.0f;
    state->phase1_left_angle = current_angle_x;
    state->phase1_right_angle = current_angle_x;
    state->phase2_step_deg = config->phase2_step_base_deg;
    state->lost_search_prompt_played = 0U;
}

void SearchControl_StartModeSearch(SearchControlState *state,
                                   const SearchControlConfig *config,
                                   uint32_t now_tick,
                                   float current_angle_x,
                                   float last_x_error)
{
    if ((state == NULL) || (config == NULL))
    {
        return;
    }

    SearchControl_Reset(state, config, current_angle_x);
    state->start_tick = now_tick;
    state->last_tick = now_tick;
    state->direction = SearchControl_GetDirectionFromError(config, last_x_error);
    state->last_error_abs = (last_x_error >= 0.0f) ? last_x_error : (-last_x_error);
}

void SearchControl_StartRecapture(SearchControlState *state,
                                  const SearchControlConfig *config,
                                  uint32_t now_tick,
                                  float current_angle_x,
                                  float last_x_error)
{
    if ((state == NULL) || (config == NULL))
    {
        return;
    }

    SearchControl_Reset(state, config, current_angle_x);
    state->phase = SEARCH_PHASE_LOCAL_RECAPTURE;
    state->start_tick = now_tick;
    state->last_tick = now_tick;
    state->phase_start_tick = now_tick;
    state->center_x = current_angle_x;
    state->direction = SearchControl_GetDirectionFromError(config, last_x_error);
    state->last_error_abs = (last_x_error >= 0.0f) ? last_x_error : (-last_x_error);
}

void SearchControl_OnTargetRecovered(SearchControlState *state,
                                     const SearchControlConfig *config,
                                     float current_angle_x)
{
    SearchControl_Reset(state, config, current_angle_x);
}

int8_t SearchControl_GetDirectionFromError(const SearchControlConfig *config, float x_error)
{
    if ((config != NULL) && (x_error > config->direction_epsilon_pixels))
    {
        return 1;
    }

    if ((config != NULL) && (x_error < (-config->direction_epsilon_pixels)))
    {
        return -1;
    }

    return 1;
}

uint8_t SearchControl_ShouldPlayLocalSearchPrompt(const SearchControlState *state)
{
    if (state == NULL)
    {
        return 0U;
    }

    return (uint8_t)((state->phase == SEARCH_PHASE_LOCAL_RECAPTURE) &&
                     (!state->lost_search_prompt_played));
}

void SearchControl_MarkLocalSearchPromptPlayed(SearchControlState *state)
{
    if (state == NULL)
    {
        return;
    }

    state->lost_search_prompt_played = 1U;
}

uint8_t SearchControl_IsActive(const SearchControlState *state)
{
    if (state == NULL)
    {
        return 0U;
    }

    return (state->start_tick != 0U) ? 1U : 0U;
}

SearchPhase_t SearchControl_GetPhase(const SearchControlState *state)
{
    if (state == NULL)
    {
        return SEARCH_PHASE_NONE;
    }

    return state->phase;
}

uint8_t SearchControl_IsLocalRecapture(const SearchControlState *state)
{
    return (SearchControl_GetPhase(state) == SEARCH_PHASE_LOCAL_RECAPTURE) ? 1U : 0U;
}

uint8_t SearchControl_IsGlobalRecapture(const SearchControlState *state)
{
    return (SearchControl_GetPhase(state) == SEARCH_PHASE_GLOBAL_RECAPTURE) ? 1U : 0U;
}

uint8_t SearchControl_Update(SearchControlState *state,
                             const SearchControlConfig *config,
                             uint32_t now_tick,
                             uint8_t target_valid,
                             TargetMode_t current_mode,
                             float *target_angle_x,
                             float *target_angle_y)
{
    float step_deg;
    float search_left_limit;
    float search_right_limit;
    uint32_t update_interval;
    float lost_seconds;

    if ((state == NULL) || (config == NULL) ||
        (target_angle_x == NULL) || (target_angle_y == NULL))
    {
        return 0U;
    }

    if (target_valid || (current_mode == TARGET_MODE_NONE))
    {
        return 0U;
    }

    if ((now_tick - state->start_tick) <= config->start_delay_ms)
    {
        *target_angle_x = state->angle_x;
        *target_angle_y = config->fixed_y_angle;
        return 1U;
    }

    update_interval = config->normal_update_ms;
    step_deg = config->normal_step_deg;
    search_left_limit = config->left_limit;
    search_right_limit = config->right_limit;

    if (state->phase == SEARCH_PHASE_LOCAL_RECAPTURE)
    {
        SearchControl_UpdateLocalWindow(state, config, now_tick, state->last_error_abs);
        search_left_limit = state->phase1_left_angle;
        search_right_limit = state->phase1_right_angle;
        step_deg = config->phase1_step_deg;
        update_interval = config->phase1_update_ms;

        if ((now_tick - state->phase_start_tick) >= config->phase1_timeout_ms)
        {
            state->phase = SEARCH_PHASE_GLOBAL_RECAPTURE;
        }
    }

    if (state->phase == SEARCH_PHASE_GLOBAL_RECAPTURE)
    {
        lost_seconds = 0.0f;

        if (state->phase_start_tick != 0U)
        {
            lost_seconds = (float)(now_tick - state->phase_start_tick) / 1000.0f;
        }

        state->phase2_step_deg = config->phase2_step_base_deg +
                                 (config->phase2_step_gain_deg_per_s * lost_seconds);
        state->phase2_step_deg = SearchControl_LimitFloat(state->phase2_step_deg,
                                                          config->phase2_step_min_deg,
                                                          config->phase2_step_max_deg);

        step_deg = state->phase2_step_deg;
        update_interval = config->normal_update_ms;
        search_left_limit = config->phase2_left_angle;
        search_right_limit = config->phase2_right_angle;
    }

    state->angle_x = SearchControl_LimitFloat(state->angle_x,
                                              search_left_limit,
                                              search_right_limit);

    if ((now_tick - state->last_tick) < update_interval)
    {
        *target_angle_x = state->angle_x;
        *target_angle_y = config->fixed_y_angle;
        return 1U;
    }

    state->last_tick = now_tick;
    state->angle_x += step_deg * state->direction;

    if (state->angle_x >= search_right_limit)
    {
        state->angle_x = search_right_limit;
        state->direction = -1;
    }
    else if (state->angle_x <= search_left_limit)
    {
        state->angle_x = search_left_limit;
        state->direction = 1;
    }

    *target_angle_x = state->angle_x;
    *target_angle_y = config->fixed_y_angle;
    return 1U;
}

static float SearchControl_LimitFloat(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }

    if (value > max)
    {
        return max;
    }

    return value;
}

static void SearchControl_UpdateLocalWindow(SearchControlState *state,
                                            const SearchControlConfig *config,
                                            uint32_t now_tick,
                                            float last_x_error)
{
    float lost_seconds = 0.0f;
    float error_ratio = 0.0f;
    float amplitude = config->phase1_base_amplitude_deg;

    if ((state == NULL) || (config == NULL))
    {
        return;
    }

    if (state->phase_start_tick != 0U)
    {
        lost_seconds = (float)(now_tick - state->phase_start_tick) / 1000.0f;
    }

    if (last_x_error < 0.0f)
    {
        last_x_error = -last_x_error;
    }

    error_ratio = last_x_error / 160.0f;

    amplitude += (config->phase1_error_gain_deg * error_ratio);
    amplitude += (config->phase1_time_gain_deg_per_s * lost_seconds);
    amplitude = SearchControl_LimitFloat(amplitude,
                                         config->phase1_min_amplitude_deg,
                                         config->phase1_max_amplitude_deg);

    state->phase1_left_angle = SearchControl_LimitFloat(state->center_x - amplitude,
                                                        config->left_limit,
                                                        config->right_limit);
    state->phase1_right_angle = SearchControl_LimitFloat(state->center_x + amplitude,
                                                         config->left_limit,
                                                         config->right_limit);

    if (state->phase1_left_angle > state->phase1_right_angle)
    {
        state->phase1_left_angle = state->phase1_right_angle;
    }
}
