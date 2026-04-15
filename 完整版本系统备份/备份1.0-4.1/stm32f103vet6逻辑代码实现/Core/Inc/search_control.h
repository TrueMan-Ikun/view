#ifndef __SEARCH_CONTROL_H
#define __SEARCH_CONTROL_H

#include "target_mode.h"

typedef enum
{
    SEARCH_PHASE_NONE = 0,
    SEARCH_PHASE_LOCAL_RECAPTURE,
    SEARCH_PHASE_GLOBAL_RECAPTURE
} SearchPhase_t;

typedef struct
{
    float left_limit;
    float right_limit;
    float fixed_y_angle;
    float normal_step_deg;
    uint32_t normal_update_ms;
    uint32_t start_delay_ms;

    float direction_epsilon_pixels;

    float phase1_base_amplitude_deg;
    float phase1_error_gain_deg;
    float phase1_time_gain_deg_per_s;
    float phase1_min_amplitude_deg;
    float phase1_max_amplitude_deg;
    float phase1_step_deg;
    uint32_t phase1_update_ms;
    uint32_t phase1_timeout_ms;

    float phase2_step_base_deg;
    float phase2_step_gain_deg_per_s;
    float phase2_step_min_deg;
    float phase2_step_max_deg;
    float phase2_left_angle;
    float phase2_right_angle;
} SearchControlConfig;

typedef struct
{
    SearchPhase_t phase;
    float angle_x;
    int8_t direction;
    uint32_t last_tick;
    uint32_t start_tick;
    uint32_t phase_start_tick;
    float center_x;
    float last_error_abs;
    float phase1_left_angle;
    float phase1_right_angle;
    float phase2_step_deg;
    uint8_t lost_search_prompt_played;
} SearchControlState;

void SearchControl_Reset(SearchControlState *state, const SearchControlConfig *config, float current_angle_x);
void SearchControl_StartModeSearch(SearchControlState *state,
                                   const SearchControlConfig *config,
                                   uint32_t now_tick,
                                   float current_angle_x,
                                   float last_x_error);
void SearchControl_StartRecapture(SearchControlState *state,
                                  const SearchControlConfig *config,
                                  uint32_t now_tick,
                                  float current_angle_x,
                                  float last_x_error);
void SearchControl_OnTargetRecovered(SearchControlState *state,
                                     const SearchControlConfig *config,
                                     float current_angle_x);
int8_t SearchControl_GetDirectionFromError(const SearchControlConfig *config, float x_error);
uint8_t SearchControl_ShouldPlayLocalSearchPrompt(const SearchControlState *state);
void SearchControl_MarkLocalSearchPromptPlayed(SearchControlState *state);
uint8_t SearchControl_IsActive(const SearchControlState *state);
SearchPhase_t SearchControl_GetPhase(const SearchControlState *state);
uint8_t SearchControl_IsLocalRecapture(const SearchControlState *state);
uint8_t SearchControl_IsGlobalRecapture(const SearchControlState *state);
uint8_t SearchControl_Update(SearchControlState *state,
                             const SearchControlConfig *config,
                             uint32_t now_tick,
                             uint8_t target_valid,
                             TargetMode_t current_mode,
                             float *target_angle_x,
                             float *target_angle_y);

#endif /* __SEARCH_CONTROL_H */
