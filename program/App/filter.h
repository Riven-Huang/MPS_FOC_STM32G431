#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

typedef struct
{
    float alpha;
    float value;
    uint8_t initialized;
} filter_lpf_f32_t;

typedef struct
{
    int32_t alpha_q15;
    int32_t value_q15;
    uint8_t initialized;
} filter_lpf_s32_t;

void filter_lpf_f32_init(filter_lpf_f32_t *filter, float alpha, float initial_value);
float filter_lpf_f32_update(filter_lpf_f32_t *filter, float input);

void filter_lpf_s32_init(filter_lpf_s32_t *filter, int32_t alpha_q15, int32_t initial_value);
int32_t filter_lpf_s32_update(filter_lpf_s32_t *filter, int32_t input);

#endif /* FILTER_H */
