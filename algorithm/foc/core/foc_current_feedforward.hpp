#ifndef _FOC_CURRENT_FEEDFORWARD_HPP
#define _FOC_CURRENT_FEEDFORWARD_HPP

#include <stdint.h>

#include "foc_type.h"

/* Values 0/1 preserve the existing telemetry contract for FF OFF/ON. */
typedef enum
{
    CURRENT_FF_MODE_OFF = 0U,
    CURRENT_FF_MODE_BOTH = 1U,
    CURRENT_FF_MODE_CROSS = 2U,
    CURRENT_FF_MODE_BEMF = 3U,
} CURRENT_FEEDFORWARD_MODE_t;

static inline qd_t FOC_CurrentFeedforward(
    uint32_t mode, float omega_e, float ld, float lq, float flux, qd_t measured_current)
{
    qd_t output = {0.0f, 0.0f};
    const bool cross_enabled = (mode == (uint32_t)CURRENT_FF_MODE_CROSS)
                               || (mode == (uint32_t)CURRENT_FF_MODE_BOTH);
    const bool bemf_enabled = (mode == (uint32_t)CURRENT_FF_MODE_BEMF)
                              || (mode == (uint32_t)CURRENT_FF_MODE_BOTH);

    if(cross_enabled){
        output.q = omega_e*ld*measured_current.d;
        output.d = -omega_e*lq*measured_current.q;
    }
    if(bemf_enabled){
        output.q += omega_e*flux;
    }

    return output;
}

#endif
