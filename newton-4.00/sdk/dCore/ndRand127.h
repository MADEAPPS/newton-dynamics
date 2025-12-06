#ifndef TINYMT32_H
#define TINYMT32_H
/**
 * @file tinymt32.h
 *
 * @brief Tiny Mersenne Twister only 127 bit internal state
 *
 * @author Mutsuo Saito (Hiroshima University)
 * @author Makoto Matsumoto (University of Tokyo)
 *
 * Copyright (C) 2011 Mutsuo Saito, Makoto Matsumoto,
 * Hiroshima University and The University of Tokyo.
 * All rights reserved.
 *
 * The 3-clause BSD License is applied to this software, see
 * LICENSE.txt
 */

#include <stdint.h>
#include <inttypes.h>
#include "ndThreadSyncUtils.h"

 /**
  * tinymt32 internal state vector and parameters
  */

#define ND_USE_STD_RAND
class ndRand127
{
    public:
    D_CORE_API ndRand127();
    D_CORE_API void Init(uint32_t seed);

    /**
     * This function outputs 32-bit unsigned integer from internal state.
     * @param random tinymt internal status
     * @return 32-bit unsigned integer r (0 <= r < 2^32)
     */
    D_CORE_API uint32_t Generate();

    private:
#ifdef ND_USE_STD_RAND
    std::mt19937 m_gen;
#else
    uint32_t Temper();
    void NextState();
    void PeriodCertification();
    uint32_t status[4];
#endif
    ndSpinLock m_lock;
};


#endif
