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

 /**
  * tinymt32 internal state vector and parameters
  */
class TINYMT32_T
{
    void Init(uint32_t seed);

    /**
     * This function outputs 32-bit unsigned integer from internal state.
     * @param random tinymt internal status
     * @return 32-bit unsigned integer r (0 <= r < 2^32)
     */
    uint32_t Generate();

    private:
    uint32_t Temper();
    void NextState();
    void PeriodCertification();

    uint32_t status[4];
    uint32_t mat1;
    uint32_t mat2;
    uint32_t tmat;
};
#endif
