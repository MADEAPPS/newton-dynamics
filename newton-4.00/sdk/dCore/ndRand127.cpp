/**
 * @file tinymt32.c
 *
 * @brief Tiny Mersenne Twister only 127 bit internal state
 *
 * @author Mutsuo Saito (Hiroshima University)
 * @author Makoto Matsumoto (The University of Tokyo)
 *
 * Copyright (C) 2011 Mutsuo Saito, Makoto Matsumoto,
 * Hiroshima University and The University of Tokyo.
 * All rights reserved.
 *
 * The 3-clause BSD License is applied to this software, see
 * LICENSE.txt
 */

#include "ndCoreStdafx.h"
#include "ndRand127.h"


#define MIN_LOOP 8
#define PRE_LOOP 8

#define TINYMT32_MEXP 127
#define TINYMT32_SH0 1
#define TINYMT32_SH1 10
#define TINYMT32_SH8 8
#define TINYMT32_MASK UINT32_C(0x7fffffff)

ndRand127::ndRand127()
{
    Init(uint32_t(47));
}

#ifdef ND_USE_STD_RAND

void ndRand127::Init(uint32_t seed)
{
    m_gen.seed(seed);
    ndUnsigned32 rand = m_gen();
}

uint32_t ndRand127::Generate()
{
    ndScopeSpinLock lock(m_lock);
    ndUnsigned32 rand = m_gen();
    return rand;
}

#else

 /**
  * This function initializes the internal state array with a 32-bit
  * unsigned integer seed.
  * @param random tinymt state vector.
  * @param seed a 32-bit unsigned integer used as a seed.
  */
void ndRand127::Init( uint32_t seed)
{
    for (ndInt32 i = 0; i < 4; ++i)
    {
        status[i] = 0;
    }
    status[0] = seed;
    for (unsigned int i = 1; i < MIN_LOOP; i++) 
    {
        status[i & 3] ^= i + UINT32_C(1812433253)
            * (status[(i - 1) & 3] ^ (status[(i - 1) & 3] >> 30));
    }

    PeriodCertification();
    for (unsigned int i = 0; i < PRE_LOOP; i++) 
    {
        NextState();
    }
}

/**
 * This function certificate the period of 2^127-1.
 * @param random tinymt state vector.
 */
void ndRand127::PeriodCertification()
{
    if ((status[0] & TINYMT32_MASK) == 0 &&
        status[1] == 0 && status[2] == 0 && status[3] == 0) 
    {
        status[0] = 'T';
        status[1] = 'I';
        status[2] = 'N';
        status[3] = 'Y';
    }
}

/**
 * This function changes internal state of tinymt32.
 * Users should not call this function directly.
 * @param random tinymt internal status
 */
void ndRand127::NextState()
{
    uint32_t x;
    uint32_t y;

    y = status[3];
    x = (status[0] & TINYMT32_MASK) ^ status[1] ^ status[2];
    x ^= (x << TINYMT32_SH0);
    y ^= (y >> TINYMT32_SH0) ^ x;
    status[0] = status[1];
    status[1] = status[2];
    status[2] = x ^ (y << TINYMT32_SH1);
    status[3] = y;
}

/**
 * This function outputs 32-bit unsigned integer from internal state.
 * Users should not call this function directly.
 * @param random tinymt internal status
 * @return 32-bit unsigned pseudorandom number
 */
uint32_t ndRand127::Temper()
{
    uint32_t t0, t1;
    t0 = status[3];
    t1 = status[0] + (status[2] >> TINYMT32_SH8);
    t0 ^= t1;
    //if ((t1 & 1) != 0) 
    //{
    //    t0 ^= tmat;
    //}
    return t0;
}

uint32_t ndRand127::Generate()
{
    ndScopeSpinLock lock(m_lock);
    NextState();
    return Temper();
}
#endif
