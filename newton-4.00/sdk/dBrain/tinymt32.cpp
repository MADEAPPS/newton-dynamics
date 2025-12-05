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

#include "ndBrainStdafx.h"

#include "tinymt32.h"
#define MIN_LOOP 8
#define PRE_LOOP 8

#define TINYMT32_MEXP 127
#define TINYMT32_SH0 1
#define TINYMT32_SH1 10
#define TINYMT32_SH8 8
#define TINYMT32_MASK UINT32_C(0x7fffffff)
#define TINYMT32_MUL (1.0f / 16777216.0f)


 /**
  * This function initializes the internal state array with a 32-bit
  * unsigned integer seed.
  * @param random tinymt state vector.
  * @param seed a 32-bit unsigned integer used as a seed.
  */
void TINYMT32_T::tinymt32_init( uint32_t seed)
{
    status[0] = seed;
    status[1] = mat1;
    status[2] = mat2;
    status[3] = tmat;
    for (unsigned int i = 1; i < MIN_LOOP; i++) 
    {
        status[i & 3] ^= i + UINT32_C(1812433253)
            * (status[(i - 1) & 3] ^ (status[(i - 1) & 3] >> 30));
    }

    period_certification();
    for (unsigned int i = 0; i < PRE_LOOP; i++) 
    {
        tinymt32_next_state();
    }
}

/**
 * This function certificate the period of 2^127-1.
 * @param random tinymt state vector.
 */
void TINYMT32_T::period_certification()
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
void TINYMT32_T::tinymt32_next_state()
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
    int32_t const a = -((int32_t)(y & 1)) & (int32_t)mat1;
    int32_t const b = -((int32_t)(y & 1)) & (int32_t)mat2;
    status[1] ^= (uint32_t)a;
    status[2] ^= (uint32_t)b;
}

#if 0
void tinymt32_init_by_array(tinymt32_t* random, uint32_t init_key[],
    int key_length);


 //inline static int tinymt32_get_mexp(tinymt32_t* random) 
inline static int tinymt32_get_mexp(tinymt32_t*)
{
    return TINYMT32_MEXP;
}


/**
 * This function represents a function used in the initialization
 * by init_by_array
 * @param x 32-bit integer
 * @return 32-bit integer
 */
static uint32_t ini_func1(uint32_t x) 
{
    return (x ^ (x >> 27)) * UINT32_C(1664525);
}

/**
 * This function represents a function used in the initialization
 * by init_by_array
 * @param x 32-bit integer
 * @return 32-bit integer
 */
static uint32_t ini_func2(uint32_t x) {
    return (x ^ (x >> 27)) * UINT32_C(1566083941);
}



/**
 * This function initializes the internal state array,
 * with an array of 32-bit unsigned integers used as seeds
 * @param random tinymt state vector.
 * @param init_key the array of 32-bit integers, used as a seed.
 * @param key_length the length of init_key.
 */
void tinymt32_init_by_array(tinymt32_t * random, uint32_t init_key[],
                            int key_length) {
    const unsigned int lag = 1;
    const unsigned int mid = 1;
    const unsigned int size = 4;
    unsigned int i, j;
    unsigned int count;
    uint32_t r;
    uint32_t * st = &random->status[0];

    st[0] = 0;
    st[1] = random->mat1;
    st[2] = random->mat2;
    st[3] = random->tmat;
    if (key_length + 1 > MIN_LOOP) {
        count = (unsigned int)key_length + 1;
    } else {
        count = MIN_LOOP;
    }
    r = ini_func1(st[0] ^ st[mid % size]
                  ^ st[(size - 1) % size]);
    st[mid % size] += r;
    r += (unsigned int)key_length;
    st[(mid + lag) % size] += r;
    st[0] = r;
    count--;
    for (i = 1, j = 0; (j < count) && (j < (unsigned int)key_length); j++) {
        r = ini_func1(st[i % size]
                      ^ st[(i + mid) % size]
                      ^ st[(i + size - 1) % size]);
        st[(i + mid) % size] += r;
        r += init_key[j] + i;
        st[(i + mid + lag) % size] += r;
        st[i % size] = r;
        i = (i + 1) % size;
    }
    for (; j < count; j++) {
        r = ini_func1(st[i % size]
                      ^ st[(i + mid) % size]
                      ^ st[(i + size - 1) % size]);
        st[(i + mid) % size] += r;
        r += i;
        st[(i + mid + lag) % size] += r;
        st[i % size] = r;
        i = (i + 1) % size;
    }
    for (j = 0; j < size; j++) {
        r = ini_func2(st[i % size]
                      + st[(i + mid) % size]
                      + st[(i + size - 1) % size]);
        st[(i + mid) % size] ^= r;
        r -= i;
        st[(i + mid + lag) % size] ^= r;
        st[i % size] = r;
        i = (i + 1) % size;
    }
    period_certification(random);
    for (i = 0; i < PRE_LOOP; i++) 
    {
        tinymt32_next_state(random);
    }
}

/**
 * This function outputs floating point number from internal state.
 * Users should not call this function directly.
 * @param random tinymt internal status
 * @return floating point number r (1.0 <= r < 2.0)
 */
inline static float tinymt32_temper_conv(tinymt32_t* random)
{
    uint32_t t0, t1;
    union
    {
        uint32_t u;
        float f;
    } conv;

    t0 = random->status[3];
#if defined(LINEARITY_CHECK)
    t1 = random->status[0]
        ^ (random->status[2] >> TINYMT32_SH8);
#else
    t1 = random->status[0]
        + (random->status[2] >> TINYMT32_SH8);
#endif
    t0 ^= t1;
    if ((t1 & 1) != 0) {
        conv.u = ((t0 ^ random->tmat) >> 9) | UINT32_C(0x3f800000);
    }
    else {
        conv.u = (t0 >> 9) | UINT32_C(0x3f800000);
    }
    return conv.f;
}


/**
 * This function outputs floating point number from internal state.
 * Users should not call this function directly.
 * @param random tinymt internal status
 * @return floating point number r (1.0 < r < 2.0)
 */
inline static float tinymt32_temper_conv_open(tinymt32_t* random) {
    uint32_t t0, t1;
    union
    {
        uint32_t u;
        float f;
    } conv;

    t0 = random->status[3];
#if defined(LINEARITY_CHECK)
    t1 = random->status[0]
        ^ (random->status[2] >> TINYMT32_SH8);
#else
    t1 = random->status[0]
        + (random->status[2] >> TINYMT32_SH8);
#endif
    t0 ^= t1;
    if ((t1 & 1) != 0) {
        conv.u = ((t0 ^ random->tmat) >> 9) | UINT32_C(0x3f800001);
    }
    else {
        conv.u = (t0 >> 9) | UINT32_C(0x3f800001);
    }
    return conv.f;
}
#endif


/**
 * This function outputs 32-bit unsigned integer from internal state.
 * Users should not call this function directly.
 * @param random tinymt internal status
 * @return 32-bit unsigned pseudorandom number
 */
uint32_t TINYMT32_T::tinymt32_temper()
{
    uint32_t t0, t1;
    t0 = status[3];
    t1 = status[0] + (status[2] >> TINYMT32_SH8);
    t0 ^= t1;
    if ((t1 & 1) != 0) 
    {
        t0 ^= tmat;
    }
    return t0;
}

uint32_t TINYMT32_T::tinymt32_generate_uint32()
{
    tinymt32_next_state();
    return tinymt32_temper();
}
