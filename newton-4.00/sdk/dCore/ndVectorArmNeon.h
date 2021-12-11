/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
*
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __ND_VECTOR_ARM_NEON_H__
#define __ND_VECTOR_ARM_NEON_H__

//#include <arm_neon.h>

#if 0
// *****************************************************************************************
//
// 4 x 1 single precision vector class declaration
//
// *****************************************************************************************
#ifdef D_NEWTON_USE_DOUBLE
#define ndVector ndBigVector
#else


#if DG_ARCH >= DG_ARCH_NEON_64

A R M 6 4

#define vec_minv vminvq_u32
#define vec_maxv maxvq_u32
#define vec_hadd4 vaddvq_f32
#define vec_floor vrndq_f32

#else


#define inline_FORCEINLINE(type) static inline type

inline_FORCEINLINE(dInt32) vec_minv(uint32x4_t v)
{
	uint32x2_t tmp = vpmin_u32(vget_low_u32(v), vget_high_u32(v));
	tmp = vpmin_u32(tmp, tmp);
	return tmp[0] != 0;
}

inline_FORCEINLINE(dInt32) vec_maxv(uint32x4_t v)
{
	uint32x2_t tmp = vpmax_u32(vget_low_u32(v), vget_high_u32(v));
	tmp = vpmax_u32(tmp, tmp);
	return tmp[0] != 0;
}


inline_FORCEINLINE(float)vec_hadd4(float32x4_t v)
{
	float32x4_t tmp = vaddq_f32(v, vrev64q_f32(v));
	tmp = vaddq_f32(tmp, vcombine_f32(vget_high_f32(tmp), vget_low_f32(tmp)));
	return tmp[0];
}




#endif

inline_FORCEINLINE(float) vec_hadd3(float32x4_t v)
{
	float32x2_t temp = vpadd_f32(vget_low_f32(v), vget_low_f32(v));
	temp = vadd_f32(temp, vget_high_f32(v));
	return vget_lane_f32(temp, 0);
}


#define vec_mul vmulq_f32
#define vec_add vaddq_f32
#define vec_sub vsubq_f32
#define vec_max vmaxq_f32
#define vec_min vminq_f32
#define vec_splat vdupq_n_f32
#define vec_div vdivq_f32
#define vec_rcp vrecpeq_f32
#define vec_store vst1q_f32
#define vec_load vld1q_f32
#define vec_abs vabsq_f32
#define vec_cvt vcvtq_s32_f32
#define vec_sqrt vrsqrtsq_f32
#define vec_recp vrecpsq_f32
#define vec_rsqrt rsqrteq_f32
#define vec_cmpne vceqq_f32
#define vec_cmpgt vcgtq_f32
#define vec_cmpge vcgeq_f32
#define vec_cmpeq vceqq_f32
#define vec_cmplt vcltq_f32
#define vec_cmple vcleq_f32
#define vec_xor veorq_u32
#define vec_or vorrq_u32
#define vec_and vandq_u32
#define vec_not vmvnq_u32
#define vec_andnot vbicq_u32

#if defined __ARM_FEATURE_FMA
//  a * b + c (no rounding, better results)
#define vec_madd vfmaq_f32
#define vec_msub vmlsq_f32
#else
#define vec_madd vmlaq_f32
#define vec_msub vmlsq_f32
#endif

static inline float32x4_t vec_set(const float w, const float z, const float y, const float x)
{
	float ptr[] = { x, y, z, w };
	return vec_load(ptr);
}


class ndBigVector;
DG_MSC_VECTOR_ALIGMENT
class ndVector 
{
	public:
    inline ndVector()
    {
    }
    
    inline ndVector(const float32x4_t type)
    : m_type(type) {
    }
    
    inline ndVector(const uint32x4_t type)
    : m_typeInt(type) {
    }

    
    inline ndVector(dFloat32 val)
    : m_type(vmovq_n_f32(val))
    {
    }
    
    inline ndVector (const ndVector& v)
    : m_type( v.m_type )
    {
        //dAssert (dCheckVector ((*this)));
    }
    
    inline ndVector (const dFloat32* const ptr)
    :m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (dFloat32 (0.0f))
    {
        dAssert (dCheckVector ((*this)));
    }
    
#ifndef    D_NEWTON_USE_DOUBLE
    inline ndVector(const dFloat64* const ptr)
    :m_x(dFloat32(ptr[0]))
    ,m_y(dFloat32(ptr[1]))
    ,m_z(dFloat32(ptr[2]))
    ,m_w(dFloat32(0.0f))
    {
    }
#endif
    
    
    inline ndVector (dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w)
    :m_x(x), m_y(y), m_z(z), m_w(w)
    {
        dAssert (dCheckVector ((*this)));
    }
    
    inline ndVector (dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
    :m_x(*((dFloat32*)&ix)), m_y(*((dFloat32*)&iy)), m_z(*((dFloat32*)&iz)), m_w(*((dFloat32*)&iw))
    {
    }
    
#ifndef  D_NEWTON_USE_DOUBLE
    inline ndVector (const ndBigVector& copy)
    :m_x(dFloat32 (((dFloat64*)&copy)[0]))
    ,m_y(dFloat32 (((dFloat64*)&copy)[1]))
    ,m_z(dFloat32 (((dFloat64*)&copy)[2]))
    ,m_w(dFloat32 (((dFloat64*)&copy)[3]))
    {
        dAssert (dCheckVector ((*this)));
    }
#endif
    
    inline dFloat32 GetScalar () const
    {
        return m_x;
    }
    
    inline void Store (dFloat32* const dst) const
    {
        vec_store(dst, m_type);
    }
    
    inline ndVector BroadcastX () const
    {
        return ndVector (m_x);
    }
    
    inline ndVector BroadcastY () const
    {
        return ndVector (m_y);
    }
    
    inline ndVector BroadcastZ () const
    {
        return ndVector (m_z);
    }
    
    inline ndVector BroadcastW () const
    {
        return ndVector (m_w);
    }
    
    
    inline dFloat32& operator[] (dInt32 i)
    {
        dAssert (i < 4);
        dAssert (i >= 0);
        return (&m_x)[i];
    }
    
    inline const dFloat32& operator[] (dInt32 i) const
    {
        dAssert (i < 4);
        dAssert (i >= 0);
        return (&m_x)[i];
    }
    
    inline ndVector operator+ (const ndVector& A) const
    {
        return vec_add(m_type, A.m_type);
    }
    
    inline ndVector operator- (const ndVector& A) const
    {
        return vec_sub(m_type, A.m_type);
    }
    
    inline ndVector operator* (const ndVector& A) const
    {
        return vec_mul(m_type, A.m_type);
    }
    
    inline ndVector& operator+= (const ndVector& A)
    {
        m_type = vec_add(m_type, A.m_type);
        return *this;
    }
    
    inline ndVector& operator-= (const ndVector& A)
    {
        m_type = vec_sub(m_type, A.m_type);
        return *this;
    }
    
    inline ndVector& operator*= (const ndVector& A)
    {
        m_type = vec_mul(m_type, A.m_type);
        return *this;
    }
    
    
    
    inline ndVector AddHorizontal () const
    {
        return vec_hadd3(m_type); // ndVector (m_x + m_y + m_z + m_w);
    }
    
    inline ndVector Scale3 (dFloat32 scale) const
    {
        return ndVector (m_x * scale, m_y * scale, m_z * scale, m_w);
    }
    
    inline ndVector Scale (dFloat32 scale) const
    {
        return vec_mul(vmovq_n_f32(scale), m_type);
    }
    
    // component wise multiplication
    inline ndVector CompProduct3 (const ndVector& A) const
    {
        return ndVector (m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, A.m_w);
    }
    
    // return cross product
    inline ndVector CrossProduct (const ndVector& B) const
    {
        /*
         float32x4_t v1 = m_type;
         float32x4_t v2 = B.m_type;
         float32x4x2_t v_1203 = vzipq_f32(vcombine_f32(vrev64_f32(vget_low_f32(v1)), vrev64_f32(vget_low_f32(v2))), vcombine_f32(vget_high_f32(v1), vget_high_f32(v2)));
         float32x4x2_t v_2013 = vzipq_f32(vcombine_f32(vrev64_f32(vget_low_f32(v_1203.val[0])), vrev64_f32(vget_low_f32(v_1203.val[1]))), vcombine_f32(vget_high_f32(v_1203.val[0]), vget_high_f32(v_1203.val[1])));
         inVec->vec = vmlsq_f32(vmulq_f32(v_1203.val[0], v_2013.val[1]), v_1203.val[1], v_2013.val[0]);
         */
        return ndVector (m_y * B.m_z - m_z * B.m_y,
                         m_z * B.m_x - m_x * B.m_z,
                         m_x * B.m_y - m_y * B.m_x, m_w);
    }
    
    inline ndVector CrossProduct (const ndVector& A, const ndVector& B) const
    {
        dFloat32 cofactor[3][3];
        dFloat32 array[4][4];
        
        const ndVector& me = *this;
        for (dInt32 i = 0; i < 4; i ++) {
            array[0][i] = me[i];
            array[1][i] = A[i];
            array[2][i] = B[i];
            array[3][i] = dFloat32 (1.0f);
        }
        
        ndVector normal;
        dFloat32  sign = dFloat32 (-1.0f);
        for (dInt32 i = 0; i < 4; i ++)  
		{
            for (dInt32 j = 0; j < 3; j ++) 
			{
                dInt32 k0 = 0;
                for (dInt32 k = 0; k < 4; k ++) 
				{
                    if (k != i) 
					{
                        cofactor[j][k0] = array[j][k];
                        k0 ++;
                    }
                }
            }
            dFloat32 x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
            dFloat32 y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
            dFloat32 z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
            dFloat32 det = x + y + z;
            
            normal[i] = sign * det;
            sign *= dFloat32 (-1.0f);
        }
        
        return normal;
    }
    
    inline ndVector GetInt () const
    {
        return vcvtq_u32_f32(m_type);
    }
    
    inline ndVector GetFloat () const
    {
        return vcvtq_f32_u32(m_type);
    }
    
    inline ndVector TestZero() const
    {
        return m_negOne & (*this == m_zero);
    }
    
    inline ndVector Floor () const
    {
#if DG_ARCH >= DG_ARCH_NEON_64
        // #if __ARM_ARCH >= 8 && defined(__ARM_FEATURE_DIRECTED_ROUNDING)
        return vec_floor(m_type);
#else
        return ndVector (dgFloor (m_x), dgFloor (m_y), dgFloor (m_z), dgFloor (m_w));
        
#endif
    }
    
    inline ndVector DotProduct (const ndVector &A) const
    {
        auto tmp = vec_mul(m_type, A.m_type);
        return vec_hadd4(tmp);
    }
    
   
    
    inline ndVector InvMagSqrt() const
    {
        return ndVector(dgRsqrt(DotProduct(*this).m_x));
    }
    
    inline ndVector Reciproc() const
    {
        float32x4_t reciprocal = vrecpeq_f32(m_type);
        reciprocal = vrecpsq_f32(m_type, reciprocal) * reciprocal;
        return reciprocal;
        
    }


    inline ndVector MulAdd(const ndVector& A, const ndVector& B) const
    {
        // a * b + this
        return vec_madd(A.m_type, B.m_type, m_type);
    }

    inline ndVector MulSub(const ndVector& A, const ndVector& B) const
    {
        // a * b - this
        return vec_msub(A.m_type, B.m_type, m_type);
    }

    inline ndVector InvSqrt() const
    {
        float32x4_t sqrt_reciprocal = vrsqrteq_f32(m_type);
        return vrsqrtsq_f32(m_type * sqrt_reciprocal, sqrt_reciprocal) * sqrt_reciprocal;
    }
    
    inline ndVector Sqrt() const
    {
        float32x4_t sqrt_reciprocal = vrsqrteq_f32(m_type);
        float32x4_t tmp = vrsqrtsq_f32(m_type * sqrt_reciprocal, sqrt_reciprocal) * sqrt_reciprocal;
        return vec_mul(m_type, tmp);
    }
    
    inline ndVector Normalize () const
    {
        dAssert (m_w == dFloat32 (0.0f));
        const ndVector& me = *this;
        return me * InvMagSqrt();
    }
    
    ndVector Abs () const
    {
        return vec_abs(m_type);
    }
    
    dFloat32 GetMax () const
    {
        return dMax(dMax(m_x, m_y), dMax(m_z, m_w));
    }
    
    ndVector GetMax (const ndVector& data) const
    {
        return vec_max(m_type, data.m_type);
    }
    
    ndVector GetMin (const ndVector& data) const
    {
        return vec_min(m_type, data.m_type);
    }
    
    // relational operators
    inline ndVector operator== (const ndVector& data) const
    {
        return vec_cmpeq(m_typeInt, data.m_typeInt);
    }
    
    inline ndVector operator!= (const ndVector& data) const
    {
        return vec_cmpne(m_typeInt, data.m_typeInt);
    }
    
    inline ndVector operator> (const ndVector& data) const
    {
        return vec_cmpgt(m_typeInt, data.m_typeInt);
    }
    
    inline ndVector operator< (const ndVector& data) const
    {
        return vec_cmplt(m_typeInt, data.m_typeInt);
    }
    
    inline ndVector operator>= (const ndVector& data) const
    {
        return vec_cmpge(m_typeInt, data.m_typeInt);
    }
    
    inline ndVector operator<= (const ndVector& data) const
    {
        return vec_cmple(m_typeInt, data.m_typeInt);
    }
    
    // logical operations
    inline ndVector operator& (const ndVector& data) const
    {
        return vec_and(m_typeInt, data.m_typeInt);
    }
    
    inline ndVector operator| (const ndVector& data) const
    {
        return vec_or(m_typeInt, data.m_typeInt);
    }
    
    inline ndVector operator^ (const ndVector& data) const
    {
        return vec_xor(m_typeInt, data.m_typeInt);
    }
    
    inline ndVector AndNot (const ndVector& data) const
    {
        return vec_andnot(m_typeInt, data.m_typeInt);
    }
    
    inline dInt32 GetSignMask() const
    {
        const dInt32* const a = (dInt32*)&m_x;
        return (((a[0] & 0x80000000) ? 1 : 0) | ((a[1] & 0x80000000) ? 2 : 0) | ((a[2] & 0x80000000) ? 4 : 0) | ((a[3] & 0x80000000) ? 8 : 0));
    }
    
    inline ndVector ShiftTripleRight () const
    {
        return ndVector (m_z, m_x, m_y, m_w);
    }
    
    inline ndVector ShiftTripleLeft () const
    {
        return ndVector (m_y, m_z, m_x, m_w);
    }
    
    inline ndVector ShiftRightLogical (dInt32 bits) const
    {
        return ndVector (dInt32 (dUnsigned32 (m_ix) >> bits), dInt32 (dUnsigned32 (m_iy) >> bits), dInt32 (dUnsigned32 (m_iz) >> bits), dInt32 (dUnsigned32 (m_iw) >> bits));
    }
    
    inline static void Transpose4x4 (ndVector& dst0, ndVector& dst1, ndVector& dst2, ndVector& dst3, const ndVector& src0, const ndVector& src1, const ndVector& src2, const ndVector& src3)
    {
        float32x4x2_t vtrn1 = vzipq_f32(src0.m_type, src2.m_type);
        float32x4x2_t vtrn2 = vzipq_f32(src1.m_type, src3.m_type);
        float32x4x2_t res1 = vzipq_f32(vtrn1.val[0], vtrn2.val[0]);
        float32x4x2_t res2 = vzipq_f32(vtrn1.val[1], vtrn2.val[1]);
        dst0.m_type = res1.val[0];
        dst1.m_type = res1.val[1];
        dst2.m_type = res2.val[0];
        dst3.m_type = res2.val[1];

    }

	DG_CLASS_ALLOCATOR(allocator)

		union {
		float32x4_t m_type;
		uint32x4_t m_typeInt;
		dInt32 m_i[4];
		struct {
			dFloat32 m_x;
			dFloat32 m_y;
			dFloat32 m_z;
			dFloat32 m_w;
		};
		struct {
			dInt32 m_ix;
			dInt32 m_iy;
			dInt32 m_iz;
			dInt32 m_iw;
		};
	};

	D_CORE_API static ndVector m_zero;
	D_CORE_API static ndVector m_one;
	D_CORE_API static ndVector m_wOne;
	D_CORE_API static ndVector m_half;
	D_CORE_API static ndVector m_two;
	D_CORE_API static ndVector m_three;
	D_CORE_API static ndVector m_negOne;
	D_CORE_API static ndVector m_xMask;
	D_CORE_API static ndVector m_yMask;
	D_CORE_API static ndVector m_zMask;
	D_CORE_API static ndVector m_wMask;
	D_CORE_API static ndVector m_xyzwMask;
	D_CORE_API static ndVector m_signMask;
	D_CORE_API static ndVector m_triplexMask;
} DG_GCC_VECTOR_ALIGMENT;

#endif

DG_MSC_VECTOR_ALIGMENT
class ndBigVector
{
public:
	inline ndBigVector()
	{
	}

	inline ndBigVector(dFloat64 val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	inline ndBigVector(const ndBigVector& v)
		: m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

#ifndef D_NEWTON_USE_DOUBLE
	inline ndBigVector(const ndVector& v)
		: m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

	inline ndBigVector(const dFloat32* const ptr)
		: m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w(dFloat32(0.0f))
	{
		dAssert(dCheckVector((*this)));
	}
#endif

	inline ndBigVector(const dFloat64* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w(dFloat32(0.0f))
	{
		dAssert(dCheckVector((*this)));
	}

	inline ndBigVector(dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w)
		: m_x(x), m_y(y), m_z(z), m_w(w)
	{
		dAssert(dCheckVector((*this)));
	}

	inline ndBigVector(dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
		: m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	inline ndBigVector(dInt64 ix, dInt64 iy, dInt64 iz, dInt64 iw)
		: m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	inline dFloat64 GetScalar() const
	{
		return m_x;
	}

	inline void Store(dFloat64* const dst) const
	{
		dst[0] = m_x;
		dst[1] = m_y;
		dst[2] = m_z;
		dst[3] = m_w;
	}

	inline ndBigVector BroadcastX() const
	{
		return ndBigVector(m_x);
	}

	inline ndBigVector BroadcastY() const
	{
		return ndBigVector(m_y);
	}

	inline ndBigVector BroadcastZ() const
	{
		return ndBigVector(m_z);
	}

	inline ndBigVector BroadcastW() const
	{
		return ndBigVector(m_w);
	}


	inline dFloat64& operator[] (dInt32 i)
	{
		dAssert(i < 4);
		dAssert(i >= 0);
		return (&m_x)[i];
	}

	inline const dFloat64& operator[] (dInt32 i) const
	{
		dAssert(i < 4);
		dAssert(i >= 0);
		return (&m_x)[i];
	}

	inline ndBigVector operator+ (const ndBigVector& A) const
	{
		return ndBigVector(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	inline ndBigVector operator- (const ndBigVector& A) const
	{
		return ndBigVector(m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	inline ndBigVector operator* (const ndBigVector& A) const
	{
		return ndBigVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}

	inline ndBigVector& operator+= (const ndBigVector& A)
	{
		return (*this = ndBigVector(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	inline ndBigVector& operator-= (const ndBigVector& A)
	{
		return (*this = ndBigVector(m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	inline ndBigVector& operator*= (const ndBigVector& A)
	{
		return (*this = ndBigVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w));
	}

	inline ndBigVector AddHorizontal() const
	{
		return ndBigVector(m_x + m_y + m_z + m_w);
	}

	inline ndBigVector Scale3(dFloat64 scale) const
	{
		return ndBigVector(m_x * scale, m_y * scale, m_z * scale, m_w);
	}

	inline ndBigVector Scale(dFloat64 scale) const
	{
		return ndBigVector(m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// component wise multiplication
	inline ndBigVector CompProduct3(const ndBigVector& A) const
	{
		return ndBigVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, A.m_w);
	}

	// return cross product
	inline ndBigVector CrossProduct(const ndBigVector& B) const
	{
		return ndBigVector(m_y * B.m_z - m_z * B.m_y, m_z * B.m_x - m_x * B.m_z, m_x * B.m_y - m_y * B.m_x, m_w);
	}

	inline ndBigVector CrossProduct(const ndBigVector& A, const ndBigVector& B) const
	{
		dFloat64 cofactor[3][3];
		dFloat64 array[4][4];

		const ndBigVector& me = *this;
		for (dInt32 i = 0; i < 4; i++) {
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = dFloat32(1.0f);
		}

		ndBigVector normal;
		dFloat64  sign = dFloat64(-1.0f);
		for (dInt32 i = 0; i < 4; i++) {

			for (dInt32 j = 0; j < 3; j++) {
				dInt32 k0 = 0;
				for (dInt32 k = 0; k < 4; k++) {
					if (k != i) {
						cofactor[j][k0] = array[j][k];
						k0++;
					}
				}
			}
			dFloat64 x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			dFloat64 y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			dFloat64 z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			dFloat64 det = x + y + z;

			normal[i] = sign * det;
			sign *= dFloat64(-1.0f);
		}

		return normal;
	}

	inline ndBigVector GetInt() const
	{
		return ndBigVector(dInt64(floor(m_x)), dInt64(floor(m_y)), dInt64(floor(m_z)), dInt64(floor(m_w)));
	}

	inline ndBigVector TestZero() const
	{
		const dInt64* const a = (dInt64*)&m_x;
		return ndBigVector((a[0] == 0) ? dFloat64(-1.0f) : dFloat64(1.0f),
			(a[1] == 0) ? dFloat64(-1.0f) : dFloat64(1.0f),
			(a[2] == 0) ? dFloat64(-1.0f) : dFloat64(1.0f),
			(a[3] == 0) ? dFloat64(-1.0f) : dFloat64(1.0f));
	}


	inline ndBigVector Floor() const
	{
		return ndBigVector(floor(m_x), floor(m_y), floor(m_z), floor(m_w));
	}

	inline ndBigVector DotProduct(const ndBigVector &A) const
	{
		return ndBigVector(m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
	}

	inline ndBigVector Reciproc() const
	{
		return ndBigVector(dFloat64(1.0f) / m_x, dFloat64(1.0f) / m_y, dFloat64(1.0f) / m_z, dFloat64(1.0f) / m_w);
	}

	inline ndBigVector Sqrt() const
	{
		return ndBigVector(sqrt(m_x), sqrt(m_y), sqrt(m_z), sqrt(m_w));
	}

	inline ndBigVector InvSqrt() const
	{
		return ndBigVector(dFloat64(1.0f) / sqrt(m_x), dFloat64(1.0f) / sqrt(m_y), dFloat64(1.0f) / sqrt(m_z), dFloat64(1.0f) / sqrt(m_w));
	}

	inline ndBigVector InvMagSqrt() const
	{
		return ndBigVector(dFloat64(1.0f) / sqrt(DotProduct(*this).m_x));
	}

	inline ndBigVector Normalize() const
	{
		dAssert(m_w == dFloat64(0.0f));
		//const ndBigVector& me = *this;
		//return *this * ndBigVector (dgRsqrt(DotProduct(*this).m_x));
		return *this * InvMagSqrt();
	}

	ndBigVector Abs() const
	{
		return ndBigVector((m_x > dFloat64(0.0f)) ? m_x : -m_x,
			(m_y > dFloat64(0.0f)) ? m_y : -m_y,
			(m_z > dFloat64(0.0f)) ? m_z : -m_z,
			(m_w > dFloat64(0.0f)) ? m_w : -m_w);
	}

	dFloat64 GetMax() const
	{
		return dMax(dMax(m_x, m_y), dMax(m_z, m_w));
	}

	ndBigVector GetMax(const ndBigVector& data) const
	{
		return ndBigVector((m_x > data.m_x) ? m_x : data.m_x,
			(m_y > data.m_y) ? m_y : data.m_y,
			(m_z > data.m_z) ? m_z : data.m_z,
			(m_w > data.m_w) ? m_w : data.m_w);
	}

	ndBigVector GetMin(const ndBigVector& data) const
	{
		return ndBigVector((m_x < data.m_x) ? m_x : data.m_x,
			(m_y < data.m_y) ? m_y : data.m_y,
			(m_z < data.m_z) ? m_z : data.m_z,
			(m_w < data.m_w) ? m_w : data.m_w);
	}

	// relational operators
	inline ndBigVector operator== (const ndBigVector& data) const
	{
		return ndBigVector((m_x == data.m_x) ? dInt64(-1) : dInt64(0),
			(m_y == data.m_y) ? dInt64(-1) : dInt64(0),
			(m_z == data.m_z) ? dInt64(-1) : dInt64(0),
			(m_w == data.m_w) ? dInt64(-1) : dInt64(0));
	}

	inline ndBigVector operator> (const ndBigVector& data) const
	{
		return ndBigVector((m_x > data.m_x) ? dInt64(-1) : dInt64(0),
			(m_y > data.m_y) ? dInt64(-1) : dInt64(0),
			(m_z > data.m_z) ? dInt64(-1) : dInt64(0),
			(m_w > data.m_w) ? dInt64(-1) : dInt64(0));
	}

	inline ndBigVector operator< (const ndBigVector& data) const
	{
		return ndBigVector((m_x < data.m_x) ? dInt64(-1) : dInt64(0),
			(m_y < data.m_y) ? dInt64(-1) : dInt64(0),
			(m_z < data.m_z) ? dInt64(-1) : dInt64(0),
			(m_w < data.m_w) ? dInt64(-1) : dInt64(0));
	}

	inline ndBigVector operator>= (const ndBigVector& data) const
	{
		return ndBigVector((m_x >= data.m_x) ? dInt64(-1) : dInt64(0),
			(m_y >= data.m_y) ? dInt64(-1) : dInt64(0),
			(m_z >= data.m_z) ? dInt64(-1) : dInt64(0),
			(m_w >= data.m_w) ? dInt64(-1) : dInt64(0));
	}

	inline ndBigVector operator<= (const ndBigVector& data) const
	{
		return ndBigVector((m_x <= data.m_x) ? dInt64(-1) : dInt64(0),
			(m_y <= data.m_y) ? dInt64(-1) : dInt64(0),
			(m_z <= data.m_z) ? dInt64(-1) : dInt64(0),
			(m_w <= data.m_w) ? dInt64(-1) : dInt64(0));
	}


	// logical operations
	inline ndBigVector operator& (const ndBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return ndBigVector(a[0] & b[0], a[1] & b[1], a[2] & b[2], a[3] & b[3]);
	}

	inline ndBigVector operator| (const ndBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return ndBigVector(a[0] | b[0], a[1] | b[1], a[2] | b[2], a[3] | b[3]);
	}

	inline ndBigVector operator^ (const ndBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return ndBigVector(a[0] ^ b[0], a[1] ^ b[1], a[2] ^ b[2], a[3] ^ b[3]);
	}

	inline ndBigVector AndNot(const ndBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return ndBigVector(a[0] & ~b[0], a[1] & ~b[1], a[2] & ~b[2], a[3] & ~b[3]);
	}

	inline dInt32 GetSignMask() const
	{
		const dInt64* const a = (dInt64*)&m_x;
		return (((a[0] >> 63) ? 1 : 0) | ((a[1] >> 63) ? 2 : 0) | ((a[2] >> 63) ? 4 : 0) | ((a[3] >> 63) ? 8 : 0));
	}

	inline ndBigVector ShiftTripleRight() const
	{
		return ndBigVector(m_z, m_x, m_y, m_w);
	}

	inline ndBigVector ShiftTripleLeft() const
	{
		return ndBigVector(m_y, m_z, m_x, m_w);
	}

	inline ndBigVector ShiftRightLogical(dInt32 bits) const
	{
		return ndBigVector(dInt64(dUnsigned64(m_ix) >> bits), dInt64(dUnsigned64(m_iy) >> bits), dInt64(dUnsigned64(m_iz) >> bits), dInt64(dUnsigned64(m_iw) >> bits));
	}

	inline static void Transpose4x4(ndBigVector& dst0, ndBigVector& dst1, ndBigVector& dst2, ndBigVector& dst3, const ndBigVector& src0, const ndBigVector& src1, const ndBigVector& src2, const ndBigVector& src3)
	{
		ndBigVector tmp0(src0);
		ndBigVector tmp1(src1);
		ndBigVector tmp2(src2);
		ndBigVector tmp3(src3);

		dst0 = ndBigVector(tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
		dst1 = ndBigVector(tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
		dst2 = ndBigVector(tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
		dst3 = ndBigVector(tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
	}

	DG_CLASS_ALLOCATOR(allocator)

	union
	{
#if DG_ARCH >= DG_ARCH_NEON_64
		struct {
			float64x2_t m_xy;
			float64x2_t m_zw;
		};
		struct {
			int64x2_t m_ixy;
			int64x2_t m_izw;
		};
#endif
		dInt64 m_i[4];
		struct
		{
			dFloat64 m_x;
			dFloat64 m_y;
			dFloat64 m_z;
			dFloat64 m_w;
		};
		struct
		{
			dInt64 m_ix;
			dInt64 m_iy;
			dInt64 m_iz;
			dInt64 m_iw;
		};
	};

	D_CORE_API static ndBigVector m_zero;
	D_CORE_API static ndBigVector m_one;
	D_CORE_API static ndBigVector m_wOne;
	D_CORE_API static ndBigVector m_half;
	D_CORE_API static ndBigVector m_two;
	D_CORE_API static ndBigVector m_three;
	D_CORE_API static ndBigVector m_negOne;
	D_CORE_API static ndBigVector m_xMask;
	D_CORE_API static ndBigVector m_yMask;
	D_CORE_API static ndBigVector m_zMask;
	D_CORE_API static ndBigVector m_wMask;
	D_CORE_API static ndBigVector m_xyzwMask;
	D_CORE_API static ndBigVector m_signMask;
	D_CORE_API static ndBigVector m_triplexMask;
} DG_GCC_VECTOR_ALIGMENT;

#endif


// *****************************************************************************************
//
// 4 x 1 single precision vector class declaration
//
// *****************************************************************************************
#ifdef D_NEWTON_USE_DOUBLE
#define ndVector ndBigVector
#else

class ndBigVector;
D_MSV_NEWTON_ALIGN_16
class ndVector
{
	public:
	D_OPERATOR_NEW_AND_DELETE

	inline ndVector()
	{
	}

	inline ndVector(dFloat32 val)
		:m_type(vmovq_n_f32(val))
	{
	}

	inline ndVector(const ndVector& v)
		:m_type(v.m_type)
	{
	}

	inline ndVector(const float32x4_t type)
		:m_type(type)
	{
	}

	inline ndVector(const dFloat32* const ptr)
		//: m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w(ptr[3])
		:m_type(vld1q_f32 (ptr))
	{
		dAssert(dCheckVector((*this)));
	}

	inline ndVector(const dFloat32* const baseAddr, const dInt32* const index)
		:m_x(baseAddr[index[0]])
		,m_y(baseAddr[index[1]])
		,m_z(baseAddr[index[2]])
		,m_w(baseAddr[index[3]])
	{
	}


#ifndef	D_NEWTON_USE_DOUBLE
	inline ndVector(const dFloat64* const ptr)
		:m_x(dFloat32(ptr[0]))
		,m_y(dFloat32(ptr[1]))
		,m_z(dFloat32(ptr[2]))
		,m_w(dFloat32(ptr[3]))
	{
	}
#endif

	inline ndVector(dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w)
		:m_x(x), m_y(y), m_z(z), m_w(w)
	{
		dAssert(dCheckVector((*this)));
	}

	inline ndVector(dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
		: m_x(*((dFloat32*)&ix)), m_y(*((dFloat32*)&iy)), m_z(*((dFloat32*)&iz)), m_w(*((dFloat32*)&iw))
	{
	}

#ifndef  D_NEWTON_USE_DOUBLE 
	inline ndVector(const ndBigVector& copy)
		:m_x(dFloat32(((dFloat64*)&copy)[0]))
		,m_y(dFloat32(((dFloat64*)&copy)[1]))
		,m_z(dFloat32(((dFloat64*)&copy)[2]))
		,m_w(dFloat32(((dFloat64*)&copy)[3]))
	{
		dAssert(dCheckVector((*this)));
	}
#endif

	inline dFloat32 GetScalar() const
	{
		return m_x;
	}

	inline void Store(dFloat32* const dst) const
	{
		vst1q_f32(dst, m_type);
	}

	inline ndVector BroadcastX() const
	{
		return ndVector(m_x);
	}

	inline ndVector BroadcastY() const
	{
		return ndVector(m_y);
	}

	inline ndVector BroadcastZ() const
	{
		return ndVector(m_z);
	}

	inline ndVector BroadcastW() const
	{
		return ndVector(m_w);
	}

	inline dFloat32& operator[] (dInt32 i)
	{
		dAssert(i < 4);
		dAssert(i >= 0);
		return (&m_x)[i];
	}

	inline const dFloat32& operator[] (dInt32 i) const
	{
		dAssert(i < 4);
		dAssert(i >= 0);
		return (&m_x)[i];
	}

	inline ndVector operator+ (const ndVector& A) const
	{
		return vaddq_f32(m_type, A.m_type);
	}

	inline ndVector operator- (const ndVector& A) const
	{
		return vsubq_f32(m_type, A.m_type);
	}

	inline ndVector operator* (const ndVector& A) const
	{
		return vmulq_f32(m_type, A.m_type);
	}

	inline ndVector& operator+= (const ndVector& A)
	{
		return (*this = vsubq_f32(m_type, A.m_type));
	}

	inline ndVector& operator-= (const ndVector& A)
	{
		return (*this = vsubq_f32(m_type, A.m_type));
	}

	inline ndVector& operator*= (const ndVector& A)
	{
		return (*this = vmulq_f32(m_type, A.m_type));
	}

	inline ndVector MulAdd(const ndVector& A, const ndVector& B) const
	{
		//return *this + A * B;
		//return vfmaq_f32(A.m_type, B.m_type, m_type);
		return vmlaq_f32(m_type, A.m_type, B.m_type);
	}

	inline ndVector MulSub(const ndVector& A, const ndVector& B) const
	{
		//return *this - A * B;
		return vmlsq_f32(m_type, A.m_type, B.m_type);
	}

	inline ndVector AddHorizontal() const
	{
		return ndVector(m_x + m_y + m_z + m_w);
		//float32x2_t temp = vpadd_f32(vget_low_f32(m_type), vget_low_f32(m_type));
		//temp = vadd_f32(temp, vget_high_f32(m_type));
		//return vget_lane_f32(temp, 0);
	}

	inline ndVector Scale(dFloat32 scale) const
	{
		return ndVector(m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// return cross product
	inline ndVector CrossProduct(const ndVector& B) const
	{
		return ndVector(m_y * B.m_z - m_z * B.m_y,
			m_z * B.m_x - m_x * B.m_z,
			m_x * B.m_y - m_y * B.m_x, m_w);
	}

	inline ndVector CrossProduct(const ndVector& A, const ndVector& B) const
	{
		dFloat32 cofactor[3][3];
		dFloat32 array[4][4];

		const ndVector& me = *this;
		for (dInt32 i = 0; i < 4; i++) 
		{
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = dFloat32(1.0f);
		}

		ndVector normal;
		dFloat32  sign = dFloat32(-1.0f);
		for (dInt32 i = 0; i < 4; i++) 
		{
			for (dInt32 j = 0; j < 3; j++) 
			{
				dInt32 k0 = 0;
				for (dInt32 k = 0; k < 4; k++) 
				{
					if (k != i) 
					{
						cofactor[j][k0] = array[j][k];
						k0++;
					}
				}
			}
			dFloat32 x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			dFloat32 y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			dFloat32 z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			dFloat32 det = x + y + z;

			normal[i] = sign * det;
			sign *= dFloat32(-1.0f);
		}

		return normal;
	}

	inline ndVector GetInt() const
	{
		return ndVector(dInt32(dFloor(m_x)), dInt32(dFloor(m_y)), dInt32(dFloor(m_z)), dInt32(dFloor(m_w)));
	}

	inline ndVector TestZero() const
	{
		const dInt32* const a = (dInt32*)&m_x;
		return ndVector((a[0] == 0) ? dFloat32(-1.0f) : dFloat32(1.0f),
			(a[1] == 0) ? dFloat32(-1.0f) : dFloat32(1.0f),
			(a[2] == 0) ? dFloat32(-1.0f) : dFloat32(1.0f),
			(a[3] == 0) ? dFloat32(-1.0f) : dFloat32(1.0f));
	}

	inline ndVector Floor() const
	{
		return ndVector(dFloor(m_x), dFloor(m_y), dFloor(m_z), dFloor(m_w));
	}

	inline ndVector DotProduct(const ndVector &A) const
	{
		return ndVector(m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
	}

	inline ndVector Reciproc() const
	{
		return ndVector(dFloat32(1.0f) / m_x, dFloat32(1.0f) / m_y, dFloat32(1.0f) / m_z, dFloat32(1.0f) / m_w);
	}

	inline ndVector Sqrt() const
	{
		return ndVector(dSqrt(m_x), dSqrt(m_y), dSqrt(m_z), dSqrt(m_w));
	}

	inline ndVector InvSqrt() const
	{
		return ndVector(dRsqrt(m_x), dRsqrt(m_y), dRsqrt(m_z), dRsqrt(m_w));
	}

	inline ndVector InvMagSqrt() const
	{
		return ndVector(dRsqrt(DotProduct(*this).m_x));
	}

	inline ndVector Normalize() const
	{
		dAssert(m_w == dFloat32(0.0f));
		const ndVector& me = *this;
		return me * InvMagSqrt();
	}

	ndVector Abs() const
	{
		return vabsq_f32(m_type);
	}

	dFloat32 GetMax() const
	{
		return dMax(dMax(m_x, m_y), dMax(m_z, m_w));
	}

	ndVector GetMax(const ndVector& data) const
	{
		return vmaxq_f32(m_type, data.m_type);
	}

	ndVector GetMin(const ndVector& data) const
	{
		return vminq_f32(m_type, data.m_type);
	}

	// relational operators
	inline ndVector operator== (const ndVector& data) const
	{
		return vceqq_f32(m_typeInt, data.m_typeInt);
	}

	inline ndVector operator> (const ndVector& data) const
	{
		return vcgtq_f32(m_typeInt, data.m_typeInt);
	}

	inline ndVector operator< (const ndVector& data) const
	{
		return vcltq_f32(m_typeInt, data.m_typeInt);
	}

	inline ndVector operator>= (const ndVector& data) const
	{
		return vcgeq_f32(m_typeInt, data.m_typeInt);
	}

	inline ndVector operator<= (const ndVector& data) const
	{
		return vcleq_f32(m_typeInt, data.m_typeInt);
	}

	// logical operations
	inline ndVector operator& (const ndVector& data) const
	{
		return vandq_u32(m_typeInt, data.m_typeInt);
	}

	inline ndVector operator| (const ndVector& data) const
	{
		return vorrq_u32(m_typeInt, data.m_typeInt);
	}

	inline ndVector operator^ (const ndVector& data) const
	{
		return veorq_u32(m_typeInt, data.m_typeInt);
	}

	inline ndVector AndNot(const ndVector& data) const
	{
		return vbicq_u32(m_typeInt, data.m_typeInt);
	}

	inline ndVector Select(const ndVector& data, const ndVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		return  (*this) ^ (mask & (data ^ (*this)));
	}

	inline dInt32 GetSignMask() const
	{
		const dInt32* const a = (dInt32*)&m_x;
		return (((a[0] & 0x80000000) ? 1 : 0) | ((a[1] & 0x80000000) ? 2 : 0) | ((a[2] & 0x80000000) ? 4 : 0) | ((a[3] & 0x80000000) ? 8 : 0));
	}

	inline ndVector ShiftRight() const
	{
		return ndVector(m_w, m_x, m_y, m_z);
	}

	inline ndVector ShiftTripleRight() const
	{
		return ndVector(m_z, m_x, m_y, m_w);
	}

	inline ndVector ShiftTripleLeft() const
	{
		return ndVector(m_y, m_z, m_x, m_w);
	}

	inline ndVector ShiftRightLogical(dInt32 bits) const
	{
		return ndVector(dInt32(dUnsigned32(m_ix) >> bits), dInt32(dUnsigned32(m_iy) >> bits), dInt32(dUnsigned32(m_iz) >> bits), dInt32(dUnsigned32(m_iw) >> bits));
	}

	inline static void Transpose4x4(ndVector& dst0, ndVector& dst1, ndVector& dst2, ndVector& dst3, const ndVector& src0, const ndVector& src1, const ndVector& src2, const ndVector& src3)
	{
		float32x4x2_t vtrn1 = vzipq_f32(src0.m_type, src2.m_type);
		float32x4x2_t vtrn2 = vzipq_f32(src1.m_type, src3.m_type);
		float32x4x2_t res1 = vzipq_f32(vtrn1.val[0], vtrn2.val[0]);
		float32x4x2_t res2 = vzipq_f32(vtrn1.val[1], vtrn2.val[1]);
		dst0.m_type = res1.val[0];
		dst1.m_type = res1.val[1];
		dst2.m_type = res2.val[0];
		dst3.m_type = res2.val[1];
	}

	union 
	{
		dFloat32 m_f[4];
		dInt32 m_i[4];
		float32x4_t m_type;
		uint32x4_t m_typeInt;
		struct 
		{
			dFloat32 m_x;
			dFloat32 m_y;
			dFloat32 m_z;
			dFloat32 m_w;
		};
		struct 
		{
			dInt32 m_ix;
			dInt32 m_iy;
			dInt32 m_iz;
			dInt32 m_iw;
		};
	};

	D_CORE_API static ndVector m_zero;
	D_CORE_API static ndVector m_one;
	D_CORE_API static ndVector m_wOne;
	D_CORE_API static ndVector m_half;
	D_CORE_API static ndVector m_two;
	D_CORE_API static ndVector m_three;
	D_CORE_API static ndVector m_negOne;
	D_CORE_API static ndVector m_xMask;
	D_CORE_API static ndVector m_yMask;
	D_CORE_API static ndVector m_zMask;
	D_CORE_API static ndVector m_wMask;
	D_CORE_API static ndVector m_xyzwMask;
	D_CORE_API static ndVector m_epsilon;
	D_CORE_API static ndVector m_signMask;
	D_CORE_API static ndVector m_triplexMask;
} D_GCC_NEWTON_ALIGN_32 ;

#endif

D_MSV_NEWTON_ALIGN_32
class ndBigVector
{
	public:
	D_OPERATOR_NEW_AND_DELETE

	inline ndBigVector()
	{
	}

	inline ndBigVector(dFloat64 val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	inline ndBigVector(const ndBigVector& v)
		: m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

#ifndef D_NEWTON_USE_DOUBLE
	inline ndBigVector(const ndVector& v)
		: m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

	inline ndBigVector(const dFloat32* const ptr)
		: m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w(dFloat32(0.0f))
	{
		dAssert(dCheckVector((*this)));
	}
#endif

	inline ndBigVector(const dFloat64* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w(ptr[3])
	{
		dAssert(dCheckVector((*this)));
	}

	inline ndBigVector(dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w)
		: m_x(x), m_y(y), m_z(z), m_w(w)
	{
		dAssert(dCheckVector((*this)));
	}

	inline ndBigVector(dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
		: m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	inline ndBigVector(dInt64 ix, dInt64 iy, dInt64 iz, dInt64 iw)
		: m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	inline dFloat64 GetScalar() const
	{
		return m_x;
	}

	inline void Store(dFloat64* const dst) const
	{
		dst[0] = m_x;
		dst[1] = m_y;
		dst[2] = m_z;
		dst[3] = m_w;
	}

	inline ndBigVector BroadcastX() const
	{
		return ndBigVector(m_x);
	}

	inline ndBigVector BroadcastY() const
	{
		return ndBigVector(m_y);
	}

	inline ndBigVector BroadcastZ() const
	{
		return ndBigVector(m_z);
	}

	inline ndBigVector BroadcastW() const
	{
		return ndBigVector(m_w);
	}

	inline dFloat64& operator[] (dInt32 i)
	{
		dAssert(i < 4);
		dAssert(i >= 0);
		return (&m_x)[i];
	}

	inline const dFloat64& operator[] (dInt32 i) const
	{
		dAssert(i < 4);
		dAssert(i >= 0);
		return (&m_x)[i];
	}

	inline ndBigVector operator+ (const ndBigVector& A) const
	{
		return ndBigVector(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	inline ndBigVector operator- (const ndBigVector& A) const
	{
		return ndBigVector(m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	inline ndBigVector operator* (const ndBigVector& A) const
	{
		return ndBigVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}

	inline ndBigVector& operator+= (const ndBigVector& A)
	{
		return (*this = ndBigVector(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	inline ndBigVector& operator-= (const ndBigVector& A)
	{
		return (*this = ndBigVector(m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	inline ndBigVector& operator*= (const ndBigVector& A)
	{
		return (*this = ndBigVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w));
	}

	inline ndBigVector MulAdd(const ndBigVector& A, const ndBigVector& B) const
	{
		return *this + A * B;
	}

	inline ndBigVector MulSub(const ndVector& A, const ndBigVector& B) const
	{
		return *this - A * B;
	}


	inline ndBigVector AddHorizontal() const
	{
		return ndBigVector(m_x + m_y + m_z + m_w);
	}

	inline ndBigVector Scale(dFloat64 scale) const
	{
		return ndBigVector(m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// return cross product
	inline ndBigVector CrossProduct(const ndBigVector& B) const
	{
		return ndBigVector(m_y * B.m_z - m_z * B.m_y, m_z * B.m_x - m_x * B.m_z, m_x * B.m_y - m_y * B.m_x, m_w);
	}

	inline ndBigVector CrossProduct(const ndBigVector& A, const ndBigVector& B) const
	{
		dFloat64 cofactor[3][3];
		dFloat64 array[4][4];

		const ndBigVector& me = *this;
		for (dInt32 i = 0; i < 4; i++) {
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = dFloat32(1.0f);
		}

		ndBigVector normal;
		dFloat64  sign = dFloat64(-1.0f);
		for (dInt32 i = 0; i < 4; i++) 
		{
			for (dInt32 j = 0; j < 3; j++) 
			{
				dInt32 k0 = 0;
				for (dInt32 k = 0; k < 4; k++) 
				{
					if (k != i) 
					{
						cofactor[j][k0] = array[j][k];
						k0++;
					}
				}
			}
			dFloat64 x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			dFloat64 y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			dFloat64 z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			dFloat64 det = x + y + z;

			normal[i] = sign * det;
			sign *= dFloat64(-1.0f);
		}

		return normal;
	}

	inline ndBigVector GetInt() const
	{
		return ndBigVector(dInt64(floor(m_x)), dInt64(floor(m_y)), dInt64(floor(m_z)), dInt64(floor(m_w)));
	}

	inline ndBigVector TestZero() const
	{
		const dInt64* const a = (dInt64*)&m_x;
		return ndBigVector((a[0] == 0) ? dFloat64(-1.0f) : dFloat64(1.0f),
			(a[1] == 0) ? dFloat64(-1.0f) : dFloat64(1.0f),
			(a[2] == 0) ? dFloat64(-1.0f) : dFloat64(1.0f),
			(a[3] == 0) ? dFloat64(-1.0f) : dFloat64(1.0f));
	}


	inline ndBigVector Floor() const
	{
		return ndBigVector(floor(m_x), floor(m_y), floor(m_z), floor(m_w));
	}

	inline ndBigVector DotProduct(const ndBigVector &A) const
	{
		return ndBigVector(m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
	}

	inline ndBigVector Reciproc() const
	{
		return ndBigVector(dFloat64(1.0f) / m_x, dFloat64(1.0f) / m_y, dFloat64(1.0f) / m_z, dFloat64(1.0f) / m_w);
	}

	inline ndBigVector Sqrt() const
	{
		return ndBigVector(sqrt(m_x), sqrt(m_y), sqrt(m_z), sqrt(m_w));
	}

	inline ndBigVector InvSqrt() const
	{
		return ndBigVector(dFloat64(1.0f) / sqrt(m_x), dFloat64(1.0f) / sqrt(m_y), dFloat64(1.0f) / sqrt(m_z), dFloat64(1.0f) / sqrt(m_w));
	}

	inline ndBigVector InvMagSqrt() const
	{
		return ndBigVector(dFloat64(1.0f) / sqrt(DotProduct(*this).m_x));
	}

	inline ndBigVector Normalize() const
	{
		dAssert(m_w == dFloat64(0.0f));
		//const ndBigVector& me = *this;
		//return *this * ndBigVector (dgRsqrt(DotProduct(*this).m_x));
		return *this * InvMagSqrt();
	}

	ndBigVector Abs() const
	{
		return ndBigVector((m_x > dFloat64(0.0f)) ? m_x : -m_x,
			(m_y > dFloat64(0.0f)) ? m_y : -m_y,
			(m_z > dFloat64(0.0f)) ? m_z : -m_z,
			(m_w > dFloat64(0.0f)) ? m_w : -m_w);
	}

	dFloat64 GetMax() const
	{
		return dMax(dMax(m_x, m_y), dMax(m_z, m_w));
	}

	ndBigVector GetMax(const ndBigVector& data) const
	{
		return ndBigVector((m_x > data.m_x) ? m_x : data.m_x,
			(m_y > data.m_y) ? m_y : data.m_y,
			(m_z > data.m_z) ? m_z : data.m_z,
			(m_w > data.m_w) ? m_w : data.m_w);
	}

	ndBigVector GetMin(const ndBigVector& data) const
	{
		return ndBigVector((m_x < data.m_x) ? m_x : data.m_x,
			(m_y < data.m_y) ? m_y : data.m_y,
			(m_z < data.m_z) ? m_z : data.m_z,
			(m_w < data.m_w) ? m_w : data.m_w);
	}

	// relational operators
	inline ndBigVector operator== (const ndBigVector& data) const
	{
		return ndBigVector((m_x == data.m_x) ? dInt64(-1) : dInt64(0),
			(m_y == data.m_y) ? dInt64(-1) : dInt64(0),
			(m_z == data.m_z) ? dInt64(-1) : dInt64(0),
			(m_w == data.m_w) ? dInt64(-1) : dInt64(0));
	}

	inline ndBigVector operator> (const ndBigVector& data) const
	{
		return ndBigVector((m_x > data.m_x) ? dInt64(-1) : dInt64(0),
			(m_y > data.m_y) ? dInt64(-1) : dInt64(0),
			(m_z > data.m_z) ? dInt64(-1) : dInt64(0),
			(m_w > data.m_w) ? dInt64(-1) : dInt64(0));
	}

	inline ndBigVector operator< (const ndBigVector& data) const
	{
		return ndBigVector((m_x < data.m_x) ? dInt64(-1) : dInt64(0),
			(m_y < data.m_y) ? dInt64(-1) : dInt64(0),
			(m_z < data.m_z) ? dInt64(-1) : dInt64(0),
			(m_w < data.m_w) ? dInt64(-1) : dInt64(0));
	}

	inline ndBigVector operator>= (const ndBigVector& data) const
	{
		return ndBigVector((m_x >= data.m_x) ? dInt64(-1) : dInt64(0),
			(m_y >= data.m_y) ? dInt64(-1) : dInt64(0),
			(m_z >= data.m_z) ? dInt64(-1) : dInt64(0),
			(m_w >= data.m_w) ? dInt64(-1) : dInt64(0));
	}

	inline ndBigVector operator<= (const ndBigVector& data) const
	{
		return ndBigVector((m_x <= data.m_x) ? dInt64(-1) : dInt64(0),
			(m_y <= data.m_y) ? dInt64(-1) : dInt64(0),
			(m_z <= data.m_z) ? dInt64(-1) : dInt64(0),
			(m_w <= data.m_w) ? dInt64(-1) : dInt64(0));
	}


	// logical operations
	inline ndBigVector operator& (const ndBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return ndBigVector(a[0] & b[0], a[1] & b[1], a[2] & b[2], a[3] & b[3]);
	}

	inline ndBigVector operator| (const ndBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return ndBigVector(a[0] | b[0], a[1] | b[1], a[2] | b[2], a[3] | b[3]);
	}

	inline ndBigVector operator^ (const ndBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return ndBigVector(a[0] ^ b[0], a[1] ^ b[1], a[2] ^ b[2], a[3] ^ b[3]);
	}

	inline ndBigVector AndNot(const ndBigVector& data) const
	{
		const dInt64* const a = (dInt64*)&m_x;
		const dInt64* const b = (dInt64*)&data.m_x;
		return ndBigVector(a[0] & ~b[0], a[1] & ~b[1], a[2] & ~b[2], a[3] & ~b[3]);
	}

	inline ndBigVector Select(const ndBigVector& data, const ndBigVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		return  (*this) ^ (mask & (data ^ (*this)));
	}

	inline dInt32 GetSignMask() const
	{
		const dInt64* const a = (dInt64*)&m_x;
		return (((a[0] >> 63) ? 1 : 0) | ((a[1] >> 63) ? 2 : 0) | ((a[2] >> 63) ? 4 : 0) | ((a[3] >> 63) ? 8 : 0));
	}

	inline ndVector ShiftRight() const
	{
		return ndBigVector(m_w, m_x, m_y, m_z);
	}

	inline ndBigVector ShiftTripleRight() const
	{
		return ndBigVector(m_z, m_x, m_y, m_w);
	}

	inline ndBigVector ShiftTripleLeft() const
	{
		return ndBigVector(m_y, m_z, m_x, m_w);
	}

	inline ndBigVector ShiftRightLogical(dInt32 bits) const
	{
		return ndBigVector(dInt64(dUnsigned64(m_ix) >> bits), dInt64(dUnsigned64(m_iy) >> bits), dInt64(dUnsigned64(m_iz) >> bits), dInt64(dUnsigned64(m_iw) >> bits));
	}

	inline static void Transpose4x4(ndBigVector& dst0, ndBigVector& dst1, ndBigVector& dst2, ndBigVector& dst3, const ndBigVector& src0, const ndBigVector& src1, const ndBigVector& src2, const ndBigVector& src3)
	{
		ndBigVector tmp0(src0);
		ndBigVector tmp1(src1);
		ndBigVector tmp2(src2);
		ndBigVector tmp3(src3);

		dst0 = ndBigVector(tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
		dst1 = ndBigVector(tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
		dst2 = ndBigVector(tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
		dst3 = ndBigVector(tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
	}

	union
	{
		dInt64 m_i[4];
		struct
		{
			dFloat64 m_x;
			dFloat64 m_y;
			dFloat64 m_z;
			dFloat64 m_w;
		};
		struct
		{
			dInt64 m_ix;
			dInt64 m_iy;
			dInt64 m_iz;
			dInt64 m_iw;
		};
	};

	D_CORE_API static ndBigVector m_zero;
	D_CORE_API static ndBigVector m_one;
	D_CORE_API static ndBigVector m_wOne;
	D_CORE_API static ndBigVector m_half;
	D_CORE_API static ndBigVector m_two;
	D_CORE_API static ndBigVector m_three;
	D_CORE_API static ndBigVector m_negOne;
	D_CORE_API static ndBigVector m_xMask;
	D_CORE_API static ndBigVector m_yMask;
	D_CORE_API static ndBigVector m_zMask;
	D_CORE_API static ndBigVector m_wMask;
	D_CORE_API static ndBigVector m_xyzwMask;
	D_CORE_API static ndBigVector m_epsilon;
	D_CORE_API static ndBigVector m_signMask;
	D_CORE_API static ndBigVector m_triplexMask;
} D_GCC_NEWTON_ALIGN_32 ;

#endif