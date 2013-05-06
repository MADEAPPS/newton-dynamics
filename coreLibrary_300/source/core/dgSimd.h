/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __DG_SIMD_INSTRUCTION_H__
#define __DG_SIMD_INSTRUCTION_H__


#include "dgStdafx.h"
#include "dgTypes.h"

#ifdef DG_BUILD_SIMD_CODE

	#if (defined (_WIN_32_VER) || defined (_WIN_64_VER) || (defined (_MACOSX_VER) && !defined (__ppc__)))
		#define DG_SSE_3
	#endif

	#define simd_env					dgUnsigned32
	#define PURMUT_MASK(w, z, y, x)		_MM_SHUFFLE (w, z, y, x)

	DG_MSC_VECTOR_ALIGMENT
	class dgSimd
	{
		public:
		DG_INLINE dgSimd () {}
		DG_INLINE dgSimd (const __m128 type): m_type(type) {}
		DG_INLINE dgSimd (const dgFloat32 a): m_type(_mm_set_ps1(a)) {}
		DG_INLINE dgSimd (const dgSimd& data): m_type(data.m_type) {}
		DG_INLINE dgSimd (const dgInt32 a): m_type (_mm_set_ps1 (*(dgFloat32*)&a)){}
		DG_INLINE dgSimd (const dgFloat32* const ptr): m_type(_mm_loadu_ps (ptr)) {}
		DG_INLINE dgSimd (const dgFloat32 x, const dgFloat32 y, const dgFloat32 z, const dgFloat32 w): m_type(_mm_set_ps(w, z, y, x)) {}
		DG_INLINE dgSimd (const dgInt32 ix, const dgInt32 iy, const dgInt32 iz, const dgInt32 iw): m_type(_mm_set_ps(*(dgFloat32*)&iw, *(dgFloat32*)&iz, *(dgFloat32*)&iy, *(dgFloat32*)&ix)) {}


		static DG_INLINE dgInt32 get_ctrl()
		{
			return _mm_getcsr ();
		}

		static DG_INLINE void set_ctrl(dgInt32 a)
		{
			_mm_setcsr (a);
		}

		static DG_INLINE void set_FZ_mode()
		{
			_MM_SET_FLUSH_ZERO_MODE (_MM_FLUSH_ZERO_ON);
		}
		
		DG_INLINE dgInt32 GetInt () const
		{
			return _mm_cvtss_si32(m_type);
		}

		DG_INLINE void StoreScalar(float* const scalar) const
		{
			_mm_store_ss (scalar, m_type);
		}

		DG_INLINE void StoreVector(float* const array) const
		{
			_mm_storeu_ps (array, m_type);
		}

		DG_INLINE dgSimd operator= (const dgSimd& data)
		{
			m_type = data.m_type;
			return (*this);
		}

		DG_INLINE dgSimd operator+ (const dgSimd& data) const
		{
			return _mm_add_ps (m_type, data.m_type);	
		}

		DG_INLINE dgSimd operator- (const dgSimd& data) const
		{
			return _mm_sub_ps (m_type, data.m_type);	
		}

		DG_INLINE dgSimd operator* (const dgSimd& data) const
		{
			return _mm_mul_ps (m_type, data.m_type);	
		}

		DG_INLINE dgSimd operator/ (const dgSimd& data) const
		{
			return _mm_div_ps (m_type, data.m_type);	
		}

		DG_INLINE dgSimd operator<= (const dgSimd& data) const
		{
			return _mm_cmple_ps (m_type, data.m_type);	
		}

		DG_INLINE dgSimd operator>= (const dgSimd& data) const
		{
			return _mm_cmpge_ps (m_type, data.m_type);	
		}

		DG_INLINE dgSimd operator< (const dgSimd& data) const
		{
			return _mm_cmplt_ps (m_type, data.m_type);	
		}

		DG_INLINE dgSimd operator> (const dgSimd& data) const
		{
			return _mm_cmpgt_ps (m_type, data.m_type);	
		}

		DG_INLINE dgSimd operator& (const dgSimd& data) const
		{
			return _mm_and_ps (m_type, data.m_type);	
		}

		DG_INLINE dgSimd operator| (const dgSimd& data) const
		{
			return _mm_or_ps (m_type, data.m_type);	
		}

		DG_INLINE dgSimd AndNot (const dgSimd& data) const
		{
			return _mm_andnot_ps (data.m_type, m_type);	
		}
	
		DG_INLINE dgSimd AddHorizontal () const
		{
			#ifdef DG_SSE_3
				dgSimd tmp (_mm_hadd_ps (m_type, m_type));
				return _mm_hadd_ps (tmp.m_type, tmp.m_type);
			#else 
				dgSimd tmp (_mm_add_ps (m_type, _mm_shuffle_ps(m_type, m_type, PURMUT_MASK(2, 3, 0, 1))));
				return _mm_add_ps (tmp.m_type, _mm_shuffle_ps(tmp.m_type, tmp.m_type, PURMUT_MASK(1, 0, 3, 2)));
			#endif
		}

		DG_INLINE dgSimd DotProduct (const dgSimd& data) const
		{
			dgSimd dot (_mm_mul_ps (m_type, data.m_type));
			return dot.AddHorizontal();
		}


		DG_INLINE dgSimd CrossProduct (const dgSimd& data) const
		{
			return _mm_sub_ps (_mm_mul_ps (_mm_shuffle_ps (m_type, m_type, PURMUT_MASK(3, 0, 2, 1)), _mm_shuffle_ps (data.m_type, data.m_type, PURMUT_MASK(3, 1, 0, 2))),
							   _mm_mul_ps (_mm_shuffle_ps (m_type, m_type, PURMUT_MASK(3, 1, 0, 2)), _mm_shuffle_ps (data.m_type, data.m_type, PURMUT_MASK(3, 0, 2, 1))));
		}

		DG_INLINE dgSimd Abs () const
		{
			__m128i shitSign = _mm_srli_epi32 (_mm_slli_epi32 (*((__m128i*) &m_type), 1), 1);
			return *(__m128*)&shitSign;
		}

		DG_INLINE dgSimd Floor () const
		{
			dgSimd mask ((dgFloat32 (1.5f) * dgFloat32 (1<<23)));
			dgSimd ret (_mm_sub_ps(_mm_add_ps(m_type, mask.m_type), mask.m_type));
			dgSimd adjust (_mm_cmplt_ps (m_type, ret.m_type));
			ret = _mm_sub_ps (ret.m_type, _mm_and_ps(_mm_set_ps1(1.0), adjust.m_type));
//			dgAssert (ret.m_f[0] == dgFloor(m_f[0]));
//			dgAssert (ret.m_f[1] == dgFloor(m_f[1]));
//			dgAssert (ret.m_f[2] == dgFloor(m_f[2]));
//			dgAssert (ret.m_f[3] == dgFloor(m_f[3]));
			return ret;
		}
		
		DG_INLINE dgInt32 GetSignMask() const
		{
			return _mm_movemask_ps(m_type);
		} 

		DG_INLINE dgSimd InvSqrt () const
		{
			dgSimd tmp0 (_mm_rsqrt_ps(m_type));
			return m_half * tmp0 * (m_three - (*this) * tmp0 * tmp0);
		}

		DG_INLINE dgSimd Elem0_ToVector() const
		{
			return dgSimd (_mm_shuffle_ps(m_type, m_type, PURMUT_MASK(0, 0, 0, 0)));
		}

		DG_INLINE dgSimd Elem1_ToVector() const
		{
			return dgSimd (_mm_shuffle_ps(m_type, m_type, PURMUT_MASK(1, 1, 1, 1)));
		}

		DG_INLINE dgSimd Elem2_ToVector() const
		{
			return dgSimd (_mm_shuffle_ps(m_type, m_type, PURMUT_MASK(2, 2, 2, 2)));
		}

		DG_INLINE dgSimd Elem3_ToVector() const
		{
			return dgSimd (_mm_shuffle_ps(m_type, m_type, PURMUT_MASK(3, 3, 3, 3)));
		}


		DG_INLINE dgSimd GetMin (const dgSimd& data) const
		{
			return _mm_min_ps (m_type, data.m_type);
		} 

		DG_INLINE dgSimd GetMax (const dgSimd& data) const
		{
			return _mm_max_ps (m_type, data.m_type);
		} 

		DG_INLINE dgSimd MoveHigh (const dgSimd& data) const
		{
			return _mm_movehl_ps (m_type, data.m_type);
		}

		DG_INLINE dgSimd MoveLow (const dgSimd& data) const
		{
			return _mm_movelh_ps (m_type, data.m_type);
		}

		DG_INLINE dgSimd PackLow (const dgSimd& data) const
		{
			return _mm_unpacklo_ps (m_type, data.m_type);
		}

		DG_INLINE dgSimd PackHigh (const dgSimd& data) const
		{
			return _mm_unpackhi_ps (m_type, data.m_type);
		}

		DG_INLINE dgSimd ShiftTripleRight () const
		{
			return _mm_shuffle_ps(m_type, m_type, PURMUT_MASK(3, 1, 0, 2));
		}

		DG_INLINE dgSimd ShiftTripleLeft () const
		{
			return _mm_shuffle_ps(m_type, m_type, PURMUT_MASK(3, 0, 2, 1));
		}

		DG_INLINE dgSimd MaximunValue() const
		{
			dgSimd tmp (GetMax (_mm_movehl_ps (m_type, m_type)));
			return tmp.GetMax (_mm_shuffle_ps(tmp.m_type, tmp.m_type, PURMUT_MASK(0, 0, 0, 1)));
		}

		DG_INLINE dgSimd MaximunValue(const dgSimd& data) const
		{
			dgSimd tmp1 (MoveHigh(*this));
			dgSimd mask ((*this) > tmp1);
			dgSimd tmp0 (GetMax(tmp1));
			dgSimd ret0 ((data & mask) | data.MoveHigh(data).AndNot(mask));

			tmp1 = tmp0.PackLow(tmp0);
			mask = tmp0 > MoveHigh(tmp1);
			dgSimd ret1 (ret0.PackLow(ret0));
			return (ret0 & mask) | ret1.MoveHigh(ret1).AndNot(mask);
		}


		DG_INLINE static void Transpose4x4 (dgSimd& dst0, dgSimd& dst1, dgSimd& dst2, dgSimd& dst3, 
											const dgSimd& src0, const dgSimd& src1, const dgSimd& src2, const dgSimd& src3)
		{
			dgSimd tmp0 (src0.PackLow(src1));
			dgSimd tmp1 (src2.PackLow(src3));
			dst0 = tmp0.MoveLow (tmp1);
			dst1 = tmp1.MoveHigh (tmp0);

			tmp0 = src0.PackHigh(src1);
			tmp1 = src2.PackHigh(src3);
			dst2 = tmp0.MoveLow (tmp1);
			dst3 = tmp1.MoveHigh (tmp0);
		}

		union {
			__m128 m_type;
			dgInt32 m_i[4];
			dgFloat32 m_f[4];
		};


		static dgSimd m_signMask;
		static dgSimd m_allOneMask;
		static dgSimd m_triplexMask;
		static dgSimd m_aabbPadding;
		static dgSimd m_index_0123;
		static dgSimd m_index_4567;
		static dgSimd m_half;
		static dgSimd m_zero;
		static dgSimd m_one;
		static dgSimd m_three;
		static dgSimd m_negOne;

	} DG_GCC_VECTOR_ALIGMENT;

#else 


	#define simd_env					dgUnsigned32
	#define simd_get_ctrl()				0
	#define simd_set_ctrl(a)			a;
	#define simd_set_FZ_mode()		

	DG_MSC_VECTOR_ALIGMENT
	class dgSimd
	{
		public:

		static DG_INLINE dgInt32 get_ctrl()
		{
			return 0;
		}

		static DG_INLINE void set_ctrl(dgInt32 a)
		{
		}

		static DG_INLINE void set_FZ_mode()
		{
		}

		DG_INLINE dgSimd () {}
		DG_INLINE dgSimd (dgFloat32 a)
		{
			m_f[0] = a; m_f[1] = a; m_f[2] = a; m_f[3] = a;
		}
		DG_INLINE dgSimd (const dgSimd& data)
		{
			m_f[0] = data.m_f[0]; m_f[1] = data.m_f[1]; m_f[2] = data.m_f[2]; m_f[3] = data.m_f[3];
		}
		DG_INLINE dgSimd (dgInt32 a) 
		{
			m_i[0] = a; m_i[1] = a; m_i[2] = a; m_i[3] = a; 
		}
		DG_INLINE dgSimd (const dgFloat32* const ptr)
		{
			m_f[0] = ptr[0]; m_f[1] = ptr[1]; m_f[2] = ptr[2]; m_f[3] = ptr[3]; 
		}
		DG_INLINE dgSimd (dgFloat32 x, dgFloat32 y, dgFloat32 z, dgFloat32 w)
		{
			m_f[0] = x; m_f[1] = y; m_f[2] = z; m_f[3] = w; 
		}
		DG_INLINE dgSimd (dgInt32 ix, dgInt32 iy, dgInt32 iz, dgInt32 iw)
		{
			m_i[0] = ix; m_i[1] = iy; m_i[2] = iz; m_i[3] = iw; 
		}
		DG_INLINE dgInt32 GetInt () const
		{
			return int (m_f[0]);
		}

		DG_INLINE dgSimd Elem0_ToVector() const
		{
			return dgSimd (m_f[0]);
		}

		DG_INLINE dgSimd Elem1_ToVector() const
		{
			return dgSimd (m_f[1]);
		}

		DG_INLINE dgSimd Elem2_ToVector() const
		{
			return dgSimd (m_f[2]);
		}

		DG_INLINE dgSimd Elem3_ToVector() const
		{
			return dgSimd (m_f[3]);
		}


		DG_INLINE void StoreScalar(float* const scalar) const
		{
			scalar[0] = m_f[0];
		}

		DG_INLINE void StoreVector(float* const array) const
		{
			array[0] = m_f[0]; array[1] = m_f[1]; array[2] = m_f[2]; array[3] = m_f[3]; 
		}

		DG_INLINE dgSimd operator= (const dgSimd& data)
		{
			m_i[0] = data.m_i[0]; m_i[1] = data.m_i[1]; m_i[2] = data.m_i[2]; m_i[3] = data.m_i[3];
			return (*this);
		}

		DG_INLINE dgSimd operator+ (const dgSimd& data) const
		{
			return dgSimd (m_f[0] + data.m_f[0], m_f[1] + data.m_f[1], m_f[2] + data.m_f[2], m_f[3] + data.m_f[3]); 
		}

		DG_INLINE dgSimd operator- (const dgSimd& data) const
		{
			return dgSimd (m_f[0] - data.m_f[0], m_f[1] - data.m_f[1], m_f[2] - data.m_f[2], m_f[3] - data.m_f[3]); 
		}

		DG_INLINE dgSimd operator* (const dgSimd& data) const
		{
			return dgSimd (m_f[0] * data.m_f[0], m_f[1] * data.m_f[1], m_f[2] * data.m_f[2], m_f[3] * data.m_f[3]); 
		}

		DG_INLINE dgSimd operator/ (const dgSimd& data) const
		{
			return dgSimd (m_f[0] / data.m_f[0], m_f[1] / data.m_f[1], m_f[2] / data.m_f[2], m_f[3] / data.m_f[3]); 
		}

		DG_INLINE dgSimd operator<= (const dgSimd& data) const
		{
			return dgSimd ((m_f[0] <= data.m_f[0]) ? dgInt32 (0xffffffff) : dgInt32 (0),
							 (m_f[1] <= data.m_f[1]) ? dgInt32 (0xffffffff) : dgInt32 (0),
							 (m_f[2] <= data.m_f[2]) ? dgInt32 (0xffffffff) : dgInt32 (0),
							 (m_f[3] <= data.m_f[3]) ? dgInt32 (0xffffffff) : dgInt32 (0));
		}

		DG_INLINE dgSimd operator>= (const dgSimd& data) const
		{
			return dgSimd ((m_f[0] >= data.m_f[0]) ? dgInt32 (0xffffffff) : dgInt32 (0),
							 (m_f[1] >= data.m_f[1]) ? dgInt32 (0xffffffff) : dgInt32 (0),
				             (m_f[2] >= data.m_f[2]) ? dgInt32 (0xffffffff) : dgInt32 (0),
				             (m_f[3] >= data.m_f[3]) ? dgInt32 (0xffffffff) : dgInt32 (0));
		}

		DG_INLINE dgSimd operator< (const dgSimd& data) const
		{
			return dgSimd ((m_f[0] < data.m_f[0]) ? dgInt32 (0xffffffff) : dgInt32 (0),
						     (m_f[1] < data.m_f[1]) ? dgInt32 (0xffffffff) : dgInt32 (0),
				             (m_f[2] < data.m_f[2]) ? dgInt32 (0xffffffff) : dgInt32 (0),
				             (m_f[3] < data.m_f[3]) ? dgInt32 (0xffffffff) : dgInt32 (0));

		}

		DG_INLINE dgSimd operator> (const dgSimd& data) const
		{
			return dgSimd ((m_f[0] > data.m_f[0]) ? dgInt32 (0xffffffff) : dgInt32 (0),
						     (m_f[1] > data.m_f[1]) ? dgInt32 (0xffffffff) : dgInt32 (0),
				             (m_f[2] > data.m_f[2]) ? dgInt32 (0xffffffff) : dgInt32 (0),
				             (m_f[3] > data.m_f[3]) ? dgInt32 (0xffffffff) : dgInt32 (0));
		}


		DG_INLINE dgSimd operator& (const dgSimd& data) const
		{
			return dgSimd (m_i[0] & data.m_i[0], m_i[1] & data.m_i[1], m_i[2] & data.m_i[2], m_i[3] & data.m_i[3]);	
		}

		DG_INLINE dgSimd operator| (const dgSimd& data) const
		{
			return dgSimd (m_i[0] | data.m_i[0], m_i[1] | data.m_i[1], m_i[2] | data.m_i[2], m_i[3] | data.m_i[3]);	
		}

		DG_INLINE dgSimd AndNot (const dgSimd& data) const
		{
			return dgSimd (m_i[0] & ~data.m_i[0], m_i[1] & ~data.m_i[1], m_i[2] & ~data.m_i[2], m_i[3] & ~data.m_i[3]);	
		}

		DG_INLINE dgSimd AddHorizontal () const
		{
			return dgSimd (m_f[0] + m_f[1] + m_f[2] + m_f[3]);
		}

		DG_INLINE dgSimd DotProduct (const dgSimd& data) const
		{
			return dgSimd (m_f[0] * data.m_f[0] + m_f[1] * data.m_f[1] + m_f[2] * data.m_f[2] + m_f[3] * data.m_f[3]);
		}

		DG_INLINE dgSimd DotProduct_SSE4 (const dgSimd& data) const
		{
			return dgSimd (m_f[0] * data.m_f[0] + m_f[1] * data.m_f[1] + m_f[2] * data.m_f[2] + m_f[3] * data.m_f[3]);
		}

		DG_INLINE dgSimd CrossProduct (const dgSimd& data) const
		{
			return dgSimd (m_f[1] * data.m_f[2] - m_f[2] * data.m_f[1],
							 m_f[2] * data.m_f[0] - m_f[0] * data.m_f[2],
				             m_f[0] * data.m_f[1] - m_f[1] * data.m_f[0], m_f[3]);
		}

		DG_INLINE dgSimd Abs () const
		{
			return dgSimd ((m_f[0] > dgFloat32 (0.0f)) ? m_f[0] : -m_f[0],
							 (m_f[1] > dgFloat32 (0.0f)) ? m_f[1] : -m_f[1],
							 (m_f[2] > dgFloat32 (0.0f)) ? m_f[2] : -m_f[2],
							 (m_f[3] > dgFloat32 (0.0f)) ? m_f[3] : -m_f[3]);
		}

		DG_INLINE dgSimd Floor () const
		{
			return dgSimd (dgFloor (m_f[0]), dgFloor (m_f[1]), dgFloor (m_f[2]), dgFloor (m_f[3]));
		}

		DG_INLINE dgInt32 GetSignMask() const
		{
			//return (((m_f[0] >= 0) ? 1 : 0) | ((m_f[1] >= 0) ? 2 : 0) | ((m_f[2] >= 0) ? 4 : 0) | ((m_f[3] >= 0) ? 8 : 0));
			return (((m_i[0] & 0x80000000) ? 1 : 0) | ((m_i[1] & 0x80000000) ? 2 : 0) | ((m_i[2] & 0x80000000) ? 4 : 0) | ((m_i[3] & 0x80000000) ? 8 : 0));
		} 

		DG_INLINE dgSimd InvSqrt () const
		{
			return dgSimd (dgRsqrt (m_f[0]), dgRsqrt (m_f[1]), dgRsqrt (m_f[2]), dgRsqrt (m_f[3]));
		}

		DG_INLINE dgSimd GetMin (const dgSimd& data) const
		{
			return dgSimd ((m_f[0] < data.m_f[0]) ? m_f[0] : data.m_f[0],
							 (m_f[1] < data.m_f[1]) ? m_f[1] : data.m_f[1],
      			             (m_f[2] < data.m_f[2]) ? m_f[2] : data.m_f[2],
				             (m_f[3] < data.m_f[3]) ? m_f[3] : data.m_f[3]);
		} 

		DG_INLINE dgSimd GetMax (const dgSimd& data) const
		{
			return dgSimd ((m_f[0] > data.m_f[0]) ? m_f[0] : data.m_f[0],
				             (m_f[1] > data.m_f[1]) ? m_f[1] : data.m_f[1],
				             (m_f[2] > data.m_f[2]) ? m_f[2] : data.m_f[2],
				             (m_f[3] > data.m_f[3]) ? m_f[3] : data.m_f[3]);
		} 


		DG_INLINE dgSimd MoveHigh (const dgSimd& data) const
		{
			return dgSimd (data.m_f[2], data.m_f[3], m_f[2], m_f[3]); 
		}

		DG_INLINE dgSimd MoveLow (const dgSimd& data) const
		{
			return dgSimd (m_f[0], m_f[1], data.m_f[0], data.m_f[1]); 
		}

		DG_INLINE dgSimd PackLow (const dgSimd& data) const
		{
			return dgSimd (m_f[0], data.m_f[0], m_f[1], data.m_f[1]); 
		}

		DG_INLINE dgSimd PackHigh (const dgSimd& data) const
		{
			return dgSimd (m_f[2], data.m_f[2], m_f[3], data.m_f[3]); 
		}

		DG_INLINE dgSimd ShiftTripleRight () const
		{
			return dgSimd (m_f[2], m_f[0], m_f[1], m_f[3]); 
		}

		DG_INLINE dgSimd ShiftTripleLeft () const
		{
			return dgSimd (m_f[1], m_f[2], m_f[0], m_f[3]); 
		}

		DG_INLINE dgSimd MaximunValue() const
		{
			dgFloat32 maxVal = ::dgMax (::dgMax (m_f[0], m_f[1]), ::dgMax (m_f[2], m_f[3]));
			return dgSimd (maxVal);
		}

		DG_INLINE dgSimd MaximunValue(const dgSimd& data) const
		{
			dgSimd tmp0 (*this);
			dgSimd tmp1 (MoveHigh(tmp0));
			dgSimd mask (tmp0 > tmp1);
			tmp0 = tmp0.GetMax(tmp1);
			dgSimd ret0 ((data & mask) | data.MoveHigh(data).AndNot(mask));

			tmp1 = tmp0.PackLow(tmp0);
			mask = tmp0 > MoveHigh(tmp1);
			dgSimd ret1 (ret0.PackLow(ret0));
			return (ret0 & mask) | ret1.MoveHigh(ret1).AndNot(mask);
		}

		DG_INLINE static void Transpose4x4 (dgSimd& dst0, dgSimd& dst1, dgSimd& dst2, dgSimd& dst3, 
			const dgSimd& src0, const dgSimd& src1, const dgSimd& src2, const dgSimd& src3)
		{
			dgSimd tmp0 (src0.PackLow(src1));
			dgSimd tmp1 (src2.PackLow(src3));
			dst0 = tmp0.MoveLow (tmp1);
			dst1 = tmp1.MoveHigh (tmp0);

			tmp0 = src0.PackHigh(src1);
			tmp1 = src2.PackHigh(src3);
			dst2 = tmp0.MoveLow (tmp1);
			dst3 = tmp1.MoveHigh (tmp0);
		}


		union {
			dgInt32 m_i[4];
			dgFloat32 m_f[4];
		};

		static dgSimd m_signMask;
		static dgSimd m_allOneMask;
		static dgSimd m_triplexMask;
		static dgSimd m_aabbPadding;
		static dgSimd m_index_0123;
		static dgSimd m_index_4567;
		static dgSimd m_half;
		static dgSimd m_zero;
		static dgSimd m_one;
		static dgSimd m_three;
		static dgSimd m_negOne;
	}DG_GCC_VECTOR_ALIGMENT;


#endif




#endif

