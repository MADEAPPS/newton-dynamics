/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_SIMD_VECTOR8_AVX2_H__
#define __ND_SIMD_VECTOR8_AVX2_H__

#include "ndCollision.h"

#ifdef D_NEWTON_USE_DOUBLE
	D_MSV_NEWTON_CLASS_ALIGN_32
	class ndVector8
	{
		public:
		inline ndVector8()
		{
		}

		inline ndVector8(const ndFloat32 val)
			:m_low(_mm256_set1_pd(val))
			,m_high(_mm256_set1_pd(val))
		{
		}

		inline ndVector8(const ndInt32 val)
			:m_low(_mm256_castsi256_pd(_mm256_set1_epi64x(ndInt64(val))))
			,m_high(_mm256_castsi256_pd(_mm256_set1_epi64x(ndInt64(val))))
		{
		}
				
		inline ndVector8(const __m256d low, const __m256d high)
			:m_low(low)
			,m_high(high)
		{
		}

		inline ndVector8(const ndVector8& copy)
			:m_low(copy.m_low)
			,m_high(copy.m_high)
		{
		}

		#ifdef D_SCALAR_VECTOR_CLASS
			inline ndVector8(const ndVector& low, const ndVector& high)
				//:m_low(low.m_type)
				//,m_high(high.m_type)
			{
				m_vector8.m_linear = low;
				m_vector8.m_angular = high;
			}
		#else
			inline ndVector8(const ndVector& low, const ndVector& high)
				:m_low(_mm256_set_m128d(low.m_typeHigh, low.m_typeLow))
				,m_high(_mm256_set_m128d(high.m_typeHigh, high.m_typeLow))
			{
			}
		#endif

		inline ndVector8(const ndVector8* const baseAddr, const ndVector8& index)
			:m_low(_mm256_i64gather_pd(&(*baseAddr)[0], index.m_lowInt, 8))
			,m_high(_mm256_i64gather_pd(&(*baseAddr)[0], index.m_highInt, 8))
		{
		}

		inline ndFloat32& operator[] (ndInt32 i)
		{
			ndAssert(i >= 0);
			ndAssert(i < D_SIMD8_WORK_GROUP_SIZE);
			ndFloat32* const ptr = (ndFloat32*)&m_low;
			return ptr[i];
		}

		inline const ndFloat32& operator[] (ndInt32 i) const
		{
			ndAssert(i >= 0);
			ndAssert(i < D_SIMD8_WORK_GROUP_SIZE);
			const ndFloat32* const ptr = (ndFloat32*)&m_low;
			return ptr[i];
		}

		inline ndVector8& operator= (const ndVector8& A)
		{
			m_low = A.m_low;
			m_high = A.m_high;
			return *this;
		}

		inline ndVector8 operator+ (const ndVector8& A) const
		{
			return ndVector8(_mm256_add_pd(m_low, A.m_low), _mm256_add_pd(m_high, A.m_high));
		}

		inline ndVector8 operator- (const ndVector8& A) const
		{
			return ndVector8(_mm256_sub_pd(m_low, A.m_low), _mm256_sub_pd(m_high, A.m_high));
		}

		inline ndVector8 operator* (const ndVector8& A) const
		{
			return ndVector8(_mm256_mul_pd(m_low, A.m_low), _mm256_mul_pd(m_high, A.m_high));
		}

		inline ndVector8 MulAdd(const ndVector8& A, const ndVector8& B) const
		{
			return ndVector8(_mm256_fmadd_pd(A.m_low, B.m_low, m_low), _mm256_fmadd_pd(A.m_high, B.m_high, m_high));
		}

		inline ndVector8 MulSub(const ndVector8& A, const ndVector8& B) const
		{
			return ndVector8(_mm256_fnmadd_pd(A.m_low, B.m_low, m_low), _mm256_fnmadd_pd(A.m_high, B.m_high, m_high));
		}

		inline ndVector8 operator> (const ndVector8& A) const
		{
			return ndVector8(_mm256_cmp_pd(m_low, A.m_low, _CMP_GT_OQ), _mm256_cmp_pd(m_high, A.m_high, _CMP_GT_OQ));
		}

		inline ndVector8 operator< (const ndVector8& A) const
		{
			return ndVector8(_mm256_cmp_pd(m_low, A.m_low, _CMP_LT_OQ), _mm256_cmp_pd(m_high, A.m_high, _CMP_LT_OQ));
		}

		inline ndVector8 operator| (const ndVector8& A) const
		{
			return ndVector8(_mm256_or_pd(m_low, A.m_low), _mm256_or_pd(m_high, A.m_high));
		}

		inline ndVector8 operator& (const ndVector8& A) const
		{
			return ndVector8(_mm256_and_pd(m_low, A.m_low), _mm256_and_pd(m_high, A.m_high));
		}

		inline ndVector8 GetMin(const ndVector8& A) const
		{
			return ndVector8(_mm256_min_pd(m_low, A.m_low), _mm256_min_pd(m_high, A.m_high));
		}

		inline ndVector8 GetMax(const ndVector8& A) const
		{
			return ndVector8(_mm256_max_pd(m_low, A.m_low), _mm256_max_pd(m_high, A.m_high));
		}

		inline ndVector8 Select(const ndVector8& data, const ndVector8& mask) const
		{
			// (((b ^ a) & mask)^a)
			//return  _mm_or_ps (_mm_and_ps (mask.m_type, data.m_type), _mm_andnot_ps(mask.m_type, m_type));
			//return  _mm256_xor_ps(m_type, _mm256_and_ps(mask.m_type, _mm256_xor_ps(m_type, data.m_type)));
			__m256d low (_mm256_xor_pd(m_low, _mm256_and_pd(mask.m_low, _mm256_xor_pd(m_low, data.m_low))));
			__m256d high(_mm256_xor_pd(m_high, _mm256_and_pd(mask.m_high, _mm256_xor_pd(m_high, data.m_high))));
			return ndVector8(low, high);
		}

		inline ndVector GetLow() const
		{
			return m_vector8.m_linear;
		}

		inline ndVector GetHigh() const
		{
			return m_vector8.m_angular;
		}

		inline ndFloat32 GetMax() const
		{
			__m256d tmp0(_mm256_max_pd(m_low, m_high));
			__m256d tmp1(_mm256_max_pd(tmp0, _mm256_permute2f128_pd(tmp0, tmp0, 1)));
			__m256d tmp2(_mm256_max_pd(tmp1, _mm256_unpackhi_pd(tmp1, tmp1)));
			return _mm256_cvtsd_f64(tmp2);
		}

		inline ndFloat32 AddHorizontal() const
		{
			__m256d tmp0(_mm256_add_pd(m_low, m_high));
			__m256d tmp1(_mm256_add_pd(tmp0, _mm256_permute2f128_pd(tmp0, tmp0, 1)));
			__m256d tmp2(_mm256_add_pd(tmp1, _mm256_unpackhi_pd(tmp1, tmp1)));
			return _mm256_cvtsd_f64(tmp2);
		}

		static inline void FlushRegisters()
		{
			_mm256_zeroall();
		}

		static inline void Transpose(
			__m256d& dst0, __m256d& dst1, __m256d& dst2, __m256d& dst3,
			const __m256d& src0, const __m256d& src1, const __m256d& src2, const __m256d& src3)
		{
			__m256d tmp[4];
			tmp[0] = _mm256_permute2f128_pd(src0, src2, 0x20);
			tmp[1] = _mm256_permute2f128_pd(src1, src3, 0x20);
			tmp[2] = _mm256_permute2f128_pd(src0, src2, 0x31);
			tmp[3] = _mm256_permute2f128_pd(src1, src3, 0x31);

			dst0 = _mm256_unpacklo_pd(tmp[0], tmp[1]);
			dst1 = _mm256_unpackhi_pd(tmp[0], tmp[1]);
			dst2 = _mm256_unpacklo_pd(tmp[2], tmp[3]);
			dst3 = _mm256_unpackhi_pd(tmp[2], tmp[3]);
		}

		static inline void Transpose(
			ndVector8& dst0, ndVector8& dst1, ndVector8& dst2, ndVector8& dst3,
			ndVector8& dst4, ndVector8& dst5, ndVector8& dst6, ndVector8& dst7,
			const ndVector8& src0, const ndVector8& src1, const ndVector8& src2, const ndVector8& src3,
			const ndVector8& src4, const ndVector8& src5, const ndVector8& src6, const ndVector8& src7)
		{
			Transpose(
				dst0.m_low, dst1.m_low, dst2.m_low, dst3.m_low,
				src0.m_low, src1.m_low, src2.m_low, src3.m_low);

			Transpose(
				dst0.m_high, dst1.m_high, dst2.m_high, dst3.m_high,
				src4.m_low, src5.m_low, src6.m_low, src7.m_low);

			Transpose(
				dst4.m_low, dst5.m_low, dst6.m_low, dst7.m_low,
				src0.m_high, src1.m_high, src2.m_high, src3.m_high);

			Transpose(
				dst4.m_high, dst5.m_high, dst6.m_high, dst7.m_high,
				src4.m_high, src5.m_high, src6.m_high, src7.m_high);
		}

		union
		{
			struct
			{
				__m256d m_low;
				__m256d m_high;
			};
			struct
			{
				__m256i m_lowInt;
				__m256i m_highInt;
			};
			ndJacobian m_vector8;
			ndInt64 m_int[D_SIMD8_WORK_GROUP_SIZE];
		};

		static ndVector8 m_one;
		static ndVector8 m_zero;
		static ndVector8 m_mask;
		static ndVector8 m_ordinals;
	} D_GCC_NEWTON_CLASS_ALIGN_32;

#else
	D_MSV_NEWTON_CLASS_ALIGN_32
	class ndVector8
	{
		public:
		#ifndef PERMUTE_MASK
		#define PERMUTE_MASK(w, z, y, x) _MM_SHUFFLE (w, z, y, x)
		#endif

		inline ndVector8()
		{
		}

		inline ndVector8(const ndFloat32 val)
			:m_type(_mm256_set1_ps(val))
		{
		}

		inline ndVector8(const ndInt32 val)
			:m_type(_mm256_castsi256_ps(_mm256_set1_epi32(val)))
		{
		}

		inline ndVector8(const __m256 type)
			: m_type(type)
		{
		}

		inline ndVector8(const ndVector8& copy)
			: m_type(copy.m_type)
		{
		}

		inline ndVector8(const ndVector& low, const ndVector& high)
			#ifdef D_SCALAR_VECTOR_CLASS
			:m_type(_mm256_set_m128(_mm_set_ps(high.m_w, high.m_z, high.m_y, high.m_x), _mm_set_ps(low.m_w, low.m_z, low.m_y, low.m_x)))
			#else
			:m_type(_mm256_set_m128(high.m_type, low.m_type))
			#endif
		{
		}

		inline ndVector8(const ndVector8* const baseAddr, const ndVector8& index)
			: m_type(_mm256_i32gather_ps(&(*baseAddr)[0], index.m_typeInt, 4))
		{
		}

		inline ndFloat32& operator[] (ndInt32 i)
		{
			ndAssert(i >= 0);
			ndAssert(i < D_SIMD8_WORK_GROUP_SIZE);
			ndFloat32* const ptr = (ndFloat32*)&m_type;
			return ptr[i];
		}

		inline const ndFloat32& operator[] (ndInt32 i) const
		{
			ndAssert(i >= 0);
			ndAssert(i < D_SIMD8_WORK_GROUP_SIZE);
			const ndFloat32* const ptr = (ndFloat32*)&m_type;
			return ptr[i];
		}

		inline ndVector8& operator= (const ndVector8& A)
		{
			m_type = A.m_type;
			return *this;
		}

		inline ndVector8 operator+ (const ndVector8& A) const
		{
			return _mm256_add_ps(m_type, A.m_type);
		}

		inline ndVector8 operator- (const ndVector8& A) const
		{
			return _mm256_sub_ps(m_type, A.m_type);
		}

		inline ndVector8 operator* (const ndVector8& A) const
		{
			return _mm256_mul_ps(m_type, A.m_type);
		}

		inline ndVector8 MulAdd(const ndVector8& A, const ndVector8& B) const
		{
			return _mm256_fmadd_ps(A.m_type, B.m_type, m_type);
		}

		inline ndVector8 MulSub(const ndVector8& A, const ndVector8& B) const
		{
			return _mm256_fnmadd_ps(A.m_type, B.m_type, m_type);
		}

		inline ndVector8 operator> (const ndVector8& A) const
		{
			return _mm256_cmp_ps(m_type, A.m_type, _CMP_GT_OQ);
		}

		inline ndVector8 operator< (const ndVector8& A) const
		{
			return _mm256_cmp_ps(m_type, A.m_type, _CMP_LT_OQ);
		}

		inline ndVector8 operator| (const ndVector8& A) const
		{
			return _mm256_or_ps(m_type, A.m_type);
		}

		inline ndVector8 operator& (const ndVector8& A) const
		{
			return _mm256_and_ps(m_type, A.m_type);
		}

		inline ndVector8 GetMin(const ndVector8& A) const
		{
			return _mm256_min_ps(m_type, A.m_type);
		}

		inline ndVector8 GetMax(const ndVector8& A) const
		{
			return _mm256_max_ps(m_type, A.m_type);
		}

		inline ndVector8 Select(const ndVector8& data, const ndVector8& mask) const
		{
			// (((b ^ a) & mask)^a)
			//return  _mm_or_ps (_mm_and_ps (mask.m_type, data.m_type), _mm_andnot_ps(mask.m_type, m_type));
			return  _mm256_xor_ps(m_type, _mm256_and_ps(mask.m_type, _mm256_xor_ps(m_type, data.m_type)));
		}

		inline ndVector GetLow() const
		{
			return m_vector8.m_linear;
		}

		inline ndVector GetHigh() const
		{
			return m_vector8.m_angular;
		}

		inline ndFloat32 GetMax() const
		{
			__m128 tmp0(_mm_max_ps(m_typeLow, m_typeHigh));
			__m128 tmp1(_mm_max_ps(tmp0, _mm_movehl_ps(tmp0, tmp0)));
			__m128 tmp2(_mm_max_ps(tmp1, _mm_shuffle_ps(tmp1, tmp1, PERMUTE_MASK(2, 3, 0, 1))));
			return _mm_cvtss_f32(tmp2);
		}

		inline ndFloat32 AddHorizontal() const
		{
			__m128 tmp0(_mm_add_ps(m_typeLow, m_typeHigh));
			__m128 tmp1(_mm_add_ps(tmp0, _mm_movehl_ps(tmp0, tmp0)));
			__m128 tmp2(_mm_add_ps(tmp1, _mm_shuffle_ps(tmp1, tmp1, PERMUTE_MASK(2, 3, 0, 1))));
			return _mm_cvtss_f32(tmp2);
		}

		static inline void FlushRegisters()
		{
			_mm256_zeroall();
		}

		static inline void Transpose(
			ndVector8& dst0, ndVector8& dst1, ndVector8& dst2, ndVector8& dst3, 
			ndVector8& dst4, ndVector8& dst5, ndVector8& dst6, ndVector8& dst7,
			const ndVector8& src0, const ndVector8& src1, const ndVector8& src2, const ndVector8& src3,
			const ndVector8& src4, const ndVector8& src5, const ndVector8& src6, const ndVector8& src7)
		{
			ndVector8 blocks4x4[8];

			blocks4x4[0].m_type = _mm256_permute2f128_ps(src0.m_type, src4.m_type, 0x20);
			blocks4x4[1].m_type = _mm256_permute2f128_ps(src0.m_type, src4.m_type, 0x31);
			blocks4x4[2].m_type = _mm256_permute2f128_ps(src1.m_type, src5.m_type, 0x20);
			blocks4x4[3].m_type = _mm256_permute2f128_ps(src1.m_type, src5.m_type, 0x31);
			blocks4x4[4].m_type = _mm256_permute2f128_ps(src2.m_type, src6.m_type, 0x20);
			blocks4x4[5].m_type = _mm256_permute2f128_ps(src2.m_type, src6.m_type, 0x31);
			blocks4x4[6].m_type = _mm256_permute2f128_ps(src3.m_type, src7.m_type, 0x20);
			blocks4x4[7].m_type = _mm256_permute2f128_ps(src3.m_type, src7.m_type, 0x31);

			ndVector8 blocks2x2[8];
			blocks2x2[0].m_type = _mm256_unpacklo_ps(blocks4x4[0].m_type, blocks4x4[4].m_type);
			blocks2x2[1].m_type = _mm256_unpackhi_ps(blocks4x4[0].m_type, blocks4x4[4].m_type);
			blocks2x2[2].m_type = _mm256_unpacklo_ps(blocks4x4[1].m_type, blocks4x4[5].m_type);
			blocks2x2[3].m_type = _mm256_unpackhi_ps(blocks4x4[1].m_type, blocks4x4[5].m_type);
			blocks2x2[4].m_type = _mm256_unpacklo_ps(blocks4x4[2].m_type, blocks4x4[6].m_type);
			blocks2x2[5].m_type = _mm256_unpackhi_ps(blocks4x4[2].m_type, blocks4x4[6].m_type);
			blocks2x2[6].m_type = _mm256_unpacklo_ps(blocks4x4[3].m_type, blocks4x4[7].m_type);
			blocks2x2[7].m_type = _mm256_unpackhi_ps(blocks4x4[3].m_type, blocks4x4[7].m_type);
			
			dst0.m_type = _mm256_unpacklo_ps(blocks2x2[0].m_type, blocks2x2[4].m_type);
			dst1.m_type = _mm256_unpackhi_ps(blocks2x2[0].m_type, blocks2x2[4].m_type);
			dst2.m_type = _mm256_unpacklo_ps(blocks2x2[1].m_type, blocks2x2[5].m_type);
			dst3.m_type = _mm256_unpackhi_ps(blocks2x2[1].m_type, blocks2x2[5].m_type);
			dst4.m_type = _mm256_unpacklo_ps(blocks2x2[2].m_type, blocks2x2[6].m_type);
			dst5.m_type = _mm256_unpackhi_ps(blocks2x2[2].m_type, blocks2x2[6].m_type);
			dst6.m_type = _mm256_unpacklo_ps(blocks2x2[3].m_type, blocks2x2[7].m_type);
			dst7.m_type = _mm256_unpackhi_ps(blocks2x2[3].m_type, blocks2x2[7].m_type);
		}

		union
		{
			ndFloat32 m_float[ND_SIMD8_WORK_GROUP_SIZE];
			ndInt32 m_int[ND_SIMD8_WORK_GROUP_SIZE];
			ndJacobian m_vector8;

			__m256 m_type;
			__m256i m_typeInt;
			struct
			{
				__m128 m_typeLow;
				__m128 m_typeHigh;
			};
		};

		static ndVector8 m_one;
		static ndVector8 m_zero;
		static ndVector8 m_mask;
		static ndVector8 m_ordinals;
	} D_GCC_NEWTON_CLASS_ALIGN_32;
#endif

#endif