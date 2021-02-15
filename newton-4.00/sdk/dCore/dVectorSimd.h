/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_VECTOR_X86_SIMD_H__
#define __D_VECTOR_X86_SIMD_H__

#ifndef DG_SCALAR_VECTOR_CLASS

#ifdef D_NEWTON_USE_DOUBLE
	#define dVector dBigVector
#else

class dBigVector;
// *****************************************************************************************
//
// 4 x 1 single precision SSE vector class declaration
//
// *****************************************************************************************
D_MSV_NEWTON_ALIGN_16
class dVector
{
	#define PERMUTE_MASK(w, z, y, x) _MM_SHUFFLE (w, z, y, x)
	public:
	D_INLINE dVector() 
	{
	}

	D_INLINE dVector(const __m128i type)
		:m_typeInt (type)
	{
	}

	D_INLINE dVector(const __m128 type)
		: m_type(type)
	{
	}

	D_INLINE dVector (const dFloat32 a)
		:m_type(_mm_set_ps1(a)) 
	{
	}

	D_INLINE dVector (const dFloat32* const ptr)
		:m_type(_mm_loadu_ps (ptr))
	{
	}

#ifndef	D_NEWTON_USE_DOUBLE
	D_INLINE dVector(const dFloat64* const ptr)
		:m_type(_mm_set_ps(dFloat32(ptr[3]), dFloat32(ptr[2]), dFloat32(ptr[1]), dFloat32(ptr[0])))
	{
	}
#endif

	D_INLINE dVector (const dVector& copy)
		:m_type(copy.m_type)
	{
	}

	D_INLINE dVector (const dBigVector& copy)
		:m_type(_mm_shuffle_ps (_mm_cvtpd_ps (((__m128d*)&copy)[0]), _mm_cvtpd_ps (((__m128d*)&copy)[1]), PERMUTE_MASK(1, 0, 1, 0)))
	{
		dAssert (dCheckVector ((*this)));
	}

	D_INLINE dVector (dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w)
		:m_type(_mm_set_ps(w, z, y, x))
	{
	}

	D_INLINE dVector (dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
		:m_type(_mm_set_ps(*(dFloat32*)&iw, *(dFloat32*)&iz, *(dFloat32*)&iy, *(dFloat32*)&ix))
	{
	}

	D_INLINE void *operator new[](size_t size)
	{
		return dMemory::Malloc(size);
	}

	D_INLINE void *operator new (size_t size)
	{
		return dMemory::Malloc(size);
	}

	D_INLINE void operator delete[] (void* ptr)
	{
		dMemory::Free(ptr);
	}

	D_INLINE void operator delete (void* ptr)
	{
		dMemory::Free(ptr);
	}


	D_INLINE dFloat32 GetScalar () const
	{
		//return m_x;
		return _mm_cvtss_f32 (m_type);
	}

	D_INLINE void Store (dFloat32* const dst) const
	{
		_mm_storeu_ps(dst, m_type);
	}

	D_INLINE dVector BroadcastX () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(0, 0, 0, 0));
	}

	D_INLINE dVector BroadcastY () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(1, 1, 1, 1));
	}

	D_INLINE dVector BroadcastZ () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(2, 2, 2, 2));
	}

	D_INLINE dVector BroadcastW () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(3, 3, 3, 3));
	}

	D_INLINE dVector Scale (dFloat32 s) const
	{
		return _mm_mul_ps (m_type, _mm_set_ps1(s));
	}

	D_INLINE dFloat32& operator[] (dInt32 i)
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return m_f[i];
	}

	D_INLINE const dFloat32& operator[] (dInt32 i) const
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return m_f[i];
	}

	D_INLINE dVector operator+ (const dVector& A) const
	{
		return _mm_add_ps (m_type, A.m_type);	
	}

	D_INLINE dVector operator- (const dVector& A) const 
	{
		return _mm_sub_ps (m_type, A.m_type);	
	}

	D_INLINE dVector operator* (const dVector& A) const
	{
		return _mm_mul_ps(m_type, A.m_type);
	}

	D_INLINE dVector& operator+= (const dVector& A)
	{
		return (*this = _mm_add_ps (m_type, A.m_type));
	}

	D_INLINE dVector& operator-= (const dVector& A)
	{
		return (*this = _mm_sub_ps (m_type, A.m_type));
	}

	D_INLINE dVector& operator*= (const dVector& A)
	{
		return (*this = _mm_mul_ps(m_type, A.m_type));
	}

	// return cross product
	D_INLINE dVector CrossProduct (const dVector& B) const
	{
		return _mm_sub_ps (_mm_mul_ps (_mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(3, 0, 2, 1)), _mm_shuffle_ps (B.m_type, B.m_type, PERMUTE_MASK(3, 1, 0, 2))),
			   _mm_mul_ps (_mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(3, 1, 0, 2)), _mm_shuffle_ps (B.m_type, B.m_type, PERMUTE_MASK(3, 0, 2, 1))));
	}

	D_INLINE dVector DotProduct(const dVector& A) const
	{
		dVector tmp(_mm_mul_ps(m_type, A.m_type));
		return tmp.AddHorizontal();
	}

	D_INLINE dVector CrossProduct (const dVector& A, const dVector& B) const
	{
		dFloat32 cofactor[3][3];
		dFloat32 array[4][4];

		const dVector& me = *this;
		for (dInt32 i = 0; i < 4; i ++) 
		{
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = dFloat32 (1.0f);
		}

		dVector normal;
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
			dFloat32  x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			dFloat32  y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			dFloat32  z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			dFloat32  det = x + y + z;

			normal[i] = sign * det;
			sign *= dFloat32 (-1.0f);
		}

		return normal;
	}

	D_INLINE dVector Reciproc () const
	{
		return _mm_div_ps (m_one.m_type, m_type);
	}

	D_INLINE dVector MulAdd(const dVector& A, const dVector& B) const
	{
		return _mm_add_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
	}

	D_INLINE dVector MulSub(const dVector& A, const dVector& B) const
	{
		return _mm_sub_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
	}

	D_INLINE dVector AddHorizontal () const
	{
		__m128 tmp (_mm_hadd_ps (m_type, m_type));
		return _mm_hadd_ps (tmp, tmp);
	}

	D_INLINE dVector Abs () const
	{
		return _mm_and_ps (m_type, m_signMask.m_type);
	}

	dFloat32 GetMax () const
	{
		__m128 tmp (_mm_max_ps (m_type, _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(3, 2, 3, 2))));
		//return dVector (_mm_max_ps (tmp, _mm_shuffle_ps (tmp, tmp, PERMUTE_MASK(3, 2, 0, 1)))).GetScalar();
		return _mm_cvtss_f32(_mm_max_ss (tmp, _mm_shuffle_ps(tmp, tmp, PERMUTE_MASK(3, 2, 0, 1))));
	}

	dVector GetMax (const dVector& data) const
	{
		return _mm_max_ps (m_type, data.m_type);
	}

	dVector GetMin (const dVector& data) const
	{
		return _mm_min_ps (m_type, data.m_type);
	}

	D_INLINE dVector GetInt () const
	{
		return dVector(_mm_cvtps_epi32(Floor().m_type));
	}

	D_INLINE dVector TestZero() const
	{
		//return dVector (_mm_cmpeq_epi32 (m_typeInt, m_zero.m_typeInt)) & m_negOne;
		return m_negOne & (*this == m_zero);
	}

	D_INLINE dVector Floor () const
	{
		dVector truncated (_mm_cvtepi32_ps (_mm_cvttps_epi32 (m_type)));
		dVector ret (truncated - (dVector::m_one & (*this < truncated)));
		dAssert (ret.m_f[0] == dFloor(m_f[0]));
		dAssert (ret.m_f[1] == dFloor(m_f[1]));
		dAssert (ret.m_f[2] == dFloor(m_f[2]));
		dAssert (ret.m_f[3] == dFloor(m_f[3]));
		return ret;
	}

	D_INLINE dVector Sqrt () const
	{
		return _mm_sqrt_ps(m_type);
	}

	D_INLINE dVector InvSqrt () const
	{
		dVector tmp0 (_mm_rsqrt_ps(m_type));
		return m_half * tmp0 * (m_three - *this * tmp0 * tmp0);
	}

	D_INLINE dVector InvMagSqrt () const
	{
		return DotProduct(*this).InvSqrt();
	}

	D_INLINE dVector Normalize () const
	{
		dAssert (m_w == dFloat32 (0.0f));
		//return *this * InvMagSqrt ();
		return Scale(dFloat32(1.0f) / dSqrt(DotProduct(*this).GetScalar()));
	}

	// relational operators
	D_INLINE dVector operator> (const dVector& data) const
	{
		return _mm_cmpgt_ps (m_type, data.m_type);	
	}

	D_INLINE dVector operator== (const dVector& data) const
	{
		return _mm_cmpeq_ps (m_type, data.m_type);	
	}

	D_INLINE dVector operator< (const dVector& data) const
	{
		return _mm_cmplt_ps (m_type, data.m_type);	
	}

	D_INLINE dVector operator>= (const dVector& data) const
	{
		return _mm_cmpge_ps (m_type, data.m_type);	
	}

	D_INLINE dVector operator<= (const dVector& data) const
	{
		return _mm_cmple_ps (m_type, data.m_type);	
	}

	// logical operations
	D_INLINE dVector operator& (const dVector& data) const
	{
		return _mm_and_ps (m_type, data.m_type);	
	}

	D_INLINE dVector operator| (const dVector& data) const
	{
		return _mm_or_ps (m_type, data.m_type);	
	}

	D_INLINE dVector operator^ (const dVector& data) const
	{
		return _mm_xor_ps (m_type, data.m_type);	
	}

	D_INLINE dVector AndNot(const dVector& data) const
	{
		return _mm_andnot_ps(data.m_type, m_type);
	}

	D_INLINE dVector Select(const dVector& data, const dVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		//return  _mm_or_ps (_mm_and_ps (mask.m_type, data.m_type), _mm_andnot_ps(mask.m_type, m_type));
		return  _mm_xor_ps(m_type, _mm_and_ps (mask.m_type, _mm_xor_ps(m_type, data.m_type)));
	}

	D_INLINE dInt32 GetSignMask() const
	{
		return _mm_movemask_ps(m_type);
	} 

	D_INLINE dVector ShiftRight() const
	{
		return _mm_shuffle_ps(m_type, m_type, PERMUTE_MASK(2, 1, 0, 3));
	}

	D_INLINE dVector ShiftTripleRight () const
	{
		return _mm_shuffle_ps(m_type, m_type, PERMUTE_MASK(3, 1, 0, 2));
	}

	D_INLINE dVector ShiftTripleLeft () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(3, 0, 2, 1));
	}

	D_INLINE dVector ShiftRightLogical (dInt32 bits) const
	{
		return dVector (_mm_srli_epi32(m_typeInt, bits)); 
	}

	D_INLINE static void Transpose4x4 (dVector& dst0, dVector& dst1, dVector& dst2, dVector& dst3, const dVector& src0, const dVector& src1, const dVector& src2, const dVector& src3)
	{
		__m128 tmp0 (_mm_unpacklo_ps (src0.m_type, src1.m_type));
		__m128 tmp1 (_mm_unpacklo_ps (src2.m_type, src3.m_type));
		__m128 tmp2 (_mm_unpackhi_ps (src0.m_type, src1.m_type));
		__m128 tmp3 (_mm_unpackhi_ps (src2.m_type, src3.m_type));

		dst0 = dVector (_mm_movelh_ps (tmp0, tmp1));
		dst1 = dVector (_mm_movehl_ps (tmp1, tmp0));
		dst2 = dVector (_mm_movelh_ps (tmp2, tmp3));
		dst3 = dVector (_mm_movehl_ps (tmp3, tmp2));
	}

#ifdef _DEBUG
	D_INLINE void Trace(char* const name) const
	{
		dAssert(0);
		//dTrace(("%s %f %f %f %f\n", name, m_x, m_y, m_z, m_w));
	}
#else 
	D_INLINE void Trace(char* const name) const {}
#endif

	union 
	{
		dFloat32 m_f[4];
		dInt32 m_i[4];
		__m128 m_type;
		__m128i m_typeInt;
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

	D_CORE_API static dVector m_zero;
	D_CORE_API static dVector m_one;
	D_CORE_API static dVector m_wOne;
	D_CORE_API static dVector m_two;
	D_CORE_API static dVector m_half;
	D_CORE_API static dVector m_three;
	D_CORE_API static dVector m_negOne;
	D_CORE_API static dVector m_xMask;
	D_CORE_API static dVector m_yMask;
	D_CORE_API static dVector m_zMask;
	D_CORE_API static dVector m_wMask;
	D_CORE_API static dVector m_epsilon;
	D_CORE_API static dVector m_signMask;
	D_CORE_API static dVector m_triplexMask;
} D_GCC_NEWTON_ALIGN_16 ;
#endif


// *****************************************************************************************
//
// 4 x 1 double precision SSE2 vector class declaration
//
// *****************************************************************************************
#ifdef D_USE_VECTOR_AVX
	D_MSV_NEWTON_ALIGN_32
	class dBigVector
	{
		public:
		D_INLINE dBigVector()
		{
		}

		D_INLINE dBigVector(const dBigVector& copy)
			:m_type(copy.m_type)
		{
		}

		D_INLINE dBigVector(const __m256d type)
			:m_type(type)
		{
		}

		D_INLINE dBigVector(const __m256i type)
			: m_typeInt(type)
		{
		}

		//D_INLINE dBigVector(const __m128 typeLow, const __m128 typeHigh)
		//	: m_typeGen(_mm256_setr_m128(typeLow, typeHigh))
		//{
		//}

		D_INLINE dBigVector(const dFloat64 a)
			:m_type(_mm256_set1_pd(a))
		{
		}

#ifdef D_NEWTON_USE_DOUBLE
		D_INLINE dBigVector(const dFloat32* const ptr)
			:m_type(_mm256_set_pd(ptr[3], ptr[2], ptr[1], ptr[0]))
		{
		}
#else

		D_INLINE dBigVector(const dVector& v)
			:m_type(_mm256_cvtps_pd(v.m_type))
		{
			dAssert(dCheckVector((*this)));
		}

		D_INLINE dBigVector(const dFloat64* const ptr)
			:m_type(_mm256_loadu_pd(ptr))
		{
		}
#endif

		D_INLINE dBigVector(dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w)
			:m_type(_mm256_set_pd(w, z, y, x))
		{
		}

		D_INLINE dBigVector(dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
			:m_ix(dInt64(ix)), m_iy(dInt64(iy)), m_iz(dInt64(iz)), m_iw(dInt64(iw))
		{
		}

		D_INLINE dBigVector(dInt64 ix, dInt64 iy, dInt64 iz, dInt64 iw)
			:m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
		{
		}

		D_INLINE dFloat64& operator[] (dInt32 i)
		{
			dAssert(i < 4);
			dAssert(i >= 0);
			return m_f[i];
		}

		D_INLINE const dFloat64& operator[] (dInt32 i) const
		{
			dAssert(i < 4);
			dAssert(i >= 0);
			return m_f[i];
		}

		D_INLINE dFloat64 GetScalar() const
		{
			//return _mm256_cvtsd_f64(m_type);
			return m_x;
		}

		D_INLINE dBigVector operator+ (const dBigVector& A) const
		{
			return _mm256_add_pd(m_type, A.m_type);
		}

		D_INLINE dBigVector operator- (const dBigVector& A) const
		{
			return _mm256_sub_pd(m_type, A.m_type);
		}

		D_INLINE dBigVector operator* (const dBigVector& A) const
		{
			return _mm256_mul_pd(m_type, A.m_type);
		}

		D_INLINE dBigVector& operator+= (const dBigVector& A)
		{
			m_type = _mm256_add_pd(m_type, A.m_type);
			return *this;
		}

		D_INLINE dBigVector& operator-= (const dBigVector& A)
		{
			m_type = _mm256_sub_pd(m_type, A.m_type);
			return *this;
		}

		D_INLINE dBigVector& operator*= (const dBigVector& A)
		{
			m_type = _mm256_mul_pd(m_type, A.m_type);
			return *this;
		}

		D_INLINE dBigVector MulAdd(const dBigVector& A, const dBigVector& B) const
		{
			return *this + A * B;
		}

		D_INLINE dBigVector MulSub(const dBigVector& A, const dBigVector& B) const
		{
			return *this - A * B;
		}

		// return cross product
		D_INLINE dBigVector CrossProduct(const dBigVector& B) const
		{
			return dBigVector(m_y * B.m_z - m_z * B.m_y, m_z * B.m_x - m_x * B.m_z, m_x * B.m_y - m_y * B.m_x, m_w);
		}

		D_INLINE dBigVector AddHorizontal() const
		{
			__m256d tmp0(_mm256_hadd_pd(m_type, m_type));
			__m256d tmp1(_mm256_permute2f128_pd(tmp0, tmp0, 3));
			return _mm256_add_pd(tmp0, tmp1);
		}

		D_INLINE dBigVector BroadcastX() const
		{
			return dBigVector(m_x);
		}

		D_INLINE dBigVector BroadcastY() const
		{
			return dBigVector(m_y);
		}

		D_INLINE dBigVector BroadcastZ() const
		{
			return dBigVector(m_z);
		}

		D_INLINE dBigVector BroadcastW() const
		{
			return dBigVector(m_w);
		}

		D_INLINE dBigVector Scale(dFloat64 s) const
		{
			__m256d tmp0(_mm256_set1_pd(s));
			return _mm256_mul_pd(m_type, tmp0);
		}

		D_INLINE dBigVector Abs() const
		{
			return _mm256_and_pd(m_type, m_signMask.m_type);
		}

		D_INLINE dBigVector Reciproc() const
		{
			return _mm256_div_pd(m_one.m_type, m_type);
		}

		D_INLINE dBigVector Sqrt() const
		{
			return _mm256_sqrt_pd(m_type);
		}

		D_INLINE dBigVector InvSqrt() const
		{
			return Sqrt().Reciproc();
		}

		D_INLINE dBigVector Normalize() const
		{
			dAssert(m_w == dFloat32(0.0f));
			dFloat64 mag2 = DotProduct(*this).GetScalar();
			return Scale(dFloat64(1.0f) / sqrt(mag2));
		}

		dFloat64 GetMax() const
		{
			__m256d tmp0(_mm256_permute2f128_pd(m_type, m_type, 5));
			__m256d tmp1(_mm256_max_pd(m_type, tmp0));
			__m256d tmp2(_mm256_unpackhi_pd(tmp1, tmp1));
			__m256d tmp3(_mm256_max_pd(tmp1, tmp2));
			dBigVector tmp4(tmp3);
			return tmp4.GetScalar();
		}

		dBigVector GetMax(const dBigVector& data) const
		{
			return _mm256_max_pd(m_type, data.m_type);
		}

		dBigVector GetMin(const dBigVector& data) const
		{
			return _mm256_min_pd(m_type, data.m_type);
		}

		D_INLINE dBigVector GetInt() const
		{
			dBigVector temp(Floor());
			union
			{
				__m128i m;
				struct
				{
					dInt32 m_x;
					dInt32 m_y;
					dInt32 m_z;
					dInt32 m_w;
				};
			} tmp;
			tmp.m = _mm256_cvttpd_epi32(temp.m_type);
			return dBigVector(m_x, m_y, m_z, m_w);
		}

		// relational operators
		D_INLINE dBigVector operator> (const dBigVector& data) const
		{
			return _mm256_cmp_pd(m_type, data.m_type, _CMP_GT_OQ);
		}

		D_INLINE dBigVector operator== (const dBigVector& data) const
		{
			return _mm256_cmp_pd(m_type, data.m_type, _CMP_EQ_OQ);
		}

		D_INLINE dBigVector operator< (const dBigVector& data) const
		{
			return _mm256_cmp_pd(m_type, data.m_type, _CMP_LT_OQ);
		}

		D_INLINE dBigVector operator>= (const dBigVector& data) const
		{
			return _mm256_cmp_pd(m_type, data.m_type, _CMP_GE_OQ);
		}

		D_INLINE dBigVector operator<= (const dBigVector& data) const
		{
			return _mm256_cmp_pd(m_type, data.m_type, _CMP_LE_OQ);
		}

		// logical operations
		D_INLINE dBigVector operator& (const dBigVector& data) const
		{
			return _mm256_and_pd(m_type, data.m_type);
		}

		D_INLINE dBigVector operator| (const dBigVector& data) const
		{
			return _mm256_or_pd(m_type, data.m_type);
		}

		D_INLINE dBigVector operator^ (const dBigVector& data) const
		{
			return _mm256_xor_pd(m_type, data.m_type);
		}

		D_INLINE dBigVector AndNot(const dBigVector& data) const
		{
			return _mm256_andnot_pd(data.m_type, m_type);
		}

		D_INLINE dBigVector Select(const dBigVector& data, const dBigVector& mask) const
		{
			// (((b ^ a) & mask)^a)
			return  _mm256_xor_pd(m_type, _mm256_and_pd(mask.m_type, _mm256_xor_pd(m_type, data.m_type)));
		}

		D_INLINE dBigVector ShiftRight() const
		{
			//return dBigVector(m_w, m_x, m_y, m_z);
			__m256d tmp0(_mm256_permute2f128_pd(m_type, m_type, 5));
			__m256d tmp1(_mm256_blend_pd(m_type, tmp0, 10));
			__m256d tmp2(_mm256_shuffle_pd(tmp1, tmp1, 5));
			return tmp2;
		}

		D_INLINE dBigVector ShiftTripleRight() const
		{
			//return dBigVector(m_z, m_x, m_y, m_w);
			__m256d tmp0(_mm256_permute2f128_pd(m_type, m_type, 5));
			__m256d tmp1(_mm256_shuffle_pd(m_type, m_type, 5));
			__m256d tmp2(_mm256_blend_pd(tmp0, tmp1, 6));
			__m256d tmp3(_mm256_shuffle_pd(tmp2, tmp2, 6));
			return tmp3;
		}

		D_INLINE dBigVector ShiftTripleLeft() const
		{
			//return dBigVector(m_y, m_z, m_x, m_w);
			__m256d tmp0(_mm256_permute2f128_pd(m_type, m_type, 5));
			__m256d tmp1(_mm256_blend_pd(m_type, tmp0, 10));
			__m256d tmp2(_mm256_permute2f128_pd(tmp1, tmp1, 5));
			__m256d tmp3(_mm256_shuffle_pd(tmp2, tmp2, 9));
			return tmp3;
		}

		D_INLINE dBigVector ShiftRightLogical(dInt32 bits) const
		{
			dUnsigned64 x = ((dUnsigned64)m_ix) >> bits;
			dUnsigned64 y = ((dUnsigned64)m_iy) >> bits;
			dUnsigned64 z = ((dUnsigned64)m_iz) >> bits;
			dUnsigned64 w = ((dUnsigned64)m_iw) >> bits;
			return dBigVector((dInt64)x, (dInt64)y, (dInt64)z, (dInt64)w);
		}

		D_INLINE dInt32 GetSignMask() const
		{
			return _mm256_movemask_pd(m_type);
		}

		D_INLINE dBigVector Floor() const
		{
			dBigVector ret(_mm256_floor_pd(m_type));
			dAssert(ret.m_f[0] == floor(m_f[0]));
			dAssert(ret.m_f[1] == floor(m_f[1]));
			dAssert(ret.m_f[2] == floor(m_f[2]));
			dAssert(ret.m_f[3] == floor(m_f[3]));
			return ret;
		}

		D_INLINE dBigVector TestZero() const
		{
			return m_negOne & (*this == dBigVector::m_zero);
		}

		D_INLINE static void Transpose4x4(
			dBigVector& dst0, dBigVector& dst1, dBigVector& dst2, dBigVector& dst3,
			const dBigVector& src0, const dBigVector& src1, 
			const dBigVector& src2, const dBigVector& src3)
		{
			__m256d tmp0(_mm256_unpacklo_pd(src0.m_type, src1.m_type));
			__m256d tmp1(_mm256_unpackhi_pd(src0.m_type, src1.m_type));
			dst2 = _mm256_unpacklo_pd(src2.m_type, src3.m_type);
			dst3 = _mm256_unpackhi_pd(src2.m_type, src3.m_type);
			
			dst0 = _mm256_permute2f128_pd(dst2.m_type, tmp0, 2);
			dst1 = _mm256_permute2f128_pd(dst3.m_type, tmp1, 2);
			tmp0 = _mm256_permute2f128_pd(tmp0, tmp0, 1);
			tmp1 = _mm256_permute2f128_pd(tmp1, tmp1, 1);
			dst2 = _mm256_blend_pd(tmp0, dst2.m_type, 12);
			dst3 = _mm256_blend_pd(tmp1, dst3.m_type, 12);
		}

		// return dot 4d dot product
		D_INLINE dBigVector DotProduct(const dBigVector &A) const
		{
			dBigVector tmp(_mm256_mul_pd(m_type, A.m_type));
			return tmp.AddHorizontal();
		}

		D_INLINE dBigVector CrossProduct(const dBigVector& A, const dBigVector& B) const
		{
			dFloat64 array[4][4];
			dFloat64 cofactor[3][3];

			const dBigVector& me = *this;
			for (dInt32 i = 0; i < 4; i++) 
			{
				array[0][i] = me[i];
				array[1][i] = A[i];
				array[2][i] = B[i];
				array[3][i] = dFloat64(1.0f);
			}

			dBigVector normal;
			dFloat64 sign = dFloat64(-1.0f);
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

		union
		{
			struct
			{
				dFloat64 m_x;
				dFloat64 m_y;
				dFloat64 m_z;
				dFloat64 m_w;
			};
			dFloat64 m_f[4];
			dInt64 m_i[4];
			__m256d m_type;
			__m256i m_typeInt;
			__m256 m_typeGen;
			struct
			{
				dInt64 m_ix;
				dInt64 m_iy;
				dInt64 m_iz;
				dInt64 m_iw;
			};
		};

		D_CORE_API static dBigVector m_zero;
		D_CORE_API static dBigVector m_one;
		D_CORE_API static dBigVector m_wOne;
		D_CORE_API static dBigVector m_two;
		D_CORE_API static dBigVector m_half;
		D_CORE_API static dBigVector m_three;
		D_CORE_API static dBigVector m_negOne;
		D_CORE_API static dBigVector m_xMask;
		D_CORE_API static dBigVector m_yMask;
		D_CORE_API static dBigVector m_zMask;
		D_CORE_API static dBigVector m_wMask;
		D_CORE_API static dBigVector m_epsilon;
		D_CORE_API static dBigVector m_signMask;
		D_CORE_API static dBigVector m_triplexMask;
	} D_GCC_NEWTON_ALIGN_32;

	D_MSV_NEWTON_ALIGN_32
	class dSpatialVector
	{
		public:
		D_INLINE dSpatialVector()
		{
		}

		D_INLINE dSpatialVector(const dFloat32 a)
			:m_d0(_mm256_set1_pd(dFloat64(a)))
			,m_d1(m_d0)
		{
		}

#ifdef D_NEWTON_USE_DOUBLE
		//#define PURMUT_MASK2(y, x)		_MM_SHUFFLE2(x, y)
		D_INLINE dSpatialVector(const dVector& low, const dVector& high)
			//:m_d0(low.m_typeLow)
			//,m_d1(_mm_shuffle_pd(low.m_typeHigh, high.m_typeLow, PURMUT_MASK2(0, 0)))
			//,m_d2(_mm_shuffle_pd(high.m_typeLow, high.m_typeHigh, PURMUT_MASK2(1, 0)))
		{
			dAssert(0);
		}
#else 
		D_INLINE dSpatialVector(const dVector& low, const dVector& high)
			:m_d0(_mm256_cvtps_pd(low.m_type))
			,m_d1(_mm256_cvtps_pd(high.ShiftTripleLeft().m_type))
		{
			m_f[3] = m_f[6];
			m_f[6] = dFloat64(0.0f);
			m_f[7] = dFloat64(0.0f);
		}
#endif

		D_INLINE dSpatialVector(const dSpatialVector& copy)
			:m_d0(copy.m_d0)
			,m_d1(copy.m_d1)
		{
		}

		D_INLINE dSpatialVector(const __m256d d0, const __m256d d1)
			:m_d0(d0)
			,m_d1(d1)
		{
		}

		D_INLINE dFloat64& operator[] (dInt32 i)
		{
			dAssert(i < 6);
			dAssert(i >= 0);
			return m_f[i];
		}

		D_INLINE const dFloat64& operator[] (dInt32 i) const
		{
			dAssert(i < 6);
			dAssert(i >= 0);
			return m_f[i];
		}

		D_INLINE dSpatialVector operator+ (const dSpatialVector& A) const
		{
			return dSpatialVector(_mm256_add_pd(m_d0, A.m_d0), _mm256_add_pd(m_d1, A.m_d1));
		}

		D_INLINE dSpatialVector operator*(const dSpatialVector& A) const
		{
			return dSpatialVector(_mm256_mul_pd(m_d0, A.m_d0), _mm256_mul_pd(m_d1, A.m_d1));
		}

		D_INLINE dFloat64 DotProduct(const dSpatialVector& v) const
		{
			dSpatialVector tmp(*this * v);
			__m256d tmp0(_mm256_add_pd(tmp.m_d0, tmp.m_d1));
			__m256d tmp1(_mm256_hadd_pd(tmp0, tmp0));
			__m256d tmp2(_mm256_permute2f128_pd(tmp1, tmp1, 1));
			__m256d tmp3(_mm256_add_pd(tmp1, tmp2));
			return *((dFloat64*)&tmp3);
		}

		D_INLINE dSpatialVector Scale(dFloat64 s) const
		{
			__m256d tmp(_mm256_set1_pd(s));
			return dSpatialVector(_mm256_mul_pd(m_d0, tmp), _mm256_mul_pd(m_d1, tmp));
		}

		union
		{
			dFloat64 m_f[8];
			struct
			{
				__m256d m_d0;
				__m256d m_d1;
			};
		};
		static dSpatialVector m_zero;
	} D_GCC_NEWTON_ALIGN_32;

#else
	D_MSV_NEWTON_ALIGN_32
	class dBigVector
	{
		#define PERMUT_MASK_DOUBLE(y, x)	_MM_SHUFFLE2 (y, x)

		public:
		D_INLINE dBigVector()
		{
		}

		D_INLINE dBigVector(const dBigVector& copy)
			:m_typeLow(copy.m_typeLow)
			,m_typeHigh(copy.m_typeHigh)
		{
		}

		D_INLINE dBigVector(const __m128d typeLow, const __m128d typeHigh)
			:m_typeLow(typeLow)
			,m_typeHigh(typeHigh)
		{
		}

		D_INLINE dBigVector(const __m128i typeLow, const __m128i typeHigh)
			:m_typeIntLow(typeLow)
			,m_typeIntHigh(typeHigh)
		{
		}

		D_INLINE dBigVector(const dFloat64 a)
			:m_typeLow(_mm_set1_pd(a))
			,m_typeHigh(_mm_set1_pd(a))
		{
		}

	#ifdef D_NEWTON_USE_DOUBLE
		D_INLINE dBigVector (const dFloat32* const ptr)
			:m_typeLow(_mm_loadu_pd(ptr))
			,m_typeHigh(_mm_loadu_pd(&ptr[2]))
		{
		}
	#else

		D_INLINE dBigVector(const dVector& v)
			:m_typeLow(_mm_cvtps_pd (v.m_type))
			,m_typeHigh(_mm_cvtps_pd (_mm_shuffle_ps (v.m_type, v.m_type, PERMUTE_MASK(3, 2, 3, 2))))
		{
			dAssert(dCheckVector((*this)));
		}

		D_INLINE dBigVector(const dFloat64* const ptr)
			:m_typeLow(_mm_loadu_pd(ptr))
			,m_typeHigh(_mm_loadu_pd(&ptr[2]))
		{
		}
	#endif

		D_INLINE dBigVector(dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w)
			:m_typeLow(_mm_set_pd(y, x))
			,m_typeHigh(_mm_set_pd(w, z))
		{
		}

		D_INLINE dBigVector(dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
			:m_ix(dInt64(ix)), m_iy(dInt64(iy)), m_iz(dInt64(iz)), m_iw(dInt64(iw))
		{
		}

		D_INLINE dBigVector(dInt64 ix, dInt64 iy, dInt64 iz, dInt64 iw)
			:m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
		{
		}

		D_INLINE dFloat64& operator[] (dInt32 i)
		{
			dAssert(i < 4);
			dAssert(i >= 0);
			return m_f[i];
		}

		D_INLINE const dFloat64& operator[] (dInt32 i) const
		{
			dAssert(i < 4);
			dAssert(i >= 0);
			return m_f[i];
		}

		D_INLINE dFloat64 GetScalar() const
		{
			//return m_x;
			return _mm_cvtsd_f64(m_typeLow);
		}

		D_INLINE dBigVector operator+ (const dBigVector& A) const
		{
			return dBigVector(_mm_add_pd(m_typeLow, A.m_typeLow), _mm_add_pd(m_typeHigh, A.m_typeHigh));
		}

		D_INLINE dBigVector operator- (const dBigVector& A) const
		{
			return dBigVector(_mm_sub_pd(m_typeLow, A.m_typeLow), _mm_sub_pd(m_typeHigh, A.m_typeHigh));
		}

		D_INLINE dBigVector operator* (const dBigVector& A) const
		{
			return dBigVector(_mm_mul_pd(m_typeLow, A.m_typeLow), _mm_mul_pd(m_typeHigh, A.m_typeHigh));
		}

		D_INLINE dBigVector& operator+= (const dBigVector& A)
		{
			m_typeLow = _mm_add_pd(m_typeLow, A.m_typeLow);
			m_typeHigh = _mm_add_pd(m_typeHigh, A.m_typeHigh);
			return *this;
		}

		D_INLINE dBigVector& operator-= (const dBigVector& A)
		{
			m_typeLow = _mm_sub_pd(m_typeLow, A.m_typeLow);
			m_typeHigh = _mm_sub_pd(m_typeHigh, A.m_typeHigh);
			return *this;
		}

		D_INLINE dBigVector& operator*= (const dBigVector& A)
		{
			m_typeLow = _mm_mul_pd(m_typeLow, A.m_typeLow);
			m_typeHigh = _mm_mul_pd(m_typeHigh, A.m_typeHigh);
			return *this;
		}

		D_INLINE dBigVector MulAdd(const dBigVector& A, const dBigVector& B) const
		{
			return *this + A * B;
		}

		D_INLINE dBigVector MulSub(const dBigVector& A, const dBigVector& B) const
		{
			return *this - A * B;
		}

		// return cross product
		D_INLINE dBigVector CrossProduct(const dBigVector& B) const
		{
			return dBigVector(m_y * B.m_z - m_z * B.m_y, m_z * B.m_x - m_x * B.m_z, m_x * B.m_y - m_y * B.m_x, m_w);
		}

		D_INLINE dBigVector AddHorizontal() const
		{
			__m128d tmp0(_mm_add_pd(m_typeHigh, m_typeLow));
			__m128d tmp1(_mm_hadd_pd(tmp0, tmp0));
			return dBigVector(tmp1, tmp1);
		}

		D_INLINE dBigVector BroadcastX() const
		{
			return dBigVector(m_x);
		}

		D_INLINE dBigVector BroadcastY() const
		{
			return dBigVector(m_y);
		}

		D_INLINE dBigVector BroadcastZ() const
		{
			return dBigVector(m_z);
		}

		D_INLINE dBigVector BroadcastW() const
		{
			return dBigVector(m_w);
		}

		D_INLINE dBigVector Scale(dFloat64 s) const
		{
			__m128d tmp0(_mm_set1_pd(s));
			return dBigVector(_mm_mul_pd(m_typeLow, tmp0), _mm_mul_pd(m_typeHigh, tmp0));
		}

		D_INLINE dBigVector Abs() const
		{
			return dBigVector(_mm_and_pd(m_typeLow, m_signMask.m_typeLow), _mm_and_pd(m_typeHigh, m_signMask.m_typeLow));
		}

		D_INLINE dBigVector Reciproc() const
		{
			return dBigVector(_mm_div_pd(m_one.m_typeLow, m_typeLow), _mm_div_pd(m_one.m_typeHigh, m_typeHigh));
		}

		D_INLINE dBigVector Sqrt() const
		{
			return dBigVector(_mm_sqrt_pd(m_typeLow), _mm_sqrt_pd(m_typeHigh));
		}

		D_INLINE dBigVector InvSqrt() const
		{
			return Sqrt().Reciproc();
		}

		D_INLINE dBigVector Normalize() const
		{
			dAssert (m_w == dFloat32 (0.0f));
			dFloat64 mag2 = DotProduct(*this).GetScalar();
			return Scale(dFloat64 (1.0f) / sqrt (mag2));
		}

		dFloat64 GetMax() const
		{
			__m128d tmp(_mm_max_pd(m_typeLow, m_typeHigh));
			return dBigVector(_mm_max_pd(tmp, _mm_shuffle_pd(tmp, tmp, PERMUT_MASK_DOUBLE(0, 1))), tmp).GetScalar();
		}

		dBigVector GetMax(const dBigVector& data) const
		{
			return dBigVector(_mm_max_pd(m_typeLow, data.m_typeLow), _mm_max_pd(m_typeHigh, data.m_typeHigh));
		}

		dBigVector GetMin(const dBigVector& data) const
		{
			return dBigVector(_mm_min_pd(m_typeLow, data.m_typeLow), _mm_min_pd(m_typeHigh, data.m_typeHigh));
		}

		D_INLINE dBigVector GetInt() const
		{
			dBigVector temp(Floor());
			dInt64 x = _mm_cvtsd_si32(temp.m_typeLow);
			dInt64 y = _mm_cvtsd_si32(_mm_shuffle_pd(temp.m_typeLow, temp.m_typeLow, PERMUT_MASK_DOUBLE(1, 1)));
			dInt64 z = _mm_cvtsd_si32(temp.m_typeHigh);
			dInt64 w = _mm_cvtsd_si32(_mm_shuffle_pd(temp.m_typeHigh, temp.m_typeHigh, PERMUT_MASK_DOUBLE(1, 1)));
			return dBigVector(_mm_set_pd(*(dFloat32*)&y, *(dFloat32*)&x), _mm_set_pd(*(dFloat32*)&w, *(dFloat32*)&z));
		}

		// relational operators
		D_INLINE dBigVector operator> (const dBigVector& data) const
		{
			return dBigVector(_mm_cmpgt_pd(m_typeLow, data.m_typeLow), _mm_cmpgt_pd(m_typeHigh, data.m_typeHigh));
		}

		D_INLINE dBigVector operator== (const dBigVector& data) const
		{
			return dBigVector(_mm_cmpeq_pd(m_typeLow, data.m_typeLow), _mm_cmpeq_pd(m_typeHigh, data.m_typeHigh));
		}

		D_INLINE dBigVector operator< (const dBigVector& data) const
		{
			return dBigVector(_mm_cmplt_pd(m_typeLow, data.m_typeLow), _mm_cmplt_pd(m_typeHigh, data.m_typeHigh));
		}

		D_INLINE dBigVector operator>= (const dBigVector& data) const
		{
			return dBigVector(_mm_cmpge_pd(m_typeLow, data.m_typeLow), _mm_cmpge_pd(m_typeHigh, data.m_typeHigh));
		}

		D_INLINE dBigVector operator<= (const dBigVector& data) const
		{
			return dBigVector(_mm_cmple_pd(m_typeLow, data.m_typeLow), _mm_cmple_pd(m_typeHigh, data.m_typeHigh));
		}

		// logical operations
		D_INLINE dBigVector operator& (const dBigVector& data) const
		{
			return dBigVector(_mm_and_pd(m_typeLow, data.m_typeLow), _mm_and_pd(m_typeHigh, data.m_typeHigh));
		}

		D_INLINE dBigVector operator| (const dBigVector& data) const
		{
			return dBigVector(_mm_or_pd(m_typeLow, data.m_typeLow), _mm_or_pd(m_typeHigh, data.m_typeHigh));
		}

		D_INLINE dBigVector operator^ (const dBigVector& data) const
		{
			return dBigVector(_mm_xor_pd(m_typeLow, data.m_typeLow), _mm_xor_pd(m_typeHigh, data.m_typeHigh));
		}

		D_INLINE dBigVector AndNot(const dBigVector& data) const
		{
			return dBigVector(_mm_andnot_pd(data.m_typeLow, m_typeLow), _mm_andnot_pd(data.m_typeHigh, m_typeHigh));
		}

		D_INLINE dBigVector Select(const dBigVector& data, const dBigVector& mask) const
		{
			// (((b ^ a) & mask)^a)
			return  dBigVector(_mm_xor_pd(m_typeLow, _mm_and_pd(mask.m_typeLow, _mm_xor_pd(m_typeLow, data.m_typeLow))),
								_mm_xor_pd(m_typeHigh, _mm_and_pd(mask.m_typeHigh, _mm_xor_pd(m_typeHigh, data.m_typeHigh))));
		}

		D_INLINE dBigVector ShiftRight() const
		{
			//return dBigVector (m_w, m_x, m_y, m_z); 
			return dBigVector(_mm_shuffle_pd(m_typeHigh, m_typeLow, PERMUT_MASK_DOUBLE(0, 1)), _mm_shuffle_pd(m_typeLow, m_typeHigh, PERMUT_MASK_DOUBLE(0, 1)));
		}

		D_INLINE dBigVector ShiftTripleRight() const
		{
			return dBigVector(_mm_shuffle_pd(m_typeHigh, m_typeLow, PERMUT_MASK_DOUBLE(0, 0)), _mm_shuffle_pd(m_typeLow, m_typeHigh, PERMUT_MASK_DOUBLE(1, 1)));
		}

		D_INLINE dBigVector ShiftTripleLeft() const
		{
			return dBigVector(_mm_shuffle_pd(m_typeLow, m_typeHigh, PERMUT_MASK_DOUBLE(0, 1)), _mm_shuffle_pd(m_typeLow, m_typeHigh, PERMUT_MASK_DOUBLE(1, 0)));
		}

		D_INLINE dBigVector ShiftRightLogical(dInt32 bits) const
		{
			//return dBigVector(dInt64(dUnsigned64(m_ix) >> bits), dInt64(dUnsigned64(m_iy) >> bits), dInt64(dUnsigned64(m_iz) >> bits), dInt64(dUnsigned64(m_iw) >> bits));
			return dBigVector(_mm_srli_epi64(m_typeIntLow, bits), _mm_srli_epi64(m_typeIntHigh, bits));
		}

		D_INLINE dInt32 GetSignMask() const
		{
			return _mm_movemask_pd(m_typeLow) | (_mm_movemask_pd(m_typeHigh) << 2);
		}

		D_INLINE dBigVector Floor() const
		{
			return dBigVector(floor(m_x), floor(m_y), floor(m_z), floor(m_w));
		}

		D_INLINE dBigVector TestZero() const
		{
			return m_negOne & (*this == m_zero);
		}

		D_INLINE static void Transpose4x4(dBigVector& dst0, dBigVector& dst1, dBigVector& dst2, dBigVector& dst3,
			const dBigVector& src0, const dBigVector& src1, const dBigVector& src2, const dBigVector& src3)
		{
			dBigVector tmp0(src0);
			dBigVector tmp1(src1);
			dBigVector tmp2(src2);
			dBigVector tmp3(src3);

			dst0 = dBigVector(tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
			dst1 = dBigVector(tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
			dst2 = dBigVector(tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
			dst3 = dBigVector(tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
		}

		// return dot 4d dot product
		D_INLINE dBigVector DotProduct(const dBigVector &A) const
		{
			dBigVector tmp(_mm_mul_pd(m_typeLow, A.m_typeLow), _mm_mul_pd(m_typeHigh, A.m_typeHigh));
			return tmp.AddHorizontal();
		}

		D_INLINE dBigVector CrossProduct(const dBigVector& A, const dBigVector& B) const
		{
			dFloat64 cofactor[3][3];
			dFloat64 array[4][4];

			const dBigVector& me = *this;
			for (dInt32 i = 0; i < 4; i++) {
				array[0][i] = me[i];
				array[1][i] = A[i];
				array[2][i] = B[i];
				array[3][i] = dFloat64(1.0f);
			}

			dBigVector normal;
			dFloat64 sign = dFloat64(-1.0f);
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

		union
		{
			dFloat64 m_f[4];
			dInt64 m_i[4];
			struct
			{
				__m128d m_typeLow;
				__m128d m_typeHigh;
			};
			struct
			{
				__m128i m_typeIntLow;
				__m128i m_typeIntHigh;
			};
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

		D_CORE_API static dBigVector m_zero;
		D_CORE_API static dBigVector m_one;
		D_CORE_API static dBigVector m_wOne;
		D_CORE_API static dBigVector m_two;
		D_CORE_API static dBigVector m_half;
		D_CORE_API static dBigVector m_three;
		D_CORE_API static dBigVector m_negOne;
		D_CORE_API static dBigVector m_xMask;
		D_CORE_API static dBigVector m_yMask;
		D_CORE_API static dBigVector m_zMask;
		D_CORE_API static dBigVector m_wMask;
		D_CORE_API static dBigVector m_epsilon;
		D_CORE_API static dBigVector m_signMask;
		D_CORE_API static dBigVector m_triplexMask;
	} D_GCC_NEWTON_ALIGN_32 ;

	D_MSV_NEWTON_ALIGN_32
	class dSpatialVector
	{
		public:
		D_INLINE dSpatialVector()
		{
		}

		D_INLINE dSpatialVector(const dFloat32 a)
			:m_d0(_mm_set1_pd(a))
			,m_d1(_mm_set1_pd(a))
			,m_d2(_mm_set1_pd(a))
		{
		}

	#ifdef D_NEWTON_USE_DOUBLE
		#define PURMUT_MASK2(y, x)		_MM_SHUFFLE2(x, y)
		D_INLINE dSpatialVector(const dVector& low, const dVector& high)
			:m_d0(low.m_typeLow)
			,m_d1(_mm_shuffle_pd(low.m_typeHigh, high.m_typeLow, PURMUT_MASK2(0, 0)))
			,m_d2(_mm_shuffle_pd(high.m_typeLow, high.m_typeHigh, PURMUT_MASK2(1, 0)))
		{
		}
	#else 
		D_INLINE dSpatialVector(const dVector& low, const dVector& high)
			:m_d0(_mm_cvtps_pd(low.m_type))
			,m_d1(_mm_cvtps_pd(_mm_unpackhi_ps(low.m_type, _mm_shuffle_ps(low.m_type, high.m_type, PERMUTE_MASK(0, 0, 0, 2)))))
			,m_d2(_mm_cvtps_pd(_mm_shuffle_ps(high.m_type, high.m_type, PERMUTE_MASK(3, 3, 2, 1))))
		{
		}
	#endif

		D_INLINE dSpatialVector(const dSpatialVector& copy)
			:m_d0(copy.m_d0)
			,m_d1(copy.m_d1)
			,m_d2(copy.m_d2)
		{
		}

		D_INLINE dSpatialVector(const __m128d d0, const __m128d d1, const __m128d d2)
			:m_d0(d0)
			,m_d1(d1)
			,m_d2(d2)
		{
		}

		D_INLINE dFloat64& operator[] (dInt32 i)
		{
			dAssert(i < 6);
			dAssert(i >= 0);
			return ((dFloat64*)&m_d0)[i];
		}

		D_INLINE const dFloat64& operator[] (dInt32 i) const
		{
			dAssert(i < 6);
			dAssert(i >= 0);
			return ((dFloat64*)&m_d0)[i];
		}

		D_INLINE dSpatialVector operator+ (const dSpatialVector& A) const
		{
			return dSpatialVector(_mm_add_pd(m_d0, A.m_d0), _mm_add_pd(m_d1, A.m_d1), _mm_add_pd(m_d2, A.m_d2));
		}

		D_INLINE dSpatialVector operator*(const dSpatialVector& A) const
		{
			return dSpatialVector(_mm_mul_pd(m_d0, A.m_d0), _mm_mul_pd(m_d1, A.m_d1), _mm_mul_pd(m_d2, A.m_d2));
		}

		D_INLINE dFloat64 DotProduct(const dSpatialVector& v) const
		{
			dSpatialVector tmp(*this * v);
			__m128d tmp2(_mm_add_pd(tmp.m_d0, _mm_add_pd(tmp.m_d1, tmp.m_d2)));
			return _mm_cvtsd_f64(_mm_hadd_pd(tmp2, tmp2));
		}

		D_INLINE dSpatialVector Scale(dFloat64 s) const
		{
			__m128d tmp(_mm_set1_pd(s));
			return dSpatialVector(_mm_mul_pd(m_d0, tmp), _mm_mul_pd(m_d1, tmp), _mm_mul_pd(m_d2, tmp));
		}

		union
		{
			dFloat64 m_f[6];
			struct
			{
				__m128d m_d0;
				__m128d m_d1;
				__m128d m_d2;
			};
		};
		D_CORE_API static dSpatialVector m_zero;
	} D_GCC_NEWTON_ALIGN_32 ;
#endif

#endif
#endif
