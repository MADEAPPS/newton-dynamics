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

#ifndef __ND_VECTOR_X86_SIMD_H__
#define __ND_VECTOR_X86_SIMD_H__

#ifndef D_SCALAR_VECTOR_CLASS

#ifdef D_NEWTON_USE_DOUBLE
	#define ndVector ndBigVector
#else

class ndBigVector;
// *****************************************************************************************
//
// 4 x 1 single precision SSE vector class declaration
//
// *****************************************************************************************
D_MSV_NEWTON_ALIGN_16
class ndVector
{
	#define PERMUTE_MASK(w, z, y, x) _MM_SHUFFLE (w, z, y, x)
	public:
	D_OPERATOR_NEW_AND_DELETE

	inline ndVector() 
	{
	}

	inline ndVector(const __m128i type)
		:m_typeInt (type)
	{
	}

	inline ndVector(const __m128 type)
		: m_type(type)
	{
	}

	inline ndVector (const ndFloat32 a)
		:m_type(_mm_set_ps1(a)) 
	{
	}

	inline ndVector (const ndFloat32* const ptr)
		:m_type(_mm_loadu_ps (ptr))
	{
	}

	// emulate gather instruction
	inline ndVector(const ndFloat32* const baseAddr, const ndInt32* const index)
		:m_x(baseAddr[index[0]])
		,m_y(baseAddr[index[1]])
		,m_z(baseAddr[index[2]])
		,m_w(baseAddr[index[3]])
	{
	}

#ifndef	D_NEWTON_USE_DOUBLE
	inline ndVector(const ndFloat64* const ptr)
		:m_type(_mm_set_ps(ndFloat32(ptr[3]), ndFloat32(ptr[2]), ndFloat32(ptr[1]), ndFloat32(ptr[0])))
	{
	}
#endif

	inline ndVector (const ndVector& copy)
		:m_type(copy.m_type)
	{
	}

	inline ndVector (const ndBigVector& copy)
		:m_type(_mm_shuffle_ps (_mm_cvtpd_ps (((__m128d*)&copy)[0]), _mm_cvtpd_ps (((__m128d*)&copy)[1]), PERMUTE_MASK(1, 0, 1, 0)))
	{
		dAssert (dCheckVector ((*this)));
	}

	inline ndVector (ndFloat32 x, ndFloat32 y, ndFloat32 z, ndFloat32 w)
		:m_type(_mm_set_ps(w, z, y, x))
	{
	}

	inline ndVector (ndInt32 ix, ndInt32 iy, ndInt32 iz, ndInt32 iw)
		:m_type(_mm_set_ps(*(ndFloat32*)&iw, *(ndFloat32*)&iz, *(ndFloat32*)&iy, *(ndFloat32*)&ix))
	{
	}

	inline ndFloat32 GetX() const
	{
		return m_x;
	}

	inline ndFloat32 GetY() const
	{
		return m_y;
	}

	inline ndFloat32 GetZ() const
	{
		return m_z;
	}

	inline ndFloat32 GetW() const
	{
		return m_w;
	}

	inline void SetX(ndFloat32 x)
	{
		m_x = x;
	}

	inline void SetY(ndFloat32 x)
	{
		m_y = x;
	}

	inline void SetZ(ndFloat32 x)
	{
		m_z = x;
	}

	inline void SetW(ndFloat32 x)
	{
		m_w = x;
	}

	inline ndFloat32 GetScalar () const
	{
		return _mm_cvtss_f32 (m_type);
	}

	inline void Store (ndFloat32* const dst) const
	{
		_mm_storeu_ps(dst, m_type);
	}

	inline ndVector BroadcastX () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(0, 0, 0, 0));
	}

	inline ndVector BroadcastY () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(1, 1, 1, 1));
	}

	inline ndVector BroadcastZ () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(2, 2, 2, 2));
	}

	inline ndVector BroadcastW () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(3, 3, 3, 3));
	}

	inline ndVector Scale (ndFloat32 s) const
	{
		return _mm_mul_ps (m_type, _mm_set_ps1(s));
	}

	inline ndFloat32& operator[] (ndInt32 i)
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return m_f[i];
	}

	inline const ndFloat32& operator[] (ndInt32 i) const
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return m_f[i];
	}

	inline ndVector operator+ (const ndVector& A) const
	{
		return _mm_add_ps (m_type, A.m_type);	
	}

	inline ndVector operator- (const ndVector& A) const 
	{
		return _mm_sub_ps (m_type, A.m_type);	
	}

	inline ndVector operator* (const ndVector& A) const
	{
		return _mm_mul_ps(m_type, A.m_type);
	}

	inline ndVector& operator+= (const ndVector& A)
	{
		return (*this = _mm_add_ps (m_type, A.m_type));
	}

	inline ndVector& operator-= (const ndVector& A)
	{
		return (*this = _mm_sub_ps (m_type, A.m_type));
	}

	inline ndVector& operator*= (const ndVector& A)
	{
		return (*this = _mm_mul_ps(m_type, A.m_type));
	}

	// return 4d cross product
	inline ndVector DotProduct(const ndVector& A) const
	{
		const ndVector tmp(_mm_mul_ps(m_type, A.m_type));
		return tmp.AddHorizontal();
	}

	// return 3d cross product
	inline ndVector CrossProduct (const ndVector& B) const
	{
		//__m128 xxx = _mm_sub_ps(_mm_mul_ps(_mm_shuffle_ps(m_type, m_type, PERMUTE_MASK(3, 0, 2, 1)), _mm_shuffle_ps(B.m_type, B.m_type, PERMUTE_MASK(3, 1, 0, 2))),
		//	                      _mm_mul_ps(_mm_shuffle_ps(m_type, m_type, PERMUTE_MASK(3, 1, 0, 2)), _mm_shuffle_ps(B.m_type, B.m_type, PERMUTE_MASK(3, 0, 2, 1))));
		__m128 tmp0_a (_mm_shuffle_ps(  m_type,   m_type, PERMUTE_MASK(3, 0, 2, 1)));
		__m128 tmp0_b (_mm_shuffle_ps(B.m_type, B.m_type, PERMUTE_MASK(3, 1, 0, 2)));
		__m128 tmp1_a (_mm_shuffle_ps(  m_type,   m_type, PERMUTE_MASK(3, 1, 0, 2)));
		__m128 tmp1_b (_mm_shuffle_ps(B.m_type, B.m_type, PERMUTE_MASK(3, 0, 2, 1)));
		__m128 tmp0 (_mm_mul_ps(tmp0_a, tmp0_b));
		__m128 tmp1 (_mm_mul_ps(tmp1_a, tmp1_b));
		__m128 tmp (_mm_sub_ps(tmp0, tmp1));
		return tmp;
	}

	// return 4d cross product
	inline ndVector CrossProduct (const ndVector& A, const ndVector& B) const
	{
		ndFloat32 cofactor[3][3];
		ndFloat32 array[4][4];

		const ndVector& me = *this;
		for (ndInt32 i = 0; i < 4; i ++) 
		{
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = ndFloat32 (1.0f);
		}

		ndVector normal;
		ndFloat32  sign = ndFloat32 (-1.0f);
		for (ndInt32 i = 0; i < 4; i ++)  
		{
			for (ndInt32 j = 0; j < 3; j ++) 
			{
				ndInt32 k0 = 0;
				for (ndInt32 k = 0; k < 4; k ++) 
				{
					if (k != i) 
					{
						cofactor[j][k0] = array[j][k];
						k0 ++;
					}
				}
			}
			ndFloat32  x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			ndFloat32  y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			ndFloat32  z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			ndFloat32  det = x + y + z;

			normal[i] = sign * det;
			sign *= ndFloat32 (-1.0f);
		}

		return normal;
	}

	inline ndVector Reciproc () const
	{
		return _mm_div_ps (m_one.m_type, m_type);
	}

	inline ndVector MulAdd(const ndVector& A, const ndVector& B) const
	{
		return _mm_add_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
	}

	inline ndVector MulSub(const ndVector& A, const ndVector& B) const
	{
		return _mm_sub_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
	}

	inline ndVector AddHorizontal () const
	{
		#ifdef D_USE_SSE3
			__m128 tmp (_mm_hadd_ps (m_type, m_type));
			return _mm_hadd_ps (tmp, tmp);
		#else
			__m128 tmp (_mm_add_ps (m_type, _mm_shuffle_ps(m_type, m_type, PERMUTE_MASK(1, 0, 3, 2))));
			return _mm_add_ps(tmp, _mm_shuffle_ps(tmp, tmp, PERMUTE_MASK(2, 3, 0, 1)));
		#endif	
	}

	inline ndVector Abs () const
	{
		return _mm_and_ps (m_type, m_signMask.m_type);
	}

	ndVector GetMax() const
	{
		__m128 tmp(_mm_max_ps(m_type, _mm_shuffle_ps(m_type, m_type, PERMUTE_MASK(1, 0, 3, 2))));
		return _mm_max_ps(tmp, _mm_shuffle_ps(tmp, tmp, PERMUTE_MASK(2, 3, 0, 1)));
	}

	ndVector GetMax (const ndVector& data) const
	{
		return _mm_max_ps (m_type, data.m_type);
	}

	ndVector GetMin (const ndVector& data) const
	{
		return _mm_min_ps (m_type, data.m_type);
	}

	inline ndVector GetInt () const
	{
		return ndVector(_mm_cvtps_epi32(Floor().m_type));
	}

	inline ndVector TestZero() const
	{
		//return ndVector (_mm_cmpeq_epi32 (m_typeInt, m_zero.m_typeInt)) & m_negOne;
		return m_negOne & (*this == m_zero);
	}

	inline ndVector Floor () const
	{
		ndVector truncated (_mm_cvtepi32_ps (_mm_cvttps_epi32 (m_type)));
		ndVector ret (truncated - (ndVector::m_one & (*this < truncated)));
		dAssert (ret.m_f[0] == ndFloor(m_f[0]));
		dAssert (ret.m_f[1] == ndFloor(m_f[1]));
		dAssert (ret.m_f[2] == ndFloor(m_f[2]));
		dAssert (ret.m_f[3] == ndFloor(m_f[3]));
		return ret;
	}

	inline ndVector Sqrt () const
	{
		return _mm_sqrt_ps(m_type);
	}

	inline ndVector InvSqrt () const
	{
		ndVector tmp0 (_mm_rsqrt_ps(m_type));
		return m_half * tmp0 * (m_three - *this * tmp0 * tmp0);
	}

	inline ndVector InvMagSqrt () const
	{
		return DotProduct(*this).InvSqrt();
	}

	inline ndVector Normalize () const
	{
		return Scale(ndFloat32(1.0f) / ndSqrt(DotProduct(*this).GetScalar()));
	}

	// relational operators
	inline ndVector operator> (const ndVector& data) const
	{
		return _mm_cmpgt_ps (m_type, data.m_type);	
	}

	inline ndVector operator== (const ndVector& data) const
	{
		return _mm_cmpeq_ps (m_type, data.m_type);	
	}

	inline ndVector operator< (const ndVector& data) const
	{
		return _mm_cmplt_ps (m_type, data.m_type);	
	}

	inline ndVector operator>= (const ndVector& data) const
	{
		return _mm_cmpge_ps (m_type, data.m_type);	
	}

	inline ndVector operator<= (const ndVector& data) const
	{
		return _mm_cmple_ps (m_type, data.m_type);	
	}

	// logical operations
	inline ndVector operator& (const ndVector& data) const
	{
		return _mm_and_ps (m_type, data.m_type);	
	}

	inline ndVector operator| (const ndVector& data) const
	{
		return _mm_or_ps (m_type, data.m_type);	
	}

	inline ndVector operator^ (const ndVector& data) const
	{
		return _mm_xor_ps (m_type, data.m_type);	
	}

	inline ndVector AndNot(const ndVector& data) const
	{
		return _mm_andnot_ps(data.m_type, m_type);
	}

	inline ndVector Select(const ndVector& data, const ndVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		//return  _mm_or_ps (_mm_and_ps (mask.m_type, data.m_type), _mm_andnot_ps(mask.m_type, m_type));
		return  _mm_xor_ps(m_type, _mm_and_ps (mask.m_type, _mm_xor_ps(m_type, data.m_type)));
	}

	inline ndInt32 GetSignMask() const
	{
		return _mm_movemask_ps(m_type);
	} 

	inline ndVector ShiftRight() const
	{
		return _mm_shuffle_ps(m_type, m_type, PERMUTE_MASK(2, 1, 0, 3));
	}

	inline ndVector ShiftTripleRight () const
	{
		return _mm_shuffle_ps(m_type, m_type, PERMUTE_MASK(3, 1, 0, 2));
	}

	inline ndVector ShiftTripleLeft () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(3, 0, 2, 1));
	}

	inline ndVector ShiftRightLogical (ndInt32 bits) const
	{
		return ndVector (_mm_srli_epi32(m_typeInt, bits)); 
	}

	inline static void Transpose4x4 (ndVector& dst0, ndVector& dst1, ndVector& dst2, ndVector& dst3, const ndVector& src0, const ndVector& src1, const ndVector& src2, const ndVector& src3)
	{
		__m128 tmp0 (_mm_unpacklo_ps (src0.m_type, src1.m_type));
		__m128 tmp1 (_mm_unpacklo_ps (src2.m_type, src3.m_type));
		__m128 tmp2 (_mm_unpackhi_ps (src0.m_type, src1.m_type));
		__m128 tmp3 (_mm_unpackhi_ps (src2.m_type, src3.m_type));

		dst0 = ndVector (_mm_movelh_ps (tmp0, tmp1));
		dst1 = ndVector (_mm_movehl_ps (tmp1, tmp0));
		dst2 = ndVector (_mm_movelh_ps (tmp2, tmp3));
		dst3 = ndVector (_mm_movehl_ps (tmp3, tmp2));
	}

#ifdef _DEBUG
	//inline void Trace(char* const name) const
	inline void Trace(char* const) const
	{
		dAssert(0);
		//dTrace(("%s %f %f %f %f\n", name, m_x, m_y, m_z, m_w));
	}
#else 
	inline void Trace(char* const) const {}
#endif

	union 
	{
		ndFloat32 m_f[4];
		ndInt32 m_i[4];
		__m128 m_type;
		__m128i m_typeInt;
		struct 
		{
			ndFloat32 m_x;
			ndFloat32 m_y;
			ndFloat32 m_z;
			ndFloat32 m_w;
		};
		struct 
		{
			ndInt32 m_ix;
			ndInt32 m_iy;
			ndInt32 m_iz;
			ndInt32 m_iw;
		};
	};

	D_CORE_API static ndVector m_zero;
	D_CORE_API static ndVector m_one;
	D_CORE_API static ndVector m_wOne;
	D_CORE_API static ndVector m_two;
	D_CORE_API static ndVector m_half;
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
} D_GCC_NEWTON_ALIGN_16 ;
#endif


// *****************************************************************************************
//
// 4 x 1 double precision SSE2 vector class declaration
//
// *****************************************************************************************
D_MSV_NEWTON_ALIGN_32
class ndBigVector
{
	#define PERMUT_MASK_DOUBLE(y, x) _MM_SHUFFLE2 (y, x)

	public:
	D_OPERATOR_NEW_AND_DELETE

	inline ndBigVector()
	{
	}

	inline ndBigVector(const ndBigVector& copy)
		:m_typeLow(copy.m_typeLow)
		,m_typeHigh(copy.m_typeHigh)
	{
	}

	inline ndBigVector(const __m128d typeLow, const __m128d typeHigh)
		:m_typeLow(typeLow)
		,m_typeHigh(typeHigh)
	{
	}

	inline ndBigVector(const __m128i typeLow, const __m128i typeHigh)
		:m_typeIntLow(typeLow)
		,m_typeIntHigh(typeHigh)
	{
	}

	inline ndBigVector(const ndFloat64 a)
		:m_typeLow(_mm_set1_pd(a))
		,m_typeHigh(_mm_set1_pd(a))
	{
	}

	inline ndBigVector(const ndFloat64* const baseAddr, const ndInt64* const index)
		:m_x(baseAddr[index[0]])
		,m_y(baseAddr[index[1]])
		,m_z(baseAddr[index[2]])
		,m_w(baseAddr[index[3]])
	{
	}

#ifdef D_NEWTON_USE_DOUBLE
	inline ndBigVector (const ndFloat32* const ptr)
		:m_typeLow(_mm_loadu_pd(ptr))
		,m_typeHigh(_mm_loadu_pd(&ptr[2]))
	{
	}
#else

	inline ndBigVector(const ndVector& v)
		:m_typeLow(_mm_cvtps_pd (v.m_type))
		,m_typeHigh(_mm_cvtps_pd (_mm_shuffle_ps (v.m_type, v.m_type, PERMUTE_MASK(3, 2, 3, 2))))
	{
		dAssert(dCheckVector((*this)));
	}

	inline ndBigVector(const ndFloat64* const ptr)
		:m_typeLow(_mm_loadu_pd(ptr))
		,m_typeHigh(_mm_loadu_pd(&ptr[2]))
	{
	}
#endif

	inline ndBigVector(ndFloat64 x, ndFloat64 y, ndFloat64 z, ndFloat64 w)
		:m_typeLow(_mm_set_pd(y, x))
		,m_typeHigh(_mm_set_pd(w, z))
	{
	}

	inline ndBigVector(ndInt32 ix, ndInt32 iy, ndInt32 iz, ndInt32 iw)
		:m_ix(ndInt64(ix)), m_iy(ndInt64(iy)), m_iz(ndInt64(iz)), m_iw(ndInt64(iw))
	{
	}

	inline ndBigVector(ndInt64 ix, ndInt64 iy, ndInt64 iz, ndInt64 iw)
		:m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	inline ndFloat64 GetX() const
	{
		return m_x;
	}

	inline ndFloat64 GetY() const
	{
		return m_y;
	}

	inline ndFloat64 GetZ() const
	{
		return m_z;
	}

	inline ndFloat64 GetW() const
	{
		return m_w;
	}

	inline void SetX(ndFloat64 x)
	{
		m_x = x;
	}

	inline void SetY(ndFloat64 x)
	{
		m_y = x;
	}

	inline void SetZ(ndFloat64 x)
	{
		m_z = x;
	}

	inline void SetW(ndFloat64 x)
	{
		m_w = x;
	}

	inline ndFloat64 GetScalar() const
	{
		//return m_x;
		return _mm_cvtsd_f64(m_typeLow);
	}

	inline ndFloat64& operator[] (ndInt32 i)
	{
		dAssert(i < 4);
		dAssert(i >= 0);
		return m_f[i];
	}

	inline const ndFloat64& operator[] (ndInt32 i) const
	{
		dAssert(i < 4);
		dAssert(i >= 0);
		return m_f[i];
	}

	inline ndBigVector operator+ (const ndBigVector& A) const
	{
		return ndBigVector(_mm_add_pd(m_typeLow, A.m_typeLow), _mm_add_pd(m_typeHigh, A.m_typeHigh));
	}

	inline ndBigVector operator- (const ndBigVector& A) const
	{
		return ndBigVector(_mm_sub_pd(m_typeLow, A.m_typeLow), _mm_sub_pd(m_typeHigh, A.m_typeHigh));
	}

	inline ndBigVector operator* (const ndBigVector& A) const
	{
		return ndBigVector(_mm_mul_pd(m_typeLow, A.m_typeLow), _mm_mul_pd(m_typeHigh, A.m_typeHigh));
	}

	inline ndBigVector& operator+= (const ndBigVector& A)
	{
		m_typeLow = _mm_add_pd(m_typeLow, A.m_typeLow);
		m_typeHigh = _mm_add_pd(m_typeHigh, A.m_typeHigh);
		return *this;
	}

	inline ndBigVector& operator-= (const ndBigVector& A)
	{
		m_typeLow = _mm_sub_pd(m_typeLow, A.m_typeLow);
		m_typeHigh = _mm_sub_pd(m_typeHigh, A.m_typeHigh);
		return *this;
	}

	inline ndBigVector& operator*= (const ndBigVector& A)
	{
		m_typeLow = _mm_mul_pd(m_typeLow, A.m_typeLow);
		m_typeHigh = _mm_mul_pd(m_typeHigh, A.m_typeHigh);
		return *this;
	}

	inline ndBigVector MulAdd(const ndBigVector& A, const ndBigVector& B) const
	{
		return *this + A * B;
	}

	inline ndBigVector MulSub(const ndBigVector& A, const ndBigVector& B) const
	{
		return *this - A * B;
	}

	inline ndBigVector AddHorizontal() const
	{
		__m128d tmp0(_mm_add_pd(m_typeHigh, m_typeLow));
		#ifdef D_USE_SSE3
			__m128d tmp1(_mm_hadd_pd(tmp0, tmp0));
		#else
			__m128d tmp1(_mm_add_pd(tmp0, _mm_shuffle_pd(tmp0, tmp0, PERMUT_MASK_DOUBLE(0, 1))));
		#endif
		return ndBigVector(tmp1, tmp1);
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

	inline ndBigVector Scale(ndFloat64 s) const
	{
		__m128d tmp0(_mm_set1_pd(s));
		return ndBigVector(_mm_mul_pd(m_typeLow, tmp0), _mm_mul_pd(m_typeHigh, tmp0));
	}

	inline ndBigVector Abs() const
	{
		return ndBigVector(_mm_and_pd(m_typeLow, m_signMask.m_typeLow), _mm_and_pd(m_typeHigh, m_signMask.m_typeLow));
	}

	inline ndBigVector Reciproc() const
	{
		return ndBigVector(_mm_div_pd(m_one.m_typeLow, m_typeLow), _mm_div_pd(m_one.m_typeHigh, m_typeHigh));
	}

	inline ndBigVector Sqrt() const
	{
		return ndBigVector(_mm_sqrt_pd(m_typeLow), _mm_sqrt_pd(m_typeHigh));
	}

	inline ndBigVector InvSqrt() const
	{
		return Sqrt().Reciproc();
	}

	inline ndBigVector InvMagSqrt() const
	{
		return DotProduct(*this).InvSqrt();
	}

	inline ndBigVector Normalize() const
	{
		ndFloat64 mag2 = DotProduct(*this).GetScalar();
		return Scale(ndFloat64 (1.0f) / sqrt (mag2));
	}

	ndBigVector GetMax() const
	{
		__m128d tmp(_mm_max_pd(m_typeLow, m_typeHigh));
		tmp = _mm_max_pd(tmp, _mm_shuffle_pd(tmp, tmp, PERMUT_MASK_DOUBLE(0, 1)));
		return ndBigVector(tmp, tmp);
	}

	ndBigVector GetMax(const ndBigVector& data) const
	{
		return ndBigVector(_mm_max_pd(m_typeLow, data.m_typeLow), _mm_max_pd(m_typeHigh, data.m_typeHigh));
	}

	ndBigVector GetMin(const ndBigVector& data) const
	{
		return ndBigVector(_mm_min_pd(m_typeLow, data.m_typeLow), _mm_min_pd(m_typeHigh, data.m_typeHigh));
	}

	inline ndBigVector GetInt() const
	{
		ndBigVector temp(Floor());
		ndInt64 x = _mm_cvtsd_si32(temp.m_typeLow);
		ndInt64 y = _mm_cvtsd_si32(_mm_shuffle_pd(temp.m_typeLow, temp.m_typeLow, PERMUT_MASK_DOUBLE(1, 1)));
		ndInt64 z = _mm_cvtsd_si32(temp.m_typeHigh);
		ndInt64 w = _mm_cvtsd_si32(_mm_shuffle_pd(temp.m_typeHigh, temp.m_typeHigh, PERMUT_MASK_DOUBLE(1, 1)));
		return ndBigVector(_mm_set_pd(*(ndFloat32*)&y, *(ndFloat32*)&x), _mm_set_pd(*(ndFloat32*)&w, *(ndFloat32*)&z));
	}

	// relational operators
	inline ndBigVector operator> (const ndBigVector& data) const
	{
		return ndBigVector(_mm_cmpgt_pd(m_typeLow, data.m_typeLow), _mm_cmpgt_pd(m_typeHigh, data.m_typeHigh));
	}

	inline ndBigVector operator== (const ndBigVector& data) const
	{
		return ndBigVector(_mm_cmpeq_pd(m_typeLow, data.m_typeLow), _mm_cmpeq_pd(m_typeHigh, data.m_typeHigh));
	}

	inline ndBigVector operator< (const ndBigVector& data) const
	{
		return ndBigVector(_mm_cmplt_pd(m_typeLow, data.m_typeLow), _mm_cmplt_pd(m_typeHigh, data.m_typeHigh));
	}

	inline ndBigVector operator>= (const ndBigVector& data) const
	{
		return ndBigVector(_mm_cmpge_pd(m_typeLow, data.m_typeLow), _mm_cmpge_pd(m_typeHigh, data.m_typeHigh));
	}

	inline ndBigVector operator<= (const ndBigVector& data) const
	{
		return ndBigVector(_mm_cmple_pd(m_typeLow, data.m_typeLow), _mm_cmple_pd(m_typeHigh, data.m_typeHigh));
	}

	// logical operations
	inline ndBigVector operator& (const ndBigVector& data) const
	{
		return ndBigVector(_mm_and_pd(m_typeLow, data.m_typeLow), _mm_and_pd(m_typeHigh, data.m_typeHigh));
	}

	inline ndBigVector operator| (const ndBigVector& data) const
	{
		return ndBigVector(_mm_or_pd(m_typeLow, data.m_typeLow), _mm_or_pd(m_typeHigh, data.m_typeHigh));
	}

	inline ndBigVector operator^ (const ndBigVector& data) const
	{
		return ndBigVector(_mm_xor_pd(m_typeLow, data.m_typeLow), _mm_xor_pd(m_typeHigh, data.m_typeHigh));
	}

	inline ndBigVector AndNot(const ndBigVector& data) const
	{
		return ndBigVector(_mm_andnot_pd(data.m_typeLow, m_typeLow), _mm_andnot_pd(data.m_typeHigh, m_typeHigh));
	}

	inline ndBigVector Select(const ndBigVector& data, const ndBigVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		return  ndBigVector(_mm_xor_pd(m_typeLow, _mm_and_pd(mask.m_typeLow, _mm_xor_pd(m_typeLow, data.m_typeLow))),
							_mm_xor_pd(m_typeHigh, _mm_and_pd(mask.m_typeHigh, _mm_xor_pd(m_typeHigh, data.m_typeHigh))));
	}

	inline ndBigVector ShiftRight() const
	{
		//return ndBigVector (m_w, m_x, m_y, m_z); 
		return ndBigVector(_mm_shuffle_pd(m_typeHigh, m_typeLow, PERMUT_MASK_DOUBLE(0, 1)), _mm_shuffle_pd(m_typeLow, m_typeHigh, PERMUT_MASK_DOUBLE(0, 1)));
	}

	inline ndBigVector ShiftTripleRight() const
	{
		return ndBigVector(_mm_shuffle_pd(m_typeHigh, m_typeLow, PERMUT_MASK_DOUBLE(0, 0)), _mm_shuffle_pd(m_typeLow, m_typeHigh, PERMUT_MASK_DOUBLE(1, 1)));
	}

	inline ndBigVector ShiftTripleLeft() const
	{
		return ndBigVector(_mm_shuffle_pd(m_typeLow, m_typeHigh, PERMUT_MASK_DOUBLE(0, 1)), _mm_shuffle_pd(m_typeLow, m_typeHigh, PERMUT_MASK_DOUBLE(1, 0)));
	}

	inline ndBigVector ShiftRightLogical(ndInt32 bits) const
	{
		//return ndBigVector(ndInt64(dUnsigned64(m_ix) >> bits), ndInt64(dUnsigned64(m_iy) >> bits), ndInt64(dUnsigned64(m_iz) >> bits), ndInt64(dUnsigned64(m_iw) >> bits));
		return ndBigVector(_mm_srli_epi64(m_typeIntLow, bits), _mm_srli_epi64(m_typeIntHigh, bits));
	}

	inline ndInt32 GetSignMask() const
	{
		return _mm_movemask_pd(m_typeLow) | (_mm_movemask_pd(m_typeHigh) << 2);
	}

	inline ndBigVector Floor() const
	{
		return ndBigVector(floor(m_x), floor(m_y), floor(m_z), floor(m_w));
	}

	inline ndBigVector TestZero() const
	{
		return m_negOne & (*this == m_zero);
	}

	inline static void Transpose4x4(ndBigVector& dst0, ndBigVector& dst1, ndBigVector& dst2, ndBigVector& dst3,
		const ndBigVector& src0, const ndBigVector& src1, const ndBigVector& src2, const ndBigVector& src3)
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

	// return dot 4d dot product
	inline ndBigVector DotProduct(const ndBigVector &A) const
	{
		const ndBigVector tmp(_mm_mul_pd(m_typeLow, A.m_typeLow), _mm_mul_pd(m_typeHigh, A.m_typeHigh));
		return tmp.AddHorizontal();
	}

	// return 3d cross product
	inline ndBigVector CrossProduct(const ndBigVector& B) const
	{
		return ndBigVector(m_y * B.m_z - m_z * B.m_y, m_z * B.m_x - m_x * B.m_z, m_x * B.m_y - m_y * B.m_x, m_w);
	}

	// return 4d cross product
	inline ndBigVector CrossProduct(const ndBigVector& A, const ndBigVector& B) const
	{
		ndFloat64 cofactor[3][3];
		ndFloat64 array[4][4];

		const ndBigVector& me = *this;
		for (ndInt32 i = 0; i < 4; i++) 
		{
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = ndFloat64(1.0f);
		}

		ndBigVector normal;
		ndFloat64 sign = ndFloat64(-1.0f);
		for (ndInt32 i = 0; i < 4; i++) 
		{
			for (ndInt32 j = 0; j < 3; j++) 
			{
				ndInt32 k0 = 0;
				for (ndInt32 k = 0; k < 4; k++) 
				{
					if (k != i) 
					{
						cofactor[j][k0] = array[j][k];
						k0++;
					}
				}
			}
			ndFloat64 x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			ndFloat64 y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			ndFloat64 z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			ndFloat64 det = x + y + z;

			normal[i] = sign * det;
			sign *= ndFloat64(-1.0f);
		}

		return normal;
	}

	union
	{
		ndFloat64 m_f[4];
		ndInt64 m_i[4];
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
			ndFloat64 m_x;
			ndFloat64 m_y;
			ndFloat64 m_z;
			ndFloat64 m_w;
		};
		struct
		{
			ndInt64 m_ix;
			ndInt64 m_iy;
			ndInt64 m_iz;
			ndInt64 m_iw;
		};
	};

	D_CORE_API static ndBigVector m_zero;
	D_CORE_API static ndBigVector m_one;
	D_CORE_API static ndBigVector m_wOne;
	D_CORE_API static ndBigVector m_two;
	D_CORE_API static ndBigVector m_half;
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
#endif
