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
	D_OPERATOR_NEW_AND_DELETE

	inline dVector() 
	{
	}

	inline dVector(const __m128i type)
		:m_typeInt (type)
	{
	}

	inline dVector(const __m128 type)
		: m_type(type)
	{
	}

	inline dVector (const dFloat32 a)
		:m_type(_mm_set_ps1(a)) 
	{
	}

	inline dVector (const dFloat32* const ptr)
		:m_type(_mm_loadu_ps (ptr))
	{
	}

	// emulate gather instruction
	inline dVector(const dFloat32* const baseAddr, const dInt32* const index)
		:m_x(baseAddr[index[0]])
		,m_y(baseAddr[index[1]])
		,m_z(baseAddr[index[2]])
		,m_w(baseAddr[index[3]])
	{
	}

#ifndef	D_NEWTON_USE_DOUBLE
	inline dVector(const dFloat64* const ptr)
		:m_type(_mm_set_ps(dFloat32(ptr[3]), dFloat32(ptr[2]), dFloat32(ptr[1]), dFloat32(ptr[0])))
	{
	}
#endif

	inline dVector (const dVector& copy)
		:m_type(copy.m_type)
	{
	}

	inline dVector (const dBigVector& copy)
		:m_type(_mm_shuffle_ps (_mm_cvtpd_ps (((__m128d*)&copy)[0]), _mm_cvtpd_ps (((__m128d*)&copy)[1]), PERMUTE_MASK(1, 0, 1, 0)))
	{
		dAssert (dCheckVector ((*this)));
	}

	inline dVector (dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w)
		:m_type(_mm_set_ps(w, z, y, x))
	{
	}

	inline dVector (dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
		:m_type(_mm_set_ps(*(dFloat32*)&iw, *(dFloat32*)&iz, *(dFloat32*)&iy, *(dFloat32*)&ix))
	{
	}

	inline dFloat32 GetX() const
	{
		return m_x;
	}

	inline dFloat32 GetY() const
	{
		return m_y;
	}

	inline dFloat32 GetZ() const
	{
		return m_z;
	}

	inline dFloat32 GetW() const
	{
		return m_w;
	}

	inline void SetX(dFloat32 x)
	{
		m_x = x;
	}

	inline void SetY(dFloat32 x)
	{
		m_y = x;
	}

	inline void SetZ(dFloat32 x)
	{
		m_z = x;
	}

	inline void SetW(dFloat32 x)
	{
		m_w = x;
	}

	inline dFloat32 GetScalar () const
	{
		return _mm_cvtss_f32 (m_type);
	}

	inline void Store (dFloat32* const dst) const
	{
		_mm_storeu_ps(dst, m_type);
	}

	inline dVector BroadcastX () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(0, 0, 0, 0));
	}

	inline dVector BroadcastY () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(1, 1, 1, 1));
	}

	inline dVector BroadcastZ () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(2, 2, 2, 2));
	}

	inline dVector BroadcastW () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(3, 3, 3, 3));
	}

	inline dVector Scale (dFloat32 s) const
	{
		return _mm_mul_ps (m_type, _mm_set_ps1(s));
	}

	inline dFloat32& operator[] (dInt32 i)
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return m_f[i];
	}

	inline const dFloat32& operator[] (dInt32 i) const
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return m_f[i];
	}

	inline dVector operator+ (const dVector& A) const
	{
		return _mm_add_ps (m_type, A.m_type);	
	}

	inline dVector operator- (const dVector& A) const 
	{
		return _mm_sub_ps (m_type, A.m_type);	
	}

	inline dVector operator* (const dVector& A) const
	{
		return _mm_mul_ps(m_type, A.m_type);
	}

	inline dVector& operator+= (const dVector& A)
	{
		return (*this = _mm_add_ps (m_type, A.m_type));
	}

	inline dVector& operator-= (const dVector& A)
	{
		return (*this = _mm_sub_ps (m_type, A.m_type));
	}

	inline dVector& operator*= (const dVector& A)
	{
		return (*this = _mm_mul_ps(m_type, A.m_type));
	}

	// return 4d cross product
	inline dVector DotProduct(const dVector& A) const
	{
		const dVector tmp(_mm_mul_ps(m_type, A.m_type));
		return tmp.AddHorizontal();
	}

	// return 3d cross product
	inline dVector CrossProduct (const dVector& B) const
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
	inline dVector CrossProduct (const dVector& A, const dVector& B) const
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

	inline dVector Reciproc () const
	{
		return _mm_div_ps (m_one.m_type, m_type);
	}

	inline dVector MulAdd(const dVector& A, const dVector& B) const
	{
		return _mm_add_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
	}

	inline dVector MulSub(const dVector& A, const dVector& B) const
	{
		return _mm_sub_ps(m_type, _mm_mul_ps(A.m_type, B.m_type));
	}

	inline dVector AddHorizontal () const
	{
		#ifdef D_USE_SSE3
			__m128 tmp (_mm_hadd_ps (m_type, m_type));
			return _mm_hadd_ps (tmp, tmp);
		#else
			__m128 tmp (_mm_add_ps (m_type, _mm_shuffle_ps(m_type, m_type, PERMUTE_MASK(1, 0, 3, 2))));
			return _mm_add_ps(tmp, _mm_shuffle_ps(tmp, tmp, PERMUTE_MASK(2, 3, 0, 1)));
		#endif	
	}

	inline dVector Abs () const
	{
		return _mm_and_ps (m_type, m_signMask.m_type);
	}

	dVector GetMax() const
	{
		__m128 tmp(_mm_max_ps(m_type, _mm_shuffle_ps(m_type, m_type, PERMUTE_MASK(1, 0, 3, 2))));
		return _mm_max_ps(tmp, _mm_shuffle_ps(tmp, tmp, PERMUTE_MASK(2, 3, 0, 1)));
	}

	dVector GetMax (const dVector& data) const
	{
		return _mm_max_ps (m_type, data.m_type);
	}

	dVector GetMin (const dVector& data) const
	{
		return _mm_min_ps (m_type, data.m_type);
	}

	inline dVector GetInt () const
	{
		return dVector(_mm_cvtps_epi32(Floor().m_type));
	}

	inline dVector TestZero() const
	{
		//return dVector (_mm_cmpeq_epi32 (m_typeInt, m_zero.m_typeInt)) & m_negOne;
		return m_negOne & (*this == m_zero);
	}

	inline dVector Floor () const
	{
		dVector truncated (_mm_cvtepi32_ps (_mm_cvttps_epi32 (m_type)));
		dVector ret (truncated - (dVector::m_one & (*this < truncated)));
		dAssert (ret.m_f[0] == dFloor(m_f[0]));
		dAssert (ret.m_f[1] == dFloor(m_f[1]));
		dAssert (ret.m_f[2] == dFloor(m_f[2]));
		dAssert (ret.m_f[3] == dFloor(m_f[3]));
		return ret;
	}

	inline dVector Sqrt () const
	{
		return _mm_sqrt_ps(m_type);
	}

	inline dVector InvSqrt () const
	{
		dVector tmp0 (_mm_rsqrt_ps(m_type));
		return m_half * tmp0 * (m_three - *this * tmp0 * tmp0);
	}

	inline dVector InvMagSqrt () const
	{
		return DotProduct(*this).InvSqrt();
	}

	inline dVector Normalize () const
	{
		return Scale(dFloat32(1.0f) / dSqrt(DotProduct(*this).GetScalar()));
	}

	// relational operators
	inline dVector operator> (const dVector& data) const
	{
		return _mm_cmpgt_ps (m_type, data.m_type);	
	}

	inline dVector operator== (const dVector& data) const
	{
		return _mm_cmpeq_ps (m_type, data.m_type);	
	}

	inline dVector operator< (const dVector& data) const
	{
		return _mm_cmplt_ps (m_type, data.m_type);	
	}

	inline dVector operator>= (const dVector& data) const
	{
		return _mm_cmpge_ps (m_type, data.m_type);	
	}

	inline dVector operator<= (const dVector& data) const
	{
		return _mm_cmple_ps (m_type, data.m_type);	
	}

	// logical operations
	inline dVector operator& (const dVector& data) const
	{
		return _mm_and_ps (m_type, data.m_type);	
	}

	inline dVector operator| (const dVector& data) const
	{
		return _mm_or_ps (m_type, data.m_type);	
	}

	inline dVector operator^ (const dVector& data) const
	{
		return _mm_xor_ps (m_type, data.m_type);	
	}

	inline dVector AndNot(const dVector& data) const
	{
		return _mm_andnot_ps(data.m_type, m_type);
	}

	inline dVector Select(const dVector& data, const dVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		//return  _mm_or_ps (_mm_and_ps (mask.m_type, data.m_type), _mm_andnot_ps(mask.m_type, m_type));
		return  _mm_xor_ps(m_type, _mm_and_ps (mask.m_type, _mm_xor_ps(m_type, data.m_type)));
	}

	inline dInt32 GetSignMask() const
	{
		return _mm_movemask_ps(m_type);
	} 

	inline dVector ShiftRight() const
	{
		return _mm_shuffle_ps(m_type, m_type, PERMUTE_MASK(2, 1, 0, 3));
	}

	inline dVector ShiftTripleRight () const
	{
		return _mm_shuffle_ps(m_type, m_type, PERMUTE_MASK(3, 1, 0, 2));
	}

	inline dVector ShiftTripleLeft () const
	{
		return _mm_shuffle_ps (m_type, m_type, PERMUTE_MASK(3, 0, 2, 1));
	}

	inline dVector ShiftRightLogical (dInt32 bits) const
	{
		return dVector (_mm_srli_epi32(m_typeInt, bits)); 
	}

	inline static void Transpose4x4 (dVector& dst0, dVector& dst1, dVector& dst2, dVector& dst3, const dVector& src0, const dVector& src1, const dVector& src2, const dVector& src3)
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
	D_CORE_API static dVector m_xyzwMask;
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
D_MSV_NEWTON_ALIGN_32
class dBigVector
{
	#define PERMUT_MASK_DOUBLE(y, x) _MM_SHUFFLE2 (y, x)

	public:
	D_OPERATOR_NEW_AND_DELETE

	inline dBigVector()
	{
	}

	inline dBigVector(const dBigVector& copy)
		:m_typeLow(copy.m_typeLow)
		,m_typeHigh(copy.m_typeHigh)
	{
	}

	inline dBigVector(const __m128d typeLow, const __m128d typeHigh)
		:m_typeLow(typeLow)
		,m_typeHigh(typeHigh)
	{
	}

	inline dBigVector(const __m128i typeLow, const __m128i typeHigh)
		:m_typeIntLow(typeLow)
		,m_typeIntHigh(typeHigh)
	{
	}

	inline dBigVector(const dFloat64 a)
		:m_typeLow(_mm_set1_pd(a))
		,m_typeHigh(_mm_set1_pd(a))
	{
	}

	inline dBigVector(const dFloat64* const baseAddr, const dInt64* const index)
		:m_x(baseAddr[index[0]])
		,m_y(baseAddr[index[1]])
		,m_z(baseAddr[index[2]])
		,m_w(baseAddr[index[3]])
	{
	}

#ifdef D_NEWTON_USE_DOUBLE
	inline dBigVector (const dFloat32* const ptr)
		:m_typeLow(_mm_loadu_pd(ptr))
		,m_typeHigh(_mm_loadu_pd(&ptr[2]))
	{
	}
#else

	inline dBigVector(const dVector& v)
		:m_typeLow(_mm_cvtps_pd (v.m_type))
		,m_typeHigh(_mm_cvtps_pd (_mm_shuffle_ps (v.m_type, v.m_type, PERMUTE_MASK(3, 2, 3, 2))))
	{
		dAssert(dCheckVector((*this)));
	}

	inline dBigVector(const dFloat64* const ptr)
		:m_typeLow(_mm_loadu_pd(ptr))
		,m_typeHigh(_mm_loadu_pd(&ptr[2]))
	{
	}
#endif

	inline dBigVector(dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w)
		:m_typeLow(_mm_set_pd(y, x))
		,m_typeHigh(_mm_set_pd(w, z))
	{
	}

	inline dBigVector(dInt32 ix, dInt32 iy, dInt32 iz, dInt32 iw)
		:m_ix(dInt64(ix)), m_iy(dInt64(iy)), m_iz(dInt64(iz)), m_iw(dInt64(iw))
	{
	}

	inline dBigVector(dInt64 ix, dInt64 iy, dInt64 iz, dInt64 iw)
		:m_ix(ix), m_iy(iy), m_iz(iz), m_iw(iw)
	{
	}

	inline dFloat64 GetX() const
	{
		return m_x;
	}

	inline dFloat64 GetY() const
	{
		return m_y;
	}

	inline dFloat64 GetZ() const
	{
		return m_z;
	}

	inline dFloat64 GetW() const
	{
		return m_w;
	}

	inline void SetX(dFloat64 x)
	{
		m_x = x;
	}

	inline void SetY(dFloat64 x)
	{
		m_y = x;
	}

	inline void SetZ(dFloat64 x)
	{
		m_z = x;
	}

	inline void SetW(dFloat64 x)
	{
		m_w = x;
	}

	inline dFloat64 GetScalar() const
	{
		//return m_x;
		return _mm_cvtsd_f64(m_typeLow);
	}

	inline dFloat64& operator[] (dInt32 i)
	{
		dAssert(i < 4);
		dAssert(i >= 0);
		return m_f[i];
	}

	inline const dFloat64& operator[] (dInt32 i) const
	{
		dAssert(i < 4);
		dAssert(i >= 0);
		return m_f[i];
	}

	inline dBigVector operator+ (const dBigVector& A) const
	{
		return dBigVector(_mm_add_pd(m_typeLow, A.m_typeLow), _mm_add_pd(m_typeHigh, A.m_typeHigh));
	}

	inline dBigVector operator- (const dBigVector& A) const
	{
		return dBigVector(_mm_sub_pd(m_typeLow, A.m_typeLow), _mm_sub_pd(m_typeHigh, A.m_typeHigh));
	}

	inline dBigVector operator* (const dBigVector& A) const
	{
		return dBigVector(_mm_mul_pd(m_typeLow, A.m_typeLow), _mm_mul_pd(m_typeHigh, A.m_typeHigh));
	}

	inline dBigVector& operator+= (const dBigVector& A)
	{
		m_typeLow = _mm_add_pd(m_typeLow, A.m_typeLow);
		m_typeHigh = _mm_add_pd(m_typeHigh, A.m_typeHigh);
		return *this;
	}

	inline dBigVector& operator-= (const dBigVector& A)
	{
		m_typeLow = _mm_sub_pd(m_typeLow, A.m_typeLow);
		m_typeHigh = _mm_sub_pd(m_typeHigh, A.m_typeHigh);
		return *this;
	}

	inline dBigVector& operator*= (const dBigVector& A)
	{
		m_typeLow = _mm_mul_pd(m_typeLow, A.m_typeLow);
		m_typeHigh = _mm_mul_pd(m_typeHigh, A.m_typeHigh);
		return *this;
	}

	inline dBigVector MulAdd(const dBigVector& A, const dBigVector& B) const
	{
		return *this + A * B;
	}

	inline dBigVector MulSub(const dBigVector& A, const dBigVector& B) const
	{
		return *this - A * B;
	}

	inline dBigVector AddHorizontal() const
	{
		__m128d tmp0(_mm_add_pd(m_typeHigh, m_typeLow));
		#ifdef D_USE_SSE3
			__m128d tmp1(_mm_hadd_pd(tmp0, tmp0));
		#else
			__m128d tmp1(_mm_add_pd(tmp0, _mm_shuffle_pd(tmp0, tmp0, PERMUT_MASK_DOUBLE(0, 1))));
		#endif
		return dBigVector(tmp1, tmp1);
	}

	inline dBigVector BroadcastX() const
	{
		return dBigVector(m_x);
	}

	inline dBigVector BroadcastY() const
	{
		return dBigVector(m_y);
	}

	inline dBigVector BroadcastZ() const
	{
		return dBigVector(m_z);
	}

	inline dBigVector BroadcastW() const
	{
		return dBigVector(m_w);
	}

	inline dBigVector Scale(dFloat64 s) const
	{
		__m128d tmp0(_mm_set1_pd(s));
		return dBigVector(_mm_mul_pd(m_typeLow, tmp0), _mm_mul_pd(m_typeHigh, tmp0));
	}

	inline dBigVector Abs() const
	{
		return dBigVector(_mm_and_pd(m_typeLow, m_signMask.m_typeLow), _mm_and_pd(m_typeHigh, m_signMask.m_typeLow));
	}

	inline dBigVector Reciproc() const
	{
		return dBigVector(_mm_div_pd(m_one.m_typeLow, m_typeLow), _mm_div_pd(m_one.m_typeHigh, m_typeHigh));
	}

	inline dBigVector Sqrt() const
	{
		return dBigVector(_mm_sqrt_pd(m_typeLow), _mm_sqrt_pd(m_typeHigh));
	}

	inline dBigVector InvSqrt() const
	{
		return Sqrt().Reciproc();
	}

	inline dBigVector InvMagSqrt() const
	{
		return DotProduct(*this).InvSqrt();
	}

	inline dBigVector Normalize() const
	{
		dFloat64 mag2 = DotProduct(*this).GetScalar();
		return Scale(dFloat64 (1.0f) / sqrt (mag2));
	}

	dBigVector GetMax() const
	{
		__m128d tmp(_mm_max_pd(m_typeLow, m_typeHigh));
		tmp = _mm_max_pd(tmp, _mm_shuffle_pd(tmp, tmp, PERMUT_MASK_DOUBLE(0, 1)));
		return dBigVector(tmp, tmp);
	}

	dBigVector GetMax(const dBigVector& data) const
	{
		return dBigVector(_mm_max_pd(m_typeLow, data.m_typeLow), _mm_max_pd(m_typeHigh, data.m_typeHigh));
	}

	dBigVector GetMin(const dBigVector& data) const
	{
		return dBigVector(_mm_min_pd(m_typeLow, data.m_typeLow), _mm_min_pd(m_typeHigh, data.m_typeHigh));
	}

	inline dBigVector GetInt() const
	{
		dBigVector temp(Floor());
		dInt64 x = _mm_cvtsd_si32(temp.m_typeLow);
		dInt64 y = _mm_cvtsd_si32(_mm_shuffle_pd(temp.m_typeLow, temp.m_typeLow, PERMUT_MASK_DOUBLE(1, 1)));
		dInt64 z = _mm_cvtsd_si32(temp.m_typeHigh);
		dInt64 w = _mm_cvtsd_si32(_mm_shuffle_pd(temp.m_typeHigh, temp.m_typeHigh, PERMUT_MASK_DOUBLE(1, 1)));
		return dBigVector(_mm_set_pd(*(dFloat32*)&y, *(dFloat32*)&x), _mm_set_pd(*(dFloat32*)&w, *(dFloat32*)&z));
	}

	// relational operators
	inline dBigVector operator> (const dBigVector& data) const
	{
		return dBigVector(_mm_cmpgt_pd(m_typeLow, data.m_typeLow), _mm_cmpgt_pd(m_typeHigh, data.m_typeHigh));
	}

	inline dBigVector operator== (const dBigVector& data) const
	{
		return dBigVector(_mm_cmpeq_pd(m_typeLow, data.m_typeLow), _mm_cmpeq_pd(m_typeHigh, data.m_typeHigh));
	}

	inline dBigVector operator< (const dBigVector& data) const
	{
		return dBigVector(_mm_cmplt_pd(m_typeLow, data.m_typeLow), _mm_cmplt_pd(m_typeHigh, data.m_typeHigh));
	}

	inline dBigVector operator>= (const dBigVector& data) const
	{
		return dBigVector(_mm_cmpge_pd(m_typeLow, data.m_typeLow), _mm_cmpge_pd(m_typeHigh, data.m_typeHigh));
	}

	inline dBigVector operator<= (const dBigVector& data) const
	{
		return dBigVector(_mm_cmple_pd(m_typeLow, data.m_typeLow), _mm_cmple_pd(m_typeHigh, data.m_typeHigh));
	}

	// logical operations
	inline dBigVector operator& (const dBigVector& data) const
	{
		return dBigVector(_mm_and_pd(m_typeLow, data.m_typeLow), _mm_and_pd(m_typeHigh, data.m_typeHigh));
	}

	inline dBigVector operator| (const dBigVector& data) const
	{
		return dBigVector(_mm_or_pd(m_typeLow, data.m_typeLow), _mm_or_pd(m_typeHigh, data.m_typeHigh));
	}

	inline dBigVector operator^ (const dBigVector& data) const
	{
		return dBigVector(_mm_xor_pd(m_typeLow, data.m_typeLow), _mm_xor_pd(m_typeHigh, data.m_typeHigh));
	}

	inline dBigVector AndNot(const dBigVector& data) const
	{
		return dBigVector(_mm_andnot_pd(data.m_typeLow, m_typeLow), _mm_andnot_pd(data.m_typeHigh, m_typeHigh));
	}

	inline dBigVector Select(const dBigVector& data, const dBigVector& mask) const
	{
		// (((b ^ a) & mask)^a)
		return  dBigVector(_mm_xor_pd(m_typeLow, _mm_and_pd(mask.m_typeLow, _mm_xor_pd(m_typeLow, data.m_typeLow))),
							_mm_xor_pd(m_typeHigh, _mm_and_pd(mask.m_typeHigh, _mm_xor_pd(m_typeHigh, data.m_typeHigh))));
	}

	inline dBigVector ShiftRight() const
	{
		//return dBigVector (m_w, m_x, m_y, m_z); 
		return dBigVector(_mm_shuffle_pd(m_typeHigh, m_typeLow, PERMUT_MASK_DOUBLE(0, 1)), _mm_shuffle_pd(m_typeLow, m_typeHigh, PERMUT_MASK_DOUBLE(0, 1)));
	}

	inline dBigVector ShiftTripleRight() const
	{
		return dBigVector(_mm_shuffle_pd(m_typeHigh, m_typeLow, PERMUT_MASK_DOUBLE(0, 0)), _mm_shuffle_pd(m_typeLow, m_typeHigh, PERMUT_MASK_DOUBLE(1, 1)));
	}

	inline dBigVector ShiftTripleLeft() const
	{
		return dBigVector(_mm_shuffle_pd(m_typeLow, m_typeHigh, PERMUT_MASK_DOUBLE(0, 1)), _mm_shuffle_pd(m_typeLow, m_typeHigh, PERMUT_MASK_DOUBLE(1, 0)));
	}

	inline dBigVector ShiftRightLogical(dInt32 bits) const
	{
		//return dBigVector(dInt64(dUnsigned64(m_ix) >> bits), dInt64(dUnsigned64(m_iy) >> bits), dInt64(dUnsigned64(m_iz) >> bits), dInt64(dUnsigned64(m_iw) >> bits));
		return dBigVector(_mm_srli_epi64(m_typeIntLow, bits), _mm_srli_epi64(m_typeIntHigh, bits));
	}

	inline dInt32 GetSignMask() const
	{
		return _mm_movemask_pd(m_typeLow) | (_mm_movemask_pd(m_typeHigh) << 2);
	}

	inline dBigVector Floor() const
	{
		return dBigVector(floor(m_x), floor(m_y), floor(m_z), floor(m_w));
	}

	inline dBigVector TestZero() const
	{
		return m_negOne & (*this == m_zero);
	}

	inline static void Transpose4x4(dBigVector& dst0, dBigVector& dst1, dBigVector& dst2, dBigVector& dst3,
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
	inline dBigVector DotProduct(const dBigVector &A) const
	{
		const dBigVector tmp(_mm_mul_pd(m_typeLow, A.m_typeLow), _mm_mul_pd(m_typeHigh, A.m_typeHigh));
		return tmp.AddHorizontal();
	}

	// return 3d cross product
	inline dBigVector CrossProduct(const dBigVector& B) const
	{
		return dBigVector(m_y * B.m_z - m_z * B.m_y, m_z * B.m_x - m_x * B.m_z, m_x * B.m_y - m_y * B.m_x, m_w);
	}

	// return 4d cross product
	inline dBigVector CrossProduct(const dBigVector& A, const dBigVector& B) const
	{
		dFloat64 cofactor[3][3];
		dFloat64 array[4][4];

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
	D_CORE_API static dBigVector m_xyzwMask;
	D_CORE_API static dBigVector m_epsilon;
	D_CORE_API static dBigVector m_signMask;
	D_CORE_API static dBigVector m_triplexMask;
} D_GCC_NEWTON_ALIGN_32 ;

#endif
#endif
