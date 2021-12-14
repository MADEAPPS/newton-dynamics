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

#ifndef __NDGoogol__
#define __NDGoogol__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndMemory.h"
#include "ndArray.h"
#include "ndVector.h"
#include "ndClassAlloc.h"
#include "ndTemplateVector.h"


//#define ND_GOOGOL_SIZE	16
#define ND_GOOGOL_SIZE		4

class ndGoogol
{
	public:
	D_OPERATOR_NEW_AND_DELETE

	ndGoogol(void);
	ndGoogol(ndFloat64 value);

	operator double() const;	
	ndGoogol operator+ (const ndGoogol &A) const; 
	ndGoogol operator- (const ndGoogol &A) const; 
	ndGoogol operator* (const ndGoogol &A) const; 
	ndGoogol operator/ (const ndGoogol &A) const; 

	ndGoogol operator+= (const ndGoogol &A); 
	ndGoogol operator-= (const ndGoogol &A); 

	bool operator> (const ndGoogol &A) const; 
	bool operator>= (const ndGoogol &A) const; 
	bool operator< (const ndGoogol &A) const; 
	bool operator<= (const ndGoogol &A) const; 
	bool operator== (const ndGoogol &A) const; 
	bool operator!= (const ndGoogol &A) const; 

	ndGoogol Abs () const;
	ndGoogol Sqrt () const;
	ndGoogol InvSqrt () const;
	ndGoogol Floor () const;
	
	void Trace () const;
	void ToString (char* const string) const;

	private:
	void InitFloatFloat (ndFloat64 value);
	void NegateMantissa (ndUnsigned64* const mantissa) const;
	void CopySignedMantissa (ndUnsigned64* const mantissa) const;
	ndInt32 NormalizeMantissa (ndUnsigned64* const mantissa) const;
	ndUnsigned64 CheckCarrier (ndUnsigned64 a, ndUnsigned64 b) const;
	void ShiftRightMantissa (ndUnsigned64* const mantissa, ndInt32 bits) const;

	ndInt32 LeadingZeros (ndUnsigned64 a) const;
	void ExtendeMultiply (ndUnsigned64 a, ndUnsigned64 b, ndUnsigned64& high, ndUnsigned64& low) const;
	void ScaleMantissa (ndUnsigned64* const out, ndUnsigned64 scale) const;

	ndInt32 m_sign;
	ndInt32 m_exponent;
	ndUnsigned64 m_mantissa[ND_GOOGOL_SIZE];

	public:
	D_CORE_API static ndGoogol m_zero; 
	D_CORE_API static ndGoogol m_one; 
	D_CORE_API static ndGoogol m_two; 
	D_CORE_API static ndGoogol m_three; 
	D_CORE_API static ndGoogol m_half; 
};

class ndHugeVector: public ndTemplateVector<ndGoogol>
{
	public:
	ndHugeVector ()
		:ndTemplateVector<ndGoogol>()
	{
	}

	ndHugeVector (const ndBigVector& a)
		:ndTemplateVector<ndGoogol>(ndGoogol (a.m_x), ndGoogol (a.m_y), ndGoogol (a.m_z), ndGoogol (a.m_w))
	{
	}

	ndHugeVector (const ndTemplateVector<ndGoogol>& a)
		:ndTemplateVector<ndGoogol>(a)
	{
	}

	ndHugeVector (ndFloat64 x, ndFloat64 y, ndFloat64 z, ndFloat64 w)
		:ndTemplateVector<ndGoogol>(x, y, z, w)
	{
	}

	ndHugeVector(const ndGoogol& x, const ndGoogol& y, const ndGoogol& z, const ndGoogol& w)
		:ndTemplateVector<ndGoogol>(x, y, z, w)
	{
	}

	ndGoogol EvaluePlane (const ndHugeVector& point) const 
	{
		//return (point % (*this)) + m_w;
		return DotProduct(point).GetScalar();
	}

#ifdef _DEBUG
	void Trace () const
	{
		m_x.Trace();
		m_y.Trace();
		m_z.Trace();
		m_w.Trace();
		dAssert(0);
//		dTrace (("\n"));
	}
#endif
};


#endif
