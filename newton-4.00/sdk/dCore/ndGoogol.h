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
	ndGoogol(dFloat64 value);

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
	void InitFloatFloat (dFloat64 value);
	void NegateMantissa (dUnsigned64* const mantissa) const;
	void CopySignedMantissa (dUnsigned64* const mantissa) const;
	dInt32 NormalizeMantissa (dUnsigned64* const mantissa) const;
	dUnsigned64 CheckCarrier (dUnsigned64 a, dUnsigned64 b) const;
	void ShiftRightMantissa (dUnsigned64* const mantissa, dInt32 bits) const;

	dInt32 LeadingZeros (dUnsigned64 a) const;
	void ExtendeMultiply (dUnsigned64 a, dUnsigned64 b, dUnsigned64& high, dUnsigned64& low) const;
	void ScaleMantissa (dUnsigned64* const out, dUnsigned64 scale) const;

	dInt32 m_sign;
	dInt32 m_exponent;
	dUnsigned64 m_mantissa[ND_GOOGOL_SIZE];

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

	ndHugeVector (dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w)
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
