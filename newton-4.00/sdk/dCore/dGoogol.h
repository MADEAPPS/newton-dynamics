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

#ifndef __dGoogol__
#define __dGoogol__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dMemory.h"
#include "dArray.h"
#include "dVector.h"
#include "dClassAlloc.h"
#include "dTemplateVector.h"


//#define DG_GOOGOL_SIZE	16
#define DG_GOOGOL_SIZE		4

class dGoogol
{
	public:
	D_OPERATOR_NEW_AND_DELETE

	dGoogol(void);
	dGoogol(dFloat64 value);

	operator double() const;	
	dGoogol operator+ (const dGoogol &A) const; 
	dGoogol operator- (const dGoogol &A) const; 
	dGoogol operator* (const dGoogol &A) const; 
	dGoogol operator/ (const dGoogol &A) const; 

	dGoogol operator+= (const dGoogol &A); 
	dGoogol operator-= (const dGoogol &A); 

	bool operator> (const dGoogol &A) const; 
	bool operator>= (const dGoogol &A) const; 
	bool operator< (const dGoogol &A) const; 
	bool operator<= (const dGoogol &A) const; 
	bool operator== (const dGoogol &A) const; 
	bool operator!= (const dGoogol &A) const; 

	dGoogol Abs () const;
	dGoogol Sqrt () const;
	dGoogol InvSqrt () const;
	dGoogol Floor () const;
	
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
	dUnsigned64 m_mantissa[DG_GOOGOL_SIZE];

	public:
	D_CORE_API static dGoogol m_zero; 
	D_CORE_API static dGoogol m_one; 
	D_CORE_API static dGoogol m_two; 
	D_CORE_API static dGoogol m_three; 
	D_CORE_API static dGoogol m_half; 
};

class dHugeVector: public dTemplateVector<dGoogol>
{
	public:
	dHugeVector ()
		:dTemplateVector<dGoogol>()
	{
	}

	dHugeVector (const dBigVector& a)
		:dTemplateVector<dGoogol>(dGoogol (a.m_x), dGoogol (a.m_y), dGoogol (a.m_z), dGoogol (a.m_w))
	{
	}

	dHugeVector (const dTemplateVector<dGoogol>& a)
		:dTemplateVector<dGoogol>(a)
	{
	}

	dHugeVector (dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w)
		:dTemplateVector<dGoogol>(x, y, z, w)
	{
	}

	dHugeVector(const dGoogol& x, const dGoogol& y, const dGoogol& z, const dGoogol& w)
		:dTemplateVector<dGoogol>(x, y, z, w)
	{
	}

	dGoogol EvaluePlane (const dHugeVector& point) const 
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
