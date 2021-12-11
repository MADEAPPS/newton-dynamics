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


#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndGoogol.h"

ndGoogol ndGoogol::m_zero(0.0); 
ndGoogol ndGoogol::m_one(1.0); 
ndGoogol ndGoogol::m_two(2.0);  
ndGoogol ndGoogol::m_three(3.0);   
ndGoogol ndGoogol::m_half(0.5);   

ndGoogol::ndGoogol(void)
	:m_sign(0)
	,m_exponent(0)
{
	memset (m_mantissa, 0, sizeof (m_mantissa));
}

ndGoogol::ndGoogol(dFloat64 value)
	:m_sign(0)
	,m_exponent(0)
{
	dInt32 exp;
	dFloat64 mantissa = fabs (frexp(value, &exp));

	m_exponent = dInt16 (exp);
	m_sign = (value >= 0) ? 0 : 1;

	memset (m_mantissa, 0, sizeof (m_mantissa));
	m_mantissa[0] = dUnsigned64 (dFloat64 (dUnsigned64(1)<<62) * mantissa);

	// it looks like GCC have problems with this
	//dAssert (m_mantissa[0] >= 0);
	dAssert ((m_mantissa[0] & dUnsigned64(1)<<63) == 0);
}

void ndGoogol::CopySignedMantissa (dUnsigned64* const mantissa) const
{
	memcpy (mantissa, m_mantissa, sizeof (m_mantissa));
	if (m_sign) 
	{
		NegateMantissa (mantissa);
	}
}

ndGoogol::operator double() const
{
	dFloat64 mantissa = (dFloat64(1.0f) / dFloat64 (dUnsigned64(1)<<62)) * dFloat64 (m_mantissa[0]);
	mantissa = ldexp(mantissa, m_exponent) * (m_sign ?  dFloat64 (-1.0f) : dFloat64 (1.0f));
	return mantissa;
}

ndGoogol ndGoogol::operator+ (const ndGoogol &A) const
{
	ndGoogol tmp;
	dAssert (dInt64 (m_mantissa[0]) >= 0);
	dAssert (dInt64 (A.m_mantissa[0]) >= 0);

	if (m_mantissa[0] && A.m_mantissa[0]) 
	{
		dUnsigned64 mantissa0[ND_GOOGOL_SIZE];
		dUnsigned64 mantissa1[ND_GOOGOL_SIZE];
		dUnsigned64 mantissa[ND_GOOGOL_SIZE];

		CopySignedMantissa (mantissa0);
		A.CopySignedMantissa (mantissa1);

		dInt32 exponetDiff = m_exponent - A.m_exponent;
		dInt32 exponent = m_exponent;
		if (exponetDiff > 0) 
		{
			ShiftRightMantissa (mantissa1, exponetDiff);
		} 
		else if (exponetDiff < 0) 
		{
			exponent = A.m_exponent;
			ShiftRightMantissa (mantissa0, -exponetDiff);
		} 

		dUnsigned64 carrier = 0;
		for (dInt32 i = ND_GOOGOL_SIZE - 1; i >= 0; i --) 
		{
			dUnsigned64 m0 = mantissa0[i];
			dUnsigned64 m1 = mantissa1[i];
			mantissa[i] = m0 + m1 + carrier;
			carrier = CheckCarrier (m0, m1) | CheckCarrier (m0 + m1, carrier);
		}

		dInt8 sign = 0;
		if (dInt64 (mantissa[0]) < 0) 
		{
			sign = 1;
			NegateMantissa (mantissa);
		}

		dInt32 bits = NormalizeMantissa (mantissa);
		if (bits <= (-64 * ND_GOOGOL_SIZE)) 
		{
			tmp.m_sign = 0;
			tmp.m_exponent = 0;
		} 
		else 
		{
			tmp.m_sign = sign;
			tmp.m_exponent =  dInt16 (exponent + bits);
		}

		memcpy (tmp.m_mantissa, mantissa, sizeof (m_mantissa));
	} 
	else if (A.m_mantissa[0]) 
	{
		tmp = A;
	} 
	else 
	{
		tmp = *this;
	}

	dAssert (dInt64 (tmp.m_mantissa[0]) >= 0);
	return tmp;
}

ndGoogol ndGoogol::operator- (const ndGoogol &A) const
{
	ndGoogol tmp (A);
	tmp.m_sign = !tmp.m_sign;
	return *this + tmp;
}

void ndGoogol::ScaleMantissa (dUnsigned64* const dst, dUnsigned64 scale) const
{
	dUnsigned64 carrier = 0;
	for (dInt32 i = ND_GOOGOL_SIZE - 1; i >= 0; i --) 
	{
		if (m_mantissa[i]) 
		{
			dUnsigned64 low;
			dUnsigned64 high;
			ExtendeMultiply (scale, m_mantissa[i], high, low);
			dUnsigned64 acc = low + carrier;
			carrier = CheckCarrier (low, carrier);	
			dAssert (CheckCarrier (carrier, high) == 0);
			carrier += high;
			dst[i + 1] = acc;
		} 
		else 
		{
			dst[i + 1] = carrier;
			carrier = 0;
		}

	}
	dst[0] = carrier;
}

ndGoogol ndGoogol::operator* (const ndGoogol &A) const
{
	dAssert (dInt64 (m_mantissa[0]) >= 0);
	dAssert (dInt64 (A.m_mantissa[0]) >= 0);

	if (m_mantissa[0] && A.m_mantissa[0]) 
	{
		dUnsigned64 mantissaAcc[ND_GOOGOL_SIZE * 2];
		memset (mantissaAcc, 0, sizeof (mantissaAcc));
		for (dInt32 i = ND_GOOGOL_SIZE - 1; i >= 0; i --) 
		{
			dUnsigned64 a = m_mantissa[i];
			if (a) 
			{
				dUnsigned64 mantissaScale[2 * ND_GOOGOL_SIZE];
				memset (mantissaScale, 0, sizeof (mantissaScale));
				A.ScaleMantissa (&mantissaScale[i], a);

				dUnsigned64 carrier = 0;
				for (dInt32 j = 0; j < 2 * ND_GOOGOL_SIZE; j ++) 
				{
					const dInt32 k = 2 * ND_GOOGOL_SIZE - 1 - j;
					dUnsigned64 m0 = mantissaAcc[k];
					dUnsigned64 m1 = mantissaScale[k];
					mantissaAcc[k] = m0 + m1 + carrier;
					carrier = CheckCarrier (m0, m1) | CheckCarrier (m0 + m1, carrier);
				}
			}
		}

		dUnsigned64 carrier = 0;
		//dInt32 bits = dUnsigned64(LeadingZeros (mantissaAcc[0]) - 2);
		dInt32 bits = LeadingZeros (mantissaAcc[0]) - 2;
		for (dInt32 i = 0; i < 2 * ND_GOOGOL_SIZE; i ++) 
		{
			const dInt32 k = 2 * ND_GOOGOL_SIZE - 1 - i;
			dUnsigned64 a = mantissaAcc[k];
			mantissaAcc[k] = (a << dUnsigned64(bits)) | carrier;
			carrier = a >> dUnsigned64(64 - bits);
		}

		dInt32 exp = m_exponent + A.m_exponent - (bits - 2);

		ndGoogol tmp;
		tmp.m_sign = m_sign ^ A.m_sign;
		tmp.m_exponent = dInt16 (exp);
		memcpy (tmp.m_mantissa, mantissaAcc, sizeof (m_mantissa));

		return tmp;
	} 
	return ndGoogol(0.0);
}

ndGoogol ndGoogol::operator/ (const ndGoogol &A) const
{
	ndGoogol tmp (1.0 / A);
	tmp = tmp * (m_two - A * tmp);
	tmp = tmp * (m_two - A * tmp);
	dInt32 test = 0;
	dInt32 passes = 0;
	do 
	{
		passes ++;
		ndGoogol tmp0 (tmp);
		tmp = tmp * (m_two - A * tmp);
		test = memcmp (&tmp0, &tmp, sizeof (ndGoogol));
	} while (test && (passes < (2 * ND_GOOGOL_SIZE)));	
	dAssert (passes <= (2 * ND_GOOGOL_SIZE));
	return (*this) * tmp;
}

ndGoogol ndGoogol::Abs () const
{
	ndGoogol tmp (*this);
	tmp.m_sign = 0;
	return tmp;
}

ndGoogol ndGoogol::Floor () const
{
	if (m_exponent < 1) 
	{
		return ndGoogol (0.0);
	} 
	dInt32 bits = m_exponent + 2;
	dInt32 start = 0;
	while (bits >= 64) 
	{
		bits -= 64;
		start ++;
	}

	ndGoogol tmp (*this);
	for (dInt32 i = ND_GOOGOL_SIZE - 1; i > start; i --) 
	{
		tmp.m_mantissa[i] = 0;
	}
	// some compilers do no like this and I do not know why is that
	//dUnsigned64 mask = (-1LL) << (64 - bits);
	dUnsigned64 mask (~0ULL);
	mask <<= (64 - bits);
	tmp.m_mantissa[start] &= mask;
	if (m_sign) {
		dAssert (0);
	}

	return tmp;
}

ndGoogol ndGoogol::InvSqrt () const
{
	const ndGoogol& me = *this;
	ndGoogol x (1.0f / sqrt (me));

	dInt32 test = 0;
	dInt32 passes = 0;
	do 
	{
		passes ++;
		ndGoogol tmp (x);
		x = m_half * x * (m_three - me * x * x);
		test = memcmp (&x, &tmp, sizeof (ndGoogol));
	} while (test && (passes < (2 * ND_GOOGOL_SIZE)));	
	dAssert (passes <= (2 * ND_GOOGOL_SIZE));
	return x;
}

ndGoogol ndGoogol::Sqrt () const
{
	return *this * InvSqrt();
}

void ndGoogol::ToString (char* const string) const
{
	ndGoogol tmp (*this);
	ndGoogol base (10.0);
	while (dFloat64 (tmp) > 1.0) 
	{
		tmp = tmp/base;
	}

	dInt32 index = 0;
	while (tmp.m_mantissa[0]) 
	{
		tmp = tmp * base;
		ndGoogol digit (tmp.Floor());
		tmp -= digit;
		dFloat64 val = digit;
		string[index] = char (val) + '0';
		index ++;
	}
	string[index] = 0;
}

void ndGoogol::NegateMantissa (dUnsigned64* const mantissa) const
{
	dUnsigned64 carrier = 1;
	for (dInt32 i = ND_GOOGOL_SIZE - 1; i >= 0; i --) 
	{
		dUnsigned64 a = ~mantissa[i] + carrier;
		if (a) 
		{
			carrier = 0;
		}
		mantissa[i] = a;
	}
}

void ndGoogol::ShiftRightMantissa (dUnsigned64* const mantissa, dInt32 bits) const
{
	dUnsigned64 carrier = 0;
	if (dInt64 (mantissa[0]) < dInt64 (0)) 
	{
		carrier = dUnsigned64 (-1);
	}
	
	while (bits >= 64) 
	{
		for (dInt32 i = ND_GOOGOL_SIZE - 2; i >= 0; i --) 
		{
			mantissa[i + 1] = mantissa[i];
		}
		mantissa[0] = carrier;
		bits -= 64;
	}

	if (bits > 0) 
	{
		carrier <<= (64 - bits);
		for (dInt32 i = 0; i < ND_GOOGOL_SIZE; i ++) 
		{
			dUnsigned64 a = mantissa[i];
			mantissa[i] = (a >> bits) | carrier;
			carrier = a << (64 - bits);
		}
	}
}

dInt32 ndGoogol::LeadingZeros (dUnsigned64 a) const
{
	#define dgCOUNTBIT(mask,add)		\
	{									\
		dUnsigned64 test = a & mask;	\
		n += test ? 0 : add;			\
		a = test ? test : (a & ~mask);	\
	}

	dInt32 n = 0;
    dAssert (a);
	dgCOUNTBIT (0xffffffff00000000LL, 32);
	dgCOUNTBIT (0xffff0000ffff0000LL, 16);
	dgCOUNTBIT (0xff00ff00ff00ff00LL,  8);
	dgCOUNTBIT (0xf0f0f0f0f0f0f0f0LL,  4);
	dgCOUNTBIT (0xccccccccccccccccLL,  2);
	dgCOUNTBIT (0xaaaaaaaaaaaaaaaaLL,  1);

	return n;
}

dInt32 ndGoogol::NormalizeMantissa (dUnsigned64* const mantissa) const
{
	dAssert (dInt64 (mantissa[0]) >= 0);

	dInt32 bits = 0;
	if(dInt64 (mantissa[0] * 2) < 0) 
	{
		bits = 1;
		ShiftRightMantissa (mantissa, 1);
	} 
	else 
	{
		while (!mantissa[0] && bits > (-64 * ND_GOOGOL_SIZE)) 
		{
			bits -= 64;
			for (dInt32 i = 1; i < ND_GOOGOL_SIZE; i ++) {
				mantissa[i - 1] = mantissa[i];
			}
			mantissa[ND_GOOGOL_SIZE - 1] = 0;
		}

		if (bits > (-64 * ND_GOOGOL_SIZE)) 
		{
			dInt32 n = LeadingZeros (mantissa[0]) - 2;
			if (n > 0) {
				dAssert (n > 0);
				dUnsigned64 carrier = 0;
				for (dInt32 i = ND_GOOGOL_SIZE-1; i >= 0; i --) {
					dUnsigned64 a = mantissa[i];
					mantissa[i] = (a << n) | carrier;
					carrier = a >> (64 - n);
				}
				bits -= n;
			} 
			else if (n < 0) 
			{
				// this is very rare but it does happens, whee the leading zeros of the mantissa is an exact multiple of 64
				dAssert (mantissa[0] & dUnsigned64(3)<<62);
				dUnsigned64 carrier = 0;
				dInt32 shift = -n;
				for (dInt32 i = 0; i < ND_GOOGOL_SIZE; i ++) 
				{
					dUnsigned64 a = mantissa[i];
					mantissa[i] = (a >> shift) | carrier;
					carrier = a << (64 - shift);
				}
				bits -= n;
			}
		}
	}
	return bits;
}

dUnsigned64 ndGoogol::CheckCarrier (dUnsigned64 a, dUnsigned64 b) const
{
	return ((dUnsigned64 (-1) - b) < a) ? dUnsigned64(1) : 0;
}

void ndGoogol::ExtendeMultiply (dUnsigned64 a, dUnsigned64 b, dUnsigned64& high, dUnsigned64& low) const
{
	dUnsigned64 bLow = b & 0xffffffff; 
	dUnsigned64 bHigh = b >> 32; 
	dUnsigned64 aLow = a & 0xffffffff; 
	dUnsigned64 aHigh = a >> 32; 

	dUnsigned64 l = bLow * aLow;

	dUnsigned64 c1 = bHigh * aLow;
	dUnsigned64 c2 = bLow * aHigh;
	dUnsigned64 m = c1 + c2;
	dUnsigned64 carrier = CheckCarrier (c1, c2) << 32;

	dUnsigned64 h = bHigh * aHigh + carrier;

	dUnsigned64 ml = m << 32;	
	dUnsigned64 ll = l + ml;
	dUnsigned64 mh = (m >> 32) + CheckCarrier (l, ml);	
	dAssert ((mh & ~0xffffffff) == 0);

	dUnsigned64 hh = h + mh;

	low = ll;
	high = hh;
}

ndGoogol ndGoogol::operator+= (const ndGoogol &A)
{
	*this = *this + A;
	return *this;
}

ndGoogol ndGoogol::operator-= (const ndGoogol &A)
{
	*this = *this - A;
	return *this;
}

bool ndGoogol::operator> (const ndGoogol &A) const
{
	ndGoogol tmp (*this - A);
	return dFloat64(tmp) > 0.0;
}

bool ndGoogol::operator>= (const ndGoogol &A) const 
{
	ndGoogol tmp (*this - A);
	return dFloat64 (tmp) >= 0.0;
}

bool ndGoogol::operator< (const ndGoogol &A) const 
{
	ndGoogol tmp (*this - A);
	return dFloat64 (tmp) < 0.0;
}

bool ndGoogol::operator<= (const ndGoogol &A) const 
{
	ndGoogol tmp (*this - A);
	return dFloat64 (tmp) <= 0.0;
}

bool ndGoogol::operator== (const ndGoogol &A) const 
{
	ndGoogol tmp (*this - A);
	return dFloat64 (tmp) == 0.0;
}

bool ndGoogol::operator!= (const ndGoogol &A) const 
{
	ndGoogol tmp (*this - A);
	return dFloat64 (tmp) != 0.0;
}

void ndGoogol::Trace () const
{
	dAssert(0);
	//dTrace (("%f ", dFloat64 (*this)));
}
