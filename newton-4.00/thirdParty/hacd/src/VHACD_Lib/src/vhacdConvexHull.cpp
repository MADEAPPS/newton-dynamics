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

#include "vhacdConvexHull.h"

#define VHACD_GOOGOL_SIZE	4
#define VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE 8

class vhacdGoogol
{
	public:
	vhacdGoogol(void);
	vhacdGoogol(double value);

	operator double() const;
	vhacdGoogol operator+ (const vhacdGoogol &A) const;
	vhacdGoogol operator- (const vhacdGoogol &A) const;
	vhacdGoogol operator* (const vhacdGoogol &A) const;
	vhacdGoogol operator/ (const vhacdGoogol &A) const;

	vhacdGoogol operator+= (const vhacdGoogol &A);
	vhacdGoogol operator-= (const vhacdGoogol &A);

	bool operator> (const vhacdGoogol &A) const;
	bool operator>= (const vhacdGoogol &A) const;
	bool operator< (const vhacdGoogol &A) const;
	bool operator<= (const vhacdGoogol &A) const;
	bool operator== (const vhacdGoogol &A) const;
	bool operator!= (const vhacdGoogol &A) const;

	vhacdGoogol Abs() const;
	vhacdGoogol Sqrt() const;
	vhacdGoogol InvSqrt() const;
	vhacdGoogol Floor() const;

	void Trace() const;
	void ToString(char* const string) const;

	private:
	void InitFloatFloat(double value);
	void NegateMantissa(uint64_t* const mantissa) const;
	void CopySignedMantissa(uint64_t* const mantissa) const;
	int NormalizeMantissa(uint64_t* const mantissa) const;
	uint64_t CheckCarrier(uint64_t a, uint64_t b) const;
	void ShiftRightMantissa(uint64_t* const mantissa, int bits) const;

	int LeadingZeros(uint64_t a) const;
	void ExtendeMultiply(uint64_t a, uint64_t b, uint64_t& high, uint64_t& low) const;
	void ScaleMantissa(uint64_t* const out, uint64_t scale) const;

	int m_sign;
	int m_exponent;
	uint64_t m_mantissa[VHACD_GOOGOL_SIZE];

	public:
	static vhacdGoogol m_zero;
	static vhacdGoogol m_one;
	static vhacdGoogol m_two;
	static vhacdGoogol m_three;
	static vhacdGoogol m_half;
};

vhacdGoogol vhacdGoogol::m_zero(0.0);
vhacdGoogol vhacdGoogol::m_one(1.0);
vhacdGoogol vhacdGoogol::m_two(2.0);
vhacdGoogol vhacdGoogol::m_three(3.0);
vhacdGoogol vhacdGoogol::m_half(0.5);

vhacdGoogol::vhacdGoogol(void)
	:m_sign(0)
	, m_exponent(0)
{
	memset(m_mantissa, 0, sizeof(m_mantissa));
}

vhacdGoogol::vhacdGoogol(double value)
	:m_sign(0)
	, m_exponent(0)
{
	int exp;
	double mantissa = fabs(frexp(value, &exp));

	m_exponent = int(exp);
	m_sign = (value >= 0) ? 0 : 1;

	memset(m_mantissa, 0, sizeof(m_mantissa));
	m_mantissa[0] = uint64_t(double(uint64_t(1) << 62) * mantissa);
}

void vhacdGoogol::CopySignedMantissa(uint64_t* const mantissa) const
{
	memcpy(mantissa, m_mantissa, sizeof(m_mantissa));
	if (m_sign)
	{
		NegateMantissa(mantissa);
	}
}

vhacdGoogol::operator double() const
{
	double mantissa = (double(1.0f) / double(uint64_t(1) << 62)) * double(m_mantissa[0]);
	mantissa = ldexp(mantissa, m_exponent) * (m_sign ? double(-1.0f) : double(1.0f));
	return mantissa;
}

vhacdGoogol vhacdGoogol::operator+ (const vhacdGoogol &A) const
{
	vhacdGoogol tmp;
	if (m_mantissa[0] && A.m_mantissa[0])
	{
		uint64_t mantissa0[VHACD_GOOGOL_SIZE];
		uint64_t mantissa1[VHACD_GOOGOL_SIZE];
		uint64_t mantissa[VHACD_GOOGOL_SIZE];

		CopySignedMantissa(mantissa0);
		A.CopySignedMantissa(mantissa1);

		int exponetDiff = m_exponent - A.m_exponent;
		int exponent = m_exponent;
		if (exponetDiff > 0)
		{
			ShiftRightMantissa(mantissa1, exponetDiff);
		}
		else if (exponetDiff < 0)
		{
			exponent = A.m_exponent;
			ShiftRightMantissa(mantissa0, -exponetDiff);
		}

		uint64_t carrier = 0;
		for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--)
		{
			uint64_t m0 = mantissa0[i];
			uint64_t m1 = mantissa1[i];
			mantissa[i] = m0 + m1 + carrier;
			carrier = CheckCarrier(m0, m1) | CheckCarrier(m0 + m1, carrier);
		}

		int sign = 0;
		if (int64_t(mantissa[0]) < 0)
		{
			sign = 1;
			NegateMantissa(mantissa);
		}

		int bits = NormalizeMantissa(mantissa);
		if (bits <= (-64 * VHACD_GOOGOL_SIZE))
		{
			tmp.m_sign = 0;
			tmp.m_exponent = 0;
		}
		else
		{
			tmp.m_sign = sign;
			tmp.m_exponent = int(exponent + bits);
		}

		memcpy(tmp.m_mantissa, mantissa, sizeof(m_mantissa));
	}
	else if (A.m_mantissa[0])
	{
		tmp = A;
	}
	else
	{
		tmp = *this;
	}

	return tmp;
}

vhacdGoogol vhacdGoogol::operator- (const vhacdGoogol &A) const
{
	vhacdGoogol tmp(A);
	tmp.m_sign = !tmp.m_sign;
	return *this + tmp;
}

void vhacdGoogol::ScaleMantissa(uint64_t* const dst, uint64_t scale) const
{
	uint64_t carrier = 0;
	for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--)
	{
		if (m_mantissa[i])
		{
			uint64_t low;
			uint64_t high;
			ExtendeMultiply(scale, m_mantissa[i], high, low);
			uint64_t acc = low + carrier;
			carrier = CheckCarrier(low, carrier);
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

vhacdGoogol vhacdGoogol::operator* (const vhacdGoogol &A) const
{
	if (m_mantissa[0] && A.m_mantissa[0])
	{
		uint64_t mantissaAcc[VHACD_GOOGOL_SIZE * 2];
		memset(mantissaAcc, 0, sizeof(mantissaAcc));
		for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--)
		{
			uint64_t a = m_mantissa[i];
			if (a)
			{
				uint64_t mantissaScale[2 * VHACD_GOOGOL_SIZE];
				memset(mantissaScale, 0, sizeof(mantissaScale));
				A.ScaleMantissa(&mantissaScale[i], a);

				uint64_t carrier = 0;
				for (int j = 0; j < 2 * VHACD_GOOGOL_SIZE; j++)
				{
					const int k = 2 * VHACD_GOOGOL_SIZE - 1 - j;
					uint64_t m0 = mantissaAcc[k];
					uint64_t m1 = mantissaScale[k];
					mantissaAcc[k] = m0 + m1 + carrier;
					carrier = CheckCarrier(m0, m1) | CheckCarrier(m0 + m1, carrier);
				}
			}
		}

		uint64_t carrier = 0;
		//int bits = uint64_t(LeadingZeros (mantissaAcc[0]) - 2);
		int bits = LeadingZeros(mantissaAcc[0]) - 2;
		for (int i = 0; i < 2 * VHACD_GOOGOL_SIZE; i++)
		{
			const int k = 2 * VHACD_GOOGOL_SIZE - 1 - i;
			uint64_t a = mantissaAcc[k];
			mantissaAcc[k] = (a << uint64_t(bits)) | carrier;
			carrier = a >> uint64_t(64 - bits);
		}

		int exp = m_exponent + A.m_exponent - (bits - 2);

		vhacdGoogol tmp;
		tmp.m_sign = m_sign ^ A.m_sign;
		tmp.m_exponent = int(exp);
		memcpy(tmp.m_mantissa, mantissaAcc, sizeof(m_mantissa));

		return tmp;
	}
	return vhacdGoogol(0.0);
}

vhacdGoogol vhacdGoogol::operator/ (const vhacdGoogol &A) const
{
	vhacdGoogol tmp(1.0 / A);
	tmp = tmp * (m_two - A * tmp);
	tmp = tmp * (m_two - A * tmp);
	int test = 0;
	int passes = 0;
	do
	{
		passes++;
		vhacdGoogol tmp0(tmp);
		tmp = tmp * (m_two - A * tmp);
		test = memcmp(&tmp0, &tmp, sizeof(vhacdGoogol));
	} while (test && (passes < (2 * VHACD_GOOGOL_SIZE)));
	return (*this) * tmp;
}

vhacdGoogol vhacdGoogol::Abs() const
{
	vhacdGoogol tmp(*this);
	tmp.m_sign = 0;
	return tmp;
}

vhacdGoogol vhacdGoogol::Floor() const
{
	if (m_exponent < 1)
	{
		return vhacdGoogol(0.0);
	}
	int bits = m_exponent + 2;
	int start = 0;
	while (bits >= 64)
	{
		bits -= 64;
		start++;
	}

	vhacdGoogol tmp(*this);
	for (int i = VHACD_GOOGOL_SIZE - 1; i > start; i--)
	{
		tmp.m_mantissa[i] = 0;
	}
	// some compilers do no like this and I do not know why is that
	//uint64_t mask = (-1LL) << (64 - bits);
	uint64_t mask(~0ULL);
	mask <<= (64 - bits);
	tmp.m_mantissa[start] &= mask;
	return tmp;
}

vhacdGoogol vhacdGoogol::InvSqrt() const
{
	const vhacdGoogol& me = *this;
	vhacdGoogol x(1.0f / sqrt(me));

	int test = 0;
	int passes = 0;
	do
	{
		passes++;
		vhacdGoogol tmp(x);
		x = m_half * x * (m_three - me * x * x);
		test = memcmp(&x, &tmp, sizeof(vhacdGoogol));
	} while (test && (passes < (2 * VHACD_GOOGOL_SIZE)));
	return x;
}

vhacdGoogol vhacdGoogol::Sqrt() const
{
	return *this * InvSqrt();
}

void vhacdGoogol::ToString(char* const string) const
{
	vhacdGoogol tmp(*this);
	vhacdGoogol base(10.0);
	while (double(tmp) > 1.0)
	{
		tmp = tmp / base;
	}

	int index = 0;
	while (tmp.m_mantissa[0])
	{
		tmp = tmp * base;
		vhacdGoogol digit(tmp.Floor());
		tmp -= digit;
		double val = digit;
		string[index] = char(val) + '0';
		index++;
	}
	string[index] = 0;
}

void vhacdGoogol::NegateMantissa(uint64_t* const mantissa) const
{
	uint64_t carrier = 1;
	for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--)
	{
		uint64_t a = ~mantissa[i] + carrier;
		if (a)
		{
			carrier = 0;
		}
		mantissa[i] = a;
	}
}

void vhacdGoogol::ShiftRightMantissa(uint64_t* const mantissa, int bits) const
{
	uint64_t carrier = 0;
	if (int64_t(mantissa[0]) < int64_t(0))
	{
		carrier = uint64_t(-1);
	}

	while (bits >= 64)
	{
		for (int i = VHACD_GOOGOL_SIZE - 2; i >= 0; i--)
		{
			mantissa[i + 1] = mantissa[i];
		}
		mantissa[0] = carrier;
		bits -= 64;
	}

	if (bits > 0)
	{
		carrier <<= (64 - bits);
		for (int i = 0; i < VHACD_GOOGOL_SIZE; i++)
		{
			uint64_t a = mantissa[i];
			mantissa[i] = (a >> bits) | carrier;
			carrier = a << (64 - bits);
		}
	}
}

int vhacdGoogol::LeadingZeros(uint64_t a) const
{
#define dgCOUNTBIT(mask,add)		\
	{									\
		uint64_t test = a & mask;	\
		n += test ? 0 : add;			\
		a = test ? test : (a & ~mask);	\
	}

	int n = 0;
	dgCOUNTBIT(0xffffffff00000000LL, 32);
	dgCOUNTBIT(0xffff0000ffff0000LL, 16);
	dgCOUNTBIT(0xff00ff00ff00ff00LL, 8);
	dgCOUNTBIT(0xf0f0f0f0f0f0f0f0LL, 4);
	dgCOUNTBIT(0xccccccccccccccccLL, 2);
	dgCOUNTBIT(0xaaaaaaaaaaaaaaaaLL, 1);

	return n;
}

int vhacdGoogol::NormalizeMantissa(uint64_t* const mantissa) const
{
	int bits = 0;
	if (int64_t(mantissa[0] * 2) < 0)
	{
		bits = 1;
		ShiftRightMantissa(mantissa, 1);
	}
	else
	{
		while (!mantissa[0] && bits > (-64 * VHACD_GOOGOL_SIZE))
		{
			bits -= 64;
			for (int i = 1; i < VHACD_GOOGOL_SIZE; i++) {
				mantissa[i - 1] = mantissa[i];
			}
			mantissa[VHACD_GOOGOL_SIZE - 1] = 0;
		}

		if (bits > (-64 * VHACD_GOOGOL_SIZE))
		{
			int n = LeadingZeros(mantissa[0]) - 2;
			if (n > 0) 
			{
				uint64_t carrier = 0;
				for (int i = VHACD_GOOGOL_SIZE - 1; i >= 0; i--) 
				{
					uint64_t a = mantissa[i];
					mantissa[i] = (a << n) | carrier;
					carrier = a >> (64 - n);
				}
				bits -= n;
			}
			else if (n < 0)
			{
				// this is very rare but it does happens, whee the leading zeros of the mantissa is an exact multiple of 64
				uint64_t carrier = 0;
				int shift = -n;
				for (int i = 0; i < VHACD_GOOGOL_SIZE; i++)
				{
					uint64_t a = mantissa[i];
					mantissa[i] = (a >> shift) | carrier;
					carrier = a << (64 - shift);
				}
				bits -= n;
			}
		}
	}
	return bits;
}

uint64_t vhacdGoogol::CheckCarrier(uint64_t a, uint64_t b) const
{
	return ((uint64_t(-1) - b) < a) ? uint64_t(1) : 0;
}

void vhacdGoogol::ExtendeMultiply(uint64_t a, uint64_t b, uint64_t& high, uint64_t& low) const
{
	uint64_t bLow = b & 0xffffffff;
	uint64_t bHigh = b >> 32;
	uint64_t aLow = a & 0xffffffff;
	uint64_t aHigh = a >> 32;

	uint64_t l = bLow * aLow;

	uint64_t c1 = bHigh * aLow;
	uint64_t c2 = bLow * aHigh;
	uint64_t m = c1 + c2;
	uint64_t carrier = CheckCarrier(c1, c2) << 32;

	uint64_t h = bHigh * aHigh + carrier;

	uint64_t ml = m << 32;
	uint64_t ll = l + ml;
	uint64_t mh = (m >> 32) + CheckCarrier(l, ml);
	uint64_t hh = h + mh;

	low = ll;
	high = hh;
}

vhacdGoogol vhacdGoogol::operator+= (const vhacdGoogol &A)
{
	*this = *this + A;
	return *this;
}

vhacdGoogol vhacdGoogol::operator-= (const vhacdGoogol &A)
{
	*this = *this - A;
	return *this;
}

bool vhacdGoogol::operator> (const vhacdGoogol &A) const
{
	vhacdGoogol tmp(*this - A);
	return double(tmp) > 0.0;
}

bool vhacdGoogol::operator>= (const vhacdGoogol &A) const
{
	vhacdGoogol tmp(*this - A);
	return double(tmp) >= 0.0;
}

bool vhacdGoogol::operator< (const vhacdGoogol &A) const
{
	vhacdGoogol tmp(*this - A);
	return double(tmp) < 0.0;
}

bool vhacdGoogol::operator<= (const vhacdGoogol &A) const
{
	vhacdGoogol tmp(*this - A);
	return double(tmp) <= 0.0;
}

bool vhacdGoogol::operator== (const vhacdGoogol &A) const
{
	vhacdGoogol tmp(*this - A);
	return double(tmp) == 0.0;
}

bool vhacdGoogol::operator!= (const vhacdGoogol &A) const
{
	vhacdGoogol tmp(*this - A);
	return double(tmp) != 0.0;
}

void vhacdGoogol::Trace() const
{
	//dTrace (("%f ", double (*this)));
}

inline int dExp2(int x)
{
	int exp;
	for (exp = -1; x; x >>= 1)
	{
		exp++;
	}
	return exp;
}

inline int dBitReversal(int v, int base)
{
	int x = 0;
	int power = dExp2(base) - 1;
	do
	{
		x += (v & 1) << power;
		v >>= 1;
		power--;
	} while (v);
	return x;
}

template <class T>
inline void vhacdSwap(T& A, T& B)
{
	T tmp(A);
	A = B;
	B = tmp;
}

template <class T, class dCompareKey>
void vhacdSort(T* const array, int elements, void* const context = nullptr)
{
	const int batchSize = 8;
	int stack[1024][2];

	stack[0][0] = 0;
	stack[0][1] = elements - 1;
	int stackIndex = 1;
	const dCompareKey comparator;
	while (stackIndex)
	{
		stackIndex--;
		int lo = stack[stackIndex][0];
		int hi = stack[stackIndex][1];
		if ((hi - lo) > batchSize)
		{
			int mid = (lo + hi) >> 1;
			if (comparator.Compare(array[lo], array[mid], context) > 0)
			{
				vhacdSwap(array[lo], array[mid]);
			}
			if (comparator.Compare(array[mid], array[hi], context) > 0)
			{
				vhacdSwap(array[mid], array[hi]);
			}
			if (comparator.Compare(array[lo], array[mid], context) > 0)
			{
				vhacdSwap(array[lo], array[mid]);
			}
			int i = lo + 1;
			int j = hi - 1;
			const T pivot(array[mid]);
			do
			{
				while (comparator.Compare(array[i], pivot, context) < 0)
				{
					i++;
				}
				while (comparator.Compare(array[j], pivot, context) > 0)
				{
					j--;
				}

				if (i <= j)
				{
					vhacdSwap(array[i], array[j]);
					i++;
					j--;
				}
			} while (i <= j);

			if (i < hi)
			{
				stack[stackIndex][0] = i;
				stack[stackIndex][1] = hi;
				stackIndex++;
			}
			if (lo < j)
			{
				stack[stackIndex][0] = lo;
				stack[stackIndex][1] = j;
				stackIndex++;
			}
			_ASSERT(stackIndex < int(sizeof(stack) / (2 * sizeof(stack[0][0]))));
		}
	}

	int stride = batchSize + 1;
	if (elements < stride)
	{
		stride = elements;
	}
	for (int i = 1; i < stride; ++i)
	{
		if (comparator.Compare(array[0], array[i], context) > 0)
		{
			vhacdSwap(array[0], array[i]);
		}
	}

	for (int i = 1; i < elements; ++i)
	{
		int j = i;
		const T tmp(array[i]);
		for (; comparator.Compare(array[j - 1], tmp, context) > 0; --j)
		{
			_ASSERT(j > 0);
			array[j] = array[j - 1];
		}
		array[j] = tmp;
	}

	#if 0
	for (int i = 0; i < (elements - 1); ++i)
	{
		_ASSERT(comparator.Compare(array[i], array[i + 1], context) <= 0);
	}
	#endif
}

#define Absolute(a)  ((a) >= 0.0 ? (a) : -(a))

static double Determinant2x2(const double matrix[2][2], double* const error)
{
	double a00xa11 = matrix[0][0] * matrix[1][1];
	double a01xa10 = matrix[0][1] * matrix[1][0];
	*error = Absolute(a00xa11) + Absolute(a01xa10);
	return a00xa11 - a01xa10;
}

static double Determinant3x3 (const double matrix[3][3], double* const error)
{
	double sign = double (-1.0f);
	double det = double (0.0f);
	double accError = double (0.0f); 
	for (int i = 0; i < 3; i ++)  
	{
		double cofactor[2][2];
		for (int j = 0; j < 2; j ++) 
		{
			int k0 = 0;
			for (int k = 0; k < 3; k ++) 
			{
				if (k != i) 
				{
					cofactor[j][k0] = matrix[j][k];
					k0 ++;
				}
			}
		}

		double parcialError;
		double minorDet = Determinant2x2 (cofactor, &parcialError);
		accError += parcialError * Absolute (matrix[2][i]);
		det += sign * minorDet * matrix[2][i];
		sign *= double (-1.0f);
	}

	*error = accError;
	return det;
}

static vhacdGoogol Determinant2x2(const vhacdGoogol matrix[2][2])
{
	vhacdGoogol a00xa11(matrix[0][0] * matrix[1][1]);
	vhacdGoogol a01xa10(matrix[0][1] * matrix[1][0]);
	return a00xa11 - a01xa10;
}


static vhacdGoogol Determinant3x3(const vhacdGoogol matrix[3][3])
{
	vhacdGoogol negOne(double(-1.0f));
	vhacdGoogol sign(double(-1.0f));
	vhacdGoogol det = double(0.0f);
	for (int i = 0; i < 3; i++)
	{
		vhacdGoogol cofactor[2][2];
		for (int j = 0; j < 2; j++)
		{
			int k0 = 0;
			for (int k = 0; k < 3; k++)
			{
				if (k != i)
				{
					cofactor[j][k0] = matrix[j][k];
					k0++;
				}
			}
		}

		vhacdGoogol minorDet(Determinant2x2(cofactor));
		det = det + sign * minorDet * matrix[2][i];
		sign = sign * negOne;
	}
	return det;
}



vhacdConvexHullFace::vhacdConvexHullFace()
{
	m_mark = 0;
	m_twin[0] = nullptr;
	m_twin[1] = nullptr;
	m_twin[2] = nullptr;
}

hullPlane vhacdConvexHullFace::GetPlaneEquation(const hullVector* const pointArray) const
{
	const hullVector& p0 = pointArray[m_index[0]];
	const hullVector& p1 = pointArray[m_index[1]];
	const hullVector& p2 = pointArray[m_index[2]];
	hullPlane plane(p0, p1, p2);
	//plane = plane.Scale(1.0f / sqrt(plane.DotProduct(plane & hullVector::m_triplexMask).GetScalar()));
	plane = plane.Scale(1.0f / sqrt(plane.DotProduct(plane)));
	return plane;
}

double vhacdConvexHullFace::Evalue(const hullVector* const pointArray, const hullVector& point) const
{
	const hullVector& p0 = pointArray[m_index[0]];
	const hullVector& p1 = pointArray[m_index[1]];
	const hullVector& p2 = pointArray[m_index[2]];

	double matrix[3][3];
	for (int i = 0; i < 3; i++) 
	{
		matrix[0][i] = p2[i] - p0[i];
		matrix[1][i] = p1[i] - p0[i];
		matrix[2][i] = point[i] - p0[i];
	}

	double error;
	double det = Determinant3x3(matrix, &error);

	// the code use double, however the threshold for accuracy test is the machine precision of a float.
	// by changing this to a smaller number, the code should run faster since many small test will be considered valid
	// the precision must be a power of two no smaller than the machine precision of a double, (1<<48)
	// float64(1<<30) can be a good value

	// double precision	= double (1.0f) / double (1<<30);
	double precision = double(1.0f) / double(1 << 24);
	double errbound = error * precision;
	if (fabs(det) > errbound) 
	{
		return det;
	}
	
	vhacdGoogol exactMatrix[3][3];
	for (int i = 0; i < 3; i++) 
	{
		exactMatrix[0][i] = vhacdGoogol(p2[i]) - vhacdGoogol(p0[i]);
		exactMatrix[1][i] = vhacdGoogol(p1[i]) - vhacdGoogol(p0[i]);
		exactMatrix[2][i] = vhacdGoogol(point[i]) - vhacdGoogol(p0[i]);
	}
	return Determinant3x3(exactMatrix);
}

#if 0
vhacdConvexHull::vhacdConvexHull ()
	:ndList<vhacdConvexHullFace>()
	,m_aabbP0(hullVector (double (0.0f)))
	,m_aabbP1(hullVector (double (0.0f)))
	,m_count(0)
	,m_diag()
	,m_points()
{
}

vhacdConvexHull::vhacdConvexHull(const vhacdConvexHull& source)
	:ndList<vhacdConvexHullFace>()
	,m_aabbP0 (source.m_aabbP0)
	,m_aabbP1 (source.m_aabbP1)
	,m_count(source.m_count)
	,m_diag(source.m_diag)
	,m_points(source.m_count)
{
	m_points.SetCount(source.m_count);
	m_points[m_count-1].m_w = double (0.0f);
	for (int i = 0; i < m_count; i ++) 
	{
		m_points[i] = source.m_points[i];
	}
	ndTree<ndNode*, ndNode*> map;
	for(ndNode* sourceNode = source.GetFirst(); sourceNode; sourceNode = sourceNode->GetNext() ) 
	{
		ndNode* const node = Append();
		map.Insert(node, sourceNode);
	}

	for(ndNode* sourceNode = source.GetFirst(); sourceNode; sourceNode = sourceNode->GetNext() ) {
		ndNode* const node = map.Find(sourceNode)->GetInfo();

		vhacdConvexHullFace& face = node->GetInfo();
		vhacdConvexHullFace& srcFace = sourceNode->GetInfo();

		face.m_mark = 0;
		for (int i = 0; i < 3; i ++) {
			face.m_index[i] = srcFace.m_index[i];
			face.m_twin[i] = map.Find (srcFace.m_twin[i])->GetInfo();
		}
	}
}


vhacdConvexHull::~vhacdConvexHull(void)
{
}


void vhacdConvexHull::DeleteFace (ndNode* const node)
{
	Remove (node);
}

bool vhacdConvexHull::Sanity() const
{
/*
	for (ndNode* node = GetFirst(); node; node = node->GetNext()) {
		vhacdConvexHullFace* const face = &node->GetInfo();
		for (int i = 0; i < 3; i ++) {
			ndNode* const twinNode = face->m_twin[i];
			if (!twinNode) {
				return false;
			}

			int count = 0;
			ndNode* me = nullptr;
			vhacdConvexHullFace* const twinFace = &twinNode->GetInfo();
			for (int j = 0; j < 3; j ++) {
				if (twinFace->m_twin[j] == node) {
					count ++;
					me = twinFace->m_twin[j];
				}
			}
			if (count != 1) {
				return false;
			}
			if (me != node) {
				return false;
			}
		}
	}
*/
	return true;
}

bool vhacdConvexHull::CheckFlatSurface(vhacdConvexHullAABBTreeNode* tree, vhacdConvexHullVertex* const points, int, double, int)
{
	hullVector e0(m_points[1] - m_points[0]);
	hullVector e1(m_points[2] - m_points[0]);
	_ASSERT(e0.m_w == double(0.0f));
	_ASSERT(e1.m_w == double(0.0f));
	_ASSERT(e0.DotProduct(e0).GetScalar() > double(1.0e-4f));
	_ASSERT(e1.DotProduct(e1).GetScalar() > double(1.0e-4f));
	hullVector normal(e1.CrossProduct(e0));
	_ASSERT(normal.m_w == double(0.0f));
	_ASSERT(normal.DotProduct(normal).GetScalar() > double(1.0e-6f));
	normal = normal.Normalize();

	int index = SupportVertex(&tree, points, normal);
	m_points[3] = points[index];

	double volume = TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]);
	if (fabs(volume) < double(1.0e-9f)) {
		normal = normal.Scale(double(-1.0f));
		index = SupportVertex(&tree, points, normal);
		m_points[3] = points[index];
		volume = TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]);
		if (fabs(volume) < double(1.0e-9f)) {
			return true;
		}
	}
	points[index].m_mark = 1;
	if (volume > double(0.0f)) {
		vhacdSwap(m_points[2], m_points[3]);
	}
	_ASSERT(TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]) < double(0.0f));
	m_count = 4;
	return false;
}


void vhacdConvexHull::CalculateConvexHull2d(vhacdConvexHullAABBTreeNode*, vhacdConvexHullVertex* const, int, double, int)
{

}


void vhacdConvexHull::CalculateVolumeAndSurfaceArea (double& volume, double& surfaceArea) const
{
	double areaAcc = double (0.0f);
	double  volumeAcc = double (0.0f);
	for (ndNode* node = GetFirst(); node; node = node->GetNext()) {
		const vhacdConvexHullFace* const face = &node->GetInfo();
		int i0 = face->m_index[0];
		int i1 = face->m_index[1];
		int i2 = face->m_index[2];
		const hullVector& p0 = m_points[i0];
		const hullVector& p1 = m_points[i1];
		const hullVector& p2 = m_points[i2];
		_ASSERT(p0.m_w == double(0.0f));
		_ASSERT(p1.m_w == double(0.0f));
		_ASSERT(p2.m_w == double(0.0f));
		hullVector normal ((p1 - p0).CrossProduct(p2 - p0));
		_ASSERT(normal.m_w == double(0.0f));
		double area = sqrt (normal.DotProduct(normal).GetScalar());
		areaAcc += area;
		volumeAcc += p2.DotProduct(p0.CrossProduct(p1)).GetScalar();
	}
	_ASSERT (volumeAcc >= double (0.0f));
	volume = volumeAcc * double (1.0f/6.0f);
	surfaceArea = areaAcc * double (0.5f);
}

// this code has linear time complexity on the number of faces
double vhacdConvexHull::RayCast (const hullVector& localP0, const hullVector& localP1) const
{
	double interset = double (1.2f);
	double tE = double (0.0f);	// for the maximum entering segment parameter;
	double tL = double (1.0f);	// for the minimum leaving segment parameter;
	hullVector dS (localP1 - localP0); // is the segment direction vector;

	_ASSERT(dS.m_w == double(0.0f));
	int hasHit = 0;

	for (ndNode* node = GetFirst(); node; node = node->GetNext()) {
		const vhacdConvexHullFace* const face = &node->GetInfo();

		int i0 = face->m_index[0];
		int i1 = face->m_index[1];
		int i2 = face->m_index[2];

		const hullVector& p0 = m_points[i0];
		_ASSERT(p0.m_w == double(0.0f));
		hullVector normal ((m_points[i1] - p0).CrossProduct(m_points[i2] - p0));

		_ASSERT(normal.m_w == double(0.0f));
		_ASSERT(localP0.m_w == double(0.0f));

		//double N = -((localP0 - p0) % normal);
		double D =  normal.DotProduct(dS).GetScalar();
		double N = -normal.DotProduct(localP0 - p0).GetScalar();

		if (fabs(D) < double (1.0e-12f)) { //
			if (N < double (0.0f)) {
				return double (1.2f);
			} else {
				continue;
			}
		}

		double t = N / D;
		if (D < double (0.0f)) {
			if (t > tE) {
				tE = t;
				hasHit = 1;
			}
			if (tE > tL) {
				return double (1.2f);
			}
		} else {
			_ASSERT (D >= double (0.0f));
			tL = dMin (tL, t);
			if (tL < tE) {
				return double (1.2f);
			}
		}
	}

	if (hasHit) {
		interset = tE;
	}

	return interset;
}

void vhacdConvexHull::Save (const char* const filename) const
{
	FILE* const file = fopen(filename, "wb");
	int index = 0;
//	fprintf(file, "final\n");
	for (ndNode* nodePtr = GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
		fprintf(file, "triangle %d\n", index);
		index++;
		const vhacdConvexHullFace& face = nodePtr->GetInfo();
		const hullVector& p0 = m_points[face.m_index[0]];
		const hullVector& p1 = m_points[face.m_index[1]];
		const hullVector& p2 = m_points[face.m_index[2]];

		fprintf(file, "p0(%f %f %f)\n", p0[0], p0[1], p0[2]);
		fprintf(file, "p1(%f %f %f)\n", p1[0], p1[1], p1[2]);
		fprintf(file, "p2(%f %f %f)\n", p2[0], p2[1], p2[2]);
	}
	fprintf(file, "\n");

	fclose(file);
}
#endif

class vhacdConvexHullVertex : public hullVector
{
	public:
	int m_mark;
};

class vhacdConvexHullAABBTreeNode
{
	public:
	hullVector m_box[2];
	vhacdConvexHullAABBTreeNode* m_left;
	vhacdConvexHullAABBTreeNode* m_right;
	vhacdConvexHullAABBTreeNode* m_parent;
};

class vhacdConvexHull3dPointCluster : public vhacdConvexHullAABBTreeNode
{
	public:
	int m_count;
	int m_indices[VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE];
};


class vhacdConvexHull::ndNormalMap
{
	public:
	ndNormalMap()
		:m_count(sizeof(m_normal) / sizeof(m_normal[0]))
	{
		hullVector p0(double(1.0f), double(0.0f), double(0.0f), double(0.0f));
		hullVector p1(double(-1.0f), double(0.0f), double(0.0f), double(0.0f));
		hullVector p2(double(0.0f), double(1.0f), double(0.0f), double(0.0f));
		hullVector p3(double(0.0f), double(-1.0f), double(0.0f), double(0.0f));
		hullVector p4(double(0.0f), double(0.0f), double(1.0f), double(0.0f));
		hullVector p5(double(0.0f), double(0.0f), double(-1.0f), double(0.0f));

		int count = 0;
		int subdivitions = 2;
		TessellateTriangle(subdivitions, p4, p0, p2, count);
		TessellateTriangle(subdivitions, p0, p5, p2, count);
		TessellateTriangle(subdivitions, p5, p1, p2, count);
		TessellateTriangle(subdivitions, p1, p4, p2, count);
		TessellateTriangle(subdivitions, p0, p4, p3, count);
		TessellateTriangle(subdivitions, p5, p0, p3, count);
		TessellateTriangle(subdivitions, p1, p5, p3, count);
		TessellateTriangle(subdivitions, p4, p1, p3, count);
	}

	static const ndNormalMap& GetNormaMap()
	{
		static ndNormalMap normalMap;
		return normalMap;
	}

	void TessellateTriangle(int level, const hullVector& p0, const hullVector& p1, const hullVector& p2, int& count)
	{
		if (level) 
		{
			_ASSERT(fabs(p0.DotProduct(p0) - double(1.0f)) < double(1.0e-4f));
			_ASSERT(fabs(p1.DotProduct(p1) - double(1.0f)) < double(1.0e-4f));
			_ASSERT(fabs(p2.DotProduct(p2) - double(1.0f)) < double(1.0e-4f));
			hullVector p01(p0 + p1);
			hullVector p12(p1 + p2);
			hullVector p20(p2 + p0);

			p01 = p01.Scale(1.0 / sqrt(p01.DotProduct(p01)));
			p12 = p12.Scale(1.0 / sqrt(p12.DotProduct(p12)));
			p20 = p20.Scale(1.0 / sqrt(p20.DotProduct(p20)));

			_ASSERT(fabs(p01.DotProduct(p01) - double(1.0f)) < double(1.0e-4f));
			_ASSERT(fabs(p12.DotProduct(p12) - double(1.0f)) < double(1.0e-4f));
			_ASSERT(fabs(p20.DotProduct(p20) - double(1.0f)) < double(1.0e-4f));

			TessellateTriangle(level - 1, p0, p01, p20, count);
			TessellateTriangle(level - 1, p1, p12, p01, count);
			TessellateTriangle(level - 1, p2, p20, p12, count);
			TessellateTriangle(level - 1, p01, p12, p20, count);
		}
		else 
		{
			hullPlane n(p0, p1, p2);
			n = n.Scale(double(1.0f) / sqrt(n.DotProduct(n)));
			n.m_w = double(0.0f);
			int index = dBitReversal(count, sizeof(m_normal) / sizeof(m_normal[0]));
			m_normal[index] = n;
			count++;
			_ASSERT(count <= int(sizeof(m_normal) / sizeof(m_normal[0])));
		}
	}

	hullVector m_normal[128];
	int m_count;
};



vhacdConvexHull::vhacdConvexHull(const double* const vertexCloud, int strideInBytes, int count, double distTol, int maxVertexCount)
	:vhacdList<vhacdConvexHullFace>()
	,m_aabbP0(0)
	,m_aabbP1(0)
	,m_diag()
	,m_points()
{
	BuildHull(vertexCloud, strideInBytes, count, distTol, maxVertexCount);
}

void vhacdConvexHull::BuildHull(const double* const vertexCloud, int strideInBytes, int count, double distTol, int maxVertexCount)
{
	int treeCount = count / (VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE >> 1);
	if (treeCount < 4)
	{
		treeCount = 4;
	}
	treeCount *= 2;

	std::vector<vhacdConvexHullVertex> points(count);
	std::vector<vhacdConvexHull3dPointCluster> treePool(treeCount + 256);
	points.resize(count);
	treePool.resize(treeCount + 256);
	count = InitVertexArray(&points[0], vertexCloud, strideInBytes, count, &treePool[0], sizeof (vhacdConvexHull3dPointCluster) * int (treePool.size()));

	if (m_points.size() >= 4)
	{
		CalculateConvexHull3d(&treePool[0], &points[0], count, distTol, maxVertexCount);
	}
}

int vhacdConvexHull::GetUniquePoints(vhacdConvexHullVertex* const points, const double* const vertexCloud, int strideInBytes, int count, void* const, int)
{
	const int stride = int(strideInBytes / sizeof(double));
	for (int i = 0; i < count; i++)
	{
		int index = i * stride;
		hullVector& vertex = points[i];
		vertex = hullVector(vertexCloud[index], vertexCloud[index + 1], vertexCloud[index + 2], double(0.0f));
		points[i].m_mark = 0;
	}

	class CompareVertex
	{
		public:
		int Compare(const vhacdConvexHullVertex& elementA, const vhacdConvexHullVertex& elementB, void* const) const
		{
			for (int i = 0; i < 3; i++)
			{
				if (elementA[i] < elementB[i])
				{
					return -1;
				}
				else if (elementA[i] > elementB[i])
				{
					return 1;
				}
			}
			return 0;
		}
	};
	vhacdSort<vhacdConvexHullVertex, CompareVertex>(points, count);

	int indexCount = 0;
	CompareVertex compareVetex;
	for (int i = 1; i < count; i++)
	{
		for (; i < count; i++)
		{
			if (compareVetex.Compare(points[indexCount], points[i], nullptr))
			{
				indexCount++;
				points[indexCount] = points[i];
				break;
			}
		}
	}
	count = indexCount + 1;
	return count;
}

vhacdConvexHullAABBTreeNode* vhacdConvexHull::BuildTree(vhacdConvexHullAABBTreeNode* const parent, vhacdConvexHullVertex* const points, int count, int baseIndex, char** memoryPool, int& maxMemSize) const
{
	vhacdConvexHullAABBTreeNode* tree = nullptr;

	_ASSERT(count);
	hullVector minP(double(1.0e15f));
	hullVector maxP(-double(1.0e15f));
	if (count <= VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE)
	{
		vhacdConvexHull3dPointCluster* const clump = new (*memoryPool) vhacdConvexHull3dPointCluster;
		*memoryPool += sizeof(vhacdConvexHull3dPointCluster);
		maxMemSize -= sizeof(vhacdConvexHull3dPointCluster);
		_ASSERT(maxMemSize >= 0);

		_ASSERT(clump);
		clump->m_count = count;
		for (int i = 0; i < count; i++)
		{
			clump->m_indices[i] = i + baseIndex;

			const hullVector& p = points[i];
			minP = minP.GetMin(p);
			maxP = maxP.GetMax(p);
		}

		clump->m_left = nullptr;
		clump->m_right = nullptr;
		tree = clump;

	}
	else
	{
		hullVector median(0);
		hullVector varian(0);
		for (int i = 0; i < count; i++)
		{
			const hullVector& p = points[i];
			minP = minP.GetMin(p);
			maxP = maxP.GetMax(p);
			median += p;
			varian += p * p;
		}

		varian = varian.Scale(double(count)) - median * median;
		int index = 0;
		double maxVarian = double(-1.0e10f);
		for (int i = 0; i < 3; i++)
		{
			if (varian[i] > maxVarian)
			{
				index = i;
				maxVarian = varian[i];
			}
		}
		hullVector center(median.Scale(double(1.0f) / double(count)));

		double test = center[index];

		int i0 = 0;
		int i1 = count - 1;
		do
		{
			for (; i0 <= i1; i0++)
			{
				double val = points[i0][index];
				if (val > test)
				{
					break;
				}
			}

			for (; i1 >= i0; i1--)
			{
				double val = points[i1][index];
				if (val < test)
				{
					break;
				}
			}

			if (i0 < i1)
			{
				vhacdSwap(points[i0], points[i1]);
				i0++;
				i1--;
			}
		} while (i0 <= i1);

		if (i0 == 0)
		{
			i0 = count / 2;
		}
		if (i0 >= (count - 1))
		{
			i0 = count / 2;
		}

		tree = new (*memoryPool) vhacdConvexHullAABBTreeNode;
		*memoryPool += sizeof(vhacdConvexHullAABBTreeNode);
		maxMemSize -= sizeof(vhacdConvexHullAABBTreeNode);
		_ASSERT(maxMemSize >= 0);

		_ASSERT(i0);
		_ASSERT(count - i0);

		tree->m_left = BuildTree(tree, points, i0, baseIndex, memoryPool, maxMemSize);
		tree->m_right = BuildTree(tree, &points[i0], count - i0, i0 + baseIndex, memoryPool, maxMemSize);
	}

	_ASSERT(tree);
	tree->m_parent = parent;
	tree->m_box[0] = minP - hullVector(double(1.0e-3f));
	tree->m_box[1] = maxP + hullVector(double(1.0e-3f));
	return tree;
}

int vhacdConvexHull::SupportVertex(vhacdConvexHullAABBTreeNode** const treePointer, const vhacdConvexHullVertex* const points, const hullVector& dirPlane, const bool removeEntry) const
{
#define DG_STACK_DEPTH_3D 64
	double aabbProjection[DG_STACK_DEPTH_3D];
	const vhacdConvexHullAABBTreeNode *stackPool[DG_STACK_DEPTH_3D];

	hullVector dir(dirPlane);

	int index = -1;
	int stack = 1;
	stackPool[0] = *treePointer;
	aabbProjection[0] = double(1.0e20f);
	double maxProj = double(-1.0e20f);
	int ix = (dir[0] > double(0.0f)) ? 1 : 0;
	int iy = (dir[1] > double(0.0f)) ? 1 : 0;
	int iz = (dir[2] > double(0.0f)) ? 1 : 0;
	while (stack)
	{
		stack--;
		double boxSupportValue = aabbProjection[stack];
		if (boxSupportValue > maxProj)
		{
			const vhacdConvexHullAABBTreeNode* const me = stackPool[stack];

			if (me->m_left && me->m_right)
			{
				const hullVector leftSupportPoint(me->m_left->m_box[ix].X(), me->m_left->m_box[iy].Y(), me->m_left->m_box[iz].Z(), 0.0f);
				double leftSupportDist = leftSupportPoint.DotProduct(dir);

				const hullVector rightSupportPoint(me->m_right->m_box[ix].X(), me->m_right->m_box[iy].Y(), me->m_right->m_box[iz].Z(), 0.0f);
				double rightSupportDist = rightSupportPoint.DotProduct(dir);

				if (rightSupportDist >= leftSupportDist)
				{
					aabbProjection[stack] = leftSupportDist;
					stackPool[stack] = me->m_left;
					stack++;
					_ASSERT(stack < DG_STACK_DEPTH_3D);
					aabbProjection[stack] = rightSupportDist;
					stackPool[stack] = me->m_right;
					stack++;
					_ASSERT(stack < DG_STACK_DEPTH_3D);
				}
				else
				{
					aabbProjection[stack] = rightSupportDist;
					stackPool[stack] = me->m_right;
					stack++;
					_ASSERT(stack < DG_STACK_DEPTH_3D);
					aabbProjection[stack] = leftSupportDist;
					stackPool[stack] = me->m_left;
					stack++;
					_ASSERT(stack < DG_STACK_DEPTH_3D);
				}
			}
			else
			{
				vhacdConvexHull3dPointCluster* const cluster = (vhacdConvexHull3dPointCluster*)me;
				for (int i = 0; i < cluster->m_count; i++)
				{
					const vhacdConvexHullVertex& p = points[cluster->m_indices[i]];
					_ASSERT(p.X() >= cluster->m_box[0].X());
					_ASSERT(p.X() <= cluster->m_box[1].X());
					_ASSERT(p.Y() >= cluster->m_box[0].Y());
					_ASSERT(p.Y() <= cluster->m_box[1].Y());
					_ASSERT(p.Z() >= cluster->m_box[0].Z());
					_ASSERT(p.Z() <= cluster->m_box[1].Z());
					if (!p.m_mark)
					{
						//_ASSERT(p.m_w == double(0.0f));
						double dist = p.DotProduct(dir);
						if (dist > maxProj)
						{
							maxProj = dist;
							index = cluster->m_indices[i];
						}
					}
					else if (removeEntry)
					{
						cluster->m_indices[i] = cluster->m_indices[cluster->m_count - 1];
						cluster->m_count = cluster->m_count - 1;
						i--;
					}
				}

				if (cluster->m_count == 0)
				{
					vhacdConvexHullAABBTreeNode* const parent = cluster->m_parent;
					if (parent)
					{
						vhacdConvexHullAABBTreeNode* const sibling = (parent->m_left != cluster) ? parent->m_left : parent->m_right;
						_ASSERT(sibling != cluster);
						vhacdConvexHullAABBTreeNode* const grandParent = parent->m_parent;
						if (grandParent)
						{
							sibling->m_parent = grandParent;
							if (grandParent->m_right == parent)
							{
								grandParent->m_right = sibling;
							}
							else
							{
								grandParent->m_left = sibling;
							}
						}
						else
						{
							sibling->m_parent = nullptr;
							*treePointer = sibling;
						}
					}
				}
			}
		}
	}

	_ASSERT(index != -1);
	return index;
}

double vhacdConvexHull::TetrahedrumVolume(const hullVector& p0, const hullVector& p1, const hullVector& p2, const hullVector& p3) const
{
	const hullVector p1p0(p1 - p0);
	const hullVector p2p0(p2 - p0);
	const hullVector p3p0(p3 - p0);
	return p3p0.DotProduct(p1p0.CrossProduct(p2p0));
}

int vhacdConvexHull::InitVertexArray(vhacdConvexHullVertex* const points, const double* const vertexCloud, int strideInBytes, int count, void* const memoryPool, int maxMemSize)
{
	count = GetUniquePoints(points, vertexCloud, strideInBytes, count, memoryPool, maxMemSize);
	if (count < 4)
	{
		m_points.resize(0);
		return count;
	}
	vhacdConvexHullAABBTreeNode* tree = BuildTree(nullptr, points, count, 0, (char**)&memoryPool, maxMemSize);
	
	m_points.resize(count);
	m_aabbP0 = tree->m_box[0];
	m_aabbP1 = tree->m_box[1];
	
	hullVector boxSize(tree->m_box[1] - tree->m_box[0]);
	m_diag = double(sqrt(boxSize.DotProduct(boxSize)));
	const ndNormalMap& normalMap = ndNormalMap::GetNormaMap();
	
	int index0 = SupportVertex(&tree, points, normalMap.m_normal[0]);
	m_points[0] = points[index0];
	points[index0].m_mark = 1;
	
	bool validTetrahedrum = false;
	hullVector e1(0.0);
	for (int i = 1; i < normalMap.m_count; i++)
	{
		int index = SupportVertex(&tree, points, normalMap.m_normal[i]);
		_ASSERT(index >= 0);
	
		e1 = points[index] - m_points[0];
		double error2 = e1.DotProduct(e1);
		if (error2 > (double(1.0e-4f) * m_diag * m_diag))
		{
			m_points[1] = points[index];
			points[index].m_mark = 1;
			validTetrahedrum = true;
			break;
		}
	}
	if (!validTetrahedrum)
	{
		m_points.resize(0);
		_ASSERT(0);
		return count;
	}
	
	validTetrahedrum = false;
	hullVector e2(0.0);
	hullVector normal(0.0);
	for (int i = 2; i < normalMap.m_count; i++)
	{
		int index = SupportVertex(&tree, points, normalMap.m_normal[i]);
		_ASSERT(index >= 0);
		e2 = points[index] - m_points[0];
		normal = e1.CrossProduct(e2);
		double error2 = sqrt(normal.DotProduct(normal));
		if (error2 > (double(1.0e-4f) * m_diag * m_diag))
		{
			m_points[2] = points[index];
			points[index].m_mark = 1;
			validTetrahedrum = true;
			break;
		}
	}
	
	if (!validTetrahedrum)
	{
		m_points.resize(0);
		_ASSERT(0);
		return count;
	}
	
	// find the largest possible tetrahedron
	validTetrahedrum = false;
	hullVector e3(0.0);
	
	index0 = SupportVertex(&tree, points, normal);
	e3 = points[index0] - m_points[0];
	double err2 = normal.DotProduct(e3);
	if (fabs(err2) > (double(1.0e-6f) * m_diag * m_diag))
	{
		// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
		m_points[3] = points[index0];
		points[index0].m_mark = 1;
		validTetrahedrum = true;
	}
	if (!validTetrahedrum)
	{
		hullVector n(normal.Scale(double(-1.0f)));
		int index = SupportVertex(&tree, points, n);
		e3 = points[index] - m_points[0];
		double error2 = normal.DotProduct(e3);
		if (fabs(error2) > (double(1.0e-6f) * m_diag * m_diag))
		{
			// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
			m_points[3] = points[index];
			points[index].m_mark = 1;
			validTetrahedrum = true;
		}
	}
	if (!validTetrahedrum)
	{
		for (int i = 3; i < normalMap.m_count; i++)
		{
			int index = SupportVertex(&tree, points, normalMap.m_normal[i]);
			_ASSERT(index >= 0);
	
			//make sure the volume of the fist tetrahedral is no negative
			e3 = points[index] - m_points[0];
			double error2 = normal.DotProduct(e3);
			if (fabs(error2) > (double(1.0e-6f) * m_diag * m_diag))
			{
				// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
				m_points[3] = points[index];
				points[index].m_mark = 1;
				validTetrahedrum = true;
				break;
			}
		}
	}
	if (!validTetrahedrum)
	{
		// the points do not form a convex hull
		m_points.resize(0);
		//_ASSERT (0);
		return count;
	}
	
	m_points.resize(4);
	double volume = TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]);
	if (volume > double(0.0f))
	{
		vhacdSwap(m_points[2], m_points[3]);
	}
	_ASSERT(TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]) < double(0.0f));
	return count;
}

vhacdConvexHull::ndNode* vhacdConvexHull::AddFace(int i0, int i1, int i2)
{
	ndNode* const node = Append();
	vhacdConvexHullFace& face = node->GetInfo();

	face.m_index[0] = i0;
	face.m_index[1] = i1;
	face.m_index[2] = i2;
	return node;
}

void vhacdConvexHull::CalculateConvexHull3d(vhacdConvexHullAABBTreeNode* vertexTree, vhacdConvexHullVertex* const points, int count, double distTol, int maxVertexCount)
{
	distTol = fabs(distTol) * m_diag;
	ndNode* const f0Node = AddFace(0, 1, 2);
	ndNode* const f1Node = AddFace(0, 2, 3);
	ndNode* const f2Node = AddFace(2, 1, 3);
	ndNode* const f3Node = AddFace(1, 0, 3);

	vhacdConvexHullFace* const f0 = &f0Node->GetInfo();
	vhacdConvexHullFace* const f1 = &f1Node->GetInfo();
	vhacdConvexHullFace* const f2 = &f2Node->GetInfo();
	vhacdConvexHullFace* const f3 = &f3Node->GetInfo();

	f0->m_twin[0] = f3Node;
	f0->m_twin[1] = f2Node;
	f0->m_twin[2] = f1Node;

	f1->m_twin[0] = f0Node;
	f1->m_twin[1] = f2Node;
	f1->m_twin[2] = f3Node;

	f2->m_twin[0] = f0Node;
	f2->m_twin[1] = f3Node;
	f2->m_twin[2] = f1Node;

	f3->m_twin[0] = f0Node;
	f3->m_twin[1] = f1Node;
	f3->m_twin[2] = f2Node;
	
	vhacdList<ndNode*> boundaryFaces;
	boundaryFaces.Append(f0Node);
	boundaryFaces.Append(f1Node);
	boundaryFaces.Append(f2Node);
	boundaryFaces.Append(f3Node);

	m_points.resize(count);

	count -= 4;
	maxVertexCount -= 4;
	int currentIndex = 4;

	//ndStack<ndNode*> stackPool(1024 + m_count);
	//ndStack<ndNode*> coneListPool(1024 + m_count);
	//ndStack<ndNode*> deleteListPool(1024 + m_count);
	std::vector<ndNode*> stackPool;
	std::vector<ndNode*> coneListPool;
	std::vector<ndNode*> deleteListPool;

	stackPool.resize(1024 + count);
	coneListPool.resize(1024 + count);
	deleteListPool.resize(1024 + count);

	ndNode** const stack = &stackPool[0];
	ndNode** const coneList = &stackPool[0];
	ndNode** const deleteList = &deleteListPool[0];

	while (boundaryFaces.GetCount() && count && (maxVertexCount > 0))
	{
		// my definition of the optimal convex hull of a given vertex count,
		// is the convex hull formed by a subset of the input vertex that minimizes the volume difference
		// between the perfect hull formed from all input vertex and the hull of the sub set of vertex.
		// When using a priority heap this algorithms will generate the an optimal of a fix vertex count.
		// Since all Newton's tools do not have a limit on the point count of a convex hull, I can use either a stack or a queue.
		// a stack maximize construction speed, a Queue tend to maximize the volume of the generated Hull approaching a perfect Hull.
		// For now we use a queue.
		// For general hulls it does not make a difference if we use a stack, queue, or a priority heap.
		// perfect optimal hull only apply for when build hull of a limited vertex count.
		//
		// Also when building Hulls of a limited vertex count, this function runs in constant time.
		// yes that is correct, it does not makes a difference if you build a N point hull from 100 vertex
		// or from 100000 vertex input array.

		// using a queue (some what slower by better hull when reduced vertex count is desired)
		ndNode* const faceNode = boundaryFaces.GetLast()->GetInfo();
		vhacdConvexHullFace* const face = &faceNode->GetInfo();
		hullPlane planeEquation(face->GetPlaneEquation(&m_points[0]));

		int index = SupportVertex(&vertexTree, points, planeEquation);
		const hullVector& p = points[index];
		double dist = planeEquation.Evalue(p);

		if ((dist >= distTol) && (face->Evalue(&m_points[0], p) > double(0.0f)))
		{
			//_ASSERT(Sanity());
			//_ASSERT(faceNode);
			stack[0] = faceNode;

			int stackIndex = 1;
			int deletedCount = 0;

			while (stackIndex)
			{
				stackIndex--;
				ndNode* const node1 = stack[stackIndex];
				vhacdConvexHullFace* const face1 = &node1->GetInfo();
			
				if (!face1->m_mark && (face1->Evalue(&m_points[0], p) > double(0.0f)))
				{
					#ifdef _DEBUG
					for (int i = 0; i < deletedCount; i++)
					{
						_ASSERT(deleteList[i] != node1);
					}
					#endif
			
					deleteList[deletedCount] = node1;
					deletedCount++;
					_ASSERT(deletedCount < int(deleteListPool.size()));
					face1->m_mark = 1;
					for (int i = 0; i < 3; i++)
					{
						ndNode* const twinNode = face1->m_twin[i];
						_ASSERT(twinNode);
						vhacdConvexHullFace* const twinFace = &twinNode->GetInfo();
						if (!twinFace->m_mark)
						{
							stack[stackIndex] = twinNode;
							stackIndex++;
							_ASSERT(stackIndex < int(stackPool.size()));
						}
					}
				}
			}
			
			m_points[currentIndex] = points[index];
			points[index].m_mark = 1;
			
			int newCount = 0;
			for (int i = 0; i < deletedCount; i++)
			{
				ndNode* const node1 = deleteList[i];
				vhacdConvexHullFace* const face1 = &node1->GetInfo();
				_ASSERT(face1->m_mark == 1);
				for (int j0 = 0; j0 < 3; j0++)
				{
					ndNode* const twinNode = face1->m_twin[j0];
					vhacdConvexHullFace* const twinFace = &twinNode->GetInfo();
					if (!twinFace->m_mark)
					{
						int j1 = (j0 == 2) ? 0 : j0 + 1;
						ndNode* const newNode = AddFace(currentIndex, face1->m_index[j0], face1->m_index[j1]);
						boundaryFaces.Addtop(newNode);
			
						vhacdConvexHullFace* const newFace = &newNode->GetInfo();
						newFace->m_twin[1] = twinNode;
						for (int k = 0; k < 3; k++)
						{
							if (twinFace->m_twin[k] == node1)
							{
								twinFace->m_twin[k] = newNode;
							}
						}
						coneList[newCount] = newNode;
						newCount++;
						_ASSERT(newCount < int(coneListPool.size()));
					}
				}
			}
			
			for (int i = 0; i < newCount - 1; i++)
			{
				ndNode* const nodeA = coneList[i];
				vhacdConvexHullFace* const faceA = &nodeA->GetInfo();
				_ASSERT(faceA->m_mark == 0);
				for (int j = i + 1; j < newCount; j++) 
				{
					ndNode* const nodeB = coneList[j];
					vhacdConvexHullFace* const faceB = &nodeB->GetInfo();
					_ASSERT(faceB->m_mark == 0);
					if (faceA->m_index[2] == faceB->m_index[1])
					{
						faceA->m_twin[2] = nodeB;
						faceB->m_twin[0] = nodeA;
						break;
					}
				}
			
				for (int j = i + 1; j < newCount; j++)
				{
					ndNode* const nodeB = coneList[j];
					vhacdConvexHullFace* const faceB = &nodeB->GetInfo();
					_ASSERT(faceB->m_mark == 0);
					if (faceA->m_index[1] == faceB->m_index[2])
					{
						faceA->m_twin[0] = nodeB;
						faceB->m_twin[2] = nodeA;
						break;
					}
				}
			}
			
			for (int i = 0; i < deletedCount; i++)
			{
				ndNode* const node = deleteList[i];
				boundaryFaces.Remove(node);
				Remove(node);
			}

			maxVertexCount--;
			currentIndex++;
			count--;
		}
		else
		{
			boundaryFaces.Remove(faceNode);
		}
	}
	m_points.resize(currentIndex);
}
