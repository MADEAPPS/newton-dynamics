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

#include "vhacdConvexHullUtils.h"

vhacdGoogol vhacdGoogol::m_zero(0.0);
vhacdGoogol vhacdGoogol::m_one(1.0);
vhacdGoogol vhacdGoogol::m_two(2.0);
vhacdGoogol vhacdGoogol::m_three(3.0);
vhacdGoogol vhacdGoogol::m_half(0.5);

vhacdGoogol::vhacdGoogol(void)
	:m_sign(0)
	,m_exponent(0)
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
		uint64_t test = a & mask;		\
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



#define Absolute(a)  ((a) >= 0.0 ? (a) : -(a))

double Determinant2x2(const double matrix[2][2], double* const error)
{
	double a00xa11 = matrix[0][0] * matrix[1][1];
	double a01xa10 = matrix[0][1] * matrix[1][0];
	*error = Absolute(a00xa11) + Absolute(a01xa10);
	return a00xa11 - a01xa10;
}

double Determinant3x3 (const double matrix[3][3], double* const error)
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

vhacdGoogol Determinant2x2(const vhacdGoogol matrix[2][2])
{
	vhacdGoogol a00xa11(matrix[0][0] * matrix[1][1]);
	vhacdGoogol a01xa10(matrix[0][1] * matrix[1][0]);
	return a00xa11 - a01xa10;
}

vhacdGoogol Determinant3x3(const vhacdGoogol matrix[3][3])
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


vhacdSemaphore::vhacdSemaphore()
	:m_count(0)
	,m_mutex()
	,m_condition()
	,m_terminate(false)
{
}

vhacdSemaphore::~vhacdSemaphore()
{
}

bool vhacdSemaphore::Wait()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	while (m_count == 0)
	{
		m_condition.wait(lock);
	}

	m_count--;
	return m_terminate.load();
}

void vhacdSemaphore::Signal()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	m_count++;
	m_condition.notify_one();
}

void vhacdSemaphore::Terminate()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	m_count++;
	m_terminate.store(true);
	m_condition.notify_one();
}

vhacdQueue::vhacdQueue()
	:vhacdList<vhacdJob*>()
	,m_mutex()
	,m_jobs(0)
{
	for (int i = 0; i < VHACD_WORKERS_THREADS; i++)
	{
		m_threads[i].m_threadID = i;
		m_threads[i].m_queue = this;
	}
}

vhacdQueue::~vhacdQueue()
{
	for (int i = 0; i < VHACD_WORKERS_THREADS; i++)
	{
		m_threads[i].Terminate();
		m_threads[i].join();
	}
}

void vhacdQueue::PushTask(vhacdJob* const job)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	Append(job);
	m_jobs.fetch_add(1);
	for (int i = 0; i < VHACD_WORKERS_THREADS; i++)
	{
		m_threads[i].Signal();
	}
}

vhacdJob* vhacdQueue::PopTask()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	vhacdJob* job = nullptr;
	if (GetCount())
	{
		ndNode* const node = GetFirst();
		job = node->GetInfo();
		Remove(node);
	}
	return job;
}

void vhacdQueue::Sync()
{
	while (m_jobs)
	{
		std::this_thread::yield();
	}
}

vhacdThread::vhacdThread()
	:vhacdSemaphore()
	,std::thread(&vhacdThread::ThreadFunctionCallback, this)
	,m_queue(nullptr)
{
}

vhacdThread::~vhacdThread()
{
}

void vhacdThread::ThreadFunctionCallback()
{
	while (!Wait())
	{
		vhacdJob* const job = m_queue->PopTask();
		if (job)
		{
			job->Execute(m_threadID);
			m_queue->m_jobs.fetch_add(-1);
		}
	}
}

