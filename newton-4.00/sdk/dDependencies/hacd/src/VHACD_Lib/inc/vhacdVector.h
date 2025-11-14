/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _ND_VHACD_VECTOR_H_
#define _ND_VHACD_VECTOR_H_

namespace nd 
{
	namespace VHACD 
	{
		class Triangle
		{
			public:
			Triangle()
			{
			}

			Triangle(ndInt32 x, ndInt32 y, ndInt32 z)
			{
				m_data[0] = x;
				m_data[1] = y;
				m_data[2] = z;
			}

			ndInt32& operator[](size_t i) 
			{ 
				return m_data[i]; 
			}

			const ndInt32& operator[](size_t i) const 
			{ 
				return m_data[i]; 
			}

			ndInt32 m_data[3];
		};

#if 1
		//!    Vector dim 3.
		class Vec3
		{
			public:
			Vec3();
			Vec3(double a);
			Vec3(double x, double y, double z);
			Vec3(const Vec3& rhs);

			double& operator[](size_t i);
			const double& operator[](size_t i) const;
			double& X();
			double& Y();
			double& Z();
			const double& X() const;
			const double& Y() const;
			const double& Z() const;
			void Normalize();
			double GetNorm() const;
			void operator=(const Vec3& rhs);
			void operator+=(const Vec3& rhs);
			void operator-=(const Vec3& rhs);
			void operator-=(double a);
			void operator+=(double a);
			void operator/=(double a);
			void operator*=(double a);
			Vec3 operator^(const Vec3& rhs) const;
			double operator*(const Vec3& rhs) const;
			Vec3 operator+(const Vec3& rhs) const;
			Vec3 operator-(const Vec3& rhs) const;
			Vec3 operator-() const;
			Vec3 operator*(double rhs) const;
			Vec3 operator/(double rhs) const;
			bool operator<(const Vec3& rhs) const;
			bool operator>(const Vec3& rhs) const;

			// Compute the center of this bounding box and return the diagonal length
			double GetCenter(const Vec3& bmin, const Vec3& bmax);

			// Update the min/max values relative to this point
			void UpdateMinMax(Vec3& bmin, Vec3& bmax) const;

			// Returns the squared distance between these two points
			double GetDistanceSquared(const Vec3& p) const;

			double GetDistance(const Vec3& p) const;

			// Returns the raw vector data as a pointer
			double* GetData(void);

			private:
			double m_data[3];
		};
	
		inline Vec3::Vec3() 
		{
			m_data[0] = m_data[1] = m_data[2] = 0.0;
		}

		inline Vec3::Vec3(double a)
		{
			m_data[0] = m_data[1] = m_data[2] = a;
		}

		inline Vec3::Vec3(double x, double y, double z)
		{
			m_data[0] = x;
			m_data[1] = y;
			m_data[2] = z;
		}

		inline Vec3::Vec3(const Vec3& rhs)
		{
			m_data[0] = rhs.m_data[0];
			m_data[1] = rhs.m_data[1];
			m_data[2] = rhs.m_data[2];
		}

		inline double& Vec3::operator[](size_t i)
		{ 
			return m_data[i];
		}

		inline const double& Vec3::operator[](size_t i) const
		{ 
			return m_data[i];
		}

		inline double& Vec3::X()
		{
			return m_data[0];
		}
		
		inline double& Vec3::Y()
		{
			return m_data[1];
		}
		
		inline double& Vec3::Z()
		{
			return m_data[2];
		}

		inline const double& Vec3::X() const
		{
			return m_data[0];
		}
		
		inline const double& Vec3::Y() const
		{
			return m_data[1];
		}
		
		inline const double& Vec3::Z() const
		{
			return m_data[2];
		}

		inline void Vec3::Normalize()
		{
			double n = GetNorm();
			if (n != 0.0)
			{
				operator*(1.0 / n);
			}
		}
		
		inline double Vec3::GetNorm() const
		{
			return sqrt(operator*(*this));
		}
		
		inline void Vec3::operator= (const Vec3& rhs)
		{
			m_data[0] = rhs.m_data[0];
			m_data[1] = rhs.m_data[1];
			m_data[2] = rhs.m_data[2];
		}
		
		inline void Vec3::operator+=(const Vec3& rhs)
		{
			m_data[0] += rhs.m_data[0];
			m_data[1] += rhs.m_data[1];
			m_data[2] += rhs.m_data[2];
		}
		
		inline void Vec3::operator-=(const Vec3& rhs)
		{
			m_data[0] -= rhs.m_data[0];
			m_data[1] -= rhs.m_data[1];
			m_data[2] -= rhs.m_data[2];
		}
		
		inline void Vec3::operator-=(double a)
		{
			m_data[0] -= a;
			m_data[1] -= a;
			m_data[2] -= a;
		}
		
		inline void Vec3::operator+=(double a)
		{
			m_data[0] += a;
			m_data[1] += a;
			m_data[2] += a;
		}
		
		inline void Vec3::operator/=(double a)
		{
			double reciproc = 1.0 / a;
			m_data[0] *= reciproc;
			m_data[1] *= reciproc;
			m_data[2] *= reciproc;
		}
		
		inline void Vec3::operator*=(double a)
		{
			m_data[0] *= a;
			m_data[1] *= a;
			m_data[2] *= a;
		}
		
		inline Vec3 Vec3::operator^ (const Vec3& rhs) const
		{
			return Vec3(
				m_data[1] * rhs.m_data[2] - m_data[2] * rhs.m_data[1],
				m_data[2] * rhs.m_data[0] - m_data[0] * rhs.m_data[2],
				m_data[0] * rhs.m_data[1] - m_data[1] * rhs.m_data[0]);
		}
		
		inline double Vec3::operator*(const Vec3& rhs) const
		{
			return (m_data[0] * rhs.m_data[0] + m_data[1] * rhs.m_data[1] + m_data[2] * rhs.m_data[2]);
		}

		inline Vec3 operator*(double lhs, const Vec3& rhs)
		{
			return Vec3(lhs * rhs.X(), lhs * rhs.Y(), lhs * rhs.Z());
		}

		inline Vec3 Vec3::operator+(const Vec3& rhs) const
		{
			return Vec3(m_data[0] + rhs.m_data[0], m_data[1] + rhs.m_data[1], m_data[2] + rhs.m_data[2]);
		}
		
		inline Vec3 Vec3::operator-(const Vec3& rhs) const
		{
			return Vec3(m_data[0] - rhs.m_data[0], m_data[1] - rhs.m_data[1], m_data[2] - rhs.m_data[2]);
		}
		
		inline Vec3 Vec3::operator-() const
		{
			return Vec3(-m_data[0], -m_data[1], -m_data[2]);
		}
		
		inline Vec3 Vec3::operator*(double rhs) const
		{
			return Vec3(rhs * m_data[0], rhs * m_data[1], rhs * m_data[2]);
		}
		
		inline Vec3 Vec3::operator/ (double rhs) const
		{
			double reciproc = 1.0 / rhs;
			return Vec3(m_data[0] * reciproc, m_data[1] * reciproc, m_data[2] * reciproc);
		}

		// Compute the center of this bounding box and return the diagonal length
		inline double Vec3::GetCenter(const Vec3& bmin, const Vec3& bmax)
		{
			X() = (bmin.X() + bmax.X()) * 0.5;
			Y() = (bmin.Y() + bmax.Y()) * 0.5;
			Z() = (bmin.Z() + bmax.Z()) * 0.5;
			double dx = bmax.X() - bmin.X();
			double dy = bmax.Y() - bmin.Y();
			double dz = bmax.Z() - bmin.Z();
			double diagonal = double(sqrt(dx * dx + dy * dy + dz * dz));
			return diagonal;
		}

		inline void Vec3::UpdateMinMax(Vec3& bmin, Vec3& bmax) const
		{
			if (X() < bmin.X())
			{
				bmin.X() = X();
			}
			if (Y() < bmin.Y())
			{
				bmin.Y() = Y();
			}
			if (Z() < bmin.Z())
			{
				bmin.Z() = Z();
			}
			if (X() > bmax.X())
			{
				bmax.X() = X();
			}
			if (X() > bmax.X())
			{
				bmax.X() = X();
			}
			if (Y() > bmax.Y())
			{
				bmax.Y() = Y();
			}
			if (Z() > bmax.Z())
			{
				bmax.Z() = Z();
			}
		}

		inline double Vec3::GetDistanceSquared(const Vec3& p) const
		{
			double dx = X() - p.X();
			double dy = Y() - p.Y();
			double dz = Z() - p.Z();
			return dx * dx + dy * dy + dz * dz;
		}

		inline double Vec3::GetDistance(const Vec3& p) const
		{
			return sqrt(GetDistanceSquared(p));
		}

		// Returns the raw vector data as a pointer
		inline double* Vec3::GetData(void)
		{
			return m_data;
		}
		
#else

		class Vec3 : public ndBigVector
		{
			public:
			Vec3();
			Vec3(double a);
			Vec3(double x, double y, double z);
			Vec3(const Vec3& rhs);
			Vec3(const ndBigVector& a);

			double& operator[](size_t i);
			const double& operator[](size_t i) const;
			double& X();
			double& Y();
			double& Z();
			const double& X() const;
			const double& Y() const;
			const double& Z() const;
			void Normalize();
			double GetNorm() const;
			void operator=(const Vec3& rhs);
			void operator+=(const Vec3& rhs);
			void operator-=(const Vec3& rhs);
			void operator-=(double a);
			void operator+=(double a);
			void operator/=(double a);
			void operator*=(double a);
			Vec3 operator^(const Vec3& rhs) const;
			double operator*(const Vec3& rhs) const;
			Vec3 operator+(const Vec3& rhs) const;
			Vec3 operator-(const Vec3& rhs) const;
			Vec3 operator-() const;
			Vec3 operator*(double rhs) const;
			Vec3 operator/(double rhs) const;
			bool operator<(const Vec3& rhs) const;
			bool operator>(const Vec3& rhs) const;

			// Compute the center of this bounding box and return the diagonal length
			double GetCenter(const Vec3& bmin, const Vec3& bmax);

			// Update the min/max values relative to this point
			void UpdateMinMax(Vec3& bmin, Vec3& bmax) const;

			// Returns the squared distance between these two points
			double GetDistanceSquared(const Vec3& p) const;

			double GetDistance(const Vec3& p) const;

			// Returns the raw vector data as a pointer
			double* GetData(void);
		};

		inline Vec3::Vec3()
			:ndBigVector(0.0)
		{
		}

		inline Vec3::Vec3(double a)
			:ndBigVector(a)
		{
		}

		inline Vec3::Vec3(double x, double y, double z)
			:ndBigVector(x, y, z, 0.0)
		{
		}

		inline Vec3::Vec3(const Vec3& rhs)
			:ndBigVector(rhs)
		{
		}

		inline Vec3::Vec3(const ndBigVector& rhs)
			:ndBigVector(rhs)
		{
		}

		inline double& Vec3::operator[](size_t i)
		{ 
			return (&m_x)[i];
		}

		inline const double& Vec3::operator[](size_t i) const
		{ 
			return (&m_x)[i];
		}

		inline Vec3 operator*(double lhs, const Vec3& rhs)
		{
			ndAssert(0);
			return Vec3(lhs * rhs.X(), lhs * rhs.Y(), lhs * rhs.Z());
		}

		inline double& Vec3::X()
		{
			ndAssert(0);
			//return m_data[0];
			return m_x;
		}

		inline double& Vec3::Y()
		{
			ndAssert(0);
			//return m_data[1];
			return m_y;
		}

		inline double& Vec3::Z()
		{
			ndAssert(0);
			//return m_data[2];
			return m_z;
		}

		inline const double& Vec3::X() const
		{
			ndAssert(0);
			//return m_data[0];
			return m_x;
		}

		inline const double& Vec3::Y() const
		{
			ndAssert(0);
			//return m_data[1];
			return m_y;
		}

		inline const double& Vec3::Z() const
		{
			ndAssert(0);
			//return m_data[2];
			return m_z;
		}

		inline void Vec3::Normalize()
		{
			ndAssert(0);
			m_w = 0.0;
			//double n = sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] + m_data[2] * m_data[2]);
			(*this) = ndBigVector::Normalize();
		}

		inline double Vec3::GetNorm() const
		{
			ndAssert(0);
			//return sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] + m_data[2] * m_data[2]);
			const Vec3 tmp(*this & ndBigVector::m_triplexMask);
			return tmp.DotProduct(tmp).GetScalar();
		}

		inline void Vec3::operator= (const Vec3& rhs)
		{
			ndBigVector::operator=(rhs);
		}

		inline void Vec3::operator+=(const Vec3& rhs)
		{
			//m_data[0] += rhs.m_data[0];
			//m_data[1] += rhs.m_data[1];
			//m_data[2] += rhs.m_data[2];
			ndBigVector::operator+=(rhs);
		}

		inline void Vec3::operator-=(const Vec3& rhs)
		{
			//m_data[0] -= rhs.m_data[0];
			//m_data[1] -= rhs.m_data[1];
			//m_data[2] -= rhs.m_data[2];
			ndBigVector::operator-=(rhs);
		}

		inline void Vec3::operator-=(double a)
		{
			//m_data[0] -= a;
			//m_data[1] -= a;
			//m_data[2] -= a;
			ndBigVector::operator-=(Vec3(a));
		}

		inline void Vec3::operator+=(double a)
		{
			//m_data[0] += a;
			//m_data[1] += a;
			//m_data[2] += a;
			ndBigVector::operator+=(Vec3(a));
		}

		inline void Vec3::operator/=(double a)
		{
			ndBigVector::operator*=(Vec3(1.0 / a));
		}

		inline void Vec3::operator*=(double a)
		{
			ndBigVector::operator*=(Vec3(a));
		}

		inline Vec3 Vec3::operator^ (const Vec3& rhs) const
		{
			//return Vec3(m_data[1] * rhs.m_data[2] - m_data[2] * rhs.m_data[1],
			//	m_data[2] * rhs.m_data[0] - m_data[0] * rhs.m_data[2],
			//	m_data[0] * rhs.m_data[1] - m_data[1] * rhs.m_data[0]);
			//this looks like a cros prodoc
			return CrossProduct(rhs);
		}

		inline double Vec3::operator*(const Vec3& rhs) const
		{
			//return (m_data[0] * rhs.m_data[0] + m_data[1] * rhs.m_data[1] + m_data[2] * rhs.m_data[2]);
			return DotProduct(rhs & ndBigVector::m_triplexMask).GetScalar();
		}

		inline Vec3 Vec3::operator+(const Vec3& rhs) const
		{
			//return Vec3(m_data[0] + rhs.m_data[0], m_data[1] + rhs.m_data[1], m_data[2] + rhs.m_data[2]);
			return ndBigVector::operator+(rhs);
		}

		inline Vec3 Vec3::operator-(const Vec3& rhs) const
		{
			//return Vec3(m_data[0] - rhs.m_data[0], m_data[1] - rhs.m_data[1], m_data[2] - rhs.m_data[2]);
			return ndBigVector::operator-(rhs);
		}

		inline Vec3 Vec3::operator-() const
		{
			ndAssert(0);
			//return Vec3(-m_data[0], -m_data[1], -m_data[2]);
			return Scale(-1.0);
		}

		inline Vec3 Vec3::operator*(double rhs) const
		{
			ndAssert(0);
			//return Vec3(rhs * m_data[0], rhs * m_data[1], rhs * m_data[2]);
			return *this * Vec3(rhs);
		}

		inline Vec3 Vec3::operator/ (double rhs) const
		{
			ndAssert(0);
			//return Vec3(m_data[0] / rhs, m_data[1] / rhs, m_data[2] / rhs);
			return *this * Vec3(1.0/rhs);
		}


		// Compute the center of this bounding box and return the diagonal length
		inline double Vec3::GetCenter(const Vec3& bmin, const Vec3& bmax)
		{
			ndAssert(0);
			X() = (bmin.X() + bmax.X()) * 0.5;
			Y() = (bmin.Y() + bmax.Y()) * 0.5;
			Z() = (bmin.Z() + bmax.Z()) * 0.5;
			double dx = bmax.X() - bmin.X();
			double dy = bmax.Y() - bmin.Y();
			double dz = bmax.Z() - bmin.Z();
			double diagonal = double(sqrt(dx * dx + dy * dy + dz * dz));
			return diagonal;
		}

		inline void Vec3::UpdateMinMax(Vec3& bmin, Vec3& bmax) const
		{
			ndAssert(0);
			if (X() < bmin.X())
			{
				bmin.X() = X();
			}
			if (Y() < bmin.Y())
			{
				bmin.Y() = Y();
			}
			if (Z() < bmin.Z())
			{
				bmin.Z() = Z();
			}
			if (X() > bmax.X())
			{
				bmax.X() = X();
			}
			if (X() > bmax.X())
			{
				bmax.X() = X();
			}
			if (Y() > bmax.Y())
			{
				bmax.Y() = Y();
			}
			if (Z() > bmax.Z())
			{
				bmax.Z() = Z();
			}
		}

		inline double Vec3::GetDistanceSquared(const Vec3& p) const
		{
			ndAssert(0);
			double dx = X() - p.X();
			double dy = Y() - p.Y();
			double dz = Z() - p.Z();
			return dx * dx + dy * dy + dz * dz;
		}

		inline double Vec3::GetDistance(const Vec3& p) const
		{
			ndAssert(0);
			return sqrt(GetDistanceSquared(p));
		}

		// Returns the raw vector data as a pointer
		inline double* Vec3::GetData(void)
		{
			ndAssert(0);
			return &(*this)[0];
		}
#endif

		inline bool Colinear(const Vec3& a, const Vec3& b, const Vec3& c)
		{
			return  ((c.Z() - a.Z()) * (b.Y() - a.Y()) - (b.Z() - a.Z()) * (c.Y() - a.Y()) == 0.0 /*EPS*/) &&
				((b.Z() - a.Z()) * (c.X() - a.X()) - (b.X() - a.X()) * (c.Z() - a.Z()) == 0.0 /*EPS*/) &&
				((b.X() - a.X()) * (c.Y() - a.Y()) - (b.Y() - a.Y()) * (c.X() - a.X()) == 0.0 /*EPS*/);
		}
		inline double ComputeVolume4(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
		{
			return (a - d) * ((b - d) ^ (c - d));
		}
	}
}
#endif