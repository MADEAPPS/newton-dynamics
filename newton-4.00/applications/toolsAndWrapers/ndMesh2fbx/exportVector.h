#pragma once
class exportVector
{
	public:
	inline exportVector()
	{
	}

	inline exportVector(float val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	inline exportVector(const exportVector& v)
		: m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

	inline exportVector(const float* const ptr)
		: m_x(ptr[0])
		, m_y(ptr[1])
		, m_z(ptr[2])
		, m_w(ptr[3])
	{
	}

	inline exportVector(float x, float y, float z, float w)
		:m_x(x), m_y(y), m_z(z), m_w(w)
	{
	}

	inline float GetScalar() const
	{
		return m_x;
	}

	inline float& operator[] (int i)
	{
		return (&m_x)[i];
	}

	inline const float& operator[] (int i) const
	{
		return (&m_x)[i];
	}

	inline exportVector operator+ (const exportVector& A) const
	{
		return exportVector(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	inline exportVector operator- (const exportVector& A) const
	{
		return exportVector(m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	inline exportVector operator* (const exportVector& A) const
	{
		return exportVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}

	inline exportVector& operator+= (const exportVector& A)
	{
		return (*this = exportVector(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	inline exportVector& operator-= (const exportVector& A)
	{
		return (*this = exportVector(m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	inline exportVector& operator*= (const exportVector& A)
	{
		return (*this = exportVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w));
	}

	inline exportVector MulAdd(const exportVector& A, const exportVector& B) const
	{
		return *this + A * B;
	}

	inline exportVector MulSub(const exportVector& A, const exportVector& B) const
	{
		return *this - A * B;
	}

	inline exportVector AddHorizontal() const
	{
		return exportVector(m_x + m_y + m_z + m_w);
	}

	inline exportVector Scale(float scale) const
	{
		return exportVector(m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// return cross product
	inline exportVector CrossProduct(const exportVector& B) const
	{
		return exportVector(m_y * B.m_z - m_z * B.m_y,
			m_z * B.m_x - m_x * B.m_z,
			m_x * B.m_y - m_y * B.m_x, m_w);
	}

	inline float DotProduct(const exportVector &A) const
	{
		return m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w;
	}


	inline exportVector Normalize() const
	{
		const exportVector& me = *this;
		return me.Scale (1.0f / float(sqrt (me.DotProduct(me))));
	}

	inline exportVector Abs() const
	{
		return exportVector(
			(m_x > float(0.0f)) ? m_x : -m_x,
			(m_y > float(0.0f)) ? m_y : -m_y,
			(m_z > float(0.0f)) ? m_z : -m_z,
			(m_w > float(0.0f)) ? m_w : -m_w);
	}

	inline static void Transpose4x4(exportVector& dst0, exportVector& dst1, exportVector& dst2, exportVector& dst3, const exportVector& src0, const exportVector& src1, const exportVector& src2, const exportVector& src3)
	{
		exportVector tmp0(src0);
		exportVector tmp1(src1);
		exportVector tmp2(src2);
		exportVector tmp3(src3);

		dst0 = exportVector(tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
		dst1 = exportVector(tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
		dst2 = exportVector(tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
		dst3 = exportVector(tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
	}

	float m_x;
	float m_y;
	float m_z;
	float m_w;
};

