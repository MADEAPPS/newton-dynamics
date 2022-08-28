#pragma once
class bvhVector
{
	public:
	inline bvhVector()
	{
	}

	inline bvhVector(float val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	inline bvhVector(const bvhVector& v)
		: m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

	inline bvhVector(const float* const ptr)
		: m_x(ptr[0])
		, m_y(ptr[1])
		, m_z(ptr[2])
		, m_w(ptr[3])
	{
	}

	inline bvhVector(float x, float y, float z, float w)
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

	inline bvhVector operator+ (const bvhVector& A) const
	{
		return bvhVector(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	inline bvhVector operator- (const bvhVector& A) const
	{
		return bvhVector(m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	inline bvhVector operator* (const bvhVector& A) const
	{
		return bvhVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}

	inline bvhVector& operator+= (const bvhVector& A)
	{
		return (*this = bvhVector(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	inline bvhVector& operator-= (const bvhVector& A)
	{
		return (*this = bvhVector(m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	inline bvhVector& operator*= (const bvhVector& A)
	{
		return (*this = bvhVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w));
	}

	inline bvhVector MulAdd(const bvhVector& A, const bvhVector& B) const
	{
		return *this + A * B;
	}

	inline bvhVector MulSub(const bvhVector& A, const bvhVector& B) const
	{
		return *this - A * B;
	}

	inline bvhVector AddHorizontal() const
	{
		return bvhVector(m_x + m_y + m_z + m_w);
	}

	inline bvhVector Scale(float scale) const
	{
		return bvhVector(m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// return cross product
	inline bvhVector CrossProduct(const bvhVector& B) const
	{
		return bvhVector(m_y * B.m_z - m_z * B.m_y,
			m_z * B.m_x - m_x * B.m_z,
			m_x * B.m_y - m_y * B.m_x, m_w);
	}

	inline float DotProduct(const bvhVector &A) const
	{
		return m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w;
	}


	inline bvhVector Normalize() const
	{
		const bvhVector& me = *this;
		return me.Scale (1.0f / float(sqrt (me.DotProduct(me))));
	}

	inline bvhVector Abs() const
	{
		return bvhVector(
			(m_x > float(0.0f)) ? m_x : -m_x,
			(m_y > float(0.0f)) ? m_y : -m_y,
			(m_z > float(0.0f)) ? m_z : -m_z,
			(m_w > float(0.0f)) ? m_w : -m_w);
	}

	inline static void Transpose4x4(bvhVector& dst0, bvhVector& dst1, bvhVector& dst2, bvhVector& dst3, const bvhVector& src0, const bvhVector& src1, const bvhVector& src2, const bvhVector& src3)
	{
		bvhVector tmp0(src0);
		bvhVector tmp1(src1);
		bvhVector tmp2(src2);
		bvhVector tmp3(src3);

		dst0 = bvhVector(tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
		dst1 = bvhVector(tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
		dst2 = bvhVector(tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
		dst3 = bvhVector(tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
	}

	float m_x;
	float m_y;
	float m_z;
	float m_w;
};

