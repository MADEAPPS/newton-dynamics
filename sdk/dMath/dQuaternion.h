/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __dQuaternion__
#define __dQuaternion__

#include "dVector.h"
class dMatrix;

D_MSC_VECTOR_ALIGMENT
class dQuaternion
{
	public:
	dQuaternion (); 
	dQuaternion (const dMatrix& matrix);
	dQuaternion (dFloat q0, dFloat q1, dFloat q2, dFloat q3); 
	dQuaternion (const dVector& unit_Axis, dFloat Angle = 0.0f);
	
	void Scale (dFloat scale); 
	void Normalize (); 
	inline dFloat DotProduct (const dQuaternion& QB) const;
	dQuaternion Inverse () const; 

	dVector RotateVector (const dVector& point) const;
	dVector UnrotateVector (const dVector& point) const;

    void GetEulerAngles(dVector& euler1, dVector& euler2, dEulerAngleOrder order = m_pitchYawRoll) const;
	dVector CalcAverageOmega (const dQuaternion &q1, dFloat invdt) const;
	dQuaternion Slerp (const dQuaternion &q1, dFloat t) const;
	dQuaternion IntegrateOmega (const dVector& omega, dFloat timestep) const;

	dQuaternion operator* (const dQuaternion &B) const;
	dQuaternion operator+ (const dQuaternion &B) const; 
	dQuaternion operator- (const dQuaternion &B) const; 

	dFloat m_q1;
	dFloat m_q2;
	dFloat m_q3;
	dFloat m_q0;
};

inline dQuaternion::dQuaternion () 
	:m_q1(0.0f)
	,m_q2(0.0f)
	,m_q3(0.0f)
	,m_q0(1.0f)
{
}

inline dQuaternion::dQuaternion (dFloat q0, dFloat q1, dFloat q2, dFloat q3) 
	:m_q1(q1)
	,m_q2(q2)
	,m_q3(q3)
	,m_q0(q0)
{
}

inline void dQuaternion::Scale (dFloat scale) 
{
	m_q0 *= scale;
	m_q1 *= scale;
	m_q2 *= scale;
	m_q3 *= scale;
}

inline void dQuaternion::Normalize () 
{
	Scale (1.0f / dSqrt (DotProduct (*this)));
}

inline dFloat dQuaternion::DotProduct (const dQuaternion &q1) const
{
	return m_q0 * q1.m_q0 + m_q1 * q1.m_q1 + m_q2 * q1.m_q2 + m_q3 * q1.m_q3;
}

inline dQuaternion dQuaternion::Inverse () const 
{
	return dQuaternion (m_q0, -m_q1, -m_q2, -m_q3);
}

inline dQuaternion dQuaternion::operator+ (const dQuaternion &q) const
{
	return dQuaternion (m_q0 + q.m_q0, m_q1 + q.m_q1, m_q2 + q.m_q2, m_q3 + q.m_q3);
}

inline dQuaternion dQuaternion::operator- (const dQuaternion &B) const
{
	return dQuaternion (m_q0 - B.m_q0, m_q1 - B.m_q1, m_q2 - B.m_q2, m_q3 - B.m_q3);
}

inline dQuaternion dQuaternion::operator* (const dQuaternion &q) const
{
	return dQuaternion (q.m_q0 * m_q0 - q.m_q1 * m_q1 - q.m_q2 * m_q2 - q.m_q3 * m_q3, 
				 		q.m_q1 * m_q0 + q.m_q0 * m_q1 - q.m_q3 * m_q2 + q.m_q2 * m_q3, 
						q.m_q2 * m_q0 + q.m_q3 * m_q1 + q.m_q0 * m_q2 - q.m_q1 * m_q3, 
						q.m_q3 * m_q0 - q.m_q2 * m_q1 + q.m_q1 * m_q2 + q.m_q0 * m_q3); 
}


#endif 

