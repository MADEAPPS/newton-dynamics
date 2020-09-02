/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_PLANE_H__
#define __D_PLANE_H__

#include "dCoreStdafx.h"
#include "dVector.h"


#ifdef D_NEWTON_USE_DOUBLE
	#define dPlane dBigPlane
#else 

D_MSV_NEWTON_ALIGN_16
class dPlane: public dVector
{
	public:
	dPlane ();
	dPlane (const dVector& point);
	dPlane (dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w);
	dPlane (const dVector &normal, dFloat32 distance); 
	dPlane (const dVector &P0, const dVector &P1, const dVector &P2);
	dPlane Scale (dFloat32 s) const;
	dFloat32 Evalue (const dFloat32* const point) const;
	dFloat32 Evalue (const dVector &point) const;
} D_GCC_NEWTON_ALIGN_32 ;

#endif

class dBigPlane: public dBigVector
{
	public:
	dBigPlane ();
	dBigPlane (const dBigVector& point);
	dBigPlane (dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w);
	dBigPlane (const dBigVector &normal, dFloat64 distance); 
	dBigPlane (const dBigVector &P0, const dBigVector &P1, const dBigVector &P2);
	dBigPlane Scale (dFloat64 s) const;
	dFloat64 Evalue (const dFloat64* const point) const;
	dFloat64 Evalue (const dBigVector &point) const;
};

#ifndef D_NEWTON_USE_DOUBLE

D_INLINE dPlane::dPlane () 
	:dVector () 
{
}

D_INLINE dPlane::dPlane (const dVector& point)
	:dVector (point)
{
}

D_INLINE dPlane::dPlane (dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w)
	:dVector (x, y, z, w) 
{
}

D_INLINE dPlane::dPlane (const dVector &normal, dFloat32 distance) 
	:dVector (normal)
{
	m_w = distance;
}

D_INLINE dPlane::dPlane (const dVector &P0, const dVector &P1, const dVector &P2)
	:dVector ((P1 - P0).CrossProduct(P2 - P0)) 
{
	m_w = - DotProduct(P0 & dVector::m_triplexMask).GetScalar();
}

D_INLINE dPlane dPlane::Scale (dFloat32 s)	const
{
	return dPlane(*this * dVector(s));
}

D_INLINE dFloat32 dPlane::Evalue (const dFloat32* const point) const
{
	dVector p (point);
	return DotProduct ((p & m_triplexMask) | m_wOne).GetScalar();
}

D_INLINE dFloat32 dPlane::Evalue (const dVector& point) const
{
	return DotProduct ((point & m_triplexMask) | m_wOne).GetScalar();
}
#endif


D_INLINE dBigPlane::dBigPlane () 
	:dBigVector () 
{
}

D_INLINE dBigPlane::dBigPlane (const dBigVector& point)
	:dBigVector (point)
{
}

D_INLINE dBigPlane::dBigPlane (dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w)
	:dBigVector (x, y, z, w) 
{
}

D_INLINE dBigPlane::dBigPlane (const dBigVector &normal, dFloat64 distance) 
	:dBigVector (normal)
{
	m_w = distance;
}

D_INLINE dBigPlane::dBigPlane (const dBigVector &P0, const dBigVector &P1, const dBigVector &P2)
	:dBigVector ((P1 - P0).CrossProduct(P2 - P0)) 
{
	m_w = - DotProduct(P0 & dBigVector::m_triplexMask).GetScalar();
}

D_INLINE dBigPlane dBigPlane::Scale (dFloat64 s) const
{
	return dBigPlane (m_x * s, m_y * s, m_z * s, m_w * s);
}

D_INLINE dFloat64 dBigPlane::Evalue (const dFloat64* const point) const
{
	return m_x * point[0] + m_y * point[1] + m_z * point[2] + m_w;
}


D_INLINE dFloat64 dBigPlane::Evalue (const dBigVector &point) const
{
	return m_x * point.m_x + m_y * point.m_y + m_z * point.m_z + m_w;
}

#endif


