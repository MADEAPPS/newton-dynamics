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

#ifndef __ND_PLANE_H__
#define __ND_PLANE_H__

#include "ndCoreStdafx.h"
#include "ndVector.h"


#ifdef D_NEWTON_USE_DOUBLE
	#define ndPlane ndBigPlane
#else 

D_MSV_NEWTON_ALIGN_16
class ndPlane: public ndVector
{
	public:
	ndPlane ();
	ndPlane (const ndVector& point);
	ndPlane (dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w);
	ndPlane (const ndVector &normal, dFloat32 distance); 
	ndPlane (const ndVector &P0, const ndVector &P1, const ndVector &P2);
	ndPlane Scale (dFloat32 s) const;
	dFloat32 Evalue (const dFloat32* const point) const;
	dFloat32 Evalue (const ndVector &point) const;
} D_GCC_NEWTON_ALIGN_32 ;

#endif

class ndBigPlane: public ndBigVector
{
	public:
	ndBigPlane ();
	ndBigPlane (const ndBigVector& point);
	ndBigPlane (dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w);
	ndBigPlane (const ndBigVector &normal, dFloat64 distance); 
	ndBigPlane (const ndBigVector &P0, const ndBigVector &P1, const ndBigVector &P2);
	ndBigPlane Scale (dFloat64 s) const;
	dFloat64 Evalue (const dFloat64* const point) const;
	dFloat64 Evalue (const ndBigVector &point) const;
};

#ifndef D_NEWTON_USE_DOUBLE

inline ndPlane::ndPlane () 
	:ndVector () 
{
}

inline ndPlane::ndPlane (const ndVector& point)
	:ndVector (point)
{
}

inline ndPlane::ndPlane (dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w)
	:ndVector (x, y, z, w) 
{
}

inline ndPlane::ndPlane (const ndVector &normal, dFloat32 distance) 
	:ndVector (normal)
{
	m_w = distance;
}

inline ndPlane::ndPlane (const ndVector &P0, const ndVector &P1, const ndVector &P2)
	:ndVector ((P1 - P0).CrossProduct(P2 - P0)) 
{
	m_w = - DotProduct(P0 & ndVector::m_triplexMask).GetScalar();
}

inline ndPlane ndPlane::Scale (dFloat32 s)	const
{
	return ndPlane(*this * ndVector(s));
}

inline dFloat32 ndPlane::Evalue (const dFloat32* const point) const
{
	ndVector p (point);
	return DotProduct ((p & m_triplexMask) | m_wOne).GetScalar();
}

inline dFloat32 ndPlane::Evalue (const ndVector& point) const
{
	return DotProduct ((point & m_triplexMask) | m_wOne).GetScalar();
}
#endif


inline ndBigPlane::ndBigPlane () 
	:ndBigVector () 
{
}

inline ndBigPlane::ndBigPlane (const ndBigVector& point)
	:ndBigVector (point)
{
}

inline ndBigPlane::ndBigPlane (dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w)
	:ndBigVector (x, y, z, w) 
{
}

inline ndBigPlane::ndBigPlane (const ndBigVector &normal, dFloat64 distance) 
	:ndBigVector (normal)
{
	m_w = distance;
}

inline ndBigPlane::ndBigPlane (const ndBigVector &P0, const ndBigVector &P1, const ndBigVector &P2)
	:ndBigVector ((P1 - P0).CrossProduct(P2 - P0)) 
{
	m_w = - DotProduct(P0 & ndBigVector::m_triplexMask).GetScalar();
}

inline ndBigPlane ndBigPlane::Scale (dFloat64 s) const
{
	return ndBigPlane (m_x * s, m_y * s, m_z * s, m_w * s);
}

inline dFloat64 ndBigPlane::Evalue (const dFloat64* const point) const
{
	return m_x * point[0] + m_y * point[1] + m_z * point[2] + m_w;
}


inline dFloat64 ndBigPlane::Evalue (const ndBigVector &point) const
{
	return m_x * point.m_x + m_y * point.m_y + m_z * point.m_z + m_w;
}

#endif


