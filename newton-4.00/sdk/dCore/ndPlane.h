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
	ndPlane (ndFloat32 x, ndFloat32 y, ndFloat32 z, ndFloat32 w);
	ndPlane (const ndVector &normal, ndFloat32 distance); 
	ndPlane (const ndVector &P0, const ndVector &P1, const ndVector &P2);
	ndPlane Scale (ndFloat32 s) const;
	ndFloat32 Evalue (const ndFloat32* const point) const;
	ndFloat32 Evalue (const ndVector &point) const;
} D_GCC_NEWTON_ALIGN_32 ;

#endif

class ndBigPlane: public ndBigVector
{
	public:
	ndBigPlane ();
	ndBigPlane (const ndBigVector& point);
	ndBigPlane (ndFloat64 x, ndFloat64 y, ndFloat64 z, ndFloat64 w);
	ndBigPlane (const ndBigVector &normal, ndFloat64 distance); 
	ndBigPlane (const ndBigVector &P0, const ndBigVector &P1, const ndBigVector &P2);
	ndBigPlane Scale (ndFloat64 s) const;
	ndFloat64 Evalue (const ndFloat64* const point) const;
	ndFloat64 Evalue (const ndBigVector &point) const;
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

inline ndPlane::ndPlane (ndFloat32 x, ndFloat32 y, ndFloat32 z, ndFloat32 w)
	:ndVector (x, y, z, w) 
{
}

inline ndPlane::ndPlane (const ndVector &normal, ndFloat32 distance) 
	:ndVector (normal)
{
	m_w = distance;
}

inline ndPlane::ndPlane (const ndVector &P0, const ndVector &P1, const ndVector &P2)
	:ndVector ((P1 - P0).CrossProduct(P2 - P0)) 
{
	m_w = - DotProduct(P0 & ndVector::m_triplexMask).GetScalar();
}

inline ndPlane ndPlane::Scale (ndFloat32 s)	const
{
	return ndPlane(*this * ndVector(s));
}

inline ndFloat32 ndPlane::Evalue (const ndFloat32* const point) const
{
	ndVector p (point);
	return DotProduct ((p & m_triplexMask) | m_wOne).GetScalar();
}

inline ndFloat32 ndPlane::Evalue (const ndVector& point) const
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

inline ndBigPlane::ndBigPlane (ndFloat64 x, ndFloat64 y, ndFloat64 z, ndFloat64 w)
	:ndBigVector (x, y, z, w) 
{
}

inline ndBigPlane::ndBigPlane (const ndBigVector &normal, ndFloat64 distance) 
	:ndBigVector (normal)
{
	m_w = distance;
}

inline ndBigPlane::ndBigPlane (const ndBigVector &P0, const ndBigVector &P1, const ndBigVector &P2)
	:ndBigVector ((P1 - P0).CrossProduct(P2 - P0)) 
{
	m_w = - DotProduct(P0 & ndBigVector::m_triplexMask).GetScalar();
}

inline ndBigPlane ndBigPlane::Scale (ndFloat64 s) const
{
	return ndBigPlane (m_x * s, m_y * s, m_z * s, m_w * s);
}

inline ndFloat64 ndBigPlane::Evalue (const ndFloat64* const point) const
{
	return m_x * point[0] + m_y * point[1] + m_z * point[2] + m_w;
}


inline ndFloat64 ndBigPlane::Evalue (const ndBigVector &point) const
{
	return m_x * point.m_x + m_y * point.m_y + m_z * point.m_z + m_w;
}

#endif


