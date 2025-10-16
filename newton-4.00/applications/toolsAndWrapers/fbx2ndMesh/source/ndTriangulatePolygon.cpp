/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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
#include "ndPolyhedra.h"
#include "ndTriangulatePolygon.h"

void ndTriangulatePolygon(ndVector* const points, ndInt32 count, ndArray<ndInt32>& triangles)
{
	triangles.SetCount(0);
	if (count < 3)
	{
		return;
	}
	if (count == 3)
	{
		triangles.PushBack(0);
		triangles.PushBack(1);
		triangles.PushBack(2);
		return;
	}

	ndInt32 index[256];
	ndFixSizeArray<ndBigVector, 256> vertexBuffer;
	for (ndInt32 i = 0; i < count; ++i)
	{
		index[i] = i;
		vertexBuffer.PushBack(points[i]);
	}

	ndPolyhedra polyhedra;
	polyhedra.BeginFace();
	polyhedra.AddFace(count, index);
	polyhedra.EndFace();

	ndPolyhedra leftOversOut;
	polyhedra.Triangulate(&vertexBuffer[0].m_x, sizeof (ndBigVector), &leftOversOut);

	ndInt32 mark = polyhedra.IncLRU();
	ndPolyhedra::Iterator it(polyhedra);
	for (it.Begin(); it; it++)
	{
		ndEdge* const edge = &it.GetNode()->GetInfo();
		if (edge->m_mark != mark && (edge->m_incidentFace > 0))
		{
			ndAssert(edge == edge->m_next->m_next->m_next);
			edge->m_mark = mark;
			edge->m_next->m_mark = mark;
			edge->m_next->m_next->m_mark = mark;
			triangles.PushBack(edge->m_incidentVertex);
			triangles.PushBack(edge->m_next->m_incidentVertex);
			triangles.PushBack(edge->m_next->m_next->m_incidentVertex);
		}
	}
}
