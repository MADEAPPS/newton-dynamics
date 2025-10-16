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
#include "ndCollisionStdafx.h"
#include "ndShapeInstance.h"
#include "ndContactSolver.h"
#include "ndCollisionStdafx.h"
#include "ndShapeStaticMesh.h"

ndShapeStaticMesh::ndShapeStaticMesh(ndShapeID id)
	:ndShape(id)
{
	ndAssert(ndMemory::CheckMemory(this));
}

ndShapeStaticMesh::~ndShapeStaticMesh()
{
	ndAssert(ndMemory::CheckMemory(this));
}

ndFloat32 ndShapeStaticMesh::GetVolume() const
{
	return ndFloat32(0.0f);
}

ndFloat32 ndShapeStaticMesh::GetBoxMinRadius() const
{
	return ndFloat32(0.0f);
}

ndFloat32 ndShapeStaticMesh::GetBoxMaxRadius() const
{
	return ndFloat32(0.0f);
}

ndVector ndShapeStaticMesh::SupportVertex(const ndVector&) const
{
	ndAssert(0);
	return ndVector::m_zero;
}

ndVector ndShapeStaticMesh::SupportVertexSpecial(const ndVector& dir, ndFloat32) const
{
	ndAssert(0);
	return SupportVertex(dir);
}

ndVector ndShapeStaticMesh::SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector&) const
{
	return point;
}

ndInt32 ndShapeStaticMesh::CalculatePlaneIntersection(const ndVector&, const ndVector&, ndVector* const) const
{
	return 0;
}

ndVector ndShapeStaticMesh::CalculateVolumeIntegral(const ndMatrix&, const ndVector&, const ndShapeInstance&) const
{
	return ndVector::m_zero;
}

ndShapeStaticMesh* ndShapeStaticMesh::GetAsShapeStaticMesh()
{
	return this;
}

void ndShapeStaticMesh::DebugShape(const ndMatrix&, ndShapeDebugNotify&) const
{
}

ndFloat32 ndShapeStaticMesh::RayCast(ndRayCastNotify&, const ndVector&, const ndVector&, ndFloat32, const ndBody* const, ndContactPoint&) const
{
	return ndFloat32(1.2f);
}

void ndShapeStaticMesh::GetCollidingFaces(ndPolygonMeshDesc* const) const
{
}

void ndShapeStaticMesh::CalculateAabb(const ndMatrix& matrix, ndVector &p0, ndVector &p1) const
{
	ndVector origin(matrix.TransformVector(m_boxOrigin));
	ndVector size(matrix.m_front.Abs().Scale(m_boxSize.m_x) + matrix.m_up.Abs().Scale(m_boxSize.m_y) + matrix.m_right.Abs().Scale(m_boxSize.m_z));

	p0 = (origin - size) & ndVector::m_triplexMask;
	p1 = (origin + size) & ndVector::m_triplexMask;
}

ndInt32 ndShapeStaticMesh::CalculatePlaneIntersection(const ndFloat32* const, const ndInt32* const, ndInt32, ndInt32, const ndPlane&, ndVector* const) const
{
	ndAssert(0);
	return 0;
	//ndInt32 count = 0;
	//ndInt32 j = index[indexCount - 1] * stride;
	//ndVector p0(&vertex[j]);
	//p0 = p0 & ndVector::m_triplexMask;
	//ndFloat32 side0 = localPlane.Evalue(p0);
	//for (ndInt32 i = 0; i < indexCount; ++i) 
	//{
	//	j = index[i] * stride;
	//	ndVector p1(&vertex[j]);
	//	p1 = p1 & ndVector::m_triplexMask;
	//	ndFloat32 side1 = localPlane.Evalue(p1);
	//
	//	if (side0 < ndFloat32(0.0f)) {
	//		if (side1 >= ndFloat32(0.0f)) {
	//			ndVector dp(p1 - p0);
	//			ndAssert(dp.m_w == ndFloat32(0.0f));
	//			ndFloat32 t = localPlane.DotProduct(dp).GetScalar();
	//			ndAssert(dgAbs(t) >= ndFloat32(0.0f));
	//			if (dgAbs(t) < ndFloat32(1.0e-8f)) {
	//				t = dgSign(t) * ndFloat32(1.0e-8f);
	//			}
	//			ndAssert(0);
	//			contactsOut[count] = p0 - dp.Scale(side0 / t);
	//			count++;
	//
	//		}
	//	}
	//	else if (side1 <= ndFloat32(0.0f)) {
	//		ndVector dp(p1 - p0);
	//		ndAssert(dp.m_w == ndFloat32(0.0f));
	//		ndFloat32 t = localPlane.DotProduct(dp).GetScalar();
	//		ndAssert(dgAbs(t) >= ndFloat32(0.0f));
	//		if (dgAbs(t) < ndFloat32(1.0e-8f)) {
	//			t = dgSign(t) * ndFloat32(1.0e-8f);
	//		}
	//		ndAssert(0);
	//		contactsOut[count] = p0 - dp.Scale(side0 / t);
	//		count++;
	//	}
	//
	//	side0 = side1;
	//	p0 = p1;
	//}
	//
	//return count;
}

