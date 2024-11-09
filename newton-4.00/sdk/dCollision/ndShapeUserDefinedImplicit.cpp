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
#include "ndContact.h"
#include "ndContactSolver.h"
#include "ndShapeUserDefinedImplicit.h"

ndShapeUserDefinedImplicit::ndShapeUserDefinedImplicit()
	:ndShapeConvex(m_userDefinedImplicit)
{
	ndAssert(0);
}

ndShapeUserDefinedImplicit::~ndShapeUserDefinedImplicit()
{
}

void ndShapeUserDefinedImplicit::MassProperties()
{
	ndAssert(0);
}

void ndShapeUserDefinedImplicit::CalculateAabb(const ndMatrix& matrix, ndVector &p0, ndVector &p1) const
{
	ndAssert(0);
	//ndVector size(m_radius);
	//p0 = (matrix[3] - size) & ndVector::m_triplexMask;
	//p1 = (matrix[3] + size) & ndVector::m_triplexMask;
}

ndVector ndShapeUserDefinedImplicit::SupportVertexSpecialProjectPoint(const ndVector&, const ndVector& dir) const
{
	ndAssert(0);
	return ndVector::m_zero;
//	return dir.Scale(m_radius - D_PENETRATION_TOL);
}

ndVector ndShapeUserDefinedImplicit::SupportVertexSpecial(const ndVector&, ndFloat32) const
{
	ndAssert(0);
	return ndVector::m_zero;
}

ndVector ndShapeUserDefinedImplicit::SupportVertex(const ndVector& dir) const
{
	ndAssert(0);
	return ndVector::m_zero;
	//ndAssert(dir.m_w == ndFloat32(0.0f));
	//ndAssert(ndAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));
	//ndAssert(dir.m_w == 0.0f);
	//return dir.Scale(m_radius);
}

ndInt32 ndShapeUserDefinedImplicit::CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const
{
	ndAssert(0);
	ndAssert(normal.m_w == 0.0f);
	ndAssert(normal.DotProduct(normal).GetScalar() > ndFloat32(0.999f));
	contactsOut[0] = normal * normal.DotProduct(point);
	return 1;
}

ndFloat32 ndShapeUserDefinedImplicit::RayCast(ndRayCastNotify&, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const, ndContactPoint& contactOut) const
{
	ndAssert(0);
	return 0;
	//ndFloat32 t = ndRayCastSphere(localP0, localP1, ndVector::m_zero, m_radius);
	//if (t < maxT) 
	//{
	//	ndVector contact(localP0 + (localP1 - localP0).Scale(t));
	//	ndAssert(contact.m_w == ndFloat32(0.0f));
	//	//contactOut.m_normal = contact.Scale (dgRsqrt (contact.DotProduct(contact).GetScalar()));
	//	contactOut.m_normal = contact.Normalize();
	//	//contactOut.m_userId = SetUserDataID();
	//}
	//return t;
}

ndShapeInfo ndShapeUserDefinedImplicit::GetShapeInfo() const
{
	ndAssert(0);
	ndShapeInfo info(ndShapeConvex::GetShapeInfo());
	//info.m_sphere.m_radius = m_radius;
	return info;
}

void ndShapeUserDefinedImplicit::DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const
{
	ndAssert(0);
	//ndVector tmpVectex[1024 * 2];
	//ndShapeDebugNotify::ndEdgeType edgeType[1024 * 2];
	//
	//ndVector p0(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	//ndVector p1(-ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	//ndVector p2(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	//ndVector p3(ndFloat32(0.0f), -ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	//ndVector p4(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f));
	//ndVector p5(ndFloat32(0.0f), ndFloat32(0.0f), -ndFloat32(1.0f), ndFloat32(0.0f));
	//
	//ndInt32 index = 3;
	//ndInt32 count = 0;
	//TesselateTriangle(index, p4, p0, p2, count, tmpVectex);
	//TesselateTriangle(index, p4, p2, p1, count, tmpVectex);
	//TesselateTriangle(index, p4, p1, p3, count, tmpVectex);
	//TesselateTriangle(index, p4, p3, p0, count, tmpVectex);
	//TesselateTriangle(index, p5, p2, p0, count, tmpVectex);
	//TesselateTriangle(index, p5, p1, p2, count, tmpVectex);
	//TesselateTriangle(index, p5, p3, p1, count, tmpVectex);
	//TesselateTriangle(index, p5, p0, p3, count, tmpVectex);
	//
	//for (ndInt32 i = 0; i < count; ++i) 
	//{
	//	edgeType[i] = ndShapeDebugNotify::m_shared;
	//	tmpVectex[i] = matrix.TransformVector(tmpVectex[i].Scale(m_radius)) & ndVector::m_triplexMask;
	//}
	//
	//for (ndInt32 i = 0; i < count; i += 3) 
	//{
	//	debugCallback.DrawPolygon(3, &tmpVectex[i], &edgeType[i]);
	//}
}

ndUnsigned64 ndShapeUserDefinedImplicit::GetHash(ndUnsigned64 hash) const
{
	ndAssert(0);
	ndShapeInfo info(GetShapeInfo());
	return info.GetHash(hash);
}