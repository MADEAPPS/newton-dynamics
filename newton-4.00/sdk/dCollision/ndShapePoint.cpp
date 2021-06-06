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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndShapePoint.h"
#include "ndContactSolver.h"

ndShapePoint::ndShapePoint()
	:ndShapeConvex(m_pointCollision)
{
}

ndShapePoint::ndShapePoint(const nd::TiXmlNode* const)
	: ndShapeConvex(m_pointCollision)
{
}

ndShapePoint::~ndShapePoint()
{
}

void ndShapePoint::MassProperties()
{
	dAssert(0);
	//m_centerOfMass = dVector::m_zero;
	//m_crossInertia = dVector::m_zero;
	//dFloat32 volume = dFloat32(4.0f * dPi / 3.0f) * m_radius *  m_radius * m_radius;
	//dFloat32 II = dFloat32(2.0f / 5.0f) * m_radius *  m_radius;
	//m_inertia = dVector(II, II, II, dFloat32(0.0f));
	//m_centerOfMass.m_w = volume;
}

void ndShapePoint::CalcAABB(const dMatrix&, dVector &, dVector &) const
{
	dAssert(0);
	//dVector size(m_radius);
	//p0 = (matrix[3] - size) & dVector::m_triplexMask;
	//p1 = (matrix[3] + size) & dVector::m_triplexMask;
}

dVector ndShapePoint::SupportVertexSpecialProjectPoint(const dVector&, const dVector&) const
{
	dAssert(0);
	//return dir.Scale(m_radius - D_PENETRATION_TOL);
	return dVector::m_zero;
}

dVector ndShapePoint::SupportVertexSpecial(const dVector&, dFloat32, dInt32* const) const
{
	return dVector::m_zero;
}

dVector ndShapePoint::SupportVertex(const dVector&, dInt32* const) const
{
	dAssert(0);
	//dAssert(dir.m_w == dFloat32(0.0f));
	//dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));
	//dAssert(dir.m_w == 0.0f);
	//return dir.Scale(m_radius);
	return dVector::m_zero;
}

dInt32 ndShapePoint::CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const
{
	dAssert(normal.m_w == 0.0f);
	dAssert(normal.DotProduct(normal).GetScalar() > dFloat32(0.999f));
	contactsOut[0] = normal * normal.DotProduct(point);
	return 1;
}

dFloat32 ndShapePoint::RayCast(ndRayCastNotify&, const dVector&, const dVector&, dFloat32, const ndBody* const, ndContactPoint&) const
{
	dAssert(0);
	return dFloat32 (1.2f);
	//dFloat32 t = dRayCastSphere(localP0, localP1, dVector::m_zero, m_radius);
	//if (t < maxT) 
	//{
	//	dVector contact(localP0 + (localP1 - localP0).Scale(t));
	//	dAssert(contact.m_w == dFloat32(0.0f));
	//	//contactOut.m_normal = contact.Scale (dgRsqrt (contact.DotProduct(contact).GetScalar()));
	//	contactOut.m_normal = contact.Normalize();
	//	//contactOut.m_userId = SetUserDataID();
	//}
	//return t;
}

ndShapeInfo ndShapePoint::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeConvex::GetShapeInfo());
	info.m_point.m_noUsed = dFloat32 (0.0f);
	return info;
}

void ndShapePoint::DebugShape(const dMatrix&, ndShapeDebugCallback&) const
{
	dAssert(0);
	//dVector tmpVectex[1024 * 2];
	//
	//dVector p0(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
	//dVector p1(-dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
	//dVector p2(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
	//dVector p3(dFloat32(0.0f), -dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
	//dVector p4(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f));
	//dVector p5(dFloat32(0.0f), dFloat32(0.0f), -dFloat32(1.0f), dFloat32(0.0f));
	//
	//dInt32 index = 3;
	//dInt32 count = 0;
	//TesselateTriangle(index, p4, p0, p2, count, tmpVectex);
	//TesselateTriangle(index, p4, p2, p1, count, tmpVectex);
	//TesselateTriangle(index, p4, p1, p3, count, tmpVectex);
	//TesselateTriangle(index, p4, p3, p0, count, tmpVectex);
	//TesselateTriangle(index, p5, p2, p0, count, tmpVectex);
	//TesselateTriangle(index, p5, p1, p2, count, tmpVectex);
	//TesselateTriangle(index, p5, p3, p1, count, tmpVectex);
	//TesselateTriangle(index, p5, p0, p3, count, tmpVectex);
	//
	//for (dInt32 i = 0; i < count; i++) 
	//{
	//	tmpVectex[i] = matrix.TransformVector(tmpVectex[i].Scale(m_radius)) & dVector::m_triplexMask;
	//}
	//
	//for (dInt32 i = 0; i < count; i += 3) 
	//{
	//	debugCallback.DrawPolygon(3, &tmpVectex[i]);
	//}
}

D_COLLISION_API void ndShapePoint::Save( nd::TiXmlElement* const xmlNode, const char* const, dInt32 nodeid ) const
{
	nd::TiXmlElement* const paramNode = new nd::TiXmlElement("ndShapePoint");
	xmlNode->LinkEndChild(paramNode);

	paramNode->SetAttribute("nodeId", nodeid);

	//xmlSaveParam(paramNode, "point", m_radius);
}