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
#include "ndContactSolver.h"
#include "ndShapeChamferCylinder.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndShapeChamferCylinder)

dInt32 ndShapeChamferCylinder::m_shapeRefCount = 0;
dVector ndShapeChamferCylinder::m_yzMask (0, 0xffffffff, 0xffffffff, 0);
dVector ndShapeChamferCylinder::m_shapesDirs[DG_MAX_CHAMFERCYLINDER_DIR_COUNT];
ndShapeConvex::ndConvexSimplexEdge ndShapeChamferCylinder::m_edgeArray[(4 * DG_CHAMFERCYLINDER_SLICES + 2)* DG_CHAMFERCYLINDER_BRAKES];

ndShapeChamferCylinder::ndShapeChamferCylinder(dFloat32 radius, dFloat32 height)
	:ndShapeConvex(m_chamferCylinder)
{
	Init (radius, height);
}

ndShapeChamferCylinder::ndShapeChamferCylinder(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndShapeConvex(m_chamferCylinder)
{
	//dVector size;
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	dFloat32 radius = xmlGetFloat(xmlNode, "radius");
	dFloat32 height = xmlGetFloat(xmlNode, "height");
	Init(radius, height);
}

ndShapeChamferCylinder::~ndShapeChamferCylinder()
{
	m_shapeRefCount --;
	dAssert (m_shapeRefCount >= 0);

	ndShapeConvex::m_simplex = nullptr;
	ndShapeConvex::m_vertex = nullptr;
}

void ndShapeChamferCylinder::Init (dFloat32 radius, dFloat32 height)
{
	m_radius = dMax (dAbs (radius), D_MIN_CONVEX_SHAPE_SIZE);
	m_height = dMax (dAbs (height * dFloat32 (0.5f)), D_MIN_CONVEX_SHAPE_SIZE);

	dFloat32 sliceAngle = dFloat32 (0.0f);
	dFloat32 sliceStep = dPi  / DG_CHAMFERCYLINDER_SLICES; 
	dFloat32 breakStep = dFloat32 (2.0f) * dPi / DG_CHAMFERCYLINDER_BRAKES;

	dMatrix rot (dPitchMatrix (breakStep));	
	dInt32 index = 0;
	for (dInt32 j = 0; j <= DG_CHAMFERCYLINDER_SLICES; j ++) 
	{
		dVector p0 (-m_height * dCos(sliceAngle), dFloat32 (0.0f), m_radius + m_height * dSin(sliceAngle), dFloat32 (0.0f));
		sliceAngle += sliceStep;
		for (dInt32 i = 0; i < DG_CHAMFERCYLINDER_BRAKES; i ++) 
		{
			m_vertex[index] = p0;
			index ++;
			p0 = rot.UnrotateVector (p0);
		}
	}

	m_edgeCount = (4 * DG_CHAMFERCYLINDER_SLICES + 2)* DG_CHAMFERCYLINDER_BRAKES;
	m_vertexCount = DG_CHAMFERCYLINDER_BRAKES * (DG_CHAMFERCYLINDER_SLICES + 1);
	ndShapeConvex::m_vertex = m_vertex;

	if (!m_shapeRefCount) 
	{
		dPolyhedra polyhedra;
		dInt32 wireframe[DG_CHAMFERCYLINDER_SLICES + 10];

		dVector locus(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
		for (dInt32 i = 0; i < DG_MAX_CHAMFERCYLINDER_DIR_COUNT; i ++) 
		{
			dMatrix matrix (dPitchMatrix (dFloat32 (2.0f) * dPi * dFloat32 (i) / DG_MAX_CHAMFERCYLINDER_DIR_COUNT));
			m_shapesDirs[i] = matrix.RotateVector (locus);
		}

		dInt32 index0 = 0;
		for (dInt32 j = 0; j < DG_CHAMFERCYLINDER_SLICES; j ++) 
		{
			dInt32 index1 = index0 + DG_CHAMFERCYLINDER_BRAKES - 1;
			for (dInt32 i = 0; i < DG_CHAMFERCYLINDER_BRAKES; i ++) 
			{
				wireframe[0] = index0;
				wireframe[1] = index1;
				wireframe[2] = index1 + DG_CHAMFERCYLINDER_BRAKES;
				wireframe[3] = index0 + DG_CHAMFERCYLINDER_BRAKES;

				index1 = index0;
				index0 ++;
				polyhedra.AddFace (4, wireframe);
			}
		}

		for (dInt32 i = 0; i < DG_CHAMFERCYLINDER_BRAKES; i ++) 
		{ 
			wireframe[i] = i;
		}
		polyhedra.AddFace (DG_CHAMFERCYLINDER_BRAKES, wireframe);

		for (dInt32 i = 0; i < DG_CHAMFERCYLINDER_BRAKES; i ++) 
		{ 
			wireframe[i] = DG_CHAMFERCYLINDER_BRAKES * (DG_CHAMFERCYLINDER_SLICES + 1) - i - 1;
		}
		polyhedra.AddFace (DG_CHAMFERCYLINDER_BRAKES, wireframe);
		polyhedra.EndFace ();

		dAssert (SanityCheck (polyhedra));

		dUnsigned64 i = 0;
		dPolyhedra::Iterator iter (polyhedra);
		for (iter.Begin(); iter; iter ++) 
		{
			dEdge* const edge = &(*iter);
			edge->m_userData = i;
			i ++;
		}

		for (iter.Begin(); iter; iter ++) 
		{
			dEdge* const edge = &(*iter);

			ndConvexSimplexEdge* const ptr = &m_edgeArray[edge->m_userData];
			ptr->m_vertex = edge->m_incidentVertex;
			ptr->m_next = &m_edgeArray[edge->m_next->m_userData];
			ptr->m_prev = &m_edgeArray[edge->m_prev->m_userData];
			ptr->m_twin = &m_edgeArray[edge->m_twin->m_userData];
		}
	}

	m_shapeRefCount ++;
	ndShapeConvex::m_simplex = m_edgeArray;

	SetVolumeAndCG ();
}


void ndShapeChamferCylinder::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndShapeConvex::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "radius", m_radius);
	xmlSaveParam(childNode, "height", m_height * dFloat32(2.0f));
}

ndShapeInfo ndShapeChamferCylinder::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeConvex::GetShapeInfo());

	info.m_chamferCylinder.m_r = m_radius;
	info.m_chamferCylinder.m_height = m_height * dFloat32(2.0f);
	return info;
}

void ndShapeChamferCylinder::CalculateAabb(const dMatrix& matrix, dVector& p0, dVector& p1) const
{
	ndShapeConvex::CalculateAabb(matrix, p0, p1);

//	dAssert(0);
	//dVector size0(m_radius0);
	//dVector size1(m_radius1);
	//dVector q0(matrix.m_posit - matrix.m_front.Scale(m_height));
	//dVector q1(matrix.m_posit + matrix.m_front.Scale(m_height));
	//
	//dVector min_q0(q0 - size0);
	//dVector min_q1(q1 - size1);
	//
	//dVector max_q0(q0 + size1);
	//dVector max_q1(q1 + size1);
	//
	//p0 = min_q0.GetMin(min_q1) & dVector::m_triplexMask;
	//p1 = max_q0.GetMax(max_q1) & dVector::m_triplexMask;
}

void ndShapeChamferCylinder::DebugShape(const dMatrix& matrix, ndShapeDebugNotify& debugCallback) const
{
	dInt32 slices = 12;
	dInt32 brakes = 24;
	dFloat32 sliceAngle = dFloat32(0.0f);
	dFloat32 sliceStep = dPi / slices;
	dFloat32 breakStep = dFloat32(2.0f) * dPi / brakes;
	dVector pool[24 * (12 + 1)];

	dMatrix rot(dPitchMatrix(breakStep));
	dInt32 index = 0;
	for (dInt32 j = 0; j <= slices; j++)
	{
		dVector p0(-m_height * dCos(sliceAngle), dFloat32(0.0f), m_radius + m_height * dSin(sliceAngle), dFloat32(0.0f));
		sliceAngle += sliceStep;
		for (dInt32 i = 0; i < brakes; i++)
		{
			pool[index] = p0;
			p0 = rot.UnrotateVector(p0);
			index++;
		}
	}

	matrix.TransformTriplex(&pool[0].m_x, sizeof(dVector), &pool[0].m_x, sizeof(dVector), 24 * (12 + 1));

	dVector face[32];
	ndShapeDebugNotify::ndEdgeType edgeType[32];
	memset(edgeType, ndShapeDebugNotify::m_shared, sizeof(edgeType));

	index = 0;
	for (dInt32 j = 0; j < slices; j++)
	{
		dInt32 index0 = index + brakes - 1;
		for (dInt32 i = 0; i < brakes; i++)
		{
			face[0] = pool[index];
			face[1] = pool[index0];
			face[2] = pool[index0 + brakes];
			face[3] = pool[index + brakes];
			index0 = index;
			index++;
			debugCallback.DrawPolygon(4, face, edgeType);
		}
	}

	for (dInt32 i = 0; i < brakes; i++)
	{
		face[i] = pool[i];
	}
	debugCallback.DrawPolygon(24, face, edgeType);

	for (dInt32 i = 0; i < brakes; i++)
	{
		face[i] = pool[brakes * (slices + 1) - i - 1];
	}
	debugCallback.DrawPolygon(24, face, edgeType);
}

dVector ndShapeChamferCylinder::SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const
{
	dAssert(dir.m_w == 0.0f);
	return point + dir.Scale(m_height - D_PENETRATION_TOL);
}

dVector ndShapeChamferCylinder::SupportVertex(const dVector& dir, dInt32* const) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));

	dFloat32 x = dir.GetScalar();
	if (dAbs(x) > dFloat32(0.9999f)) 
	{
		return dVector(dSign(x) * m_height, m_radius, dFloat32(0.0f), dFloat32(0.0f));
	}

	dVector sideDir(m_yzMask & dir);
	sideDir = sideDir.Normalize();
	return sideDir.Scale(m_radius) + dir.Scale(m_height);
}

dVector ndShapeChamferCylinder::SupportVertexSpecial(const dVector& dir, dFloat32, dInt32* const) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));

	dFloat32 x = dir.GetScalar();
	if (dAbs(x) > dFloat32(0.99995f)) 
	{
		return dVector(dFloat32(0.0f), m_radius, dFloat32(0.0f), dFloat32(0.0f));
	}

	dVector sideDir(m_yzMask & dir);
	dAssert(sideDir.DotProduct(sideDir).GetScalar() > dFloat32(0.0f));
	return sideDir.Normalize().Scale(m_radius);
}

dFloat32 ndShapeChamferCylinder::RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
{
	if (localP0.m_x > m_height) 
	{
		if (localP1.m_x < m_height) 
		{
			dFloat32 t1 = (m_height - localP0.m_x) / (localP1.m_x - localP0.m_x);
			dFloat32 y = localP0.m_y + (localP1.m_y - localP0.m_y) * t1;
			dFloat32 z = localP0.m_z + (localP1.m_z - localP0.m_z) * t1;
			if ((y * y + z * z) < m_radius * m_radius) 
			{
				contactOut.m_normal = dVector(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
				return t1;
			}
		}
	}

	if (localP0.m_x < -m_height) 
	{
		if (localP1.m_x > -m_height) 
		{
			dFloat32 t1 = (-m_height - localP0.m_x) / (localP1.m_x - localP0.m_x);
			dFloat32 y = localP0.m_y + (localP1.m_y - localP0.m_y) * t1;
			dFloat32 z = localP0.m_z + (localP1.m_z - localP0.m_z) * t1;
			if ((y * y + z * z) < m_radius * m_radius) 
			{
				contactOut.m_normal = dVector(dFloat32(-1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
				return t1;
			}
		}
	}

	dVector dq((localP1 - localP0) & dVector::m_triplexMask);

	// avoid NaN as a result of a division by zero
	if (dq.DotProduct(dq).GetScalar() <= 0.0f) 
	{
		return dFloat32(1.2f);
	}

	//dVector dir(dq * dq.InvMagSqrt());
	dVector dir(dq.Normalize());
	if (dAbs(dir.m_x) > 0.9999f) 
	{
		//return ndShapeConvex::RayCast(localP0, localP1, maxT, contactOut, body, nullptr, nullptr);
		return ndShapeConvex::RayCast(callback, localP0, localP1, maxT, body, contactOut);
	}

	dVector p0(localP0 & dVector::m_triplexMask);
	dVector p1(localP1 & dVector::m_triplexMask);

	p0.m_x = dFloat32(0.0f);
	p1.m_x = dFloat32(0.0f);

	dVector dp(p1 - p0);
	dFloat32 a = dp.DotProduct(dp).GetScalar();
	dFloat32 b = dFloat32(2.0f) * dp.DotProduct(p0).GetScalar();
	dFloat32 c = p0.DotProduct(p0).GetScalar() - m_radius * m_radius;

	dFloat32 disc = b * b - dFloat32(4.0f) * a * c;
	if (disc >= dFloat32(0.0f)) 
	{
		disc = dSqrt(disc);
		dVector origin0(p0 + dp.Scale((-b + disc) / (dFloat32(2.0f) * a)));
		dVector origin1(p0 + dp.Scale((-b - disc) / (dFloat32(2.0f) * a)));
		dFloat32 t0 = dRayCastSphere(localP0, localP1, origin0, m_height);
		dFloat32 t1 = dRayCastSphere(localP0, localP1, origin1, m_height);
		if (t1 < t0) 
		{
			t0 = t1;
			origin0 = origin1;
		}

		if ((t0 >= 0.0f) && (t0 <= 1.0f)) 
		{
			contactOut.m_normal = localP0 + dq.Scale(t0) - origin0;
			dAssert(contactOut.m_normal.m_w == dFloat32(0.0f));

			//contactOut.m_normal = contactOut.m_normal * contactOut.m_normal.DotProduct(contactOut.m_normal).InvSqrt();
			contactOut.m_normal = contactOut.m_normal.Normalize();
			return t0;
		}
	}
	else 
	{
		dVector origin0(dPointToRayDistance(dVector::m_zero, p0, p1));
		origin0 = origin0.Scale(m_radius / dSqrt(origin0.DotProduct(origin0).GetScalar()));
		dFloat32 t0 = dRayCastSphere(localP0, localP1, origin0, m_height);
		if ((t0 >= 0.0f) && (t0 <= 1.0f)) 
		{
			contactOut.m_normal = localP0 + dq.Scale(t0) - origin0;
			dAssert(contactOut.m_normal.m_w == dFloat32(0.0f));

			//contactOut.m_normal = contactOut.m_normal * contactOut.m_normal.DotProduct(contactOut.m_normal).InvSqrt();
			contactOut.m_normal = contactOut.m_normal.Normalize();
			return t0;
		}
	}
	return dFloat32(1.2f);
}

dInt32 ndShapeChamferCylinder::CalculatePlaneIntersection(const dVector& normal, const dVector& origin, dVector* const contactsOut) const
{
	dInt32 count = 0;
	const dFloat32 inclination = dFloat32(0.9999f);
	if (normal.m_x < -inclination) 
	{
		dMatrix matrix(normal);
		dFloat32 x = dSqrt(dMax(m_height * m_height - origin.m_x * origin.m_x, dFloat32(0.0f)));
		matrix.m_posit.m_x = origin.m_x;
		count = BuildCylinderCapPoly(m_radius + x, matrix, contactsOut);
		//count = RectifyConvexSlice(n, normal, contactsOut);
	}
	else if (normal.m_x > inclination) 
	{
		dMatrix matrix(normal);
		dFloat32 x = dSqrt(dMax(m_height * m_height - origin.m_x * origin.m_x, dFloat32(0.0f)));
		matrix.m_posit.m_x = origin.m_x;
		count = BuildCylinderCapPoly(m_radius + x, matrix, contactsOut);
		//count = RectifyConvexSlice(n, normal, contactsOut);
	}
	else 
	{
		count = 1;
		contactsOut[0] = SupportVertex(normal, nullptr);
	}
	return count;
}

