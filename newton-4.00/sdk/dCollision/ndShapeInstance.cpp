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


#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndShapeInstance.h"
#include "ndRayCastNotify.h"
#include "ndBodyKinematic.h"

dVector ndShapeInstance::m_padding(D_MAX_SHAPE_AABB_PADDING, D_MAX_SHAPE_AABB_PADDING, D_MAX_SHAPE_AABB_PADDING, dFloat32(0.0f));

ndShapeInstance::ndShapeInstance(ndShape* const shape)
	:dClassAlloc()
	,m_globalMatrix(dGetIdentityMatrix())
	,m_localMatrix(dGetIdentityMatrix())
	,m_aligmentMatrix(dGetIdentityMatrix())
	,m_scale(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f))
	,m_invScale(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f))
	,m_maxScale(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f))
	,m_shape(shape->AddRef())
	,m_ownerBody(nullptr)
	,m_skinThickness(dFloat32(0.0f))
	,m_scaleType(m_unit)
	,m_collisionMode(true)
{
}

ndShapeInstance::ndShapeInstance(const ndShapeInstance& instance)
	:dClassAlloc()
	,m_globalMatrix(instance.m_globalMatrix)
	,m_localMatrix(instance.m_localMatrix)
	,m_aligmentMatrix(instance.m_aligmentMatrix)
	,m_scale(instance.m_scale)
	,m_invScale(instance.m_invScale)
	,m_maxScale(instance.m_maxScale)
	,m_shape(instance.m_shape->AddRef())
	,m_ownerBody(instance.m_ownerBody)
	,m_skinThickness(instance.m_skinThickness)
	,m_scaleType(instance.m_scaleType)
	,m_collisionMode(instance.m_collisionMode)
{
}

ndShapeInstance::ndShapeInstance(const nd::TiXmlNode* const xmlNode, const dTree<const ndShape*, dUnsigned32>& shapesCache)
	:dClassAlloc()
	,m_globalMatrix(dGetIdentityMatrix())
	,m_localMatrix(dGetIdentityMatrix())
	,m_aligmentMatrix(dGetIdentityMatrix())
	,m_scale(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f))
	,m_invScale(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f))
	,m_maxScale(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f))
	,m_shape(nullptr)
	,m_ownerBody(nullptr)
	,m_skinThickness(dFloat32(0.0f))
	,m_scaleType(m_unit)
	,m_collisionMode(true)
{
	dInt32 index = xmlGetInt(xmlNode, "shapeNodeId");
	m_shape = shapesCache.Find(index)->GetInfo()->AddRef();

	m_localMatrix = xmlGetMatrix(xmlNode, "localMatrix");
	m_aligmentMatrix = xmlGetMatrix(xmlNode, "aligmentMatrix");
	m_skinThickness = xmlGetFloat(xmlNode, "skinThickness");
	m_collisionMode = xmlGetInt(xmlNode, "collisionMode") ? true : false;
	m_shapeMaterial.m_userId = xmlGetInt64(xmlNode, "materialID");
	m_shapeMaterial.m_alignPad = xmlGetInt64(xmlNode, "materialUserData");

	for (dInt32 i = 0; i < sizeof(m_shapeMaterial.m_userParam) / sizeof(m_shapeMaterial.m_userParam[0]); i++)
	{
		char name[64];
		sprintf(name, "intData%d", i);
		m_shapeMaterial.m_userParam[i].m_intData = xmlGetInt64(xmlNode, name);
	}
	
	dVector scale (xmlGetVector3(xmlNode, "scale"));
	SetScale(scale);
}

ndShapeInstance::~ndShapeInstance()
{
	m_shape->Release();
}

ndShapeInstance& ndShapeInstance::operator=(const ndShapeInstance& instance)
{
	m_globalMatrix = instance.m_globalMatrix;
	m_localMatrix = instance.m_localMatrix;
	m_aligmentMatrix = instance.m_aligmentMatrix;
	m_scale = instance.m_scale;
	m_invScale = instance.m_invScale;
	m_maxScale = instance.m_maxScale;
	m_scaleType = instance.m_scaleType;
	m_shapeMaterial = instance.m_shapeMaterial;
	m_skinThickness = instance.m_skinThickness;
	m_collisionMode = instance.m_collisionMode;

	m_shape->Release();
	m_shape = instance.m_shape->AddRef();
	m_ownerBody = instance.m_ownerBody;
	return *this;
}

void ndShapeInstance::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
{
	debugCallback.m_instance = this;
	m_shape->DebugShape(GetScaledTransform(matrix), debugCallback);
}

ndShapeInfo ndShapeInstance::GetShapeInfo() const
{
	ndShapeInfo info(m_shape->GetShapeInfo());
	info.m_offsetMatrix = m_localMatrix;
	//info.m_collisionMaterial = m_material;
	return info;
}

dMatrix ndShapeInstance::CalculateInertia() const
{
	ndShape* const shape = (ndShape*)m_shape;
	if (shape->GetAsShapeNull() || !shape->GetAsShapeConvex()) 
	{
		return dGetZeroMatrix();
	}
	else 
	{
		return m_shape->CalculateInertiaAndCenterOfMass(m_aligmentMatrix, m_scale, m_localMatrix);
	}
}

void ndShapeInstance::CalculateAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const
{
	m_shape->CalcAABB(matrix, p0, p1);
	switch (m_scaleType)
	{
		case m_unit:
		{
			p0 -= m_padding;
			p1 += m_padding;
			break;
		}

		case m_uniform:
		case m_nonUniform:
		{
			dMatrix matrix1(matrix);
			matrix1[0] = matrix1[0].Scale(m_scale.m_x);
			matrix1[1] = matrix1[1].Scale(m_scale.m_y);
			matrix1[2] = matrix1[2].Scale(m_scale.m_z);
			matrix1 = matrix.Inverse() * matrix1;

			dVector size0((p1 - p0) * dVector::m_half);
			dVector origin(matrix1.TransformVector((p0 + p1) * dVector::m_half));
			dVector size(matrix1.m_front.Abs().Scale(size0.m_x) + matrix1.m_up.Abs().Scale(size0.m_y) + matrix1.m_right.Abs().Scale(size0.m_z));

			p0 = (origin - size - m_padding) & dVector::m_triplexMask;
			p1 = (origin + size + m_padding) & dVector::m_triplexMask;
			break;
		}

		case m_global:
		default:
		{
			dAssert(0);
			//dMatrix matrix1(matrix);
			//matrix1[0] = matrix1[0].Scale(m_scale.m_x);
			//matrix1[1] = matrix1[1].Scale(m_scale.m_y);
			//matrix1[2] = matrix1[2].Scale(m_scale.m_z);
			//m_shape->CalcAABB(m_aligmentMatrix * matrix1, p0, p1);
			//p0 -= m_padding;
			//p1 += m_padding;
			break;
		}
	}

	dAssert(p0.m_w == dFloat32(0.0f));
	dAssert(p1.m_w == dFloat32(0.0f));
}

void ndShapeInstance::CalculateObb(dVector& origin, dVector& size) const
{
	size = m_shape->GetObbSize();
	origin = m_shape->GetObbOrigin();

	switch (m_scaleType)
	{
		case m_unit:
		{
			size += m_padding;
			break;
		}

		case m_uniform:
		case m_nonUniform:
		{
			size = size * m_scale + m_padding;
			origin = origin * m_scale;
			break;
		}
		case m_global:
		{
			//dMatrix matrix1 (matrix);
			//matrix1[0] = matrix1[0].Scale(m_scale.m_x);
			//matrix1[1] = matrix1[1].Scale(m_scale.m_y);
			//matrix1[2] = matrix1[2].Scale(m_scale.m_z);
			//m_shape->CalcAABB (m_aligmentMatrix * matrix1, p0, p1);
			//p0 -= m_padding;
			//p1 += m_padding;

			dVector p0;
			dVector p1;
			m_shape->CalcAABB(m_aligmentMatrix, p0, p1);
			size = (dVector::m_half * (p1 - p0) * m_scale + m_padding) & dVector::m_triplexMask;
			origin = (dVector::m_half * (p1 + p0) * m_scale) & dVector::m_triplexMask;;
			break;
		}
	}

	dAssert(size.m_w == dFloat32(0.0f));
	dAssert(origin.m_w == dFloat32(0.0f));
}


dFloat32 ndShapeInstance::RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, const ndBody* const body, ndContactPoint& contactOut) const
{
	dFloat32 t = dFloat32(1.2f);
	if (callback.OnRayPrecastAction(body, this))
	{
		switch (m_scaleType)
		{
			case m_unit:
			{
				t = m_shape->RayCast(callback, localP0, localP1, body, contactOut);
				if (t < dFloat32 (1.0f)) 
				{
					//dAssert(((ndShape*)m_shape)->GetAsShapeBox() || ((ndShape*)m_shape)->GetAsShapeSphere());
					dAssert(!((ndShape*)m_shape)->GetAsShapeCompound());
				//	if (!(m_shape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_shape->IsType(dgCollision::dgCollisionCompound_RTTI))) 
				//	{
				//		contactOut.m_shapeId0 = GetUserDataID();
				//		contactOut.m_shapeId1 = GetUserDataID();
				//	}
				//	if (!m_shape->IsType(dgCollision::dgCollisionCompound_RTTI)) 
				//	{
				//		contactOut.m_collision0 = this;
				//		contactOut.m_collision1 = this;
				//	}
					contactOut.m_shapeInstance0 = this;
					contactOut.m_shapeInstance1 = this;
				}
				break;
			}

			case m_uniform:
			{
				dVector p0(localP0 * m_invScale);
				dVector p1(localP1 * m_invScale);
				t = m_shape->RayCast(callback, p0, p1, body, contactOut);
				if (t < dFloat32(1.0f))
				{
					dAssert(!((ndShape*)m_shape)->GetAsShapeCompound());
				//	if (!(m_shape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_shape->IsType(dgCollision::dgCollisionCompound_RTTI))) 
				//	{
				//		contactOut.m_shapeId0 = GetUserDataID();
				//		contactOut.m_shapeId1 = GetUserDataID();
				//	}
				//	if (!m_shape->IsType(dgCollision::dgCollisionCompound_RTTI)) 
				//	{
				//		contactOut.m_collision0 = this;
				//		contactOut.m_collision1 = this;
				//	}
					contactOut.m_shapeInstance0 = this;
					contactOut.m_shapeInstance1 = this;
				}
				break;
			}

			case m_nonUniform:
			{
				dVector p0(localP0 * m_invScale);
				dVector p1(localP1 * m_invScale);
				t = m_shape->RayCast(callback, p0, p1, body, contactOut);
				if (t < dFloat32(1.0f))
				{
					dAssert(!((ndShape*)m_shape)->GetAsShapeCompound());
				//	if (!(m_shape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_shape->IsType(dgCollision::dgCollisionCompound_RTTI))) 
				//	{
				//		contactOut.m_shapeId0 = GetUserDataID();
				//		contactOut.m_shapeId1 = GetUserDataID();
				//		dVector n(m_invScale * contactOut.m_normal);
				//		contactOut.m_normal = n.Normalize();
				//	}
				//	if (!m_shape->IsType(dgCollision::dgCollisionCompound_RTTI)) 
				//	{
				//		contactOut.m_collision0 = this;
				//		contactOut.m_collision1 = this;
				//	}

					dVector normal(m_invScale * contactOut.m_normal);
					contactOut.m_normal = normal.Normalize();
					contactOut.m_shapeInstance0 = this;
					contactOut.m_shapeInstance1 = this;
				}
				break;
			}

			case m_global:
			default:
			{
				dVector p0(m_aligmentMatrix.UntransformVector(localP0 * m_invScale));
				dVector p1(m_aligmentMatrix.UntransformVector(localP1 * m_invScale));
				t = m_shape->RayCast(callback, p0, p1, body, contactOut);
				if (t < dFloat32(1.0f))
				{
					dAssert(!((ndShape*)m_shape)->GetAsShapeCompound());
				//	if (!(m_shape->IsType(dgCollision::dgCollisionMesh_RTTI) || m_shape->IsType(dgCollision::dgCollisionCompound_RTTI))) 
				//	{
				//		contactOut.m_shapeId0 = GetUserDataID();
				//		contactOut.m_shapeId1 = GetUserDataID();
				//		dVector n(m_aligmentMatrix.RotateVector(m_invScale * contactOut.m_normal));
				//		contactOut.m_normal = n.Normalize();
				//	}
				//	if (!(m_shape->IsType(dgCollision::dgCollisionCompound_RTTI))) 
				//	{
				//		contactOut.m_collision0 = this;
				//		contactOut.m_collision1 = this;
				//	}

					dVector normal(m_aligmentMatrix.RotateVector(m_invScale * contactOut.m_normal));
					contactOut.m_normal = normal.Normalize();
					contactOut.m_shapeInstance0 = this;
					contactOut.m_shapeInstance1 = this;
				}
				break;
			}
		}
	}
	return t;
}

dInt32 ndShapeInstance::CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const
{
	dInt32 count = 0;
	dAssert(normal.m_w == dFloat32(0.0f));
	switch (m_scaleType)
	{
		case m_unit:
		{
			count = m_shape->CalculatePlaneIntersection(normal, point, contactsOut);
			break;
		}
		case m_uniform:
		{
			dVector point1(m_invScale * point);
			count = m_shape->CalculatePlaneIntersection(normal, point1, contactsOut);
			for (dInt32 i = 0; i < count; i++) {
				contactsOut[i] = m_scale * contactsOut[i];
			}
			break;
		}

		case m_nonUniform:
		{
			// support((p * S), n) = S * support (p, n * transp(S)) 
			dVector point1(m_invScale * point);
			dVector normal1(m_scale * normal);
			normal1 = normal1.Normalize();
			count = m_shape->CalculatePlaneIntersection(normal1, point1, contactsOut);
			for (dInt32 i = 0; i < count; i++) {
				contactsOut[i] = m_scale * contactsOut[i];
			}
			break;
		}

		case m_global:
		default:
		{
			dVector point1(m_aligmentMatrix.UntransformVector(m_invScale * point));
			dVector normal1(m_aligmentMatrix.UntransformVector(m_scale * normal));
			normal1 = normal1.Normalize();
			count = m_shape->CalculatePlaneIntersection(normal1, point1, contactsOut);
			for (dInt32 i = 0; i < count; i++) {
				contactsOut[i] = m_scale * m_aligmentMatrix.TransformVector(contactsOut[i]);
			}
		}
	}
	return count;
}

void ndShapeInstance::SetScale(const dVector& scale)
{
	dFloat32 scaleX = dAbs(scale.m_x);
	dFloat32 scaleY = dAbs(scale.m_y);
	dFloat32 scaleZ = dAbs(scale.m_z);
	dAssert(scaleX > dFloat32(0.0f));
	dAssert(scaleY > dFloat32(0.0f));
	dAssert(scaleZ > dFloat32(0.0f));

	//if (IsType(dgCollision::dgCollisionCompound_RTTI)) 
	if (((ndShape*)m_shape)->GetAsShapeCompound())
	{
		dAssert(0);
		//dAssert(m_scaleType == m_unit);
		//dgCollisionCompound* const compound = (dgCollisionCompound*)m_shape;
		//compound->ApplyScale(scale);
	}
	else if ((dAbs(scaleX - scaleY) < dFloat32(1.0e-4f)) && (dAbs(scaleX - scaleZ) < dFloat32(1.0e-4f))) 
	{
		if ((dAbs(scaleX - dFloat32(1.0f)) < dFloat32(1.0e-4f))) 
		{
			m_scaleType = m_unit;
			m_scale = dVector(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f));
			m_maxScale = m_scale;
			m_invScale = m_scale;
		}
		else 
		{
			m_scaleType = m_uniform;
			m_scale = dVector(scaleX, scaleX, scaleX, dFloat32(0.0f));
			m_maxScale = m_scale;
			m_invScale = dVector(dFloat32(1.0f) / scaleX, dFloat32(1.0f) / scaleX, dFloat32(1.0f) / scaleX, dFloat32(0.0f));
		}
	}
	else 
	{
		m_scaleType = m_nonUniform;
		m_maxScale = dMax(scaleX, scaleY, scaleZ);
		m_scale = dVector(scaleX, scaleY, scaleZ, dFloat32(0.0f));
		m_invScale = dVector(dFloat32(1.0f) / scaleX, dFloat32(1.0f) / scaleY, dFloat32(1.0f) / scaleZ, dFloat32(0.0f));
	}
}

dFloat32 ndShapeInstance::CalculateBuoyancyCenterOfPresure(dVector& com, const dMatrix& matrix, const dVector& fluidPlane) const
{
	com = m_shape->CalculateVolumeIntegral(m_localMatrix * matrix, fluidPlane, *this);
	dFloat32 volume = com.m_w;
	com.m_w = dFloat32(0.0f);
	return volume;
}

void ndShapeInstance::Save(nd::TiXmlElement* const rootNode, const dTree<dUnsigned32, const ndShape*>& shapesCache) const
{
	nd::TiXmlElement* const paramNode = new nd::TiXmlElement("ndShapeInstance");
	rootNode->LinkEndChild(paramNode);

	dAssert(shapesCache.Find(m_shape));
	xmlSaveParam(paramNode, "localMatrix", m_localMatrix);
	xmlSaveParam(paramNode, "aligmentMatrix", m_aligmentMatrix);
	xmlSaveParam(paramNode, "scale", m_scale);
	xmlSaveParam(paramNode, "shapeNodeId", dInt32 (shapesCache.Find(m_shape)->GetInfo()));

	xmlSaveParam(paramNode, "skinThickness", m_skinThickness);
	xmlSaveParam(paramNode, "collisionMode", m_collisionMode ? 1 : 0);
	xmlSaveParam(paramNode, "materialID", m_shapeMaterial.m_userId);
	xmlSaveParam(paramNode, "materialUserData", dInt64(m_shapeMaterial.m_alignPad));
	for (dInt32 i = 0; i < sizeof(m_shapeMaterial.m_userParam) / sizeof(m_shapeMaterial.m_userParam[0]); i++)
	{
		char name[64];
		sprintf(name, "intData%d", i);
		xmlSaveParam(paramNode, name, dInt64(m_shapeMaterial.m_userParam[i].m_intData));
	}
}

dVector ndShapeInstance::GetBoxPadding()
{
	return m_padding;
}

bool ndShapeInstance::ndDistanceCalculator::ClosestPoint()
{
	ndContact contact;
	ndBodyKinematic body0;
	ndBodyKinematic body1;

	body0.SetCollisionShape(*m_shape0);
	body1.SetCollisionShape(*m_shape1);

	dMatrix matrix0(m_matrix0);
	dMatrix matrix1(m_matrix1);

	matrix0.m_posit = dVector::m_wOne;
	matrix1.m_posit = (m_matrix1.m_posit - m_matrix0.m_posit) | dVector::m_wOne;

	body0.SetMatrix(matrix0);
	body1.SetMatrix(matrix1);
	body0.SetMassMatrix(dVector::m_one);
	contact.SetBodies(&body0, &body1);

	ndShapeInstance& shape0 = body0.GetCollisionShape();
	ndShapeInstance& shape1 = body1.GetCollisionShape();
	shape0.SetGlobalMatrix(shape0.GetLocalMatrix() * body0.GetMatrix());
	shape1.SetGlobalMatrix(shape1.GetLocalMatrix() * body1.GetMatrix());

	ndContactSolver solver(&contact);
	bool ret = solver.CalculateClosestPoints();

	m_normal = solver.m_separatingVector;
	m_point0 = solver.m_closestPoint0 + m_matrix0.m_posit;
	m_point1 = solver.m_closestPoint1 + m_matrix0.m_posit;
	m_point0.m_w = dFloat32(1.0f);
	m_point1.m_w = dFloat32(1.0f);
	return ret;
}

//dInt32 ndShapeInstance::ClosestPoint(const dMatrix& matrix, const dVector& point, dVector& contactPoint) const
//{
//	return ndContactSolver::CalculatePointOnsurface(this, matrix, point, contactPoint);
//}
