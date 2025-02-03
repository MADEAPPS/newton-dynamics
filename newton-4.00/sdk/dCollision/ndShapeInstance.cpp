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
#include "ndScene.h"
#include "ndContact.h"
#include "ndShapeInstance.h"
#include "ndRayCastNotify.h"
#include "ndBodyKinematic.h"
#include "ndShapeCompound.h"

ndVector ndShapeInstance::m_padding(D_MAX_SHAPE_AABB_PADDING, D_MAX_SHAPE_AABB_PADDING, D_MAX_SHAPE_AABB_PADDING, ndFloat32(0.0f));

ndShapeInstance::ndShapeInstance(ndShape* const shape)
	:ndContainersFreeListAlloc<ndShapeInstance>()
	,m_globalMatrix(ndGetIdentityMatrix())
	,m_localMatrix(ndGetIdentityMatrix())
	,m_alignmentMatrix(ndGetIdentityMatrix())
	,m_scale(ndVector::m_one & ndVector::m_triplexMask)
	,m_invScale(ndVector::m_one & ndVector::m_triplexMask)
	,m_maxScale(ndVector::m_one & ndVector::m_triplexMask)
	,m_shape(shape ? shape->AddRef() : shape)
	,m_ownerBody(nullptr)
	,m_subCollisionHandle(nullptr)
	,m_skinMargin(ndFloat32(0.0f))
	,m_scaleType(m_unit)
	,m_collisionMode(true)
{
	ndAssert(!m_shape || ndMemory::CheckMemory(m_shape));
}

ndShapeInstance::ndShapeInstance(const ndShapeInstance& instance)
	:ndContainersFreeListAlloc<ndShapeInstance>()
	,m_globalMatrix(instance.m_globalMatrix)
	,m_localMatrix(instance.m_localMatrix)
	,m_alignmentMatrix(instance.m_alignmentMatrix)
	,m_scale(instance.m_scale)
	,m_invScale(instance.m_invScale)
	,m_maxScale(instance.m_maxScale)
	,m_shapeMaterial(instance.m_shapeMaterial)
	,m_shape(instance.m_shape->AddRef())
	,m_ownerBody(instance.m_ownerBody)
	,m_subCollisionHandle(instance.m_subCollisionHandle)
	,m_skinMargin(instance.m_skinMargin)
	,m_scaleType(instance.m_scaleType)
	,m_collisionMode(instance.m_collisionMode)
{
	ndShapeCompound* const compound = ((ndShape*)m_shape)->GetAsShapeCompound();
	if (compound)
	{
		m_shape->Release();
		m_shape = new ndShapeCompound(*compound);
		m_shape->AddRef();
	}
	ndAssert(!m_shape || ndMemory::CheckMemory(m_shape));
}

ndShapeInstance::ndShapeInstance(const ndShapeInstance& instance, ndShape* const shape)
	:ndContainersFreeListAlloc<ndShapeInstance>()
	,m_globalMatrix(instance.m_globalMatrix)
	,m_localMatrix(instance.m_localMatrix)
	,m_alignmentMatrix(instance.m_alignmentMatrix)
	,m_scale(instance.m_scale)
	,m_invScale(instance.m_invScale)
	,m_maxScale(instance.m_maxScale)
	,m_shapeMaterial(instance.m_shapeMaterial)
	,m_shape(shape->AddRef())
	,m_ownerBody(instance.m_ownerBody)
	,m_subCollisionHandle(instance.m_subCollisionHandle)
	,m_skinMargin(instance.m_skinMargin)
	,m_scaleType(instance.m_scaleType)
	,m_collisionMode(instance.m_collisionMode)
{
	ndAssert(!m_shape || ndMemory::CheckMemory(m_shape));
}

ndShapeInstance::~ndShapeInstance()
{
	ndAssert(!m_shape || ndMemory::CheckMemory(m_shape));
	if (m_shape)
	{
		m_shape->Release();
	}
}

ndShape* ndShapeInstance::GetShape()
{
	return (ndShape*)m_shape;
}

const ndShape* ndShapeInstance::GetShape() const
{
	return m_shape;
}

const ndMatrix& ndShapeInstance::GetAlignmentMatrix() const
{
	return m_alignmentMatrix;
}

const ndMatrix& ndShapeInstance::GetLocalMatrix() const
{
	return m_localMatrix;
}

void ndShapeInstance::SetLocalMatrix(const ndMatrix& matrix)
{
	m_localMatrix = matrix;
}

const ndMatrix& ndShapeInstance::GetGlobalMatrix() const
{
	return m_globalMatrix;
}

void ndShapeInstance::SetGlobalMatrix(const ndMatrix& matrix)
{
	m_globalMatrix = matrix;
}

ndMatrix ndShapeInstance::GetScaledTransform(const ndMatrix& matrix) const
{
	ndMatrix scale(ndGetIdentityMatrix());
	scale[0][0] = m_scale.m_x;
	scale[1][1] = m_scale.m_y;
	scale[2][2] = m_scale.m_z;
	return m_alignmentMatrix * scale * m_localMatrix * matrix;
}

ndInt32 ndShapeInstance::GetConvexVertexCount() const
{
	return m_shape->GetConvexVertexCount();
}

ndVector ndShapeInstance::SupportVertex(const ndVector& inDir) const
{
	const ndVector dir(inDir & ndVector::m_triplexMask);
	ndAssert(dir.m_w == ndFloat32(0.0f));
	ndAssert(ndAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-2f));
	switch (m_scaleType)
	{
	case m_unit:
	{
		return m_shape->SupportVertex(dir);
	}
	case m_uniform:
	{
		return m_scale * m_shape->SupportVertex(dir);
	}
	case m_nonUniform:
	{
		// support((p * S), n) = S * support (p, n * transp(S)) 
		const ndVector dir1((m_scale * dir).Normalize());
		return m_scale * m_shape->SupportVertex(dir1);
	}

	case m_global:
	default:
	{
		const ndVector dir1(m_alignmentMatrix.UnrotateVector((m_scale * dir).Normalize()));
		return m_scale * m_alignmentMatrix.TransformVector(m_shape->SupportVertex(dir1));
	}
	}
}

ndVector ndShapeInstance::SupportVertexSpecial(const ndVector& inDir) const
{
	const ndVector dir(inDir & ndVector::m_triplexMask);
	ndAssert(dir.m_w == ndFloat32(0.0f));
	ndAssert(ndAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-2f));
	switch (m_scaleType)
	{
	case m_unit:
	{
		return m_shape->SupportVertexSpecial(dir, m_skinMargin);
	}
	case m_uniform:
	{
		return m_scale * m_shape->SupportVertexSpecial(dir, m_skinMargin);
	}

	case m_global:
	case m_nonUniform:
	default:
		return SupportVertex(dir);
	}
}

ndVector ndShapeInstance::SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& inDir) const
{
	const ndVector dir(inDir & ndVector::m_triplexMask);
	ndAssert(dir.m_w == ndFloat32(0.0f));
	ndAssert(ndAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-2f));
	switch (m_scaleType)
	{
	case m_unit:
	{
		return m_shape->SupportVertexSpecialProjectPoint(point, dir);
	}
	case m_uniform:
	{
		return m_scale * m_shape->SupportVertexSpecialProjectPoint(point * m_invScale, dir);
	}

	case m_global:
	case m_nonUniform:
	default:
		return point;

#if 0
	case m_nonUniform:
	{
		// support((p * S), n) = S * support (p/S, n * transp(S)) 
		dVector dir1((m_scale * dir).Normalize());
		return m_scale * m_shape->SupportVertexSpecialProjectPoint(point * m_invScale, dir1);
	}

	case m_global:
	default:
	{
		dVector dir1(m_alignmentMatrix.UnrotateVector((m_scale * dir).Normalize()));
		return m_scale * m_alignmentMatrix.TransformVector(m_shape->SupportVertexSpecialProjectPoint(m_alignmentMatrix.UntransformVector(point * m_invScale), dir1));
	}
#endif
	}
}

bool ndShapeInstance::GetCollisionMode() const
{
	return m_collisionMode;
}

void ndShapeInstance::SetCollisionMode(bool mode)
{
	m_collisionMode = mode;
}

const ndVector& ndShapeInstance::GetScale() const
{
	return m_scale;
}

const ndVector& ndShapeInstance::GetInvScale() const
{
	return m_invScale;
}

ndFloat32 ndShapeInstance::GetBoxMinRadius() const
{
	return m_shape->GetBoxMinRadius() * m_maxScale.m_x;
}

ndFloat32 ndShapeInstance::GetBoxMaxRadius() const
{
	return m_shape->GetBoxMaxRadius() * m_maxScale.m_x;
}

ndFloat32 ndShapeInstance::GetVolume() const
{
	return m_shape->GetVolume() * m_scale.m_x * m_scale.m_y * m_scale.m_z;
}

ndShapeMaterial ndShapeInstance::GetMaterial() const
{
	return m_shapeMaterial;
}

void ndShapeInstance::SetMaterial(const ndShapeMaterial& material)
{
	m_shapeMaterial = material;
}

ndShapeInstance::ndScaleType ndShapeInstance::GetScaleType() const
{
	return m_scaleType;
}

ndFloat32 ndShapeInstance::GetUmbraClipSize() const
{
	return m_shape->GetUmbraClipSize() * m_maxScale.m_x;
}

ndUnsigned64 ndShapeInstance::GetUserDataID() const
{
	return ndUnsigned64(m_shapeMaterial.m_userId);
}

void ndShapeInstance::SetShape(ndShape* const shape)
{
	if (m_shape)
	{
		m_shape->Release();
	}
	m_shape = shape ? shape->AddRef() : shape;
}

const char* ndShapeInstance::ClassName() const
{
	return "ndShapeInstance";
}

const char* ndShapeInstance::StaticClassName()
{
	return "ndShapeInstance";
}

const char* ndShapeInstance::SuperClassName() const
{
	return "ndShapeInstance";
}


void ndShapeInstance::SavePLY(const char* const fileName) const
{
	class ndDrawShape : public ndShapeDebugNotify
	{
		public:
		ndDrawShape()
			:ndShapeDebugNotify()
		{
		}

		virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceVertex, const ndEdgeType* const)
		{
			m_faceVertexCount.PushBack(vertexCount);
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				m_vertex.PushBack(faceVertex[i] & ndVector::m_triplexMask);
			}
		}

		ndArray<ndVector> m_vertex;
		ndArray<ndInt32> m_faceVertexCount;
	};

	ndDrawShape drawShapes;
	DebugShape(ndGetIdentityMatrix(), drawShapes);
	if (drawShapes.m_vertex.GetCount())
	{
		FILE* const file = fopen(fileName, "wb");
		fprintf(file, "ply\n");
		fprintf(file, "format ascii 1.0\n");

		fprintf(file, "element vertex %d\n", ndInt32(drawShapes.m_vertex.GetCount()));
		fprintf(file, "property float x\n");
		fprintf(file, "property float y\n");
		fprintf(file, "property float z\n");
		fprintf(file, "element face %d\n", ndInt32(drawShapes.m_faceVertexCount.GetCount()));
		fprintf(file, "property list uchar int vertex_index\n");
		fprintf(file, "end_header\n");

		for (ndInt32 i = 0; i < drawShapes.m_vertex.GetCount(); ++i)
		{
			const ndVector& point = drawShapes.m_vertex[i];
			fprintf(file, "%f %f %f\n", point.m_x, point.m_y, point.m_z);
		}

		ndInt32 index = 0;
		for (ndInt32 i = 0; i < drawShapes.m_faceVertexCount.GetCount(); ++i)
		{
			ndInt32 count = drawShapes.m_faceVertexCount[i];
			fprintf(file, "%d", count);
			for (ndInt32 j = 0; j < count; ++j)
			{
				fprintf(file, " %d", index + j);
			}
			index += count;
			fprintf(file, "\n");
		}
		fclose(file);
	}
}

ndShapeInstance& ndShapeInstance::operator=(const ndShapeInstance& instance)
{
	m_globalMatrix = instance.m_globalMatrix;
	m_localMatrix = instance.m_localMatrix;
	m_alignmentMatrix = instance.m_alignmentMatrix;
	m_scale = instance.m_scale;
	m_invScale = instance.m_invScale;
	m_maxScale = instance.m_maxScale;
	m_scaleType = instance.m_scaleType;
	m_shapeMaterial = instance.m_shapeMaterial;
	m_skinMargin = instance.m_skinMargin;
	m_collisionMode = instance.m_collisionMode;
	if (m_shape != nullptr)
	{
		m_shape->Release();
	}
	m_shape = instance.m_shape->AddRef();
	m_ownerBody = instance.m_ownerBody;

	m_subCollisionHandle = instance.m_subCollisionHandle;
	//m_parent____ = instance.m_parent____;

	return *this;
}

void ndShapeInstance::DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const
{
	debugCallback.m_instance = this;
	m_shape->DebugShape(GetScaledTransform(matrix), debugCallback);
}

ndShapeInfo ndShapeInstance::GetShapeInfo() const
{
	ndShapeInfo info(m_shape->GetShapeInfo());
	info.m_offsetMatrix = m_localMatrix;
	info.m_scale = m_scale;
	return info;
}

ndMatrix ndShapeInstance::CalculateInertia() const
{
	ndShape* const shape = (ndShape*)m_shape;
	if (shape->GetAsShapeNull() || !(shape->GetAsShapeConvex() || shape->GetAsShapeCompound()))
	{
		return ndGetZeroMatrix();
	}
	else 
	{
		return m_shape->CalculateInertiaAndCenterOfMass(m_alignmentMatrix, m_scale, m_localMatrix);
	}
}

void ndShapeInstance::CalculateObb(ndVector& origin, ndVector& size) const
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
			ndVector p0;
			ndVector p1;
			m_shape->CalculateAabb(m_alignmentMatrix, p0, p1);
			size = (ndVector::m_half * (p1 - p0) * m_scale + m_padding) & ndVector::m_triplexMask;
			origin = (ndVector::m_half * (p1 + p0) * m_scale) & ndVector::m_triplexMask;;
			break;
		}
	}

	ndAssert(size.m_w == ndFloat32(0.0f));
	ndAssert(origin.m_w == ndFloat32(0.0f));
}

ndFloat32 ndShapeInstance::RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, const ndBody* const body, ndContactPoint& contactOut) const
{
	ndFloat32 t = ndFloat32(1.2f);
	if (callback.OnRayPrecastAction(body, this))
	{
		switch (m_scaleType)
		{
			case m_unit:
			{
				t = m_shape->RayCast(callback, localP0, localP1, ndFloat32(1.0f), body, contactOut);
				if (t < ndFloat32 (1.0f)) 
				{
					contactOut.m_shapeInstance0 = this;
					contactOut.m_shapeInstance1 = this;
				}
				break;
			}

			case m_uniform:
			{
				ndVector p0(localP0 * m_invScale);
				ndVector p1(localP1 * m_invScale);
				t = m_shape->RayCast(callback, p0, p1, ndFloat32(1.0f), body, contactOut);
				if (t < ndFloat32(1.0f))
				{
					ndAssert(!((ndShape*)m_shape)->GetAsShapeCompound());
					contactOut.m_shapeInstance0 = this;
					contactOut.m_shapeInstance1 = this;
				}
				break;
			}

			case m_nonUniform:
			{
				ndVector p0(localP0 * m_invScale);
				ndVector p1(localP1 * m_invScale);
				t = m_shape->RayCast(callback, p0, p1, ndFloat32(1.0f), body, contactOut);
				if (t < ndFloat32(1.0f))
				{
					ndAssert(!((ndShape*)m_shape)->GetAsShapeCompound());
					ndVector normal(m_invScale * contactOut.m_normal);
					contactOut.m_normal = normal.Normalize();
					contactOut.m_shapeInstance0 = this;
					contactOut.m_shapeInstance1 = this;
				}
				break;
			}

			case m_global:
			default:
			{
				ndVector p0(m_alignmentMatrix.UntransformVector(localP0 * m_invScale));
				ndVector p1(m_alignmentMatrix.UntransformVector(localP1 * m_invScale));
				t = m_shape->RayCast(callback, p0, p1, ndFloat32(1.0f), body, contactOut);
				if (t < ndFloat32(1.0f))
				{
					ndAssert(!((ndShape*)m_shape)->GetAsShapeCompound());
					ndVector normal(m_alignmentMatrix.RotateVector(m_invScale * contactOut.m_normal));
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

ndInt32 ndShapeInstance::CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const
{
	ndInt32 count = 0;
	ndAssert(normal.m_w == ndFloat32(0.0f));
	switch (m_scaleType)
	{
		case m_unit:
		{
			count = m_shape->CalculatePlaneIntersection(normal, point, contactsOut);
			break;
		}
		case m_uniform:
		{
			ndVector point1(m_invScale * point);
			count = m_shape->CalculatePlaneIntersection(normal, point1, contactsOut);
			for (ndInt32 i = 0; i < count; ++i) 
			{
				contactsOut[i] = m_scale * contactsOut[i];
			}
			break;
		}

		case m_nonUniform:
		{
			// support((p * S), n) = S * support (p, n * transp(S)) 
			ndVector point1(m_invScale * point);
			ndVector normal1(m_scale * normal);
			normal1 = normal1.Normalize();
			count = m_shape->CalculatePlaneIntersection(normal1, point1, contactsOut);
			for (ndInt32 i = 0; i < count; ++i) 
			{
				contactsOut[i] = m_scale * contactsOut[i];
			}
			break;
		}

		case m_global:
		default:
		{
			ndVector point1(m_alignmentMatrix.UntransformVector(m_invScale * point));
			//ndVector normal1(m_alignmentMatrix.UntransformVector(m_scale * normal));
			ndVector normal1(m_alignmentMatrix.UnrotateVector(m_scale * normal));
			normal1 = normal1.Normalize();
			count = m_shape->CalculatePlaneIntersection(normal1, point1, contactsOut);
			for (ndInt32 i = 0; i < count; ++i) 
			{
				contactsOut[i] = m_scale * m_alignmentMatrix.TransformVector(contactsOut[i]);
			}
		}
	}
	return count;
}

void ndShapeInstance::SetScale(const ndVector& scale)
{
	ndFloat32 scaleX = ndAbs(scale.m_x);
	ndFloat32 scaleY = ndAbs(scale.m_y);
	ndFloat32 scaleZ = ndAbs(scale.m_z);
	ndAssert(scaleX > ndFloat32(0.0f));
	ndAssert(scaleY > ndFloat32(0.0f));
	ndAssert(scaleZ > ndFloat32(0.0f));

	if (((ndShape*)m_shape)->GetAsShapeCompound())
	{
		ndAssert(m_scaleType == m_unit);
		ndShapeCompound* const compound = ((ndShape*)m_shape)->GetAsShapeCompound();
		compound->ApplyScale(scale);
	}
	else if ((ndAbs(scaleX - scaleY) < ndFloat32(1.0e-4f)) && (ndAbs(scaleX - scaleZ) < ndFloat32(1.0e-4f))) 
	{
		if ((ndAbs(scaleX - ndFloat32(1.0f)) < ndFloat32(1.0e-4f))) 
		{
			m_scaleType = m_unit;
			m_scale = ndVector(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(0.0f));
			m_maxScale = m_scale;
			m_invScale = m_scale;
		}
		else 
		{
			m_scaleType = m_uniform;
			m_scale = ndVector(scaleX, scaleX, scaleX, ndFloat32(0.0f));
			m_maxScale = m_scale;
			m_invScale = ndVector(ndFloat32(1.0f) / scaleX, ndFloat32(1.0f) / scaleX, ndFloat32(1.0f) / scaleX, ndFloat32(0.0f));
		}
	}
	else 
	{
		m_scaleType = m_nonUniform;
		m_maxScale = ndMax(ndMax(scaleX, scaleY), scaleZ);
		m_scale = ndVector(scaleX, scaleY, scaleZ, ndFloat32(0.0f));
		m_invScale = ndVector(ndFloat32(1.0f) / scaleX, ndFloat32(1.0f) / scaleY, ndFloat32(1.0f) / scaleZ, ndFloat32(0.0f));
	}
}

void ndShapeInstance::SetGlobalScale(const ndMatrix& scaleMatrix)
{
	const ndMatrix matrix(scaleMatrix * m_localMatrix);

	ndVector scale;
	matrix.PolarDecomposition(m_localMatrix, scale, m_alignmentMatrix);
	bool uniform = (ndAbs(scale[0] - scale[1]) < ndFloat32(1.0e-4f)) && (ndAbs(scale[0] - scale[2]) < ndFloat32(1.0e-4f));
	if (uniform) 
	{
		SetScale(scale);
	}
	else
	{
		bool isIdentity = m_alignmentMatrix.TestIdentity();
		m_scaleType = isIdentity ? m_nonUniform : m_global;
		m_scale = scale;
		m_invScale = ndVector(ndFloat32(1.0f) / m_scale[0], ndFloat32(1.0f) / m_scale[1], ndFloat32(1.0f) / m_scale[2], ndFloat32(0.0f));
	}
}

void ndShapeInstance::SetGlobalScale(const ndVector& scale)
{
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix[0][0] = scale.m_x;
	matrix[1][1] = scale.m_y;
	matrix[2][2] = scale.m_z;
	SetGlobalScale(matrix);
}

ndFloat32 ndShapeInstance::CalculateBuoyancyCenterOfPresure(ndVector& com, const ndMatrix& matrix, const ndVector& fluidPlane) const
{
	com = m_shape->CalculateVolumeIntegral(m_localMatrix * matrix, fluidPlane, *this);
	ndFloat32 volume = com.m_w;
	com.m_w = ndFloat32(0.0f);
	return volume;
}

ndFloat32 ndShapeInstance::GetBoxPadding()
{
	return m_padding.m_x;
}

bool ndShapeInstance::ndDistanceCalculator::ClosestPoint()
{
	ndContact contact;
	ndBodyKinematic body0;
	ndBodyKinematic body1;

	body0.SetCollisionShape(*m_shape0);
	body1.SetCollisionShape(*m_shape1);

	ndMatrix matrix0(m_matrix0);
	ndMatrix matrix1(m_matrix1);

	matrix0.m_posit = ndVector::m_wOne;
	matrix1.m_posit = (m_matrix1.m_posit - m_matrix0.m_posit) | ndVector::m_wOne;

	body0.SetMatrix(matrix0);
	body1.SetMatrix(matrix1);
	body0.SetMassMatrix(ndVector::m_one);
	contact.SetBodies(&body0, &body1);

	ndShapeInstance& shape0 = body0.GetCollisionShape();
	ndShapeInstance& shape1 = body1.GetCollisionShape();
	shape0.SetGlobalMatrix(shape0.GetLocalMatrix() * body0.GetMatrix());
	shape1.SetGlobalMatrix(shape1.GetLocalMatrix() * body1.GetMatrix());

	ndContactSolver solver(&contact, m_scene->GetContactNotify(), m_scene->GetTimestep(), 0);
	bool ret = solver.CalculateClosestPoints();

	m_normal = solver.m_separatingVector;
	m_point0 = solver.m_closestPoint0 + m_matrix0.m_posit;
	m_point1 = solver.m_closestPoint1 + m_matrix0.m_posit;
	m_point0.m_w = ndFloat32(1.0f);
	m_point1.m_w = ndFloat32(1.0f);
	return ret;
}

void ndShapeInstance::CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const
{
	ndMatrix scaleMatrix;
	scaleMatrix[0] = matrix[0].Scale(m_scale.m_x);
	scaleMatrix[1] = matrix[1].Scale(m_scale.m_y);
	scaleMatrix[2] = matrix[2].Scale(m_scale.m_z);
	scaleMatrix[3] = matrix[3];
	scaleMatrix = m_alignmentMatrix * scaleMatrix;

	const ndVector size0(m_shape->GetObbSize());
	const ndVector origin(scaleMatrix.TransformVector(m_shape->GetObbOrigin()));
	const ndVector size(scaleMatrix.m_front.Abs().Scale(size0.m_x) + scaleMatrix.m_up.Abs().Scale(size0.m_y) + scaleMatrix.m_right.Abs().Scale(size0.m_z));

	p0 = (origin - size - m_padding) & ndVector::m_triplexMask;
	p1 = (origin + size + m_padding) & ndVector::m_triplexMask;
	ndAssert(p0.m_w == ndFloat32(0.0f));
	ndAssert(p1.m_w == ndFloat32(0.0f));
}
