/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgWorld.h"
#include "dgContact.h"
#include "dgCollisionMesh.h"
#include "dgCollisionConvexPolygon.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


dgCollisionMesh::dgCollisionMesh(dgWorld* const world, dgCollisionID type)
	:dgCollision(world->GetAllocator(), 0, type)
{
	m_rtti |= dgCollisionMesh_RTTI;
	m_debugCallback = NULL;
	SetCollisionBBox (dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)),
					  dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)));
}

dgCollisionMesh::dgCollisionMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollision(world, deserialization, userData)
{
	dgAssert (m_rtti | dgCollisionMesh_RTTI);

	m_debugCallback = NULL;
	SetCollisionBBox (dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)),
					  dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)));
}

dgCollisionMesh::~dgCollisionMesh()
{
}

void dgCollisionMesh::SetCollisionBBox (const dgVector& p0, const dgVector& p1)
{
	dgAssert (p0.m_x <= p1.m_x);
	dgAssert (p0.m_y <= p1.m_y);
	dgAssert (p0.m_z <= p1.m_z);

	m_boxSize = (p1 - p0).Scale3 (dgFloat32 (0.5f)); 
	m_boxOrigin = (p1 + p0).Scale3 (dgFloat32 (0.5f)); 

	m_boxSize.m_w = dgFloat32 (0.0f);
	m_boxOrigin.m_w = dgFloat32 (0.0f);
}

dgInt32 dgCollisionMesh::CalculateSignature () const
{
	dgAssert (0);
	return 0;
}


dgInt32 dgCollisionMesh::CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut) const
{
	dgAssert (0);
	return 0;
}

void dgCollisionMesh::SetCollisionCallback (dgCollisionMeshCollisionCallback debugCallback)
{
	m_debugCallback = debugCallback;
}




#ifdef DG_DEBUG_AABB
dgVector dgCollisionMesh::BoxSupportMapping  (const dgVector& dir) const
{
	return dgVector (dir.m_x < dgFloat32 (0.0f) ? m_p0.m_x : m_p1.m_x, 
					 dir.m_y < dgFloat32 (0.0f) ? m_p0.m_y : m_p1.m_y, 
					 dir.m_z < dgFloat32 (0.0f) ? m_p0.m_z : m_p1.m_z, dgFloat32 (0.0f));
}
#endif




dgVector dgCollisionMesh::CalculateVolumeIntegral (const dgMatrix& globalMatrix__, GetBuoyancyPlane buoyancuPlane__, void* context__) const
{
	return dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
}


void dgCollisionMesh::DebugCollision (const dgMatrix& matrixPtr, OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgAssert (0);
}


dgFloat32 dgCollisionMesh::GetVolume () const
{
//	dgAssert (0);
	return dgFloat32 (0.0f); 
}

dgFloat32 dgCollisionMesh::GetBoxMinRadius () const
{
	return dgFloat32 (0.0f);  
}

dgFloat32 dgCollisionMesh::GetBoxMaxRadius () const
{
	return dgFloat32 (0.0f);  
}



void dgCollisionMesh::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgAssert (0);
	dgCollision::GetCollisionInfo(info);
//	info->m_offsetMatrix = GetLocalMatrix();
}

void dgCollisionMesh::Serialize(dgSerialize callback, void* const userData) const
{
	dgAssert (0);
}

dgVector dgCollisionMesh::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (0);
	return dgVector (0, 0, 0, 0);
}


void dgCollisionMesh::CalcAABB(const dgMatrix& matrix, dgVector &p0, dgVector &p1) const
{
	dgVector origin (matrix.TransformVector(m_boxOrigin));
	//dgVector size (m_boxSize.m_x * dgAbsf(matrix[0][0]) + m_boxSize.m_y * dgAbsf(matrix[1][0]) + m_boxSize.m_z * dgAbsf(matrix[2][0]),  
	//			     m_boxSize.m_x * dgAbsf(matrix[0][1]) + m_boxSize.m_y * dgAbsf(matrix[1][1]) + m_boxSize.m_z * dgAbsf(matrix[2][1]),  
	//			     m_boxSize.m_x * dgAbsf(matrix[0][2]) + m_boxSize.m_y * dgAbsf(matrix[1][2]) + m_boxSize.m_z * dgAbsf(matrix[2][2]),
	//	             dgFloat32 (0.0f));
	dgVector size (matrix.m_front.Abs().Scale4(m_boxSize.m_x) + matrix.m_up.Abs().Scale4(m_boxSize.m_y) + matrix.m_right.Abs().Scale4(m_boxSize.m_z));

	p0 = (origin - size) & dgVector::m_triplexMask;
	p1 = (origin + size) & dgVector::m_triplexMask;

	p0.m_w = dgFloat32 (0.0f);
	p1.m_w = dgFloat32 (0.0f);

#ifdef DG_DEBUG_AABB
	dgInt32 i;
	dgVector q0;
	dgVector q1;
	dgMatrix trans (matrix.Transpose());
	for (i = 0; i < 3; i ++) {
		q0[i] = matrix.m_posit[i] + matrix.RotateVector (BoxSupportMapping(trans[i].Scale3 (-1.0f)))[i];
		q1[i] = matrix.m_posit[i] + matrix.RotateVector (BoxSupportMapping(trans[i]))[i];
	}

	dgVector err0 (p0 - q0);
	dgVector err1 (p1 - q1);
	dgFloat32 err; 
	err = dgMax (size.m_x, size.m_y, size.m_z) * 0.5f; 
	dgAssert ((err0 % err0) < err);
	dgAssert ((err1 % err1) < err);
#endif
}


dgInt32 dgCollisionMesh::CalculatePlaneIntersection (const dgFloat32* const vertex, const dgInt32* const index, dgInt32 indexCount, dgInt32 stride, const dgPlane& localPlane, dgVector* const contactsOut) const
{
	dgInt32 count = 0;
	dgInt32 j = index[indexCount - 1] * stride;
	dgVector p0 (&vertex[j]);
	dgFloat32 side0 = localPlane.Evalue (p0);
	for (dgInt32 i = 0; i < indexCount; i ++) {
		dgInt32 j = index[i] * stride;
		dgVector p1 (&vertex[j]);
		dgFloat32 side1 = localPlane.Evalue (p1);

		if (side0 < dgFloat32 (0.0f)) {
			if (side1 >= dgFloat32 (0.0f)) {
				dgVector dp (p1 - p0);
				dgFloat32 t = localPlane % dp;
				dgAssert (dgAbsf (t) >= dgFloat32 (0.0f));
				if (dgAbsf (t) < dgFloat32 (1.0e-8f)) {
					t = dgSign(t) * dgFloat32 (1.0e-8f);	
				}
				dgAssert (0);
				contactsOut[count] = p0 - dp.Scale3 (side0 / t);
				count ++;

			} 
		} else if (side1 <= dgFloat32 (0.0f)) {
			dgVector dp (p1 - p0);
			dgFloat32 t = localPlane % dp;
			dgAssert (dgAbsf (t) >= dgFloat32 (0.0f));
			if (dgAbsf (t) < dgFloat32 (1.0e-8f)) {
				t = dgSign(t) * dgFloat32 (1.0e-8f);	
			}
			dgAssert (0);
			contactsOut[count] = p0 - dp.Scale3 (side0 / t);
			count ++;
		}

		side0 = side1;
		p0 = p1;
	}

	return count;
}


dgFloat32 dgCollisionMesh::ConvexRayCast (const dgCollisionInstance* const castingShape, const dgMatrix& shapeMatrix, const dgVector& shapeVeloc, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const referenceBody, const dgCollisionInstance* const referenceCollision, void* const userData, dgInt32 threadId) const
{
	dgAssert (castingShape->IsType (dgCollision::dgCollisionConvexShape_RTTI));
	dgAssert (referenceCollision->IsType (dgCollision::dgCollisionMesh_RTTI));
	dgAssert (referenceCollision->GetChildShape() == this);

	dgCollisionMesh* const polysoup = (dgCollisionMesh *) referenceCollision->GetChildShape();

	dgMatrix hullMatrix (castingShape->m_localMatrix * shapeMatrix);
	const dgMatrix& soupMatrix = referenceCollision->m_globalMatrix;

	dgMatrix matrix (hullMatrix * soupMatrix.Inverse());

	const dgVector& scale = referenceCollision->m_scale;
	const dgVector& invScale = referenceCollision->m_invScale;

	dgMatrix polySoupScaledMatrix (invScale.CompProduct4(matrix[0]), invScale.CompProduct4(matrix[1]), invScale.CompProduct4(matrix[2]), invScale.CompProduct4(matrix[3])); 
	dgPolygonMeshDesc polyMeshData;
	dgPolygonMeshDesc data;
	castingShape->CalcAABB (polySoupScaledMatrix, data.m_boxP0, data.m_boxP1);
	data.m_vertex = NULL;
	data.m_threadNumber = threadId;
	data.m_faceCount = 0;
	data.m_vertexStrideInBytes = 0;
	data.m_skinThickness = dgFloat32 (0.0f);
	data.m_faceIndexCount = NULL;
	data.m_faceVertexIndex = NULL;
	data.m_userData = referenceCollision->GetUserData();
//	data.m_objBody = hullBody;
//	data.m_polySoupBody = soupBody;
	data.m_objBody = NULL;
	data.m_polySoupBody = NULL;
	data.m_objCollision = (dgCollisionInstance*)castingShape;
	data.m_polySoupCollision = (dgCollisionInstance*)referenceCollision;
//	data.m_meshToShapeMatrix = castingShape->CalculateSpaceMatrix(referenceCollision);

	data.m_doContinuesCollisionTest = true;
	dgVector distanceTravel (shapeVeloc.Scale4 ((maxT > dgFloat32 (1.0f)) ? dgFloat32 (1.0f) : maxT));
	data.m_boxDistanceTravelInMeshSpace = referenceCollision->m_invScale.CompProduct4(soupMatrix.UnrotateVector(distanceTravel.CompProduct4(castingShape->m_invScale)));
//	data.m_distanceTravelInCollidingShapeSpace = castingShape->m_invScale.CompProduct4(hullMatrix.UnrotateVector(distanceTravel.CompProduct4(referenceCollision->m_invScale)));

	polysoup->GetCollidingFaces (&data);

	dgCollisionConvexPolygon polygon (m_allocator);
	dgCollisionInstance polyInstance (*referenceCollision, &polygon);
	polyInstance.SetScale (dgVector (1.0f));

	polygon.m_vertex = data.m_vertex;
	polygon.m_stride = dgInt32 (data.m_vertexStrideInBytes / sizeof (dgFloat32));

	const dgInt32 stride = polygon.m_stride;
	const dgFloat32* const vertex = polygon.m_vertex;

	dgFloat32 maxPolyScale = referenceCollision->m_maxScale.m_x;
	dgInt32 indexCount = 0;
	dgInt32* const indexArray = (dgInt32*)data.m_faceVertexIndex;

	for (dgInt32 j = 0; j < data.m_faceCount; j ++) {
		const dgInt32* const localIndexArray = &indexArray[indexCount];

		polygon.m_vertexIndex = localIndexArray;
		polygon.m_count = data.m_faceIndexCount[j];
		polygon.m_adjacentFaceEdgeNormalIndex = data.GetAdjacentFaceEdgeNormalArray (localIndexArray, polygon.m_count);
		polygon.m_faceId = data.GetFaceId (localIndexArray, polygon.m_count);
		polygon.m_faceClipSize = data.GetFaceSize (localIndexArray, polygon.m_count) * maxPolyScale;
		polygon.m_faceNormalIndex = data.GetNormalIndex (localIndexArray, polygon.m_count);
		dgVector normal (&vertex[polygon.m_faceNormalIndex * polygon.m_stride]);
		normal = invScale.CompProduct4(normal);
		polygon.m_normal = normal.Scale4(dgRsqrt (normal % normal));
		dgAssert (polygon.m_normal.m_w == dgFloat32 (0.0f));

		for (dgInt32 i = 0; i < polygon.m_count; i ++) {
			dgInt32 index = localIndexArray[i] * stride;
			polygon.m_localPoly[i] = scale.CompProduct4(dgVector (&vertex[index]));
			dgAssert (polygon.m_localPoly[i].m_w == dgFloat32 (0.0f));
		}

		dgFloat32 t = polygon.ConvexRayCast (castingShape, shapeMatrix, shapeVeloc, maxT, contactOut, referenceBody, &polyInstance, userData, threadId);
		if (t < maxT) {
			maxT = t;
		}
		indexCount += data.GetFaceIndexCount (data.m_faceIndexCount[j]);
	}

	return  maxT;
}

