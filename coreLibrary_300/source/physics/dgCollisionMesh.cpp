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


void dgCollisionMesh::CalcAABB(const dgMatrix &matrix, dgVector &p0, dgVector &p1) const
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


dgFloat32 dgCollisionMesh::ConvexRayCast (const dgCollisionInstance* const castingShape, const dgMatrix& shapeMatrix, const dgVector& shapeVeloc, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const referenceBody, const dgCollisionInstance* const referenceCollision, void* const userData) const
{
return 0;
//	dgInt32 count = 0;
//	dgAssert (proxy.m_floatingCollision->IsType (dgCollision::dgCollisionMesh_RTTI));
//	dgAssert (proxy.m_referenceCollision->IsType (dgCollision::dgCollisionConvexShape_RTTI));
	dgAssert (castingShape->IsType (dgCollision::dgCollisionConvexShape_RTTI));
	dgAssert (referenceCollision->IsType (dgCollision::dgCollisionMesh_RTTI));
	dgAssert (referenceCollision->GetChildShape() == this);

//	dgCollisionInstance* const polySoupInstance = proxy.m_floatingCollision;
//	dgCollisionInstance* const convexHullInstance = proxy.m_referenceCollision;
//	const dgPolygonMeshDesc& data = *proxy.m_polyMeshData;

	dgMatrix hullMatrix (castingShape->m_localMatrix * shapeMatrix);
	const dgMatrix& soupMatrix = referenceCollision->m_globalMatrix;

	dgMatrix matrix (hullMatrix * soupMatrix.Inverse());
	const dgVector& invScale = referenceCollision->m_invScale;
	dgMatrix polySoupScaledMatrix (invScale.CompProduct4(matrix[0]), invScale.CompProduct4(matrix[1]), invScale.CompProduct4(matrix[2]), invScale.CompProduct4(matrix[3])); 
	dgPolygonMeshDesc polyMeshData;
	dgPolygonMeshDesc data;
	castingShape->CalcAABB (polySoupScaledMatrix, data.m_boxP0, data.m_boxP1);
//	data.m_vertex = NULL;
//	data.m_threadNumber = proxy.m_threadIndex;
//	data.m_faceCount = 0;
//	data.m_vertexStrideInBytes = 0;
//	data.m_skinThickness = proxy.m_skinThickness;
//	data.m_faceIndexCount = NULL;
//	data.m_faceVertexIndex = NULL;
//	data.m_userData = polySoupInstance->GetUserData();
//	data.m_objBody = hullBody;
//	data.m_polySoupBody = soupBody;
//	data.m_objCollision = convexInstance;
//	data.m_polySoupCollision = polySoupInstance;
//	data.m_meshToShapeMatrix = convexInstance->CalculateSpaceMatrix(polySoupInstance);


/*
	dgAssert (data.m_faceCount); 
	dgInt32* const indexArray = (dgInt32*)data.m_faceVertexIndex;

	dgCollisionConvexPolygon polygon (m_allocator);
	dgCollisionInstance polyInstance (*polySoupInstance, &polygon);
	polyInstance.SetScale (dgVector (1.0f));
	proxy.m_floatingCollision = &polyInstance;

	polygon.m_vertex = data.m_vertex;
	polygon.m_stride = dgInt32 (data.m_vertexStrideInBytes / sizeof (dgFloat32));

	dgInt32 indexCount = 0;
	dgInt32 maxContacts = proxy.m_maxContacts;
	dgInt32 maxReduceLimit = maxContacts >> 2;
	dgInt32 countleft = maxContacts;

	dgAssert (proxy.m_contactJoint);
	dgVector separatingVector (proxy.m_matrix.m_posit);
	dgFloat32 mag2 = separatingVector % separatingVector;
	if (mag2 > dgFloat32 (0.0f)) {
		separatingVector = separatingVector.Scale3 (dgRsqrt (mag2));
	} else {
		separatingVector =  dgVector(dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	}

	const dgInt32 stride = polygon.m_stride;
	const dgFloat32* const vertex = polygon.m_vertex;
	const dgVector& scale = polySoupInstance->m_scale;
	const dgVector& invScale = polySoupInstance->m_invScale;

	dgFloat32 maxPolyScale = polySoupInstance->m_maxScale.m_x;

	polyInstance.m_scaleIsUnit = true;
	polyInstance.m_scaleIsUniform = true;

	dgVector* const face = polygon.m_localPoly;
	dgContactPoint* const contactOut = proxy.m_contacts;
	dgContact* const contactJoint = proxy.m_contactJoint;
	dgFloat32 closestDist = dgFloat32 (1.0e10f);

	dgCollisionConvex* const convexShape = (dgCollisionConvex*) convexHullInstance->m_childShape;

	dgUnsigned64 shapeFaceID = dgUnsigned64(-1);
	dgVector n (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector p (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgVector q (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	
	dgFloat32 saveTimeStep = proxy.m_timestep;
	dgFloat32 epsilon = dgFloat32 (-1.0e-3f) * proxy.m_timestep;
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
		polygon.m_normal = normal.Scale3(dgRsqrt (normal % normal));
		polygon.m_normal.m_w = dgFloat32 (0.0f);
		contactJoint->m_separtingVector = separatingVector;
		for (dgInt32 i = 0; i < polygon.m_count; i ++) {
			dgVector p (&vertex[localIndexArray[i] * stride]);
			face[i] = scale.CompProduct4(p);
		}

		proxy.m_maxContacts = countleft;
		proxy.m_contacts = &contactOut[count];
		proxy.m_shapeFaceID = polygon.m_faceId;
		dgInt32 count1 = convexShape->CalculateConvexCastContacts (proxy);
		if (count1 > 0) {
			dgFloat32 error = proxy.m_timestep - saveTimeStep;
			if (error < epsilon) {
				count = 0;
				countleft = maxContacts;
				for (dgInt32 i = 0; i < count1; i ++) {
					contactOut[i] = proxy.m_contacts[i];
				}
			}
			count += count1;
			countleft -= count1;
			dgAssert (countleft >= 0); 
			if (count >= maxReduceLimit) {
				count = ReduceContacts (count, contactOut, maxReduceLimit >> 1, dgFloat32 (1.0e-2f));
				//countleft = proxy.m_maxContacts - count;
				countleft = maxContacts - count;
				dgAssert (countleft >= 0); 
			}
		}

		if (proxy.m_timestep <= saveTimeStep) {
			saveTimeStep = proxy.m_timestep;
			n = proxy.m_normal;
			p = proxy.m_closestPointBody0;
			q = proxy.m_closestPointBody1;
			shapeFaceID = proxy.m_shapeFaceID;
		}

		indexCount += data.GetFaceIndexCount (data.m_faceIndexCount[j]);
	}

	proxy.m_contacts = contactOut;
	contactJoint->m_closestDistance = closestDist;

	// restore the pointer
	proxy.m_normal = n;
	proxy.m_closestPointBody0 = p;
	proxy.m_closestPointBody1 = q;
	proxy.m_floatingCollision = polySoupInstance;
	proxy.m_shapeFaceID = shapeFaceID;
	return count;
*/

	return  maxT;
}

