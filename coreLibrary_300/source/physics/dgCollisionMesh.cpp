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


dgPolygonMeshDesc::dgPolygonMeshDesc(dgCollisionParamProxy& proxy, void* const userData)
	:dgFastAABBInfo()
	,m_boxDistanceTravelInMeshSpace(dgFloat32 (0.0f))
	,m_threadNumber(proxy.m_threadIndex)
	,m_faceCount(0)
	,m_vertexStrideInBytes(0)
	,m_skinThickness(proxy.m_skinThickness)
	,m_userData (userData)
	,m_objBody (proxy.m_referenceBody)
	,m_polySoupBody(proxy.m_floatingBody)
	,m_objCollision(proxy.m_referenceCollision)
	,m_polySoupCollision(proxy.m_floatingCollision)
	,m_vertex(NULL)
	,m_faceIndexCount(NULL)
	,m_faceVertexIndex(NULL)
	,m_faceIndexStart(NULL)
	,m_hitDistance(NULL)
	,m_maxT(dgFloat32 (1.0f))
	,m_doContinuesCollisionTest(proxy.m_continueCollision)
{
	dgAssert (m_polySoupCollision->IsType (dgCollision::dgCollisionMesh_RTTI));
	dgAssert (m_objCollision->IsType (dgCollision::dgCollisionConvexShape_RTTI));

	const dgMatrix& hullMatrix = m_objCollision->GetGlobalMatrix();
	const dgMatrix& soupMatrix = m_polySoupCollision->GetGlobalMatrix();
	proxy.m_matrix = hullMatrix * soupMatrix.Inverse();

	switch (m_objCollision->GetCombinedScaleType(m_polySoupCollision->GetScaleType()))
	{
		case dgCollisionInstance::m_unit:
		{
			dgMatrix& matrix = *this;
			matrix = proxy.m_matrix;
			SetTransposeAbsMatrix (matrix);

			const dgCollision* const collision = m_objCollision->GetChildShape();
			m_objCollision->CalcAABB (*this, m_p0, m_p1);
			m_posit += matrix.RotateVector (collision->GetObbOrigin());
			m_size = collision->GetObbSize() + dgCollisionInstance::m_padding;
			break;
		}

		case dgCollisionInstance::m_uniform:
		{
			dgMatrix& matrix = *this;
			matrix = proxy.m_matrix;

			const dgVector& meshInvScale = m_polySoupCollision->GetInvScale();

			matrix.m_posit = matrix.m_posit.CompProduct4(meshInvScale) | dgVector::m_wOne;
			SetTransposeAbsMatrix (matrix);

			dgMatrix scaleMatrix (meshInvScale.CompProduct4(m_front), meshInvScale.CompProduct4(m_up), meshInvScale.CompProduct4(m_right), m_posit);
			m_objCollision->CalcAABB (scaleMatrix, m_p0, m_p1);

			dgVector scale (m_objCollision->GetScale().CompProduct4(m_polySoupCollision->GetInvScale())); 
			const dgCollision* const collision = m_objCollision->GetChildShape();
			m_posit += matrix.RotateVector (collision->GetObbOrigin().CompProduct4(scale));
			m_size = collision->GetObbSize().CompProduct4(scale) + dgCollisionInstance::m_padding;
			break;
		}

		case dgCollisionInstance::m_nonUniform:
		{
			dgMatrix& matrix = *this;
			matrix = proxy.m_matrix;
			const dgVector& meshInvScale = m_polySoupCollision->GetInvScale();

			matrix.m_posit = matrix.m_posit.CompProduct4(meshInvScale) | dgVector::m_wOne;
			SetTransposeAbsMatrix (matrix);
			
			dgMatrix scaleMatrix (meshInvScale.CompProduct4(m_front), meshInvScale.CompProduct4(m_up), meshInvScale.CompProduct4(m_right), m_posit);
			m_objCollision->CalcAABB (scaleMatrix, m_p0, m_p1);

			//const dgCollision* const collision = m_objCollision->GetChildShape();

			dgMatrix obbScaledMatrix (scaleMatrix * matrix.Inverse());
			dgVector obbP0;
			dgVector obbP1;
			m_objCollision->CalcAABB (obbScaledMatrix, obbP0, obbP1);
			m_size = (obbP1 - obbP0).CompProduct4(dgVector::m_half);
			m_posit += matrix.RotateVector ((obbP1 + obbP0).CompProduct4(dgVector::m_half));
			break;
		}

		default:
		{
			dgAssert (0);
		}
	}
}



void dgPolygonMeshDesc::SortFaceArray ()
{
	dgInt32 stride = 8;
	if (m_faceCount >= 8) {
		dgInt32 stack[DG_MAX_COLLIDING_FACES][2];

		stack[0][0] = 0;
		stack[0][1] = m_faceCount - 1;
		dgInt32 stackIndex = 1;
		while (stackIndex) {
			stackIndex --;
			dgInt32 lo = stack[stackIndex][0];
			dgInt32 hi = stack[stackIndex][1];
			if ((hi - lo) > stride) {
				dgInt32 i = lo;
				dgInt32 j = hi;
				dgFloat32 dist = m_hitDistance[(lo + hi) >> 1];
				do {    
					while (m_hitDistance[i] < dist) i ++;
					while (m_hitDistance[j] > dist) j --;

					if (i <= j)	{
						dgSwap (m_hitDistance[i], m_hitDistance[j]);
						dgSwap (m_faceIndexStart[i], m_faceIndexStart[j]);
						dgSwap (m_faceIndexCount[i], m_faceIndexCount[j]);
						i++; 
						j--;
					}
				} while (i <= j);

				if (i < hi) {
					stack[stackIndex][0] = i;
					stack[stackIndex][1] = hi;
					stackIndex ++;
				}
				if (lo < j) {
					stack[stackIndex][0] = lo;
					stack[stackIndex][1] = j;
					stackIndex ++;
				}
				dgAssert (stackIndex < dgInt32 (sizeof (stack) / (2 * sizeof (stack[0][0]))));
			}
		}
	}

	stride = stride * 2;
	if (m_faceCount < stride) {
		stride = m_faceCount;
	}
	for (dgInt32 i = 1; i < stride; i ++) {
		if (m_hitDistance[i] < m_hitDistance[0]) {
			dgSwap (m_hitDistance[i], m_hitDistance[0]);
			dgSwap (m_faceIndexStart[i], m_faceIndexStart[0]);
			dgSwap (m_faceIndexCount[i], m_faceIndexCount[0]);
		}
	}

	for (dgInt32 i = 1; i < m_faceCount; i ++) {
		dgInt32 j = i;
		dgInt32 ptr = m_faceIndexStart[i];
		dgInt32 count = m_faceIndexCount[i];
		dgFloat32 dist = m_hitDistance[i];
		for ( ; dist < m_hitDistance[j - 1]; j --) {
			dgAssert (j > 0);
			m_hitDistance[j] = m_hitDistance [j-1];
			m_faceIndexStart[j] = m_faceIndexStart[j-1];
			m_faceIndexCount[j] = m_faceIndexCount[j-1];
		}
		m_hitDistance[j] = dist;
		m_faceIndexStart[j] = ptr;
		m_faceIndexCount[j] = count;
	}

#ifdef _DEBUG
	for (dgInt32 i = 0; i < m_faceCount - 1; i ++) {
		dgAssert (m_hitDistance[i] <= m_hitDistance[i+1]);
	}
#endif
}


dgCollisionMesh::dgCollisionMesh(dgWorld* const world, dgCollisionID type)
	:dgCollision(world->GetAllocator(), 0, type)
{
	m_rtti |= dgCollisionMesh_RTTI;
	m_debugCallback = NULL;
	SetCollisionBBox (dgVector (dgFloat32 (0.0f)), dgVector (dgFloat32 (0.0f)));
}

dgCollisionMesh::dgCollisionMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollision(world, deserialization, userData)
{
	dgAssert (m_rtti | dgCollisionMesh_RTTI);

	m_debugCallback = NULL;
	SetCollisionBBox (dgVector (dgFloat32 (0.0f)), dgVector (dgFloat32 (0.0f)));
}

dgCollisionMesh::~dgCollisionMesh()
{
}

void dgCollisionMesh::SetCollisionBBox (const dgVector& p0, const dgVector& p1)
{
	dgAssert (p0.m_x <= p1.m_x);
	dgAssert (p0.m_y <= p1.m_y);
	dgAssert (p0.m_z <= p1.m_z);

	m_boxSize = (p1 - p0).Scale4 (dgFloat32 (0.5f)) & dgVector::m_triplexMask;
	m_boxOrigin = (p1 + p0).Scale4 (dgFloat32 (0.5f)) & dgVector::m_triplexMask; 
}

dgInt32 dgCollisionMesh::CalculateSignature () const
{
	dgAssert (0);
	return 0;
}


dgInt32 dgCollisionMesh::CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut, dgFloat32 normalSign) const
{
	dgAssert (0);
	return 0;
}

void dgCollisionMesh::SetDebugCollisionCallback (dgCollisionMeshCollisionCallback debugCallback)
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




dgVector dgCollisionMesh::CalculateVolumeIntegral (const dgMatrix& globalMatrix, const dgVector& plane, const dgCollisionInstance& parentScale) const
{
	return dgVector (dgFloat32 (0.0f));
}


void dgCollisionMesh::DebugCollision (const dgMatrix& matrixPtr, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
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
	dgVector size (matrix.m_front.Abs().Scale4(m_boxSize.m_x) + matrix.m_up.Abs().Scale4(m_boxSize.m_y) + matrix.m_right.Abs().Scale4(m_boxSize.m_z));

	p0 = (origin - size) & dgVector::m_triplexMask;
	p1 = (origin + size) & dgVector::m_triplexMask;
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
	dgCollisionInstance tmpCastingInstance (*castingShape);
	tmpCastingInstance.SetGlobalMatrix(castingShape->GetLocalMatrix() * shapeMatrix);
	
	dgCollisionParamProxy proxy (NULL, &contactOut, threadId, true, true);

	proxy.m_continueCollision = true;
	proxy.m_skinThickness = dgFloat32 (0.0f);
	proxy.m_floatingBody = NULL;
	proxy.m_referenceBody = (dgBody*)referenceBody;
	proxy.m_referenceCollision = &tmpCastingInstance;
	proxy.m_floatingCollision = (dgCollisionInstance*)referenceCollision;

//	dgPolygonMeshDesc data (proxy, referenceCollision->GetUserData());
	dgPolygonMeshDesc data (proxy, NULL);

//	castingShape->CalcAABB (polySoupScaledMatrix, data.m_boxP0, data.m_b

	data.m_maxT = dgMin (maxT, dgFloat32 (1.0f));
	dgFloat32 maxTime = (maxT > dgFloat32 (1.0f)) ? dgFloat32 (1.0f) : maxT;
	dgVector distanceTravel (shapeVeloc.Scale4 (maxTime));
//	data.m_boxDistanceTravelInMeshSpace = referenceCollision->m_invScale.CompProduct4(soupMatrix.UnrotateVector(distanceTravel.CompProduct4(castingShape->m_invScale)));
	data.SetDistanceTravel (distanceTravel);

	polysoup->GetCollidingFaces (&data);

	dgCollisionConvexPolygon polygon (m_allocator);
	dgCollisionInstance polyInstance (*referenceCollision, &polygon);
	polyInstance.SetScale (dgVector (1.0f));

	polygon.m_vertex = data.m_vertex;
	polygon.m_stride = dgInt32 (data.m_vertexStrideInBytes / sizeof (dgFloat32));

	const dgInt32 stride = polygon.m_stride;
	const dgFloat32* const vertex = polygon.m_vertex;

	dgFloat32 maxPolyScale = referenceCollision->m_maxScale.m_x;

	const dgVector& scale = referenceCollision->GetScale();
	const dgVector& invScale = referenceCollision->GetInvScale();
	const dgMatrix& hullMatrix = tmpCastingInstance.GetGlobalMatrix();
	
	data.SortFaceArray();
	dgContactPoint tmpContact;
	dgFloat32 paramScale = maxT;
	dgInt32* const indexArray = (dgInt32*)data.m_faceVertexIndex;
	for (dgInt32 j = 0; (j < data.m_faceCount) && ((data.m_hitDistance[j] * paramScale) < maxT); j ++) {
		dgInt32 address = data.m_faceIndexStart[j];
		const dgInt32* const localIndexArray = &indexArray[address];

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

		const dgVector& origin = hullMatrix.m_posit;
		for (dgInt32 i = 0; i < polygon.m_count; i ++) {
			dgInt32 index = localIndexArray[i] * stride;
			polygon.m_localPoly[i] = (scale.CompProduct4(dgVector (&vertex[index])) - origin) & dgVector::m_triplexMask;
		}
		polyInstance.m_localMatrix.m_posit = referenceCollision->GetLocalMatrix().TransformVector(origin);
		polyInstance.m_globalMatrix.m_posit = referenceCollision->GetGlobalMatrix().TransformVector(origin);
		dgFloat32 t = polygon.ConvexRayCast (castingShape, shapeMatrix, shapeVeloc, maxT, tmpContact, referenceBody, &polyInstance, userData, threadId);
		if (t < maxT) {
			maxT = t;
			contactOut = tmpContact;
			contactOut.m_shapeId0 = polygon.m_faceId;
			contactOut.m_shapeId1 = polygon.m_faceId;
		}
	}

	return maxT;
}



