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
#include "ndContactSolver.h"
#include "ndBodyKinematic.h"
#include "ndShapeInstance.h"
#include "ndPolygonMeshDesc.h"

//ndPolygonMeshDesc::ndPolygonMeshDesc()
//	:ndFastAabb()
//	,m_boxDistanceTravelInMeshSpace(ndVector::m_zero)
//	,m_maxT(ndFloat32(1.0f))
//	,m_threadId(0)
//	,m_ownTempBuffers(false)
//	,m_doContinueCollisionTest(false)
//{
//}

ndPolygonMeshDesc::ndPolygonMeshDesc(ndContactSolver& proxy, bool ccdMode)
	:ndFastAabb()
	,m_boxDistanceTravelInMeshSpace(ndVector::m_zero)
	,m_vertexStrideInBytes(0)
	,m_skinMargin(proxy.m_skinMargin)
	,m_convexInstance(&proxy.m_instance0)
	,m_polySoupInstance(&proxy.m_instance1)
	,m_vertex(nullptr)
	,m_staticMeshQuery(nullptr)
	,m_proceduralStaticMeshFaceQuery(nullptr)
	,m_maxT(ndFloat32(1.0f))
	,m_threadId(proxy.m_threadId)
	//,m_ownTempBuffers(false)
	,m_doContinueCollisionTest(ccdMode)
{
	ndAssert(proxy.m_notification->m_scene);
	//ndScene* const scene = proxy.m_contact->GetBody0()->GetScene();
	//if (scene)
	//{
	//	m_staticMeshQuery = &scene->m_staticMeshQuery[proxy.m_threadId];
	//	m_proceduralStaticMeshFaceQuery = &scene->m_proceduralStaticMeshQuery[proxy.m_threadId];
	//}
	//else
	//{
	//	m_ownTempBuffers = true;
	//	m_staticMeshQuery = new ndStaticMeshFaceQuery;
	//	m_proceduralStaticMeshFaceQuery = new ndProceduralStaticMeshFaceQuery;
	//}

	ndScene* const scene = proxy.m_notification->m_scene;
	m_staticMeshQuery = &scene->m_staticMeshQuery[proxy.m_threadId];
	m_proceduralStaticMeshFaceQuery = &scene->m_proceduralStaticMeshQuery[proxy.m_threadId];
	Init();
}

ndPolygonMeshDesc::~ndPolygonMeshDesc()
{
	//if (m_ownTempBuffers)
	//{
	//	delete m_proceduralStaticMeshFaceQuery;
	//	delete m_staticMeshQuery;
	//}
}

void ndPolygonMeshDesc::Init()
{
	const ndMatrix& hullMatrix = m_convexInstance->GetGlobalMatrix();
	const ndMatrix& soupMatrix = m_polySoupInstance->GetGlobalMatrix();

	ndMatrix& matrix = *this;
	matrix = hullMatrix * soupMatrix.OrthoInverse();
	ndMatrix convexMatrix(ndGetIdentityMatrix());

	m_staticMeshQuery->Reset();
	m_proceduralStaticMeshFaceQuery->Reset();
	switch (m_polySoupInstance->GetScaleType())
	{
		case ndShapeInstance::m_unit:
		{
			break;
		}

		case ndShapeInstance::m_uniform:
		{
			const ndVector& invScale = m_polySoupInstance->GetInvScale();
			convexMatrix[0][0] = invScale.GetScalar();
			convexMatrix[1][1] = invScale.GetScalar();
			convexMatrix[2][2] = invScale.GetScalar();
			matrix.m_posit = matrix.m_posit * (invScale | ndVector::m_wOne);
			break;
		}

		case ndShapeInstance::m_nonUniform:
		{
			const ndVector& invScale = m_polySoupInstance->GetInvScale();
			ndMatrix tmp(matrix[0] * invScale, matrix[1] * invScale, matrix[2] * invScale, ndVector::m_wOne);
			convexMatrix = tmp * matrix.OrthoInverse();
			convexMatrix.m_posit = ndVector::m_wOne;
			matrix.m_posit = matrix.m_posit * (invScale | ndVector::m_wOne);
			break;
		}

		case ndShapeInstance::m_global:
		default:
		{
			ndAssert(0);
		}
	}

	ndMatrix fullMatrix(convexMatrix * matrix);
	m_convexInstance->CalculateAabb(fullMatrix, m_p0, m_p1);

	ndVector p0;
	ndVector p1;
	SetTransposeAbsMatrix(matrix);
	m_convexInstance->CalculateAabb(convexMatrix, p0, p1);
	m_size = ndVector::m_half * (p1 - p0);
	m_posit = matrix.TransformVector(ndVector::m_half * (p1 + p0));
	ndAssert(m_posit.m_w == ndFloat32(1.0f));
}

ndInt32 ndPolygonMeshDesc::GetFaceIndexCount(ndInt32 indexCount) const
{
	return indexCount * 2 + 3;
}

const ndInt32* ndPolygonMeshDesc::GetAdjacentFaceEdgeNormalArray(const ndInt32* const faceIndexArray, ndInt32 indexCount) const
{
	return &faceIndexArray[indexCount + 2];
}

ndInt32 ndPolygonMeshDesc::GetNormalIndex(const ndInt32* const faceIndexArray, ndInt32 indexCount) const
{
	return faceIndexArray[indexCount + 1];
}

ndInt32 ndPolygonMeshDesc::GetFaceId(const ndInt32* const faceIndexArray, ndInt32 indexCount) const
{
	return faceIndexArray[indexCount];
}

ndFloat32 ndPolygonMeshDesc::GetFaceSize(const ndInt32* const faceIndexArray, ndInt32 indexCount) const
{
	ndInt32 size = faceIndexArray[indexCount * 2 + 2];
	ndAssert(size >= 1);
	return D_FACE_CLIP_DIAGONAL_SCALE * ndFloat32(size);
}

ndFloat32 ndPolygonMeshDesc::GetSeparetionDistance() const
{
	return m_separationDistance[0] * m_polySoupInstance->GetScale().GetScalar();
}

void ndPolygonMeshDesc::SetDistanceTravel(const ndVector& distanceInGlobalSpace)
{
	const ndMatrix& soupMatrix = m_polySoupInstance->GetGlobalMatrix();
	m_boxDistanceTravelInMeshSpace = m_polySoupInstance->GetInvScale() * soupMatrix.UnrotateVector(distanceInGlobalSpace * m_convexInstance->GetInvScale());
	if (m_boxDistanceTravelInMeshSpace.DotProduct(m_boxDistanceTravelInMeshSpace).GetScalar() < ndFloat32(1.0e-2f))
	{
		m_doContinueCollisionTest = false;
	}
}

void ndPolygonMeshDesc::SortFaceArray()
{
	ndPolygonMeshDesc::ndStaticMeshFaceQuery& query = *m_staticMeshQuery;
	ndInt32 stride = 8;
	if (query.m_faceIndexCount.GetCount() >= 8)
	{
		ndInt32 stack[256][2];

		stack[0][0] = 0;
		stack[0][1] = ndInt32(query.m_faceIndexCount.GetCount() - 1);
		ndInt32 stackIndex = 1;
		while (stackIndex)
		{
			stackIndex--;
			ndInt32 lo = stack[stackIndex][0];
			ndInt32 hi = stack[stackIndex][1];
			if ((hi - lo) > stride)
			{
				ndInt32 i = lo;
				ndInt32 j = hi;
				ndFloat32 dist = query.m_hitDistance[(lo + hi) >> 1];
				do
				{
					while (query.m_hitDistance[i] < dist) i++;
					while (query.m_hitDistance[j] > dist) j--;

					if (i <= j)
					{
						ndSwap(query.m_hitDistance[i], query.m_hitDistance[j]);
						ndSwap(query.m_faceIndexStart[i], query.m_faceIndexStart[j]);
						ndSwap(query.m_faceIndexCount[i], query.m_faceIndexCount[j]);
						i++;
						j--;
					}
				} while (i <= j);

				if (i < hi)
				{
					stack[stackIndex][0] = i;
					stack[stackIndex][1] = hi;
					stackIndex++;
				}
				if (lo < j) {
					stack[stackIndex][0] = lo;
					stack[stackIndex][1] = j;
					stackIndex++;
				}
				ndAssert(stackIndex < ndInt32(sizeof(stack) / (2 * sizeof(stack[0][0]))));
			}
		}
	}

	stride = stride * 2;
	if (query.m_faceIndexCount.GetCount() < stride)
	{
		stride = ndInt32(query.m_faceIndexCount.GetCount());
	}
	for (ndInt32 i = 1; i < stride; ++i)
	{
		if (query.m_hitDistance[i] < query.m_hitDistance[0])
		{
			ndSwap(query.m_hitDistance[i], query.m_hitDistance[0]);
			ndSwap(query.m_faceIndexStart[i], query.m_faceIndexStart[0]);
			ndSwap(query.m_faceIndexCount[i], query.m_faceIndexCount[0]);
		}
	}

	for (ndInt32 i = 1; i < query.m_faceIndexCount.GetCount(); ++i)
	{
		ndInt32 j = i;
		ndInt32 ptr = query.m_faceIndexStart[i];
		ndInt32 count = query.m_faceIndexCount[i];
		ndFloat32 dist = query.m_hitDistance[i];
		for (; dist < query.m_hitDistance[j - 1]; j--)
		{
			ndAssert(j > 0);
			query.m_hitDistance[j] = query.m_hitDistance[j - 1];
			query.m_faceIndexStart[j] = query.m_faceIndexStart[j - 1];
			query.m_faceIndexCount[j] = query.m_faceIndexCount[j - 1];
		}
		query.m_hitDistance[j] = dist;
		query.m_faceIndexStart[j] = ptr;
		query.m_faceIndexCount[j] = count;
	}

	#ifdef _DEBUG
	for (ndInt32 i = 0; i < query.m_faceIndexCount.GetCount() - 1; ++i)
	{
		ndAssert(query.m_hitDistance[i] <= query.m_hitDistance[i + 1]);
	}
	#endif
}

//ndPolygonMeshLocalDesc::ndPolygonMeshLocalDesc(ndContactSolver& proxy, bool ccdMode)
//	:ndPolygonMeshDesc(proxy, ccdMode)
//	,m_localStaticMeshQuery()
//	,m_localProceduralStaticMeshQuery()
//{
//	m_vertex = nullptr;
//	m_vertexStrideInBytes = 0;
//	m_skinMargin = proxy.m_skinMargin;
//	m_convexInstance = &proxy.m_instance0;
//	m_polySoupInstance = &proxy.m_instance1;
//
//	m_ownTempBuffers = false;
//	m_doContinueCollisionTest = ccdMode;
//
//	m_staticMeshQuery = &m_localStaticMeshQuery;
//	m_proceduralStaticMeshFaceQuery = &m_localProceduralStaticMeshQuery;
//	Init();
//}
