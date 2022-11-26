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

#ifndef __ND_POLOGON_MESH_DESC_H__
#define __ND_POLOGON_MESH_DESC_H__

#include "ndCollisionStdafx.h"
#include "ndShape.h"

class ndBodyKinematic;
class ndShapeInstance;
class ndContactSolver;
class ndShapeStaticMesh;

D_MSV_NEWTON_ALIGN_32 
class ndPolygonMeshDesc: public ndFastAabb
{
	public:
	class ndStaticMeshFaceQuery
	{
		public:
		ndStaticMeshFaceQuery()
		{
			m_hitDistance.Resize(256);
			m_faceIndexCount.Resize(256);
			m_faceIndexStart.Resize(256);
			m_faceVertexIndex.Resize(256);
		}

		void Reset()
		{
			m_hitDistance.SetCount(0);
			m_faceIndexCount.SetCount(0);
			m_faceIndexStart.SetCount(0);
			m_faceVertexIndex.SetCount(0);
		}

		ndArray<ndFloat32> m_hitDistance;
		ndArray<ndInt32> m_faceIndexCount;
		ndArray<ndInt32> m_faceIndexStart;
		ndArray<ndInt32> m_faceVertexIndex;
	};

	// colliding box in polygonSoup local space
	D_COLLISION_API ndPolygonMeshDesc();
	D_COLLISION_API ndPolygonMeshDesc(ndContactSolver& proxy, bool ccdMode);

	D_COLLISION_API void SortFaceArray();
	D_COLLISION_API ndFloat32 GetSeparetionDistance() const;
	D_COLLISION_API ndInt32 GetFaceIndexCount(ndInt32 indexCount) const;
	D_COLLISION_API void SetDistanceTravel(const ndVector& distanceInGlobalSpace);
	D_COLLISION_API ndInt32 GetFaceId(const ndInt32* const faceIndexArray, ndInt32 indexCount) const;
	D_COLLISION_API ndInt32 GetNormalIndex(const ndInt32* const faceIndexArray, ndInt32 indexCount) const;
	D_COLLISION_API ndFloat32 GetFaceSize(const ndInt32* const faceIndexArray, ndInt32 indexCount) const;
	D_COLLISION_API const ndInt32* GetAdjacentFaceEdgeNormalArray(const ndInt32* const faceIndexArray, ndInt32 indexCount) const;

	ndVector m_boxDistanceTravelInMeshSpace;
	ndInt32 m_vertexStrideInBytes;
	ndFloat32 m_skinMargin;
	ndShapeInstance* m_convexInstance;
	ndShapeInstance* m_polySoupInstance;
	ndFloat32* m_vertex;
	ndArray<ndVector>* m_tmpVertexArray;

	// private data;
	ndStaticMeshFaceQuery* m_staticMeshQuery;
	const ndShapeStaticMesh* m_shapeStaticMesh;
	ndFloat32 m_maxT;
	ndInt32 m_threadId;
	bool m_doContinueCollisionTest;
} D_GCC_NEWTON_ALIGN_32;

#endif 



