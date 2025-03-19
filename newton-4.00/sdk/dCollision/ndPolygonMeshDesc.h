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

D_MSV_NEWTON_CLASS_ALIGN_32 
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

	class ndProceduralStaticMeshFaceQuery
	{
		public:
		ndProceduralStaticMeshFaceQuery()
		{
			m_vertex.Resize(256);
			m_faceMaterial.Resize(256);
			m_indexListList.Resize(256);
		}

		void Reset()
		{
			m_vertex.SetCount(0);
			m_faceMaterial.SetCount(0);
			m_indexListList.SetCount(0);
		}

		ndArray<ndVector> m_vertex;
		ndArray<ndInt32> m_faceMaterial;
		ndArray<ndInt32> m_indexListList;
	};

	// colliding box in polygonSoup local space
	//D_COLLISION_API ndPolygonMeshDesc();
	D_COLLISION_API ndPolygonMeshDesc(ndContactSolver& proxy, bool ccdMode);
	D_COLLISION_API ~ndPolygonMeshDesc();

	D_COLLISION_API void SortFaceArray();
	D_COLLISION_API ndFloat32 GetSeparetionDistance() const;
	D_COLLISION_API ndInt32 GetFaceIndexCount(ndInt32 indexCount) const;
	D_COLLISION_API void SetDistanceTravel(const ndVector& distanceInGlobalSpace);
	D_COLLISION_API ndInt32 GetFaceId(const ndInt32* const faceIndexArray, ndInt32 indexCount) const;
	D_COLLISION_API ndInt32 GetNormalIndex(const ndInt32* const faceIndexArray, ndInt32 indexCount) const;
	D_COLLISION_API ndFloat32 GetFaceSize(const ndInt32* const faceIndexArray, ndInt32 indexCount) const;
	D_COLLISION_API const ndInt32* GetAdjacentFaceEdgeNormalArray(const ndInt32* const faceIndexArray, ndInt32 indexCount) const;

	void Init();

	ndVector m_boxDistanceTravelInMeshSpace;
	ndInt32 m_vertexStrideInBytes;
	ndFloat32 m_skinMargin;
	ndShapeInstance* m_convexInstance;
	ndShapeInstance* m_polySoupInstance;
	ndFloat32* m_vertex;

	// private data;
	ndStaticMeshFaceQuery* m_staticMeshQuery;
	ndProceduralStaticMeshFaceQuery* m_proceduralStaticMeshFaceQuery;
	ndFloat32 m_maxT;
	ndInt32 m_threadId;
	bool m_doContinueCollisionTest;

} D_GCC_NEWTON_CLASS_ALIGN_32;


//class ndPolygonMeshLocalDesc : public ndPolygonMeshDesc
//{
//	public:
//	D_COLLISION_API ndPolygonMeshLocalDesc(ndContactSolver& proxy, bool ccdMode);
//
//	ndPolygonMeshDesc::ndStaticMeshFaceQuery m_localStaticMeshQuery;
//	ndPolygonMeshDesc::ndProceduralStaticMeshFaceQuery m_localProceduralStaticMeshQuery;
//};

#endif 



