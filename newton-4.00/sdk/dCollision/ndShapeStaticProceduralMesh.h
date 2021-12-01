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

#ifndef __ND_SHAPE_STATIC_PROCEDURAL_MESH__
#define __ND_SHAPE_STATIC_PROCEDURAL_MESH__

#include "ndCollisionStdafx.h"
#include "ndShapeStaticMesh.h"

class ndShapeStaticProceduralMesh: public ndShapeStaticMesh
{
	public:
	class ndEdge
	{
		public:
		ndEdge();
		ndEdge(dUnsigned64 key);
		bool operator< (const ndEdge& edge) const;
		bool operator> (const ndEdge& edge) const;
		union
		{
			dUnsigned64 m_key;
			struct
			{
				dInt32 m_i0;
				dInt32 m_i1;
			};
		};
	};

	class ndEdgeMap : public dTree<dInt32, ndEdge, dContainersFreeListAlloc<dInt32>>
	{
		public:
		ndEdgeMap();
	};

	D_CLASS_REFLECTION(ndShapeStaticProceduralMesh);
	D_COLLISION_API ndShapeStaticProceduralMesh(const dLoadSaveBase::dLoadDescriptor& desc);
	D_COLLISION_API ndShapeStaticProceduralMesh(dFloat32 sizex, dFloat32 sizey, dFloat32 sizez);
	D_COLLISION_API virtual ~ndShapeStaticProceduralMesh();

	virtual ndShapeStaticProceduralMesh* GetAsShapeStaticProceduralMesh() { return this; }

	virtual void GetCollidingFaces(const dVector& minBox, const dVector& maxBox, dArray<dVector>& vertex, dArray<dInt32>& faceList, dArray<dInt32>& faceMaterial, dArray<dInt32>& indexListList) const;

	D_COLLISION_API virtual ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API void SetMaxVertexAndFaces(dInt32 maxVertex, dInt32 maxFaces);
	D_COLLISION_API virtual void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;

	private:
	D_COLLISION_API virtual void GetCollidingFaces(ndPolygonMeshDesc* const data) const;

	class ndLocalThreadData
	{
		public:
		ndLocalThreadData()
			:m_threadId()
		{
		}

		dArray<dVector> m_vertex;
		std::thread::id m_threadId;
	};

	void CalculateLocalObb();

	dVector m_minBox;
	dVector m_maxBox;
	mutable dList<ndLocalThreadData> m_localData;
	dInt32 m_maxFaceCount;
	dInt32 m_maxVertexCount;

	friend class ndContactSolver;
};

inline void ndShapeStaticProceduralMesh::GetCollidingFaces(const dVector&, const dVector&, dArray<dVector>&, dArray<dInt32>&, dArray<dInt32>&, dArray<dInt32>&) const
{
	dAssert(0);
}

inline ndShapeStaticProceduralMesh::ndEdge::ndEdge()
{
}

inline ndShapeStaticProceduralMesh::ndEdge::ndEdge(dUnsigned64 key)
	:m_key(key)
{
}

inline bool ndShapeStaticProceduralMesh::ndEdge::operator< (const ndEdge& edge) const
{
	return m_key < edge.m_key;
}

inline bool ndShapeStaticProceduralMesh::ndEdge::operator> (const ndEdge& edge) const
{
	return m_key > edge.m_key;
}

inline ndShapeStaticProceduralMesh::ndEdgeMap::ndEdgeMap()
	:dTree<dInt32, ndEdge, dContainersFreeListAlloc<dInt32>>()
{
}

#endif
