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

#ifndef __NDG_CONVEXHULL_4D__
#define __NDG_CONVEXHULL_4D__

#include "ndCoreStdafx.h"
#include "ndList.h"
#include "ndArray.h"
#include "ndPlane.h"
#include "ndVector.h"
#include "ndMatrix.h"
#include "ndQuaternion.h"

class dConvexHull4dAABBTreeNode;

class dConvexHull4dVector: public dBigVector
{
	public:
	void operator = (const dBigVector& a) 
	{
		m_x = a.m_x;
		m_y = a.m_y;
		m_z = a.m_z;
		m_w = a.m_w;
		m_index = 0;
		m_mark = 0;
	}

	dInt32 m_index;
	dInt32 m_mark;
};

class dConvexHull4dTetraherum
{
	public:
	class dgTetrahedrumFace 
	{
		public:
		dInt32 m_index[4];
		dList<dConvexHull4dTetraherum>::dNode* m_twin;
	};


	class dgTetrahedrumPlane: public dBigVector
	{
		public: 
		dgTetrahedrumPlane (const dBigVector& p0, const dBigVector& p1, const dBigVector& p2, const dBigVector& p3);
		dFloat64 Evalue (const dBigVector& point) const;
		dFloat64 m_dist;
	};

	dConvexHull4dTetraherum();
	dgTetrahedrumPlane GetPlaneEquation (const dConvexHull4dVector* const points) const;
	dFloat64 GetTetraVolume (const dConvexHull4dVector* const pointArray) const;	
	D_CORE_API dBigVector CircumSphereCenter (const dConvexHull4dVector* const pointArray) const;
	dFloat64 Evalue (const dConvexHull4dVector* const pointArray, const dBigVector& point) const;

	dInt32 GetMark() const { return m_mark; }
	void SetMark(dInt32 mark) { m_mark = mark; }

	private:
	void Init (const dConvexHull4dVector* const points, dInt32 v0, dInt32 v1, dInt32 v2, dInt32 v3);

	public:
	dgTetrahedrumFace m_faces[4];
	dInt32 m_mark;
	dInt32 m_uniqueID;

#ifdef _DEBUG
	dInt32 m_debugID;
#endif
	friend class dConvexHull4d;
	friend class dDelaunayTetrahedralization;
};

class dConvexHull4d: public dList<dConvexHull4dTetraherum>
{
	public:
	class ndTempList: public dList<dNode*, dContainersFreeListAlloc<dNode*>>
	{
	};

	D_CORE_API dConvexHull4d(const dConvexHull4d& source);
	D_CORE_API dConvexHull4d(const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, dFloat64 distTol);
	D_CORE_API virtual ~dConvexHull4d();

	dInt32 GetVertexCount() const;
	dInt32 GetVertexIndex(dInt32 i) const;
	const dBigVector& GetVertex(dInt32 i) const;

	const dConvexHull4dVector* GetHullVertexArray() const;
	dFloat64 GetTetraVolume (const dConvexHull4dTetraherum* const tetra) const;

	dInt32 IncMark (); 
	void Save (const char* const filename) const;

	protected:
	dConvexHull4d();

	void BuildHull (const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, dFloat64 distTol);

	virtual dInt32 AddVertex (const dBigVector& vertex);
	virtual dInt32 InitVertexArray(dConvexHull4dVector* const points, const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, void* const memoryPool, dInt32 maxMemSize);
	
	virtual dNode* AddFace (dInt32 i0, dInt32 i1, dInt32 i2, dInt32 i3);
	virtual void DeleteFace (dNode* const node);

	dNode* FindFacingNode(const dBigVector& vertex);
	
	void InsertNewVertex(dInt32 vertexIndex, dNode* const frontFace, ndTempList& deletedFaces, ndTempList& newFaces);
	dInt32 SupportVertex (dConvexHull4dAABBTreeNode** const tree, const dConvexHull4dVector* const points, const dBigVector& dir, const bool removeEntry = true) const;
	
	void CalculateConvexHull (dConvexHull4dAABBTreeNode* vertexTree, dConvexHull4dVector* const points, dInt32 count, dFloat64 distTol);
	void LinkSibling (dNode* node0, dNode* node1)	const;
	bool Sanity() const;
	dConvexHull4dAABBTreeNode* BuildTree (dConvexHull4dAABBTreeNode* const parent, dConvexHull4dVector* const points, dInt32 count, dInt32 baseIndex, dInt8** const memoryPool, dInt32& maxMemSize) const;
	
	class dgNormalMap
	{
		public:
		dgNormalMap();
		private:
		void TessellateTriangle(dInt32 level, const dVector& p0, const dVector& p1, const dVector& p2, dBigVector* const buffer, dInt32& count);

		dBigVector m_normal[1024];
		dInt32 m_count;
		friend class dConvexHull4d; 
	};
	static const dgNormalMap& GetNormaMap();

	dInt32 m_mark;
	dInt32 m_count;
	dFloat64 m_diag;
	dArray<dConvexHull4dVector> m_points;
};

inline dInt32 dConvexHull4d::IncMark ()
{
	m_mark ++;
	return m_mark;
}

inline dInt32 dConvexHull4d::GetVertexCount() const
{
	return m_count;
}

inline dInt32 dConvexHull4d::GetVertexIndex(dInt32 index) const
{
	dAssert (index >= 0);
	dAssert (index < m_count);
	return m_points[index].m_index;
}


inline const dBigVector& dConvexHull4d::GetVertex(dInt32 index) const
{
	dAssert (index >= 0);
	dAssert (index < m_count);
	return m_points[index];
}

inline const dConvexHull4dVector* dConvexHull4d::GetHullVertexArray() const
{
	return &m_points[0];
}

inline dFloat64 dConvexHull4d::GetTetraVolume (const dConvexHull4dTetraherum* const tetra) const
{
	return tetra->GetTetraVolume (&m_points[0]);
}

#endif
