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

#ifndef __ND_CONVEXHULL_3D__
#define __ND_CONVEXHULL_3D__

#include "ndCoreStdafx.h"
#include "ndList.h"
#include "ndArray.h"
#include "ndPlane.h"
#include "ndVector.h"
#include "ndMatrix.h"
#include "ndQuaternion.h"

#define D_OLD_CONVEXHULL_3D

class ndConvexHull3dVertex;
class ndConvexHull3dAABBTreeNode;

class ndConvexHull3dFace
{
	public:
	ndConvexHull3dFace();
	
	void SetMark(dInt32 mark) {m_mark = mark;}
	dInt32 GetMark() const {return m_mark;}
	ndList<ndConvexHull3dFace>::ndNode* GetTwin(dInt32 index) const { return m_twin[index];}

	private:
	dFloat64 Evalue (const ndBigVector* const pointArray, const ndBigVector& point) const;
	ndBigPlane GetPlaneEquation (const ndBigVector* const pointArray) const;

	public:
	dInt32 m_index[3]; 
	private:
	dInt32 m_mark;
	ndList<ndConvexHull3dFace>::ndNode* m_twin[3];
	friend class ndConvexHull3d;
};

D_MSV_NEWTON_ALIGN_32
class ndConvexHull3d: public ndList<ndConvexHull3dFace>
{
#ifdef	D_OLD_CONVEXHULL_3D
	class ndNormalMap;
#endif

	public:
	D_CORE_API ndConvexHull3d(const ndConvexHull3d& source);
	D_CORE_API ndConvexHull3d(const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount = 0x7fffffff);
	D_CORE_API virtual ~ndConvexHull3d();

	dInt32 GetVertexCount() const;
	const ndBigVector* GetVertexPool() const;
	const ndBigVector& GetVertex(dInt32 i) const;

	dFloat64 GetDiagonal() const;
	void GetAABB (ndBigVector& boxP0, ndBigVector& boxP1) const;
	dFloat64 RayCast (const ndBigVector& localP0, const ndBigVector& localP1) const;
	void CalculateVolumeAndSurfaceArea (dFloat64& volume, dFloat64& surcafeArea) const;

	protected:
	ndConvexHull3d();
	void BuildHull (const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount);

	virtual ndNode* AddFace (dInt32 i0, dInt32 i1, dInt32 i2);
	virtual void DeleteFace (ndNode* const node) ;
	virtual dInt32 InitVertexArray(ndConvexHull3dVertex* const points, const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, void* const memoryPool, dInt32 maxMemSize);

	bool CheckFlatSurface(ndConvexHull3dAABBTreeNode* vertexTree, ndConvexHull3dVertex* const points, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount);
	void CalculateConvexHull2d (ndConvexHull3dAABBTreeNode* vertexTree, ndConvexHull3dVertex* const points, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount);
	void CalculateConvexHull3d (ndConvexHull3dAABBTreeNode* vertexTree, ndConvexHull3dVertex* const points, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount);
	
	dInt32 SupportVertex (ndConvexHull3dAABBTreeNode** const tree, const ndConvexHull3dVertex* const points, const ndBigVector& dir, const bool removeEntry = true) const;
	dFloat64 TetrahedrumVolume (const ndBigVector& p0, const ndBigVector& p1, const ndBigVector& p2, const ndBigVector& p3) const;

	dInt32 GetUniquePoints(ndConvexHull3dVertex* const points, const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, void* const memoryPool, dInt32 maxMemSize);
	ndConvexHull3dAABBTreeNode* BuildTree (ndConvexHull3dAABBTreeNode* const parent, ndConvexHull3dVertex* const points, dInt32 count, dInt32 baseIndex, dInt8** const memoryPool, dInt32& maxMemSize) const;
	//static dInt32 ConvexCompareVertex(const ndConvexHull3dVertex* const A, const ndConvexHull3dVertex* const B, void* const context);
	bool Sanity() const;
	void Save (const char* const filename) const;

	ndBigVector m_aabbP0;
	ndBigVector m_aabbP1;
	dInt32 m_count;
	dFloat64 m_diag;
	ndArray<ndBigVector> m_points;
} D_GCC_NEWTON_ALIGN_32;

inline dInt32 ndConvexHull3d::GetVertexCount() const
{
	return m_count;
}

inline const ndBigVector* ndConvexHull3d::GetVertexPool() const
{
	return &m_points[0];
}

inline const ndBigVector& ndConvexHull3d::GetVertex(dInt32 index) const
{
	return m_points[index];
}

inline dFloat64 ndConvexHull3d::GetDiagonal() const
{
	return m_diag;
}


inline void ndConvexHull3d::GetAABB (ndBigVector& boxP0, ndBigVector& boxP1) const
{
	boxP0 = m_aabbP0;
	boxP1 = m_aabbP1;
}

#endif
