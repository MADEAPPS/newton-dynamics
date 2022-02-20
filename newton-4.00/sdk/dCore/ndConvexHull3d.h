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
	D_CORE_API ndConvexHull3dFace();

	private:
	void SetMark(ndInt32 mark) { m_mark = mark; }
	ndInt32 GetMark() const { return m_mark; }
	ndList<ndConvexHull3dFace>::ndNode* GetTwin(ndInt32 index) const;
	ndFloat64 Evalue (const ndBigVector* const pointArray, const ndBigVector& point) const;
	ndBigPlane GetPlaneEquation (const ndBigVector* const pointArray, bool& isvalid) const;

	public:
	ndInt32 m_index[3]; 

	private:
	ndInt32 m_mark;
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
	D_CORE_API ndConvexHull3d(const ndFloat64* const vertexCloud, ndInt32 strideInBytes, ndInt32 count, ndFloat64 distTol, ndInt32 maxVertexCount = 0x7fffffff);
	D_CORE_API virtual ~ndConvexHull3d();

	const ndArray<ndBigVector>& GetVertexPool() const;

	ndFloat64 GetDiagonal() const;
	void GetAABB (ndBigVector& boxP0, ndBigVector& boxP1) const;
	ndFloat64 RayCast (const ndBigVector& localP0, const ndBigVector& localP1) const;
	void CalculateVolumeAndSurfaceArea (ndFloat64& volume, ndFloat64& surcafeArea) const;

	protected:
	ndConvexHull3d();
	void BuildHull (const ndFloat64* const vertexCloud, ndInt32 strideInBytes, ndInt32 count, ndFloat64 distTol, ndInt32 maxVertexCount);

	virtual ndNode* AddFace (ndInt32 i0, ndInt32 i1, ndInt32 i2);
	virtual void DeleteFace (ndNode* const node) ;
	virtual ndInt32 InitVertexArray(ndConvexHull3dVertex* const points, ndInt32 count, void* const memoryPool, ndInt32 maxMemSize);

	bool CheckFlatSurface(ndConvexHull3dAABBTreeNode* vertexTree, ndConvexHull3dVertex* const points, ndInt32 count, ndFloat64 distTol, ndInt32 maxVertexCount);
	void CalculateConvexHull2d (ndConvexHull3dAABBTreeNode* vertexTree, ndConvexHull3dVertex* const points, ndInt32 count, ndFloat64 distTol, ndInt32 maxVertexCount);
	void CalculateConvexHull3d (ndConvexHull3dAABBTreeNode* vertexTree, ndConvexHull3dVertex* const points, ndInt32 count, ndFloat64 distTol, ndInt32 maxVertexCount);

	ndInt32 SupportVertex (ndConvexHull3dAABBTreeNode** const tree, const ndConvexHull3dVertex* const points, const ndBigVector& dir, const bool removeEntry = true) const;
	ndFloat64 TetrahedrumVolume (const ndBigVector& p0, const ndBigVector& p1, const ndBigVector& p2, const ndBigVector& p3) const;

	ndInt32 GetUniquePoints(ndConvexHull3dVertex* const points, ndInt32 count);
	ndConvexHull3dAABBTreeNode* BuildTree (ndConvexHull3dAABBTreeNode* const parent, ndConvexHull3dVertex* const points, ndInt32 count, ndInt32 baseIndex, ndInt8** const memoryPool, ndInt32& maxMemSize) const;

	bool Sanity() const;
	void Save (const char* const filename) const;

	ndBigVector m_aabbP0;
	ndBigVector m_aabbP1;
	ndFloat64 m_diag;
	ndArray<ndBigVector> m_points;
} D_GCC_NEWTON_ALIGN_32;

inline const ndArray<ndBigVector>& ndConvexHull3d::GetVertexPool() const
{
	return m_points;
}

inline ndFloat64 ndConvexHull3d::GetDiagonal() const
{
	return m_diag;
}

inline void ndConvexHull3d::GetAABB (ndBigVector& boxP0, ndBigVector& boxP1) const
{
	boxP0 = m_aabbP0;
	boxP1 = m_aabbP1;
}

#endif
