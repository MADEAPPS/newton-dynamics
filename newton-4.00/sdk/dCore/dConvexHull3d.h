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

#ifndef __DG_CONVEXHULL_3D__
#define __DG_CONVEXHULL_3D__

#include "dCoreStdafx.h"
#include "dList.h"
#include "dArray.h"
#include "dPlane.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"

#define D_OLD_CONVEXHULL_3D

class dConvexHull3dVertex;
class dConvexHull3dAABBTreeNode;

class dConvexHull3dFace
{
	public:
	dConvexHull3dFace();
	
	void SetMark(dInt32 mark) {m_mark = mark;}
	dInt32 GetMark() const {return m_mark;}
	dList<dConvexHull3dFace>::dNode* GetTwin(dInt32 index) const { return m_twin[index];}

	private:
	dFloat64 Evalue (const dBigVector* const pointArray, const dBigVector& point) const;
	dBigPlane GetPlaneEquation (const dBigVector* const pointArray) const;

	public:
	dInt32 m_index[3]; 
	private:
	dInt32 m_mark;
	dList<dConvexHull3dFace>::dNode* m_twin[3];
	friend class dConvexHull3d;
};

D_MSV_NEWTON_ALIGN_32
class dConvexHull3d: public dClassAlloc, public dList<dConvexHull3dFace>
{
#ifdef	D_OLD_CONVEXHULL_3D
	class dNormalMap;
#endif

	public:
	D_CORE_API dConvexHull3d(const dConvexHull3d& source);
	D_CORE_API dConvexHull3d(const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount = 0x7fffffff);
	D_CORE_API virtual ~dConvexHull3d();

	dInt32 GetVertexCount() const;
	const dBigVector* GetVertexPool() const;
	const dBigVector& GetVertex(dInt32 i) const;

	dFloat64 GetDiagonal() const;
	void GetAABB (dBigVector& boxP0, dBigVector& boxP1) const;
	dFloat64 RayCast (const dBigVector& localP0, const dBigVector& localP1) const;
	void CalculateVolumeAndSurfaceArea (dFloat64& volume, dFloat64& surcafeArea) const;

	protected:
	dConvexHull3d();
	void BuildHull (const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount);

	virtual dNode* AddFace (dInt32 i0, dInt32 i1, dInt32 i2);
	virtual void DeleteFace (dNode* const node) ;
	virtual dInt32 InitVertexArray(dConvexHull3dVertex* const points, const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, void* const memoryPool, dInt32 maxMemSize);

	bool CheckFlatSurface(dConvexHull3dAABBTreeNode* vertexTree, dConvexHull3dVertex* const points, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount);
	void CalculateConvexHull2d (dConvexHull3dAABBTreeNode* vertexTree, dConvexHull3dVertex* const points, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount);
	void CalculateConvexHull3d (dConvexHull3dAABBTreeNode* vertexTree, dConvexHull3dVertex* const points, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount);
	
	dInt32 SupportVertex (dConvexHull3dAABBTreeNode** const tree, const dConvexHull3dVertex* const points, const dBigVector& dir, const bool removeEntry = true) const;
	dFloat64 TetrahedrumVolume (const dBigVector& p0, const dBigVector& p1, const dBigVector& p2, const dBigVector& p3) const;

	dInt32 GetUniquePoints(dConvexHull3dVertex* const points, const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, void* const memoryPool, dInt32 maxMemSize);
	dConvexHull3dAABBTreeNode* BuildTree (dConvexHull3dAABBTreeNode* const parent, dConvexHull3dVertex* const points, dInt32 count, dInt32 baseIndex, dInt8** const memoryPool, dInt32& maxMemSize) const;
	static dInt32 ConvexCompareVertex(const dConvexHull3dVertex* const A, const dConvexHull3dVertex* const B, void* const context);
	bool Sanity() const;
	void Save (const char* const filename) const;

	dBigVector m_aabbP0;
	dBigVector m_aabbP1;
	dInt32 m_count;
	dFloat64 m_diag;
	dArray<dBigVector> m_points;
} D_GCC_NEWTON_ALIGN_32;

inline dInt32 dConvexHull3d::GetVertexCount() const
{
	return m_count;
}

inline const dBigVector* dConvexHull3d::GetVertexPool() const
{
	return &m_points[0];
}

inline const dBigVector& dConvexHull3d::GetVertex(dInt32 index) const
{
	return m_points[index];
}

inline dFloat64 dConvexHull3d::GetDiagonal() const
{
	return m_diag;
}


inline void dConvexHull3d::GetAABB (dBigVector& boxP0, dBigVector& boxP1) const
{
	boxP0 = m_aabbP0;
	boxP1 = m_aabbP1;
}

#endif
