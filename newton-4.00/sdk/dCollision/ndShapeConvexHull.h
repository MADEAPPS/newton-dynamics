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

#ifndef __ND_SHAPE_CONVEXHULL_H__
#define __ND_SHAPE_CONVEXHULL_H__

#include "ndShapeConvex.h"

D_MSV_NEWTON_ALIGN_32
class ndShapeConvexHull : public ndShapeConvex
{
	class ndConvexBox;

	public:
	D_CLASS_REFLECTION(ndShapeConvexHull,ndShapeConvex)
	D_COLLISION_API ndShapeConvexHull(ndInt32 count, ndInt32 strideInBytes, ndFloat32 tolerance, const ndFloat32* const vertexArray, ndInt32 maxPointsOut = 0x7fffffff);
	D_COLLISION_API virtual ~ndShapeConvexHull();

	virtual ndShapeConvexHull* GetAsShapeConvexHull() { return this; }

	protected:
	D_COLLISION_API ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API ndUnsigned64 GetHash(ndUnsigned64 hash) const;
	ndBigVector FaceNormal(const ndEdge *face, const ndBigVector* const pool) const;
	bool RemoveCoplanarEdge(ndPolyhedra& convex, const ndBigVector* const hullVertexArray) const;
	bool Create(ndInt32 count, ndInt32 strideInBytes, const ndFloat32* const vertexArray, ndFloat32 tolerance, ndInt32 maxPointsOut);
	virtual ndVector SupportVertex(const ndVector& dir) const;
	virtual ndVector SupportFeatureVertex(const ndVector& dir, ndInt32* const vertexIndex) const;
	
	private:
	ndVector SupportVertexBruteForce(const ndVector& dir, ndInt32* const vertexIndex) const;
	ndVector SupportVertexhierarchical(const ndVector& dir, ndInt32* const vertexIndex) const;
	
	void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;

	ndConvexBox* m_supportTree;
	ndConvexSimplexEdge** m_faceArray;
	ndVector* m_soa_x;
	ndVector* m_soa_y;
	ndVector* m_soa_z;
	ndVector* m_soa_index;
	const ndConvexSimplexEdge** m_vertexToEdgeMapping;
	ndInt32 m_faceCount;
	ndInt32 m_soaVertexCount;
	ndInt32 m_supportTreeCount;

	D_MEMORY_ALIGN_FIXUP
} D_GCC_NEWTON_ALIGN_32;

#endif 

