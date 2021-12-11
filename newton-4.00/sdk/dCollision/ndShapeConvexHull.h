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

#ifndef __ND_SHAPE_CONVEXHULL_H__
#define __ND_SHAPE_CONVEXHULL_H__

#include "ndShapeConvex.h"

D_MSV_NEWTON_ALIGN_32
class ndShapeConvexHull : public ndShapeConvex
{
	class ndConvexBox;

	public:
	D_CLASS_REFLECTION(ndShapeConvexHull);
	D_COLLISION_API ndShapeConvexHull(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_COLLISION_API ndShapeConvexHull(dInt32 count, dInt32 strideInBytes, dFloat32 tolerance, const dFloat32* const vertexArray);
	D_COLLISION_API virtual ~ndShapeConvexHull();

	protected:
	ndShapeInfo GetShapeInfo() const;
	ndBigVector FaceNormal(const ndEdge *face, const ndBigVector* const pool) const;
	bool RemoveCoplanarEdge(ndPolyhedra& convex, const ndBigVector* const hullVertexArray) const;
	bool Create(dInt32 count, dInt32 strideInBytes, const dFloat32* const vertexArray, dFloat32 tolerance);
	virtual ndVector SupportVertex(const ndVector& dir, dInt32* const vertexIndex) const;
	D_COLLISION_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	private:
	ndVector SupportVertexBruteForce(const ndVector& dir, dInt32* const vertexIndex) const;
	ndVector SupportVertexhierarchical(const ndVector& dir, dInt32* const vertexIndex) const;
	
	void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;

	//protected:
	//dInt32 GetFaceIndices (dInt32 index, dInt32* const indices) const;
	//virtual const ndConvexSimplexEdge** GetVertexToEdgeMapping() const {return m_vertexToEdgeMapping;}
	ndConvexBox* m_supportTree;
	ndConvexSimplexEdge** m_faceArray;
	ndVector* m_soa_x;
	ndVector* m_soa_y;
	ndVector* m_soa_z;
	ndVector* m_soa_index;

	const ndConvexSimplexEdge** m_vertexToEdgeMapping;
	dInt32 m_faceCount;
	dInt32 m_soaVertexCount;
	dInt32 m_supportTreeCount;
} D_GCC_NEWTON_ALIGN_32;

#endif 

