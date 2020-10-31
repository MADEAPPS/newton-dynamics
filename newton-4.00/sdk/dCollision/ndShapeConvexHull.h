/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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


#ifndef __D_SHAPE_CONVEXHULL_H__
#define __D_SHAPE_CONVEXHULL_H__


#include "ndShapeConvex.h"

D_MSV_NEWTON_ALIGN_32
class ndShapeConvexHull : public ndShapeConvex
{
	class ndConvexBox;

	public:
	D_COLLISION_API ndShapeConvexHull(dInt32 count, dInt32 strideInBytes, dFloat32 tolerance, const dFloat32* const vertexArray);
	D_COLLISION_API virtual ~ndShapeConvexHull();

	protected:
	ndShapeInfo GetShapeInfo() const;
	dBigVector FaceNormal(const dEdge *face, const dBigVector* const pool) const;
	bool RemoveCoplanarEdge(dPolyhedra& convex, const dBigVector* const hullVertexArray) const;
	bool Create(dInt32 count, dInt32 strideInBytes, const dFloat32* const vertexArray, dFloat32 tolerance);
	virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const;

	D_COLLISION_API virtual void Save(nd::TiXmlElement* const xmlNode, dInt32 nodeid) const;

	private:
	dVector SupportVertexBruteForce(const dVector& dir, dInt32* const vertexIndex) const;
	dVector SupportVertexhierarchical(const dVector& dir, dInt32* const vertexIndex) const;
	
	void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const;

	//protected:
	//dInt32 GetFaceIndices (dInt32 index, dInt32* const indices) const;
	//virtual const ndConvexSimplexEdge** GetVertexToEdgeMapping() const {return m_vertexToEdgeMapping;}
	ndConvexBox* m_supportTree;
	ndConvexSimplexEdge** m_faceArray;
	dVector* m_soa_x;
	dVector* m_soa_y;
	dVector* m_soa_z;
	dVector* m_soa_index;

	const ndConvexSimplexEdge** m_vertexToEdgeMapping;
	dInt32 m_faceCount;
	dInt32 m_soaVertexCount;
	dInt32 m_supportTreeCount;
} D_GCC_NEWTON_ALIGN_32;

#endif 

