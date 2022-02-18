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


#include "vhacdConvexHullUtils.h"

class vhacdConvexHullVertex;
class vhacdConvexHullAABBTreeNode;

class vhacdConvexHullFace
{
	public:
	vhacdConvexHullFace();
	double Evalue (const hullVector* const pointArray, const hullVector& point) const;
	hullPlane GetPlaneEquation (const hullVector* const pointArray) const;
	
	public:
	int m_index[3]; 
	private:
	int m_mark;
	vhacdList<vhacdConvexHullFace>::ndNode* m_twin[3];
	
	friend class vhacdConvexHull;
};

class vhacdConvexHull: public vhacdList<vhacdConvexHullFace>
{
	class ndNormalMap;
	public:
	vhacdConvexHull(const vhacdConvexHull& source);
	vhacdConvexHull(const double* const vertexCloud, int strideInBytes, int count, double distTol, int maxVertexCount = 0x7fffffff);
	~vhacdConvexHull();
		
	const std::vector<hullVector>& GetVertexPool() const;

	private:
	void BuildHull (const double* const vertexCloud, int strideInBytes, int count, double distTol, int maxVertexCount);

	int GetUniquePoints(vhacdConvexHullVertex* const points, const double* const vertexCloud, int strideInBytes, int count, void* const memoryPool, int maxMemSize);
	int InitVertexArray(vhacdConvexHullVertex* const points, const double* const vertexCloud, int strideInBytes, int count, void* const memoryPool, int maxMemSize);
	vhacdConvexHullAABBTreeNode* BuildTree (vhacdConvexHullAABBTreeNode* const parent, vhacdConvexHullVertex* const points, int count, int baseIndex, char** const memoryPool, int& maxMemSize) const;

	ndNode* AddFace (int i0, int i1, int i2);
	
	void CalculateConvexHull3d (vhacdConvexHullAABBTreeNode* vertexTree, vhacdConvexHullVertex* const points, int count, double distTol, int maxVertexCount);
	
	int SupportVertex (vhacdConvexHullAABBTreeNode** const tree, const vhacdConvexHullVertex* const points, const hullVector& dir, const bool removeEntry = true) const;
	double TetrahedrumVolume (const hullVector& p0, const hullVector& p1, const hullVector& p2, const hullVector& p3) const;

	hullVector m_aabbP0;
	hullVector m_aabbP1;
	double m_diag;
	std::vector<hullVector> m_points;
};

#endif
