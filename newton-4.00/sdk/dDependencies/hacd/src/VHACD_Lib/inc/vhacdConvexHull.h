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

#ifndef __ND_VHACD_CONVEXHULL_3D__
#define __ND_VHACD_CONVEXHULL_3D__

#include "ndConvexHull3d.h"
#include "vhacdConvexHullUtils.h"

namespace nd
{
	namespace VHACD
	{
		#define VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE 16
		class ConvexHullVertex : public ndBigVector
		{
			public:
			ndInt32 m_mark;
		};

		class ConvexHullAABBTreeNode
		{
			public:
			ConvexHullAABBTreeNode()
				:m_left(nullptr)
				,m_right(nullptr)
				,m_parent(nullptr)
				,m_count(0)
			{
			}

			ConvexHullAABBTreeNode(ConvexHullAABBTreeNode* const parent)
				:m_left(nullptr)
				,m_right(nullptr)
				,m_parent(parent)
				,m_count(0)
			{
			}

			ndBigVector m_box[2];
			ConvexHullAABBTreeNode* m_left;
			ConvexHullAABBTreeNode* m_right;
			ConvexHullAABBTreeNode* m_parent;
			ndInt32 m_count;
		};

		class ConvexHull3dPointCluster : public ConvexHullAABBTreeNode
		{
			public:
			ConvexHull3dPointCluster()
				:ConvexHullAABBTreeNode()
			{
			}

			ConvexHull3dPointCluster(ConvexHullAABBTreeNode* const parent)
				:ConvexHullAABBTreeNode(parent)
			{
			}

			//ndInt32 m_count;
			ndInt32 m_indices[VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE];
		};

		class ConvexHull3dPointSet : public ndArray<ConvexHullVertex>
		{
			public:
			ConvexHull3dPointSet();
			ConvexHull3dPointSet(const double* const vertexCloud, ndInt32 strideInBytes, ndInt32 count);
			ConvexHullAABBTreeNode* BuildAccelerator();

			const ConvexHullAABBTreeNode* GetTree() const;

			private:
			ConvexHullAABBTreeNode* BuildRecurse(ConvexHullAABBTreeNode* const parent, ConvexHullVertex* const points, ndInt32 count, ndInt32 baseIndex, ndInt32& memoryPool);
			ndArray<ConvexHull3dPointCluster> m_treeBuffer;
		};

		class ndConvexHull3dFace
		{
			public:
			ndConvexHull3dFace();
			double Evalue(const ndBigVector* const pointArray, const ndBigVector& point) const;
			ndBigPlane GetPlaneEquation(const ndBigVector* const pointArray, bool& isValid) const;

			public:
			ndInt32 m_index[3];

			private:
			ndInt32 m_mark;
			ndList<ndConvexHull3dFace, ndContainersFreeListAlloc<ndConvexHull3dFace>>::ndNode* m_twin[3];

			friend class ConvexHull;
		};

		class ConvexHull : public ndList<ndConvexHull3dFace, ndContainersFreeListAlloc<ndConvexHull3dFace>>
		{
			class ndNormalMap;

			public:
			ConvexHull(ConvexHull3dPointSet& accelerator, double distTol, ndInt32 maxVertexCount = 0x7fffffff);
			ConvexHull(const double* const vertexCloud, ndInt32 strideInBytes, ndInt32 count, double distTol, ndInt32 maxVertexCount = 0x7fffffff);
			~ConvexHull();

			const ndArray<ndBigVector>& GetVertexPool() const;

			private:
			ConvexHullAABBTreeNode* InitVertexArray(ConvexHull3dPointSet& accelerator);
			void BuildHull(ConvexHull3dPointSet& accelerator, double distTol, ndInt32 maxVertexCount);

			ndNode* AddFace(ndInt32 i0, ndInt32 i1, ndInt32 i2);

			void CalculateConvexHull3d(ConvexHullAABBTreeNode* vertexTree, ndArray<ConvexHullVertex>& points, ndInt32 count, double distTol, ndInt32 maxVertexCount);

			ndInt32 SupportVertex(ConvexHullAABBTreeNode** const tree, const ndArray<ConvexHullVertex>& points, const ndBigVector& dir, const bool removeEntry = true) const;
			double TetrahedrumVolume(const ndBigVector& p0, const ndBigVector& p1, const ndBigVector& p2, const ndBigVector& p3) const;

			ndBigVector m_aabbP0;
			ndBigVector m_aabbP1;
			ndArray<ndBigVector> m_points;
			double m_diag;
		};
	}
}
#endif
