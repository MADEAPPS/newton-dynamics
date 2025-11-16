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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "vhacdConvexHull.h"

namespace nd
{
	namespace VHACD
	{
		#define DG_STACK_DEPTH_3D 64
		ndConvexHull3dFace::ndConvexHull3dFace()
		{
			m_mark = 0;
			m_twin[0] = nullptr;
			m_twin[1] = nullptr;
			m_twin[2] = nullptr;
		}

		ndBigPlane ndConvexHull3dFace::GetPlaneEquation(const ndBigVector* const pointArray, bool& isvalid) const
		{
			const ndBigVector& p0 = pointArray[m_index[0]];
			const ndBigVector& p1 = pointArray[m_index[1]];
			const ndBigVector& p2 = pointArray[m_index[2]];
			ndBigPlane plane(p0, p1, p2);

			isvalid = false;
			double mag2 = plane.DotProduct(plane).GetScalar();
			if (mag2 > 1.0e-16f)
			{
				isvalid = true;
				plane = plane.Scale(1.0f / sqrt(mag2));
			}
			return plane;
		}

		double ndConvexHull3dFace::Evalue(const ndBigVector* const pointArray, const ndBigVector& point) const
		{
			const ndBigVector& p0 = pointArray[m_index[0]];
			const ndBigVector& p1 = pointArray[m_index[1]];
			const ndBigVector& p2 = pointArray[m_index[2]];

			double matrix[3][3];
			for (ndInt32 i = 0; i < 3; ++i)
			{
				matrix[0][i] = p2[i] - p0[i];
				matrix[1][i] = p1[i] - p0[i];
				matrix[2][i] = point[i] - p0[i];
			}

			double error;
			double det = Determinant3x3(matrix, &error);

			// the code use double, however the threshold for accuracy test is the machine precision of a float.
			// by changing this to a smaller number, the code should run faster since many small test will be considered valid
			// the precision must be a power of two no smaller than the machine precision of a double, (1<<48)
			// float64(1<<30) can be a good value

			// double precision	= double (1.0f) / double (1<<30);
			double precision = double(1.0f) / double(1 << 24);
			double errbound = error * precision;
			if (fabs(det) > errbound) 
			{
				return det;
			}
	
			ndGoogol exactMatrix[3][3];
			for (ndInt32 i = 0; i < 3; ++i)
			{
				exactMatrix[0][i] = ndGoogol(p2[i]) - ndGoogol(p0[i]);
				exactMatrix[1][i] = ndGoogol(p1[i]) - ndGoogol(p0[i]);
				exactMatrix[2][i] = ndGoogol(point[i]) - ndGoogol(p0[i]);
			}
			return Determinant3x3(exactMatrix);
		}

		ConvexHull3dPointSet::ConvexHull3dPointSet()
		{
		}

		ConvexHull3dPointSet::ConvexHull3dPointSet(const double* const vertexCloud, ndInt32 strideInBytes, ndInt32 count)
		{
			SetCount(count);

			ndArray<ConvexHullVertex>& array = *this;
			const ndInt32 stride = ndInt32(strideInBytes / sizeof(double));
			for (ndInt32 i = 0; i < count; ++i)
			{
				ndInt32 index = i * stride;
				ndBigVector& vertex = array[i];
				vertex = ndBigVector(vertexCloud[index], vertexCloud[index + 1], vertexCloud[index + 2], double(0.0f));
				array[i].m_mark = 0;
			}
			
			class CompareVertex
			{
				public:
				CompareVertex() {}
				CompareVertex(void*) {}

				ndInt32 Compare(const ConvexHullVertex& elementA, const ConvexHullVertex& elementB) const
				{
					for (ndInt32 i = 0; i < 3; i++)
					{
						if (elementA[i] < elementB[i])
						{
							return -1;
						}
						else if (elementA[i] > elementB[i])
						{
							return 1;
						}
					}
					return 0;
				}
			};
			
			count = ndInt32(GetCount());
			ndSort<ConvexHullVertex, CompareVertex>(&array[0], count, nullptr);
			
			ndInt32 indexCount = 0;
			CompareVertex compareVetex;
			for (ndInt32 i = 1; i < count; ++i)
			{
				for (; i < count; ++i)
				{
					if (compareVetex.Compare(array[indexCount], array[i]))
					{
						indexCount++;
						array[indexCount] = array[i];
						break;
					}
				}
			}
			count = indexCount + 1;
			array.SetCount(count);
		}

		const ConvexHullAABBTreeNode* ConvexHull3dPointSet::GetTree() const
		{
			return &m_treeBuffer[0];
		}

		ConvexHullAABBTreeNode* ConvexHull3dPointSet::BuildAccelerator()
		{
			ndInt32 treeCount = ndInt32(GetCount() / (VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE >> 1));
			if (treeCount < 4)
			{
				treeCount = 4;
			}
			treeCount *= 2;
			m_treeBuffer.SetCount(treeCount + 256);

			if (GetCount() > 4)
			{
				ndInt32 memoryIndex = 0;
				ndArray<ConvexHullVertex>& array = *this;
				return BuildRecurse(nullptr, &array[0], ndInt32 (GetCount()), 0, memoryIndex);
			}
			return nullptr;
		}

		ConvexHullAABBTreeNode* ConvexHull3dPointSet::BuildRecurse(ConvexHullAABBTreeNode* const parent, ConvexHullVertex* const points, ndInt32 count, ndInt32 baseIndex, ndInt32& memoryIndex)
		{
			ConvexHullAABBTreeNode* tree = nullptr;
		
			ndAssert(count);
			ndBigVector minP(double(1.0e15f));
			ndBigVector maxP(-double(1.0e15f));
			if (count <= VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE)
			{
				ConvexHull3dPointCluster* const clump = new (&m_treeBuffer[memoryIndex]) ConvexHull3dPointCluster();
				memoryIndex++;
				ndAssert(memoryIndex <= ndInt32 (m_treeBuffer.GetCount()));
		
				ndAssert(clump);
				clump->m_count = count;
				for (ndInt32 i = 0; i < count; ++i)
				{
					clump->m_indices[i] = i + baseIndex;
		
					const ndBigVector& p = points[i];
					minP = minP.GetMin(p);
					maxP = maxP.GetMax(p);
				}
		
				clump->m_left = nullptr;
				clump->m_right = nullptr;
				tree = clump;
			}
			else
			{
				ndBigVector median(ndBigVector::m_zero);
				ndBigVector varian(ndBigVector::m_zero);
				for (ndInt32 i = 0; i < count; ++i)
				{
					const ndBigVector& p = points[i];
					median += p;
					varian += p * p;
					minP = minP.GetMin(p);
					maxP = maxP.GetMax(p);
				}

				ndInt32 index = 0;
				double maxVarian = double(-1.0e10f);
				varian = varian.Scale(double(count)) - median * median;
				for (ndInt32 i = 0; i < 3; ++i)
				{
					if (varian[i] > maxVarian)
					{
						index = i;
						maxVarian = varian[i];
					}
				}
				ndBigVector center(median.Scale(double(1.0f) / double(count)));
		
				double test = center[index];
		
				ndInt32 i0 = 0;
				ndInt32 i1 = count - 1;
				do
				{
					for (; i0 <= i1; i0++)
					{
						double val = points[i0][index];
						if (val > test)
						{
							break;
						}
					}
		
					for (; i1 >= i0; i1--)
					{
						double val = points[i1][index];
						if (val < test)
						{
							break;
						}
					}
		
					if (i0 < i1)
					{
						ndSwap(points[i0], points[i1]);
						i0++;
						i1--;
					}
				} while (i0 <= i1);
		
				if (i0 == 0)
				{
					i0 = count / 2;
				}
				if (i0 >= (count - 1))
				{
					i0 = count / 2;
				}
		
				tree = new (&m_treeBuffer[memoryIndex]) ConvexHullAABBTreeNode();
				memoryIndex++;
				ndAssert(memoryIndex <= m_treeBuffer.GetCount());
		
				ndAssert(i0);
				ndAssert(count - i0);
		
				tree->m_count = count;
				tree->m_left = BuildRecurse(tree, points, i0, baseIndex, memoryIndex);
				tree->m_right = BuildRecurse(tree, &points[i0], count - i0, i0 + baseIndex, memoryIndex);
			}
		
			ndAssert(tree);
			tree->m_parent = parent;
			tree->m_box[0] = minP - ndBigVector(double(1.0e-3f));
			tree->m_box[1] = maxP + ndBigVector(double(1.0e-3f));
			return tree;
		}

		class ConvexHull::ndNormalMap
		{
			public:
			ndNormalMap()
				:m_count(sizeof(m_normal) / sizeof(m_normal[0]))
			{
				ndBigVector p0(double(1.0f), double(0.0f), double(0.0f), double(0.0f));
				ndBigVector p1(double(-1.0f), double(0.0f), double(0.0f), double(0.0f));
				ndBigVector p2(double(0.0f), double(1.0f), double(0.0f), double(0.0f));
				ndBigVector p3(double(0.0f), double(-1.0f), double(0.0f), double(0.0f));
				ndBigVector p4(double(0.0f), double(0.0f), double(1.0f), double(0.0f));
				ndBigVector p5(double(0.0f), double(0.0f), double(-1.0f), double(0.0f));

				ndInt32 count = 0;
				ndInt32 subdivitions = 2;
				TessellateTriangle(subdivitions, p4, p0, p2, count);
				TessellateTriangle(subdivitions, p0, p5, p2, count);
				TessellateTriangle(subdivitions, p5, p1, p2, count);
				TessellateTriangle(subdivitions, p1, p4, p2, count);
				TessellateTriangle(subdivitions, p0, p4, p3, count);
				TessellateTriangle(subdivitions, p5, p0, p3, count);
				TessellateTriangle(subdivitions, p1, p5, p3, count);
				TessellateTriangle(subdivitions, p4, p1, p3, count);
			}

			static const ndNormalMap& GetNormaMap()
			{
				static ndNormalMap normalMap;
				return normalMap;
			}

			void TessellateTriangle(ndInt32 level, const ndBigVector& p0, const ndBigVector& p1, const ndBigVector& p2, ndInt32& count)
			{
				if (level) 
				{
					ndAssert(fabs(p0.DotProduct(p0).GetScalar() - double(1.0f)) < double(1.0e-4f));
					ndAssert(fabs(p1.DotProduct(p1).GetScalar() - double(1.0f)) < double(1.0e-4f));
					ndAssert(fabs(p2.DotProduct(p2).GetScalar() - double(1.0f)) < double(1.0e-4f));
					ndBigVector p01(p0 + p1);
					ndBigVector p12(p1 + p2);
					ndBigVector p20(p2 + p0);

					p01 = p01.Scale(1.0 / sqrt(p01.DotProduct(p01).GetScalar()));
					p12 = p12.Scale(1.0 / sqrt(p12.DotProduct(p12).GetScalar()));
					p20 = p20.Scale(1.0 / sqrt(p20.DotProduct(p20).GetScalar()));

					ndAssert(fabs(p01.DotProduct(p01).GetScalar() - double(1.0f)) < double(1.0e-4f));
					ndAssert(fabs(p12.DotProduct(p12).GetScalar() - double(1.0f)) < double(1.0e-4f));
					ndAssert(fabs(p20.DotProduct(p20).GetScalar() - double(1.0f)) < double(1.0e-4f));

					TessellateTriangle(level - 1, p0, p01, p20, count);
					TessellateTriangle(level - 1, p1, p12, p01, count);
					TessellateTriangle(level - 1, p2, p20, p12, count);
					TessellateTriangle(level - 1, p01, p12, p20, count);
				}
				else 
				{
					ndBigPlane n(p0, p1, p2);
					n = n.Scale(double(1.0f) / sqrt(n.DotProduct(n).GetScalar()));
					n.m_w = double(0.0f);
					ndInt32 index = ndBitReversal(count, sizeof(m_normal) / sizeof(m_normal[0]));
					m_normal[index] = n;
					count++;
					ndAssert(count <= ndInt32(sizeof(m_normal) / sizeof(m_normal[0])));
				}
			}

			ndBigVector m_normal[128];
			ndInt32 m_count;
		};

		ConvexHull::ConvexHull(const double* const vertexCloud, ndInt32 strideInBytes, ndInt32 count, double distTol, ndInt32 maxVertexCount)
			:ndList<ndConvexHull3dFace, ndContainersFreeListAlloc<ndConvexHull3dFace>>()
			,m_aabbP0(ndBigVector::m_zero)
			,m_aabbP1(ndBigVector::m_zero)
			,m_points()
			,m_diag(0.0)
		{
			m_points.SetCount(0);
			ConvexHull3dPointSet accelerator(vertexCloud, strideInBytes, count);
			BuildHull(accelerator, distTol, maxVertexCount);
		}

		ConvexHull::ConvexHull(ConvexHull3dPointSet& accelerator, double distTol, ndInt32 maxVertexCount)
			:ndList<ndConvexHull3dFace, ndContainersFreeListAlloc<ndConvexHull3dFace>>()
			,m_aabbP0(ndBigVector::m_zero)
			,m_aabbP1(ndBigVector::m_zero)
			,m_points()
			,m_diag(0.0)
		{
			m_points.SetCount(0);
			#ifdef _DEBUG
			for (ndInt32 i = 0; i < ndInt32 (accelerator.GetCount()); i++)
			{
				ndAssert (accelerator[i].m_mark == 0);
			}
			#endif
			BuildHull(accelerator, distTol, maxVertexCount);
		}

		ConvexHull::~ConvexHull()
		{
		}

		const ndArray<ndBigVector>& ConvexHull::GetVertexPool() const
		{
			return m_points;
		}

		void ConvexHull::BuildHull(ConvexHull3dPointSet& accelerator, double distTol, ndInt32 maxVertexCount)
		{
			ConvexHullAABBTreeNode* const tree = InitVertexArray(accelerator);
			if (tree)
			{
				CalculateConvexHull3d(tree, accelerator, ndInt32 (accelerator.GetCount()), distTol, maxVertexCount);
			}
		}

		ndInt32 ConvexHull::SupportVertex(ConvexHullAABBTreeNode** const treePointer, const ndArray<ConvexHullVertex>& points, const ndBigVector& dirPlane, const bool removeEntry) const
		{
			ndFixSizeArray<double, DG_STACK_DEPTH_3D> aabbProjection;
			ndFixSizeArray<const ConvexHullAABBTreeNode*, DG_STACK_DEPTH_3D> stackPool;
			
			const ndBigVector dir(dirPlane);

			ndInt32 index = -1;
			stackPool.PushBack(*treePointer);
			aabbProjection.PushBack (double(1.0e20f));
			double maxProj = double(-1.0e20f);
			ndInt32 ix = (dir[0] > double(0.0f)) ? 1 : 0;
			ndInt32 iy = (dir[1] > double(0.0f)) ? 1 : 0;
			ndInt32 iz = (dir[2] > double(0.0f)) ? 1 : 0;
			while (stackPool.GetCount())
			{
				double boxSupportValue = aabbProjection.Pop();
				const ConvexHullAABBTreeNode* const me = stackPool.Pop();
				if (boxSupportValue > maxProj)
				{
					if (me->m_left && me->m_right)
					{
						const ndBigVector leftSupportPoint(me->m_left->m_box[ix].m_x, me->m_left->m_box[iy].m_y, me->m_left->m_box[iz].m_z, 0.0);
						double leftSupportDist = leftSupportPoint.DotProduct(dir).GetScalar();

						const ndBigVector rightSupportPoint(me->m_right->m_box[ix].m_x, me->m_right->m_box[iy].m_y, me->m_right->m_box[iz].m_z, 0.0);
						double rightSupportDist = rightSupportPoint.DotProduct(dir).GetScalar();

						if (rightSupportDist >= leftSupportDist)
						{
							aabbProjection.PushBack(leftSupportDist);
							stackPool.PushBack(me->m_left);
							aabbProjection.PushBack(rightSupportDist);
							stackPool.PushBack(me->m_right);
						}
						else
						{
							aabbProjection.PushBack(rightSupportDist);
							stackPool.PushBack(me->m_right);
							aabbProjection.PushBack(leftSupportDist);
							stackPool.PushBack(me->m_left);
						}
					}
					else
					{
						ConvexHull3dPointCluster* const cluster = (ConvexHull3dPointCluster*)me;
						for (ndInt32 i = 0; i < cluster->m_count; ++i)
						{
							const ConvexHullVertex& p = points[cluster->m_indices[i]];
							ndAssert(p.m_x >= cluster->m_box[0].m_x);
							ndAssert(p.m_x <= cluster->m_box[1].m_x);
							ndAssert(p.m_y >= cluster->m_box[0].m_y);
							ndAssert(p.m_y <= cluster->m_box[1].m_y);
							ndAssert(p.m_z >= cluster->m_box[0].m_z);
							ndAssert(p.m_z <= cluster->m_box[1].m_z);
							if (!p.m_mark)
							{
								//ndAssert(p.m_w == double(0.0f));
								double dist = p.DotProduct(dir).GetScalar();
								if (dist > maxProj)
								{
									maxProj = dist;
									index = cluster->m_indices[i];
								}
							}
							else if (removeEntry)
							{
								cluster->m_indices[i] = cluster->m_indices[cluster->m_count - 1];
								cluster->m_count = cluster->m_count - 1;
								i--;
							}
						}

						if (cluster->m_count == 0)
						{
							ConvexHullAABBTreeNode* const parent = cluster->m_parent;
							if (parent)
							{
								ConvexHullAABBTreeNode* const sibling = (parent->m_left != cluster) ? parent->m_left : parent->m_right;
								ndAssert(sibling != cluster);
								ConvexHullAABBTreeNode* const grandParent = parent->m_parent;
								if (grandParent)
								{
									sibling->m_parent = grandParent;
									if (grandParent->m_right == parent)
									{
										grandParent->m_right = sibling;
									}
									else
									{
										grandParent->m_left = sibling;
									}
								}
								else
								{
									sibling->m_parent = nullptr;
									*treePointer = sibling;
								}
							}
						}
					}
				}
			}

			ndAssert(index != -1);
			return index;
		}

		double ConvexHull::TetrahedrumVolume(const ndBigVector& p0, const ndBigVector& p1, const ndBigVector& p2, const ndBigVector& p3) const
		{
			const ndBigVector p1p0(p1 - p0);
			const ndBigVector p2p0(p2 - p0);
			const ndBigVector p3p0(p3 - p0);
			return p3p0.DotProduct(p1p0.CrossProduct(p2p0)).GetScalar();
		}

		ConvexHullAABBTreeNode* ConvexHull::InitVertexArray(ConvexHull3dPointSet& accelerator)
		{
			ConvexHullAABBTreeNode* tree = accelerator.BuildAccelerator();
			if (tree)
			{
				ndInt32 count = ndInt32(accelerator.GetCount());
				if (count < 4)
				{
					m_points.SetCount(0);
					return nullptr;
				}
				m_points.SetCount(4);

				m_aabbP0 = tree->m_box[0];
				m_aabbP1 = tree->m_box[1];

				ndBigVector boxSize(tree->m_box[1] - tree->m_box[0]);
				m_diag = double(sqrt(boxSize.DotProduct(boxSize).GetScalar()));
				const ndNormalMap& normalMap = ndNormalMap::GetNormaMap();

				ndInt32 index0 = SupportVertex(&tree, accelerator, normalMap.m_normal[0]);
				m_points[0] = accelerator[index0];
				accelerator[index0].m_mark = 1;

				bool validTetrahedrum = false;
				ndBigVector e1(ndBigVector::m_zero);
				for (ndInt32 i = 1; i < normalMap.m_count; ++i)
				{
					ndInt32 index = SupportVertex(&tree, accelerator, normalMap.m_normal[i]);

					e1 = accelerator[index] - m_points[0];
					double error2 = e1.DotProduct(e1).GetScalar();
					if (error2 > (double(1.0e-4f) * m_diag * m_diag))
					{
						m_points[1] = accelerator[index];
						accelerator[index].m_mark = 1;
						validTetrahedrum = true;
						break;
					}
				}
				if (!validTetrahedrum)
				{
					m_points.SetCount(0);
					return nullptr;
				}

				validTetrahedrum = false;
				ndBigVector e2(ndBigVector::m_zero);
				ndBigVector normal(ndBigVector::m_zero);
				for (ndInt32 i = 2; i < normalMap.m_count; ++i)
				{
					ndInt32 index = SupportVertex(&tree, accelerator, normalMap.m_normal[i]);
					e2 = accelerator[index] - m_points[0];
					normal = e1.CrossProduct(e2);
					double error2 = sqrt(normal.DotProduct(normal).GetScalar());
					if (error2 > (double(1.0e-4f) * m_diag * m_diag))
					{
						m_points[2] = accelerator[index];
						accelerator[index].m_mark = 1;
						validTetrahedrum = true;
						break;
					}
				}

				if (!validTetrahedrum)
				{
					m_points.SetCount(0);
					return nullptr;
				}

				// find the largest possible tetrahedron
				validTetrahedrum = false;
				ndBigVector e3(ndBigVector::m_zero);

				index0 = SupportVertex(&tree, accelerator, normal);
				e3 = accelerator[index0] - m_points[0];
				double err2 = normal.DotProduct(e3).GetScalar();
				if (fabs(err2) > (double(1.0e-6f) * m_diag * m_diag))
				{
					// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
					m_points[3] = accelerator[index0];
					accelerator[index0].m_mark = 1;
					validTetrahedrum = true;
				}
				if (!validTetrahedrum)
				{
					ndBigVector n(normal.Scale(double(-1.0f)));
					ndInt32 index = SupportVertex(&tree, accelerator, n);
					e3 = accelerator[index] - m_points[0];
					double error2 = normal.DotProduct(e3).GetScalar();
					if (fabs(error2) > (double(1.0e-6f) * m_diag * m_diag))
					{
						// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
						m_points[3] = accelerator[index];
						accelerator[index].m_mark = 1;
						validTetrahedrum = true;
					}
				}
				if (!validTetrahedrum)
				{
					for (ndInt32 i = 3; i < normalMap.m_count; ++i)
					{
						ndInt32 index = SupportVertex(&tree, accelerator, normalMap.m_normal[i]);

						//make sure the volume of the fist tetrahedral is no negative
						e3 = accelerator[index] - m_points[0];
						double error2 = normal.DotProduct(e3).GetScalar();
						if (fabs(error2) > (double(1.0e-6f) * m_diag * m_diag))
						{
							// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
							m_points[3] = accelerator[index];
							accelerator[index].m_mark = 1;
							validTetrahedrum = true;
							break;
						}
					}
				}
				if (!validTetrahedrum)
				{
					// the points do not form a convex hull
					m_points.SetCount(0);
					return nullptr;
				}

				double volume = TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]);
				if (volume > double(0.0f))
				{
					ndSwap(m_points[2], m_points[3]);
				}
				ndAssert(TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]) < double(0.0f));
			}
			return tree;
		}

		ConvexHull::ndNode* ConvexHull::AddFace(ndInt32 i0, ndInt32 i1, ndInt32 i2)
		{
			ndNode* const node = Append();
			ndConvexHull3dFace& face = node->GetInfo();

			face.m_index[0] = i0;
			face.m_index[1] = i1;
			face.m_index[2] = i2;
			return node;
		}

		void ConvexHull::CalculateConvexHull3d(ConvexHullAABBTreeNode* vertexTree, ndArray<ConvexHullVertex>& points, ndInt32 count, double distTol, ndInt32 maxVertexCount)
		{
			distTol = fabs(distTol) * m_diag;
			ndNode* const f0Node = AddFace(0, 1, 2);
			ndNode* const f1Node = AddFace(0, 2, 3);
			ndNode* const f2Node = AddFace(2, 1, 3);
			ndNode* const f3Node = AddFace(1, 0, 3);

			ndConvexHull3dFace* const f0 = &f0Node->GetInfo();
			ndConvexHull3dFace* const f1 = &f1Node->GetInfo();
			ndConvexHull3dFace* const f2 = &f2Node->GetInfo();
			ndConvexHull3dFace* const f3 = &f3Node->GetInfo();

			f0->m_twin[0] = f3Node;
			f0->m_twin[1] = f2Node;
			f0->m_twin[2] = f1Node;

			f1->m_twin[0] = f0Node;
			f1->m_twin[1] = f2Node;
			f1->m_twin[2] = f3Node;

			f2->m_twin[0] = f0Node;
			f2->m_twin[1] = f3Node;
			f2->m_twin[2] = f1Node;

			f3->m_twin[0] = f0Node;
			f3->m_twin[1] = f1Node;
			f3->m_twin[2] = f2Node;
	
			ndList<ndNode*, ndContainersFreeListAlloc<ndNode*>> boundaryFaces;
			boundaryFaces.Append(f0Node);
			boundaryFaces.Append(f1Node);
			boundaryFaces.Append(f2Node);
			boundaryFaces.Append(f3Node);

			m_points.SetCount(count);

			count -= 4;
			maxVertexCount -= 4;
			size_t currentIndex = 4;

			ndFixSizeArray<ndNode*, 1024 * 2> stack;
			ndFixSizeArray<ndNode*, 1024 * 2> coneList;
			ndFixSizeArray<ndNode*, 1024 * 2> deleteList;

			while (boundaryFaces.GetCount() && count && (maxVertexCount > 0))
			{
				// my definition of the optimal convex hull of a given vertex count,
				// is the convex hull formed by a subset of the input vertex that minimizes the volume difference
				// between the perfect hull formed from all input vertex and the hull of the sub set of vertex.
				// When using a priority heap this algorithms will generate the an optimal of a fix vertex count.
				// Since all Newton's tools do not have a limit on the point count of a convex hull, I can use either a stack or a queue.
				// a stack maximize construction speed, a Queue tend to maximize the volume of the generated Hull approaching a perfect Hull.
				// For now we use a queue.
				// For general hulls it does not make a difference if we use a stack, queue, or a priority heap.
				// perfect optimal hull only apply for when build hull of a limited vertex count.
				//
				// Also when building Hulls of a limited vertex count, this function runs in constant time.
				// yes that is correct, it does not makes a difference if you build a N point hull from 100 vertex
				// or from 100000 vertex input array.

				// using a queue (some what slower by better hull when reduced vertex count is desired)
				bool isvalid;
				ndNode* const faceNode = boundaryFaces.GetLast()->GetInfo();
				ndConvexHull3dFace* const face = &faceNode->GetInfo();
				ndBigPlane planeEquation(face->GetPlaneEquation(&m_points[0], isvalid));

				ndInt32 index = 0;
				double dist = 0;
				ndBigVector p;
				if (isvalid)
				{
					index = SupportVertex(&vertexTree, points, planeEquation);
					p = points[index];
					dist = planeEquation.Evalue(p);
				}

				if (isvalid && (dist >= distTol) && (face->Evalue(&m_points[0], p) > double(0.0f)))
				{
					ndAssert(faceNode);
					ndAssert(stack.GetCount() == 0);
					ndAssert(deleteList.GetCount() == 0);

					stack.PushBack(faceNode);
					while (stack.GetCount())
					{
						ndNode* const node1 = stack.Pop();
						ndConvexHull3dFace* const face1 = &node1->GetInfo();
			
						if (!face1->m_mark && (face1->Evalue(&m_points[0], p) > double(0.0f)))
						{
							#ifdef _DEBUG
							for (ndInt32 i = 0; i < deleteList.GetCount(); ++i)
							{
								ndAssert(deleteList[i] != node1);
							}
							#endif

							face1->m_mark = 1;
							deleteList.PushBack(node1);
							for (ndInt32 i = 0; i < 3; ++i)
							{
								ndNode* const twinNode = face1->m_twin[i];
								ndAssert(twinNode);
								ndConvexHull3dFace* const twinFace = &twinNode->GetInfo();
								if (!twinFace->m_mark)
								{
									stack.PushBack(twinNode);
								}
							}
						}
					}
			
					m_points[ndInt32(currentIndex)] = points[index];
					points[index].m_mark = 1;
			
					coneList.SetCount(0);
					for (ndInt32 i = 0; i < deleteList.GetCount(); ++i)
					{
						ndNode* const node1 = deleteList[i];
						ndConvexHull3dFace* const face1 = &node1->GetInfo();
						ndAssert(face1->m_mark == 1);
						for (ndInt32 j0 = 0; j0 < 3; j0++)
						{
							ndNode* const twinNode = face1->m_twin[j0];
							ndConvexHull3dFace* const twinFace = &twinNode->GetInfo();
							if (!twinFace->m_mark)
							{
								ndInt32 j1 = (j0 == 2) ? 0 : j0 + 1;
								ndNode* const newNode = AddFace(ndInt32(currentIndex), face1->m_index[j0], face1->m_index[j1]);
								boundaryFaces.Addtop(newNode);
			
								ndConvexHull3dFace* const newFace = &newNode->GetInfo();
								newFace->m_twin[1] = twinNode;
								for (ndInt32 k = 0; k < 3; ++k)
								{
									if (twinFace->m_twin[k] == node1)
									{
										twinFace->m_twin[k] = newNode;
									}
								}
								coneList.PushBack(newNode);
							}
						}
					}
			
					for (ndInt32 i = 0; i < (coneList.GetCount() - 1); ++i)
					{
						ndNode* const nodeA = coneList[i];
						ndConvexHull3dFace* const faceA = &nodeA->GetInfo();
						ndAssert(faceA->m_mark == 0);
						for (ndInt32 j = i + 1; j < coneList.GetCount(); ++j)
						{
							ndNode* const nodeB = coneList[j];
							ndConvexHull3dFace* const faceB = &nodeB->GetInfo();
							ndAssert(faceB->m_mark == 0);
							if (faceA->m_index[2] == faceB->m_index[1])
							{
								faceA->m_twin[2] = nodeB;
								faceB->m_twin[0] = nodeA;
								break;
							}
						}
			
						for (ndInt32 j = i + 1; j < coneList.GetCount(); ++j)
						{
							ndNode* const nodeB = coneList[j];
							ndConvexHull3dFace* const faceB = &nodeB->GetInfo();
							ndAssert(faceB->m_mark == 0);
							if (faceA->m_index[1] == faceB->m_index[2])
							{
								faceA->m_twin[0] = nodeB;
								faceB->m_twin[2] = nodeA;
								break;
							}
						}
					}
			
					for (ndInt32 i = deleteList.GetCount() - 1; i >= 0; --i)
					{
						ndNode* const node = deleteList.Pop();
						boundaryFaces.Remove(node);
						Remove(node);
					}

					maxVertexCount--;
					currentIndex++;
					count--;
				}
				else
				{
					boundaryFaces.Remove(faceNode);
				}
			}
			m_points.SetCount(ndInt32 (currentIndex));
		}
	}
}
