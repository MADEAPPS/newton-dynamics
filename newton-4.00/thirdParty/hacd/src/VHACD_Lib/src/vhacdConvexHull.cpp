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

#include "vhacdConvexHull.h"

namespace nd
{
	namespace VHACD
	{

		#define VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE 8

		ConvexHullFace::ConvexHullFace()
		{
			m_mark = 0;
			m_twin[0] = nullptr;
			m_twin[1] = nullptr;
			m_twin[2] = nullptr;
		}

		hullPlane ConvexHullFace::GetPlaneEquation(const hullVector* const pointArray, bool& isvalid) const
		{
			const hullVector& p0 = pointArray[m_index[0]];
			const hullVector& p1 = pointArray[m_index[1]];
			const hullVector& p2 = pointArray[m_index[2]];
			hullPlane plane(p0, p1, p2);

			isvalid = false;
			double mag2 = plane.DotProduct(plane);
			if (mag2 > 1.0e-16f)
			{
				isvalid = true;
				plane = plane.Scale(1.0f / sqrt(mag2));
			}
			return plane;
		}

		double ConvexHullFace::Evalue(const hullVector* const pointArray, const hullVector& point) const
		{
			const hullVector& p0 = pointArray[m_index[0]];
			const hullVector& p1 = pointArray[m_index[1]];
			const hullVector& p2 = pointArray[m_index[2]];

			double matrix[3][3];
			for (int i = 0; i < 3; ++i) 
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
	
			Googol exactMatrix[3][3];
			for (int i = 0; i < 3; ++i) 
			{
				exactMatrix[0][i] = Googol(p2[i]) - Googol(p0[i]);
				exactMatrix[1][i] = Googol(p1[i]) - Googol(p0[i]);
				exactMatrix[2][i] = Googol(point[i]) - Googol(p0[i]);
			}
			return Determinant3x3(exactMatrix);
		}

		class ConvexHullVertex : public hullVector
		{
			public:
			int m_mark;
		};

		class ConvexHullAABBTreeNode
		{
			public:
			//ConvexHullAABBTreeNode(ConvexHullAABBTreeNode* const parent)
			ConvexHullAABBTreeNode()
				:m_left(nullptr)
				,m_right(nullptr)
				,m_parent(nullptr)
			{
			}

			ConvexHullAABBTreeNode(ConvexHullAABBTreeNode* const parent)
				:m_left(nullptr)
				,m_right(nullptr)
				,m_parent(parent)
			{
			}

			hullVector m_box[2];
			ConvexHullAABBTreeNode* m_left;
			ConvexHullAABBTreeNode* m_right;
			ConvexHullAABBTreeNode* m_parent;
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

			int m_count;
			int m_indices[VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE];
		};

		class ConvexHull::ndNormalMap
		{
			public:
			ndNormalMap()
				:m_count(sizeof(m_normal) / sizeof(m_normal[0]))
			{
				hullVector p0(double(1.0f), double(0.0f), double(0.0f), double(0.0f));
				hullVector p1(double(-1.0f), double(0.0f), double(0.0f), double(0.0f));
				hullVector p2(double(0.0f), double(1.0f), double(0.0f), double(0.0f));
				hullVector p3(double(0.0f), double(-1.0f), double(0.0f), double(0.0f));
				hullVector p4(double(0.0f), double(0.0f), double(1.0f), double(0.0f));
				hullVector p5(double(0.0f), double(0.0f), double(-1.0f), double(0.0f));

				int count = 0;
				int subdivitions = 2;
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

			void TessellateTriangle(int level, const hullVector& p0, const hullVector& p1, const hullVector& p2, int& count)
			{
				if (level) 
				{
					_ASSERT(fabs(p0.DotProduct(p0) - double(1.0f)) < double(1.0e-4f));
					_ASSERT(fabs(p1.DotProduct(p1) - double(1.0f)) < double(1.0e-4f));
					_ASSERT(fabs(p2.DotProduct(p2) - double(1.0f)) < double(1.0e-4f));
					hullVector p01(p0 + p1);
					hullVector p12(p1 + p2);
					hullVector p20(p2 + p0);

					p01 = p01.Scale(1.0 / sqrt(p01.DotProduct(p01)));
					p12 = p12.Scale(1.0 / sqrt(p12.DotProduct(p12)));
					p20 = p20.Scale(1.0 / sqrt(p20.DotProduct(p20)));

					_ASSERT(fabs(p01.DotProduct(p01) - double(1.0f)) < double(1.0e-4f));
					_ASSERT(fabs(p12.DotProduct(p12) - double(1.0f)) < double(1.0e-4f));
					_ASSERT(fabs(p20.DotProduct(p20) - double(1.0f)) < double(1.0e-4f));

					TessellateTriangle(level - 1, p0, p01, p20, count);
					TessellateTriangle(level - 1, p1, p12, p01, count);
					TessellateTriangle(level - 1, p2, p20, p12, count);
					TessellateTriangle(level - 1, p01, p12, p20, count);
				}
				else 
				{
					hullPlane n(p0, p1, p2);
					n = n.Scale(double(1.0f) / sqrt(n.DotProduct(n)));
					n.m_w = double(0.0f);
					int index = dBitReversal(count, sizeof(m_normal) / sizeof(m_normal[0]));
					m_normal[index] = n;
					count++;
					_ASSERT(count <= int(sizeof(m_normal) / sizeof(m_normal[0])));
				}
			}

			hullVector m_normal[128];
			int m_count;
		};

		ConvexHull::ConvexHull(const double* const vertexCloud, int strideInBytes, int count, double distTol, int maxVertexCount)
			:List<ConvexHullFace>()
			,m_aabbP0(0)
			,m_aabbP1(0)
			,m_diag()
			,m_points()
		{
			m_points.resize(0);
			if (count >= 4)
			{
				BuildHull(vertexCloud, strideInBytes, count, distTol, maxVertexCount);
			}
		}

		ConvexHull::~ConvexHull()
		{
		}

		const std::vector<hullVector>& ConvexHull::GetVertexPool() const
		{
			return m_points;
		}


		void ConvexHull::BuildHull(const double* const vertexCloud, int strideInBytes, int count, double distTol, int maxVertexCount)
		{
			int treeCount = count / (VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE >> 1);
			if (treeCount < 4)
			{
				treeCount = 4;
			}
			treeCount *= 2;

			std::vector<ConvexHullVertex> points(count);
			std::vector<ConvexHull3dPointCluster> treePool(treeCount + 256);
			points.resize(count);
			treePool.resize(treeCount + 256);

			const int stride = int(strideInBytes / sizeof(double));
			for (int i = 0; i < count; ++i)
			{
				int index = i * stride;
				hullVector& vertex = points[i];
				vertex = hullVector(vertexCloud[index], vertexCloud[index + 1], vertexCloud[index + 2], double(0.0f));
				points[i].m_mark = 0;
			}
			count = InitVertexArray(points, &treePool[0], sizeof (ConvexHull3dPointCluster) * int (treePool.size()));

			if (m_points.size() >= 4)
			{
				CalculateConvexHull3d(&treePool[0], points, count, distTol, maxVertexCount);
			}
		}

		void ConvexHull::GetUniquePoints(std::vector<ConvexHullVertex>& points)
		{
			class CompareVertex
			{
				public:
				int Compare(const ConvexHullVertex& elementA, const ConvexHullVertex& elementB) const
				{
					for (int i = 0; i < 3; i++)
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

			int count = int(points.size());
			Sort<ConvexHullVertex, CompareVertex>(&points[0], count);

			int indexCount = 0;
			CompareVertex compareVetex;
			for (int i = 1; i < count; ++i)
			{
				for (; i < count; ++i)
				{
					if (compareVetex.Compare(points[indexCount], points[i]))
					{
						indexCount++;
						points[indexCount] = points[i];
						break;
					}
				}
			}
			points.resize(indexCount + 1);
		}

		ConvexHullAABBTreeNode* ConvexHull::BuildTreeRecurse(ConvexHullAABBTreeNode* const parent, ConvexHullVertex* const points, int count, int baseIndex, char** memoryPool, int& maxMemSize) const
		{
			ConvexHullAABBTreeNode* tree = nullptr;

			_ASSERT(count);
			hullVector minP(double(1.0e15f));
			hullVector maxP(-double(1.0e15f));
			if (count <= VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE)
			{
				ConvexHull3dPointCluster* const clump = new (*memoryPool) ConvexHull3dPointCluster();
				*memoryPool += sizeof(ConvexHull3dPointCluster);
				maxMemSize -= sizeof(ConvexHull3dPointCluster);
				_ASSERT(maxMemSize >= 0);

				_ASSERT(clump);
				clump->m_count = count;
				for (int i = 0; i < count; ++i)
				{
					clump->m_indices[i] = i + baseIndex;

					const hullVector& p = points[i];
					minP = minP.GetMin(p);
					maxP = maxP.GetMax(p);
				}

				clump->m_left = nullptr;
				clump->m_right = nullptr;
				tree = clump;
			}
			else
			{
				hullVector median(0);
				hullVector varian(0);
				for (int i = 0; i < count; ++i)
				{
					const hullVector& p = points[i];
					median += p;
					varian += p * p;
					minP = minP.GetMin(p);
					maxP = maxP.GetMax(p);
				}

				varian = varian.Scale(double(count)) - median * median;
				int index = 0;
				double maxVarian = double(-1.0e10f);
				for (int i = 0; i < 3; ++i)
				{
					if (varian[i] > maxVarian)
					{
						index = i;
						maxVarian = varian[i];
					}
				}
				hullVector center(median.Scale(double(1.0f) / double(count)));

				double test = center[index];

				int i0 = 0;
				int i1 = count - 1;
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
						Swap(points[i0], points[i1]);
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

				tree = new (*memoryPool) ConvexHullAABBTreeNode();
				*memoryPool += sizeof(ConvexHullAABBTreeNode);
				maxMemSize -= sizeof(ConvexHullAABBTreeNode);
				_ASSERT(maxMemSize >= 0);

				_ASSERT(i0);
				_ASSERT(count - i0);

				tree->m_left = BuildTreeRecurse(tree, points, i0, baseIndex, memoryPool, maxMemSize);
				tree->m_right = BuildTreeRecurse(tree, &points[i0], count - i0, i0 + baseIndex, memoryPool, maxMemSize);
			}

			_ASSERT(tree);
			tree->m_parent = parent;
			tree->m_box[0] = minP - hullVector(double(1.0e-3f));
			tree->m_box[1] = maxP + hullVector(double(1.0e-3f));
			return tree;
		}

		ConvexHullAABBTreeNode* ConvexHull::BuildTreeOld(std::vector<ConvexHullVertex>& points, char** const memoryPool, int& maxMemSize)
		{
			GetUniquePoints(points);
			int count = int(points.size());
			if (count < 4)
			{
				return nullptr;
			}
			return BuildTreeRecurse(nullptr, &points[0], count, 0, memoryPool, maxMemSize);
		}

		ConvexHullAABBTreeNode* ConvexHull::BuildTreeNew(std::vector<ConvexHullVertex>& points, char** const memoryPool, int& maxMemSize) const
		{
			class dCluster
			{
				public:
				hullVector m_sum;
				hullVector m_sum2;
				int m_start;
				int m_count;
			};

			dCluster firstCluster;
			firstCluster.m_start = 0;
			firstCluster.m_count = int (points.size());
			firstCluster.m_sum = hullVector(0);
			firstCluster.m_sum2 = hullVector(0);

			for (int i = 0; i < firstCluster.m_count; ++i)
			{
				const hullVector& p = points[i];
				firstCluster.m_sum += p;
				firstCluster.m_sum2 += p * p;
			}

			int baseCount = 0;
			const int clusterSize = 16;

			if (firstCluster.m_count > clusterSize)
			{
				dCluster spliteStack[128];
				spliteStack[0] = firstCluster;
				int stack = 1;

				while (stack)
				{
					stack--;
					dCluster cluster (spliteStack[stack]);

					const hullVector origin(cluster.m_sum.Scale(1.0f / cluster.m_count));
					const hullVector variance2(cluster.m_sum2.Scale(1.0f / cluster.m_count) - origin * origin);
					double maxVariance2 = Max(Max(variance2.X(), variance2.Y()), variance2.Z());

					if ((cluster.m_count <= clusterSize) || (stack > (sizeof(spliteStack) / sizeof(spliteStack[0]) - 4)) || (maxVariance2 < 1.e-4f))
					{
						// no sure if this is beneficial, 
						// the array is so small that seem too much overhead
						//int maxIndex = 0;
						//double min_x = 1.0e20f;
						//for (int i = 0; i < cluster.m_count; ++i)
						//{
						//	if (points[cluster.m_start + i].X() < min_x)
						//	{
						//		maxIndex = i;
						//		min_x = points[cluster.m_start + i].X();
						//	}
						//}
						//Swap(points[cluster.m_start], points[cluster.m_start + maxIndex]);
						//
						//for (int i = 2; i < cluster.m_count; ++i)
						//{
						//	int j = i;
						//	ConvexHullVertex tmp(points[cluster.m_start + i]);
						//	for (; points[cluster.m_start + j - 1].X() > tmp.X(); --j)
						//	{
						//		_ASSERT(j > 0);
						//		points[cluster.m_start + j] = points[cluster.m_start + j - 1];
						//	}
						//	points[cluster.m_start + j] = tmp;
						//}

						int count = cluster.m_count;
						for (int i = cluster.m_count - 1; i > 0; --i)
						{
							for (int j = i - 1; j >= 0; --j)
							{
								hullVector error(points[cluster.m_start + j] - points[cluster.m_start + i]);
								double mag2 = error.DotProduct(error);
								if (mag2 < 1.0e-6)
								{
									points[cluster.m_start + j] = points[cluster.m_start + i];
									count--;
									break;
								}
							}
						}

						_ASSERT(baseCount <= cluster.m_start);
						for (int i = 0; i < count; ++i)
						{
							points[baseCount] = points[cluster.m_start + i];
							baseCount++;
						}
					}
					else
					{
						int firstSortAxis = 0;
						if ((variance2.Y() >= variance2.X()) && (variance2.Y() >= variance2.Z()))
						{
							firstSortAxis = 1;
						}
						else if ((variance2.Z() >= variance2.X()) && (variance2.Z() >= variance2.Y()))
						{
							firstSortAxis = 2;
						}
						double axisVal = origin[firstSortAxis];

						int i0 = 0;
						int i1 = cluster.m_count - 1;

						const int start = cluster.m_start;
						while (i0 < i1)
						{
							while ((points[start + i0][firstSortAxis] <= axisVal) && (i0 < i1))
							{
								++i0;
							};

							while ((points[start + i1][firstSortAxis] > axisVal) && (i0 < i1))
							{
								--i1;
							}

							_ASSERT(i0 <= i1);
							if (i0 < i1)
							{
								Swap(points[start + i0], points[start + i1]);
								++i0;
								--i1;
							}
						}

						while ((points[start + i0][firstSortAxis] <= axisVal) && (i0 < cluster.m_count))
						{
							++i0;
						};

						#ifdef _DEBUG
						for (int i = 0; i < i0; ++i)
						{
							_ASSERT(points[start + i][firstSortAxis] <= axisVal);
						}

						for (int i = i0; i < cluster.m_count; ++i)
						{
							_ASSERT(points[start + i][firstSortAxis] > axisVal);
						}
						#endif

						hullVector xc(0);
						hullVector x2c(0);
						for (int i = 0; i < i0; ++i)
						{
							const hullVector& x = points[start + i];
							xc += x;
							x2c += x * x;
						}

						dCluster cluster_i1(cluster);
						cluster_i1.m_start = start + i0;
						cluster_i1.m_count = cluster.m_count - i0;
						cluster_i1.m_sum -= xc;
						cluster_i1.m_sum2 -= x2c;
						spliteStack[stack] = cluster_i1;
						_ASSERT(cluster_i1.m_count > 0);
						stack++;

						dCluster cluster_i0(cluster);
						cluster_i0.m_start = start;
						cluster_i0.m_count = i0;
						cluster_i0.m_sum = xc;
						cluster_i0.m_sum2 = x2c;
						_ASSERT(cluster_i0.m_count > 0);
						spliteStack[stack] = cluster_i0;
						stack++;
					}
				}
			}

			points.resize(baseCount);
			if (baseCount < 4)
			{
				return nullptr;
			}

			hullVector sum(0);
			hullVector sum2(0);
			hullVector minP(double(1.0e15f));
			hullVector maxP(-double(1.0e15f));
			class dTreeBox
			{
				public:
				hullVector m_min;
				hullVector m_max;
				hullVector m_sum;
				hullVector m_sum2;
				ConvexHullAABBTreeNode* m_parent;
				ConvexHullAABBTreeNode** m_child;
				int m_start;
				int m_count;
			};

			for (int i = 0; i < baseCount; ++i)
			{
				const hullVector& p = points[i];
				sum += p;
				sum2 += p * p;
				minP = minP.GetMin(p);
				maxP = maxP.GetMax(p);
			}
	
			dTreeBox treeBoxStack[128];
			treeBoxStack[0].m_start = 0;
			treeBoxStack[0].m_count = baseCount;
			treeBoxStack[0].m_sum = sum;
			treeBoxStack[0].m_sum2 = sum2;
			treeBoxStack[0].m_min = minP;
			treeBoxStack[0].m_max = maxP;
			treeBoxStack[0].m_child = nullptr;
			treeBoxStack[0].m_parent = nullptr;

			int stack = 1;
			ConvexHullAABBTreeNode* root = nullptr;
			while (stack)
			{
				stack--;
				dTreeBox box (treeBoxStack[stack]);
				if (box.m_count <= VHACD_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE)
				{
					ConvexHull3dPointCluster* const clump = new (*memoryPool) ConvexHull3dPointCluster(box.m_parent);
					*memoryPool += sizeof(ConvexHull3dPointCluster);
					maxMemSize -= sizeof(ConvexHull3dPointCluster);
					_ASSERT(maxMemSize >= 0);
		
					_ASSERT(clump);
					clump->m_count = box.m_count;
					for (int i = 0; i < box.m_count; ++i)
					{
						clump->m_indices[i] = i + box.m_start;
					}
					clump->m_box[0] = box.m_min;
					clump->m_box[1] = box.m_max;

					if (box.m_child)
					{
						*box.m_child = clump;
					}

					if (!root)
					{
						root = clump;
					}
				}
				else
				{
					const hullVector origin(box.m_sum.Scale(1.0f / box.m_count));
					const hullVector variance2(box.m_sum2.Scale(1.0f / box.m_count) - origin * origin);

					int firstSortAxis = 0;
					if ((variance2.Y() >= variance2.X()) && (variance2.Y() >= variance2.Z()))
					{
						firstSortAxis = 1;
					}
					else if ((variance2.Z() >= variance2.X()) && (variance2.Z() >= variance2.Y()))
					{
						firstSortAxis = 2;
					}
					double axisVal = origin[firstSortAxis];

					int i0 = 0;
					int i1 = box.m_count - 1;

					const int start = box.m_start;
					while (i0 < i1)
					{
						while ((points[start + i0][firstSortAxis] <= axisVal) && (i0 < i1))
						{
							++i0;
						};

						while ((points[start + i1][firstSortAxis] > axisVal) && (i0 < i1))
						{
							--i1;
						}

						_ASSERT(i0 <= i1);
						if (i0 < i1)
						{
							Swap(points[start + i0], points[start + i1]);
							++i0;
							--i1;
						}
					}

					while ((points[start + i0][firstSortAxis] <= axisVal) && (i0 < box.m_count))
					{
						++i0;
					};

					#ifdef _DEBUG
					for (int i = 0; i < i0; ++i)
					{
						_ASSERT(points[start + i][firstSortAxis] <= axisVal);
					}

					for (int i = i0; i < box.m_count; ++i)
					{
						_ASSERT(points[start + i][firstSortAxis] > axisVal);
					}
					#endif

					ConvexHullAABBTreeNode* const node = new (*memoryPool) ConvexHullAABBTreeNode(box.m_parent);
					*memoryPool += sizeof(ConvexHullAABBTreeNode);
					maxMemSize -= sizeof(ConvexHullAABBTreeNode);
					_ASSERT(maxMemSize >= 0);

					node->m_box[0] = box.m_min;
					node->m_box[1] = box.m_max;
					if (box.m_child)
					{
						*box.m_child = node;
					}

					if (!root)
					{
						root = node;
					}

					{
						hullVector xc(0);
						hullVector x2c(0);
						hullVector p0(double(1.0e15f));
						hullVector p1(-double(1.0e15f));
						for (int i = i0; i < box.m_count; ++i)
						{
							const hullVector& p = points[start + i];
							xc += p;
							x2c += p * p;
							p0 = p0.GetMin(p);
							p1 = p1.GetMax(p);
						}

						dTreeBox cluster_i1(box);
						cluster_i1.m_start = start + i0;
						cluster_i1.m_count = box.m_count - i0;
						cluster_i1.m_sum = xc;
						cluster_i1.m_sum2 = x2c;
						cluster_i1.m_min = p0;
						cluster_i1.m_max = p1;
						cluster_i1.m_parent = node;
						cluster_i1.m_child = &node->m_right;
						treeBoxStack[stack] = cluster_i1;
						_ASSERT(cluster_i1.m_count > 0);
						stack++;
					}

					{
						hullVector xc(0);
						hullVector x2c(0);
						hullVector p0(double(1.0e15f));
						hullVector p1(-double(1.0e15f));
						for (int i = 0; i < i0; ++i)
						{
							const hullVector& p = points[start + i];
							xc += p;
							x2c += p * p;
							p0 = p0.GetMin(p);
							p1 = p1.GetMax(p);
						}

						dTreeBox cluster_i0(box);
						cluster_i0.m_start = start;
						cluster_i0.m_count = i0;
						cluster_i0.m_min = p0;
						cluster_i0.m_max = p1;
						cluster_i0.m_sum = xc;
						cluster_i0.m_sum2 = x2c;
						cluster_i0.m_parent = node;
						cluster_i0.m_child = &node->m_left;
						_ASSERT(cluster_i0.m_count > 0);
						treeBoxStack[stack] = cluster_i0;
						stack++;
					}
				}
			}
	
			return root;
		}

		int ConvexHull::SupportVertex(ConvexHullAABBTreeNode** const treePointer, const std::vector<ConvexHullVertex>& points, const hullVector& dirPlane, const bool removeEntry) const
		{
		#define DG_STACK_DEPTH_3D 64
			double aabbProjection[DG_STACK_DEPTH_3D];
			const ConvexHullAABBTreeNode *stackPool[DG_STACK_DEPTH_3D];

			hullVector dir(dirPlane);

			int index = -1;
			int stack = 1;
			stackPool[0] = *treePointer;
			aabbProjection[0] = double(1.0e20f);
			double maxProj = double(-1.0e20f);
			int ix = (dir[0] > double(0.0f)) ? 1 : 0;
			int iy = (dir[1] > double(0.0f)) ? 1 : 0;
			int iz = (dir[2] > double(0.0f)) ? 1 : 0;
			while (stack)
			{
				stack--;
				double boxSupportValue = aabbProjection[stack];
				if (boxSupportValue > maxProj)
				{
					const ConvexHullAABBTreeNode* const me = stackPool[stack];

					if (me->m_left && me->m_right)
					{
						const hullVector leftSupportPoint(me->m_left->m_box[ix].X(), me->m_left->m_box[iy].Y(), me->m_left->m_box[iz].Z(), 0.0f);
						double leftSupportDist = leftSupportPoint.DotProduct(dir);

						const hullVector rightSupportPoint(me->m_right->m_box[ix].X(), me->m_right->m_box[iy].Y(), me->m_right->m_box[iz].Z(), 0.0f);
						double rightSupportDist = rightSupportPoint.DotProduct(dir);

						if (rightSupportDist >= leftSupportDist)
						{
							aabbProjection[stack] = leftSupportDist;
							stackPool[stack] = me->m_left;
							stack++;
							_ASSERT(stack < DG_STACK_DEPTH_3D);
							aabbProjection[stack] = rightSupportDist;
							stackPool[stack] = me->m_right;
							stack++;
							_ASSERT(stack < DG_STACK_DEPTH_3D);
						}
						else
						{
							aabbProjection[stack] = rightSupportDist;
							stackPool[stack] = me->m_right;
							stack++;
							_ASSERT(stack < DG_STACK_DEPTH_3D);
							aabbProjection[stack] = leftSupportDist;
							stackPool[stack] = me->m_left;
							stack++;
							_ASSERT(stack < DG_STACK_DEPTH_3D);
						}
					}
					else
					{
						ConvexHull3dPointCluster* const cluster = (ConvexHull3dPointCluster*)me;
						for (int i = 0; i < cluster->m_count; ++i)
						{
							const ConvexHullVertex& p = points[cluster->m_indices[i]];
							_ASSERT(p.X() >= cluster->m_box[0].X());
							_ASSERT(p.X() <= cluster->m_box[1].X());
							_ASSERT(p.Y() >= cluster->m_box[0].Y());
							_ASSERT(p.Y() <= cluster->m_box[1].Y());
							_ASSERT(p.Z() >= cluster->m_box[0].Z());
							_ASSERT(p.Z() <= cluster->m_box[1].Z());
							if (!p.m_mark)
							{
								//_ASSERT(p.m_w == double(0.0f));
								double dist = p.DotProduct(dir);
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
								_ASSERT(sibling != cluster);
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

			_ASSERT(index != -1);
			return index;
		}

		double ConvexHull::TetrahedrumVolume(const hullVector& p0, const hullVector& p1, const hullVector& p2, const hullVector& p3) const
		{
			const hullVector p1p0(p1 - p0);
			const hullVector p2p0(p2 - p0);
			const hullVector p3p0(p3 - p0);
			return p3p0.DotProduct(p1p0.CrossProduct(p2p0));
		}

		int ConvexHull::InitVertexArray(std::vector<ConvexHullVertex>& points, void* const memoryPool, int maxMemSize)
		{
		#if 1
			ConvexHullAABBTreeNode* tree = BuildTreeOld(points, (char**)&memoryPool, maxMemSize);
		#else
			ConvexHullAABBTreeNode* tree = BuildTreeNew(points, (char**)&memoryPool, maxMemSize);
		#endif
			int count = int (points.size());
			if (count < 4)
			{
				m_points.resize(0);
				return 0;
			}
		
			m_points.resize(count);
			m_aabbP0 = tree->m_box[0];
			m_aabbP1 = tree->m_box[1];
	
			hullVector boxSize(tree->m_box[1] - tree->m_box[0]);
			m_diag = double(sqrt(boxSize.DotProduct(boxSize)));
			const ndNormalMap& normalMap = ndNormalMap::GetNormaMap();
	
			int index0 = SupportVertex(&tree, points, normalMap.m_normal[0]);
			m_points[0] = points[index0];
			points[index0].m_mark = 1;
	
			bool validTetrahedrum = false;
			hullVector e1(0.0);
			for (int i = 1; i < normalMap.m_count; ++i)
			{
				int index = SupportVertex(&tree, points, normalMap.m_normal[i]);
				_ASSERT(index >= 0);
	
				e1 = points[index] - m_points[0];
				double error2 = e1.DotProduct(e1);
				if (error2 > (double(1.0e-4f) * m_diag * m_diag))
				{
					m_points[1] = points[index];
					points[index].m_mark = 1;
					validTetrahedrum = true;
					break;
				}
			}
			if (!validTetrahedrum)
			{
				m_points.resize(0);
				_ASSERT(0);
				return count;
			}
	
			validTetrahedrum = false;
			hullVector e2(0.0);
			hullVector normal(0.0);
			for (int i = 2; i < normalMap.m_count; ++i)
			{
				int index = SupportVertex(&tree, points, normalMap.m_normal[i]);
				_ASSERT(index >= 0);
				e2 = points[index] - m_points[0];
				normal = e1.CrossProduct(e2);
				double error2 = sqrt(normal.DotProduct(normal));
				if (error2 > (double(1.0e-4f) * m_diag * m_diag))
				{
					m_points[2] = points[index];
					points[index].m_mark = 1;
					validTetrahedrum = true;
					break;
				}
			}
	
			if (!validTetrahedrum)
			{
				m_points.resize(0);
				_ASSERT(0);
				return count;
			}
	
			// find the largest possible tetrahedron
			validTetrahedrum = false;
			hullVector e3(0.0);
	
			index0 = SupportVertex(&tree, points, normal);
			e3 = points[index0] - m_points[0];
			double err2 = normal.DotProduct(e3);
			if (fabs(err2) > (double(1.0e-6f) * m_diag * m_diag))
			{
				// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
				m_points[3] = points[index0];
				points[index0].m_mark = 1;
				validTetrahedrum = true;
			}
			if (!validTetrahedrum)
			{
				hullVector n(normal.Scale(double(-1.0f)));
				int index = SupportVertex(&tree, points, n);
				e3 = points[index] - m_points[0];
				double error2 = normal.DotProduct(e3);
				if (fabs(error2) > (double(1.0e-6f) * m_diag * m_diag))
				{
					// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
					m_points[3] = points[index];
					points[index].m_mark = 1;
					validTetrahedrum = true;
				}
			}
			if (!validTetrahedrum)
			{
				for (int i = 3; i < normalMap.m_count; ++i)
				{
					int index = SupportVertex(&tree, points, normalMap.m_normal[i]);
					_ASSERT(index >= 0);
	
					//make sure the volume of the fist tetrahedral is no negative
					e3 = points[index] - m_points[0];
					double error2 = normal.DotProduct(e3);
					if (fabs(error2) > (double(1.0e-6f) * m_diag * m_diag))
					{
						// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
						m_points[3] = points[index];
						points[index].m_mark = 1;
						validTetrahedrum = true;
						break;
					}
				}
			}
			if (!validTetrahedrum)
			{
				// the points do not form a convex hull
				m_points.resize(0);
				return count;
			}
	
			m_points.resize(4);
			double volume = TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]);
			if (volume > double(0.0f))
			{
				Swap(m_points[2], m_points[3]);
			}
			_ASSERT(TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]) < double(0.0f));
			return count;
		}

		ConvexHull::ndNode* ConvexHull::AddFace(int i0, int i1, int i2)
		{
			ndNode* const node = Append();
			ConvexHullFace& face = node->GetInfo();

			face.m_index[0] = i0;
			face.m_index[1] = i1;
			face.m_index[2] = i2;
			return node;
		}

		void ConvexHull::CalculateConvexHull3d(ConvexHullAABBTreeNode* vertexTree, std::vector<ConvexHullVertex>& points, int count, double distTol, int maxVertexCount)
		{
			distTol = fabs(distTol) * m_diag;
			ndNode* const f0Node = AddFace(0, 1, 2);
			ndNode* const f1Node = AddFace(0, 2, 3);
			ndNode* const f2Node = AddFace(2, 1, 3);
			ndNode* const f3Node = AddFace(1, 0, 3);

			ConvexHullFace* const f0 = &f0Node->GetInfo();
			ConvexHullFace* const f1 = &f1Node->GetInfo();
			ConvexHullFace* const f2 = &f2Node->GetInfo();
			ConvexHullFace* const f3 = &f3Node->GetInfo();

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
	
			List<ndNode*> boundaryFaces;
			boundaryFaces.Append(f0Node);
			boundaryFaces.Append(f1Node);
			boundaryFaces.Append(f2Node);
			boundaryFaces.Append(f3Node);

			m_points.resize(count);

			count -= 4;
			maxVertexCount -= 4;
			int currentIndex = 4;

			std::vector<ndNode*> stackPool;
			std::vector<ndNode*> coneListPool;
			std::vector<ndNode*> deleteListPool;

			stackPool.resize(1024 + count);
			coneListPool.resize(1024 + count);
			deleteListPool.resize(1024 + count);

			ndNode** const stack = &stackPool[0];
			ndNode** const coneList = &stackPool[0];
			ndNode** const deleteList = &deleteListPool[0];

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
				ConvexHullFace* const face = &faceNode->GetInfo();
				hullPlane planeEquation(face->GetPlaneEquation(&m_points[0], isvalid));

				int index = 0;
				double dist = 0;
				hullVector p;
				if (isvalid)
				{
					index = SupportVertex(&vertexTree, points, planeEquation);
					p = points[index];
					dist = planeEquation.Evalue(p);
				}

				if (isvalid && (dist >= distTol) && (face->Evalue(&m_points[0], p) > double(0.0f)))
				{
					stack[0] = faceNode;

					int stackIndex = 1;
					int deletedCount = 0;

					while (stackIndex)
					{
						stackIndex--;
						ndNode* const node1 = stack[stackIndex];
						ConvexHullFace* const face1 = &node1->GetInfo();
			
						if (!face1->m_mark && (face1->Evalue(&m_points[0], p) > double(0.0f)))
						{
							#ifdef _DEBUG
							for (int i = 0; i < deletedCount; ++i)
							{
								_ASSERT(deleteList[i] != node1);
							}
							#endif
			
							deleteList[deletedCount] = node1;
							deletedCount++;
							_ASSERT(deletedCount < int(deleteListPool.size()));
							face1->m_mark = 1;
							for (int i = 0; i < 3; ++i)
							{
								ndNode* const twinNode = face1->m_twin[i];
								_ASSERT(twinNode);
								ConvexHullFace* const twinFace = &twinNode->GetInfo();
								if (!twinFace->m_mark)
								{
									stack[stackIndex] = twinNode;
									stackIndex++;
									_ASSERT(stackIndex < int(stackPool.size()));
								}
							}
						}
					}
			
					m_points[currentIndex] = points[index];
					points[index].m_mark = 1;
			
					int newCount = 0;
					for (int i = 0; i < deletedCount; ++i)
					{
						ndNode* const node1 = deleteList[i];
						ConvexHullFace* const face1 = &node1->GetInfo();
						_ASSERT(face1->m_mark == 1);
						for (int j0 = 0; j0 < 3; j0++)
						{
							ndNode* const twinNode = face1->m_twin[j0];
							ConvexHullFace* const twinFace = &twinNode->GetInfo();
							if (!twinFace->m_mark)
							{
								int j1 = (j0 == 2) ? 0 : j0 + 1;
								ndNode* const newNode = AddFace(currentIndex, face1->m_index[j0], face1->m_index[j1]);
								boundaryFaces.Addtop(newNode);
			
								ConvexHullFace* const newFace = &newNode->GetInfo();
								newFace->m_twin[1] = twinNode;
								for (int k = 0; k < 3; k++)
								{
									if (twinFace->m_twin[k] == node1)
									{
										twinFace->m_twin[k] = newNode;
									}
								}
								coneList[newCount] = newNode;
								newCount++;
								_ASSERT(newCount < int(coneListPool.size()));
							}
						}
					}
			
					for (int i = 0; i < newCount - 1; ++i)
					{
						ndNode* const nodeA = coneList[i];
						ConvexHullFace* const faceA = &nodeA->GetInfo();
						_ASSERT(faceA->m_mark == 0);
						for (int j = i + 1; j < newCount; j++) 
						{
							ndNode* const nodeB = coneList[j];
							ConvexHullFace* const faceB = &nodeB->GetInfo();
							_ASSERT(faceB->m_mark == 0);
							if (faceA->m_index[2] == faceB->m_index[1])
							{
								faceA->m_twin[2] = nodeB;
								faceB->m_twin[0] = nodeA;
								break;
							}
						}
			
						for (int j = i + 1; j < newCount; j++)
						{
							ndNode* const nodeB = coneList[j];
							ConvexHullFace* const faceB = &nodeB->GetInfo();
							_ASSERT(faceB->m_mark == 0);
							if (faceA->m_index[1] == faceB->m_index[2])
							{
								faceA->m_twin[0] = nodeB;
								faceB->m_twin[2] = nodeA;
								break;
							}
						}
					}
			
					for (int i = 0; i < deletedCount; ++i)
					{
						ndNode* const node = deleteList[i];
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
			m_points.resize(currentIndex);
		}
	}
}


