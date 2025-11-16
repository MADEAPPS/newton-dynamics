/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"

#include "VHACD.h"
#include "vhacdMesh.h"
#include "vhacdVHACD.h"
#include "vhacdSArray.h"
#include "vhacdVector.h"
#include "vhacdVolume.h"
#include "vhacdConvexHull.h"
#include "vhacdConvexHullUtils.h"

namespace nd
{
	namespace VHACD 
	{
		#define MAX_DOUBLE (1.79769e+30)
		IVHACD* CreateVHACD(void)
		{
			return new VHACD();
		}

		void VHACD::ComputePrimitiveSet(const Parameters&)
		{
			VoxelSet* const vset = new VoxelSet;
			m_volume->Convert(*vset);
			m_pset = vset;

			delete m_volume;
			m_volume = nullptr;
		}

		bool VHACD::Compute(const float* const points,const uint32_t nPoints,
			const uint32_t* const triangles,const uint32_t nTriangles, const Parameters& params)
		{
			return ComputeACD(points, nPoints, triangles, nTriangles, params);
		}

		double ComputePreferredCuttingDirection(const PrimitiveSet* const tset, Vec3& dir)
		{
			double ex = tset->GetEigenValue(AXIS_X);
			double ey = tset->GetEigenValue(AXIS_Y);
			double ez = tset->GetEigenValue(AXIS_Z);
			double vx = (ey - ez) * (ey - ez);
			double vy = (ex - ez) * (ex - ez);
			double vz = (ex - ey) * (ex - ey);
			if (vx < vy && vx < vz) 
			{
				double e = ey * ey + ez * ez;
				dir[0] = 1.0;
				dir[1] = 0.0;
				dir[2] = 0.0;
				return (e == 0.0) ? 0.0 : 1.0 - vx / e;
			}
			else if (vy < vx && vy < vz) 
			{
				double e = ex * ex + ez * ez;
				dir[0] = 0.0;
				dir[1] = 1.0;
				dir[2] = 0.0;
				return (e == 0.0) ? 0.0 : 1.0 - vy / e;
			}
			else 
			{
				double e = ex * ex + ey * ey;
				dir[0] = 0.0;
				dir[1] = 0.0;
				dir[2] = 1.0;
				return (e == 0.0) ? 0.0 : 1.0 - vz / e;
			}
		}

		void ComputeAxesAlignedClippingPlanes(const VoxelSet& vset, const short downsampling, SArray<Plane>& planes)
		{
			const Triangle minV = vset.GetMinBBVoxels();
			const Triangle maxV = vset.GetMaxBBVoxels();
			Vec3 pt;
			Plane plane;
			const short i0 = short(minV[0]);
			const short i1 = short(maxV[0]);
			plane.m_a = 1.0;
			plane.m_b = 0.0;
			plane.m_c = 0.0;
			plane.m_axis = AXIS_X;
			for (short i = i0; i <= i1; i += downsampling) 
			{
				pt = vset.GetPoint(Vec3(i + 0.5, 0.0, 0.0));
				plane.m_d = -pt[0];
				plane.m_index = i;
				planes.PushBack(plane);
			}
			const short j0 = short(minV[1]);
			const short j1 = short(maxV[1]);
			plane.m_a = 0.0;
			plane.m_b = 1.0;
			plane.m_c = 0.0;
			plane.m_axis = AXIS_Y;
			for (short j = j0; j <= j1; j += downsampling) 
			{
				pt = vset.GetPoint(Vec3(0.0, j + 0.5, 0.0));
				plane.m_d = -pt[1];
				plane.m_index = j;
				planes.PushBack(plane);
			}
			const short k0 = short(minV[2]);
			const short k1 = short(maxV[2]);
			plane.m_a = 0.0;
			plane.m_b = 0.0;
			plane.m_c = 1.0;
			plane.m_axis = AXIS_Z;
			for (short k = k0; k <= k1; k += downsampling) 
			{
				pt = vset.GetPoint(Vec3(0.0, 0.0, k + 0.5));
				plane.m_d = -pt[2];
				plane.m_index = k;
				planes.PushBack(plane);
			}
		}

		void RefineAxesAlignedClippingPlanes(const VoxelSet& vset, const Plane& bestPlane, const short downsampling,
			SArray<Plane>& planes)
		{
			const Triangle minV = vset.GetMinBBVoxels();
			const Triangle maxV = vset.GetMaxBBVoxels();
			Vec3 pt;
			Plane plane;

			if (bestPlane.m_axis == AXIS_X) 
			{
				const short i0 = ndMax(short(minV[0]), short(bestPlane.m_index - downsampling));
				const short i1 = ndMin(short(maxV[0]), short(bestPlane.m_index + downsampling));
				plane.m_a = 1.0;
				plane.m_b = 0.0;
				plane.m_c = 0.0;
				plane.m_axis = AXIS_X;
				for (short i = i0; i <= i1; ++i) 
				{
					pt = vset.GetPoint(Vec3(i + 0.5, 0.0, 0.0));
					plane.m_d = -pt[0];
					plane.m_index = i;
					planes.PushBack(plane);
				}
			}
			else if (bestPlane.m_axis == AXIS_Y) 
			{
				const short j0 = ndMax(short(minV[1]), short(bestPlane.m_index - downsampling));
				const short j1 = ndMin(short(maxV[1]), short(bestPlane.m_index + downsampling));
				plane.m_a = 0.0;
				plane.m_b = 1.0;
				plane.m_c = 0.0;
				plane.m_axis = AXIS_Y;
				for (short j = j0; j <= j1; ++j) 
				{
					pt = vset.GetPoint(Vec3(0.0, j + 0.5, 0.0));
					plane.m_d = -pt[1];
					plane.m_index = j;
					planes.PushBack(plane);
				}
			}
			else 
			{
				const short k0 = ndMax(short(minV[2]), short(bestPlane.m_index - downsampling));
				const short k1 = ndMin(short(maxV[2]), short(bestPlane.m_index + downsampling));
				plane.m_a = 0.0;
				plane.m_b = 0.0;
				plane.m_c = 1.0;
				plane.m_axis = AXIS_Z;
				for (short k = k0; k <= k1; ++k) {
					pt = vset.GetPoint(Vec3(0.0, 0.0, k + 0.5));
					plane.m_d = -pt[2];
					plane.m_index = k;
					planes.PushBack(plane);
				}
			}
		}

		inline double ComputeLocalConcavity(const double volume, const double volumeCH)
		{
			return fabs(volumeCH - volume) / volumeCH;
		}
		inline double ComputeConcavity(const double volume, const double volumeCH, const double volume0)
		{
			return fabs(volumeCH - volume) / volume0;
		}

		void VHACD::ComputeBestClippingPlane(const PrimitiveSet* inputPSet, const double, const SArray<Plane>& planes,
			const Vec3& preferredCuttingDirection, const double w, const double alpha, const double beta,
			const int32_t convexhullDownsampling, Plane& bestPlane, double& minConcavity, const Parameters& params)
		{
			ndInt32 iBest = -1;
			ndInt32 nPlanes = ndInt32(planes.Size());
			double minTotal = MAX_DOUBLE;
			double minBalance = MAX_DOUBLE;
			double minSymmetry = MAX_DOUBLE;
			minConcavity = MAX_DOUBLE;

			PrimitiveSet* const onSurfacePSet = inputPSet->Create();
			inputPSet->SelectOnSurface(onSurfacePSet);

			PrimitiveSet** psets = new PrimitiveSet*[2];
			for (int32_t i = 0; i < 2; ++i) 
			{
				psets[i] = inputPSet->Create();
			}

			class CommonData
			{
				public:
				CommonData(VHACD* me, const Parameters& params)
					:m_me(me)
					,m_params(params)
				{
				}

				VHACD* m_me;
				double m_w;
				double m_beta;
				double m_alpha;
				const Parameters& m_params;
				PrimitiveSet* m_onSurfacePSet;
				const PrimitiveSet* m_inputPSet;
				const ConvexHullAABBTreeNode* m_voxelSpace;
				ndInt32 m_convexhullDownsampling;
				Mesh chs[VHACD_WORKERS_THREADS][2];
				SArray<Vec3> chPts[VHACD_WORKERS_THREADS][2];

				Vec3 m_preferredCuttingDirection;
				PrimitiveSet** m_psets;
			};

			class BestClippingPlaneJob : public Job
			{
				public:
				BestClippingPlaneJob()
					:Job()
				{
				}

				void Execute(int threadId)
				{
					Mesh& leftCH = m_commonData->chs[threadId][0];
					Mesh& rightCH = m_commonData->chs[threadId][1];
					rightCH.ResizePoints(0);
					leftCH.ResizePoints(0);
					rightCH.ResizeTriangles(0);
					leftCH.ResizeTriangles(0);

					SArray<Vec3>& leftCHPts = m_commonData->chPts[threadId][0];
					SArray<Vec3>& rightCHPts = m_commonData->chPts[threadId][1];

					leftCHPts.Resize(0);
					rightCHPts.Resize(0);
					m_commonData->m_onSurfacePSet->Intersect(m_plane, &rightCHPts, &leftCHPts, size_t (m_commonData->m_convexhullDownsampling * 32));
					m_commonData->m_inputPSet->GetConvexHull().Clip(m_plane, rightCHPts, leftCHPts);
					leftCH.ComputeConvexHull(leftCHPts.Data(), leftCHPts.Size());
					rightCH.ComputeConvexHull(rightCHPts.Data(), rightCHPts.Size());

					double volumeLeftCH = leftCH.ComputeVolume();
					double volumeRightCH = rightCH.ComputeVolume();
			
					// compute clipped volumes
					//double volumeLeft;
					//double volumeRight;
					//m_commonData->m_inputPSet->ComputeClippedVolumes(m_plane, volumeRight, volumeLeft);

					double volumeLeft;
					double volumeRight;
					m_commonData->m_inputPSet->ComputeClippedVolumes(m_commonData->m_voxelSpace, m_plane, volumeRight, volumeLeft);
					//ndAssert(ndAreEqual(volumeLeft__, volumeLeft, 1.0e-5));
					//ndAssert(ndAreEqual(volumeRight__, volumeRight, 1.0e-5));
			
					double concavityLeft = float(ComputeConcavity(volumeLeft, volumeLeftCH, m_commonData->m_me->m_volumeCH0));
					double concavityRight = float(ComputeConcavity(volumeRight, volumeRightCH, m_commonData->m_me->m_volumeCH0));
					double concavity = (concavityLeft + concavityRight);
			
					// compute cost
					m_concavity = concavity;
					m_balance = m_commonData->m_alpha * fabs(volumeLeft - volumeRight) / m_commonData->m_me->m_volumeCH0;
					double d = m_commonData->m_w * (m_commonData->m_preferredCuttingDirection[0] * m_plane.m_a + m_commonData->m_preferredCuttingDirection[1] * m_plane.m_b + m_commonData->m_preferredCuttingDirection[2] * m_plane.m_c);
					m_symmetry = m_commonData->m_beta * d;
				}

				double m_concavity;
				double m_balance;
				double m_symmetry;
				Plane m_plane;
				CommonData* m_commonData;
			};
			ndFixSizeArray<BestClippingPlaneJob, 1024> jobs;

			ConvexHull3dPointSet space;
			inputPSet->CalculateVoxelSpace(space);

			CommonData data(this, params);
			data.m_w = w;
			data.m_beta = beta;
			data.m_alpha = alpha;
			data.m_psets = psets;
			data.m_inputPSet = inputPSet;
			data.m_voxelSpace = space.GetTree();
			data.m_onSurfacePSet = onSurfacePSet;
			data.m_convexhullDownsampling = convexhullDownsampling;
			data.m_preferredCuttingDirection = preferredCuttingDirection;
#if 1
			for (ndInt32 i = 0; i < nPlanes; ++i)
			{
				jobs.PushBack(BestClippingPlaneJob());
				jobs[i].m_plane = planes[i];
				jobs[i].m_commonData = &data;
				m_parallelQueue.PushTask(&jobs[i]);
			}
			m_parallelQueue.Sync();
#else
			for (ndInt32 i = 0; i < nPlanes; ++i)
			{
				jobs.PushBack(BestClippingPlaneJob());
				jobs[i].m_plane = planes[i];
				jobs[i].m_commonData = &data;
				jobs[i].Execute(0);
			}
#endif

			iBest = 0;
			minConcavity = jobs[0].m_concavity;
			minBalance = jobs[0].m_balance;
			minSymmetry = jobs[0].m_symmetry;
			bestPlane = jobs[0].m_plane;
			minTotal = jobs[0].m_concavity + jobs[0].m_balance + jobs[0].m_symmetry;
			for (ndInt32 i = 1; i < nPlanes; ++i)
			{
				double total = jobs[i].m_concavity + jobs[i].m_balance + jobs[i].m_symmetry;
				if ((total < minTotal && int32_t(i) < iBest) || total < minTotal)
				{
					minConcavity = jobs[i].m_concavity;
					minBalance = jobs[i].m_balance;
					minSymmetry = jobs[i].m_symmetry;
					bestPlane = jobs[i].m_plane;
					minTotal = total;
					iBest = i;
				}
			}

			if (psets) 
			{
				for (int32_t i = 0; i < 2; ++i) 
				{
					delete psets[i];
				}
				delete[] psets;
			}
			delete onSurfacePSet;
		}

		void VHACD::ComputeACD(const Parameters& params)
		{
			SArray<PrimitiveSet*> temp;
			SArray<PrimitiveSet*> parts;
			SArray<PrimitiveSet*> inputParts;

			inputParts.PushBack(m_pset);
			m_pset = nullptr;
			SArray<Plane> planes;
			SArray<Plane> planesRef;
			ndInt32 sub = 0;
			bool firstIteration = true;
			m_volumeCH0 = 1.0;

			// Compute the decomposition depth based on the number of convex hulls being requested..
			ndInt32 depth = 1;
			ndInt32 hullCount = 2;
			while (params.m_maxConvexHulls > hullCount)
			{
				depth++;
				hullCount *= 2;
			}
			// We must always increment the decomposition depth one higher than the maximum number of hulls requested.
			// The reason for this is as follows.
			// Say, for example, the user requests 32 convex hulls exactly.  This would be a decomposition depth of 5.
			// However, when we do that, we do *not* necessarily get 32 hulls as a result.  This is because, during
			// the recursive descent of the binary tree, one or more of the leaf nodes may have no concavity and
			// will not be split.  So, in this way, even with a decomposition depth of 5, you can produce fewer than
			// 32 hulls.  So, in this case, we would set the decomposition depth to 6 (producing up to as high as 64 convex hulls).
			// Then, the merge step which combines over-described hulls down to the user requested amount, we will end up
			// getting exactly 32 convex hulls as a result.
			// We could just allow the artist to directly control the decomposition depth directly, but this would be a bit
			// too complex and the preference is simply to let them specify how many hulls they want and derive the solution
			// from that.
			depth++;

			while ((sub++ < depth) && (inputParts.Size() > 0)) 
			{
				double maxConcavity = 0.0;
				const size_t nInputParts = inputParts.Size();

				for (size_t p = 0; p < nInputParts; ++p) 
				{
					PrimitiveSet* const pset = inputParts[p];
					inputParts[p] = nullptr;
					double volume = pset->ComputeVolume();
					pset->ComputeBB();
					pset->ComputePrincipalAxes();

					pset->ComputeConvexHull(pset->GetConvexHull());
					double volumeCH = fabs(pset->GetConvexHull().ComputeVolume());
					if (firstIteration) 
					{
						m_volumeCH0 = volumeCH;
					}

					double concavity = float(ComputeConcavity(volume, volumeCH, m_volumeCH0));
					double error = 1.01 * pset->ComputeMaxVolumeError() / m_volumeCH0;
					// make the value smaller, later put it the parameters.
					error *= params.m_concavityToVolumeWeigh;

					if (firstIteration) 
					{
						firstIteration = false;
					}
			
					if (concavity > params.m_concavity && concavity > error) 
					{
						Vec3 preferredCuttingDirection;
						double w = ComputePreferredCuttingDirection(pset, preferredCuttingDirection);
						planes.Resize(0);

						ComputeAxesAlignedClippingPlanes(*((VoxelSet*)pset), short(params.m_planeDownsampling), planes);

						Plane bestPlane;
						double minConcavity = MAX_DOUBLE;
						ComputeBestClippingPlane(pset,
							volume,
							planes,
							preferredCuttingDirection,
							w,
							concavity * params.m_alpha,
							concavity * params.m_beta,
							int32_t(params.m_convexhullDownsampling),
							bestPlane,
							minConcavity,
							params);
						if ((params.m_planeDownsampling > 1 || params.m_convexhullDownsampling > 1)) 
						{
							planesRef.Resize(0);

							//VoxelSet* vset = (VoxelSet*)pset;
							RefineAxesAlignedClippingPlanes(*((VoxelSet*)pset), bestPlane, short(params.m_planeDownsampling), planesRef);

							ComputeBestClippingPlane(pset,
								volume,
								planesRef,
								preferredCuttingDirection,
								w,
								concavity * params.m_alpha,
								concavity * params.m_beta,
								1, // convexhullDownsampling = 1
								bestPlane,
								minConcavity,
								params);
						}

						{
							if (maxConcavity < minConcavity) 
							{
								maxConcavity = minConcavity;
							}
							PrimitiveSet* bestLeft = pset->Create();
							PrimitiveSet* bestRight = pset->Create();
							temp.PushBack(bestLeft);
							temp.PushBack(bestRight);
							pset->Clip(bestPlane, bestRight, bestLeft);
							delete pset;
						}
					}
					else 
					{
						parts.PushBack(pset);
					}
				}

				inputParts = temp;
				temp.Resize(0);
			}

			const size_t nInputParts = inputParts.Size();
			for (size_t p = 0; p < nInputParts; ++p) 
			{
				parts.PushBack(inputParts[p]);
			}

			size_t nConvexHulls = parts.Size();

			m_convexHulls.Resize(0);
			for (size_t p = 0; p < nConvexHulls; ++p) 
			{
				m_convexHulls.PushBack(new Mesh);
				parts[p]->ComputeConvexHull(*m_convexHulls[p]);
				size_t nv = m_convexHulls[p]->GetNPoints();
				double x, y, z;
				for (size_t i = 0; i < nv; ++i) 
				{
					Vec3& pt = m_convexHulls[p]->GetPoint(i);
					x = pt[0];
					y = pt[1];
					z = pt[2];
					pt[0] = m_rot[0][0] * x + m_rot[0][1] * y + m_rot[0][2] * z + m_barycenter[0];
					pt[1] = m_rot[1][0] * x + m_rot[1][1] * y + m_rot[1][2] * z + m_barycenter[1];
					pt[2] = m_rot[2][0] * x + m_rot[2][1] * y + m_rot[2][2] * z + m_barycenter[2];
				}
			}

			const size_t nParts = parts.Size();
			for (size_t p = 0; p < nParts; ++p) {
				delete parts[p];
				parts[p] = 0;
			}
			parts.Resize(0);
		}
		void AddPoints(const Mesh* const mesh, SArray<Vec3>& pts)
		{
			const size_t n = mesh->GetNPoints();
			for (size_t i = 0; i < n; ++i) 
			{
				pts.PushBack(mesh->GetPoint(i));
			}
		}
		void ComputeConvexHull(const Mesh* const ch1, const Mesh* const ch2, SArray<Vec3>& pts, Mesh* const combinedCH)
		{
			pts.Resize(0);
			AddPoints(ch1, pts);
			AddPoints(ch2, pts);
	
			ConvexHull ch((double*)pts.Data(), sizeof(Vec3), (int32_t)pts.Size(), 1.0e-5f);

			combinedCH->ResizePoints(0);
			combinedCH->ResizeTriangles(0);

			const ndArray<ndBigVector>& convexPoints = ch.GetVertexPool();
			for (ndInt32 v = 0; v < ndInt32(convexPoints.GetCount()); v++)
			{
				const Vec3 hullPoint(convexPoints[v].m_x, convexPoints[v].m_y, convexPoints[v].m_z);
				combinedCH->AddPoint(hullPoint);
			}
		
			for (ConvexHull::ndNode* node = ch.GetFirst(); node; node = node->GetNext())
			{
				const ndConvexHull3dFace* const face = &node->GetInfo();
				combinedCH->AddTriangle(Triangle(face->m_index[0], face->m_index[1], face->m_index[2]));
			}
		}	

		void VHACD::MergeConvexHulls(const Parameters& params)
		{
			if (m_convexHulls.Size() <= 1) 
			{
				return;
			}

			struct ConvexProxy
			{
				Vec3 m_bmin;
				Vec3 m_bmax;
				Mesh* m_hull;
				ndInt32 m_id;
			};

			struct ConvexKey
			{
				ConvexKey()
				{
				}

				ConvexKey(ndInt32 i0, ndInt32 i1)
					:m_p0(ndMin(i0, i1))
					,m_p1(ndMax(i0, i1))
				{
				}

				bool operator< (const ConvexKey& key) const
				{
					ndInt32  key0 = (m_p1 << 16) + m_p0;
					ndInt32  key1 = (key.m_p1 << 16) + key.m_p0;
					return key0 < key1;
				}

				bool operator> (const ConvexKey& key) const
				{
					ndInt32  key0 = (m_p1 << 16) + m_p0;
					ndInt32  key1 = (key.m_p1 << 16) + key.m_p0;
					return key0 > key1;
				}

				ndInt32 m_p0;
				ndInt32 m_p1;
			};

			struct ConvexPair: public ConvexKey
			{
				ConvexPair()
				{
				}
		
				ConvexPair(int i0, int i1)
					:ConvexKey(i0, i1)
				{
				}
		
				float m_cost;
			};
		
			ndTree<ndInt32, ConvexKey, ndContainersFreeListAlloc<ndInt32>> hullGraph;
			ndFixSizeArray<ConvexPair, 1024 * 2> convexPairArray;
			ndFixSizeArray<ConvexProxy, 1024 * 2> convexProxyArray;

			ndUpHeap<ConvexKey, float> priority(int(8 * m_convexHulls.Size() * m_convexHulls.Size()));

			class MergeConvexJob : public Job
			{
				public:
				MergeConvexJob()
					:Job()
				{
				}

				void Execute(int)
				{
					Mesh combinedCH;
					SArray<Vec3> pts;
					for (int i = 0; i < m_pairsCount; i++)
					{
						ConvexPair& pair = m_pairs[i];
						const float volume0 = float(m_convexHulls[pair.m_p0]->ComputeVolume());
						const float volume1 = float(m_convexHulls[pair.m_p1]->ComputeVolume());
						ComputeConvexHull(m_convexHulls[pair.m_p0], m_convexHulls[pair.m_p1], pts, &combinedCH);
						pair.m_cost = float(ComputeConcavity(volume0 + volume1, combinedCH.ComputeVolume(), m_volumeCH0));
					}
				}

				ConvexPair* m_pairs;
				Mesh** m_convexHulls;
				double m_volumeCH0;
				int m_pairsCount;
			};

			MergeConvexJob jobBatches[VHACD_WORKERS_THREADS * 4 + 1];

			for (ndInt32 i = 0; i < ndInt32(m_convexHulls.Size()); ++i)
			{
				convexProxyArray.PushBack(ConvexProxy());
				convexProxyArray[i].m_hull = new Mesh (*m_convexHulls[i]);
				convexProxyArray[i].m_id = i;
				convexProxyArray[i].m_hull->CalculateBoundingBox(convexProxyArray[i].m_bmin, convexProxyArray[i].m_bmax);
			}

			for (ndInt32 i = 1; i < convexProxyArray.GetCount(); ++i)
			{
				Vec3 bmin1(convexProxyArray[i].m_bmin);
				Vec3 bmax1(convexProxyArray[i].m_bmax);
				for (ndInt32 j = 0; j < i; ++j)
				{
					Vec3 bmin0(convexProxyArray[j].m_bmin);
					Vec3 bmax0(convexProxyArray[j].m_bmax);
					Vec3 box1(bmax1 - bmin0);
					Vec3 box0(bmin1 - bmax0);
					Vec3 size(box0.X() * box1.X(), box0.Y()* box1.Y(), box0.Z()* box1.Z());

					if ((size[0] <= 0.0) && (size[1] <= 0.0) && (size[2] <= 0.0))
					{
						ndInt32 i0 = ndInt32(i);
						ndInt32 j0 = ndInt32(j);
						ConvexKey key(i0, j0);
						hullGraph.Insert(0, key);
						convexPairArray.PushBack(ConvexPair(i0, j0));
					}
				}
			}

			ndInt32 nConvexHulls = ndInt32(m_convexHulls.Size());
			if (nConvexHulls > 1)
			{
				ndInt32 start = 0;
				ndInt32 batchSize = convexPairArray.GetCount() / (VHACD_WORKERS_THREADS * 4);
				for (ndInt32 j = 0; j < VHACD_WORKERS_THREADS * 4; ++j)
				{
					ndInt32 count = (j + 1) * batchSize - start;
					if (count > 0)
					{
						jobBatches[j].m_pairs = &convexPairArray[start];
						jobBatches[j].m_pairsCount = int(count);
						jobBatches[j].m_volumeCH0 = m_volumeCH0;
						jobBatches[j].m_convexHulls = &m_convexHulls[0];
						m_parallelQueue.PushTask(&jobBatches[j]);
					}
					start += batchSize;
				}
				ndInt32 count = convexPairArray.GetCount() - start;
				if (count > 0)
				{
					jobBatches[VHACD_WORKERS_THREADS * 4].m_pairs = &convexPairArray[start];
					jobBatches[VHACD_WORKERS_THREADS * 4].m_pairsCount = int(count);
					jobBatches[VHACD_WORKERS_THREADS * 4].m_volumeCH0 = m_volumeCH0;
					jobBatches[VHACD_WORKERS_THREADS * 4].m_convexHulls = &m_convexHulls[0];
					m_parallelQueue.PushTask(&jobBatches[VHACD_WORKERS_THREADS * 4]);
				}
				m_parallelQueue.Sync();

				for (ndInt32 i = 0; i < convexPairArray.GetCount(); i++)
				{
					ConvexPair& pair = convexPairArray[i];
					if (pair.m_cost < (2.0f * params.m_minMergeToleranace))
					{
						priority.Push(pair, pair.m_cost);
					}
				}

				Mesh combinedCH;
				SArray<Vec3> pts;
				while (((nConvexHulls > params.m_maxConvexHulls) || (priority.Value() <= params.m_minMergeToleranace)) && priority.GetCount())
				{
					ConvexKey key(priority[0]);
					ndTree<ndInt32, ConvexKey, ndContainersFreeListAlloc<ndInt32>>::ndNode* const it = hullGraph.Find(key);
					if (it)
					{
						hullGraph.Remove(it);
						for (ndInt32 i = 0; i < convexProxyArray.GetCount(); i++)
						{
							if (convexProxyArray[i].m_hull)
							{
								ConvexKey key0(key.m_p0, int(i));
								hullGraph.Remove(key0);

								ConvexKey key1(key.m_p1, int(i));
								hullGraph.Remove(key1);
							}
						}

						Mesh* const newHull = new Mesh();
						ndInt32 index = convexProxyArray.GetCount();
						ComputeConvexHull(convexProxyArray[key.m_p0].m_hull, convexProxyArray[key.m_p1].m_hull, pts, newHull);
						convexProxyArray.PushBack(ConvexProxy());
						convexProxyArray[index].m_hull = newHull;
						convexProxyArray[index].m_id = int(index);
						convexProxyArray[index].m_hull->CalculateBoundingBox(convexProxyArray[index].m_bmin, convexProxyArray[index].m_bmax);

						delete convexProxyArray[key.m_p0].m_hull;
						delete convexProxyArray[key.m_p1].m_hull;
						convexProxyArray[key.m_p0].m_hull = nullptr;
						convexProxyArray[key.m_p1].m_hull = nullptr;

						const float volume0 = float(newHull->ComputeVolume());

						const Vec3 bmin(convexProxyArray[index].m_bmin);
						const Vec3 bmax(convexProxyArray[index].m_bmax);

						for (ndInt32 i = 0; i < convexProxyArray.GetCount() - 1; i++)
						{
							if (convexProxyArray[i].m_hull)
							{
								Vec3 bmin0(convexProxyArray[i].m_bmin);
								Vec3 bmax0(convexProxyArray[i].m_bmax);
								Vec3 box1(bmax - bmin0);
								Vec3 box0(bmin - bmax0);
								Vec3 size(box0.X() * box1.X(), box0.Y() * box1.Y(), box0.Z() * box1.Z());

								if ((size[0] <= 0.0) && (size[1] <= 0.0) && (size[2] <= 0.0))
								{
									int i0 = int(i);
									ConvexPair pair(i0, int(index));
									const float volume1 = float(convexProxyArray[i].m_hull->ComputeVolume());
									ComputeConvexHull(newHull, convexProxyArray[i].m_hull, pts, &combinedCH);
									float cost = float(ComputeConcavity(volume0 + volume1, combinedCH.ComputeVolume(), m_volumeCH0));
									priority.Push(pair, cost);
								}
							}
						}

						nConvexHulls--;
					}
					priority.Pop();
				}
	
				for (ndInt32 i = ndInt32(m_convexHulls.Size())-1; i >= 0; --i)
				{
					delete m_convexHulls[i];
				}
				m_convexHulls.SetCount(0);

				for (ndInt32 i = 0; i < convexProxyArray.GetCount(); i++)
				{
					if (convexProxyArray[i].m_hull)
					{
						m_convexHulls.PushBack(convexProxyArray[i].m_hull);
					}
				}
			}
		}

		void VHACD::VoxelizeMesh(
			const ndReal* const points, 
			const uint32_t stridePoints,
			const uint32_t nPoints,
			const int32_t* const triangles,
			const uint32_t strideTriangles,
			const uint32_t nTriangles,
			const Parameters& params)
		{
			delete m_volume;
			m_volume = nullptr;
			int32_t iteration = 0;
			const int32_t maxIteration = 5;
			while (iteration++ < maxIteration)
			{
				m_volume = new Volume;
				m_volume->Voxelize(points, stridePoints, nPoints,
					triangles, strideTriangles, nTriangles,
					m_dim, m_barycenter, m_rot);

				ndInt32 n = ndInt32(m_volume->GetNPrimitivesOnSurf() + m_volume->GetNPrimitivesInsideSurf());

				double a = pow(double(params.m_resolution) / double(n), 0.33);
				size_t dim_next = size_t(double(m_dim) * a + 0.5);
				if ((n < params.m_resolution) && iteration < maxIteration && (ndInt32(m_volume->GetNPrimitivesOnSurf()) < params.m_resolution / 8) && (m_dim != dim_next))
				{
					delete m_volume;
					m_volume = 0;
					m_dim = dim_next;
				}
				else
				{
					break;
				}
			}
		}

		bool VHACD::ComputeACD(const ndReal* const points,
			const uint32_t nPoints,
			const uint32_t* const triangles,
			const uint32_t nTriangles,
			const Parameters& params)
		{
			Init();
			VoxelizeMesh(points, 3, nPoints, (int32_t*)triangles, 3, nTriangles, params);
			ComputePrimitiveSet(params);
			ComputeACD(params);
			MergeConvexHulls(params);
			return true;
		}

	} // end of VHACD namespace
}
