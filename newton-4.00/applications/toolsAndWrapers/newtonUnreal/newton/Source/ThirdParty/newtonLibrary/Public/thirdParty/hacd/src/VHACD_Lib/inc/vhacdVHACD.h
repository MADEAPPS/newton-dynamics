/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
All rights reserved.


Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once
#ifndef ND_VHACD_VHACD_H
#define ND_VHACD_VHACD_H
#include "vhacdDefines.h"
#include "vhacdVolume.h"
#include "vhacdRaycastMesh.h"

namespace nd_
{
	#define CH_APP_MIN_NUM_PRIMITIVES 64000
	namespace VHACD 
	{
		class VHACD : public IVHACD 
		{
			public:
			//! Constructor.
			VHACD()
			{
				Init();
			}
			//! Destructor.
			~VHACD(void)
			{
			}
			uint32_t GetNConvexHulls() const
			{
				return (uint32_t)m_convexHulls.Size();
			}
			void Cancel()
			{
				SetCancel(true);
			}
			void GetConvexHull(const uint32_t index, ConvexHull& ch) const
			{
				Mesh* mesh = m_convexHulls[index];
				ch.m_nPoints = (uint32_t)mesh->GetNPoints();
				ch.m_nTriangles = (uint32_t)mesh->GetNTriangles();
				ch.m_points = mesh->GetPoints();
				ch.m_triangles = (uint32_t *)mesh->GetTriangles();
			}
			void Clean(void)
			{
				delete m_volume;
				delete m_pset;
				size_t nCH = m_convexHulls.Size();
				for (size_t p = 0; p < nCH; ++p) {
					delete m_convexHulls[p];
				}
				m_convexHulls.Clear();
				Init();
			}
			void Release(void)
			{
				delete this;
			}
			bool Compute(const float* const points,
				const uint32_t nPoints,
				const uint32_t* const triangles,
				const uint32_t nTriangles,
				const Parameters& params);
			bool Compute(const double* const points,
				const uint32_t nPoints,
				const uint32_t* const triangles,
				const uint32_t nTriangles,
				const Parameters& params);

			private:
			void SetCancel(bool cancel)
			{
				m_cancel = cancel;
			}
			bool GetCancel()
			{
				bool cancel = m_cancel;
				return cancel;
			}
			void Update(const double stageProgress,
				const double operationProgress,
				const Parameters& params)
			{
				m_stageProgress = stageProgress;
				m_operationProgress = operationProgress;
				if (params.m_callback) {
					params.m_callback->Update(m_overallProgress,
						m_stageProgress,
						m_operationProgress,
						m_stage.c_str(),
						m_operation.c_str());
				}
			}
			void Init()
			{
				m_raycastMesh = nullptr;
				memset(m_rot, 0, sizeof(double) * 9);
				m_dim = 64;
				m_volume = 0;
				m_volumeCH0 = 0.0;
				m_pset = 0;
				m_overallProgress = 0.0;
				m_stageProgress = 0.0;
				m_operationProgress = 0.0;
				m_stage = "";
				m_operation = "";
				m_barycenter[0] = m_barycenter[1] = m_barycenter[2] = 0.0;
				m_rot[0][0] = m_rot[1][1] = m_rot[2][2] = 1.0;
				SetCancel(false);
			}
			void ComputePrimitiveSet(const Parameters& params);
			void ComputeACD(const Parameters& params);
			void MergeConvexHulls(const Parameters& params);
			void SimplifyConvexHull(Mesh* const ch, const size_t nvertices, const double minVolume);
			void SimplifyConvexHulls(const Parameters& params);
			void ComputeBestClippingPlane(const PrimitiveSet* inputPSet,
				const double volume,
				const SArray<Plane>& planes,
				const Vec3<double>& preferredCuttingDirection,
				const double w,
				const double alpha,
				const double beta,
				const int32_t convexhullDownsampling,
				const double progress0,
				const double progress1,
				Plane& bestPlane,
				double& minConcavity,
				const Parameters& params);
			template <class T>
			void AlignMesh(const T* const points,
				const uint32_t stridePoints,
				const uint32_t nPoints,
				const int32_t* const triangles,
				const uint32_t strideTriangles,
				const uint32_t nTriangles,
				const Parameters& params)
			{
				if (GetCancel() || !params.m_pca) {
					return;
				}
				m_timer.Tic();

				m_stage = "Align mesh";
				m_operation = "Voxelization";

				std::ostringstream msg;
				if (params.m_logger) {
					msg << "+ " << m_stage << std::endl;
					params.m_logger->Log(msg.str().c_str());
				}

				Update(0.0, 0.0, params);
				if (GetCancel()) {
					return;
				}
				m_dim = (size_t)(pow((double)params.m_resolution, 1.0 / 3.0) + 0.5);
				Volume volume;
				volume.Voxelize(points, stridePoints, nPoints,
					triangles, strideTriangles, nTriangles,
					m_dim, m_barycenter, m_rot);
				size_t n = volume.GetNPrimitivesOnSurf() + volume.GetNPrimitivesInsideSurf();
				Update(50.0, 100.0, params);

				if (params.m_logger) {
					msg.str("");
					msg << "\t dim = " << m_dim << "\t-> " << n << " voxels" << std::endl;
					params.m_logger->Log(msg.str().c_str());
				}
				if (GetCancel()) {
					return;
				}
				m_operation = "PCA";
				Update(50.0, 0.0, params);
				volume.AlignToPrincipalAxes(m_rot);
				m_overallProgress = 1.0;
				Update(100.0, 100.0, params);

				m_timer.Toc();
				if (params.m_logger) {
					msg.str("");
					msg << "\t time " << m_timer.GetElapsedTime() / 1000.0 << "s" << std::endl;
					params.m_logger->Log(msg.str().c_str());
				}
			}
			template <class T>
			void VoxelizeMesh(const T* const points,
				const uint32_t stridePoints,
				const uint32_t nPoints,
				const int32_t* const triangles,
				const uint32_t strideTriangles,
				const uint32_t nTriangles,
				const Parameters& params)
			{
				if (GetCancel()) {
					return;
				}

				m_timer.Tic();
				m_stage = "Voxelization";

				std::ostringstream msg;
				if (params.m_logger) {
					msg << "+ " << m_stage << std::endl;
					params.m_logger->Log(msg.str().c_str());
				}

				delete m_volume;
				m_volume = 0;
				int32_t iteration = 0;
				const int32_t maxIteration = 5;
				double progress = 0.0;
				while (iteration++ < maxIteration && !m_cancel) {
					msg.str("");
					msg << "Iteration " << iteration;
					m_operation = msg.str();

					progress = iteration * 100.0 / maxIteration;
					Update(progress, 0.0, params);

					m_volume = new Volume;
					m_volume->Voxelize(points, stridePoints, nPoints,
						triangles, strideTriangles, nTriangles,
						m_dim, m_barycenter, m_rot);

					Update(progress, 100.0, params);

					size_t n = m_volume->GetNPrimitivesOnSurf() + m_volume->GetNPrimitivesInsideSurf();
					if (params.m_logger) {
						msg.str("");
						msg << "\t dim = " << m_dim << "\t-> " << n << " voxels" << std::endl;
						params.m_logger->Log(msg.str().c_str());
					}

					double a = pow(double(params.m_resolution) / double(n), 0.33);
					size_t dim_next = (size_t)(double(m_dim) * a + 0.5);
					if (n < params.m_resolution && iteration < maxIteration && m_volume->GetNPrimitivesOnSurf() < params.m_resolution / 8 && m_dim != dim_next) {
						delete m_volume;
						m_volume = 0;
						m_dim = dim_next;
					}
					else {
						break;
					}
				}
				m_overallProgress = 10.0;
				Update(100.0, 100.0, params);

				m_timer.Toc();
				if (params.m_logger) {
					msg.str("");
					msg << "\t time " << m_timer.GetElapsedTime() / 1000.0 << "s" << std::endl;
					params.m_logger->Log(msg.str().c_str());
				}
			}
			template <class T>
			bool ComputeACD(const T* const points,
				const uint32_t nPoints,
				const uint32_t* const triangles,
				const uint32_t nTriangles,
				const Parameters& params)
			{
				Init();
				AlignMesh(points, 3, nPoints, (int32_t *)triangles, 3, nTriangles, params);
				VoxelizeMesh(points, 3, nPoints, (int32_t *)triangles, 3, nTriangles, params);
				ComputePrimitiveSet(params);
				ComputeACD(params);
				MergeConvexHulls(params);
				if (params.m_projectHullVertices)
				{
					m_raycastMesh = RaycastMesh::createRaycastMesh(nPoints, points, nTriangles, (const uint32_t *)triangles);
					SimplifyConvexHulls(params);
					m_raycastMesh->release();
					m_raycastMesh = nullptr;
				}
				if (GetCancel()) {
					Clean();
					return false;
				}
				return true;
			}

		private:
			RaycastMesh* m_raycastMesh;
			SArray<Mesh*> m_convexHulls;
			std::string m_stage;
			std::string m_operation;
			double m_overallProgress;
			double m_stageProgress;
			double m_operationProgress;
			double m_rot[3][3];
			double m_volumeCH0;
			Vec3<double> m_barycenter;
			Timer m_timer;
			size_t m_dim;
			Volume* m_volume;
			PrimitiveSet* m_pset;
			bool m_cancel;
			Queue m_parallelQueue;
		};
	}
}
#endif // VHACD_VHACD_H
