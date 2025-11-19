/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
All rights reserved.


Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef ND_VHACD_VHACD_H
#define ND_VHACD_VHACD_H

#include "vhacdVolume.h"
#include "vhacdConvexHullUtils.h"

namespace nd
{
	namespace VHACD 
	{
		class ndWorkingBuffers
		{
			public:
			Mesh m_leftMesh;
			Mesh m_rightMesh;
			ConvexHull3dPointSet m_leftBuffer;
			ConvexHull3dPointSet m_rightBuffer;
		};

		class VHACD : public IVHACD 
		{
			public:
			VHACD();
			~VHACD(void);

			uint32_t GetNConvexHulls() const;
			void GetConvexHull(const uint32_t index, ConvexHull& ch) const;
			void Compute(
				const float* const points, const uint32_t nPoints,
				const uint32_t* const triangles, const uint32_t nTriangles,
				const Parameters& params);

			private:
			void ComputePrimitiveSet(const Parameters& params);
			void ComputeACD(const Parameters& params);
			void MergeConvexHulls(const Parameters& params);
			void ComputeBestClippingPlane(const PrimitiveSet* inputPSet,
				const double volume,
				const SArray<Plane>& planes,
				const Vec3& preferredCuttingDirection,
				const double w,
				const double alpha,
				const double beta,
				const int32_t convexhullDownsampling,
				Plane& bestPlane,
				double& minConcavity,
				ndWorkingBuffers* const workBuffers,
				//ConvexHull3dPointSet& posBuffer,
				//ConvexHull3dPointSet& negBuffer,
				const Parameters& params);

			void VoxelizeMesh(
				const ndReal* const points, const uint32_t stridePoints,
				const uint32_t nPoints,	const int32_t* const triangles,
				const uint32_t strideTriangles,	const uint32_t nTriangles,
				const Parameters& params);
			
			void ComputeACD(
				const ndReal* const points,	const uint32_t nPoints,		
				const uint32_t* const triangles, const uint32_t nTriangles,
				const Parameters& params);

		private:
			SArray<Mesh*> m_convexHulls;
			double m_rot[3][3];
			double m_volumeCH0;
			Vec3 m_barycenter;
			size_t m_dim;
			Volume* m_volume;
			PrimitiveSet* m_pset;
			Queue m_parallelQueue;
		};
	}
}
#endif
