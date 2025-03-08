/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include "vhacdConvexHull.h"
#include "VHACD.h"
#include "vhacdICHull.h"
#include "vhacdMesh.h"
#include "vhacdSArray.h"
#include "vhacdTimer.h"
#include "vhacdVHACD.h"
#include "vhacdVector.h"
#include "vhacdVolume.h"
#include "FloatMath.h"

namespace nd_
{
	#define MAX(a, b) (((a) > (b)) ? (a) : (b))
	#define MIN(a, b) (((a) < (b)) ? (a) : (b))
	#define ABS(a) (((a) < 0) ? -(a) : (a))
	#define ZSGN(a) (((a) < 0) ? -1 : (a) > 0 ? 1 : 0)
	#define MAX_DOUBLE (1.79769e+308)

	#define USE_CPP_11_THREADS

	inline int32_t FindMinimumElement(const float* const d, float* const m, const int32_t begin, const int32_t end)
	{
		int32_t idx = -1;
		float min = (std::numeric_limits<float>::max)();
		for (size_t i = size_t(begin); i < size_t(end); ++i) {
			if (d[i] < min) {
				idx = int32_t(i);
				min = d[i];
			}
		}

		*m = min;
		return idx;
	}

	namespace VHACD {
	IVHACD* CreateVHACD(void)
	{
		return new VHACD();
	}

	void VHACD::ComputePrimitiveSet(const Parameters& params)
	{
		if (GetCancel()) {
			return;
		}
		m_timer.Tic();

		m_stage = "Compute primitive set";
		m_operation = "Convert volume to pset";

		std::ostringstream msg;
		if (params.m_logger) {
			msg << "+ " << m_stage << std::endl;
			params.m_logger->Log(msg.str().c_str());
		}

		Update(0.0, 0.0, params);
		if (params.m_mode == 0) {
			VoxelSet* vset = new VoxelSet;
			m_volume->Convert(*vset);
			m_pset = vset;
		}
		else {
			TetrahedronSet* tset = new TetrahedronSet;
			m_volume->Convert(*tset);
			m_pset = tset;
		}

		delete m_volume;
		m_volume = 0;

		if (params.m_logger) {
			msg.str("");
			msg << "\t # primitives               " << m_pset->GetNPrimitives() << std::endl;
			msg << "\t # inside surface           " << m_pset->GetNPrimitivesInsideSurf() << std::endl;
			msg << "\t # on surface               " << m_pset->GetNPrimitivesOnSurf() << std::endl;
			params.m_logger->Log(msg.str().c_str());
		}

		m_overallProgress = 15.0;
		Update(100.0, 100.0, params);
		m_timer.Toc();
		if (params.m_logger) {
			msg.str("");
			msg << "\t time " << m_timer.GetElapsedTime() / 1000.0 << "s" << std::endl;
			params.m_logger->Log(msg.str().c_str());
		}
	}
	bool VHACD::Compute(const double* const points, const uint32_t nPoints,
		const uint32_t* const triangles,const uint32_t nTriangles, const Parameters& params)
	{
		return ComputeACD(points, nPoints, triangles, nTriangles, params);
	}
	bool VHACD::Compute(const float* const points,const uint32_t nPoints,
		const uint32_t* const triangles,const uint32_t nTriangles, const Parameters& params)
	{
		return ComputeACD(points, nPoints, triangles, nTriangles, params);
	}
	double ComputePreferredCuttingDirection(const PrimitiveSet* const tset, Vec3<double>& dir)
	{
		double ex = tset->GetEigenValue(AXIS_X);
		double ey = tset->GetEigenValue(AXIS_Y);
		double ez = tset->GetEigenValue(AXIS_Z);
		double vx = (ey - ez) * (ey - ez);
		double vy = (ex - ez) * (ex - ez);
		double vz = (ex - ey) * (ex - ey);
		if (vx < vy && vx < vz) {
			double e = ey * ey + ez * ez;
			dir[0] = 1.0;
			dir[1] = 0.0;
			dir[2] = 0.0;
			return (e == 0.0) ? 0.0 : 1.0 - vx / e;
		}
		else if (vy < vx && vy < vz) {
			double e = ex * ex + ez * ez;
			dir[0] = 0.0;
			dir[1] = 1.0;
			dir[2] = 0.0;
			return (e == 0.0) ? 0.0 : 1.0 - vy / e;
		}
		else {
			double e = ex * ex + ey * ey;
			dir[0] = 0.0;
			dir[1] = 0.0;
			dir[2] = 1.0;
			return (e == 0.0) ? 0.0 : 1.0 - vz / e;
		}
	}
	void ComputeAxesAlignedClippingPlanes(const VoxelSet& vset, const short downsampling, SArray<Plane>& planes)
	{
		const Vec3<short> minV = vset.GetMinBBVoxels();
		const Vec3<short> maxV = vset.GetMaxBBVoxels();
		Vec3<double> pt;
		Plane plane;
		const short i0 = minV[0];
		const short i1 = maxV[0];
		plane.m_a = 1.0;
		plane.m_b = 0.0;
		plane.m_c = 0.0;
		plane.m_axis = AXIS_X;
		for (short i = i0; i <= i1; i += downsampling) {
			pt = vset.GetPoint(Vec3<double>(i + 0.5, 0.0, 0.0));
			plane.m_d = -pt[0];
			plane.m_index = i;
			planes.PushBack(plane);
		}
		const short j0 = minV[1];
		const short j1 = maxV[1];
		plane.m_a = 0.0;
		plane.m_b = 1.0;
		plane.m_c = 0.0;
		plane.m_axis = AXIS_Y;
		for (short j = j0; j <= j1; j += downsampling) {
			pt = vset.GetPoint(Vec3<double>(0.0, j + 0.5, 0.0));
			plane.m_d = -pt[1];
			plane.m_index = j;
			planes.PushBack(plane);
		}
		const short k0 = minV[2];
		const short k1 = maxV[2];
		plane.m_a = 0.0;
		plane.m_b = 0.0;
		plane.m_c = 1.0;
		plane.m_axis = AXIS_Z;
		for (short k = k0; k <= k1; k += downsampling) {
			pt = vset.GetPoint(Vec3<double>(0.0, 0.0, k + 0.5));
			plane.m_d = -pt[2];
			plane.m_index = k;
			planes.PushBack(plane);
		}
	}
	void ComputeAxesAlignedClippingPlanes(const TetrahedronSet& tset, const short downsampling, SArray<Plane>& planes)
	{
		const Vec3<double> minV = tset.GetMinBB();
		const Vec3<double> maxV = tset.GetMaxBB();
		const double scale = tset.GetSacle();
		const short i0 = 0;
		const short j0 = 0;
		const short k0 = 0;
		const short i1 = static_cast<short>((maxV[0] - minV[0]) / scale + 0.5);
		const short j1 = static_cast<short>((maxV[1] - minV[1]) / scale + 0.5);
		const short k1 = static_cast<short>((maxV[2] - minV[2]) / scale + 0.5);

		Plane plane;
		plane.m_a = 1.0;
		plane.m_b = 0.0;
		plane.m_c = 0.0;
		plane.m_axis = AXIS_X;
		for (short i = i0; i <= i1; i += downsampling) {
			double x = minV[0] + scale * i;
			plane.m_d = -x;
			plane.m_index = i;
			planes.PushBack(plane);
		}
		plane.m_a = 0.0;
		plane.m_b = 1.0;
		plane.m_c = 0.0;
		plane.m_axis = AXIS_Y;
		for (short j = j0; j <= j1; j += downsampling) {
			double y = minV[1] + scale * j;
			plane.m_d = -y;
			plane.m_index = j;
			planes.PushBack(plane);
		}
		plane.m_a = 0.0;
		plane.m_b = 0.0;
		plane.m_c = 1.0;
		plane.m_axis = AXIS_Z;
		for (short k = k0; k <= k1; k += downsampling) {
			double z = minV[2] + scale * k;
			plane.m_d = -z;
			plane.m_index = k;
			planes.PushBack(plane);
		}
	}
	void RefineAxesAlignedClippingPlanes(const VoxelSet& vset, const Plane& bestPlane, const short downsampling,
		SArray<Plane>& planes)
	{
		const Vec3<short> minV = vset.GetMinBBVoxels();
		const Vec3<short> maxV = vset.GetMaxBBVoxels();
		Vec3<double> pt;
		Plane plane;

		if (bestPlane.m_axis == AXIS_X) {
			const short i0 = MAX(minV[0], bestPlane.m_index - downsampling);
			const short i1 = MIN(maxV[0], bestPlane.m_index + downsampling);
			plane.m_a = 1.0;
			plane.m_b = 0.0;
			plane.m_c = 0.0;
			plane.m_axis = AXIS_X;
			for (short i = i0; i <= i1; ++i) {
				pt = vset.GetPoint(Vec3<double>(i + 0.5, 0.0, 0.0));
				plane.m_d = -pt[0];
				plane.m_index = i;
				planes.PushBack(plane);
			}
		}
		else if (bestPlane.m_axis == AXIS_Y) {
			const short j0 = MAX(minV[1], bestPlane.m_index - downsampling);
			const short j1 = MIN(maxV[1], bestPlane.m_index + downsampling);
			plane.m_a = 0.0;
			plane.m_b = 1.0;
			plane.m_c = 0.0;
			plane.m_axis = AXIS_Y;
			for (short j = j0; j <= j1; ++j) {
				pt = vset.GetPoint(Vec3<double>(0.0, j + 0.5, 0.0));
				plane.m_d = -pt[1];
				plane.m_index = j;
				planes.PushBack(plane);
			}
		}
		else {
			const short k0 = MAX(minV[2], bestPlane.m_index - downsampling);
			const short k1 = MIN(maxV[2], bestPlane.m_index + downsampling);
			plane.m_a = 0.0;
			plane.m_b = 0.0;
			plane.m_c = 1.0;
			plane.m_axis = AXIS_Z;
			for (short k = k0; k <= k1; ++k) {
				pt = vset.GetPoint(Vec3<double>(0.0, 0.0, k + 0.5));
				plane.m_d = -pt[2];
				plane.m_index = k;
				planes.PushBack(plane);
			}
		}
	}
	void RefineAxesAlignedClippingPlanes(const TetrahedronSet& tset, const Plane& bestPlane, const short downsampling,
		SArray<Plane>& planes)
	{
		const Vec3<double> minV = tset.GetMinBB();
		const Vec3<double> maxV = tset.GetMaxBB();
		const double scale = tset.GetSacle();
		Plane plane;

		if (bestPlane.m_axis == AXIS_X) {
			const short i0 = MAX(0, bestPlane.m_index - downsampling);
			const short i1 = static_cast<short>(MIN((maxV[0] - minV[0]) / scale + 0.5, bestPlane.m_index + downsampling));
			plane.m_a = 1.0;
			plane.m_b = 0.0;
			plane.m_c = 0.0;
			plane.m_axis = AXIS_X;
			for (short i = i0; i <= i1; ++i) {
				double x = minV[0] + scale * i;
				plane.m_d = -x;
				plane.m_index = i;
				planes.PushBack(plane);
			}
		}
		else if (bestPlane.m_axis == AXIS_Y) {
			const short j0 = MAX(0, bestPlane.m_index - downsampling);
			const short j1 = static_cast<short>(MIN((maxV[1] - minV[1]) / scale + 0.5, bestPlane.m_index + downsampling));
			plane.m_a = 0.0;
			plane.m_b = 1.0;
			plane.m_c = 0.0;
			plane.m_axis = AXIS_Y;
			for (short j = j0; j <= j1; ++j) {
				double y = minV[1] + scale * j;
				plane.m_d = -y;
				plane.m_index = j;
				planes.PushBack(plane);
			}
		}
		else {
			const short k0 = MAX(0, bestPlane.m_index - downsampling);
			const short k1 = static_cast<short>(MIN((maxV[2] - minV[2]) / scale + 0.5, bestPlane.m_index + downsampling));
			plane.m_a = 0.0;
			plane.m_b = 0.0;
			plane.m_c = 1.0;
			plane.m_axis = AXIS_Z;
			for (short k = k0; k <= k1; ++k) {
				double z = minV[2] + scale * k;
				plane.m_d = -z;
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
		const Vec3<double>& preferredCuttingDirection, const double w, const double alpha, const double beta,
		const int32_t convexhullDownsampling, const double, const double, Plane& bestPlane,
		double& minConcavity, const Parameters& params)
	{
		if (GetCancel()) 
		{
			return;
		}
		char msg[256];
		int32_t iBest = -1;
		int32_t nPlanes = static_cast<int32_t>(planes.Size());
		double minTotal = MAX_DOUBLE;
		double minBalance = MAX_DOUBLE;
		double minSymmetry = MAX_DOUBLE;
		minConcavity = MAX_DOUBLE;

		PrimitiveSet* onSurfacePSet = inputPSet->Create();
		inputPSet->SelectOnSurface(onSurfacePSet);

		PrimitiveSet** psets = 0;
		if (!params.m_convexhullApproximation) 
		{
			psets = new PrimitiveSet*[2];
			for (int32_t i = 0; i < 2; ++i) 
			{
				psets[i] = inputPSet->Create();
			}
		}

	#ifdef DEBUG_TEMP
		Timer timerComputeCost;
		timerComputeCost.Tic();
	#endif // DEBUG_TEMP


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
		int32_t m_convexhullDownsampling;
		Mesh chs[VHACD_WORKERS_THREADS][2];
		SArray<Vec3<double>> chPts[VHACD_WORKERS_THREADS][2];

		Vec3<double> m_preferredCuttingDirection;
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
			
			if (m_commonData->m_params.m_convexhullApproximation)
			{
				SArray<Vec3<double> >& leftCHPts = m_commonData->chPts[threadId][0];
				SArray<Vec3<double> >& rightCHPts = m_commonData->chPts[threadId][1];;
				rightCHPts.Resize(0);
				leftCHPts.Resize(0);
				m_commonData->m_onSurfacePSet->Intersect(m_plane, &rightCHPts, &leftCHPts, size_t (m_commonData->m_convexhullDownsampling * 32));
				m_commonData->m_inputPSet->GetConvexHull().Clip(m_plane, rightCHPts, leftCHPts);
				rightCH.ComputeConvexHull((double*)rightCHPts.Data(), rightCHPts.Size());
				leftCH.ComputeConvexHull((double*)leftCHPts.Data(), leftCHPts.Size());
			}
			else 
			{
				_ASSERT(0);
				PrimitiveSet* const right = m_commonData->m_psets[threadId * 2 + 0];
				PrimitiveSet* const left = m_commonData->m_psets[threadId * 2 + 1];
				m_commonData->m_onSurfacePSet->Clip(m_plane, right, left);
				right->ComputeConvexHull(rightCH, size_t(m_commonData->m_convexhullDownsampling));
				left->ComputeConvexHull(leftCH, size_t(m_commonData->m_convexhullDownsampling));
			}

			double volumeLeftCH = leftCH.ComputeVolume();
			double volumeRightCH = rightCH.ComputeVolume();
			
			// compute clipped volumes
			double volumeLeft = 0.0;
			double volumeRight = 0.0;
			
			m_commonData->m_inputPSet->ComputeClippedVolumes(m_plane, volumeRight, volumeLeft);
			
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

		std::vector<BestClippingPlaneJob> jobs;
		jobs.resize(size_t(nPlanes));

		CommonData data(this, params);
		data.m_w = w;
		data.m_beta = beta;
		data.m_alpha = alpha;
		data.m_psets = psets;
		data.m_inputPSet = inputPSet;
		data.m_onSurfacePSet = onSurfacePSet;
		data.m_preferredCuttingDirection = preferredCuttingDirection;
		data.m_convexhullDownsampling = convexhullDownsampling;
		for (size_t i = 0; i < size_t(nPlanes); ++i)
		{
			jobs[i].m_plane = planes[i];
			jobs[i].m_commonData = &data;
			m_parallelQueue.PushTask(&jobs[i]);
		}
		m_parallelQueue.Sync();

		iBest = 0;
		minConcavity = jobs[0].m_concavity;
		minBalance = jobs[0].m_balance;
		minSymmetry = jobs[0].m_symmetry;
		bestPlane = jobs[0].m_plane;
		minTotal = jobs[0].m_concavity + jobs[0].m_balance + jobs[0].m_symmetry;
		for (size_t i = 1; i < size_t(nPlanes); ++i)
		{
			double total = jobs[i].m_concavity + jobs[i].m_balance + jobs[i].m_symmetry;
			if ((total < minTotal && int32_t(i) < iBest) || total < minTotal)
			{
				minConcavity = jobs[i].m_concavity;
				minBalance = jobs[i].m_balance;
				minSymmetry = jobs[i].m_symmetry;
				bestPlane = jobs[i].m_plane;
				minTotal = total;
				iBest = int32_t(i);
			}
		}

	#ifdef DEBUG_TEMP
		timerComputeCost.Toc();
		printf_s("Cost[%i] = %f\n", nPlanes, timerComputeCost.GetElapsedTime());
	#endif // DEBUG_TEMP

		if (psets) 
		{
			for (int32_t i = 0; i < 2; ++i) 
			{
				delete psets[i];
			}
			delete[] psets;
		}
		delete onSurfacePSet;
		if (params.m_logger) 
		{
			snprintf(msg, sizeof (msg), "\n\t\t\t Best  %04i T=%2.6f C=%2.6f B=%2.6f S=%2.6f (%1.1f, %1.1f, %1.1f, %3.3f)\n\n", int (iBest), minTotal, minConcavity, minBalance, minSymmetry, bestPlane.m_a, bestPlane.m_b, bestPlane.m_c, bestPlane.m_d);
			params.m_logger->Log(msg);
		}
	}

	void VHACD::ComputeACD(const Parameters& params)
	{
		if (GetCancel()) {
			return;
		}
		m_timer.Tic();

		m_stage = "Approximate Convex Decomposition";
		m_stageProgress = 0.0;
		std::ostringstream msg;
		if (params.m_logger) {
			msg << "+ " << m_stage << std::endl;
			params.m_logger->Log(msg.str().c_str());
		}

		SArray<PrimitiveSet*> parts;
		SArray<PrimitiveSet*> inputParts;
		SArray<PrimitiveSet*> temp;
		inputParts.PushBack(m_pset);
		m_pset = 0;
		SArray<Plane> planes;
		SArray<Plane> planesRef;
		uint32_t sub = 0;
		bool firstIteration = true;
		m_volumeCH0 = 1.0;

		// Compute the decomposition depth based on the number of convex hulls being requested..
		uint32_t hullCount = 2;
		uint32_t depth = 1;
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


		while (sub++ < depth && inputParts.Size() > 0 && !m_cancel) {
			msg.str("");
			msg << "Subdivision level " << sub;
			m_operation = msg.str();

			if (params.m_logger) {
				msg.str("");
				msg << "\t Subdivision level " << sub << std::endl;
				params.m_logger->Log(msg.str().c_str());
			}

			double maxConcavity = 0.0;
			const size_t nInputParts = inputParts.Size();
			Update(m_stageProgress, 0.0, params);
			for (size_t p = 0; p < nInputParts && !m_cancel; ++p) {
				const double progress0 = double(p) * 100.0 / (double)nInputParts;
				const double progress1 = (double(p) + 0.75) * 100.0 / (double)nInputParts;
				const double progress2 = (double(p) + 1.00) * 100.0 / (double)nInputParts;

				Update(m_stageProgress, progress0, params);

				PrimitiveSet* pset = inputParts[p];
				inputParts[p] = 0;
				double volume = pset->ComputeVolume();
				pset->ComputeBB();
				pset->ComputePrincipalAxes();
				if (params.m_pca) {
					pset->AlignToPrincipalAxes();
				}

				pset->ComputeConvexHull(pset->GetConvexHull());
				double volumeCH = fabs(pset->GetConvexHull().ComputeVolume());
				if (firstIteration) {
					m_volumeCH0 = volumeCH;
				}

				double concavity = float(ComputeConcavity(volume, volumeCH, m_volumeCH0));
				double error = 1.01 * pset->ComputeMaxVolumeError() / m_volumeCH0;
				// make the value smaller, later put it the parameters.
				error *= params.m_concavityToVolumeWeigh;

				if (firstIteration) {
					firstIteration = false;
				}

				if (params.m_logger) {
					msg.str("");
					msg << "\t -> Part[" << p
						<< "] C  = " << concavity
						<< ", E  = " << error
						<< ", VS = " << pset->GetNPrimitivesOnSurf()
						<< ", VI = " << pset->GetNPrimitivesInsideSurf()
						<< std::endl;
					params.m_logger->Log(msg.str().c_str());
				}
			
				if (concavity > params.m_concavity && concavity > error) {
					Vec3<double> preferredCuttingDirection;
					double w = ComputePreferredCuttingDirection(pset, preferredCuttingDirection);
					planes.Resize(0);
					if (params.m_mode == 0) {
						VoxelSet* vset = (VoxelSet*)pset;
						ComputeAxesAlignedClippingPlanes(*vset, short(params.m_planeDownsampling), planes);
					}
					else {
						TetrahedronSet* tset = (TetrahedronSet*)pset;
						ComputeAxesAlignedClippingPlanes(*tset, short(params.m_planeDownsampling), planes);
					}

					if (params.m_logger) {
						msg.str("");
						msg << "\t\t [Regular sampling] Number of clipping planes " << planes.Size() << std::endl;
						params.m_logger->Log(msg.str().c_str());
					}

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
						progress0,
						progress1,
						bestPlane,
						minConcavity,
						params);
					if (!m_cancel && (params.m_planeDownsampling > 1 || params.m_convexhullDownsampling > 1)) {
						planesRef.Resize(0);

						if (params.m_mode == 0) {
							VoxelSet* vset = (VoxelSet*)pset;
							RefineAxesAlignedClippingPlanes(*vset, bestPlane, short(params.m_planeDownsampling), planesRef);
						}
						else {
							TetrahedronSet* tset = (TetrahedronSet*)pset;
							RefineAxesAlignedClippingPlanes(*tset, bestPlane, short(params.m_planeDownsampling), planesRef);
						}

						if (params.m_logger) {
							msg.str("");
							msg << "\t\t [Refining] Number of clipping planes " << planesRef.Size() << std::endl;
							params.m_logger->Log(msg.str().c_str());
						}
						ComputeBestClippingPlane(pset,
							volume,
							planesRef,
							preferredCuttingDirection,
							w,
							concavity * params.m_alpha,
							concavity * params.m_beta,
							1, // convexhullDownsampling = 1
							progress1,
							progress2,
							bestPlane,
							minConcavity,
							params);
					}
					if (GetCancel()) {
						delete pset; // clean up
						break;
					}
					else {
						if (maxConcavity < minConcavity) {
							maxConcavity = minConcavity;
						}
						PrimitiveSet* bestLeft = pset->Create();
						PrimitiveSet* bestRight = pset->Create();
						temp.PushBack(bestLeft);
						temp.PushBack(bestRight);
						pset->Clip(bestPlane, bestRight, bestLeft);
						if (params.m_pca) {
							bestRight->RevertAlignToPrincipalAxes();
							bestLeft->RevertAlignToPrincipalAxes();
						}
						delete pset;
					}
				}
				else {
					if (params.m_pca) {
						pset->RevertAlignToPrincipalAxes();
					}
					parts.PushBack(pset);
				}
			}

			Update(95.0 * (1.0 - maxConcavity) / (1.0 - params.m_concavity), 100.0, params);
			if (GetCancel()) {
				const size_t nTempParts = temp.Size();
				for (size_t p = 0; p < nTempParts; ++p) {
					delete temp[p];
				}
				temp.Resize(0);
			}
			else {
				inputParts = temp;
				temp.Resize(0);
			}
		}
		const size_t nInputParts = inputParts.Size();
		for (size_t p = 0; p < nInputParts; ++p) {
			parts.PushBack(inputParts[p]);
		}

		if (GetCancel()) {
			const size_t nParts = parts.Size();
			for (size_t p = 0; p < nParts; ++p) {
				delete parts[p];
			}
			return;
		}

		m_overallProgress = 90.0;
		Update(m_stageProgress, 100.0, params);

		msg.str("");
		msg << "Generate convex-hulls";
		m_operation = msg.str();
		size_t nConvexHulls = parts.Size();
		if (params.m_logger) {
			msg.str("");
			msg << "+ Generate " << nConvexHulls << " convex-hulls " << std::endl;
			params.m_logger->Log(msg.str().c_str());
		}

		Update(m_stageProgress, 0.0, params);
		m_convexHulls.Resize(0);
		for (size_t p = 0; p < nConvexHulls && !m_cancel; ++p) {
			Update(m_stageProgress, (double)p * 100.0 / (double)nConvexHulls, params);
			m_convexHulls.PushBack(new Mesh);
			parts[p]->ComputeConvexHull(*m_convexHulls[p]);
			size_t nv = m_convexHulls[p]->GetNPoints();
			double x, y, z;
			for (size_t i = 0; i < nv; ++i) {
				Vec3<double>& pt = m_convexHulls[p]->GetPoint(i);
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

		if (GetCancel()) {
			for (size_t p = 0; p < m_convexHulls.Size(); ++p) {
				delete m_convexHulls[p];
			}
			m_convexHulls.Clear();
			return;
		}

		m_overallProgress = 95.0;
		Update(100.0, 100.0, params);
		m_timer.Toc();
		if (params.m_logger) {
			msg.str("");
			msg << "\t time " << m_timer.GetElapsedTime() / 1000.0 << "s" << std::endl;
			params.m_logger->Log(msg.str().c_str());
		}
	}
	void AddPoints(const Mesh* const mesh, SArray<Vec3<double> >& pts)
	{
		const size_t n = mesh->GetNPoints();
		for (size_t i = 0; i < n; ++i) {
			pts.PushBack(mesh->GetPoint(i));
		}
	}
	void ComputeConvexHull(const Mesh* const ch1, const Mesh* const ch2, SArray<Vec3<double> >& pts, Mesh* const combinedCH)
	{
		pts.Resize(0);
		AddPoints(ch1, pts);
		AddPoints(ch2, pts);
	
		ConvexHull ch((double*)pts.Data(), 3 * sizeof(double), (int32_t)pts.Size(), 1.0e-5f);

		combinedCH->ResizePoints(0);
		combinedCH->ResizeTriangles(0);
		const std::vector<hullVector>& convexPoints = ch.GetVertexPool();
		for (size_t v = 0; v < convexPoints.size(); v++) {
			combinedCH->AddPoint(convexPoints[v]);
		}

		for (ConvexHull::ndNode* node = ch.GetFirst(); node; node = node->GetNext())
		{
			ConvexHullFace* const face = &node->GetInfo();
			combinedCH->AddTriangle(Vec3<int32_t>(face->m_index[0], face->m_index[1], face->m_index[2]));
		}
	}

	void VHACD::MergeConvexHulls(const Parameters& params)
	{
		if (GetCancel() || (m_convexHulls.Size() <= 1)) 
		{
			return;
		}
		m_timer.Tic();

		m_stage = "Merge Convex Hulls";

		std::ostringstream msg;
		if (params.m_logger) {
			msg << "+ " << m_stage << std::endl;
			params.m_logger->Log(msg.str().c_str());
		}

		struct ConvexProxy
		{
			Vec3<double> m_bmin;
			Vec3<double> m_bmax;
			Mesh* m_hull;
			int m_id;
		};

		struct ConvexKey
		{
			ConvexKey()
			{
			}

			ConvexKey(int i0, int i1)
				:m_p0(Min(i0, i1))
				,m_p1(Max(i0, i1))
			{
			}

			bool operator< (const ConvexKey& key) const
			{
				int key0 = (m_p1 << 16) + m_p0;
				int key1 = (key.m_p1 << 16) + key.m_p0;
				return key0 < key1;
			}

			int m_p0;
			int m_p1;
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
		
		std::set<ConvexKey> hullGraph;
		std::vector<ConvexPair> convexPairArray;
		std::vector<ConvexProxy> convexProxyArray;
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
				SArray<Vec3<double> > pts;
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

		MergeConvexJob jobBashes[VHACD_WORKERS_THREADS * 4 + 1];

		size_t pairsCount = 0;
		convexPairArray.resize(((m_convexHulls.Size()* m_convexHulls.Size()) - m_convexHulls.Size()) >> 1);

		for (size_t i = 0; i < m_convexHulls.Size(); ++i)
		{
			convexProxyArray.push_back(ConvexProxy());
			convexProxyArray[i].m_hull = new Mesh (*m_convexHulls[i]);
			convexProxyArray[i].m_id = int (i);
			convexProxyArray[i].m_hull->CalculateBoundingBox(convexProxyArray[i].m_bmin, convexProxyArray[i].m_bmax);
		}

		for (size_t i = 1; i < convexProxyArray.size(); ++i)
		{
			Vec3<double> bmin1(convexProxyArray[i].m_bmin);
			Vec3<double> bmax1(convexProxyArray[i].m_bmax);
			for (size_t j = 0; j < i; ++j)
			{
				Vec3<double> bmin0(convexProxyArray[j].m_bmin);
				Vec3<double> bmax0(convexProxyArray[j].m_bmax);
				Vec3<double> box1(bmax1 - bmin0);
				Vec3<double> box0(bmin1 - bmax0);
				Vec3<double> size(box0.X() * box1.X(), box0.Y()* box1.Y(), box0.Z()* box1.Z());

				if ((size[0] <= 0.0) && (size[1] <= 0.0) && (size[2] <= 0.0))
				{
					int i0 = int(i);
					int j0 = int(j);
					ConvexKey key(i0, j0);
					hullGraph.insert(key);
					convexPairArray[pairsCount] = ConvexPair(i0, j0);
					pairsCount++;
				}
			}
		}

		size_t nConvexHulls = m_convexHulls.Size();
		if (nConvexHulls > 1 && !m_cancel)
		{
			size_t start = 0;
			size_t bashSize = pairsCount / (VHACD_WORKERS_THREADS * 4);
			for (size_t j = 0; j < VHACD_WORKERS_THREADS * 4; ++j)
			{
				size_t count = (j + 1) * bashSize - start;
				if (count > 0)
				{
					jobBashes[j].m_pairs = &convexPairArray[start];
					jobBashes[j].m_pairsCount = int(count);
					jobBashes[j].m_volumeCH0 = m_volumeCH0;
					jobBashes[j].m_convexHulls = &m_convexHulls[0];
					m_parallelQueue.PushTask(&jobBashes[j]);
				}
				start += bashSize;
			}
			size_t count = pairsCount - start;
			if (count > 0)
			{
				jobBashes[VHACD_WORKERS_THREADS * 4].m_pairs = &convexPairArray[start];
				jobBashes[VHACD_WORKERS_THREADS * 4].m_pairsCount = int(count);
				jobBashes[VHACD_WORKERS_THREADS * 4].m_volumeCH0 = m_volumeCH0;
				jobBashes[VHACD_WORKERS_THREADS * 4].m_convexHulls = &m_convexHulls[0];
				m_parallelQueue.PushTask(&jobBashes[VHACD_WORKERS_THREADS * 4]);
			}

			m_parallelQueue.Sync();

			for (size_t i = 0; i < pairsCount; i++)
			{
				ConvexPair& pair = convexPairArray[i];
				if (pair.m_cost < (2.0f * params.m_minMergeToleranace))
				{
					priority.Push(pair, pair.m_cost);
				}
			}

			Mesh combinedCH;
			SArray<Vec3<double> > pts;
			while (((nConvexHulls > params.m_maxConvexHulls) || (priority.Value() <= params.m_minMergeToleranace)) && priority.GetCount())
			{
				ConvexKey key(priority[0]);
				std::set<ConvexKey>::iterator it = hullGraph.find(key);
				if (it != hullGraph.end())
				{
					hullGraph.erase(it);
					for (size_t i = 0; i < convexProxyArray.size(); i++)
					{
						if (convexProxyArray[i].m_hull)
						{
							ConvexKey key0(key.m_p0, int(i));
							hullGraph.erase(key0);

							ConvexKey key1(key.m_p1, int(i));
							hullGraph.erase(key1);
						}
					}

					Mesh* const newHull = new Mesh();
					size_t index = convexProxyArray.size();
					ComputeConvexHull(convexProxyArray[size_t(key.m_p0)].m_hull, convexProxyArray[size_t(key.m_p1)].m_hull, pts, newHull);
					convexProxyArray.push_back(ConvexProxy());
					convexProxyArray[index].m_hull = newHull;
					convexProxyArray[index].m_id = int(index);
					convexProxyArray[index].m_hull->CalculateBoundingBox(convexProxyArray[index].m_bmin, convexProxyArray[index].m_bmax);

					delete convexProxyArray[size_t(key.m_p0)].m_hull;
					delete convexProxyArray[size_t(key.m_p1)].m_hull;
					convexProxyArray[size_t(key.m_p0)].m_hull = nullptr;
					convexProxyArray[size_t(key.m_p1)].m_hull = nullptr;

					const float volume0 = float(newHull->ComputeVolume());

					const Vec3<double> bmin(convexProxyArray[index].m_bmin);
					const Vec3<double> bmax(convexProxyArray[index].m_bmax);

					for (size_t i = 0; i < convexProxyArray.size() - 1; i++)
					{
						if (convexProxyArray[i].m_hull)
						{
							Vec3<double> bmin0(convexProxyArray[i].m_bmin);
							Vec3<double> bmax0(convexProxyArray[i].m_bmax);
							Vec3<double> box1(bmax - bmin0);
							Vec3<double> box0(bmin - bmax0);
							Vec3<double> size(box0.X() * box1.X(), box0.Y() * box1.Y(), box0.Z() * box1.Z());

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
	
			for (int i = int (m_convexHulls.Size()-1); i >= 0; --i)
			{
				delete m_convexHulls[size_t(i)];
				m_convexHulls.PopBack();
			}

			for (size_t i = 0; i < convexProxyArray.size(); i++)
			{
				if (convexProxyArray[i].m_hull)
				{
					m_convexHulls.PushBack(convexProxyArray[i].m_hull);
				}
			}
		}

		m_overallProgress = 99.0;
		Update(100.0, 100.0, params);
		m_timer.Toc();
		if (params.m_logger) {
			msg.str("");
			msg << "\t time " << m_timer.GetElapsedTime() / 1000.0 << "s" << std::endl;
			params.m_logger->Log(msg.str().c_str());
		}
	}


	void VHACD::SimplifyConvexHull(Mesh* const ch, const size_t nvertices, const double minVolume)
	{
		if (nvertices <= 4) {
			return;
		}
		ICHull icHull;
		if (m_raycastMesh)
		{
			// We project these points onto the original source mesh to increase precision
			// The voxelization process drops floating point precision so returned data points are not exactly lying on the 
			// surface of the original source mesh.
			// The first step is we need to compute the bounding box of the mesh we are trying to build a convex hull for.
			// From this bounding box, we compute the length of the diagonal to get a relative size and center for point projection
			uint32_t nPoints = uint32_t(ch->GetNPoints());
			Vec3<double> *inputPoints = ch->GetPointsBuffer();
			Vec3<double> bmin(inputPoints[0]);
			Vec3<double> bmax(inputPoints[1]);
			for (uint32_t i = 1; i < nPoints; i++)
			{
				const Vec3<double> &p = inputPoints[i];
				p.UpdateMinMax(bmin, bmax);
			}
			Vec3<double> center;
			double diagonalLength = center.GetCenter(bmin, bmax);   // Get the center of the bounding box
			// This is the error threshold for determining if we should use the raycast result data point vs. the voxelized result.
			double pointDistanceThreshold = diagonalLength * 0.05;
			// If a new point is within 1/100th the diagonal length of the bounding volume we do not add it.  To do so would create a
			// thin sliver in the resulting convex hull
			double snapDistanceThreshold = diagonalLength * 0.01;
			double snapDistanceThresholdSquared = snapDistanceThreshold*snapDistanceThreshold;

			// Allocate buffer for projected vertices
			Vec3<double> *outputPoints = new Vec3<double>[nPoints];
			uint32_t outCount = 0;
			for (uint32_t i = 0; i < nPoints; i++)
			{
				Vec3<double> &inputPoint = inputPoints[i];
				Vec3<double> &outputPoint = outputPoints[outCount];
				// Compute the direction vector from the center of this mesh to the vertex
				Vec3<double> dir = inputPoint - center;
				// Normalize the direction vector.
				dir.Normalize();
				// Multiply times the diagonal length of the mesh
				dir *= diagonalLength;
				// Add the center back in again to get the destination point
				dir += center;
				// By default the output point is equal to the input point
				outputPoint = inputPoint;
				double pointDistance;
				if (m_raycastMesh->raycast(center.GetData(), dir.GetData(), inputPoint.GetData(), outputPoint.GetData(),&pointDistance) )
				{
					// If the nearest intersection point is too far away, we keep the original source data point.
					// Not all points lie directly on the original mesh surface
					if (pointDistance > pointDistanceThreshold)
					{
						outputPoint = inputPoint;
					}
				}
				// Ok, before we add this point, we do not want to create points which are extremely close to each other.
				// This will result in tiny sliver triangles which are really bad for collision detection.
				bool foundNearbyPoint = false;
				for (uint32_t j = 0; j < outCount; ++j)
				{
					// If this new point is extremely close to an existing point, we do not add it!
					double squaredDistance = outputPoints[j].GetDistanceSquared(outputPoint);
					if (squaredDistance < snapDistanceThresholdSquared )
					{
						foundNearbyPoint = true;
						break;
					}
				}
				if (!foundNearbyPoint)
				{
					outCount++;
				}
			}
			icHull.AddPoints(outputPoints, outCount);
			delete[]outputPoints;
		}
		else
		{
			icHull.AddPoints(ch->GetPointsBuffer(), ch->GetNPoints());
		}
		icHull.Process((uint32_t)nvertices, minVolume);
		TMMesh& mesh = icHull.GetMesh();
		const size_t nT = mesh.GetNTriangles();
		const size_t nV = mesh.GetNVertices();
		ch->ResizePoints(nV);
		ch->ResizeTriangles(nT);
		mesh.GetIFS(ch->GetPointsBuffer(), ch->GetTrianglesBuffer());
	}
	void VHACD::SimplifyConvexHulls(const Parameters& params)
	{
		if (m_cancel || params.m_maxNumVerticesPerCH < 4) {
			return;
		}
		m_timer.Tic();

		m_stage = "Simplify convex-hulls";
		m_operation = "Simplify convex-hulls";

		std::ostringstream msg;
		const size_t nConvexHulls = m_convexHulls.Size();
		if (params.m_logger) {
			msg << "+ Simplify " << nConvexHulls << " convex-hulls " << std::endl;
			params.m_logger->Log(msg.str().c_str());
		}

		Update(0.0, 0.0, params);
		for (size_t i = 0; i < nConvexHulls && !m_cancel; ++i) {
			if (params.m_logger) {
				msg.str("");
				msg << "\t\t Simplify CH[" << std::setfill('0') << std::setw(5) << i << "] " << m_convexHulls[i]->GetNPoints() << " V, " << m_convexHulls[i]->GetNTriangles() << " T" << std::endl;
				params.m_logger->Log(msg.str().c_str());
			}
			SimplifyConvexHull(m_convexHulls[i], params.m_maxNumVerticesPerCH, m_volumeCH0 * params.m_minVolumePerCH);
		}

		m_overallProgress = 100.0;
		Update(100.0, 100.0, params);
		m_timer.Toc();
		if (params.m_logger) {
			msg.str("");
			msg << "\t time " << m_timer.GetElapsedTime() / 1000.0 << "s" << std::endl;
			params.m_logger->Log(msg.str().c_str());
		}
	}
} // end of VHACD namespace
}
