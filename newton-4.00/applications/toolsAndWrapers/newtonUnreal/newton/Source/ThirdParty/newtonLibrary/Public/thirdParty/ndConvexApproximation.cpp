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
#include "VHACD.h"
#include "ndConvexApproximation.h"

class ndConvexApproximation::ProgressBar: public nd_::VHACD::IVHACD::IUserCallback
{
	public:	
	ProgressBar(ndConvexApproximation* const owner)
		:m_owner(owner)
	{
	}

	virtual void Update(const double overallProgress,
		const double stageProgress,
		const double operationProgress,
		const char* const stage,
		const char* const operation)
	{
		m_owner->ShowProgress();
	}

	ndConvexApproximation* m_owner;
};

ndConvexApproximation::ndConvexApproximation(ndInt32 maxConvexes, bool quality)
	:ndClassAlloc()
	,m_tolerance(ndFloat32 (1.0e-3f))
	,m_maxConvexes(maxConvexes)
	,m_quality(quality)
	,m_maxPointPerHull(64)
{
}

ndConvexApproximation::~ndConvexApproximation()
{
	for (ndInt32 i = ndInt32(m_ouputHulls.GetCount()) - 1; i >= 0; --i)
	{
		delete m_ouputHulls[i];
	}
}

void ndConvexApproximation::ShowProgress()
{
}

void ndConvexApproximation::Execute()
{
	ndArray<ndHullPoint> points;
	ndArray<ndInt32> remapIndex;
	ndArray<ndHullInputMesh::ndFace> faces;
	for (ndInt32 i = 0; i < m_inputMesh.m_points.GetCount(); ++i)
	{
		points.PushBack(m_inputMesh.m_points[i]);
	}
	remapIndex.SetCount(points.GetCount());
	ndInt32 vCount = ndVertexListToIndexList(&points[0].m_x, sizeof(ndHullPoint), 3, ndInt32(points.GetCount()), &remapIndex[0], ndFloat32(1.0e-6f));

	for (ndInt32 i = ndInt32(m_inputMesh.m_faces.GetCount()) - 1; i >= 0; --i)
	{
		ndInt32 j;
		ndHullInputMesh::ndFace face;

		j = m_inputMesh.m_faces[i].m_i0;
		face.m_i0 = remapIndex[j];

		j = m_inputMesh.m_faces[i].m_i1;
		face.m_i1 = remapIndex[j];

		j = m_inputMesh.m_faces[i].m_i2;
		face.m_i2 = remapIndex[j];

		faces.PushBack(face);
	}

	ProgressBar progressBar(this);
	nd_::VHACD::IVHACD* const interfaceVHACD = nd_::VHACD::CreateVHACD();
	nd_::VHACD::IVHACD::Parameters paramsVHACD;
	paramsVHACD.m_callback = &progressBar;
	paramsVHACD.m_maxConvexHulls = m_maxConvexes;
	paramsVHACD.m_concavityToVolumeWeigh = m_quality ? 1.0f : 0.5f;

	interfaceVHACD->Compute(&points[0].m_x, uint32_t(vCount), (uint32_t*)&faces[0].m_i0, uint32_t(faces.GetCount()), paramsVHACD);

	ndInt32 hullCount = ndInt32(interfaceVHACD->GetNConvexHulls());
	for (ndInt32 i = 0; i < hullCount; ++i)
	{
		nd_::VHACD::IVHACD::ConvexHull ch;
		interfaceVHACD->GetConvexHull(uint32_t(i), ch);
		ndHullOutput* const outputHull = new ndHullOutput;
		if (ndInt32 (ch.m_nPoints) <= m_maxPointPerHull)
		{
			for (ndInt32 j = 0; j < ndInt32(ch.m_nPoints); ++j)
			{
				ndHullPoint point;
				point.m_x = ndReal(ch.m_points[j * 3 + 0]);
				point.m_y = ndReal(ch.m_points[j * 3 + 1]);
				point.m_z = ndReal(ch.m_points[j * 3 + 2]);
				outputHull->PushBack(point);
			}
		}
		else
		{
			ndConvexHull3d convexHull(&ch.m_points[0], 3 * sizeof(ndFloat64), ch.m_nPoints, m_tolerance, m_maxPointPerHull);
			const ndArray<ndBigVector>& points = convexHull.GetVertexPool();
			for (ndInt32 j = 0; j < ndInt32(points.GetCount()); ++j)
			{
				ndHullPoint point;
				point.m_x = ndReal(points[j].m_x);
				point.m_y = ndReal(points[j].m_y);
				point.m_z = ndReal(points[j].m_z);
				outputHull->PushBack(point);
			}
		}
		m_ouputHulls.PushBack(outputHull);
	}

	interfaceVHACD->Clean();
	interfaceVHACD->Release();
}
