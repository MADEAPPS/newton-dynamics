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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndShapeInstance.h"
#include "ndContactSolver.h"
#include "ndCollisionStdafx.h"
#include "ndShapeStaticMesh.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndShapeStaticMesh)

void ndPolygonMeshDesc::SortFaceArray()
{
	ndInt32 stride = 8;
	if (m_faceCount >= 8)
	{
		ndInt32 stack[D_MAX_COLLIDING_FACES][2];

		stack[0][0] = 0;
		stack[0][1] = m_faceCount - 1;
		ndInt32 stackIndex = 1;
		while (stackIndex)
		{
			stackIndex--;
			ndInt32 lo = stack[stackIndex][0];
			ndInt32 hi = stack[stackIndex][1];
			if ((hi - lo) > stride)
			{
				ndInt32 i = lo;
				ndInt32 j = hi;
				ndFloat32 dist = m_hitDistance[(lo + hi) >> 1];
				do
				{
					while (m_hitDistance[i] < dist) i++;
					while (m_hitDistance[j] > dist) j--;

					if (i <= j)
					{
						dSwap(m_hitDistance[i], m_hitDistance[j]);
						dSwap(m_faceIndexStart[i], m_faceIndexStart[j]);
						dSwap(m_faceIndexCount[i], m_faceIndexCount[j]);
						i++;
						j--;
					}
				} while (i <= j);

				if (i < hi)
				{
					stack[stackIndex][0] = i;
					stack[stackIndex][1] = hi;
					stackIndex++;
				}
				if (lo < j) {
					stack[stackIndex][0] = lo;
					stack[stackIndex][1] = j;
					stackIndex++;
				}
				dAssert(stackIndex < ndInt32(sizeof(stack) / (2 * sizeof(stack[0][0]))));
			}
		}
	}

	stride = stride * 2;
	if (m_faceCount < stride)
	{
		stride = m_faceCount;
	}
	for (ndInt32 i = 1; i < stride; i++)
	{
		if (m_hitDistance[i] < m_hitDistance[0])
		{
			dSwap(m_hitDistance[i], m_hitDistance[0]);
			dSwap(m_faceIndexStart[i], m_faceIndexStart[0]);
			dSwap(m_faceIndexCount[i], m_faceIndexCount[0]);
		}
	}

	for (ndInt32 i = 1; i < m_faceCount; i++)
	{
		ndInt32 j = i;
		ndInt32 ptr = m_faceIndexStart[i];
		ndInt32 count = m_faceIndexCount[i];
		ndFloat32 dist = m_hitDistance[i];
		for (; dist < m_hitDistance[j - 1]; j--)
		{
			dAssert(j > 0);
			m_hitDistance[j] = m_hitDistance[j - 1];
			m_faceIndexStart[j] = m_faceIndexStart[j - 1];
			m_faceIndexCount[j] = m_faceIndexCount[j - 1];
		}
		m_hitDistance[j] = dist;
		m_faceIndexStart[j] = ptr;
		m_faceIndexCount[j] = count;
	}

#ifdef _DEBUG
	for (ndInt32 i = 0; i < m_faceCount - 1; i++) {
		dAssert(m_hitDistance[i] <= m_hitDistance[i + 1]);
	}
#endif
}

ndPolygonMeshDesc::ndPolygonMeshDesc(ndContactSolver& proxy, bool ccdMode)
	:ndFastAabb()
	,m_boxDistanceTravelInMeshSpace(ndVector::m_zero)
	,m_faceCount(0)
	,m_vertexStrideInBytes(0)
	,m_skinMargin(proxy.m_skinMargin)
	,m_convexInstance(&proxy.m_instance0)
	,m_polySoupInstance(&proxy.m_instance1)
	,m_vertex(nullptr)
	,m_faceIndexCount(nullptr)
	,m_faceVertexIndex(nullptr)
	,m_faceIndexStart(nullptr)
	,m_hitDistance(nullptr)
	,m_me(nullptr)
	,m_maxT(ndFloat32(1.0f))
	,m_globalIndexCount(0)
	,m_threadId(proxy.m_threadId)
	,m_doContinueCollisionTest(ccdMode)
{
	const ndMatrix& hullMatrix = m_convexInstance->GetGlobalMatrix();
	const ndMatrix& soupMatrix = m_polySoupInstance->GetGlobalMatrix();
	
	ndMatrix& matrix = *this;
	matrix = hullMatrix * soupMatrix.Inverse();
	ndMatrix convexMatrix (dGetIdentityMatrix());
	
	switch (m_polySoupInstance->GetScaleType())
	{
		case ndShapeInstance::m_unit:
		{
			break;
		}
	
		case ndShapeInstance::m_uniform:
		{
			const ndVector& invScale = m_polySoupInstance->GetInvScale();
			convexMatrix[0][0] = invScale.GetScalar();
			convexMatrix[1][1] = invScale.GetScalar();
			convexMatrix[2][2] = invScale.GetScalar();
			matrix.m_posit = matrix.m_posit * (invScale | ndVector::m_wOne);
			break;
		}
	
		case ndShapeInstance::m_nonUniform:
		{
			const ndVector& invScale = m_polySoupInstance->GetInvScale();
			ndMatrix tmp (matrix[0] * invScale, matrix[1] * invScale, matrix[2] * invScale, ndVector::m_wOne);
			convexMatrix = tmp * matrix.Inverse();
			convexMatrix.m_posit = ndVector::m_wOne;
			matrix.m_posit = matrix.m_posit * (invScale | ndVector::m_wOne);
			break;
		}
	
		case ndShapeInstance::m_global:
		default:
		{
		   dAssert (0);
		}
	}
	
	ndMatrix fullMatrix (convexMatrix * matrix);
	m_convexInstance->CalculateAabb(fullMatrix, m_p0, m_p1);
	
	ndVector p0;
	ndVector p1;
	SetTransposeAbsMatrix(matrix);
	m_convexInstance->CalculateAabb(convexMatrix, p0, p1);
	m_size = ndVector::m_half * (p1 - p0);
	m_posit = matrix.TransformVector(ndVector::m_half * (p1 + p0));
	dAssert (m_posit.m_w == ndFloat32 (1.0f));
}

ndShapeStaticMesh::ndShapeStaticMesh(ndShapeID id)
	:ndShape(id)
{
}

ndShapeStaticMesh::ndShapeStaticMesh(const ndLoadSaveBase::ndLoadDescriptor&)
	:ndShape(m_staticMesh)
{
}


ndShapeStaticMesh::~ndShapeStaticMesh()
{
}

void ndShapeStaticMesh::CalculateAabb(const ndMatrix& matrix, ndVector &p0, ndVector &p1) const
{
	ndVector origin(matrix.TransformVector(m_boxOrigin));
	ndVector size(matrix.m_front.Abs().Scale(m_boxSize.m_x) + matrix.m_up.Abs().Scale(m_boxSize.m_y) + matrix.m_right.Abs().Scale(m_boxSize.m_z));

	p0 = (origin - size) & ndVector::m_triplexMask;
	p1 = (origin + size) & ndVector::m_triplexMask;
}


//ndInt32 ndShapeStaticMesh::CalculatePlaneIntersection(const ndFloat32* const vertex, const ndInt32* const index, ndInt32 indexCount, ndInt32 stride, const dPlane& localPlane, ndVector* const contactsOut) const
ndInt32 ndShapeStaticMesh::CalculatePlaneIntersection(const ndFloat32* const, const ndInt32* const, ndInt32, ndInt32, const ndPlane&, ndVector* const) const
{
	dAssert(0);
	return 0;
	//ndInt32 count = 0;
	//ndInt32 j = index[indexCount - 1] * stride;
	//ndVector p0(&vertex[j]);
	//p0 = p0 & ndVector::m_triplexMask;
	//ndFloat32 side0 = localPlane.Evalue(p0);
	//for (ndInt32 i = 0; i < indexCount; i++) {
	//	j = index[i] * stride;
	//	ndVector p1(&vertex[j]);
	//	p1 = p1 & ndVector::m_triplexMask;
	//	ndFloat32 side1 = localPlane.Evalue(p1);
	//
	//	if (side0 < ndFloat32(0.0f)) {
	//		if (side1 >= ndFloat32(0.0f)) {
	//			ndVector dp(p1 - p0);
	//			dAssert(dp.m_w == ndFloat32(0.0f));
	//			ndFloat32 t = localPlane.DotProduct(dp).GetScalar();
	//			dAssert(dgAbs(t) >= ndFloat32(0.0f));
	//			if (dgAbs(t) < ndFloat32(1.0e-8f)) {
	//				t = dgSign(t) * ndFloat32(1.0e-8f);
	//			}
	//			dAssert(0);
	//			contactsOut[count] = p0 - dp.Scale(side0 / t);
	//			count++;
	//
	//		}
	//	}
	//	else if (side1 <= ndFloat32(0.0f)) {
	//		ndVector dp(p1 - p0);
	//		dAssert(dp.m_w == ndFloat32(0.0f));
	//		ndFloat32 t = localPlane.DotProduct(dp).GetScalar();
	//		dAssert(dgAbs(t) >= ndFloat32(0.0f));
	//		if (dgAbs(t) < ndFloat32(1.0e-8f)) {
	//			t = dgSign(t) * ndFloat32(1.0e-8f);
	//		}
	//		dAssert(0);
	//		contactsOut[count] = p0 - dp.Scale(side0 / t);
	//		count++;
	//	}
	//
	//	side0 = side1;
	//	p0 = p1;
	//}
	//
	//return count;
}

void ndShapeStaticMesh::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndShape::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));
}