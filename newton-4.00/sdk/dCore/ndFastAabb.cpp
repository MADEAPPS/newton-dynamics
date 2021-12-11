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
#include "ndTypes.h"
#include "ndMemory.h"
#include "ndVector.h"
#include "ndPlane.h"
#include "ndGoogol.h"
#include "ndFastRay.h"
#include "ndFastAabb.h"
#include "ndIntersections.h"

#define D_USE_FLOAT_VERSION
#define D_RAY_TOL_ERROR (dFloat32 (-1.0e-3f))
#define D_RAY_TOL_ADAPTIVE_ERROR (dFloat32 (1.0e-1f))

void ndFastAabb::MakeBox1(dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, ndVector& minBox, ndVector& maxBox) const
{
	ndVector faceBoxP0(&vertexArray[indexArray[0] * stride]);
	faceBoxP0 = faceBoxP0 & ndVector::m_triplexMask;
	ndVector faceBoxP1(faceBoxP0);
	for (dInt32 i = 1; i < indexCount; i++)
	{
		ndVector p(&vertexArray[indexArray[i] * stride]);
		p = p & ndVector::m_triplexMask;
		faceBoxP0 = faceBoxP0.GetMin(p);
		faceBoxP1 = faceBoxP1.GetMax(p);
	}

	minBox = faceBoxP0 - m_p1;
	maxBox = faceBoxP1 - m_p0;
}

void ndFastAabb::MakeBox2(const ndMatrix& faceMatrix, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, ndVector& minBox, ndVector& maxBox) const
{
	ndVector faceBoxP0(faceMatrix.TransformVector(ndVector(&vertexArray[indexArray[0] * stride]) & ndVector::m_triplexMask));
	ndVector faceBoxP1(faceBoxP0);
	for (dInt32 i = 1; i < indexCount; i++)
	{
		ndVector p(faceMatrix.TransformVector(ndVector(&vertexArray[indexArray[i] * stride]) & ndVector::m_triplexMask));
		faceBoxP0 = faceBoxP0.GetMin(p);
		faceBoxP1 = faceBoxP1.GetMax(p);
	}
	faceBoxP0 = faceBoxP0 & ndVector::m_triplexMask;
	faceBoxP1 = faceBoxP1 & ndVector::m_triplexMask;

	ndMatrix matrix = *this * faceMatrix;
	ndVector size(matrix[0].Abs().Scale(m_size.m_x) + matrix[1].Abs().Scale(m_size.m_y) + matrix[2].Abs().Scale(m_size.m_z));
	ndVector boxP0((matrix.m_posit - size) & ndVector::m_triplexMask);
	ndVector boxP1((matrix.m_posit + size) & ndVector::m_triplexMask);

	minBox = faceBoxP0 - boxP1;
	maxBox = faceBoxP1 - boxP0;
}

ndMatrix ndFastAabb::MakeFaceMatrix(const ndVector& faceNormal, dInt32, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray) const
{
	ndMatrix faceMatrix;
	ndVector origin(&vertexArray[indexArray[0] * stride]);
	ndVector pin(&vertexArray[indexArray[0] * stride]);
	pin = pin & ndVector::m_triplexMask;
	origin = origin & ndVector::m_triplexMask;

	ndVector pin1(&vertexArray[indexArray[1] * stride]);
	pin1 = pin1 & ndVector::m_triplexMask;

	faceMatrix[0] = faceNormal;
	faceMatrix[1] = pin1 - origin;
	faceMatrix[1] = faceMatrix[1].Normalize();
	faceMatrix[2] = faceMatrix[0].CrossProduct(faceMatrix[1]);
	faceMatrix[3] = origin | ndVector::m_wOne;
	return faceMatrix.Inverse();
}

dFloat32 ndFastAabb::PolygonBoxRayDistance(const ndVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, const ndFastRay& ray) const
{
	ndVector minBox;
	ndVector maxBox;
	MakeBox1(indexCount, indexArray, stride, vertexArray, minBox, maxBox);
	dFloat32 dist0 = ray.BoxIntersect(minBox, maxBox);
	if (dist0 < dFloat32(1.0f))
	{
		ndMatrix faceMatrix(MakeFaceMatrix(faceNormal, indexCount, indexArray, stride, vertexArray));

		MakeBox2(faceMatrix, indexCount, indexArray, stride, vertexArray, minBox, maxBox);
		ndVector veloc(faceMatrix.RotateVector(ray.m_diff) & ndVector::m_triplexMask);
		ndFastRay localRay(ndVector(dFloat32(0.0f)), veloc);
		dFloat32 dist1 = localRay.BoxIntersect(minBox, maxBox);
		dist0 = dMax(dist1, dist0);
	}
	return dist0;
}

dFloat32 ndFastAabb::PolygonBoxDistance(const ndVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray) const
{
	ndVector minBox;
	ndVector maxBox;
	MakeBox1(indexCount, indexArray, stride, vertexArray, minBox, maxBox);

	ndVector mask(minBox * maxBox < ndVector(dFloat32(0.0f)));
	ndVector dist(maxBox.GetMin(minBox.Abs()) & mask);
	dist = dist.GetMin(dist.ShiftTripleRight());
	dist = dist.GetMin(dist.ShiftTripleRight());
	dFloat32 dist0 = dist.GetScalar();
	if (dist0 > dFloat32(0.0f))
	{
		ndMatrix faceMatrix(MakeFaceMatrix(faceNormal, indexCount, indexArray, stride, vertexArray));
		MakeBox2(faceMatrix, indexCount, indexArray, stride, vertexArray, minBox, maxBox);

		ndVector mask2(minBox * maxBox < ndVector(dFloat32(0.0f)));
		ndVector dist2(maxBox.GetMin(minBox.Abs()) & mask2);
		dist2 = dist2.GetMin(dist2.ShiftTripleRight());
		dist2 = dist2.GetMin(dist2.ShiftTripleRight());
		dFloat32 dist1 = dist2.GetScalar();
		dist0 = (dist1 > dFloat32(0.0f)) ? dMax(dist0, dist1) : dFloat32(0.0f);
		if (dist0 <= dFloat32(0.0f))
		{
			// some how clang crashes in relese and release with debug, 
			// but not Visual studio, Intel or Gcc
			// I do can't determine why because Clang inline teh code without debug 
			//ndVector p1p0((minBox.Abs()).GetMin(maxBox.Abs()).AndNot(mask2));
			const ndVector box0(minBox.Abs());
			const ndVector box1(maxBox.Abs());
			const ndVector p1p0((box0.GetMin(box1)).AndNot(mask2));
			dist2 = p1p0.DotProduct(p1p0);
			dist2 = dist2.Sqrt() * ndVector::m_negOne;
			dist0 = dist2.GetScalar();
		}
	}
	else
	{
		//ndVector p1p0((minBox.Abs()).GetMin(maxBox.Abs()).AndNot(mask));
		const ndVector box0(minBox.Abs());
		const ndVector box1(maxBox.Abs());
		const ndVector p1p0(box0.GetMin(box1).AndNot(mask));
		dist = p1p0.DotProduct(p1p0);
		dist = dist.Sqrt() * ndVector::m_negOne;
		dist0 = dist.GetScalar();
	}
	return	dist0;
}


