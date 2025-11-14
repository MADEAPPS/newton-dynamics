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

#include <fstream>
#include <iosfwd>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "vhacdMesh.h"
#include "vhacdConvexHull.h"

namespace nd
{
	namespace VHACD 
	{
		Mesh::Mesh()
		{
		}
		Mesh::~Mesh()
		{
		}

		Mesh::Mesh(const Mesh& src)
		{
			size_t nV = src.m_points.Size();
			size_t nT = src.m_triangles.Size();
			for (size_t v = 0; v < nV; v++) 
			{
				m_points.PushBack(src.m_points[v]);
			}
			for (size_t f = 0; f < nT; f++) 
			{
				m_triangles.PushBack(src.m_triangles[f]);
			}
		}

		double Mesh::ComputeVolume() const
		{
			const size_t nV = GetNPoints();
			const size_t nT = GetNTriangles();
			if (nV == 0 || nT == 0) {
				return 0.0;
			}

			Vec3 bary(0.0, 0.0, 0.0);
			for (size_t v = 0; v < nV; v++) 
			{
				bary += GetPoint(v);
			}
			bary /= static_cast<double>(nV);

			Vec3 ver0, ver1, ver2;
			double totalVolume = 0.0;
			for (size_t t = 0; t < nT; t++) 
			{
				const Triangle& tri = GetTriangle(t);
				ver0 = GetPoint(size_t(tri[0]));
				ver1 = GetPoint(size_t(tri[1]));
				ver2 = GetPoint(size_t(tri[2]));
				totalVolume += ComputeVolume4(ver0, ver1, ver2, bary);
			}
			return totalVolume / 6.0;
		}

		void Mesh::ComputeConvexHull(const Vec3* const pts, const size_t nPts)
		{
			ResizePoints(0);
			ResizeTriangles(0);
			ConvexHull ch(&pts[0][0], sizeof(Vec3), (int32_t)nPts, 1.0e-5f);
		
			const ndArray<ndBigVector>& convexPoints = ch.GetVertexPool();
			for (ndInt32 v = 0; v < ndInt32(convexPoints.GetCount()); v++)
			{
				const Vec3 hullPoint(convexPoints[v].m_x, convexPoints[v].m_y, convexPoints[v].m_z);
				AddPoint(hullPoint);
			}
			
			for (ConvexHull::ndNode* node = ch.GetFirst(); node; node = node->GetNext())
			{
				const ndConvexHull3dFace* const face = &node->GetInfo();
				AddTriangle(Triangle(face->m_index[0], face->m_index[1], face->m_index[2]));
			}
		}

		void Mesh::Clip(const Plane& plane,
			SArray<Vec3>& positivePart,
			SArray<Vec3>& negativePart) const
		{
			const size_t nV = GetNPoints();
			if (nV == 0) 
			{
				return;
			}
			double d;
			for (size_t v = 0; v < nV; v++) 
			{
				const Vec3& pt = GetPoint(v);
				d = plane.m_a * pt[0] + plane.m_b * pt[1] + plane.m_c * pt[2] + plane.m_d;
				if (d > 0.0) 
				{
					positivePart.PushBack(pt);
				}
				else if (d < 0.0) 
				{
					negativePart.PushBack(pt);
				}
				else 
				{
					positivePart.PushBack(pt);
					negativePart.PushBack(pt);
				}
			}
		}

		bool Mesh::IsInside(const Vec3& pt) const
		{
			const size_t nV = GetNPoints();
			const size_t nT = GetNTriangles();
			if (nV == 0 || nT == 0) {
				return false;
			}
			Vec3 ver0, ver1, ver2;
			double volume;
			for (size_t t = 0; t < nT; t++) {
				const Triangle& tri = GetTriangle(t);
				ver0 = GetPoint(size_t(tri[0]));
				ver1 = GetPoint(size_t(tri[1]));
				ver2 = GetPoint(size_t(tri[2]));
				volume = ComputeVolume4(ver0, ver1, ver2, pt);
				if (volume < 0.0) {
					return false;
				}
			}
			return true;
		}

		void Mesh::CalculateBoundingBox(Vec3& p0, Vec3& p1) const
		{
			Vec3 bmin(m_points[0]);
			Vec3 bmax(m_points[1]);
			for (uint32_t i = 1; i < m_points.Size(); i++)
			{
				const Vec3& p = m_points[i];
				p.UpdateMinMax(bmin, bmax);
			}
			p0 = bmin;
			p1 = bmax;
		}
	}
}