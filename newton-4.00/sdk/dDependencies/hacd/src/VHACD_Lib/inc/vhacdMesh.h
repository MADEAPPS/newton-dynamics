/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ND_VHACD_MESH_H
#define ND_VHACD_MESH_H

#include "vhacdSArray.h"
#include "vhacdVector.h"

namespace nd
{
	namespace VHACD 
	{
		enum AXIS 
		{
			AXIS_X = 0,
			AXIS_Y = 1,
			AXIS_Z = 2
		};
		struct Plane 
		{
			double m_a;
			double m_b;
			double m_c;
			double m_d;
			AXIS m_axis;
			short m_index;
		};

		//! Triangular mesh data structure
		class Mesh 
		{
			public:
			void AddPoint(const Vec3& pt) { m_points.PushBack(pt); }
			void SetPoint(size_t index, const Vec3& pt) { m_points[index] = pt; }
			const Vec3& GetPoint(size_t index) const { return m_points[index]; }
			Vec3& GetPoint(size_t index) { return m_points[index]; }
			size_t GetNPoints() const { return m_points.Size(); }
			double* GetPoints() { return (double*)m_points.Data(); } // ugly
			const double* GetPoints() const { return (double*)m_points.Data(); } // ugly
			const Vec3* GetPointsBuffer() const { return m_points.Data(); } //
			Vec3* GetPointsBuffer() { return m_points.Data(); } //
			void AddTriangle(const Triangle& tri) { m_triangles.PushBack(tri); }
			void SetTriangle(size_t index, const Triangle& tri) { m_triangles[index] = tri; }
			const Triangle& GetTriangle(size_t index) const { return m_triangles[index]; }
			Triangle& GetTriangle(size_t index) { return m_triangles[index]; }
			size_t GetNTriangles() const { return m_triangles.Size(); }
			int32_t* GetTriangles() { return (int32_t*)m_triangles.Data(); } // ugly
			const int32_t* GetTriangles() const { return (int32_t*)m_triangles.Data(); } // ugly
			const Triangle* GetTrianglesBuffer() const { return m_triangles.Data(); }
			Triangle* GetTrianglesBuffer() { return m_triangles.Data(); }
			void ClearPoints() { m_points.Clear(); }
			void ClearTriangles() { m_triangles.Clear(); }
			void Clear()
			{
				ClearPoints();
				ClearTriangles();
			}
			void ResizePoints(size_t nPts) { m_points.Resize(nPts); }
			void ResizeTriangles(size_t nTri) { m_triangles.Resize(nTri); }
			void CopyPoints(SArray<Vec3>& points) const { points = m_points; }
			double ComputeVolume() const;
			void ComputeConvexHull(const Vec3* const pts, const size_t nPts);
			void Clip(const Plane& plane,
				SArray<Vec3>& positivePart,
				SArray<Vec3>& negativePart) const;
			bool IsInside(const Vec3& pt) const;
			void CalculateBoundingBox(Vec3& p0, Vec3& p1) const;

			//! Constructor.
			Mesh();
			//! Destructor.
			~Mesh(void);

			Mesh(const Mesh& src);

			private:
			SArray<Vec3> m_points;
			SArray<Triangle> m_triangles;
		};
	}
}
#endif