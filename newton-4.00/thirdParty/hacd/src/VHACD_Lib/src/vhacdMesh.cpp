/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "vhacdMesh.h"
#include "FloatMath.h"
#include <fstream>
#include <iosfwd>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "vhacdConvexHull.h"

namespace nd_
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

			Vec3<double> bary(0.0, 0.0, 0.0);
			for (size_t v = 0; v < nV; v++) {
				bary += GetPoint(v);
			}
			bary /= static_cast<double>(nV);

			Vec3<double> ver0, ver1, ver2;
			double totalVolume = 0.0;
			for (size_t t = 0; t < nT; t++) {
				const Vec3<int32_t>& tri = GetTriangle(t);
				ver0 = GetPoint(size_t(tri[0]));
				ver1 = GetPoint(size_t(tri[1]));
				ver2 = GetPoint(size_t(tri[2]));
				totalVolume += ComputeVolume4(ver0, ver1, ver2, bary);
			}
			return totalVolume / 6.0;
		}

		void Mesh::ComputeConvexHull(const double* const pts, const size_t nPts)
		{
			ResizePoints(0);
			ResizeTriangles(0);
			ConvexHull ch(pts, 3 * sizeof(double), (int32_t)nPts, 1.0e-5f);

			const std::vector<hullVector>& convexPoints = ch.GetVertexPool();
			for (size_t v = 0; v < convexPoints.size(); v++) {
				AddPoint(convexPoints[v]);
			}

			for (ConvexHull::ndNode* node = ch.GetFirst(); node; node = node->GetNext())
			{
				ConvexHullFace* const face = &node->GetInfo();
				AddTriangle(Vec3<int32_t>(face->m_index[0], face->m_index[1], face->m_index[2]));
			}
		}

		void Mesh::Clip(const Plane& plane,
			SArray<Vec3<double> >& positivePart,
			SArray<Vec3<double> >& negativePart) const
		{
			const size_t nV = GetNPoints();
			if (nV == 0) {
				return;
			}
			double d;
			for (size_t v = 0; v < nV; v++) {
				const Vec3<double>& pt = GetPoint(v);
				d = plane.m_a * pt[0] + plane.m_b * pt[1] + plane.m_c * pt[2] + plane.m_d;
				if (d > 0.0) {
					positivePart.PushBack(pt);
				}
				else if (d < 0.0) {
					negativePart.PushBack(pt);
				}
				else {
					positivePart.PushBack(pt);
					negativePart.PushBack(pt);
				}
			}
		}
		bool Mesh::IsInside(const Vec3<double>& pt) const
		{
			const size_t nV = GetNPoints();
			const size_t nT = GetNTriangles();
			if (nV == 0 || nT == 0) {
				return false;
			}
			Vec3<double> ver0, ver1, ver2;
			double volume;
			for (size_t t = 0; t < nT; t++) {
				const Vec3<int32_t>& tri = GetTriangle(t);
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

		void Mesh::CalculateBoundingBox(Vec3<double>& p0, Vec3<double>& p1) const
		{
			Vec3<double> bmin(m_points[0]);
			Vec3<double> bmax(m_points[1]);
			for (uint32_t i = 1; i < m_points.Size(); i++)
			{
				const Vec3<double>& p = m_points[i];
				p.UpdateMinMax(bmin, bmax);
			}
			p0 = bmin;
			p1 = bmax;
		}

		#ifdef VHACD_DEBUG_MESH
		bool Mesh::SaveVRML2(const std::string& fileName) const
		{
			std::ofstream fout(fileName.c_str());
			if (fout.is_open()) {
				const Material material;

				if (SaveVRML2(fout, material)) {
					fout.close();
					return true;
				}
				return false;
			}
			return false;
		}
		bool Mesh::SaveVRML2(std::ofstream& fout, const Material& material) const
		{
			if (fout.is_open()) {
				fout.setf(std::ios::fixed, std::ios::floatfield);
				fout.setf(std::ios::showpoint);
				fout.precision(6);
				size_t nV = m_points.Size();
				size_t nT = m_triangles.Size();
				fout << "#VRML V2.0 utf8" << std::endl;
				fout << "" << std::endl;
				fout << "# Vertices: " << nV << std::endl;
				fout << "# Triangles: " << nT << std::endl;
				fout << "" << std::endl;
				fout << "Group {" << std::endl;
				fout << "    children [" << std::endl;
				fout << "        Shape {" << std::endl;
				fout << "            appearance Appearance {" << std::endl;
				fout << "                material Material {" << std::endl;
				fout << "                    diffuseColor " << material.m_diffuseColor[0] << " "
					<< material.m_diffuseColor[1] << " "
					<< material.m_diffuseColor[2] << std::endl;
				fout << "                    ambientIntensity " << material.m_ambientIntensity << std::endl;
				fout << "                    specularColor " << material.m_specularColor[0] << " "
					<< material.m_specularColor[1] << " "
					<< material.m_specularColor[2] << std::endl;
				fout << "                    emissiveColor " << material.m_emissiveColor[0] << " "
					<< material.m_emissiveColor[1] << " "
					<< material.m_emissiveColor[2] << std::endl;
				fout << "                    shininess " << material.m_shininess << std::endl;
				fout << "                    transparency " << material.m_transparency << std::endl;
				fout << "                }" << std::endl;
				fout << "            }" << std::endl;
				fout << "            geometry IndexedFaceSet {" << std::endl;
				fout << "                ccw TRUE" << std::endl;
				fout << "                solid TRUE" << std::endl;
				fout << "                convex TRUE" << std::endl;
				if (nV > 0) {
					fout << "                coord DEF co Coordinate {" << std::endl;
					fout << "                    point [" << std::endl;
					for (size_t v = 0; v < nV; v++) {
						fout << "                        " << m_points[v][0] << " "
							<< m_points[v][1] << " "
							<< m_points[v][2] << "," << std::endl;
					}
					fout << "                    ]" << std::endl;
					fout << "                }" << std::endl;
				}
				if (nT > 0) {
					fout << "                coordIndex [ " << std::endl;
					for (size_t f = 0; f < nT; f++) {
						fout << "                        " << m_triangles[f][0] << ", "
							<< m_triangles[f][1] << ", "
							<< m_triangles[f][2] << ", -1," << std::endl;
					}
					fout << "                ]" << std::endl;
				}
				fout << "            }" << std::endl;
				fout << "        }" << std::endl;
				fout << "    ]" << std::endl;
				fout << "}" << std::endl;
				return true;
			}
			return false;
		}
		bool Mesh::SaveOFF(const std::string& fileName) const
		{
			std::ofstream fout(fileName.c_str());
			if (fout.is_open()) {
				size_t nV = m_points.Size();
				size_t nT = m_triangles.Size();
				fout << "OFF" << std::endl;
				fout << nV << " " << nT << " " << 0 << std::endl;
				for (size_t v = 0; v < nV; v++) {
					fout << m_points[v][0] << " "
						<< m_points[v][1] << " "
						<< m_points[v][2] << std::endl;
				}
				for (size_t f = 0; f < nT; f++) {
					fout << "3 " << m_triangles[f][0] << " "
						<< m_triangles[f][1] << " "
						<< m_triangles[f][2] << std::endl;
				}
				fout.close();
				return true;
			}
			return false;
		}

		bool Mesh::LoadOFF(const std::string& fileName, bool invert)
		{
			int ret = 0;
			ret++;
			FILE* fid = fopen(fileName.c_str(), "r");
			if (fid) {
				const std::string strOFF("OFF");
				char temp[1024];
				ret = fscanf(fid, "%s", temp);
				if (std::string(temp) != strOFF) {
					fclose(fid);
					return false;
				}
				else {
					int32_t nv = 0;
					int32_t nf = 0;
					int32_t ne = 0;
					ret = fscanf(fid, "%i", &nv);
					ret = fscanf(fid, "%i", &nf);
					ret = fscanf(fid, "%i", &ne);
					m_points.Resize(size_t(nv));
					m_triangles.Resize(size_t(nf));
					Vec3<double> coord;
					float x, y, z;
					for (int32_t p = 0; p < nv; p++) {
						ret = fscanf(fid, "%f", &x);
						ret = fscanf(fid, "%f", &y);
						ret = fscanf(fid, "%f", &z);
						m_points[size_t(p)][0] = x;
						m_points[size_t(p)][1] = y;
						m_points[size_t(p)][2] = z;
					}
					int32_t i, j, k, s;
					for (size_t t = 0; t < size_t(nf); ++t) {
						ret = fscanf(fid, "%i", &s);
						if (s == 3) {
							ret = fscanf(fid, "%i", &i);
							ret = fscanf(fid, "%i", &j);
							ret = fscanf(fid, "%i", &k);
							m_triangles[t][0] = i;
							if (invert) {
								m_triangles[t][1] = k;
								m_triangles[t][2] = j;
							}
							else {
								m_triangles[t][1] = j;
								m_triangles[t][2] = k;
							}
						}
						else // Fix me: support only triangular meshes
						{
							for (int32_t h = 0; h < s; ++h)
								ret = fscanf(fid, "%i", &s);
						}
					}
					fclose(fid);
				}
			}
			else {
				return false;
			}
			return true;
		}
		#endif // VHACD_DEBUG_MESH
	}
}