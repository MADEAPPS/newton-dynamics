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

#ifndef __ND_CONVEXHULL_3D__
#define __ND_CONVEXHULL_3D__

#include <list>
#include <vector>
#include "vhacdVector.h"
//#include "ndCoreStdafx.h"
//#include "ndList.h"
//#include "ndArray.h"
//#include "ndPlane.h"
//#include "ndVector.h"
//#include "ndMatrix.h"
//#include "ndQuaternion.h"

class vhacdConvexHullVertex;
class vhacdConvexHullAABBTreeNode;

class hullVector : public VHACD::Vec3<double>
{
	public:
	hullVector()
		:Vec3<double>(0, 0, 0)
	{
	}

	hullVector(double x)
		:Vec3<double>(x, x, x)
	{
	}

	hullVector(const hullVector& x)
		:Vec3<double>(x.X(), x.Y(), x.Z())
	{
	}

	hullVector(double x, double y, double z, double)
		:Vec3<double>(x, y, z)
	{
	}

	hullVector GetMin(const hullVector& p) const
	{
		return hullVector(
			X() < p.X() ? X() : p.X(), 
			Y() < p.Y() ? Y() : p.Y(), 
			Z() < p.Z() ? Z() : p.Z(), 0.0);
	}

	hullVector GetMax(const hullVector& p) const
	{
		return hullVector(
			X() > p.X() ? X() : p.X(),
			Y() > p.Y() ? Y() : p.Y(),
			Z() > p.Z() ? Z() : p.Z(), 0.0);
	}

	hullVector Scale(double s) const
	{
		return hullVector(X() * s, Y() * s, Z() * s, 0.0);
	}

	inline hullVector operator+(const hullVector & rhs) const
	{
		return hullVector(X() + rhs.X(), Y() + rhs.Y(), Z() + rhs.Z(), 0.0f);
	}

	inline hullVector operator-(const hullVector & rhs) const
	{
		return hullVector(X() - rhs.X(), Y() - rhs.Y(), Z() - rhs.Z(), 0.0f);
	}

	inline hullVector operator*(const hullVector & rhs) const
	{
		return hullVector(X() * rhs.X(), Y() * rhs.Y(), Z() * rhs.Z(), 0.0f);
	}


	inline double DotProduct(const hullVector & rhs) const
	{
		return X() * rhs.X() + Y() * rhs.Y() + Z() * rhs.Z();
	}


	inline hullVector operator= (const Vec3 & rhs)
	{
		X() = rhs.X();
		Y() = rhs.Y();
		Z() = rhs.Z();
		return *this;
	}


};

class vhacdConvexHullFace
{
	public:
	vhacdConvexHullFace();
	
	//void SetMark(int mark) {m_mark = mark;}
	//int GetMark() const {return m_mark;}
	//ndList<vhacdConvexHullFace>::ndNode* GetTwin(int index) const { return m_twin[index];}
	//
	//private:
	//double Evalue (const hullVector* const pointArray, const hullVector& point) const;
	//ndBigPlane GetPlaneEquation (const hullVector* const pointArray) const;
	//
	//public:
	//int m_index[3]; 
	//private:
	//int m_mark;
	//ndList<vhacdConvexHullFace>::ndNode* m_twin[3];
	//friend class vhacdConvexHull;
};

//class vhacdConvexHull: public ndList<vhacdConvexHullFace>
class vhacdConvexHull: public std::list<vhacdConvexHullFace>
{
	class ndNormalMap;
	public:
	vhacdConvexHull(const vhacdConvexHull& source);
	vhacdConvexHull(const double* const vertexCloud, int strideInBytes, int count, double distTol, int maxVertexCount = 0x7fffffff);
	//virtual ~vhacdConvexHull();
	//
	//int GetVertexCount() const;
	const std::vector<hullVector>& GetVertexPool() const;
	//const hullVector& GetVertex(int i) const;
	//
	//double GetDiagonal() const;
	//void GetAABB (hullVector& boxP0, hullVector& boxP1) const;
	//double RayCast (const hullVector& localP0, const hullVector& localP1) const;
	//void CalculateVolumeAndSurfaceArea (double& volume, double& surcafeArea) const;
	//
	//protected:
	private:
	//vhacdConvexHull();
	void BuildHull (const double* const vertexCloud, int strideInBytes, int count, double distTol, int maxVertexCount);

	int GetUniquePoints(vhacdConvexHullVertex* const points, const double* const vertexCloud, int strideInBytes, int count, void* const memoryPool, int maxMemSize);
	int InitVertexArray(vhacdConvexHullVertex* const points, const double* const vertexCloud, int strideInBytes, int count, void* const memoryPool, int maxMemSize);
	vhacdConvexHullAABBTreeNode* BuildTree (vhacdConvexHullAABBTreeNode* const parent, vhacdConvexHullVertex* const points, int count, int baseIndex, char** const memoryPool, int& maxMemSize) const;

	//virtual ndNode* AddFace (int i0, int i1, int i2);
	//virtual void DeleteFace (ndNode* const node) ;
	
	
	//bool CheckFlatSurface(vhacdConvexHullAABBTreeNode* vertexTree, vhacdConvexHullVertex* const points, int count, double distTol, int maxVertexCount);
	//void CalculateConvexHull2d (vhacdConvexHullAABBTreeNode* vertexTree, vhacdConvexHullVertex* const points, int count, double distTol, int maxVertexCount);
	//void CalculateConvexHull3d (vhacdConvexHullAABBTreeNode* vertexTree, vhacdConvexHullVertex* const points, int count, double distTol, int maxVertexCount);
	//
	//int SupportVertex (vhacdConvexHullAABBTreeNode** const tree, const vhacdConvexHullVertex* const points, const hullVector& dir, const bool removeEntry = true) const;
	//double TetrahedrumVolume (const hullVector& p0, const hullVector& p1, const hullVector& p2, const hullVector& p3) const;

	
	
	////static int ConvexCompareVertex(const vhacdConvexHullVertex* const A, const vhacdConvexHullVertex* const B, void* const context);
	//bool Sanity() const;
	//void Save (const char* const filename) const;
	//
	hullVector m_aabbP0;
	hullVector m_aabbP1;
	int m_count;
	double m_diag;
	std::vector<hullVector> m_points;
};

//inline int vhacdConvexHull::GetVertexCount() const
//{
//	return m_count;
//}

inline const std::vector<hullVector>& vhacdConvexHull::GetVertexPool() const
{
	return m_points;
}

//inline const hullVector& vhacdConvexHull::GetVertex(int index) const
//{
//	return m_points[index];
//}
//
//inline double vhacdConvexHull::GetDiagonal() const
//{
//	return m_diag;
//}
//
//inline void vhacdConvexHull::GetAABB (hullVector& boxP0, hullVector& boxP1) const
//{
//	boxP0 = m_aabbP0;
//	boxP1 = m_aabbP1;
//}

#endif
