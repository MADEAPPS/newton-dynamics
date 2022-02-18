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

#include <vector>
#include "vhacdVector.h"

class vhacdConvexHullVertex;
class vhacdConvexHullAABBTreeNode;

template<class T>
class vhacdList
{
	public:
	class ndNode
	{
		ndNode(ndNode* const prev, ndNode* const next)
			:m_info()
			,m_next(next)
			,m_prev(prev)
		{
			if (m_prev)
			{
				m_prev->m_next = this;
			}
			if (m_next)
			{
				m_next->m_prev = this;
			}
		}

		ndNode(const T &info, ndNode* const prev, ndNode* const next)
			:m_info(info)
			,m_next(next)
			,m_prev(prev)
		{
			if (m_prev)
			{
				m_prev->m_next = this;
			}
			if (m_next)
			{
				m_next->m_prev = this;
			}
		}

		~ndNode()
		{
		}

		void Unlink()
		{
			if (m_prev)
			{
				m_prev->m_next = m_next;
			}
			if (m_next)
			{
				m_next->m_prev = m_prev;
			}
			m_prev = nullptr;
			m_next = nullptr;
		}

		void AddLast(ndNode* const node)
		{
			m_next = node;
			node->m_prev = this;
		}

		void AddFirst(ndNode* const node)
		{
			m_prev = node;
			node->m_next = this;
		}

		public:
		T& GetInfo()
		{
			return m_info;
		}

		ndNode *GetNext() const
		{
			return m_next;
		}

		ndNode *GetPrev() const
		{
			return m_prev;
		}

		private:
		T m_info;
		ndNode *m_next;
		ndNode *m_prev;
		friend class vhacdList<T>;
	};

	public:
	vhacdList()
		:m_first(nullptr)
		,m_last(nullptr)
		,m_count(0)
	{
	}

	~vhacdList()
	{
		RemoveAll();
	}

	void RemoveAll()
	{
		for (ndNode *node = m_first; node; node = m_first)
		{
			m_count--;
			m_first = node->GetNext();
			node->Unlink();
			delete node;
		}
		m_last = nullptr;
		m_first = nullptr;
	}

	ndNode* Append()
	{
		m_count++;
		if (m_first == nullptr)
		{
			m_first = new ndNode(nullptr, nullptr);
			m_last = m_first;
		}
		else
		{
			m_last = new ndNode(m_last, nullptr);
		}
		return m_last;
	}

	ndNode* Append(const T &element)
	{
		m_count++;
		if (m_first == nullptr)
		{
			m_first = new ndNode(element, nullptr, nullptr);
			m_last = m_first;
		}
		else
		{
			m_last = new ndNode(element, m_last, nullptr);
		}
		return m_last;
	}

	ndNode* Addtop(const T &element)
	{
		m_count++;
		if (m_last == nullptr)
		{
			m_last = new ndNode(element, nullptr, nullptr);
			m_first = m_last;
		}
		else
		{
			m_first = new ndNode(element, nullptr, m_first);
		}
		return m_first;
	}

	int GetCount() const
	{
		return m_count;
	}
	
	//operator int() const;
	
	ndNode* GetLast() const
	{
		return m_last;
	}

	ndNode* GetFirst() const
	{
		return m_first;
	}
		
	void Remove(ndNode* const node)
	{
		Unlink(node);
		delete node;
	}

	void Unlink(ndNode* const node)
	{
		m_count--;
		if (node == m_first)
		{
			m_first = m_first->GetNext();
		}
		if (node == m_last)
		{
			m_last = m_last->GetPrev();
		}
		node->Unlink();
	}

	void Remove(const T &element)
	{
		ndNode *const node = Find(element);
		if (node)
		{
			Remove(node);
		}
	}

	ndNode* Find(const T &element) const
	{
		ndNode *node;
		for (node = m_first; node; node = node->GetNext())
		{
			if (element == node->m_info)
			{
				break;
			}
		}
		return node;
	}

	private:
	ndNode* m_first;
	ndNode* m_last;
	int m_count;
	friend class ndNode;
};

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

	inline hullVector CrossProduct(const hullVector & rhs) const
	{
		return hullVector(Y() * rhs.Z() - Z() * rhs.Y(), Z() * rhs.X() - X() * rhs.Z(), X() * rhs.Y() - Y() * rhs.X(), 0.0);
	}

	inline hullVector operator= (const Vec3 & rhs)
	{
		X() = rhs.X();
		Y() = rhs.Y();
		Z() = rhs.Z();
		return *this;
	}
};

class hullPlane : public hullVector
{
	public:
	hullPlane(double x, double y, double z, double w)
		:hullVector(x, y, z, 0.0)
		, m_w(w)
	{
	}

	hullPlane(const hullVector &P0, const hullVector &P1, const hullVector &P2)
		:hullVector((P1 - P0).CrossProduct(P2 - P0))
	{
		m_w = -DotProduct(P0);
	}

	hullPlane Scale(double s) const
	{
		return hullPlane(X() * s, Y() * s, Z() * s, m_w * s);
	}

	inline hullPlane operator= (const hullPlane &rhs)
	{
		X() = rhs.X();
		Y() = rhs.Y();
		Z() = rhs.Z();
		m_w = rhs.m_w;
		return *this;
	}

	inline hullVector operator*(const hullVector & rhs) const
	{
		return hullVector(X() * rhs.X(), Y() * rhs.Y(), Z() * rhs.Z(), 0.0f);
	}

	double Evalue(const hullVector &point) const
	{
		return DotProduct(point) + m_w;
	}

	double m_w;
};


class vhacdConvexHullFace;

class vhacdConvexHullFace
{
	public:
	vhacdConvexHullFace();
	double Evalue (const hullVector* const pointArray, const hullVector& point) const;
	hullPlane GetPlaneEquation (const hullVector* const pointArray) const;
	
	public:
	int m_index[3]; 
	private:
	int m_mark;
	vhacdList<vhacdConvexHullFace>::ndNode* m_twin[3];
	
	friend class vhacdConvexHull;
};

class vhacdConvexHull: public vhacdList<vhacdConvexHullFace>
{
	class ndNormalMap;
	public:
	vhacdConvexHull(const vhacdConvexHull& source);
	vhacdConvexHull(const double* const vertexCloud, int strideInBytes, int count, double distTol, int maxVertexCount = 0x7fffffff);
	//~vhacdConvexHull();
		
	const std::vector<hullVector>& GetVertexPool() const;

	private:
	void BuildHull (const double* const vertexCloud, int strideInBytes, int count, double distTol, int maxVertexCount);

	int GetUniquePoints(vhacdConvexHullVertex* const points, const double* const vertexCloud, int strideInBytes, int count, void* const memoryPool, int maxMemSize);
	int InitVertexArray(vhacdConvexHullVertex* const points, const double* const vertexCloud, int strideInBytes, int count, void* const memoryPool, int maxMemSize);
	vhacdConvexHullAABBTreeNode* BuildTree (vhacdConvexHullAABBTreeNode* const parent, vhacdConvexHullVertex* const points, int count, int baseIndex, char** const memoryPool, int& maxMemSize) const;

	ndNode* AddFace (int i0, int i1, int i2);
	
	void CalculateConvexHull3d (vhacdConvexHullAABBTreeNode* vertexTree, vhacdConvexHullVertex* const points, int count, double distTol, int maxVertexCount);
	
	int SupportVertex (vhacdConvexHullAABBTreeNode** const tree, const vhacdConvexHullVertex* const points, const hullVector& dir, const bool removeEntry = true) const;
	double TetrahedrumVolume (const hullVector& p0, const hullVector& p1, const hullVector& p2, const hullVector& p3) const;

	hullVector m_aabbP0;
	hullVector m_aabbP1;
	double m_diag;
	std::vector<hullVector> m_points;
};

inline const std::vector<hullVector>& vhacdConvexHull::GetVertexPool() const
{
	return m_points;
}
#endif
