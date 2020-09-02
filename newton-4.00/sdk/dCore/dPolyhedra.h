/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_POLYHEDRA_H__
#define __D_POLYHEDRA_H__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dList.h"
#include "dTree.h"
#include "dHeap.h"
#include "dHeap.h"
#include "dDebug.h"
#include "dClassAlloc.h"

class dEdge;
//class dgObb;
class dMatrix;
class dBigPlane;
class dBigVector;
class dPolyhedra;
class dVertexCollapseVertexMetric;

typedef dInt64 dEdgeKey;

class dEdge
{
	public:
	dEdge ();
	dEdge (dInt32 vertex, dInt32 face, dUnsigned64 userdata = 0);
	~dEdge ();

	dInt32 m_incidentVertex;
	dInt32 m_incidentFace;
	dUnsigned64 m_userData;
	dEdge* m_next;
	dEdge* m_prev;
	dEdge* m_twin;
	dInt32 m_mark;
} D_GCC_NEWTON_ALIGN_32 ;

class dPolyhedra: public dClassAlloc, public dTree <dEdge, dEdgeKey>
{
	public:
	class dgPairKey
	{
		public:
		dgPairKey()
		{
		}

		dgPairKey(dInt64 key)
			:m_key(dUnsigned64(key))
		{
		}

		dgPairKey(dInt32 keyHigh, dInt32 keyLow)
			:m_keyLow(dUnsigned32 (keyLow))
			,m_keyHigh(dUnsigned32 (keyHigh))
		{
		}

		dInt64 GetVal() const
		{
			return dInt64(m_key);
		}

		dInt32 GetLowKey() const
		{
			return dInt32(m_keyLow);
		}

		dInt32 GetHighKey() const
		{
			return dInt32(m_keyHigh);
		}

		bool operator<(const dgPairKey& key) const
		{
			return m_key < key.m_key;
		}

		bool operator>(const dgPairKey& key) const
		{
			return m_key > key.m_key;
		}


		private:
		union 
		{
			dUnsigned64 m_key;
			struct 
			{
				dUnsigned32 m_keyLow;
				dUnsigned32 m_keyHigh;
			};
		};
	};

	dPolyhedra ();
	D_CORE_API dPolyhedra (const dPolyhedra &polyhedra);
	D_CORE_API virtual ~dPolyhedra();

	virtual bool ReportProgress(dFloat32 percentProgress) const { return true;}

	virtual void BeginFace();
	dEdge* AddFace (dInt32 v0, dInt32 v1, dInt32 v2);
	dEdge* AddFace (dInt32 count, const dInt32* const index);
	D_CORE_API dEdge* AddFace (dInt32 count, const dInt32* const index, const dInt64* const userdata);
	D_CORE_API virtual bool EndFace ();
	D_CORE_API virtual void DeleteFace(dEdge* const edge);

	D_CORE_API dInt32 GetFaceCount() const;
	dInt32 GetEdgeCount() const;
	dInt32 GetLastVertexIndex() const;

	dInt32 IncLRU() const;
	dInt32 GetLRU() const;
	void SetLRU(dInt32 lru) const;

	dEdge* FindEdge (dInt32 v0, dInt32 v1) const;
	dTreeNode* FindEdgeNode (dInt32 v0, dInt32 v1) const;

	D_CORE_API dEdge* AddHalfEdge (dInt32 v0, dInt32 v1);
	D_CORE_API void DeleteEdge (dEdge* const edge);
	void DeleteEdge (dInt32 v0, dInt32 v1);

	D_CORE_API dEdge* ConnectVertex (dEdge* const e0, dEdge* const e1);
	
	D_CORE_API bool FlipEdge (dEdge* const edge);
	D_CORE_API dEdge* SpliteEdge (dInt32 newIndex, dEdge* const edge);
	D_CORE_API dBigVector FaceNormal (const dEdge* const face, const dFloat64* const vertex, dInt32 strideInBytes) const;

	D_CORE_API void SavePLY(const char* const fileName, const dFloat64* const vertex, dInt32 strideInBytes) const;

	void BeginConectedSurface() const;
	D_CORE_API bool GetConectedSurface (dPolyhedra &polyhedra) const;
	void EndConectedSurface() const;

//	dgObb CalculateSphere (const dFloat64* const vertex, dInt32 strideInBytes, const dMatrix* const basis = nullptr) const;
	D_CORE_API void ChangeEdgeIncidentVertex (dEdge* const edge, dInt32 newIndex);
	D_CORE_API void DeleteDegenerateFaces (const dFloat64* const pool, dInt32 dstStrideInBytes, dFloat64 minArea);

	D_CORE_API bool Optimize (const dFloat64* const pool, dInt32 strideInBytes, dFloat64 tol, dInt32 maxFaceCount = 1<<28);
	D_CORE_API void Triangulate (const dFloat64* const vertex, dInt32 strideInBytes, dPolyhedra* const leftOversOut);
	D_CORE_API void ConvexPartition (const dFloat64* const vertex, dInt32 strideInBytes, dPolyhedra* const leftOversOut);
	D_CORE_API bool IsFaceConvex(dEdge* const face, const dFloat64* const pool, dInt32 strideInBytes) const;

	protected:
	D_CORE_API dEdge* CollapseEdge(dEdge* const edge);
	D_CORE_API bool PolygonizeFace(dEdge* const face, const dFloat64* const pool, dInt32 stride);
	D_CORE_API bool TriangulateFace(dEdge* const face, const dFloat64* const pool, dInt32 stride);

	private:
	void RefineTriangulation (const dFloat64* const vertex, dInt32 stride);
	void RefineTriangulation (const dFloat64* const vertex, dInt32 stride, const dBigVector& normal, dInt32 perimeterCount, dEdge** const perimeter);
	void OptimizeTriangulation (const dFloat64* const vertex, dInt32 strideInBytes);
	void RemoveInteriorEdges (dPolyhedra& polyhedraOut, const dFloat64* const vertex, dInt32 strideInBytes);
	void MarkAdjacentCoplanarFaces (dPolyhedra& polyhedraOut, dEdge* const face, const dFloat64* const pool, dInt32 strideInBytes);
	dEdge* FindEarTip (dEdge* const face, const dFloat64* const pool, dInt32 stride, dDownHeap<dEdge*, dFloat64>& heap, const dBigVector &normal) const;
	dEdge* TriangulateFace (dEdge* const face, const dFloat64* const pool, dInt32 stride, dDownHeap<dEdge*, dFloat64>& heap, dBigVector* const faceNormalOut);
		
	void RemoveHalfEdge (dEdge* const edge);
	dEdge* OptimizeCollapseEdge (dEdge* const edge);
	bool IsOkToCollapse (const dBigVector* const pool, dEdge* const edge) const;
	dFloat64 EdgePenalty (const dBigVector* const pool, dEdge* const edge, dFloat64 dist) const;
	dBigPlane EdgePlane (dInt32 i0, dInt32 i1, dInt32 i2, const dBigVector* const pool) const;
	void CalculateAllMetrics (dVertexCollapseVertexMetric* const table, const dBigVector* const pool) const;
	void CalculateVertexMetrics (dVertexCollapseVertexMetric* const table, const dBigVector* const pool, dEdge* const edge) const;
	dEdge* BestEdgePolygonizeFace(const dBigVector& normal, dEdge* const edge, const dFloat64* const pool, dInt32 stride, const dBigVector& point) const;

	static dInt32 GetInteriorDiagonals (dPolyhedra& polyhedra, dEdge** const diagonals, dInt32 maxCount);
	static dBigPlane UnboundedLoopPlane (dInt32 i0, dInt32 i1, dInt32 i2, const dBigVector* const pool);
	static void RemoveOuterColinearEdges(dPolyhedra& flatFace, const dFloat64* const vertex, dInt32 stride);
	static void RemoveInteriorColinearEdges(dPolyhedra& flatFace, const dFloat64* const vertex, dInt32 stride);
	static bool IsEssensialDiagonal (dEdge* const diagonal, const dBigVector& normal, const dFloat64* const pool,  dInt32 stride);
	static bool IsEssensialPointDiagonal (dEdge* const diagonal, const dBigVector& normal, const dFloat64* const pool, dInt32 stride);
	
	mutable dInt32 m_baseMark;
	mutable dInt32 m_edgeMark;
	mutable dInt32 m_faceSecuence;
	friend class dPolyhedraDescriptor;
};

inline dEdge::dEdge ()
{
}

inline dEdge::dEdge (dInt32 vertex, dInt32 face, dUnsigned64 userdata)
	:m_incidentVertex(vertex)
	,m_incidentFace(face)
	,m_userData(userdata)
	,m_next(nullptr)
	,m_prev(nullptr)
	,m_twin(nullptr)
	,m_mark(0)
{
}

inline dEdge::~dEdge ()
{
}

inline void dPolyhedra::BeginFace ()
{
}

inline dEdge* dPolyhedra::AddFace (dInt32 count, const dInt32* const index) 
{
	return AddFace (count, index, nullptr);
}

inline dEdge* dPolyhedra::AddFace (dInt32 v0, dInt32 v1, dInt32 v2)
{
	dInt32 vertex[3];

	vertex [0] = v0;
	vertex [1] = v1;
	vertex [2] = v2;
	return AddFace (3, vertex, nullptr);
}

inline dInt32 dPolyhedra::GetEdgeCount() const
{
#ifdef _DEBUG
	dInt32 edgeCount = 0;
	Iterator iter(*this);
	for (iter.Begin(); iter; iter ++) 
	{
		edgeCount ++;
	}
	dAssert (edgeCount == GetCount());;
#endif
	return GetCount();
}

inline dPolyhedra::dPolyhedra()
	:dClassAlloc()
	,dTree <dEdge, dInt64>()
	,m_baseMark(0)
	,m_edgeMark(0)
	,m_faceSecuence(0)
{
}

inline dInt32 dPolyhedra::GetLastVertexIndex() const
{
	dInt32 maxVertexIndex = -1;
	Iterator iter(*this);
	for (iter.Begin(); iter; iter ++) 
	{
		const dEdge* const edge = &(*iter);
		if (edge->m_incidentVertex > maxVertexIndex) 
		{
			maxVertexIndex = edge->m_incidentVertex;
		}
	}
	return maxVertexIndex + 1;
}

inline dInt32 dPolyhedra::IncLRU() const
{	
	m_edgeMark ++;
	dAssert (m_edgeMark < 0x7fffffff);
	return m_edgeMark;
}

inline dInt32 dPolyhedra::GetLRU() const
{
	return m_edgeMark;
}

inline void dPolyhedra::SetLRU(dInt32 lru) const
{
	if (lru > m_edgeMark) 
	{
		m_edgeMark = lru;
	}
}

inline void dPolyhedra::BeginConectedSurface() const
{
	m_baseMark = IncLRU();
}

inline void dPolyhedra::EndConectedSurface() const
{
}

inline dPolyhedra::dTreeNode* dPolyhedra::FindEdgeNode (dInt32 i0, dInt32 i1) const
{
	dgPairKey key (i0, i1);
	return Find (key.GetVal());
}

inline dEdge *dPolyhedra::FindEdge (dInt32 i0, dInt32 i1) const
{
	dTreeNode* const node = FindEdgeNode (i0, i1);
	return node ? &node->GetInfo() : nullptr;
}

inline void dPolyhedra::DeleteEdge (dInt32 v0, dInt32 v1)
{
	dgPairKey pairKey (v0, v1);
	dTreeNode* const node = Find(pairKey.GetVal());
	dEdge* const edge = node ? &node->GetInfo() : nullptr;
	if (!edge) 
	{
		return;
	}
	DeleteEdge (edge);
}

#endif

