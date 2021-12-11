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

#ifndef __ND_POLYHEDRA_H__
#define __ND_POLYHEDRA_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndList.h"
#include "ndTree.h"
#include "ndHeap.h"
#include "ndHeap.h"
#include "ndDebug.h"
#include "ndClassAlloc.h"

class ndEdge;
class ndMatrix;
class ndBigPlane;
class ndBigVector;
class ndPolyhedra;
class ndVertexCollapseVertexMetric;

typedef dInt64 ndEdgeKey;

class ndEdge
{
	public:
	ndEdge ();
	ndEdge (dInt32 vertex, dInt32 face, dUnsigned64 userdata = 0);
	~ndEdge ();

	dInt32 m_incidentVertex;
	dInt32 m_incidentFace;
	dUnsigned64 m_userData;
	ndEdge* m_next;
	ndEdge* m_prev;
	ndEdge* m_twin;
	dInt32 m_mark;
} D_GCC_NEWTON_ALIGN_32 ;

class ndPolyhedra: public ndTree <ndEdge, ndEdgeKey>
{
	public:
	class ndPairKey
	{
		public:
		ndPairKey()
		{
		}

		ndPairKey(dInt64 key)
			:m_key(dUnsigned64(key))
		{
		}

		ndPairKey(dInt32 keyHigh, dInt32 keyLow)
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

		bool operator<(const ndPairKey& key) const
		{
			return m_key < key.m_key;
		}

		bool operator>(const ndPairKey& key) const
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

	ndPolyhedra ();
	D_CORE_API ndPolyhedra (const ndPolyhedra &polyhedra);
	D_CORE_API virtual ~ndPolyhedra();

	virtual bool ReportProgress(dFloat32) const { return true;}

	virtual void BeginFace();
	ndEdge* AddFace (dInt32 v0, dInt32 v1, dInt32 v2);
	ndEdge* AddFace (dInt32 count, const dInt32* const index);
	D_CORE_API ndEdge* AddFace (dInt32 count, const dInt32* const index, const dInt64* const userdata);
	D_CORE_API virtual bool EndFace ();
	D_CORE_API virtual void DeleteFace(ndEdge* const edge);

	D_CORE_API dInt32 GetFaceCount() const;
	dInt32 GetEdgeCount() const;
	dInt32 GetLastVertexIndex() const;

	dInt32 IncLRU() const;
	dInt32 GetLRU() const;
	void SetLRU(dInt32 lru) const;

	ndEdge* FindEdge (dInt32 v0, dInt32 v1) const;
	ndNode* FindEdgeNode (dInt32 v0, dInt32 v1) const;

	D_CORE_API ndEdge* AddHalfEdge (dInt32 v0, dInt32 v1);
	D_CORE_API void DeleteEdge (ndEdge* const edge);
	void DeleteEdge (dInt32 v0, dInt32 v1);

	D_CORE_API ndEdge* ConnectVertex (ndEdge* const e0, ndEdge* const e1);
	
	D_CORE_API bool FlipEdge (ndEdge* const edge);
	D_CORE_API ndEdge* SpliteEdge (dInt32 newIndex, ndEdge* const edge);
	D_CORE_API ndBigVector FaceNormal (const ndEdge* const face, const dFloat64* const vertex, dInt32 strideInBytes) const;

	D_CORE_API void SavePLY(const char* const fileName, const dFloat64* const vertex, dInt32 strideInBytes) const;

	void BeginConectedSurface() const;
	D_CORE_API bool GetConectedSurface (ndPolyhedra &polyhedra) const;
	void EndConectedSurface() const;

	D_CORE_API ndMatrix CalculateSphere(ndBigVector& size, const dFloat64* const vertex, dInt32 strideInBytes) const;
	
	D_CORE_API void ChangeEdgeIncidentVertex (ndEdge* const edge, dInt32 newIndex);
	D_CORE_API void DeleteDegenerateFaces (const dFloat64* const pool, dInt32 dstStrideInBytes, dFloat64 minArea);

	D_CORE_API bool Optimize (const dFloat64* const pool, dInt32 strideInBytes, dFloat64 tol, dInt32 maxFaceCount = 1<<28);
	D_CORE_API void Triangulate (const dFloat64* const vertex, dInt32 strideInBytes, ndPolyhedra* const leftOversOut);
	D_CORE_API void ConvexPartition (const dFloat64* const vertex, dInt32 strideInBytes, ndPolyhedra* const leftOversOut);
	D_CORE_API bool IsFaceConvex(ndEdge* const face, const dFloat64* const pool, dInt32 strideInBytes) const;

	protected:
	D_CORE_API ndEdge* CollapseEdge(ndEdge* const edge);
	D_CORE_API bool PolygonizeFace(ndEdge* const face, const dFloat64* const pool, dInt32 stride);
	D_CORE_API bool TriangulateFace(ndEdge* const face, const dFloat64* const pool, dInt32 stride);

	private:
	void RefineTriangulation (const dFloat64* const vertex, dInt32 stride);
	void RefineTriangulation (const dFloat64* const vertex, dInt32 stride, const ndBigVector& normal, dInt32 perimeterCount, ndEdge** const perimeter);
	void OptimizeTriangulation (const dFloat64* const vertex, dInt32 strideInBytes);
	void RemoveInteriorEdges (ndPolyhedra& polyhedraOut, const dFloat64* const vertex, dInt32 strideInBytes);
	void MarkAdjacentCoplanarFaces (ndPolyhedra& polyhedraOut, ndEdge* const face, const dFloat64* const pool, dInt32 strideInBytes);
	ndEdge* FindEarTip (ndEdge* const face, const dFloat64* const pool, dInt32 stride, ndDownHeap<ndEdge*, dFloat64>& heap, const ndBigVector &normal) const;
	ndEdge* TriangulateFace (ndEdge* const face, const dFloat64* const pool, dInt32 stride, ndDownHeap<ndEdge*, dFloat64>& heap, ndBigVector* const faceNormalOut);
		
	void RemoveHalfEdge (ndEdge* const edge);
	ndEdge* OptimizeCollapseEdge (ndEdge* const edge);
	bool IsOkToCollapse (const ndBigVector* const pool, ndEdge* const edge) const;
	dFloat64 EdgePenalty (const ndBigVector* const pool, ndEdge* const edge, dFloat64 dist) const;
	ndBigPlane EdgePlane (dInt32 i0, dInt32 i1, dInt32 i2, const ndBigVector* const pool) const;
	void CalculateAllMetrics (ndVertexCollapseVertexMetric* const table, const ndBigVector* const pool) const;
	void CalculateVertexMetrics (ndVertexCollapseVertexMetric* const table, const ndBigVector* const pool, ndEdge* const edge) const;
	ndEdge* BestEdgePolygonizeFace(const ndBigVector& normal, ndEdge* const edge, const dFloat64* const pool, dInt32 stride, const ndBigVector& point) const;

	static dInt32 GetInteriorDiagonals (ndPolyhedra& polyhedra, ndEdge** const diagonals, dInt32 maxCount);
	static ndBigPlane UnboundedLoopPlane (dInt32 i0, dInt32 i1, dInt32 i2, const ndBigVector* const pool);
	static void RemoveOuterColinearEdges(ndPolyhedra& flatFace, const dFloat64* const vertex, dInt32 stride);
	static void RemoveInteriorColinearEdges(ndPolyhedra& flatFace, const dFloat64* const vertex, dInt32 stride);
	static bool IsEssensialDiagonal (ndEdge* const diagonal, const ndBigVector& normal, const dFloat64* const pool,  dInt32 stride);
	static bool IsEssensialPointDiagonal (ndEdge* const diagonal, const ndBigVector& normal, const dFloat64* const pool, dInt32 stride);
	
	mutable dInt32 m_baseMark;
	mutable dInt32 m_edgeMark;
	mutable dInt32 m_faceSecuence;
	friend class dPolyhedraDescriptor;
};

inline ndEdge::ndEdge ()
{
}

inline ndEdge::ndEdge (dInt32 vertex, dInt32 face, dUnsigned64 userdata)
	:m_incidentVertex(vertex)
	,m_incidentFace(face)
	,m_userData(userdata)
	,m_next(nullptr)
	,m_prev(nullptr)
	,m_twin(nullptr)
	,m_mark(0)
{
}

inline ndEdge::~ndEdge ()
{
}

inline void ndPolyhedra::BeginFace ()
{
}

inline ndEdge* ndPolyhedra::AddFace (dInt32 count, const dInt32* const index) 
{
	return AddFace (count, index, nullptr);
}

inline ndEdge* ndPolyhedra::AddFace (dInt32 v0, dInt32 v1, dInt32 v2)
{
	dInt32 vertex[3];

	vertex [0] = v0;
	vertex [1] = v1;
	vertex [2] = v2;
	return AddFace (3, vertex, nullptr);
}

inline dInt32 ndPolyhedra::GetEdgeCount() const
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

inline ndPolyhedra::ndPolyhedra()
	:ndTree <ndEdge, dInt64>()
	,m_baseMark(0)
	,m_edgeMark(0)
	,m_faceSecuence(0)
{
}

inline dInt32 ndPolyhedra::GetLastVertexIndex() const
{
	dInt32 maxVertexIndex = -1;
	Iterator iter(*this);
	for (iter.Begin(); iter; iter ++) 
	{
		const ndEdge* const edge = &(*iter);
		if (edge->m_incidentVertex > maxVertexIndex) 
		{
			maxVertexIndex = edge->m_incidentVertex;
		}
	}
	return maxVertexIndex + 1;
}

inline dInt32 ndPolyhedra::IncLRU() const
{	
	m_edgeMark ++;
	dAssert (m_edgeMark < 0x7fffffff);
	return m_edgeMark;
}

inline dInt32 ndPolyhedra::GetLRU() const
{
	return m_edgeMark;
}

inline void ndPolyhedra::SetLRU(dInt32 lru) const
{
	if (lru > m_edgeMark) 
	{
		m_edgeMark = lru;
	}
}

inline void ndPolyhedra::BeginConectedSurface() const
{
	m_baseMark = IncLRU();
}

inline void ndPolyhedra::EndConectedSurface() const
{
}

inline ndPolyhedra::ndNode* ndPolyhedra::FindEdgeNode (dInt32 i0, dInt32 i1) const
{
	ndPairKey key (i0, i1);
	return Find (key.GetVal());
}

inline ndEdge *ndPolyhedra::FindEdge (dInt32 i0, dInt32 i1) const
{
	ndNode* const node = FindEdgeNode (i0, i1);
	return node ? &node->GetInfo() : nullptr;
}

inline void ndPolyhedra::DeleteEdge (dInt32 v0, dInt32 v1)
{
	ndPairKey pairKey (v0, v1);
	ndNode* const node = Find(pairKey.GetVal());
	ndEdge* const edge = node ? &node->GetInfo() : nullptr;
	if (!edge) 
	{
		return;
	}
	DeleteEdge (edge);
}

#endif

