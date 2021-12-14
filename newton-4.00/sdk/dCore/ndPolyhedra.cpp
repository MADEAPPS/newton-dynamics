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
#include "ndHeap.h"
#include "ndPlane.h"
#include "ndDebug.h"
#include "ndStack.h"
#include "ndVector.h"
#include "ndMatrix.h"
#include "ndPolyhedra.h"
#include "ndConvexHull3d.h"
#include "ndSmallDeterminant.h"

#define D_LOCAL_BUFFER_SIZE  1024

#define dPointerToInt(x) ((size_t)x)
#define dIntToPointer(x) ((void*)(size_t(x)))


class dgDiagonalEdge
{
	public:
	dgDiagonalEdge (ndEdge* const edge)
		:m_i0(edge->m_incidentVertex), m_i1(edge->m_twin->m_incidentVertex)
	{
	}
	ndInt32 m_i0;
	ndInt32 m_i1;
};

struct dEdgeCollapseEdgeHandle
{
	dEdgeCollapseEdgeHandle (ndEdge* const newEdge)
		:m_edge(newEdge)
		,m_inList(0)
	{
	}

	dEdgeCollapseEdgeHandle (const dEdgeCollapseEdgeHandle &dataHandle)
		:m_edge(dataHandle.m_edge)
		,m_inList(1)
	{
		dEdgeCollapseEdgeHandle* const handle = (dEdgeCollapseEdgeHandle *)dIntToPointer (m_edge->m_userData);
		if (handle) 
		{
			dAssert (handle != this);
			handle->m_edge = nullptr;
		}
		m_edge->m_userData = ndUnsigned64 (dPointerToInt(this));
	}

	~dEdgeCollapseEdgeHandle ()
	{
		if (m_inList) 
		{
			if (m_edge) 
			{
				dEdgeCollapseEdgeHandle* const handle = (dEdgeCollapseEdgeHandle *)dIntToPointer (m_edge->m_userData);
				if (handle == this) 
				{
					m_edge->m_userData = dPointerToInt (nullptr);
				}
			}
		}
		m_edge = nullptr;
	}

	ndEdge* m_edge;
	ndUnsigned32 m_inList;
};

class ndVertexCollapseVertexMetric
{
	public:
	ndVertexCollapseVertexMetric (const ndBigPlane &plane) 
	{
		elem[0] = plane.m_x * plane.m_x;  
		elem[1] = plane.m_y * plane.m_y;  
		elem[2] = plane.m_z * plane.m_z;  
		elem[3] = plane.m_w * plane.m_w;  
		elem[4] = ndFloat64 (2.0) * plane.m_x * plane.m_y;  
		elem[5] = ndFloat64 (2.0) * plane.m_x * plane.m_z;  
		elem[6] = ndFloat64 (2.0) * plane.m_x * plane.m_w;  
		elem[7] = ndFloat64 (2.0) * plane.m_y * plane.m_z;  
		elem[8] = ndFloat64 (2.0) * plane.m_y * plane.m_w;  
		elem[9] = ndFloat64 (2.0) * plane.m_z * plane.m_w;  
	}

	void Clear ()
	{
		memset (elem, 0, 10 * sizeof (ndFloat64));
	}

	void Accumulate (const ndVertexCollapseVertexMetric& p) 
	{
		elem[0] += p.elem[0]; 
		elem[1] += p.elem[1]; 
		elem[2] += p.elem[2]; 
		elem[3] += p.elem[3]; 
		elem[4] += p.elem[4]; 
		elem[5] += p.elem[5]; 
		elem[6] += p.elem[6]; 
		elem[7] += p.elem[7]; 
		elem[8] += p.elem[8]; 
		elem[9] += p.elem[9]; 
	}

	void Accumulate (const ndBigPlane& plane) 
	{
		elem[0] += plane.m_x * plane.m_x;  
		elem[1] += plane.m_y * plane.m_y;  
		elem[2] += plane.m_z * plane.m_z;  
		elem[3] += plane.m_w * plane.m_w;  

		elem[4] += ndFloat64 (2.0f) * plane.m_x * plane.m_y;  
		elem[5] += ndFloat64 (2.0f) * plane.m_x * plane.m_z;  
		elem[7] += ndFloat64 (2.0f) * plane.m_y * plane.m_z;  

		elem[6] += ndFloat64 (2.0f) * plane.m_x * plane.m_w;  
		elem[8] += ndFloat64 (2.0f) * plane.m_y * plane.m_w;  
		elem[9] += ndFloat64 (2.0f) * plane.m_z * plane.m_w;  
	}

	ndFloat64 Evalue (const ndBigVector &p) const 
	{
		ndFloat64 acc = elem[0] * p.m_x * p.m_x + elem[1] * p.m_y * p.m_y + elem[2] * p.m_z * p.m_z + 
					   elem[4] * p.m_x * p.m_y + elem[5] * p.m_x * p.m_z + elem[7] * p.m_y * p.m_z + 
					   elem[6] * p.m_x + elem[8] * p.m_y + elem[9] * p.m_z + elem[3];  
		return fabs (acc);
	}

	ndFloat64 elem[10];
};

ndPolyhedra::ndPolyhedra (const ndPolyhedra &polyhedra)
	:ndTree <ndEdge, ndInt64>()
	,m_baseMark(0)
	,m_edgeMark(0)
	,m_faceSecuence(0)
{
	ndStack<ndInt32> indexPool (D_LOCAL_BUFFER_SIZE * 16);
	ndStack<ndUnsigned64> userPool (D_LOCAL_BUFFER_SIZE * 16);
	ndInt32* const index = &indexPool[0];
	ndUnsigned64* const user = &userPool[0];

	BeginFace ();
	Iterator iter(polyhedra);
	for (iter.Begin(); iter; iter ++) {
		ndEdge* const edge = &(*iter);
		if (edge->m_incidentFace < 0) {
			continue;
		}

		if (!FindEdge(edge->m_incidentVertex, edge->m_twin->m_incidentVertex))	{
			ndInt32 indexCount = 0;
			ndEdge* ptr = edge;
			do {
				user[indexCount] = ptr->m_userData;
				index[indexCount] = ptr->m_incidentVertex;
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != edge);

			ndEdge* const face = AddFace (indexCount, index, (ndInt64*) user);
			if (face) {
				ptr = face;
				do {
					ptr->m_incidentFace = edge->m_incidentFace;
					ptr = ptr->m_next;
				} while (ptr != face);
			}
		}
	}
	EndFace();

	m_faceSecuence = polyhedra.m_faceSecuence;

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
	dAssert (SanityCheck());
#endif
}

ndPolyhedra::~ndPolyhedra ()
{
}

void ndPolyhedra::SavePLY(const char* const fileName, const ndFloat64* const vertexArray, ndInt32 strideInBytes) const
{
	FILE* const file = fopen(fileName, "wb");

	fprintf(file, "ply\n");
	fprintf(file, "format ascii 1.0\n");

	ndPolyhedra copy(*this);

	ndInt32 faceCount = 0;
	Iterator iter(copy);
	ndInt32 mark = copy.IncLRU();
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const face = &iter.GetNode()->GetInfo();
		if ((face->m_mark < mark) && (face->m_incidentFace > 0)) 
		{
			faceCount++;
			ndEdge* edge = face;
			do 
			{
				edge->m_mark = mark;
				edge = edge->m_next;
			} while (edge != face);
		}
	}

	mark = copy.IncLRU();
	ndInt32 vertexCount = 0;
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const vertex = &iter.GetNode()->GetInfo();
		if (vertex->m_mark < mark) 
		{
			ndEdge* edge = vertex;
			do 
			{
				edge->m_userData = vertexCount;
				edge->m_mark = mark;
				edge = edge->m_twin->m_next;
			} while (edge != vertex);
			vertexCount++;
		}
	}

	fprintf(file, "element vertex %d\n", vertexCount);
	fprintf(file, "property float x\n");
	fprintf(file, "property float y\n");
	fprintf(file, "property float z\n");
	fprintf(file, "element face %d\n", faceCount);
	fprintf(file, "property list uchar int vertex_index\n");
	fprintf(file, "end_header\n");

	mark = copy.IncLRU();
	const ndInt8* const points = (ndInt8*)vertexArray;
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const vertex = &iter.GetNode()->GetInfo();
		if (vertex->m_mark < mark) 
		{
			ndEdge* edge = vertex;
			do 
			{
				edge->m_mark = mark;
				edge = edge->m_twin->m_next;
			} while (edge != vertex);
			ndInt32 index = edge->m_incidentVertex * strideInBytes;

			const ndFloat64* const p = (ndFloat64*)&points[index];
			ndBigVector point(p[0], p[1], p[2], ndFloat64(0.0f));
			fprintf(file, "%f %f %f\n", point.m_x, point.m_y, point.m_z);
		}
	}

	mark = copy.IncLRU();
	for (iter.Begin(); iter; iter++) 
	{
		ndInt32 indices[1024];
		ndInt32 count = 0;
		ndEdge* const face = &iter.GetNode()->GetInfo();
		if ((face->m_mark < mark) && (face->m_incidentFace > 0)) 
		{
			ndEdge* edge = face;
			do 
			{
				indices[count] = ndInt32 (edge->m_userData);
				count++;
				edge->m_mark = mark;
				edge = edge->m_next;
			} while (edge != face);

			fprintf(file, "%d", count);
			for (ndInt32 j = 0; j < count; j++) 
			{
				fprintf(file, " %d", indices[j]);
			}
			fprintf(file, "\n");
		}
	}
	fclose(file);
}

ndInt32 ndPolyhedra::GetFaceCount() const
{
	Iterator iter (*this);
	ndInt32 count = 0;
	ndInt32 mark = IncLRU();
	for (iter.Begin(); iter; iter ++) {
		ndEdge* const edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}

		if (edge->m_incidentFace < 0) {
			continue;
		}

		count ++;
		ndEdge* ptr = edge;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);
	}
	return count;
}

ndEdge* ndPolyhedra::AddFace (ndInt32 count, const ndInt32* const index, const ndInt64* const userdata)
{
	class IntersectionFilter
	{
		public:
		IntersectionFilter ()
		{
			m_count = 0;
		}

		bool Insert (ndInt64 value)
		{
			ndInt32 i = 0;				
			for (; i < m_count; i ++) 
			{
				if (m_array[i] == value) 
				{
					return false;
				}
			}
			m_array[i] = value;
			m_count ++;
			return true;
		}

		ndInt32 m_count;
		ndInt64 m_array[2048];
	};

	IntersectionFilter selfIntersectingFaceFilter;

	ndInt32 i0 = index[count-1];
	for (ndInt32 i = 0; i < count; i ++) 
	{
		ndInt32 i1 = index[i];
		ndPairKey code0 (i0, i1);
		if (!selfIntersectingFaceFilter.Insert (code0.GetVal())) 
		{
			return nullptr;
		}

		ndPairKey code1 (i1, i0);
		if (!selfIntersectingFaceFilter.Insert (code1.GetVal())) 
		{
			return nullptr;
		}

		if (i0 == i1) 
		{
			return nullptr;
		}
		if (FindEdge (i0, i1)) 
		{
			return nullptr;
		}
		i0 = i1;
	}

	m_faceSecuence ++;

	i0 = index[count-1];
	ndInt32 i1 = index[0];
	ndUnsigned64 udata0 = 0;
	ndUnsigned64 udata1 = 0;
	if (userdata) 
	{
		udata0 = ndUnsigned64 (userdata[count-1]);
		udata1 = ndUnsigned64 (userdata[0]);
	} 

	bool state;
	ndPairKey code (i0, i1);
	ndEdge tmpEdge (i0, m_faceSecuence, udata0);
	ndNode* const node = Insert (tmpEdge, code.GetVal(), state); 
	dAssert (!state);
	ndEdge* edge0 = &node->GetInfo();
	ndEdge* const first = edge0;

	for (ndInt32 i = 1; i < count; i ++) 
	{
		i0 = i1;
		i1 = index[i];
		udata0 = udata1;
		udata1 = ndUnsigned64 (userdata ? userdata[i] : 0);

		ndPairKey code1 (i0, i1);
		ndEdge tmpEdge1 (i0, m_faceSecuence, udata0);
		ndNode* const node1 = Insert (tmpEdge1, code1.GetVal(), state); 
		dAssert (!state);

		ndEdge* const edge1 = &node1->GetInfo();
		edge0->m_next = edge1;
		edge1->m_prev = edge0;
		edge0 = edge1;
	}

	first->m_prev = edge0;
	edge0->m_next = first;

	return first->m_next;
}

bool ndPolyhedra::EndFace ()
{
	ndPolyhedra::Iterator iter (*this);

	// Connect all twin edge
	for (iter.Begin(); iter; iter ++) 
	{
		ndEdge* const edge = &(*iter);
		if (!edge->m_twin) 
		{
			edge->m_twin = FindEdge (edge->m_next->m_incidentVertex, edge->m_incidentVertex);
			if (edge->m_twin) 
			{
				edge->m_twin->m_twin = edge; 
			}
		}
	}

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
	dAssert (SanityCheck());
#endif
	ndStack<ndEdge*> edgeArrayPool(GetCount() * 2 + 256);

	ndInt32 edgeCount = 0;
	ndEdge** const edgeArray = &edgeArrayPool[0];
	for (iter.Begin(); iter; iter ++) 
	{
		ndEdge* const edge = &(*iter);
		if (!edge->m_twin) 
		{
			bool state;
			ndPolyhedra::ndPairKey code (edge->m_next->m_incidentVertex, edge->m_incidentVertex);
			ndEdge tmpEdge (edge->m_next->m_incidentVertex, -1);
			tmpEdge.m_incidentFace = -1; 
			ndPolyhedra::ndNode* const node = Insert (tmpEdge, code.GetVal(), state); 
			dAssert (!state);
			edge->m_twin = &node->GetInfo();
			edge->m_twin->m_twin = edge; 
			edgeArray[edgeCount] = edge->m_twin;
			edgeCount ++;
		}
	}

	for (ndInt32 i = 0; i < edgeCount; i ++) 
	{
		ndEdge* const edge = edgeArray[i];
		dAssert (!edge->m_prev);
		ndEdge *ptr = edge->m_twin;
		for (; ptr->m_next; ptr = ptr->m_next->m_twin){}
		ptr->m_next = edge;
		edge->m_prev = ptr;
	}

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
	dAssert (SanityCheck ());
#endif

	return true;
}

void ndPolyhedra::DeleteFace(ndEdge* const face)
{
	ndEdge* edgeList[D_LOCAL_BUFFER_SIZE * 16];

	if (face->m_incidentFace > 0) 
	{
		ndInt32 count = 0;
		ndEdge* ptr = face;
		do 
		{
			ptr->m_incidentFace = -1;
			ndInt32 i = 0;
			for (; i < count; i ++) 
			{
				if ((edgeList[i] == ptr) || (edgeList[i]->m_twin == ptr)) 
				{
					break;
				}
			}
			if (i == count) 
			{
				edgeList[count] = ptr;
				count ++;
			}
			ptr = ptr->m_next;
		} while (ptr != face);

		for (ndInt32 i = 0; i < count; i ++) 
		{
			ndEdge* const ptr1 = edgeList[i];
			if (ptr1->m_twin->m_incidentFace < 0) 
			{
				DeleteEdge (ptr1);
			}
		}
	}
}

ndBigVector ndPolyhedra::FaceNormal (const ndEdge* const face, const ndFloat64* const pool, ndInt32 strideInBytes) const
{
	ndInt32 stride = ndInt32 (strideInBytes / sizeof (ndFloat64));
	const ndEdge* edge = face;
	ndBigVector p0 (ndBigVector::m_triplexMask & ndBigVector(&pool[edge->m_incidentVertex * stride]));
	edge = edge->m_next;
	ndBigVector p1 (ndBigVector::m_triplexMask & ndBigVector(&pool[edge->m_incidentVertex * stride]));
	ndBigVector e1 (p1 - p0);

	ndBigVector normal (ndBigVector::m_zero);
	for (edge = edge->m_next; edge != face; edge = edge->m_next) 
	{
		ndBigVector p2 (ndBigVector::m_triplexMask & ndBigVector(&pool[edge->m_incidentVertex * stride]));
		ndBigVector e2 (p2 - p0);
		normal += e1.CrossProduct(e2);
		e1 = e2;
	} 
	dAssert (normal.m_w == ndFloat32 (0.0f));
	return normal;
}

ndEdge* ndPolyhedra::AddHalfEdge (ndInt32 v0, ndInt32 v1)
{
	if (v0 != v1) 
	{
		ndPairKey pairKey (v0, v1);
		ndEdge tmpEdge (v0, -1);

		ndNode* node = Insert (tmpEdge, pairKey.GetVal()); 
		return node ? &node->GetInfo() : nullptr;
	}
	else 
	{
		return nullptr;
	}
}

void ndPolyhedra::DeleteEdge (ndEdge* const edge)
{
	ndEdge *const twin = edge->m_twin;

	edge->m_prev->m_next = twin->m_next;
	twin->m_next->m_prev = edge->m_prev;
	edge->m_next->m_prev = twin->m_prev;
	twin->m_prev->m_next = edge->m_next;

	ndNode *const nodeA = GetNodeFromInfo (*edge);
	ndNode *const nodeB = GetNodeFromInfo (*twin);

	dAssert (&nodeA->GetInfo() == edge);
	dAssert (&nodeB->GetInfo() == twin);

	Remove (nodeA);
	Remove (nodeB);
}

ndEdge* ndPolyhedra::ConnectVertex (ndEdge* const e0, ndEdge* const e1)
{
	ndEdge* const edge = AddHalfEdge(e1->m_incidentVertex, e0->m_incidentVertex);
	ndEdge* const twin = AddHalfEdge(e0->m_incidentVertex, e1->m_incidentVertex);
	dAssert ((edge && twin) || !(edge || twin));
	if (edge) {
		edge->m_twin = twin;
		twin->m_twin = edge;

		edge->m_incidentFace = e0->m_incidentFace;
		twin->m_incidentFace = e1->m_incidentFace;

		edge->m_userData = e1->m_userData;
		twin->m_userData = e0->m_userData;

		edge->m_next = e0;
		edge->m_prev = e1->m_prev;

		twin->m_next = e1;
		twin->m_prev = e0->m_prev;

		e0->m_prev->m_next = twin;
		e0->m_prev = edge;

		e1->m_prev->m_next = edge;
		e1->m_prev = twin;
	}

	return edge;
}

ndEdge* ndPolyhedra::SpliteEdge (ndInt32 newIndex,	ndEdge* const edge)
{
	ndEdge* const edge00 = edge->m_prev;
	ndEdge* const edge01 = edge->m_next;
	ndEdge* const twin00 = edge->m_twin->m_next;
	ndEdge* const twin01 = edge->m_twin->m_prev;

	ndInt32 i0 = edge->m_incidentVertex;
	ndInt32 i1 = edge->m_twin->m_incidentVertex;

	ndInt32 f0 = edge->m_incidentFace;
	ndInt32 f1 = edge->m_twin->m_incidentFace;

	DeleteEdge (edge);

	ndEdge* const edge0 = AddHalfEdge (i0, newIndex);
	ndEdge* const edge1 = AddHalfEdge (newIndex, i1);

	ndEdge* const twin0 = AddHalfEdge (newIndex, i0);
	ndEdge* const twin1 = AddHalfEdge (i1, newIndex);
	dAssert (edge0);
	dAssert (edge1);
	dAssert (twin0);
	dAssert (twin1);

	edge0->m_twin = twin0;
	twin0->m_twin = edge0;

	edge1->m_twin = twin1;
	twin1->m_twin = edge1;

	edge0->m_next = edge1;
	edge1->m_prev = edge0;

	twin1->m_next = twin0;
	twin0->m_prev = twin1;

	edge0->m_prev = edge00;
	edge00 ->m_next = edge0;

	edge1->m_next = edge01;
	edge01->m_prev = edge1;

	twin0->m_next = twin00;
	twin00->m_prev = twin0;

	twin1->m_prev = twin01;
	twin01->m_next = twin1;

	edge0->m_incidentFace = f0;
	edge1->m_incidentFace = f0;

	twin0->m_incidentFace = f1;
	twin1->m_incidentFace = f1;

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
	//	dAssert (SanityCheck ());
#endif

	return edge0;
}

bool ndPolyhedra::FlipEdge (ndEdge* const edge)
{
	//	dNode *node;
	if (edge->m_next->m_next->m_next != edge) {
		return false;
	}

	if (edge->m_twin->m_next->m_next->m_next != edge->m_twin) {
		return false;
	}

	if (FindEdge(edge->m_prev->m_incidentVertex, edge->m_twin->m_prev->m_incidentVertex)) {
		return false;
	}

	ndEdge *const prevEdge = edge->m_prev;
	ndEdge *const prevTwin = edge->m_twin->m_prev;

	ndPairKey edgeKey (prevTwin->m_incidentVertex, prevEdge->m_incidentVertex);
	ndPairKey twinKey (prevEdge->m_incidentVertex, prevTwin->m_incidentVertex);

	ReplaceKey (GetNodeFromInfo (*edge), edgeKey.GetVal());
	//	dAssert (node);

	ReplaceKey (GetNodeFromInfo (*edge->m_twin), twinKey.GetVal());
	//	dAssert (node);

	edge->m_incidentVertex = prevTwin->m_incidentVertex;
	edge->m_twin->m_incidentVertex = prevEdge->m_incidentVertex;

	edge->m_userData = prevTwin->m_userData;
	edge->m_twin->m_userData = prevEdge->m_userData;

	prevEdge->m_next = edge->m_twin->m_next;
	prevTwin->m_prev->m_prev = edge->m_prev;

	prevTwin->m_next = edge->m_next;
	prevEdge->m_prev->m_prev = edge->m_twin->m_prev;

	edge->m_prev = prevTwin->m_prev;
	edge->m_next = prevEdge;

	edge->m_twin->m_prev = prevEdge->m_prev;
	edge->m_twin->m_next = prevTwin;

	prevTwin->m_prev->m_next = edge;
	prevTwin->m_prev = edge->m_twin;

	prevEdge->m_prev->m_next = edge->m_twin;
	prevEdge->m_prev = edge;

	edge->m_next->m_incidentFace = edge->m_incidentFace;
	edge->m_prev->m_incidentFace = edge->m_incidentFace;

	edge->m_twin->m_next->m_incidentFace = edge->m_twin->m_incidentFace;
	edge->m_twin->m_prev->m_incidentFace = edge->m_twin->m_incidentFace;


#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
	dAssert (SanityCheck ());
#endif

	return true;
}

bool ndPolyhedra::GetConectedSurface (ndPolyhedra &polyhedra) const
{
	if (!GetCount()) {
		return false;
	}

	ndEdge* edge = nullptr;
	Iterator iter(*this);
	for (iter.Begin (); iter; iter ++) {
		edge = &(*iter);
		if ((edge->m_mark < m_baseMark) && (edge->m_incidentFace > 0)) {
			break;
		}
	}

	if (!iter) {
		return false;
	}

	ndInt32 faceIndex[4096];
	ndInt64 faceDataIndex[4096];
	ndStack<ndEdge*> stackPool (GetCount()); 
	ndEdge** const stack = &stackPool[0];

	ndInt32 mark = IncLRU();

	polyhedra.BeginFace ();
	stack[0] = edge;
	ndInt32 index = 1;
	while (index) {
		index --;
		ndEdge* const edge1 = stack[index];
		dAssert (edge1);
		if (edge1->m_mark == mark) {
			continue;
		}

		ndInt32 count = 0;
		ndEdge* ptr = edge1;
		do {
			dAssert (ptr);
			ptr->m_mark = mark;
			faceIndex[count] = ptr->m_incidentVertex;
			faceDataIndex[count] = ndInt64 (ptr->m_userData);
			count ++;
			dAssert (count <  ndInt32 ((sizeof (faceIndex)/sizeof(faceIndex[0]))));

			if ((ptr->m_twin->m_incidentFace > 0) && (ptr->m_twin->m_mark != mark)) {
				stack[index] = ptr->m_twin;
				index ++;
				dAssert (index < GetCount());
			}

			ptr = ptr->m_next;
		} while (ptr != edge1);

		polyhedra.AddFace (count, &faceIndex[0], &faceDataIndex[0]);
	}

	polyhedra.EndFace ();

	return true;
}

void ndPolyhedra::ChangeEdgeIncidentVertex (ndEdge* const edge, ndInt32 newIndex)
{
	ndEdge* ptr = edge;
	do {
		ndNode* node = GetNodeFromInfo(*ptr);
		ndPairKey Key0 (newIndex, ptr->m_twin->m_incidentVertex);
		ReplaceKey (node, Key0.GetVal());

		node = GetNodeFromInfo(*ptr->m_twin);
		ndPairKey Key1 (ptr->m_twin->m_incidentVertex, newIndex);
		ReplaceKey (node, Key1.GetVal());

		ptr->m_incidentVertex = newIndex;

		ptr = ptr->m_twin->m_next;
	} while (ptr != edge);
}

void ndPolyhedra::DeleteDegenerateFaces (const ndFloat64* const pool, ndInt32 strideInBytes, ndFloat64 area)
{
	if (!GetCount()) 
	{
		return;
	}

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
	dAssert (SanityCheck ());
#endif
	ndStack <ndPolyhedra::ndNode*> faceArrayPool(GetCount() / 2 + 100);

	ndInt32 count = 0;
	ndPolyhedra::ndNode** const faceArray = &faceArrayPool[0];
	ndInt32 mark = IncLRU();
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) 
	{
		ndEdge* const edge = &(*iter);

		if ((edge->m_mark != mark) && (edge->m_incidentFace > 0)) 
		{
			faceArray[count] = iter.GetNode();
			count ++;
			ndEdge* ptr = edge;
			do	{
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}

	ndFloat64 area2 = area * area;
	area2 *= ndFloat64 (4.0f);

	for (ndInt32 i = 0; i < count; i ++) 
	{
		ndPolyhedra::ndNode* const faceNode = faceArray[i];
		ndEdge* const edge = &faceNode->GetInfo();

		ndBigVector normal (FaceNormal (edge, pool, strideInBytes));

		ndFloat64 faceArea = normal.DotProduct(normal).GetScalar();
		if (faceArea < area2) 
		{
			DeleteFace (edge);
		}
	}

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
	mark = IncLRU();
	for (iter.Begin(); iter; iter ++) 
	{
		ndEdge* const edge = &(*iter);
		if ((edge->m_mark != mark) && (edge->m_incidentFace > 0)) 
		{
			//dAssert (edge->m_next->m_next->m_next == edge);
			ndEdge* ptr = edge;
			do	
			{
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);

			ndBigVector normal (FaceNormal (edge, pool, strideInBytes));

			ndFloat64 faceArea = normal.DotProduct(normal).GetScalar();
			dAssert (faceArea >= area2);
		}
	}
	dAssert (SanityCheck ());
#endif
}

ndBigPlane ndPolyhedra::UnboundedLoopPlane (ndInt32 i0, ndInt32 i1, ndInt32 i2, const ndBigVector* const pool)
{
	const ndBigVector p0 = pool[i0];
	const ndBigVector p1 = pool[i1];
	const ndBigVector p2 = pool[i2];
	ndBigVector E0 (p1 - p0); 
	ndBigVector E1 (p2 - p0); 

	ndBigVector N ((E0.CrossProduct(E1)).CrossProduct(E0) & ndBigVector::m_triplexMask); 
	ndFloat64 dist = - N.DotProduct(p0).GetScalar();
	ndBigPlane plane (N, dist);

	ndFloat64 mag = sqrt (plane.DotProduct(plane & ndBigVector::m_triplexMask).GetScalar());
	if (mag < ndFloat64 (1.0e-12f)) 
	{
		mag = ndFloat64 (1.0e-12f);
	}
	mag = ndFloat64 (10.0f) / mag;

	plane.m_x *= mag;
	plane.m_y *= mag;
	plane.m_z *= mag;
	plane.m_w *= mag;

	return plane;
}

ndEdge* ndPolyhedra::CollapseEdge(ndEdge* const edge)
{
	ndInt32 v0 = edge->m_incidentVertex;
	ndInt32 v1 = edge->m_twin->m_incidentVertex;

	ndEdge* retEdge = edge->m_twin->m_prev->m_twin;
	if (retEdge	== edge->m_twin->m_next) {
		return nullptr;
	}
	if (retEdge	== edge->m_twin) {
		return nullptr;
	}
	if (retEdge	== edge->m_next) {
		retEdge = edge->m_prev->m_twin;
		if (retEdge	== edge->m_twin->m_next) {
			return nullptr;
		}
		if (retEdge	== edge->m_twin) {
			return nullptr;
		}
	}

	ndEdge* lastEdge = nullptr;
	ndEdge* firstEdge = nullptr;
	if ((edge->m_incidentFace >= 0)	&& (edge->m_twin->m_incidentFace >= 0)) {	
		lastEdge = edge->m_prev->m_twin;
		firstEdge = edge->m_twin->m_next->m_twin->m_next;
	} else if (edge->m_twin->m_incidentFace >= 0) {
		firstEdge = edge->m_twin->m_next->m_twin->m_next;
		lastEdge = edge;
	} else {
		lastEdge = edge->m_prev->m_twin;
		firstEdge = edge->m_twin->m_next;
	}

	for (ndEdge* ptr = firstEdge; ptr != lastEdge; ptr = ptr->m_twin->m_next) {
		ndEdge* const badEdge = FindEdge (edge->m_twin->m_incidentVertex, ptr->m_twin->m_incidentVertex);
		if (badEdge) {
			return nullptr;
		}
	} 

	ndEdge* const twin = edge->m_twin;
	if (twin->m_next == twin->m_prev->m_prev) {
		twin->m_prev->m_twin->m_twin = twin->m_next->m_twin;
		twin->m_next->m_twin->m_twin = twin->m_prev->m_twin;

		Remove (GetNodeFromInfo(*twin->m_prev));
		Remove (GetNodeFromInfo(*twin->m_next));
	} else {
		twin->m_next->m_userData = twin->m_userData;
		twin->m_next->m_prev = twin->m_prev;
		twin->m_prev->m_next = twin->m_next;
	}

	if (edge->m_next == edge->m_prev->m_prev) {
		edge->m_next->m_twin->m_twin = edge->m_prev->m_twin;
		edge->m_prev->m_twin->m_twin = edge->m_next->m_twin;
		Remove (GetNodeFromInfo(*edge->m_next));
		Remove (GetNodeFromInfo(*edge->m_prev));
	} else {
		edge->m_next->m_prev = edge->m_prev;
		edge->m_prev->m_next = edge->m_next;
	}

	dAssert (twin->m_twin->m_incidentVertex == v0);
	dAssert (edge->m_twin->m_incidentVertex == v1);
	Remove (GetNodeFromInfo(*twin));
	Remove (GetNodeFromInfo(*edge));

	ndEdge* ptr = retEdge;
	do {
		ndPolyhedra::ndPairKey pairKey (v0, ptr->m_twin->m_incidentVertex);

		ndPolyhedra::ndNode* node = Find (pairKey.GetVal());
		if (node) {
			if (&node->GetInfo() == ptr) {
				ndPolyhedra::ndPairKey key (v1, ptr->m_twin->m_incidentVertex);
				ptr->m_incidentVertex = v1;
				node = ReplaceKey (node, key.GetVal());
				dAssert (node);
			} 
		}

		ndPolyhedra::ndPairKey TwinKey (ptr->m_twin->m_incidentVertex, v0);
		node = Find (TwinKey.GetVal());
		if (node) {
			if (&node->GetInfo() == ptr->m_twin) {
				ndPolyhedra::ndPairKey key (ptr->m_twin->m_incidentVertex, v1);
				node = ReplaceKey (node, key.GetVal());
				dAssert (node);
			}
		}

		ptr = ptr->m_twin->m_next;
	} while (ptr != retEdge);

	return retEdge;
}

void ndPolyhedra::RemoveHalfEdge (ndEdge* const edge)
{
	dEdgeCollapseEdgeHandle* const handle = (dEdgeCollapseEdgeHandle *) dIntToPointer (edge->m_userData);
	if (handle) { 
		handle->m_edge = nullptr;
	}

	ndPolyhedra::ndNode* const node = GetNodeFromInfo(*edge);
	dAssert (node);
	Remove (node);
}

ndEdge* ndPolyhedra::FindEarTip (ndEdge* const face, const ndFloat64* const pool, ndInt32 stride, ndDownHeap<ndEdge*, ndFloat64>& heap, const ndBigVector &normal) const
{
	ndEdge* ptr = face;
	ndBigVector p0 (ndBigVector::m_triplexMask & ndBigVector(&pool[ptr->m_prev->m_incidentVertex * stride]));
	ndBigVector p1 (ndBigVector::m_triplexMask & ndBigVector(&pool[ptr->m_incidentVertex * stride]));
	ndBigVector d0 (p1 - p0);
	ndFloat64 val = sqrt (d0.DotProduct(d0 & ndBigVector::m_triplexMask).GetScalar());
	if (val < ndFloat64 (1.0e-10f)) {
		val = ndFloat64 (1.0e-10f);
	}
	d0 = d0.Scale (ndFloat64 (1.0f) / val);

	ndFloat64 minAngle = ndFloat32 (10.0f);
	do {
		ndBigVector p2 (ndBigVector::m_triplexMask & ndBigVector(&pool [ptr->m_next->m_incidentVertex * stride]));
		ndBigVector d1 (p2 - p1);
		ndFloat64 val1 = ndFloat64 (1.0f) / sqrt (d1.DotProduct(d1).GetScalar());
		if (val1 < ndFloat64 (1.0e-10f)) {
			val1 = ndFloat64 (1.0e-10f);
		}
		d1 = d1.Scale (ndFloat32 (1.0f) / val1);
		ndBigVector n (d0.CrossProduct(d1));

		ndFloat64 angle = normal.DotProduct(n & ndBigVector::m_triplexMask).GetScalar();
		if (angle >= ndFloat64 (0.0f)) {
			heap.Push (ptr, angle);
		}

		if (angle < minAngle) {
			minAngle = angle;
		}

		d0 = d1;
		p1 = p2;
		ptr = ptr->m_next;
	} while (ptr != face);

	if (minAngle > ndFloat32 (0.1f)) {
		return heap[0];
	}

	ndEdge* ear = nullptr;
	while (heap.GetCount()) {
		ear = heap[0];
		heap.Pop();

		if (FindEdge (ear->m_prev->m_incidentVertex, ear->m_next->m_incidentVertex)) {
			continue;
		}

		ndBigVector q0 (ndBigVector::m_triplexMask & ndBigVector(&pool [ear->m_prev->m_incidentVertex * stride]));
		ndBigVector q1 (ndBigVector::m_triplexMask & ndBigVector(&pool [ear->m_incidentVertex * stride]));
		ndBigVector q2 (ndBigVector::m_triplexMask & ndBigVector(&pool [ear->m_next->m_incidentVertex * stride]));

		ndBigVector p10 (q1 - q0);
		ndBigVector p21 (q2 - q1);
		ndBigVector p02 (q0 - q2);
		dAssert(normal.m_w == ndFloat32(0.0f));

		for (ptr = ear->m_next->m_next; ptr != ear->m_prev; ptr = ptr->m_next) {
			if (!((ptr->m_incidentVertex == ear->m_incidentVertex) || (ptr->m_incidentVertex == ear->m_prev->m_incidentVertex) || (ptr->m_incidentVertex == ear->m_next->m_incidentVertex))) { 
				ndBigVector p (ndBigVector::m_triplexMask & ndBigVector(&pool [ptr->m_incidentVertex * stride]));

				//ndFloat64 side = ((p - p0) * p10) % normal;
				ndFloat64 side = normal.DotProduct((p - q0).CrossProduct(p10)).GetScalar();
				if (side < ndFloat64 (0.05f)) {
					//side = ((p - p1) * p21) % normal;
					side = normal.DotProduct((p - q1).CrossProduct(p21)).GetScalar();
					if (side < ndFloat64 (0.05f)) {
						//side = ((p - p2) * p02) % normal;
						side = normal.DotProduct((p - q2).CrossProduct(p02)).GetScalar();
						if (side < ndFloat32 (0.05f)) {
							break;
						}
					}
				}
			}
		}

		if (ptr == ear->m_prev) {
			break;
		}
	}

	return ear;
}

ndEdge* ndPolyhedra::TriangulateFace (ndEdge* const faceIn, const ndFloat64* const pool, ndInt32 stride, ndDownHeap<ndEdge*, ndFloat64>& heap, ndBigVector* const faceNormalOut)
{
	ndEdge* face = faceIn;
	ndBigVector normal (FaceNormal (face, pool, ndInt32 (stride * sizeof (ndFloat64))));
	dAssert(normal.m_w == ndFloat32(0.0f));
	ndFloat64 dot = normal.DotProduct(normal).GetScalar();
	if (dot < ndFloat64 (1.0e-12f)) {
		if (faceNormalOut) {
			*faceNormalOut = ndBigVector (ndFloat32 (0.0f)); 
		}
		return face;
	}
	normal = normal.Scale (ndFloat64 (1.0f) / sqrt (dot));
	if (faceNormalOut) {
		*faceNormalOut = normal;
	}

	while (face->m_next->m_next->m_next != face) {
		ndEdge* const ear = FindEarTip (face, pool, stride, heap, normal); 
		if (!ear) {
			return face;
		}
		if ((face == ear)	|| (face == ear->m_prev)) {
			face = ear->m_prev->m_prev;
		}
		ndEdge* const edge = AddHalfEdge (ear->m_next->m_incidentVertex, ear->m_prev->m_incidentVertex);
		if (!edge) {
			return face;
		}
		ndEdge* const twin = AddHalfEdge (ear->m_prev->m_incidentVertex, ear->m_next->m_incidentVertex);
		if (!twin) {
			return face;
		}
		dAssert (twin);


		edge->m_mark = ear->m_mark;
		edge->m_userData = ear->m_next->m_userData;
		edge->m_incidentFace = ear->m_incidentFace;

		twin->m_mark = ear->m_mark;
		twin->m_userData = ear->m_prev->m_userData;
		twin->m_incidentFace = ear->m_incidentFace;

		edge->m_twin = twin;
		twin->m_twin = edge;

		twin->m_prev = ear->m_prev->m_prev;
		twin->m_next = ear->m_next;
		ear->m_prev->m_prev->m_next = twin;
		ear->m_next->m_prev = twin;

		edge->m_next = ear->m_prev;
		edge->m_prev = ear;
		ear->m_prev->m_prev = edge;
		ear->m_next = edge;

		heap.Flush ();
	}
	return nullptr;
}

void ndPolyhedra::MarkAdjacentCoplanarFaces (ndPolyhedra& polyhedraOut, ndEdge* const face, const ndFloat64* const pool, ndInt32 strideInBytes)
{
	const ndFloat64 normalDeviation = ndFloat64 (0.9999f);
	const ndFloat64 distanceFromPlane = ndFloat64 (1.0f / 128.0f);

	ndInt32 faceIndex[D_LOCAL_BUFFER_SIZE * 8];
	ndInt64 userIndex[D_LOCAL_BUFFER_SIZE * 8];
	ndEdge* stack[D_LOCAL_BUFFER_SIZE * 8];
	ndEdge* deleteEdge[D_LOCAL_BUFFER_SIZE * 32];

	ndInt32 deleteCount = 1;
	deleteEdge[0] = face;
	ndInt32 stride = ndInt32 (strideInBytes / sizeof (ndFloat64));

	dAssert (face->m_incidentFace > 0);

	ndBigVector normalAverage (FaceNormal (face, pool, strideInBytes));
	dAssert (normalAverage.m_w == ndFloat32 (0.0f));
	ndFloat64 dot = normalAverage.DotProduct(normalAverage).GetScalar();
	if (dot > ndFloat64 (1.0e-12f)) {
		ndInt32 testPointsCount = 1;
		dot = ndFloat64 (1.0f) / sqrt (dot);
		ndBigVector normal (normalAverage.Scale (dot));

		ndBigVector averageTestPoint (ndBigVector::m_triplexMask & ndBigVector(&pool[face->m_incidentVertex * stride]));
		ndBigPlane testPlane(normal, - normal.DotProduct(averageTestPoint & ndBigVector::m_triplexMask).GetScalar());

		polyhedraOut.BeginFace();

		IncLRU();
		ndInt32 faceMark = IncLRU();

		ndInt32 faceIndexCount = 0;
		ndEdge* ptr = face;
		do {
			ptr->m_mark = faceMark;
			faceIndex[faceIndexCount] = ptr->m_incidentVertex;
			userIndex[faceIndexCount] = ndInt64 (ptr->m_userData);
			faceIndexCount ++;
			dAssert (faceIndexCount < ndInt32 (sizeof (faceIndex) / sizeof (faceIndex[0])));
			ptr = ptr ->m_next;
		} while (ptr != face);
		polyhedraOut.AddFace(faceIndexCount, faceIndex, userIndex);

		ndInt32 index = 1;
		deleteCount = 0;
		stack[0] = face;
		while (index) {
			index --;
			ndEdge* const face1 = stack[index];
			deleteEdge[deleteCount] = face1;
			deleteCount ++;
			dAssert (deleteCount < ndInt32 (sizeof (deleteEdge) / sizeof (deleteEdge[0])));
			dAssert (face1->m_next->m_next->m_next == face1);

			ndEdge* edge = face1;
			do {
				ndEdge* const ptr1 = edge->m_twin;
				if (ptr1->m_incidentFace > 0) {
					if (ptr1->m_mark != faceMark) {
						ndEdge* ptr2 = ptr1;
						faceIndexCount = 0;
						do {
							ptr2->m_mark = faceMark;
							faceIndex[faceIndexCount] = ptr2->m_incidentVertex;
							userIndex[faceIndexCount] = ndInt64 (ptr2->m_userData);
							dAssert (faceIndexCount < ndInt32 (sizeof (faceIndex) / sizeof (faceIndex[0])));
							faceIndexCount ++;
							ptr2 = ptr2 ->m_next;
						} while (ptr2 != ptr1);

						ndBigVector normal1 (FaceNormal (ptr1, pool, strideInBytes));
						dot = normal1.DotProduct(normal1).GetScalar();
						if (dot < ndFloat64 (1.0e-12f)) {
							deleteEdge[deleteCount] = ptr1;
							deleteCount ++;
							dAssert (deleteCount < ndInt32 (sizeof (deleteEdge) / sizeof (deleteEdge[0])));
						} else {
							ndBigVector testNormal (normal1.Scale (ndFloat64 (1.0f) / sqrt (dot)));
							dAssert (testNormal.m_w == ndFloat32 (0.0f));
							dot = normal.DotProduct(testNormal).GetScalar();
							if (dot >= normalDeviation) {
								ndBigVector testPoint (ndBigVector::m_triplexMask & ndBigVector(&pool[ptr1->m_prev->m_incidentVertex * stride]));
								ndFloat64 dist = fabs (testPlane.Evalue (testPoint));
								if (dist < distanceFromPlane) {
									testPointsCount ++;

									averageTestPoint += testPoint;
									testPoint = averageTestPoint.Scale (ndFloat64 (1.0f) / ndFloat64(testPointsCount));

									normalAverage += normal1;
									dAssert (normalAverage.m_w == ndFloat32 (0.0f));
									testNormal = normalAverage.Scale (ndFloat64 (1.0f) / sqrt (normalAverage.DotProduct(normalAverage).GetScalar()));
									testPlane = ndBigPlane (testNormal, - testPoint.DotProduct (testNormal).GetScalar());

									polyhedraOut.AddFace(faceIndexCount, faceIndex, userIndex);
									stack[index] = ptr1;
									index ++;
									dAssert (index < ndInt32 (sizeof (stack) / sizeof (stack[0])));
								}
							}
						}
					}
				}

				edge = edge->m_next;
			} while (edge != face1);
		}
		polyhedraOut.EndFace();
	}

	for (ndInt32 index = 0; index < deleteCount; index ++) {
		DeleteFace (deleteEdge[index]);
	}
}

void ndPolyhedra::RefineTriangulation (const ndFloat64* const vertex, ndInt32 stride, const ndBigVector& normal, ndInt32 perimeterCount, ndEdge** const perimeter)
{
	ndList<dgDiagonalEdge> dignonals;

	for (ndInt32 i = 1; i <= perimeterCount; i ++) {
		ndEdge* const last = perimeter[i - 1];
		for (ndEdge* ptr = perimeter[i]->m_prev; ptr != last; ptr = ptr->m_twin->m_prev) {
			ndList<dgDiagonalEdge>::ndNode* node = dignonals.GetFirst();
			for (; node; node = node->GetNext()) {
				const dgDiagonalEdge& key = node->GetInfo();
				if (((key.m_i0 == ptr->m_incidentVertex) && (key.m_i1 == ptr->m_twin->m_incidentVertex)) ||
					((key.m_i1 == ptr->m_incidentVertex) && (key.m_i0 == ptr->m_twin->m_incidentVertex))) {
						break;
				}
			}
			if (!node) {
				dgDiagonalEdge key (ptr);
				dignonals.Append(key);
			}
		}
	}

	ndEdge* const face = perimeter[0];
	ndInt32 i0 = face->m_incidentVertex * stride;
	ndInt32 i1 = face->m_next->m_incidentVertex * stride;
	ndBigVector p0 (vertex[i0], vertex[i0 + 1], vertex[i0 + 2], ndFloat32 (0.0f));
	ndBigVector p1 (vertex[i1], vertex[i1 + 1], vertex[i1 + 2], ndFloat32 (0.0f));

	ndBigVector p1p0 (p1 - p0);
	ndFloat64 mag2 = p1p0.DotProduct(p1p0).GetScalar();
	for (ndEdge* ptr = face->m_next->m_next; mag2 < ndFloat32 (1.0e-12f); ptr = ptr->m_next) {
		ndInt32 i2 = ptr->m_incidentVertex * stride;
		ndBigVector p2 (vertex[i2], vertex[i2 + 1], vertex[i2 + 2], ndFloat32 (0.0f));
		p1p0 = p2 - p0;
		mag2 = p1p0.DotProduct(p1p0).GetScalar();
	}

	dAssert (p1p0.m_w == ndFloat32 (0.0f));
	ndMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = p0;
	matrix.m_front = ndVector (p1p0.Scale (ndFloat64 (1.0f) / sqrt (mag2)));
	matrix.m_right = ndVector (normal.Scale (ndFloat64 (1.0f) / sqrt (normal.DotProduct(normal).GetScalar())));
	matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);
	matrix = matrix.Inverse();
	dAssert (matrix.m_posit.m_w == ndFloat32 (1.0f));
//	matrix.m_posit.m_w = ndFloat32 (1.0f);

	ndInt32 maxCount = dignonals.GetCount() * dignonals.GetCount();
	while (dignonals.GetCount() && maxCount) {
		maxCount --;
		ndList<dgDiagonalEdge>::ndNode* const node = dignonals.GetFirst();
		dgDiagonalEdge key (node->GetInfo());
		dignonals.Remove(node);
		ndEdge* const edge = FindEdge(key.m_i0, key.m_i1);
		if (edge) {
			ndInt32 k0 = edge->m_incidentVertex * stride;
			ndInt32 k1 = edge->m_next->m_incidentVertex * stride;
			ndInt32 k2 = edge->m_next->m_next->m_incidentVertex * stride;
			ndInt32 k3 = edge->m_twin->m_prev->m_incidentVertex * stride;

			ndBigVector q0 (vertex[k0], vertex[k0 + 1], vertex[k0 + 2], ndFloat64 (1.0f));
			ndBigVector q1 (vertex[k1], vertex[k1 + 1], vertex[k1 + 2], ndFloat64 (1.0f));
			ndBigVector q2 (vertex[k2], vertex[k2 + 1], vertex[k2 + 2], ndFloat64 (1.0f));
			ndBigVector q3 (vertex[k3], vertex[k3 + 1], vertex[k3 + 2], ndFloat64 (1.0f));

			q0 = matrix.TransformVector(q0);
			q1 = matrix.TransformVector(q1);
			q2 = matrix.TransformVector(q2);
			q3 = matrix.TransformVector(q3);

			ndFloat64 circleTest[3][3];
			circleTest[0][0] = q0[0] - q3[0];
			circleTest[0][1] = q0[1] - q3[1];
			circleTest[0][2] = circleTest[0][0] * circleTest[0][0] + circleTest[0][1] * circleTest[0][1];

			circleTest[1][0] = q1[0] - q3[0];
			circleTest[1][1] = q1[1] - q3[1];
			circleTest[1][2] = circleTest[1][0] * circleTest[1][0] + circleTest[1][1] * circleTest[1][1];

			circleTest[2][0] = q2[0] - q3[0];
			circleTest[2][1] = q2[1] - q3[1];
			circleTest[2][2] = circleTest[2][0] * circleTest[2][0] + circleTest[2][1] * circleTest[2][1];

			ndFloat64 error;
			ndFloat64 det = Determinant3x3 (circleTest, &error);
			if (det < ndFloat32 (0.0f)) {
				ndEdge* frontFace0 = edge->m_prev;
				ndEdge* backFace0 = edge->m_twin->m_prev;

				FlipEdge(edge);

				if (perimeterCount > 4) {
					ndEdge* backFace1 = backFace0->m_next;
					ndEdge* frontFace1 = frontFace0->m_next;
					for (ndInt32 i = 0; i < perimeterCount; i ++) {
						if (frontFace0 == perimeter[i]) {
							frontFace0 = nullptr;
						}
						if (frontFace1 == perimeter[i]) {
							frontFace1 = nullptr;
						}

						if (backFace0 == perimeter[i]) {
							backFace0 = nullptr;
						}
						if (backFace1 == perimeter[i]) {
							backFace1 = nullptr;
						}
					}

					if (backFace0 && (backFace0->m_incidentFace > 0) && (backFace0->m_twin->m_incidentFace > 0)) {
						dgDiagonalEdge key0 (backFace0);
						dignonals.Append(key0);
					}
					if (backFace1 && (backFace1->m_incidentFace > 0) && (backFace1->m_twin->m_incidentFace > 0)) {
						dgDiagonalEdge key1 (backFace1);
						dignonals.Append(key1);
					}

					if (frontFace0 && (frontFace0->m_incidentFace > 0) && (frontFace0->m_twin->m_incidentFace > 0)) {
						dgDiagonalEdge key0 (frontFace0);
						dignonals.Append(key0);
					}

					if (frontFace1 && (frontFace1->m_incidentFace > 0) && (frontFace1->m_twin->m_incidentFace > 0)) {
						dgDiagonalEdge key1 (frontFace1);
						dignonals.Append(key1);
					}
				}
			}
		}
	}
}

void ndPolyhedra::RefineTriangulation (const ndFloat64* const vertex, ndInt32 stride)
{
	if (GetCount() <= 6) {
		return;
	}

	ndInt32 mark = IncLRU();
	ndInt32 loopCount = 0;
	
	ndPolyhedra::Iterator iter (*this);
	ndEdge* edgePerimeters[D_LOCAL_BUFFER_SIZE * 16];
	ndInt32 perimeterCount = 0;
	ndTree<ndEdge*, ndInt32> filter;
	for (iter.Begin(); iter && (loopCount <= 1) ; iter ++) {
		ndEdge* const edge = &(*iter);
		if ((edge->m_incidentFace < 0) && (edge->m_mark != mark)){
			loopCount ++;
			ndEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				if (!filter.Insert(ptr, ptr->m_incidentVertex)) {
					loopCount = 2;
					break;
				}
				edgePerimeters[perimeterCount] = ptr->m_twin;
				perimeterCount ++;
				dAssert (perimeterCount < ndInt32 (sizeof (edgePerimeters) / sizeof (edgePerimeters[0])));
				ptr = ptr->m_prev;
			} while (ptr != edge);
		}
	}

	if (loopCount == 1) {
		#ifdef _DEBUG
		for (ndInt32 i = 0; i < perimeterCount; i ++) {
			for (ndInt32 j = i + 1; j < perimeterCount; j ++) {
				dAssert (edgePerimeters[i]->m_incidentVertex != edgePerimeters[j]->m_incidentVertex);
			}
		}
		#endif

		dAssert (perimeterCount);
		dAssert (perimeterCount < ndInt32 (sizeof (edgePerimeters) / sizeof (edgePerimeters[0])));
		edgePerimeters[perimeterCount] = edgePerimeters[0];

		ndBigVector normal (FaceNormal(edgePerimeters[0], vertex, ndInt32 (stride * sizeof (ndFloat64))));
		if (normal.DotProduct(normal).GetScalar() > ndFloat32 (1.0e-12f)) {
			RefineTriangulation (vertex, stride, normal, perimeterCount, edgePerimeters);
		}
	}
}

void ndPolyhedra::OptimizeTriangulation (const ndFloat64* const vertex, ndInt32 strideInBytes)
{
	ndInt32 polygon[D_LOCAL_BUFFER_SIZE * 8];
	ndInt64 userData[D_LOCAL_BUFFER_SIZE * 8];
	ndInt32 stride = ndInt32 (strideInBytes / sizeof (ndFloat64));

	ndPolyhedra leftOver;
	ndPolyhedra buildConvex;

	buildConvex.BeginFace();
	ndPolyhedra::Iterator iter (*this);

	for (iter.Begin(); iter; ) {
		ndEdge* const edge = &(*iter);
		iter++;

		if (edge->m_incidentFace > 0) {
			ndPolyhedra flatFace;
			MarkAdjacentCoplanarFaces (flatFace, edge, vertex, strideInBytes);
			//dAssert (flatFace.GetCount());

			if (flatFace.GetCount()) {
				flatFace.RefineTriangulation (vertex, stride);

				ndInt32 mark = flatFace.IncLRU();
				ndPolyhedra::Iterator iter1 (flatFace);
				for (iter1.Begin(); iter1; iter1 ++) {
					ndEdge* const edge1 = &(*iter1);
					if (edge1->m_mark != mark) {
						if (edge1->m_incidentFace > 0) {
							ndEdge* ptr = edge1;
							ndInt32 vertexCount = 0;
							do {
								polygon[vertexCount] = ptr->m_incidentVertex;				
								userData[vertexCount] = ndInt64 (ptr->m_userData);
								vertexCount ++;
								dAssert (vertexCount < ndInt32 (sizeof (polygon) / sizeof (polygon[0])));
								ptr->m_mark = mark;
								ptr = ptr->m_next;
							} while (ptr != edge1);
							if (vertexCount >= 3) {
								buildConvex.AddFace (vertexCount, polygon, userData);
							}
						}
					}
				}
			}
			iter.Begin();
		}
	}
	buildConvex.EndFace();
	dAssert (GetCount() == 0);
	SwapInfo(buildConvex);
}

void ndPolyhedra::Triangulate (const ndFloat64* const vertex, ndInt32 strideInBytes, ndPolyhedra* const leftOver)
{
	ndInt32 stride = ndInt32 (strideInBytes / sizeof (ndFloat64));

	ndInt32 count = GetCount() / 2;
	ndStack<char> memPool (ndInt32 ((count + 512) * (2 * sizeof (ndFloat64)))); 
	ndDownHeap<ndEdge*, ndFloat64> heap(&memPool[0], memPool.GetSizeInBytes());

	ndInt32 mark = IncLRU();
	Iterator iter (*this);
	for (iter.Begin(); iter; ) {
		ndEdge* const thisEdge = &(*iter);
		iter ++;

		if (thisEdge->m_mark == mark) {
			continue;
		}
		if (thisEdge->m_incidentFace < 0) {
			continue;
		}

		count = 0;
		ndEdge* ptr = thisEdge;
		do {
			count ++;
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != thisEdge);

		if (count > 3) {
			ndEdge* const edge = TriangulateFace (thisEdge, vertex, stride, heap, nullptr);
			heap.Flush ();

			if (edge) {
				dAssert (edge->m_incidentFace > 0);

				if (leftOver) {
					ndInt32* const index = (ndInt32 *) &heap[0];
					ndInt64* const data = (ndInt64 *)&index[count];
					ndInt32 i = 0;
					ndEdge* ptr1 = edge;
					do {
						index[i] = ptr1->m_incidentVertex;
						data[i] = ndInt64 (ptr1->m_userData);
						i ++;
						ptr1 = ptr1->m_next;
					} while (ptr1 != edge);
					leftOver->AddFace(i, index, data);

				} else {
					dTrace (("Deleting face:"));					
					ptr = edge;
					do {
						dTrace (("%d ", ptr->m_incidentVertex));
					} while (ptr != edge);
					dTrace (("\n"));					
				}

				DeleteFace (edge);
				iter.Begin();
			}
		}
	}

	OptimizeTriangulation (vertex, strideInBytes);

	mark = IncLRU();
	m_faceSecuence = 1;
	for (iter.Begin(); iter; iter ++) {
		ndEdge* edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}
		if (edge->m_incidentFace < 0) {
			continue;
		}
		dAssert (edge == edge->m_next->m_next->m_next);

		for (ndInt32 i = 0; i < 3; i ++) { 
			edge->m_incidentFace = m_faceSecuence; 
			edge->m_mark = mark;
			edge = edge->m_next;
		}
		m_faceSecuence ++;
	}
}

bool ndPolyhedra::IsFaceConvex(ndEdge* const face, const ndFloat64* const vertex, ndInt32 strideInBytes) const
{
	if (face->m_next->m_next->m_next == face) 
	{
		return true;
	}
	ndBigVector normal(FaceNormal(face, vertex, strideInBytes));
	dAssert(normal.m_w == ndFloat32(0.0f));

	ndInt32 stride = strideInBytes / sizeof(ndFloat64);
	ndEdge* ptr = face;
	do 
	{
		ndBigVector p0(ndBigVector::m_triplexMask & ndBigVector(&vertex[ptr->m_incidentVertex * stride]));
		ndBigVector p1(ndBigVector::m_triplexMask & ndBigVector(&vertex[ptr->m_prev->m_incidentVertex * stride]));
		ndBigVector p2(ndBigVector::m_triplexMask & ndBigVector(&vertex[ptr->m_next->m_incidentVertex * stride]));

		ndBigVector e0(p1 - p0);
		ndBigVector e1(p2 - p1);
		ndBigVector cornerNormal(e1.CrossProduct(e0));
		ndFloat64 project(normal.DotProduct(cornerNormal).GetScalar());
		if (project < ndFloat32(0.0f)) 
		{
			return false;
		}

		ptr = ptr->m_next;
	} while (ptr != face);
	
	return true;
}

void ndPolyhedra::RemoveOuterColinearEdges (ndPolyhedra& flatFace, const ndFloat64* const vertex, ndInt32 stride)
{
	ndEdge* edgePerimeters[D_LOCAL_BUFFER_SIZE];

	ndInt32 perimeterCount = 0;
	ndInt32 mark = flatFace.IncLRU();
	ndPolyhedra::Iterator iter (flatFace);
	for (iter.Begin(); iter; iter ++) {
		ndEdge* const edge = &(*iter);
		if ((edge->m_incidentFace < 0) && (edge->m_mark != mark)) {
			ndEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);

			edgePerimeters[perimeterCount] = edge;
			perimeterCount++;
			dAssert(perimeterCount < ndInt32(sizeof(edgePerimeters) / sizeof(edgePerimeters[0])));
		}
	}

	ndInt8 buffer[2048 * sizeof (ndFloat64)];
	ndDownHeap<ndEdge*, ndFloat64> heap(&buffer[0], sizeof (buffer));
	for (ndInt32 i = 0; i < perimeterCount; i ++) {
		ndEdge* edge = edgePerimeters[i];
		ndEdge* ptr = edge;
		ndBigVector p0 (ndBigVector::m_triplexMask & ndBigVector(&vertex[ptr->m_incidentVertex * stride]) );
		ndBigVector p1 (ndBigVector::m_triplexMask & ndBigVector(&vertex[ptr->m_next->m_incidentVertex * stride]));
		ndBigVector e0 ((p1 - p0));
		e0 = e0.Scale (ndFloat32(1.0f) / sqrt (e0.DotProduct(e0).GetScalar() + ndFloat32 (1.0e-12f)));
		ndInt32 ignoreTest = 1;
		do {
			ignoreTest = 0;
			ndBigVector p2 (ndBigVector::m_triplexMask & ndBigVector(&vertex[ptr->m_next->m_next->m_incidentVertex * stride]));
			ndBigVector e1 (p2 - p1);
			//e1 = e1.Scale (dRsqrt (e1.DotProduct3(e1) + ndFloat32 (1.0e-12f)));
			e1 = e1.Scale(ndFloat32(1.0f) / sqrt(e1.DotProduct(e1).GetScalar() + ndFloat32(1.0e-12f)));
			ndFloat64 dot = e1.DotProduct(e0).GetScalar();
			if (dot > ndFloat32 (ndFloat32 (0.9999f))) 
			{
				for (ndEdge* interiorEdge = ptr->m_next->m_twin->m_next; interiorEdge != ptr->m_twin; interiorEdge = ptr->m_next->m_twin->m_next) 
				{
					dAssert((interiorEdge->m_incidentFace > 0) && (interiorEdge->m_twin->m_incidentFace > 0));
					if ((interiorEdge->m_incidentFace > 0) && (interiorEdge->m_twin->m_incidentFace > 0)) 
					{
						flatFace.DeleteEdge(interiorEdge);
					} 
					else 
					{
						return;
					}
				} 

				if (ptr->m_twin->m_next->m_next->m_next == ptr->m_twin) {
					dAssert (ptr->m_twin->m_next->m_incidentFace > 0);
					flatFace.DeleteEdge (ptr->m_twin->m_next);
				}

				dAssert (ptr->m_next->m_twin->m_next->m_twin == ptr);
				edge = ptr->m_next;

				if (!flatFace.FindEdge (ptr->m_incidentVertex, edge->m_twin->m_incidentVertex) && 
					!flatFace.FindEdge (edge->m_twin->m_incidentVertex, ptr->m_incidentVertex)) {
						ptr->m_twin->m_prev = edge->m_twin->m_prev;
						edge->m_twin->m_prev->m_next = ptr->m_twin;

						edge->m_next->m_prev = ptr;
						ptr->m_next = edge->m_next;

						edge->m_next = edge->m_twin;
						edge->m_prev = edge->m_twin;
						edge->m_twin->m_next = edge;
						edge->m_twin->m_prev = edge;
						flatFace.DeleteEdge (edge);								
						flatFace.ChangeEdgeIncidentVertex (ptr->m_twin, ptr->m_next->m_incidentVertex);

						if (!flatFace.IsFaceConvex(ptr->m_twin, vertex, stride * sizeof(ndFloat64))) {
							heap.Flush();
							flatFace.TriangulateFace(ptr->m_twin, vertex, stride, heap, nullptr);
						}

						e1 = e0;
						p1 = p2;
						edge = ptr;
						ignoreTest = 1;
						continue;
				}
			}

			e0 = e1;
			p1 = p2;
			ptr = ptr->m_next;
		} while ((ptr != edge) || ignoreTest);
	}
}

void ndPolyhedra::RemoveInteriorColinearEdges(ndPolyhedra& flatFace, const ndFloat64* const vertex, ndInt32 stride)
{
	bool foundEdge = true;
	while (foundEdge) {
		foundEdge = false;
		ndPolyhedra::Iterator iter(flatFace);
		for (iter.Begin(); iter; iter++) {
			ndEdge* const edge = &(*iter);
			if ((edge->m_incidentFace > 0) && (edge->m_twin->m_incidentFace > 0)) {
				if (edge->m_twin->m_next->m_twin->m_next == edge) {
					ndBigVector p0(ndBigVector::m_triplexMask & ndBigVector(&vertex[edge->m_prev->m_incidentVertex * stride]));
					ndBigVector p1(ndBigVector::m_triplexMask & ndBigVector(&vertex[edge->m_incidentVertex * stride]));
					ndBigVector p2(ndBigVector::m_triplexMask & ndBigVector(&vertex[edge->m_next->m_incidentVertex * stride]));
					
					ndBigVector e0(p1 - p0);
					ndBigVector e1(p2 - p1);
					e0 = e0.Scale(ndFloat32 (1.0f) / sqrt(e0.DotProduct(e0).GetScalar() + ndFloat32(1.0e-12f)));
					e1 = e1.Scale(ndFloat32 (1.0f) / sqrt(e1.DotProduct(e1).GetScalar() + ndFloat32(1.0e-12f)));
					ndFloat64 dot = e1.DotProduct(e0).GetScalar();
					if (dot > ndFloat32(0.9999f)) 
					{
						ndInt32 v = edge->m_twin->m_incidentVertex;
						ndEdge* const nextEdge = edge->m_twin->m_next;
						edge->m_next->m_prev = edge->m_prev;
						edge->m_prev->m_next = edge->m_next;
						edge->m_twin->m_next->m_prev = edge->m_twin->m_prev;
						edge->m_twin->m_prev->m_next = edge->m_twin->m_next;

						edge->m_next = edge->m_twin;
						edge->m_prev = edge->m_twin;
						edge->m_twin->m_next = edge;
						edge->m_twin->m_prev = edge;
						flatFace.DeleteEdge(edge);
						flatFace.ChangeEdgeIncidentVertex(nextEdge, v);
						foundEdge = true;
						break;
					}
				}
			}
		}
	}
}

ndInt32 ndPolyhedra::GetInteriorDiagonals (ndPolyhedra& polyhedra, ndEdge** const diagonals, ndInt32 maxCount)
{
	ndInt32 count = 0;
	ndInt32 mark = polyhedra.IncLRU();
	ndPolyhedra::Iterator iter (polyhedra);
	for (iter.Begin(); iter; iter++) {
		ndEdge* const edge = &(*iter);
		if (edge->m_mark != mark) { 
			if (edge->m_incidentFace > 0) {
				if (edge->m_twin->m_incidentFace > 0) {
					edge->m_twin->m_mark = mark;
					if (count < maxCount){
						diagonals[count] = edge;
						count ++;
					}
					dAssert (count <= maxCount);
				}
			}
		}
		edge->m_mark = mark;
	}

	return count;
}

bool ndPolyhedra::IsEssensialPointDiagonal (ndEdge* const diagonal, const ndBigVector& normal, const ndFloat64* const pool, ndInt32 stride)
{
	if (diagonal->m_twin->m_next->m_twin->m_next != diagonal) {
		ndBigVector p0 (ndBigVector::m_triplexMask & ndBigVector(&pool[diagonal->m_incidentVertex * stride]));
		ndBigVector p1 (ndBigVector::m_triplexMask & ndBigVector(&pool[diagonal->m_twin->m_next->m_twin->m_incidentVertex * stride]));
		ndBigVector p2 (ndBigVector::m_triplexMask & ndBigVector(&pool[diagonal->m_prev->m_incidentVertex * stride]));

		ndBigVector e1 (p1 - p0);
		ndFloat64 dot = e1.DotProduct(e1).GetScalar();
		if (dot < ndFloat64 (1.0e-12f)) {
			return false;
		}
		e1 = e1.Scale (ndFloat64 (1.0f) / sqrt(dot));

		ndBigVector e2 (p2 - p0);
		dot = e2.DotProduct(e2).GetScalar();
		if (dot < ndFloat64 (1.0e-12f)) {
			return false;
		}
		e2 = e2.Scale (ndFloat64 (1.0f) / sqrt(dot));

		ndBigVector n1 (e1.CrossProduct(e2)); 
		dAssert(normal.m_w == ndFloat32(0.0f));
		dot = normal.DotProduct(n1).GetScalar();
		if (dot >= ndFloat64 (0.0f)) {
			return false;
		}
	}
	return true;
}

bool ndPolyhedra::IsEssensialDiagonal (ndEdge* const diagonal, const ndBigVector& normal, const ndFloat64* const pool,  ndInt32 stride)
{
	return IsEssensialPointDiagonal (diagonal, normal, pool, stride) || IsEssensialPointDiagonal (diagonal->m_twin, normal, pool, stride); 
}

ndBigPlane ndPolyhedra::EdgePlane (ndInt32 i0, ndInt32 i1, ndInt32 i2, const ndBigVector* const pool) const
{
	const ndBigVector& p0 = pool[i0];
	const ndBigVector& p1 = pool[i1];
	const ndBigVector& p2 = pool[i2];

	ndBigPlane plane (p0, p1, p2);
	ndFloat64 mag = sqrt (plane.DotProduct(plane & ndBigPlane::m_triplexMask).GetScalar());
	if (mag < ndFloat64 (1.0e-12f)) {
		mag = ndFloat64 (1.0e-12f);
	}
	mag = ndFloat64 (1.0f) / mag;

	plane.m_x *= mag;
	plane.m_y *= mag;
	plane.m_z *= mag;
	plane.m_w *= mag;

	return plane;
}

void ndPolyhedra::CalculateVertexMetrics (ndVertexCollapseVertexMetric* const table, const ndBigVector* const pool, ndEdge* const edge) const
{
	ndInt32 i0 = edge->m_incidentVertex;

	table[i0].Clear ();
	ndEdge* ptr = edge;
	do {

		if (ptr->m_incidentFace > 0) {
			ndInt32 i1 = ptr->m_next->m_incidentVertex;
			ndInt32 i2 = ptr->m_prev->m_incidentVertex;
			ndBigPlane constrainPlane (EdgePlane (i0, i1, i2, pool));
			table[i0].Accumulate (constrainPlane);

		} else {
			ndInt32 i1 = ptr->m_twin->m_incidentVertex;
			ndInt32 i2 = ptr->m_twin->m_prev->m_incidentVertex;
			ndBigPlane constrainPlane (UnboundedLoopPlane (i0, i1, i2, pool));
			table[i0].Accumulate (constrainPlane);

			i1 = ptr->m_prev->m_incidentVertex;
			i2 = ptr->m_prev->m_twin->m_prev->m_incidentVertex;
			constrainPlane = UnboundedLoopPlane (i0, i1, i2, pool);
			table[i0].Accumulate (constrainPlane);
		}

		ptr = ptr->m_twin->m_next;
	} while (ptr != edge);
}

void ndPolyhedra::CalculateAllMetrics (ndVertexCollapseVertexMetric* const table, const ndBigVector* const pool) const
{
	ndInt32 edgeMark = IncLRU();
	ndPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		ndEdge* const edge = &(*iter);

		dAssert (edge);
		if (edge->m_mark != edgeMark) {

			if (edge->m_incidentFace > 0) {
				ndInt32 i0 = edge->m_incidentVertex;
				ndInt32 i1 = edge->m_next->m_incidentVertex;
				ndInt32 i2 = edge->m_prev->m_incidentVertex;

				ndBigPlane constrainPlane (EdgePlane (i0, i1, i2, pool));
				ndVertexCollapseVertexMetric tmp (constrainPlane);

				ndEdge* ptr = edge;
				do {
					ptr->m_mark = edgeMark;
					i0 = ptr->m_incidentVertex;
					table[i0].Accumulate(tmp);

					ptr = ptr->m_next;
				} while (ptr != edge);

			} else {
				dAssert (edge->m_twin->m_incidentFace > 0);
				ndInt32 i0 = edge->m_twin->m_incidentVertex;
				ndInt32 i1 = edge->m_twin->m_next->m_incidentVertex;
				ndInt32 i2 = edge->m_twin->m_prev->m_incidentVertex;

				edge->m_mark = edgeMark;
				ndBigPlane constrainPlane (UnboundedLoopPlane (i0, i1, i2, pool));
				ndVertexCollapseVertexMetric tmp (constrainPlane);

				i0 = edge->m_incidentVertex;
				table[i0].Accumulate(tmp);

				i0 = edge->m_twin->m_incidentVertex;
				table[i0].Accumulate(tmp);
			}
		}
	}
}

bool ndPolyhedra::IsOkToCollapse (const ndBigVector* const pool, ndEdge* const edge) const
{
	const ndBigVector& q = pool[edge->m_incidentVertex];
	const ndBigVector& p = pool[edge->m_twin->m_incidentVertex];
	for (ndEdge* triangle = edge->m_prev->m_twin; triangle != edge->m_twin->m_next; triangle = triangle->m_prev->m_twin) {
		if (triangle->m_incidentFace > 0) {
			dAssert ((edge->m_incidentFace < 0) || (edge->m_incidentVertex == edge->m_next->m_next->m_next->m_incidentVertex));

			ndBigVector originalArea (ndBigVector::m_triplexMask & (pool[triangle->m_next->m_incidentVertex] - q).CrossProduct(pool[triangle->m_prev->m_incidentVertex] - q));
			ndBigVector newArea (ndBigVector::m_triplexMask & (pool[triangle->m_next->m_incidentVertex] - p).CrossProduct(pool[triangle->m_prev->m_incidentVertex] - p));

			ndFloat64 projectedArea = newArea.DotProduct(originalArea).GetScalar();
			if (projectedArea <= ndFloat64 (0.0f)) {
				return false;
			}

			ndFloat64 mag20 = newArea.DotProduct(newArea).GetScalar();
			ndFloat64 mag21 = originalArea.DotProduct(originalArea).GetScalar();;
			if ((projectedArea * projectedArea)  < (mag20 * mag21 * ndFloat64 (1.0e-10f)))  {
				return false;
			}
		}
	}

	return true;
}

ndEdge* ndPolyhedra::OptimizeCollapseEdge (ndEdge* const edge)
{
	ndInt32 v0 = edge->m_incidentVertex;
	ndInt32 v1 = edge->m_twin->m_incidentVertex;

#ifdef _DEBUG
	ndPolyhedra::ndPairKey TwinKey (v1, v0);
	ndPolyhedra::ndNode* const node = Find (TwinKey.GetVal());
	ndEdge* const twin1 = node ? &node->GetInfo() : nullptr;
	dAssert (twin1);
	dAssert (edge->m_twin == twin1);
	dAssert (twin1->m_twin == edge);
	dAssert (edge->m_incidentFace != 0);
	dAssert (twin1->m_incidentFace != 0);
	dAssert ((edge->m_incidentFace < 0) || (edge->m_incidentVertex == edge->m_next->m_next->m_next->m_incidentVertex));
	dAssert ((edge->m_twin->m_incidentFace < 0) || (edge->m_twin->m_incidentVertex == edge->m_twin->m_next->m_next->m_next->m_incidentVertex));
#endif

	ndEdge* retEdge = edge->m_twin->m_prev->m_twin;
	if (retEdge	== edge->m_twin->m_next) {
		return nullptr;
	}
	if (retEdge	== edge->m_twin) {
		return nullptr;
	}
	if (retEdge	== edge->m_next) {
		retEdge = edge->m_prev->m_twin;
		if (retEdge	== edge->m_twin->m_next) {
			return nullptr;
		}
		if (retEdge	== edge->m_twin) {
			return nullptr;
		}
	}

	ndEdge* lastEdge = nullptr;
	ndEdge* firstEdge = nullptr;
	if ((edge->m_incidentFace >= 0)	&& (edge->m_twin->m_incidentFace >= 0)) {	
		lastEdge = edge->m_prev->m_twin;
		firstEdge = edge->m_twin->m_next->m_twin->m_next;
	} else if (edge->m_twin->m_incidentFace >= 0) {
		firstEdge = edge->m_twin->m_next->m_twin->m_next;
		lastEdge = edge;
	} else {
		lastEdge = edge->m_prev->m_twin;
		firstEdge = edge->m_twin->m_next;
	}

	for (ndEdge* ptr = firstEdge; ptr != lastEdge; ptr = ptr->m_twin->m_next) {
		ndEdge* badEdge = FindEdge (edge->m_twin->m_incidentVertex, ptr->m_twin->m_incidentVertex);
		if (badEdge) {
			return nullptr;
		}
	} 

	ndEdge* const twin = edge->m_twin;
	if (twin->m_next == twin->m_prev->m_prev) {
		twin->m_prev->m_twin->m_twin = twin->m_next->m_twin;
		twin->m_next->m_twin->m_twin = twin->m_prev->m_twin;

		RemoveHalfEdge (twin->m_prev);
		RemoveHalfEdge (twin->m_next);
	} else {
		twin->m_next->m_prev = twin->m_prev;
		twin->m_prev->m_next = twin->m_next;
	}

	if (edge->m_next == edge->m_prev->m_prev) {
		edge->m_next->m_twin->m_twin = edge->m_prev->m_twin;
		edge->m_prev->m_twin->m_twin = edge->m_next->m_twin;
		RemoveHalfEdge (edge->m_next);
		RemoveHalfEdge (edge->m_prev);
	} else {
		edge->m_next->m_prev = edge->m_prev;
		edge->m_prev->m_next = edge->m_next;
	}

	dAssert (twin->m_twin->m_incidentVertex == v0);
	dAssert (edge->m_twin->m_incidentVertex == v1);
	RemoveHalfEdge (twin);
	RemoveHalfEdge (edge);

	ndEdge* remapPtr = retEdge;
	do {
		ndPolyhedra::ndPairKey pairKey (v0, remapPtr->m_twin->m_incidentVertex);
		ndPolyhedra::ndNode* const pairEdgeNode = Find (pairKey.GetVal());
		if (pairEdgeNode) {
			if (&pairEdgeNode->GetInfo() == remapPtr) {
				ndPolyhedra::ndPairKey key (v1, remapPtr->m_twin->m_incidentVertex);
				remapPtr->m_incidentVertex = v1;
				ReplaceKey (pairEdgeNode, key.GetVal());
			} 
		}

		ndPolyhedra::ndPairKey twinKey1 (remapPtr->m_twin->m_incidentVertex, v0);
		ndPolyhedra::ndNode* const pairTwinNode = Find (twinKey1.GetVal());
		if (pairTwinNode) {
			if (&pairTwinNode->GetInfo() == remapPtr->m_twin) {
				ndPolyhedra::ndPairKey key (remapPtr->m_twin->m_incidentVertex, v1);
				ReplaceKey (pairTwinNode, key.GetVal());
			}
		}

		remapPtr = remapPtr->m_twin->m_next;
	} while (remapPtr != retEdge);

	return retEdge;
}

ndFloat64 ndPolyhedra::EdgePenalty (const ndBigVector* const pool, ndEdge* const edge, ndFloat64 dist) const
{
	ndInt32 i0 = edge->m_incidentVertex;
	ndInt32 i1 = edge->m_next->m_incidentVertex;

	ndFloat32 maxPenalty = ndFloat32 (1.0e14f);

	const ndBigVector& p0 = pool[i0];
	const ndBigVector& p1 = pool[i1];
	ndBigVector dp (p1 - p0);

	dAssert(dp.m_w == ndFloat32(0.0f));
	ndFloat64 dot = dp.DotProduct(dp).GetScalar();;
	if (dot < ndFloat64(1.0e-6f)) {
		return dist * maxPenalty;
	}

	if ((edge->m_incidentFace > 0) && (edge->m_twin->m_incidentFace > 0)) {
		ndBigVector edgeNormal (FaceNormal (edge, &pool[0].m_x, sizeof (ndBigVector)));
		ndBigVector twinNormal (FaceNormal (edge->m_twin, &pool[0].m_x, sizeof (ndBigVector)));

		ndFloat64 mag0 = edgeNormal.DotProduct(edgeNormal).GetScalar();
		ndFloat64 mag1 = twinNormal.DotProduct(twinNormal).GetScalar();
		if ((mag0 < ndFloat64 (1.0e-24f)) || (mag1 < ndFloat64 (1.0e-24f))) {
			return dist * maxPenalty;
		}

		edgeNormal = edgeNormal.Scale (ndFloat64 (1.0f) / sqrt(mag0));
		twinNormal = twinNormal.Scale (ndFloat64 (1.0f) / sqrt(mag1));

		dot = edgeNormal.DotProduct(twinNormal).GetScalar();;
		if (dot < ndFloat64 (-0.9f)) {
			return dist * maxPenalty;
		}

		ndEdge* ptr = edge;
		do {
			if ((ptr->m_incidentFace <= 0) || (ptr->m_twin->m_incidentFace <= 0)){
				ndEdge* const adj = edge->m_twin;
				ptr = edge;
				do {
					if ((ptr->m_incidentFace <= 0) || (ptr->m_twin->m_incidentFace <= 0)){
						return dist * maxPenalty;
					}
					ptr = ptr->m_twin->m_next;
				} while (ptr != adj);
			}
			ptr = ptr->m_twin->m_next;
		} while (ptr != edge);
	}

	ndInt32 faceA = edge->m_incidentFace;
	ndInt32 faceB = edge->m_twin->m_incidentFace;

	i0 = edge->m_twin->m_incidentVertex;
	ndBigVector p (pool[i0].m_x, pool[i0].m_y, pool[i0].m_z, ndFloat32 (0.0f));

	bool penalty = false;
	ndEdge* ptr = edge;
	do {
		ndEdge* const adj = ptr->m_twin;

		ndInt32 face = adj->m_incidentFace;
		if ((face != faceB) && (face != faceA) && (face >= 0) && (adj->m_next->m_incidentFace == face) && (adj->m_prev->m_incidentFace == face)){

			ndInt32 k0 = adj->m_next->m_incidentVertex;
			const ndBigVector& q0 = pool[k0];

			ndInt32 k1 = adj->m_incidentVertex;
			const ndBigVector& q1 = pool[k1];

			ndInt32 k2 = adj->m_prev->m_incidentVertex;
			const ndBigVector& q2 = pool[k2];

			ndBigVector n0 (ndBigVector::m_triplexMask & (q1 - q0).CrossProduct(q2 - q0));
			ndBigVector n1 (ndBigVector::m_triplexMask & (q1 - p).CrossProduct(q2 - p));
			ndFloat64 project = n0.DotProduct(n1).GetScalar();
			if (project < ndFloat64 (0.0f)) {
				penalty = true;
				break;
			}
		}

		ptr = ptr->m_twin->m_next;
	} while (ptr != edge);

	ndFloat64 aspect = ndFloat32 (0.0f);
	if (!penalty) {
		ndInt32 k0 = edge->m_twin->m_incidentVertex;
		ndBigVector q0 (pool[k0]);

		aspect = ndFloat32 (1.0f);
		for (ndEdge* ptr1 = edge->m_twin->m_next->m_twin->m_next; ptr1 != edge; ptr1 = ptr1->m_twin->m_next) {
			if (ptr1->m_incidentFace > 0) {
				ndInt32 k1 = ptr1->m_next->m_incidentVertex;
				const ndBigVector& q1 = pool[k1];

				ndInt32 k2 = ptr1->m_prev->m_incidentVertex;
				const ndBigVector& q2 = pool[k2];

				ndBigVector e0 (q1 - q0);
				ndBigVector e1 (q2 - q1);
				ndBigVector e2 (q0 - q2);
				dAssert(e0.m_w == ndFloat32(0.0f));
				dAssert(e1.m_w == ndFloat32(0.0f));
				dAssert(e2.m_w == ndFloat32(0.0f));

				ndFloat64 mag0 = e0.DotProduct(e0).GetScalar();
				ndFloat64 mag1 = e1.DotProduct(e1).GetScalar();
				ndFloat64 mag2 = e2.DotProduct(e2).GetScalar();
				ndFloat64 maxMag = dMax(dMax (mag0, mag1), mag2);
				ndFloat64 minMag = dMin(dMin (mag0, mag1), mag2);
				ndFloat64 ratio = minMag / maxMag;

				if (ratio < aspect) {
					aspect = ratio;
				}
			}
		}
		aspect = ndFloat32 (1.0f) - aspect;
	}
	return aspect * aspect * dist;
}

bool ndPolyhedra::Optimize (const ndFloat64* const array, ndInt32 strideInBytes, ndFloat64 tol, ndInt32 maxFaceCount)
{
	ndInt32 stride = ndInt32 (strideInBytes / sizeof (ndFloat64));

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
	dAssert (SanityCheck ());
#endif

	ndFloat32 progressDen = ndFloat32 (1.0f / GetEdgeCount());
	ndInt32 edgeCount = GetEdgeCount() * 4 + D_LOCAL_BUFFER_SIZE * 16;
	ndInt32 maxVertexIndex = GetLastVertexIndex();
	
	ndStack<ndBigVector> vertexPool (maxVertexIndex); 
	ndStack<ndVertexCollapseVertexMetric> vertexMetrics (maxVertexIndex + 512); 

	ndList <dEdgeCollapseEdgeHandle> edgeHandleList;
	ndStack<char> heapPool (2 * edgeCount * ndInt32 (sizeof (ndFloat64) + sizeof (dEdgeCollapseEdgeHandle*) + sizeof (ndInt32))); 
	ndUpHeap<ndList <dEdgeCollapseEdgeHandle>::ndNode* , ndFloat64> bigHeapArray(&heapPool[0], heapPool.GetSizeInBytes());

	for (ndInt32 i = 0; i < maxVertexIndex; i ++) 
	{
		vertexPool[i].m_x = array[i * stride + 0];
		vertexPool[i].m_y = array[i * stride + 1];
		vertexPool[i].m_z = array[i * stride + 2];
		vertexPool[i].m_w= ndFloat64 (0.0f);
	}

	memset (&vertexMetrics[0], 0, maxVertexIndex * sizeof (ndVertexCollapseVertexMetric));
	CalculateAllMetrics (&vertexMetrics[0], &vertexPool[0]);

	const ndFloat64 maxCost = ndFloat32 (1.0e-3f);
	ndFloat64 tol2 = tol * tol;
	ndFloat64 distTol = dMax (tol2, ndFloat64 (1.0e-12f));
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) 
	{
		ndEdge* const edge = &(*iter);

		edge->m_userData = 0;
		ndInt32 index0 = edge->m_incidentVertex;
		ndInt32 index1 = edge->m_twin->m_incidentVertex;

		ndVertexCollapseVertexMetric &metric = vertexMetrics[index0];
		const ndBigVector& p = vertexPool[index1];
		ndFloat64 faceCost = metric.Evalue (p); 
		ndFloat64 edgePenalty = EdgePenalty (&vertexPool[0], edge, distTol);
		dAssert (edgePenalty >= ndFloat32 (0.0f));
		dEdgeCollapseEdgeHandle handle (edge);
		ndList <dEdgeCollapseEdgeHandle>::ndNode* handleNodePtr = edgeHandleList.Addtop (handle);
		bigHeapArray.Push (handleNodePtr, faceCost + edgePenalty);
	}

	bool progress = true;
	ndInt32 interPasses = 0;
	ndInt32 faceCount = GetFaceCount();
	while (bigHeapArray.GetCount() && (bigHeapArray.Value() < maxCost) && ((bigHeapArray.Value() < tol2) || (faceCount > maxFaceCount)) && progress ) 
	{
		ndList <dEdgeCollapseEdgeHandle>::ndNode* const handleNodePtr = bigHeapArray[0];

		ndEdge* edge = handleNodePtr->GetInfo().m_edge;
		bigHeapArray.Pop();
		edgeHandleList.Remove (handleNodePtr);

		if (edge) 
		{
			if (IsOkToCollapse (&vertexPool[0], edge)) 
			{
				interPasses++;
				faceCount -= 2;
				if (interPasses >= 400)
				{
					interPasses = 0;
					faceCount = GetFaceCount();
					progress = ReportProgress(ndFloat32(1.0f) - GetEdgeCount() * progressDen);
				}

				if (bigHeapArray.GetCount() > (bigHeapArray.GetMaxCount() - 100)) 
				{
					for(ndInt32 i = bigHeapArray.GetCount() - 1; i >= 0; i --) 
					{
						ndList <dEdgeCollapseEdgeHandle>::ndNode* const emptyHandle = bigHeapArray[i];
						if (!emptyHandle->GetInfo().m_edge) 
						{
							bigHeapArray.Remove(i);
							edgeHandleList.Remove (emptyHandle);
						}
					}
				}

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
				dAssert (SanityCheck ());
#endif

				edge = OptimizeCollapseEdge(edge);

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
				dAssert (SanityCheck ());
#endif
				if (edge) 
				{
					// Update vertex metrics
					CalculateVertexMetrics (&vertexMetrics[0], &vertexPool[0], edge);

					// Update metrics for all surrounding vertex
					ndEdge* ptr = edge;
					do 
					{
						CalculateVertexMetrics (&vertexMetrics[0], &vertexPool[0], ptr->m_twin);
						ptr = ptr->m_twin->m_next;
					} while (ptr != edge);

					// calculate edge cost of all incident edges
					ndInt32 mark = IncLRU();
					ptr = edge;
					do 
					{
						dAssert (ptr->m_mark != mark);
						ptr->m_mark = mark;

						ndInt32 index0 = ptr->m_incidentVertex;
						ndInt32 index1 = ptr->m_twin->m_incidentVertex;

						ndVertexCollapseVertexMetric &metric = vertexMetrics[index0];
						const ndBigVector& p = vertexPool[index1];

						ndFloat64 faceCost = metric.Evalue (p); 
						ndFloat64 edgePenalty = EdgePenalty (&vertexPool[0], ptr, distTol);
						dAssert (edgePenalty >= ndFloat32 (0.0f));
						dEdgeCollapseEdgeHandle handle (ptr);
						ndList <dEdgeCollapseEdgeHandle>::ndNode* handleNodePtr1 = edgeHandleList.Addtop (handle);
						bigHeapArray.Push (handleNodePtr1, faceCost + edgePenalty);

						ptr = ptr->m_twin->m_next;
					} while (ptr != edge);

					// calculate edge cost of all incident edges to a surrounding vertex
					ptr = edge;
					do 
					{
						ndEdge* const incidentEdge = ptr->m_twin;		

						ndEdge* ptr1 = incidentEdge;
						do 
						{
							ndInt32 index0 = ptr1->m_incidentVertex;
							ndInt32 index1 = ptr1->m_twin->m_incidentVertex;

							if (ptr1->m_mark != mark) 
							{
								ptr1->m_mark = mark;
								ndVertexCollapseVertexMetric &metric = vertexMetrics[index0];
								const ndBigVector& p = vertexPool[index1];

								ndFloat64 faceCost = metric.Evalue (p); 
								ndFloat64 edgePenalty = EdgePenalty (&vertexPool[0], ptr1, distTol);
								dAssert (edgePenalty >= ndFloat32 (0.0f));
								dEdgeCollapseEdgeHandle handle (ptr1);
								ndList <dEdgeCollapseEdgeHandle>::ndNode* handleNodePtr1 = edgeHandleList.Addtop (handle);
								bigHeapArray.Push (handleNodePtr1, faceCost + edgePenalty);
							}

							if (ptr1->m_twin->m_mark != mark) 
							{
								ptr1->m_twin->m_mark = mark;
								ndVertexCollapseVertexMetric &metric = vertexMetrics[index1];
								const ndBigVector& p = vertexPool[index0];
								ndFloat64 faceCost = metric.Evalue (p); 
								ndFloat64 edgePenalty = EdgePenalty (&vertexPool[0], ptr1->m_twin, distTol);
								dAssert (edgePenalty >= ndFloat32 (0.0f));
								dEdgeCollapseEdgeHandle handle (ptr1->m_twin);
								ndList <dEdgeCollapseEdgeHandle>::ndNode* handleNodePtr1 = edgeHandleList.Addtop (handle);
								bigHeapArray.Push (handleNodePtr1, faceCost + edgePenalty);
							}

							ptr1 = ptr1->m_twin->m_next;
						} while (ptr1 != incidentEdge);

						ptr = ptr->m_twin->m_next;
					} while (ptr != edge);
				}
			}
		}
	}

	progress = ReportProgress(ndFloat32 (1.0f));
	return progress;
}

bool ndPolyhedra::TriangulateFace(ndEdge* const face, const ndFloat64* const pool, ndInt32 strideInBytes)
{
	if (face->m_next->m_next->m_next != face) 
	{
		ndInt32 mark = IncLRU();
		ndEdge* ptr = face;
		do 
		{
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != face);
		char memPool[D_LOCAL_BUFFER_SIZE * (sizeof (ndEdge*)+sizeof (ndFloat64))];
		ndDownHeap<ndEdge*, ndFloat64> heap(&memPool[0], sizeof (memPool));

		ndInt32 stride = ndInt32(strideInBytes / sizeof (ndFloat64));
		ndEdge* const edge = TriangulateFace(face, pool, stride, heap, nullptr);
		dAssert(!edge);
		return !edge;
	}
	return true;
}

ndEdge* ndPolyhedra::BestEdgePolygonizeFace(const ndBigVector& normal, ndEdge* const edge, const ndFloat64* const pool, ndInt32 stride, const ndBigVector& point) const
{
	ndBigVector p0(ndBigVector::m_triplexMask & ndBigVector(&pool[edge->m_incidentVertex * stride]));
	ndBigVector r(point - p0);
	ndEdge* e0 = edge;
	do 
	{
		ndBigVector p1(ndBigVector::m_triplexMask & ndBigVector(&pool[e0->m_twin->m_incidentVertex * stride]));
		ndBigVector p2(ndBigVector::m_triplexMask & ndBigVector(&pool[e0->m_prev->m_incidentVertex * stride]));
		ndFloat64 test0 = r.DotProduct(normal.CrossProduct(p1 - p0)).GetScalar();
		ndFloat64 test1 = r.DotProduct((p2 - p0).CrossProduct(normal)).GetScalar();
		
		if ((test0 > 0.0f) && (test1 > 0.0f)) 
		{
			break;
		}
		e0 = e0->m_prev->m_twin;
	} while (e0 != edge);
	return e0;
}

bool ndPolyhedra::PolygonizeFace(ndEdge* const face, const ndFloat64* const pool, ndInt32 strideInBytes)
{
	ndPolyhedra flatFace;
	ndEdge* array[D_LOCAL_BUFFER_SIZE];

	ndInt32 count = 0;		
	ndEdge* edge = face;
	do 
	{
		ndEdge* const perimeter = flatFace.AddHalfEdge (edge->m_incidentVertex, edge->m_twin->m_incidentVertex);
		dAssert (perimeter);
		perimeter->m_userData = edge->m_userData;
		perimeter->m_incidentFace = 1;
		perimeter->m_twin = nullptr;
		perimeter->m_prev = nullptr;
		perimeter->m_next = nullptr;

		array[count] = perimeter;
		count++;
		dAssert(count <= D_LOCAL_BUFFER_SIZE);
		edge = edge->m_next;
	} while (edge != face);

	ndInt32 i0 = count - 1;
	for(ndInt32 i = 0; i < count; i ++) 
	{
		ndEdge* const edge1 = array[i];
		ndEdge* const prev1 = array[i0];

		edge1->m_prev = prev1;
		prev1->m_next = edge1;
		i0 = i;
	} 

	for(ndInt32 i = 0; i < count; i ++) 
	{
		ndEdge* const edge1 = array[i];
		ndEdge* const twin1 = flatFace.FindEdge (edge1->m_next->m_incidentVertex, edge1->m_incidentVertex);
		if (twin1) 
		{
			twin1->m_twin = edge1;
			edge1->m_twin = twin1;
		} 
		else 
		{
			ndEdge* const perimeter = flatFace.AddHalfEdge (edge1->m_next->m_incidentVertex, edge1->m_incidentVertex);
			perimeter->m_twin = edge1;
			edge1->m_twin = perimeter;
			perimeter->m_incidentFace = -1;
			perimeter->m_prev = nullptr;
			perimeter->m_next = nullptr;
		}
	}

	for (ndInt32 i = 0; i < count; i++) 
	{
		ndEdge* const edge1 = array[i];
		ndEdge* const twin1 = edge1->m_twin;
		if (!twin1->m_next) 
		{
			ndEdge* next = edge1->m_prev->m_twin;
			while (next->m_prev) 
			{
				next = next->m_prev->m_twin;
			}
			twin1->m_next = next;	
			next->m_prev = next;
		}
	}

	ndBigVector normal (flatFace.FaceNormal(array[0], pool, strideInBytes));
	if (flatFace.TriangulateFace(array[0], pool, strideInBytes)) 
	{
		ndInt32 stride = ndInt32(strideInBytes / sizeof (ndFloat64));
		flatFace.RefineTriangulation(pool, stride);

		//RemoveOuterColinearEdges(*this, vertex, stride);
		ndInt32 polygon[D_LOCAL_BUFFER_SIZE];
		ndEdge* diagonalsPool[D_LOCAL_BUFFER_SIZE];

		ndInt32 diagonalCount = GetInteriorDiagonals(flatFace, diagonalsPool, sizeof (diagonalsPool) / sizeof (diagonalsPool[0]));

		if (diagonalCount) 
		{
			ndEdge* edge1 = &flatFace.GetRoot()->GetInfo();
			if (edge1->m_incidentFace < 0) 
			{
				edge1 = edge1->m_twin;
			}

			dAssert(edge1->m_incidentFace > 0);

			ndBigVector normal1(flatFace.FaceNormal(edge1, pool, strideInBytes));
			normal1 = normal1.Scale(ndFloat64(1.0f) / sqrt(normal1.DotProduct(normal1).GetScalar()));

			edge1 = nullptr;
			ndPolyhedra::Iterator iter0(flatFace);
			for (iter0.Begin(); iter0; iter0++) 
			{
				edge1 = &(*iter0);
				if (edge1->m_incidentFace < 0) 
				{
					break;
				}
			}
			dAssert(edge1);

			ndInt32 isConvex = 1;
			ndEdge* ptr = edge1;
			ndInt32 mark = flatFace.IncLRU();

			ndBigVector normal2(normal1);
			ndBigVector p0(ndBigVector::m_triplexMask & ndBigVector(&pool[ptr->m_prev->m_incidentVertex * stride]));
			ndBigVector p1(ndBigVector::m_triplexMask & ndBigVector(&pool[ptr->m_incidentVertex * stride]));
			ndBigVector e0(p1 - p0);

			dAssert(normal2.m_w == ndFloat32(0.0f));
			e0 = e0.Scale(ndFloat64(1.0f) / sqrt(e0.DotProduct(e0).GetScalar() + ndFloat64(1.0e-24f)));
			do 
			{
				ndBigVector p2(ndBigVector::m_triplexMask & ndBigVector(&pool[ptr->m_next->m_incidentVertex * stride]));
				ndBigVector e1(p2 - p1);
				e1 = e1.Scale(ndFloat64(1.0f) / sqrt(e1.DotProduct(e1).GetScalar() + ndFloat32(1.0e-24f)));
				ndFloat64 dot = normal2.DotProduct(e0.CrossProduct(e1)).GetScalar();
				
				if (dot > ndFloat32(5.0e-3f)) 
				{
					isConvex = 0;
					break;
				}
				ptr->m_mark = mark;
				e0 = e1;
				p1 = p2;
				ptr = ptr->m_next;
			} while (ptr != edge1);

			if (isConvex) 
			{
				ndPolyhedra::Iterator iter(flatFace);
				for (iter.Begin(); iter; iter++) 
				{
					ptr = &(*iter);
					if (ptr->m_incidentFace < 0) 
					{
						if (ptr->m_mark < mark) 
						{
							isConvex = 0;
							break;
						}
					}
				}
			}

			if (isConvex) 
			{
				if (diagonalCount > 2) 
				{
					ndInt32 count1 = 0;
					ptr = edge1;
					do 
					{
						polygon[count1] = ptr->m_incidentVertex;
						count1++;
						dAssert(count1 < ndInt32(sizeof (polygon) / sizeof (polygon[0])));
						ptr = ptr->m_next;
					} while (ptr != edge1);

					for (ndInt32 i = 0; i < count1 - 1; i++) 
					{
						for (ndInt32 j = i + 1; j < count1; j++) 
						{
							if (polygon[i] == polygon[j]) 
							{
								i = count1;
								isConvex = 0;
								break;
							}
						}
					}
				}
			}

			if (isConvex) 
			{
				for (ndInt32 j = 0; j < diagonalCount; j++) 
				{
					ndEdge* const diagonal = diagonalsPool[j];
					flatFace.DeleteEdge(diagonal);
				}
			} 
			else 
			{
				for (ndInt32 j = 0; j < diagonalCount; j++) 
				{
					ndEdge* const diagonal = diagonalsPool[j];
					if (!IsEssensialDiagonal(diagonal, normal1, pool, stride)) 
					{
						flatFace.DeleteEdge(diagonal);
					}
				}
			}
		}

		ndInt32 mark = flatFace.IncLRU();
		ndPolyhedra::Iterator iter0(flatFace);
		for (iter0.Begin(); iter0; iter0++) 
		{
			ndEdge* const edge1 = &(*iter0);
			if ((edge1->m_mark != mark) && (edge1->m_incidentFace > 0)) 
			{
				edge1->m_mark = mark;
				edge1->m_twin->m_mark = mark;
				if (!FindEdge(edge1->m_incidentVertex, edge1->m_twin->m_incidentVertex)) 
				{
					ndPairKey key0 (edge1->m_incidentVertex, 0);
					ndPairKey key1 (edge1->m_twin->m_incidentVertex, 0);
					ndNode* const node0 = FindGreater (key0.GetVal());
					ndNode* const node1 = FindGreater (key1.GetVal());
					dAssert (node0);
					dAssert (node1);
					ndEdge* e0 = &node0->GetInfo();
					ndEdge* e1 = &node1->GetInfo();

					ndBigVector p0 (ndBigVector::m_triplexMask & ndBigVector(&pool[e0->m_incidentVertex * stride]));
					ndBigVector p1 (ndBigVector::m_triplexMask & ndBigVector(&pool[e1->m_incidentVertex * stride]));
					e0 = BestEdgePolygonizeFace (normal, e0, pool, stride, p1);
					e1 = BestEdgePolygonizeFace (normal, e1, pool, stride, p0);
					ConnectVertex (e0, e1);
				}
			}
		}
	}

	return true;
}

void ndPolyhedra::RemoveInteriorEdges (ndPolyhedra& buildConvex, const ndFloat64* const vertex, ndInt32 strideInBytes)
{
	ndInt32 polygon[D_LOCAL_BUFFER_SIZE * 8];
	ndEdge* diagonalsPool[D_LOCAL_BUFFER_SIZE * 8];

	ndInt32 stride = ndInt32 (strideInBytes / sizeof (ndFloat64));

	buildConvex.BeginFace();
	ndPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter;) 
	{
		ndEdge* edge = &(*iter);
		iter++;
		if (edge->m_incidentFace > 0) 
		{

			ndPolyhedra flatFace;
			MarkAdjacentCoplanarFaces(flatFace, edge, vertex, strideInBytes);
			if (flatFace.GetCount()) 
			{
				//flatFace.RefineTriangulation(vertex, stride);
				RemoveOuterColinearEdges(flatFace, vertex, stride);
				RemoveInteriorColinearEdges(flatFace, vertex, stride);
				flatFace.RefineTriangulation(vertex, stride);

				ndInt32 diagonalCount = GetInteriorDiagonals(flatFace, diagonalsPool, sizeof(diagonalsPool) / sizeof(diagonalsPool[0]));
				if (diagonalCount) 
				{
					edge = &flatFace.GetRoot()->GetInfo();
					if (edge->m_incidentFace < 0) 
					{
						edge = edge->m_twin;
					}
					dAssert(edge->m_incidentFace > 0);

					ndBigVector normal(FaceNormal(edge, vertex, strideInBytes));
					normal = normal.Scale(ndFloat64(1.0f) / sqrt(normal.DotProduct(normal).GetScalar()));

					edge = nullptr;
					ndPolyhedra::Iterator iter1(flatFace);
					for (iter1.Begin(); iter1; iter1++) 
					{
						edge = &(*iter1);
						if (edge->m_incidentFace < 0) 
						{
							break;
						}
					}
					dAssert(edge);

					ndInt32 isConvex = 1;
					ndEdge* ptr = edge;
					ndInt32 mark = flatFace.IncLRU();

					ndBigVector normal2(normal);
					ndBigVector p0(ndBigVector::m_triplexMask & ndBigVector(&vertex[ptr->m_prev->m_incidentVertex * stride]));
					ndBigVector p1(ndBigVector::m_triplexMask & ndBigVector(&vertex[ptr->m_incidentVertex * stride]));
					ndBigVector e0(p1 - p0);
					e0 = e0.Scale(ndFloat64(1.0f) / sqrt(e0.DotProduct(e0).GetScalar() + ndFloat64(1.0e-24f)));
					do 
					{
						ndBigVector p2(ndBigVector::m_triplexMask & ndBigVector(&vertex[ptr->m_next->m_incidentVertex * stride]));
						ndBigVector e1(p2 - p1);
						e1 = e1.Scale(ndFloat64(1.0f) / sqrt(e1.DotProduct(e1).GetScalar() + ndFloat32(1.0e-24f)));
						ndFloat64 dot = normal2.DotProduct(e0.CrossProduct(e1)).GetScalar();

						if (dot > ndFloat32(5.0e-3f)) 
						{
							isConvex = 0;
							break;
						}
						ptr->m_mark = mark;
						e0 = e1;
						p1 = p2;
						ptr = ptr->m_next;
					} while (ptr != edge);

					if (isConvex) 
					{
						ndPolyhedra::Iterator iter2(flatFace);
						for (iter2.Begin(); iter2; iter2++) 
						{
							ptr = &(*iter2);
							if (ptr->m_incidentFace < 0) 
							{
								if (ptr->m_mark < mark) 
								{
									isConvex = 0;
									break;
								}
							}
						}
					}

					if (isConvex) 
					{
						if (diagonalCount > 2) 
						{
							ndInt32 count = 0;
							ptr = edge;
							do 
							{
								polygon[count] = ptr->m_incidentVertex;
								count++;
								dAssert(count < ndInt32(sizeof(polygon) / sizeof(polygon[0])));
								ptr = ptr->m_next;
							} while (ptr != edge);

							for (ndInt32 i = 0; i < count - 1; i++) 
							{
								for (ndInt32 j = i + 1; j < count; j++) 
								{
									if (polygon[i] == polygon[j]) 
									{
										i = count;
										isConvex = 0;
										break;
									}
								}
							}
						}
					}

					if (isConvex) 
					{
						for (ndInt32 j = 0; j < diagonalCount; j++) 
						{
							ndEdge* const diagonal = diagonalsPool[j];
							flatFace.DeleteEdge(diagonal);
						}
					} 
					else 
					{
						for (ndInt32 j = 0; j < diagonalCount; j++) 
						{
							ndEdge* const diagonal = diagonalsPool[j];
							if (!IsEssensialDiagonal(diagonal, normal, vertex, stride)) 
							{
								flatFace.DeleteEdge(diagonal);
							}
						}
					}
				}

				ndInt32 mark = flatFace.IncLRU();
				ndPolyhedra::Iterator iter1(flatFace);
				for (iter1.Begin(); iter1; iter1++) 
				{
					ndEdge* const edge1 = &(*iter1);
					if (edge1->m_mark != mark) 
					{
						if (edge1->m_incidentFace > 0) 
						{
							ndEdge* ptr = edge1;
							ndInt32 diagonalCount1 = 0;
							do 
							{
								polygon[diagonalCount1] = ptr->m_incidentVertex;
								diagonalCount1++;
								dAssert(diagonalCount1 < ndInt32(sizeof(polygon) / sizeof(polygon[0])));
								ptr->m_mark = mark;
								ptr = ptr->m_next;
							} while (ptr != edge1);
							if (diagonalCount1 >= 3) 
							{
								buildConvex.AddFace(diagonalCount1, polygon);
							}
						}
					}
				}
			}

			iter.Begin();
		}
	}

	buildConvex.EndFace();
	dAssert(GetCount() == 0);
}

void ndPolyhedra::ConvexPartition (const ndFloat64* const vertex, ndInt32 strideInBytes, ndPolyhedra* const leftOversOut)
{
	if (GetCount()) 
	{
		Triangulate (vertex, strideInBytes, leftOversOut);
		DeleteDegenerateFaces (vertex, strideInBytes, ndFloat32 (1.0e-5f));
		Optimize (vertex, strideInBytes, ndFloat32 (1.0e-3f));
		DeleteDegenerateFaces (vertex, strideInBytes, ndFloat32 (1.0e-5f));

		if (GetCount()) 
		{
			ndPolyhedra buildConvex;
			RemoveInteriorEdges (buildConvex, vertex, strideInBytes);
			SwapInfo(buildConvex);
		}
	}
}


ndMatrix ndPolyhedra::CalculateSphere(ndBigVector& size, const ndFloat64* const vertex, ndInt32 strideInBytes) const
{
	ndInt32 stride = ndInt32(strideInBytes / sizeof(ndFloat64));

	ndInt32 vertexCount = 0;
	ndInt32 mark = IncLRU();
	ndPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &(*iter);
		if (edge->m_mark != mark) 
		{
			ndEdge* ptr = edge;
			do 
			{
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			vertexCount++;
		}
	}
	dAssert(vertexCount);

	mark = IncLRU();
	ndInt32 vertexCountIndex = 0;
	ndStack<ndBigVector> pool(vertexCount);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &(*iter);
		if (edge->m_mark != mark) 
		{
			ndEdge* ptr = edge;
			do 
			{
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			ndInt32 incidentVertex = edge->m_incidentVertex * stride;
			pool[vertexCountIndex] = ndBigVector(vertex[incidentVertex + 0], vertex[incidentVertex + 1], vertex[incidentVertex + 2], ndFloat32(0.0f));
			vertexCountIndex++;
		}
	}
	dAssert(vertexCountIndex <= vertexCount);

	//ndMatrix axis(dGetIdentityMatrix());
	//dgObb sphere(axis);
	ndConvexHull3d convexHull(&pool[0].m_x, sizeof(ndBigVector), vertexCountIndex, 0.0f);

	size = ndBigVector::m_zero;
	ndMatrix sphere(dGetIdentityMatrix());
	if (convexHull.GetCount()) 
	{
		ndStack<ndInt32> triangleList(convexHull.GetCount() * 3);
		ndInt32 trianglesCount = 0;
		for (ndConvexHull3d::ndNode* node = convexHull.GetFirst(); node; node = node->GetNext()) 
		{
			ndConvexHull3dFace* const face = &node->GetInfo();
			triangleList[trianglesCount * 3 + 0] = face->m_index[0];
			triangleList[trianglesCount * 3 + 1] = face->m_index[1];
			triangleList[trianglesCount * 3 + 2] = face->m_index[2];
			trianglesCount++;
			dAssert((trianglesCount * 3) <= triangleList.GetElementsCount());
		}
		
		//ndVector* const dst = (ndVector*)&pool[0].m_x;
		for (ndInt32 i = 0; i < convexHull.GetVertexCount(); i++) 
		{
			pool[i] = convexHull.GetVertex(i);
		}

		ndVector eigen;
		ndBigVector var(ndBigVector::m_zero);
		ndBigVector cov(ndBigVector::m_zero);
		ndBigVector origin(ndBigVector::m_zero);

		for (ndInt32 i = 0; i < vertexCount; i++) 
		{
			const ndBigVector p(pool[i] & ndBigVector::m_triplexMask);
			const ndBigVector q(p.ShiftTripleLeft());
			origin += p;
			var += p * p;
			cov += p * q;
		}
		dSwap(cov.m_y, cov.m_z);

		ndFloat64 k = ndFloat64(1.0) / vertexCount;
		var = var.Scale(k);
		cov = cov.Scale(k);
		origin = origin.Scale(k);
		
		ndFloat64 Ixx = var.m_x - origin.m_x * origin.m_x;
		ndFloat64 Iyy = var.m_y - origin.m_y * origin.m_y;
		ndFloat64 Izz = var.m_z - origin.m_z * origin.m_z;
		
		ndFloat64 Ixy = cov.m_x - origin.m_x * origin.m_y;
		ndFloat64 Ixz = cov.m_y - origin.m_x * origin.m_z;
		ndFloat64 Iyz = cov.m_z - origin.m_y * origin.m_z;
		
		sphere.m_front = ndVector(ndFloat32(Ixx), ndFloat32(Ixy), ndFloat32(Ixz), ndFloat32(0.0f));
		sphere.m_up    = ndVector(ndFloat32(Ixy), ndFloat32(Iyy), ndFloat32(Iyz), ndFloat32(0.0f));
		sphere.m_right = ndVector(ndFloat32(Ixz), ndFloat32(Iyz), ndFloat32(Izz), ndFloat32(0.0f));
		//ndVector eigenValues(sphere.EigenVectors());

		ndVector minVal(ndFloat32(1e15f));
		ndVector maxVal(ndFloat32(-1e15f));
		for (ndInt32 i = 0; i < vertexCount; i++)
		{
			ndVector tmp(sphere.UnrotateVector(pool[i]));
			minVal = minVal.GetMin(tmp);
			maxVal = maxVal.GetMax(tmp);
		}

		ndVector massCenter((maxVal + minVal) * ndVector::m_half);
		massCenter.m_w = ndFloat32(1.0f);
		sphere.m_posit = sphere.TransformVector(massCenter);
		size = ndVector ((maxVal - minVal) * ndVector::m_half);
	}
	return sphere;
}
