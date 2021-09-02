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

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dHeap.h"
#include "dPlane.h"
#include "dDebug.h"
#include "dStack.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dPolyhedra.h"
#include "dConvexHull3d.h"
#include "dSmallDeterminant.h"

#define D_LOCAL_BUFFER_SIZE  1024

#define dPointerToInt(x) ((size_t)x)
#define dIntToPointer(x) ((void*)(size_t(x)))


class dgDiagonalEdge
{
	public:
	dgDiagonalEdge (dEdge* const edge)
		:m_i0(edge->m_incidentVertex), m_i1(edge->m_twin->m_incidentVertex)
	{
	}
	dInt32 m_i0;
	dInt32 m_i1;
};

struct dEdgeCollapseEdgeHandle
{
	dEdgeCollapseEdgeHandle (dEdge* const newEdge)
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
		m_edge->m_userData = dUnsigned64 (dPointerToInt(this));
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

	dEdge* m_edge;
	dUnsigned32 m_inList;
};

class dVertexCollapseVertexMetric
{
	public:
	dVertexCollapseVertexMetric (const dBigPlane &plane) 
	{
		elem[0] = plane.m_x * plane.m_x;  
		elem[1] = plane.m_y * plane.m_y;  
		elem[2] = plane.m_z * plane.m_z;  
		elem[3] = plane.m_w * plane.m_w;  
		elem[4] = dFloat64 (2.0) * plane.m_x * plane.m_y;  
		elem[5] = dFloat64 (2.0) * plane.m_x * plane.m_z;  
		elem[6] = dFloat64 (2.0) * plane.m_x * plane.m_w;  
		elem[7] = dFloat64 (2.0) * plane.m_y * plane.m_z;  
		elem[8] = dFloat64 (2.0) * plane.m_y * plane.m_w;  
		elem[9] = dFloat64 (2.0) * plane.m_z * plane.m_w;  
	}

	void Clear ()
	{
		memset (elem, 0, 10 * sizeof (dFloat64));
	}

	void Accumulate (const dVertexCollapseVertexMetric& p) 
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

	void Accumulate (const dBigPlane& plane) 
	{
		elem[0] += plane.m_x * plane.m_x;  
		elem[1] += plane.m_y * plane.m_y;  
		elem[2] += plane.m_z * plane.m_z;  
		elem[3] += plane.m_w * plane.m_w;  

		elem[4] += dFloat64 (2.0f) * plane.m_x * plane.m_y;  
		elem[5] += dFloat64 (2.0f) * plane.m_x * plane.m_z;  
		elem[7] += dFloat64 (2.0f) * plane.m_y * plane.m_z;  

		elem[6] += dFloat64 (2.0f) * plane.m_x * plane.m_w;  
		elem[8] += dFloat64 (2.0f) * plane.m_y * plane.m_w;  
		elem[9] += dFloat64 (2.0f) * plane.m_z * plane.m_w;  
	}

	dFloat64 Evalue (const dBigVector &p) const 
	{
		dFloat64 acc = elem[0] * p.m_x * p.m_x + elem[1] * p.m_y * p.m_y + elem[2] * p.m_z * p.m_z + 
					   elem[4] * p.m_x * p.m_y + elem[5] * p.m_x * p.m_z + elem[7] * p.m_y * p.m_z + 
					   elem[6] * p.m_x + elem[8] * p.m_y + elem[9] * p.m_z + elem[3];  
		return fabs (acc);
	}

	dFloat64 elem[10];
};

dPolyhedra::dPolyhedra (const dPolyhedra &polyhedra)
	:dTree <dEdge, dInt64>()
	,m_baseMark(0)
	,m_edgeMark(0)
	,m_faceSecuence(0)
{
	dStack<dInt32> indexPool (D_LOCAL_BUFFER_SIZE * 16);
	dStack<dUnsigned64> userPool (D_LOCAL_BUFFER_SIZE * 16);
	dInt32* const index = &indexPool[0];
	dUnsigned64* const user = &userPool[0];

	BeginFace ();
	Iterator iter(polyhedra);
	for (iter.Begin(); iter; iter ++) {
		dEdge* const edge = &(*iter);
		if (edge->m_incidentFace < 0) {
			continue;
		}

		if (!FindEdge(edge->m_incidentVertex, edge->m_twin->m_incidentVertex))	{
			dInt32 indexCount = 0;
			dEdge* ptr = edge;
			do {
				user[indexCount] = ptr->m_userData;
				index[indexCount] = ptr->m_incidentVertex;
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != edge);

			dEdge* const face = AddFace (indexCount, index, (dInt64*) user);
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

dPolyhedra::~dPolyhedra ()
{
}

void dPolyhedra::SavePLY(const char* const fileName, const dFloat64* const vertexArray, dInt32 strideInBytes) const
{
	FILE* const file = fopen(fileName, "wb");

	fprintf(file, "ply\n");
	fprintf(file, "format ascii 1.0\n");

	dPolyhedra copy(*this);

	dInt32 faceCount = 0;
	Iterator iter(copy);
	dInt32 mark = copy.IncLRU();
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const face = &iter.GetNode()->GetInfo();
		if ((face->m_mark < mark) && (face->m_incidentFace > 0)) 
		{
			faceCount++;
			dEdge* edge = face;
			do 
			{
				edge->m_mark = mark;
				edge = edge->m_next;
			} while (edge != face);
		}
	}

	mark = copy.IncLRU();
	dInt32 vertexCount = 0;
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const vertex = &iter.GetNode()->GetInfo();
		if (vertex->m_mark < mark) 
		{
			dEdge* edge = vertex;
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
	const dInt8* const points = (dInt8*)vertexArray;
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const vertex = &iter.GetNode()->GetInfo();
		if (vertex->m_mark < mark) 
		{
			dEdge* edge = vertex;
			do 
			{
				edge->m_mark = mark;
				edge = edge->m_twin->m_next;
			} while (edge != vertex);
			dInt32 index = edge->m_incidentVertex * strideInBytes;

			const dFloat64* const p = (dFloat64*)&points[index];
			dBigVector point(p[0], p[1], p[2], dFloat64(0.0f));
			fprintf(file, "%f %f %f\n", point.m_x, point.m_y, point.m_z);
		}
	}

	mark = copy.IncLRU();
	for (iter.Begin(); iter; iter++) 
	{
		dInt32 indices[1024];
		dInt32 count = 0;
		dEdge* const face = &iter.GetNode()->GetInfo();
		if ((face->m_mark < mark) && (face->m_incidentFace > 0)) 
		{
			dEdge* edge = face;
			do 
			{
				indices[count] = dInt32 (edge->m_userData);
				count++;
				edge->m_mark = mark;
				edge = edge->m_next;
			} while (edge != face);

			fprintf(file, "%d", count);
			for (dInt32 j = 0; j < count; j++) 
			{
				fprintf(file, " %d", indices[j]);
			}
			fprintf(file, "\n");
		}
	}
	fclose(file);
}

dInt32 dPolyhedra::GetFaceCount() const
{
	Iterator iter (*this);
	dInt32 count = 0;
	dInt32 mark = IncLRU();
	for (iter.Begin(); iter; iter ++) {
		dEdge* const edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}

		if (edge->m_incidentFace < 0) {
			continue;
		}

		count ++;
		dEdge* ptr = edge;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);
	}
	return count;
}

dEdge* dPolyhedra::AddFace (dInt32 count, const dInt32* const index, const dInt64* const userdata)
{
	class IntersectionFilter
	{
		public:
		IntersectionFilter ()
		{
			m_count = 0;
		}

		bool Insert (dInt64 value)
		{
			dInt32 i = 0;				
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

		dInt32 m_count;
		dInt64 m_array[2048];
	};

	IntersectionFilter selfIntersectingFaceFilter;

	dInt32 i0 = index[count-1];
	for (dInt32 i = 0; i < count; i ++) 
	{
		dInt32 i1 = index[i];
		dgPairKey code0 (i0, i1);
		if (!selfIntersectingFaceFilter.Insert (code0.GetVal())) 
		{
			return nullptr;
		}

		dgPairKey code1 (i1, i0);
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
	dInt32 i1 = index[0];
	dUnsigned64 udata0 = 0;
	dUnsigned64 udata1 = 0;
	if (userdata) 
	{
		udata0 = dUnsigned64 (userdata[count-1]);
		udata1 = dUnsigned64 (userdata[0]);
	} 

	bool state;
	dgPairKey code (i0, i1);
	dEdge tmpEdge (i0, m_faceSecuence, udata0);
	dNode* const node = Insert (tmpEdge, code.GetVal(), state); 
	dAssert (!state);
	dEdge* edge0 = &node->GetInfo();
	dEdge* const first = edge0;

	for (dInt32 i = 1; i < count; i ++) 
	{
		i0 = i1;
		i1 = index[i];
		udata0 = udata1;
		udata1 = dUnsigned64 (userdata ? userdata[i] : 0);

		dgPairKey code1 (i0, i1);
		dEdge tmpEdge1 (i0, m_faceSecuence, udata0);
		dNode* const node1 = Insert (tmpEdge1, code1.GetVal(), state); 
		dAssert (!state);

		dEdge* const edge1 = &node1->GetInfo();
		edge0->m_next = edge1;
		edge1->m_prev = edge0;
		edge0 = edge1;
	}

	first->m_prev = edge0;
	edge0->m_next = first;

	return first->m_next;
}

bool dPolyhedra::EndFace ()
{
	dPolyhedra::Iterator iter (*this);

	// Connect all twin edge
	for (iter.Begin(); iter; iter ++) 
	{
		dEdge* const edge = &(*iter);
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
	dStack<dEdge*> edgeArrayPool(GetCount() * 2 + 256);

	dInt32 edgeCount = 0;
	dEdge** const edgeArray = &edgeArrayPool[0];
	for (iter.Begin(); iter; iter ++) 
	{
		dEdge* const edge = &(*iter);
		if (!edge->m_twin) 
		{
			bool state;
			dPolyhedra::dgPairKey code (edge->m_next->m_incidentVertex, edge->m_incidentVertex);
			dEdge tmpEdge (edge->m_next->m_incidentVertex, -1);
			tmpEdge.m_incidentFace = -1; 
			dPolyhedra::dNode* const node = Insert (tmpEdge, code.GetVal(), state); 
			dAssert (!state);
			edge->m_twin = &node->GetInfo();
			edge->m_twin->m_twin = edge; 
			edgeArray[edgeCount] = edge->m_twin;
			edgeCount ++;
		}
	}

	for (dInt32 i = 0; i < edgeCount; i ++) 
	{
		dEdge* const edge = edgeArray[i];
		dAssert (!edge->m_prev);
		dEdge *ptr = edge->m_twin;
		for (; ptr->m_next; ptr = ptr->m_next->m_twin){}
		ptr->m_next = edge;
		edge->m_prev = ptr;
	}

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
	dAssert (SanityCheck ());
#endif

	return true;
}

void dPolyhedra::DeleteFace(dEdge* const face)
{
	dEdge* edgeList[D_LOCAL_BUFFER_SIZE * 16];

	if (face->m_incidentFace > 0) 
	{
		dInt32 count = 0;
		dEdge* ptr = face;
		do 
		{
			ptr->m_incidentFace = -1;
			dInt32 i = 0;
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

		for (dInt32 i = 0; i < count; i ++) 
		{
			dEdge* const ptr1 = edgeList[i];
			if (ptr1->m_twin->m_incidentFace < 0) 
			{
				DeleteEdge (ptr1);
			}
		}
	}
}

dBigVector dPolyhedra::FaceNormal (const dEdge* const face, const dFloat64* const pool, dInt32 strideInBytes) const
{
	dInt32 stride = dInt32 (strideInBytes / sizeof (dFloat64));
	const dEdge* edge = face;
	dBigVector p0 (dBigVector::m_triplexMask & dBigVector(&pool[edge->m_incidentVertex * stride]));
	edge = edge->m_next;
	dBigVector p1 (dBigVector::m_triplexMask & dBigVector(&pool[edge->m_incidentVertex * stride]));
	dBigVector e1 (p1 - p0);

	dBigVector normal (dBigVector::m_zero);
	for (edge = edge->m_next; edge != face; edge = edge->m_next) 
	{
		dBigVector p2 (dBigVector::m_triplexMask & dBigVector(&pool[edge->m_incidentVertex * stride]));
		dBigVector e2 (p2 - p0);
		normal += e1.CrossProduct(e2);
		e1 = e2;
	} 
	dAssert (normal.m_w == dFloat32 (0.0f));
	return normal;
}

dEdge* dPolyhedra::AddHalfEdge (dInt32 v0, dInt32 v1)
{
	if (v0 != v1) 
	{
		dgPairKey pairKey (v0, v1);
		dEdge tmpEdge (v0, -1);

		dNode* node = Insert (tmpEdge, pairKey.GetVal()); 
		return node ? &node->GetInfo() : nullptr;
	}
	else 
	{
		return nullptr;
	}
}

void dPolyhedra::DeleteEdge (dEdge* const edge)
{
	dEdge *const twin = edge->m_twin;

	edge->m_prev->m_next = twin->m_next;
	twin->m_next->m_prev = edge->m_prev;
	edge->m_next->m_prev = twin->m_prev;
	twin->m_prev->m_next = edge->m_next;

	dNode *const nodeA = GetNodeFromInfo (*edge);
	dNode *const nodeB = GetNodeFromInfo (*twin);

	dAssert (&nodeA->GetInfo() == edge);
	dAssert (&nodeB->GetInfo() == twin);

	Remove (nodeA);
	Remove (nodeB);
}

dEdge* dPolyhedra::ConnectVertex (dEdge* const e0, dEdge* const e1)
{
	dEdge* const edge = AddHalfEdge(e1->m_incidentVertex, e0->m_incidentVertex);
	dEdge* const twin = AddHalfEdge(e0->m_incidentVertex, e1->m_incidentVertex);
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

dEdge* dPolyhedra::SpliteEdge (dInt32 newIndex,	dEdge* const edge)
{
	dEdge* const edge00 = edge->m_prev;
	dEdge* const edge01 = edge->m_next;
	dEdge* const twin00 = edge->m_twin->m_next;
	dEdge* const twin01 = edge->m_twin->m_prev;

	dInt32 i0 = edge->m_incidentVertex;
	dInt32 i1 = edge->m_twin->m_incidentVertex;

	dInt32 f0 = edge->m_incidentFace;
	dInt32 f1 = edge->m_twin->m_incidentFace;

	DeleteEdge (edge);

	dEdge* const edge0 = AddHalfEdge (i0, newIndex);
	dEdge* const edge1 = AddHalfEdge (newIndex, i1);

	dEdge* const twin0 = AddHalfEdge (newIndex, i0);
	dEdge* const twin1 = AddHalfEdge (i1, newIndex);
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

bool dPolyhedra::FlipEdge (dEdge* const edge)
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

	dEdge *const prevEdge = edge->m_prev;
	dEdge *const prevTwin = edge->m_twin->m_prev;

	dgPairKey edgeKey (prevTwin->m_incidentVertex, prevEdge->m_incidentVertex);
	dgPairKey twinKey (prevEdge->m_incidentVertex, prevTwin->m_incidentVertex);

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

bool dPolyhedra::GetConectedSurface (dPolyhedra &polyhedra) const
{
	if (!GetCount()) {
		return false;
	}

	dEdge* edge = nullptr;
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

	dInt32 faceIndex[4096];
	dInt64 faceDataIndex[4096];
	dStack<dEdge*> stackPool (GetCount()); 
	dEdge** const stack = &stackPool[0];

	dInt32 mark = IncLRU();

	polyhedra.BeginFace ();
	stack[0] = edge;
	dInt32 index = 1;
	while (index) {
		index --;
		dEdge* const edge1 = stack[index];
		dAssert (edge1);
		if (edge1->m_mark == mark) {
			continue;
		}

		dInt32 count = 0;
		dEdge* ptr = edge1;
		do {
			dAssert (ptr);
			ptr->m_mark = mark;
			faceIndex[count] = ptr->m_incidentVertex;
			faceDataIndex[count] = dInt64 (ptr->m_userData);
			count ++;
			dAssert (count <  dInt32 ((sizeof (faceIndex)/sizeof(faceIndex[0]))));

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

void dPolyhedra::ChangeEdgeIncidentVertex (dEdge* const edge, dInt32 newIndex)
{
	dEdge* ptr = edge;
	do {
		dNode* node = GetNodeFromInfo(*ptr);
		dgPairKey Key0 (newIndex, ptr->m_twin->m_incidentVertex);
		ReplaceKey (node, Key0.GetVal());

		node = GetNodeFromInfo(*ptr->m_twin);
		dgPairKey Key1 (ptr->m_twin->m_incidentVertex, newIndex);
		ReplaceKey (node, Key1.GetVal());

		ptr->m_incidentVertex = newIndex;

		ptr = ptr->m_twin->m_next;
	} while (ptr != edge);
}

void dPolyhedra::DeleteDegenerateFaces (const dFloat64* const pool, dInt32 strideInBytes, dFloat64 area)
{
	if (!GetCount()) 
	{
		return;
	}

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
	dAssert (SanityCheck ());
#endif
	dStack <dPolyhedra::dNode*> faceArrayPool(GetCount() / 2 + 100);

	dInt32 count = 0;
	dPolyhedra::dNode** const faceArray = &faceArrayPool[0];
	dInt32 mark = IncLRU();
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) 
	{
		dEdge* const edge = &(*iter);

		if ((edge->m_mark != mark) && (edge->m_incidentFace > 0)) 
		{
			faceArray[count] = iter.GetNode();
			count ++;
			dEdge* ptr = edge;
			do	{
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}

	dFloat64 area2 = area * area;
	area2 *= dFloat64 (4.0f);

	for (dInt32 i = 0; i < count; i ++) 
	{
		dPolyhedra::dNode* const faceNode = faceArray[i];
		dEdge* const edge = &faceNode->GetInfo();

		dBigVector normal (FaceNormal (edge, pool, strideInBytes));

		dFloat64 faceArea = normal.DotProduct(normal).GetScalar();
		if (faceArea < area2) 
		{
			DeleteFace (edge);
		}
	}

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
	mark = IncLRU();
	for (iter.Begin(); iter; iter ++) 
	{
		dEdge* const edge = &(*iter);
		if ((edge->m_mark != mark) && (edge->m_incidentFace > 0)) 
		{
			//dAssert (edge->m_next->m_next->m_next == edge);
			dEdge* ptr = edge;
			do	
			{
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);

			dBigVector normal (FaceNormal (edge, pool, strideInBytes));

			dFloat64 faceArea = normal.DotProduct(normal).GetScalar();
			dAssert (faceArea >= area2);
		}
	}
	dAssert (SanityCheck ());
#endif
}

dBigPlane dPolyhedra::UnboundedLoopPlane (dInt32 i0, dInt32 i1, dInt32 i2, const dBigVector* const pool)
{
	const dBigVector p0 = pool[i0];
	const dBigVector p1 = pool[i1];
	const dBigVector p2 = pool[i2];
	dBigVector E0 (p1 - p0); 
	dBigVector E1 (p2 - p0); 

	dBigVector N ((E0.CrossProduct(E1)).CrossProduct(E0) & dBigVector::m_triplexMask); 
	dFloat64 dist = - N.DotProduct(p0).GetScalar();
	dBigPlane plane (N, dist);

	dFloat64 mag = sqrt (plane.DotProduct(plane & dBigVector::m_triplexMask).GetScalar());
	if (mag < dFloat64 (1.0e-12f)) 
	{
		mag = dFloat64 (1.0e-12f);
	}
	mag = dFloat64 (10.0f) / mag;

	plane.m_x *= mag;
	plane.m_y *= mag;
	plane.m_z *= mag;
	plane.m_w *= mag;

	return plane;
}

dEdge* dPolyhedra::CollapseEdge(dEdge* const edge)
{
	dInt32 v0 = edge->m_incidentVertex;
	dInt32 v1 = edge->m_twin->m_incidentVertex;

	dEdge* retEdge = edge->m_twin->m_prev->m_twin;
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

	dEdge* lastEdge = nullptr;
	dEdge* firstEdge = nullptr;
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

	for (dEdge* ptr = firstEdge; ptr != lastEdge; ptr = ptr->m_twin->m_next) {
		dEdge* const badEdge = FindEdge (edge->m_twin->m_incidentVertex, ptr->m_twin->m_incidentVertex);
		if (badEdge) {
			return nullptr;
		}
	} 

	dEdge* const twin = edge->m_twin;
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

	dEdge* ptr = retEdge;
	do {
		dPolyhedra::dgPairKey pairKey (v0, ptr->m_twin->m_incidentVertex);

		dPolyhedra::dNode* node = Find (pairKey.GetVal());
		if (node) {
			if (&node->GetInfo() == ptr) {
				dPolyhedra::dgPairKey key (v1, ptr->m_twin->m_incidentVertex);
				ptr->m_incidentVertex = v1;
				node = ReplaceKey (node, key.GetVal());
				dAssert (node);
			} 
		}

		dPolyhedra::dgPairKey TwinKey (ptr->m_twin->m_incidentVertex, v0);
		node = Find (TwinKey.GetVal());
		if (node) {
			if (&node->GetInfo() == ptr->m_twin) {
				dPolyhedra::dgPairKey key (ptr->m_twin->m_incidentVertex, v1);
				node = ReplaceKey (node, key.GetVal());
				dAssert (node);
			}
		}

		ptr = ptr->m_twin->m_next;
	} while (ptr != retEdge);

	return retEdge;
}

void dPolyhedra::RemoveHalfEdge (dEdge* const edge)
{
	dEdgeCollapseEdgeHandle* const handle = (dEdgeCollapseEdgeHandle *) dIntToPointer (edge->m_userData);
	if (handle) { 
		handle->m_edge = nullptr;
	}

	dPolyhedra::dNode* const node = GetNodeFromInfo(*edge);
	dAssert (node);
	Remove (node);
}

dEdge* dPolyhedra::FindEarTip (dEdge* const face, const dFloat64* const pool, dInt32 stride, dDownHeap<dEdge*, dFloat64>& heap, const dBigVector &normal) const
{
	dEdge* ptr = face;
	dBigVector p0 (dBigVector::m_triplexMask & dBigVector(&pool[ptr->m_prev->m_incidentVertex * stride]));
	dBigVector p1 (dBigVector::m_triplexMask & dBigVector(&pool[ptr->m_incidentVertex * stride]));
	dBigVector d0 (p1 - p0);
	dFloat64 val = sqrt (d0.DotProduct(d0 & dBigVector::m_triplexMask).GetScalar());
	if (val < dFloat64 (1.0e-10f)) {
		val = dFloat64 (1.0e-10f);
	}
	d0 = d0.Scale (dFloat64 (1.0f) / val);

	dFloat64 minAngle = dFloat32 (10.0f);
	do {
		dBigVector p2 (dBigVector::m_triplexMask & dBigVector(&pool [ptr->m_next->m_incidentVertex * stride]));
		dBigVector d1 (p2 - p1);
		dFloat64 val1 = dFloat64 (1.0f) / sqrt (d1.DotProduct(d1).GetScalar());
		if (val1 < dFloat64 (1.0e-10f)) {
			val1 = dFloat64 (1.0e-10f);
		}
		d1 = d1.Scale (dFloat32 (1.0f) / val1);
		dBigVector n (d0.CrossProduct(d1));

		dFloat64 angle = normal.DotProduct(n & dBigVector::m_triplexMask).GetScalar();
		if (angle >= dFloat64 (0.0f)) {
			heap.Push (ptr, angle);
		}

		if (angle < minAngle) {
			minAngle = angle;
		}

		d0 = d1;
		p1 = p2;
		ptr = ptr->m_next;
	} while (ptr != face);

	if (minAngle > dFloat32 (0.1f)) {
		return heap[0];
	}

	dEdge* ear = nullptr;
	while (heap.GetCount()) {
		ear = heap[0];
		heap.Pop();

		if (FindEdge (ear->m_prev->m_incidentVertex, ear->m_next->m_incidentVertex)) {
			continue;
		}

		dBigVector q0 (dBigVector::m_triplexMask & dBigVector(&pool [ear->m_prev->m_incidentVertex * stride]));
		dBigVector q1 (dBigVector::m_triplexMask & dBigVector(&pool [ear->m_incidentVertex * stride]));
		dBigVector q2 (dBigVector::m_triplexMask & dBigVector(&pool [ear->m_next->m_incidentVertex * stride]));

		dBigVector p10 (q1 - q0);
		dBigVector p21 (q2 - q1);
		dBigVector p02 (q0 - q2);
		dAssert(normal.m_w == dFloat32(0.0f));

		for (ptr = ear->m_next->m_next; ptr != ear->m_prev; ptr = ptr->m_next) {
			if (!((ptr->m_incidentVertex == ear->m_incidentVertex) || (ptr->m_incidentVertex == ear->m_prev->m_incidentVertex) || (ptr->m_incidentVertex == ear->m_next->m_incidentVertex))) { 
				dBigVector p (dBigVector::m_triplexMask & dBigVector(&pool [ptr->m_incidentVertex * stride]));

				//dFloat64 side = ((p - p0) * p10) % normal;
				dFloat64 side = normal.DotProduct((p - q0).CrossProduct(p10)).GetScalar();
				if (side < dFloat64 (0.05f)) {
					//side = ((p - p1) * p21) % normal;
					side = normal.DotProduct((p - q1).CrossProduct(p21)).GetScalar();
					if (side < dFloat64 (0.05f)) {
						//side = ((p - p2) * p02) % normal;
						side = normal.DotProduct((p - q2).CrossProduct(p02)).GetScalar();
						if (side < dFloat32 (0.05f)) {
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

dEdge* dPolyhedra::TriangulateFace (dEdge* const faceIn, const dFloat64* const pool, dInt32 stride, dDownHeap<dEdge*, dFloat64>& heap, dBigVector* const faceNormalOut)
{
	dEdge* face = faceIn;
	dBigVector normal (FaceNormal (face, pool, dInt32 (stride * sizeof (dFloat64))));
	dAssert(normal.m_w == dFloat32(0.0f));
	dFloat64 dot = normal.DotProduct(normal).GetScalar();
	if (dot < dFloat64 (1.0e-12f)) {
		if (faceNormalOut) {
			*faceNormalOut = dBigVector (dFloat32 (0.0f)); 
		}
		return face;
	}
	normal = normal.Scale (dFloat64 (1.0f) / sqrt (dot));
	if (faceNormalOut) {
		*faceNormalOut = normal;
	}

	while (face->m_next->m_next->m_next != face) {
		dEdge* const ear = FindEarTip (face, pool, stride, heap, normal); 
		if (!ear) {
			return face;
		}
		if ((face == ear)	|| (face == ear->m_prev)) {
			face = ear->m_prev->m_prev;
		}
		dEdge* const edge = AddHalfEdge (ear->m_next->m_incidentVertex, ear->m_prev->m_incidentVertex);
		if (!edge) {
			return face;
		}
		dEdge* const twin = AddHalfEdge (ear->m_prev->m_incidentVertex, ear->m_next->m_incidentVertex);
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

void dPolyhedra::MarkAdjacentCoplanarFaces (dPolyhedra& polyhedraOut, dEdge* const face, const dFloat64* const pool, dInt32 strideInBytes)
{
	const dFloat64 normalDeviation = dFloat64 (0.9999f);
	const dFloat64 distanceFromPlane = dFloat64 (1.0f / 128.0f);

	dInt32 faceIndex[D_LOCAL_BUFFER_SIZE * 8];
	dInt64 userIndex[D_LOCAL_BUFFER_SIZE * 8];
	dEdge* stack[D_LOCAL_BUFFER_SIZE * 8];
	dEdge* deleteEdge[D_LOCAL_BUFFER_SIZE * 32];

	dInt32 deleteCount = 1;
	deleteEdge[0] = face;
	dInt32 stride = dInt32 (strideInBytes / sizeof (dFloat64));

	dAssert (face->m_incidentFace > 0);

	dBigVector normalAverage (FaceNormal (face, pool, strideInBytes));
	dAssert (normalAverage.m_w == dFloat32 (0.0f));
	dFloat64 dot = normalAverage.DotProduct(normalAverage).GetScalar();
	if (dot > dFloat64 (1.0e-12f)) {
		dInt32 testPointsCount = 1;
		dot = dFloat64 (1.0f) / sqrt (dot);
		dBigVector normal (normalAverage.Scale (dot));

		dBigVector averageTestPoint (dBigVector::m_triplexMask & dBigVector(&pool[face->m_incidentVertex * stride]));
		dBigPlane testPlane(normal, - normal.DotProduct(averageTestPoint & dBigVector::m_triplexMask).GetScalar());

		polyhedraOut.BeginFace();

		IncLRU();
		dInt32 faceMark = IncLRU();

		dInt32 faceIndexCount = 0;
		dEdge* ptr = face;
		do {
			ptr->m_mark = faceMark;
			faceIndex[faceIndexCount] = ptr->m_incidentVertex;
			userIndex[faceIndexCount] = dInt64 (ptr->m_userData);
			faceIndexCount ++;
			dAssert (faceIndexCount < dInt32 (sizeof (faceIndex) / sizeof (faceIndex[0])));
			ptr = ptr ->m_next;
		} while (ptr != face);
		polyhedraOut.AddFace(faceIndexCount, faceIndex, userIndex);

		dInt32 index = 1;
		deleteCount = 0;
		stack[0] = face;
		while (index) {
			index --;
			dEdge* const face1 = stack[index];
			deleteEdge[deleteCount] = face1;
			deleteCount ++;
			dAssert (deleteCount < dInt32 (sizeof (deleteEdge) / sizeof (deleteEdge[0])));
			dAssert (face1->m_next->m_next->m_next == face1);

			dEdge* edge = face1;
			do {
				dEdge* const ptr1 = edge->m_twin;
				if (ptr1->m_incidentFace > 0) {
					if (ptr1->m_mark != faceMark) {
						dEdge* ptr2 = ptr1;
						faceIndexCount = 0;
						do {
							ptr2->m_mark = faceMark;
							faceIndex[faceIndexCount] = ptr2->m_incidentVertex;
							userIndex[faceIndexCount] = dInt64 (ptr2->m_userData);
							dAssert (faceIndexCount < dInt32 (sizeof (faceIndex) / sizeof (faceIndex[0])));
							faceIndexCount ++;
							ptr2 = ptr2 ->m_next;
						} while (ptr2 != ptr1);

						dBigVector normal1 (FaceNormal (ptr1, pool, strideInBytes));
						dot = normal1.DotProduct(normal1).GetScalar();
						if (dot < dFloat64 (1.0e-12f)) {
							deleteEdge[deleteCount] = ptr1;
							deleteCount ++;
							dAssert (deleteCount < dInt32 (sizeof (deleteEdge) / sizeof (deleteEdge[0])));
						} else {
							dBigVector testNormal (normal1.Scale (dFloat64 (1.0f) / sqrt (dot)));
							dAssert (testNormal.m_w == dFloat32 (0.0f));
							dot = normal.DotProduct(testNormal).GetScalar();
							if (dot >= normalDeviation) {
								dBigVector testPoint (dBigVector::m_triplexMask & dBigVector(&pool[ptr1->m_prev->m_incidentVertex * stride]));
								dFloat64 dist = fabs (testPlane.Evalue (testPoint));
								if (dist < distanceFromPlane) {
									testPointsCount ++;

									averageTestPoint += testPoint;
									testPoint = averageTestPoint.Scale (dFloat64 (1.0f) / dFloat64(testPointsCount));

									normalAverage += normal1;
									dAssert (normalAverage.m_w == dFloat32 (0.0f));
									testNormal = normalAverage.Scale (dFloat64 (1.0f) / sqrt (normalAverage.DotProduct(normalAverage).GetScalar()));
									testPlane = dBigPlane (testNormal, - testPoint.DotProduct (testNormal).GetScalar());

									polyhedraOut.AddFace(faceIndexCount, faceIndex, userIndex);
									stack[index] = ptr1;
									index ++;
									dAssert (index < dInt32 (sizeof (stack) / sizeof (stack[0])));
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

	for (dInt32 index = 0; index < deleteCount; index ++) {
		DeleteFace (deleteEdge[index]);
	}
}

void dPolyhedra::RefineTriangulation (const dFloat64* const vertex, dInt32 stride, const dBigVector& normal, dInt32 perimeterCount, dEdge** const perimeter)
{
	dList<dgDiagonalEdge> dignonals;

	for (dInt32 i = 1; i <= perimeterCount; i ++) {
		dEdge* const last = perimeter[i - 1];
		for (dEdge* ptr = perimeter[i]->m_prev; ptr != last; ptr = ptr->m_twin->m_prev) {
			dList<dgDiagonalEdge>::dNode* node = dignonals.GetFirst();
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

	dEdge* const face = perimeter[0];
	dInt32 i0 = face->m_incidentVertex * stride;
	dInt32 i1 = face->m_next->m_incidentVertex * stride;
	dBigVector p0 (vertex[i0], vertex[i0 + 1], vertex[i0 + 2], dFloat32 (0.0f));
	dBigVector p1 (vertex[i1], vertex[i1 + 1], vertex[i1 + 2], dFloat32 (0.0f));

	dBigVector p1p0 (p1 - p0);
	dFloat64 mag2 = p1p0.DotProduct(p1p0).GetScalar();
	for (dEdge* ptr = face->m_next->m_next; mag2 < dFloat32 (1.0e-12f); ptr = ptr->m_next) {
		dInt32 i2 = ptr->m_incidentVertex * stride;
		dBigVector p2 (vertex[i2], vertex[i2 + 1], vertex[i2 + 2], dFloat32 (0.0f));
		p1p0 = p2 - p0;
		mag2 = p1p0.DotProduct(p1p0).GetScalar();
	}

	dAssert (p1p0.m_w == dFloat32 (0.0f));
	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = p0;
	matrix.m_front = dVector (p1p0.Scale (dFloat64 (1.0f) / sqrt (mag2)));
	matrix.m_right = dVector (normal.Scale (dFloat64 (1.0f) / sqrt (normal.DotProduct(normal).GetScalar())));
	matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);
	matrix = matrix.Inverse();
	dAssert (matrix.m_posit.m_w == dFloat32 (1.0f));
//	matrix.m_posit.m_w = dFloat32 (1.0f);

	dInt32 maxCount = dignonals.GetCount() * dignonals.GetCount();
	while (dignonals.GetCount() && maxCount) {
		maxCount --;
		dList<dgDiagonalEdge>::dNode* const node = dignonals.GetFirst();
		dgDiagonalEdge key (node->GetInfo());
		dignonals.Remove(node);
		dEdge* const edge = FindEdge(key.m_i0, key.m_i1);
		if (edge) {
			dInt32 k0 = edge->m_incidentVertex * stride;
			dInt32 k1 = edge->m_next->m_incidentVertex * stride;
			dInt32 k2 = edge->m_next->m_next->m_incidentVertex * stride;
			dInt32 k3 = edge->m_twin->m_prev->m_incidentVertex * stride;

			dBigVector q0 (vertex[k0], vertex[k0 + 1], vertex[k0 + 2], dFloat64 (1.0f));
			dBigVector q1 (vertex[k1], vertex[k1 + 1], vertex[k1 + 2], dFloat64 (1.0f));
			dBigVector q2 (vertex[k2], vertex[k2 + 1], vertex[k2 + 2], dFloat64 (1.0f));
			dBigVector q3 (vertex[k3], vertex[k3 + 1], vertex[k3 + 2], dFloat64 (1.0f));

			q0 = matrix.TransformVector(q0);
			q1 = matrix.TransformVector(q1);
			q2 = matrix.TransformVector(q2);
			q3 = matrix.TransformVector(q3);

			dFloat64 circleTest[3][3];
			circleTest[0][0] = q0[0] - q3[0];
			circleTest[0][1] = q0[1] - q3[1];
			circleTest[0][2] = circleTest[0][0] * circleTest[0][0] + circleTest[0][1] * circleTest[0][1];

			circleTest[1][0] = q1[0] - q3[0];
			circleTest[1][1] = q1[1] - q3[1];
			circleTest[1][2] = circleTest[1][0] * circleTest[1][0] + circleTest[1][1] * circleTest[1][1];

			circleTest[2][0] = q2[0] - q3[0];
			circleTest[2][1] = q2[1] - q3[1];
			circleTest[2][2] = circleTest[2][0] * circleTest[2][0] + circleTest[2][1] * circleTest[2][1];

			dFloat64 error;
			dFloat64 det = Determinant3x3 (circleTest, &error);
			if (det < dFloat32 (0.0f)) {
				dEdge* frontFace0 = edge->m_prev;
				dEdge* backFace0 = edge->m_twin->m_prev;

				FlipEdge(edge);

				if (perimeterCount > 4) {
					dEdge* backFace1 = backFace0->m_next;
					dEdge* frontFace1 = frontFace0->m_next;
					for (dInt32 i = 0; i < perimeterCount; i ++) {
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

void dPolyhedra::RefineTriangulation (const dFloat64* const vertex, dInt32 stride)
{
	if (GetCount() <= 6) {
		return;
	}

	dInt32 mark = IncLRU();
	dInt32 loopCount = 0;
	
	dPolyhedra::Iterator iter (*this);
	dEdge* edgePerimeters[D_LOCAL_BUFFER_SIZE * 16];
	dInt32 perimeterCount = 0;
	dTree<dEdge*, dInt32> filter;
	for (iter.Begin(); iter && (loopCount <= 1) ; iter ++) {
		dEdge* const edge = &(*iter);
		if ((edge->m_incidentFace < 0) && (edge->m_mark != mark)){
			loopCount ++;
			dEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				if (!filter.Insert(ptr, ptr->m_incidentVertex)) {
					loopCount = 2;
					break;
				}
				edgePerimeters[perimeterCount] = ptr->m_twin;
				perimeterCount ++;
				dAssert (perimeterCount < dInt32 (sizeof (edgePerimeters) / sizeof (edgePerimeters[0])));
				ptr = ptr->m_prev;
			} while (ptr != edge);
		}
	}

	if (loopCount == 1) {
		#ifdef _DEBUG
		for (dInt32 i = 0; i < perimeterCount; i ++) {
			for (dInt32 j = i + 1; j < perimeterCount; j ++) {
				dAssert (edgePerimeters[i]->m_incidentVertex != edgePerimeters[j]->m_incidentVertex);
			}
		}
		#endif

		dAssert (perimeterCount);
		dAssert (perimeterCount < dInt32 (sizeof (edgePerimeters) / sizeof (edgePerimeters[0])));
		edgePerimeters[perimeterCount] = edgePerimeters[0];

		dBigVector normal (FaceNormal(edgePerimeters[0], vertex, dInt32 (stride * sizeof (dFloat64))));
		if (normal.DotProduct(normal).GetScalar() > dFloat32 (1.0e-12f)) {
			RefineTriangulation (vertex, stride, normal, perimeterCount, edgePerimeters);
		}
	}
}

void dPolyhedra::OptimizeTriangulation (const dFloat64* const vertex, dInt32 strideInBytes)
{
	dInt32 polygon[D_LOCAL_BUFFER_SIZE * 8];
	dInt64 userData[D_LOCAL_BUFFER_SIZE * 8];
	dInt32 stride = dInt32 (strideInBytes / sizeof (dFloat64));

	dPolyhedra leftOver;
	dPolyhedra buildConvex;

	buildConvex.BeginFace();
	dPolyhedra::Iterator iter (*this);

	for (iter.Begin(); iter; ) {
		dEdge* const edge = &(*iter);
		iter++;

		if (edge->m_incidentFace > 0) {
			dPolyhedra flatFace;
			MarkAdjacentCoplanarFaces (flatFace, edge, vertex, strideInBytes);
			//dAssert (flatFace.GetCount());

			if (flatFace.GetCount()) {
				flatFace.RefineTriangulation (vertex, stride);

				dInt32 mark = flatFace.IncLRU();
				dPolyhedra::Iterator iter1 (flatFace);
				for (iter1.Begin(); iter1; iter1 ++) {
					dEdge* const edge1 = &(*iter1);
					if (edge1->m_mark != mark) {
						if (edge1->m_incidentFace > 0) {
							dEdge* ptr = edge1;
							dInt32 vertexCount = 0;
							do {
								polygon[vertexCount] = ptr->m_incidentVertex;				
								userData[vertexCount] = dInt64 (ptr->m_userData);
								vertexCount ++;
								dAssert (vertexCount < dInt32 (sizeof (polygon) / sizeof (polygon[0])));
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

void dPolyhedra::Triangulate (const dFloat64* const vertex, dInt32 strideInBytes, dPolyhedra* const leftOver)
{
	dInt32 stride = dInt32 (strideInBytes / sizeof (dFloat64));

	dInt32 count = GetCount() / 2;
	dStack<char> memPool (dInt32 ((count + 512) * (2 * sizeof (dFloat64)))); 
	dDownHeap<dEdge*, dFloat64> heap(&memPool[0], memPool.GetSizeInBytes());

	dInt32 mark = IncLRU();
	Iterator iter (*this);
	for (iter.Begin(); iter; ) {
		dEdge* const thisEdge = &(*iter);
		iter ++;

		if (thisEdge->m_mark == mark) {
			continue;
		}
		if (thisEdge->m_incidentFace < 0) {
			continue;
		}

		count = 0;
		dEdge* ptr = thisEdge;
		do {
			count ++;
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != thisEdge);

		if (count > 3) {
			dEdge* const edge = TriangulateFace (thisEdge, vertex, stride, heap, nullptr);
			heap.Flush ();

			if (edge) {
				dAssert (edge->m_incidentFace > 0);

				if (leftOver) {
					dInt32* const index = (dInt32 *) &heap[0];
					dInt64* const data = (dInt64 *)&index[count];
					dInt32 i = 0;
					dEdge* ptr1 = edge;
					do {
						index[i] = ptr1->m_incidentVertex;
						data[i] = dInt64 (ptr1->m_userData);
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
		dEdge* edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}
		if (edge->m_incidentFace < 0) {
			continue;
		}
		dAssert (edge == edge->m_next->m_next->m_next);

		for (dInt32 i = 0; i < 3; i ++) { 
			edge->m_incidentFace = m_faceSecuence; 
			edge->m_mark = mark;
			edge = edge->m_next;
		}
		m_faceSecuence ++;
	}
}

bool dPolyhedra::IsFaceConvex(dEdge* const face, const dFloat64* const vertex, dInt32 strideInBytes) const
{
	if (face->m_next->m_next->m_next == face) 
	{
		return true;
	}
	dBigVector normal(FaceNormal(face, vertex, strideInBytes));
	dAssert(normal.m_w == dFloat32(0.0f));

	dInt32 stride = strideInBytes / sizeof(dFloat64);
	dEdge* ptr = face;
	do 
	{
		dBigVector p0(dBigVector::m_triplexMask & dBigVector(&vertex[ptr->m_incidentVertex * stride]));
		dBigVector p1(dBigVector::m_triplexMask & dBigVector(&vertex[ptr->m_prev->m_incidentVertex * stride]));
		dBigVector p2(dBigVector::m_triplexMask & dBigVector(&vertex[ptr->m_next->m_incidentVertex * stride]));

		dBigVector e0(p1 - p0);
		dBigVector e1(p2 - p1);
		dBigVector cornerNormal(e1.CrossProduct(e0));
		dFloat64 project(normal.DotProduct(cornerNormal).GetScalar());
		if (project < dFloat32(0.0f)) 
		{
			return false;
		}

		ptr = ptr->m_next;
	} while (ptr != face);
	
	return true;
}

void dPolyhedra::RemoveOuterColinearEdges (dPolyhedra& flatFace, const dFloat64* const vertex, dInt32 stride)
{
	dEdge* edgePerimeters[D_LOCAL_BUFFER_SIZE];

	dInt32 perimeterCount = 0;
	dInt32 mark = flatFace.IncLRU();
	dPolyhedra::Iterator iter (flatFace);
	for (iter.Begin(); iter; iter ++) {
		dEdge* const edge = &(*iter);
		if ((edge->m_incidentFace < 0) && (edge->m_mark != mark)) {
			dEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);

			edgePerimeters[perimeterCount] = edge;
			perimeterCount++;
			dAssert(perimeterCount < dInt32(sizeof(edgePerimeters) / sizeof(edgePerimeters[0])));
		}
	}

	dInt8 buffer[2048 * sizeof (dFloat64)];
	dDownHeap<dEdge*, dFloat64> heap(&buffer[0], sizeof (buffer));
	for (dInt32 i = 0; i < perimeterCount; i ++) {
		dEdge* edge = edgePerimeters[i];
		dEdge* ptr = edge;
		dBigVector p0 (dBigVector::m_triplexMask & dBigVector(&vertex[ptr->m_incidentVertex * stride]) );
		dBigVector p1 (dBigVector::m_triplexMask & dBigVector(&vertex[ptr->m_next->m_incidentVertex * stride]));
		dBigVector e0 ((p1 - p0));
		e0 = e0.Scale (dFloat32(1.0f) / sqrt (e0.DotProduct(e0).GetScalar() + dFloat32 (1.0e-12f)));
		dInt32 ignoreTest = 1;
		do {
			ignoreTest = 0;
			dBigVector p2 (dBigVector::m_triplexMask & dBigVector(&vertex[ptr->m_next->m_next->m_incidentVertex * stride]));
			dBigVector e1 (p2 - p1);
			//e1 = e1.Scale (dRsqrt (e1.DotProduct3(e1) + dFloat32 (1.0e-12f)));
			e1 = e1.Scale(dFloat32(1.0f) / sqrt(e1.DotProduct(e1).GetScalar() + dFloat32(1.0e-12f)));
			dFloat64 dot = e1.DotProduct(e0).GetScalar();
			if (dot > dFloat32 (dFloat32 (0.9999f))) 
			{
				for (dEdge* interiorEdge = ptr->m_next->m_twin->m_next; interiorEdge != ptr->m_twin; interiorEdge = ptr->m_next->m_twin->m_next) 
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

						if (!flatFace.IsFaceConvex(ptr->m_twin, vertex, stride * sizeof(dFloat64))) {
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

void dPolyhedra::RemoveInteriorColinearEdges(dPolyhedra& flatFace, const dFloat64* const vertex, dInt32 stride)
{
	bool foundEdge = true;
	while (foundEdge) {
		foundEdge = false;
		dPolyhedra::Iterator iter(flatFace);
		for (iter.Begin(); iter; iter++) {
			dEdge* const edge = &(*iter);
			if ((edge->m_incidentFace > 0) && (edge->m_twin->m_incidentFace > 0)) {
				if (edge->m_twin->m_next->m_twin->m_next == edge) {
					dBigVector p0(dBigVector::m_triplexMask & dBigVector(&vertex[edge->m_prev->m_incidentVertex * stride]));
					dBigVector p1(dBigVector::m_triplexMask & dBigVector(&vertex[edge->m_incidentVertex * stride]));
					dBigVector p2(dBigVector::m_triplexMask & dBigVector(&vertex[edge->m_next->m_incidentVertex * stride]));
					
					dBigVector e0(p1 - p0);
					dBigVector e1(p2 - p1);
					e0 = e0.Scale(dFloat32 (1.0f) / sqrt(e0.DotProduct(e0).GetScalar() + dFloat32(1.0e-12f)));
					e1 = e1.Scale(dFloat32 (1.0f) / sqrt(e1.DotProduct(e1).GetScalar() + dFloat32(1.0e-12f)));
					dFloat64 dot = e1.DotProduct(e0).GetScalar();
					if (dot > dFloat32(0.9999f)) 
					{
						dInt32 v = edge->m_twin->m_incidentVertex;
						dEdge* const nextEdge = edge->m_twin->m_next;
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

dInt32 dPolyhedra::GetInteriorDiagonals (dPolyhedra& polyhedra, dEdge** const diagonals, dInt32 maxCount)
{
	dInt32 count = 0;
	dInt32 mark = polyhedra.IncLRU();
	dPolyhedra::Iterator iter (polyhedra);
	for (iter.Begin(); iter; iter++) {
		dEdge* const edge = &(*iter);
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

bool dPolyhedra::IsEssensialPointDiagonal (dEdge* const diagonal, const dBigVector& normal, const dFloat64* const pool, dInt32 stride)
{
	if (diagonal->m_twin->m_next->m_twin->m_next != diagonal) {
		dBigVector p0 (dBigVector::m_triplexMask & dBigVector(&pool[diagonal->m_incidentVertex * stride]));
		dBigVector p1 (dBigVector::m_triplexMask & dBigVector(&pool[diagonal->m_twin->m_next->m_twin->m_incidentVertex * stride]));
		dBigVector p2 (dBigVector::m_triplexMask & dBigVector(&pool[diagonal->m_prev->m_incidentVertex * stride]));

		dBigVector e1 (p1 - p0);
		dFloat64 dot = e1.DotProduct(e1).GetScalar();
		if (dot < dFloat64 (1.0e-12f)) {
			return false;
		}
		e1 = e1.Scale (dFloat64 (1.0f) / sqrt(dot));

		dBigVector e2 (p2 - p0);
		dot = e2.DotProduct(e2).GetScalar();
		if (dot < dFloat64 (1.0e-12f)) {
			return false;
		}
		e2 = e2.Scale (dFloat64 (1.0f) / sqrt(dot));

		dBigVector n1 (e1.CrossProduct(e2)); 
		dAssert(normal.m_w == dFloat32(0.0f));
		dot = normal.DotProduct(n1).GetScalar();
		if (dot >= dFloat64 (0.0f)) {
			return false;
		}
	}
	return true;
}

bool dPolyhedra::IsEssensialDiagonal (dEdge* const diagonal, const dBigVector& normal, const dFloat64* const pool,  dInt32 stride)
{
	return IsEssensialPointDiagonal (diagonal, normal, pool, stride) || IsEssensialPointDiagonal (diagonal->m_twin, normal, pool, stride); 
}

dBigPlane dPolyhedra::EdgePlane (dInt32 i0, dInt32 i1, dInt32 i2, const dBigVector* const pool) const
{
	const dBigVector& p0 = pool[i0];
	const dBigVector& p1 = pool[i1];
	const dBigVector& p2 = pool[i2];

	dBigPlane plane (p0, p1, p2);
	dFloat64 mag = sqrt (plane.DotProduct(plane & dBigPlane::m_triplexMask).GetScalar());
	if (mag < dFloat64 (1.0e-12f)) {
		mag = dFloat64 (1.0e-12f);
	}
	mag = dFloat64 (1.0f) / mag;

	plane.m_x *= mag;
	plane.m_y *= mag;
	plane.m_z *= mag;
	plane.m_w *= mag;

	return plane;
}

void dPolyhedra::CalculateVertexMetrics (dVertexCollapseVertexMetric* const table, const dBigVector* const pool, dEdge* const edge) const
{
	dInt32 i0 = edge->m_incidentVertex;

	table[i0].Clear ();
	dEdge* ptr = edge;
	do {

		if (ptr->m_incidentFace > 0) {
			dInt32 i1 = ptr->m_next->m_incidentVertex;
			dInt32 i2 = ptr->m_prev->m_incidentVertex;
			dBigPlane constrainPlane (EdgePlane (i0, i1, i2, pool));
			table[i0].Accumulate (constrainPlane);

		} else {
			dInt32 i1 = ptr->m_twin->m_incidentVertex;
			dInt32 i2 = ptr->m_twin->m_prev->m_incidentVertex;
			dBigPlane constrainPlane (UnboundedLoopPlane (i0, i1, i2, pool));
			table[i0].Accumulate (constrainPlane);

			i1 = ptr->m_prev->m_incidentVertex;
			i2 = ptr->m_prev->m_twin->m_prev->m_incidentVertex;
			constrainPlane = UnboundedLoopPlane (i0, i1, i2, pool);
			table[i0].Accumulate (constrainPlane);
		}

		ptr = ptr->m_twin->m_next;
	} while (ptr != edge);
}

void dPolyhedra::CalculateAllMetrics (dVertexCollapseVertexMetric* const table, const dBigVector* const pool) const
{
	dInt32 edgeMark = IncLRU();
	dPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dEdge* const edge = &(*iter);

		dAssert (edge);
		if (edge->m_mark != edgeMark) {

			if (edge->m_incidentFace > 0) {
				dInt32 i0 = edge->m_incidentVertex;
				dInt32 i1 = edge->m_next->m_incidentVertex;
				dInt32 i2 = edge->m_prev->m_incidentVertex;

				dBigPlane constrainPlane (EdgePlane (i0, i1, i2, pool));
				dVertexCollapseVertexMetric tmp (constrainPlane);

				dEdge* ptr = edge;
				do {
					ptr->m_mark = edgeMark;
					i0 = ptr->m_incidentVertex;
					table[i0].Accumulate(tmp);

					ptr = ptr->m_next;
				} while (ptr != edge);

			} else {
				dAssert (edge->m_twin->m_incidentFace > 0);
				dInt32 i0 = edge->m_twin->m_incidentVertex;
				dInt32 i1 = edge->m_twin->m_next->m_incidentVertex;
				dInt32 i2 = edge->m_twin->m_prev->m_incidentVertex;

				edge->m_mark = edgeMark;
				dBigPlane constrainPlane (UnboundedLoopPlane (i0, i1, i2, pool));
				dVertexCollapseVertexMetric tmp (constrainPlane);

				i0 = edge->m_incidentVertex;
				table[i0].Accumulate(tmp);

				i0 = edge->m_twin->m_incidentVertex;
				table[i0].Accumulate(tmp);
			}
		}
	}
}

bool dPolyhedra::IsOkToCollapse (const dBigVector* const pool, dEdge* const edge) const
{
	const dBigVector& q = pool[edge->m_incidentVertex];
	const dBigVector& p = pool[edge->m_twin->m_incidentVertex];
	for (dEdge* triangle = edge->m_prev->m_twin; triangle != edge->m_twin->m_next; triangle = triangle->m_prev->m_twin) {
		if (triangle->m_incidentFace > 0) {
			dAssert ((edge->m_incidentFace < 0) || (edge->m_incidentVertex == edge->m_next->m_next->m_next->m_incidentVertex));

			dBigVector originalArea (dBigVector::m_triplexMask & (pool[triangle->m_next->m_incidentVertex] - q).CrossProduct(pool[triangle->m_prev->m_incidentVertex] - q));
			dBigVector newArea (dBigVector::m_triplexMask & (pool[triangle->m_next->m_incidentVertex] - p).CrossProduct(pool[triangle->m_prev->m_incidentVertex] - p));

			dFloat64 projectedArea = newArea.DotProduct(originalArea).GetScalar();
			if (projectedArea <= dFloat64 (0.0f)) {
				return false;
			}

			dFloat64 mag20 = newArea.DotProduct(newArea).GetScalar();
			dFloat64 mag21 = originalArea.DotProduct(originalArea).GetScalar();;
			if ((projectedArea * projectedArea)  < (mag20 * mag21 * dFloat64 (1.0e-10f)))  {
				return false;
			}
		}
	}

	return true;
}

dEdge* dPolyhedra::OptimizeCollapseEdge (dEdge* const edge)
{
	dInt32 v0 = edge->m_incidentVertex;
	dInt32 v1 = edge->m_twin->m_incidentVertex;

#ifdef _DEBUG
	dPolyhedra::dgPairKey TwinKey (v1, v0);
	dPolyhedra::dNode* const node = Find (TwinKey.GetVal());
	dEdge* const twin1 = node ? &node->GetInfo() : nullptr;
	dAssert (twin1);
	dAssert (edge->m_twin == twin1);
	dAssert (twin1->m_twin == edge);
	dAssert (edge->m_incidentFace != 0);
	dAssert (twin1->m_incidentFace != 0);
	dAssert ((edge->m_incidentFace < 0) || (edge->m_incidentVertex == edge->m_next->m_next->m_next->m_incidentVertex));
	dAssert ((edge->m_twin->m_incidentFace < 0) || (edge->m_twin->m_incidentVertex == edge->m_twin->m_next->m_next->m_next->m_incidentVertex));
#endif

	dEdge* retEdge = edge->m_twin->m_prev->m_twin;
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

	dEdge* lastEdge = nullptr;
	dEdge* firstEdge = nullptr;
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

	for (dEdge* ptr = firstEdge; ptr != lastEdge; ptr = ptr->m_twin->m_next) {
		dEdge* badEdge = FindEdge (edge->m_twin->m_incidentVertex, ptr->m_twin->m_incidentVertex);
		if (badEdge) {
			return nullptr;
		}
	} 

	dEdge* const twin = edge->m_twin;
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

	dEdge* remapPtr = retEdge;
	do {
		dPolyhedra::dgPairKey pairKey (v0, remapPtr->m_twin->m_incidentVertex);
		dPolyhedra::dNode* const pairEdgeNode = Find (pairKey.GetVal());
		if (pairEdgeNode) {
			if (&pairEdgeNode->GetInfo() == remapPtr) {
				dPolyhedra::dgPairKey key (v1, remapPtr->m_twin->m_incidentVertex);
				remapPtr->m_incidentVertex = v1;
				ReplaceKey (pairEdgeNode, key.GetVal());
			} 
		}

		dPolyhedra::dgPairKey twinKey1 (remapPtr->m_twin->m_incidentVertex, v0);
		dPolyhedra::dNode* const pairTwinNode = Find (twinKey1.GetVal());
		if (pairTwinNode) {
			if (&pairTwinNode->GetInfo() == remapPtr->m_twin) {
				dPolyhedra::dgPairKey key (remapPtr->m_twin->m_incidentVertex, v1);
				ReplaceKey (pairTwinNode, key.GetVal());
			}
		}

		remapPtr = remapPtr->m_twin->m_next;
	} while (remapPtr != retEdge);

	return retEdge;
}

dFloat64 dPolyhedra::EdgePenalty (const dBigVector* const pool, dEdge* const edge, dFloat64 dist) const
{
	dInt32 i0 = edge->m_incidentVertex;
	dInt32 i1 = edge->m_next->m_incidentVertex;

	dFloat32 maxPenalty = dFloat32 (1.0e14f);

	const dBigVector& p0 = pool[i0];
	const dBigVector& p1 = pool[i1];
	dBigVector dp (p1 - p0);

	dAssert(dp.m_w == dFloat32(0.0f));
	dFloat64 dot = dp.DotProduct(dp).GetScalar();;
	if (dot < dFloat64(1.0e-6f)) {
		return dist * maxPenalty;
	}

	if ((edge->m_incidentFace > 0) && (edge->m_twin->m_incidentFace > 0)) {
		dBigVector edgeNormal (FaceNormal (edge, &pool[0].m_x, sizeof (dBigVector)));
		dBigVector twinNormal (FaceNormal (edge->m_twin, &pool[0].m_x, sizeof (dBigVector)));

		dFloat64 mag0 = edgeNormal.DotProduct(edgeNormal).GetScalar();
		dFloat64 mag1 = twinNormal.DotProduct(twinNormal).GetScalar();
		if ((mag0 < dFloat64 (1.0e-24f)) || (mag1 < dFloat64 (1.0e-24f))) {
			return dist * maxPenalty;
		}

		edgeNormal = edgeNormal.Scale (dFloat64 (1.0f) / sqrt(mag0));
		twinNormal = twinNormal.Scale (dFloat64 (1.0f) / sqrt(mag1));

		dot = edgeNormal.DotProduct(twinNormal).GetScalar();;
		if (dot < dFloat64 (-0.9f)) {
			return dist * maxPenalty;
		}

		dEdge* ptr = edge;
		do {
			if ((ptr->m_incidentFace <= 0) || (ptr->m_twin->m_incidentFace <= 0)){
				dEdge* const adj = edge->m_twin;
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

	dInt32 faceA = edge->m_incidentFace;
	dInt32 faceB = edge->m_twin->m_incidentFace;

	i0 = edge->m_twin->m_incidentVertex;
	dBigVector p (pool[i0].m_x, pool[i0].m_y, pool[i0].m_z, dFloat32 (0.0f));

	bool penalty = false;
	dEdge* ptr = edge;
	do {
		dEdge* const adj = ptr->m_twin;

		dInt32 face = adj->m_incidentFace;
		if ((face != faceB) && (face != faceA) && (face >= 0) && (adj->m_next->m_incidentFace == face) && (adj->m_prev->m_incidentFace == face)){

			dInt32 k0 = adj->m_next->m_incidentVertex;
			const dBigVector& q0 = pool[k0];

			dInt32 k1 = adj->m_incidentVertex;
			const dBigVector& q1 = pool[k1];

			dInt32 k2 = adj->m_prev->m_incidentVertex;
			const dBigVector& q2 = pool[k2];

			dBigVector n0 (dBigVector::m_triplexMask & (q1 - q0).CrossProduct(q2 - q0));
			dBigVector n1 (dBigVector::m_triplexMask & (q1 - p).CrossProduct(q2 - p));
			dFloat64 project = n0.DotProduct(n1).GetScalar();
			if (project < dFloat64 (0.0f)) {
				penalty = true;
				break;
			}
		}

		ptr = ptr->m_twin->m_next;
	} while (ptr != edge);

	dFloat64 aspect = dFloat32 (0.0f);
	if (!penalty) {
		dInt32 k0 = edge->m_twin->m_incidentVertex;
		dBigVector q0 (pool[k0]);

		aspect = dFloat32 (1.0f);
		for (dEdge* ptr1 = edge->m_twin->m_next->m_twin->m_next; ptr1 != edge; ptr1 = ptr1->m_twin->m_next) {
			if (ptr1->m_incidentFace > 0) {
				dInt32 k1 = ptr1->m_next->m_incidentVertex;
				const dBigVector& q1 = pool[k1];

				dInt32 k2 = ptr1->m_prev->m_incidentVertex;
				const dBigVector& q2 = pool[k2];

				dBigVector e0 (q1 - q0);
				dBigVector e1 (q2 - q1);
				dBigVector e2 (q0 - q2);
				dAssert(e0.m_w == dFloat32(0.0f));
				dAssert(e1.m_w == dFloat32(0.0f));
				dAssert(e2.m_w == dFloat32(0.0f));

				dFloat64 mag0 = e0.DotProduct(e0).GetScalar();
				dFloat64 mag1 = e1.DotProduct(e1).GetScalar();
				dFloat64 mag2 = e2.DotProduct(e2).GetScalar();
				dFloat64 maxMag = dMax (mag0, mag1, mag2);
				dFloat64 minMag = dMin (mag0, mag1, mag2);
				dFloat64 ratio = minMag / maxMag;

				if (ratio < aspect) {
					aspect = ratio;
				}
			}
		}
		aspect = dFloat32 (1.0f) - aspect;
	}
	return aspect * aspect * dist;
}

bool dPolyhedra::Optimize (const dFloat64* const array, dInt32 strideInBytes, dFloat64 tol, dInt32 maxFaceCount)
{
	dInt32 stride = dInt32 (strideInBytes / sizeof (dFloat64));

#ifdef __ENABLE_DG_CONTAINERS_SANITY_CHECK 
	dAssert (SanityCheck ());
#endif

	dFloat32 progressDen = dFloat32 (1.0f / GetEdgeCount());
	dInt32 edgeCount = GetEdgeCount() * 4 + D_LOCAL_BUFFER_SIZE * 16;
	dInt32 maxVertexIndex = GetLastVertexIndex();
	
	dStack<dBigVector> vertexPool (maxVertexIndex); 
	dStack<dVertexCollapseVertexMetric> vertexMetrics (maxVertexIndex + 512); 

	dList <dEdgeCollapseEdgeHandle> edgeHandleList;
	dStack<char> heapPool (2 * edgeCount * dInt32 (sizeof (dFloat64) + sizeof (dEdgeCollapseEdgeHandle*) + sizeof (dInt32))); 
	dUpHeap<dList <dEdgeCollapseEdgeHandle>::dNode* , dFloat64> bigHeapArray(&heapPool[0], heapPool.GetSizeInBytes());

	for (dInt32 i = 0; i < maxVertexIndex; i ++) 
	{
		vertexPool[i].m_x = array[i * stride + 0];
		vertexPool[i].m_y = array[i * stride + 1];
		vertexPool[i].m_z = array[i * stride + 2];
		vertexPool[i].m_w= dFloat64 (0.0f);
	}

	memset (&vertexMetrics[0], 0, maxVertexIndex * sizeof (dVertexCollapseVertexMetric));
	CalculateAllMetrics (&vertexMetrics[0], &vertexPool[0]);

	const dFloat64 maxCost = dFloat32 (1.0e-3f);
	dFloat64 tol2 = tol * tol;
	dFloat64 distTol = dMax (tol2, dFloat64 (1.0e-12f));
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) 
	{
		dEdge* const edge = &(*iter);

		edge->m_userData = 0;
		dInt32 index0 = edge->m_incidentVertex;
		dInt32 index1 = edge->m_twin->m_incidentVertex;

		dVertexCollapseVertexMetric &metric = vertexMetrics[index0];
		const dBigVector& p = vertexPool[index1];
		dFloat64 faceCost = metric.Evalue (p); 
		dFloat64 edgePenalty = EdgePenalty (&vertexPool[0], edge, distTol);
		dAssert (edgePenalty >= dFloat32 (0.0f));
		dEdgeCollapseEdgeHandle handle (edge);
		dList <dEdgeCollapseEdgeHandle>::dNode* handleNodePtr = edgeHandleList.Addtop (handle);
		bigHeapArray.Push (handleNodePtr, faceCost + edgePenalty);
	}

	bool progress = true;
	dInt32 interPasses = 0;
	dInt32 faceCount = GetFaceCount();
	while (bigHeapArray.GetCount() && (bigHeapArray.Value() < maxCost) && ((bigHeapArray.Value() < tol2) || (faceCount > maxFaceCount)) && progress ) 
	{
		dList <dEdgeCollapseEdgeHandle>::dNode* const handleNodePtr = bigHeapArray[0];

		dEdge* edge = handleNodePtr->GetInfo().m_edge;
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
					progress = ReportProgress(dFloat32(1.0f) - GetEdgeCount() * progressDen);
				}

				if (bigHeapArray.GetCount() > (bigHeapArray.GetMaxCount() - 100)) 
				{
					for(dInt32 i = bigHeapArray.GetCount() - 1; i >= 0; i --) 
					{
						dList <dEdgeCollapseEdgeHandle>::dNode* const emptyHandle = bigHeapArray[i];
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
					dEdge* ptr = edge;
					do 
					{
						CalculateVertexMetrics (&vertexMetrics[0], &vertexPool[0], ptr->m_twin);
						ptr = ptr->m_twin->m_next;
					} while (ptr != edge);

					// calculate edge cost of all incident edges
					dInt32 mark = IncLRU();
					ptr = edge;
					do 
					{
						dAssert (ptr->m_mark != mark);
						ptr->m_mark = mark;

						dInt32 index0 = ptr->m_incidentVertex;
						dInt32 index1 = ptr->m_twin->m_incidentVertex;

						dVertexCollapseVertexMetric &metric = vertexMetrics[index0];
						const dBigVector& p = vertexPool[index1];

						dFloat64 faceCost = metric.Evalue (p); 
						dFloat64 edgePenalty = EdgePenalty (&vertexPool[0], ptr, distTol);
						dAssert (edgePenalty >= dFloat32 (0.0f));
						dEdgeCollapseEdgeHandle handle (ptr);
						dList <dEdgeCollapseEdgeHandle>::dNode* handleNodePtr1 = edgeHandleList.Addtop (handle);
						bigHeapArray.Push (handleNodePtr1, faceCost + edgePenalty);

						ptr = ptr->m_twin->m_next;
					} while (ptr != edge);

					// calculate edge cost of all incident edges to a surrounding vertex
					ptr = edge;
					do 
					{
						dEdge* const incidentEdge = ptr->m_twin;		

						dEdge* ptr1 = incidentEdge;
						do 
						{
							dInt32 index0 = ptr1->m_incidentVertex;
							dInt32 index1 = ptr1->m_twin->m_incidentVertex;

							if (ptr1->m_mark != mark) 
							{
								ptr1->m_mark = mark;
								dVertexCollapseVertexMetric &metric = vertexMetrics[index0];
								const dBigVector& p = vertexPool[index1];

								dFloat64 faceCost = metric.Evalue (p); 
								dFloat64 edgePenalty = EdgePenalty (&vertexPool[0], ptr1, distTol);
								dAssert (edgePenalty >= dFloat32 (0.0f));
								dEdgeCollapseEdgeHandle handle (ptr1);
								dList <dEdgeCollapseEdgeHandle>::dNode* handleNodePtr1 = edgeHandleList.Addtop (handle);
								bigHeapArray.Push (handleNodePtr1, faceCost + edgePenalty);
							}

							if (ptr1->m_twin->m_mark != mark) 
							{
								ptr1->m_twin->m_mark = mark;
								dVertexCollapseVertexMetric &metric = vertexMetrics[index1];
								const dBigVector& p = vertexPool[index0];
								dFloat64 faceCost = metric.Evalue (p); 
								dFloat64 edgePenalty = EdgePenalty (&vertexPool[0], ptr1->m_twin, distTol);
								dAssert (edgePenalty >= dFloat32 (0.0f));
								dEdgeCollapseEdgeHandle handle (ptr1->m_twin);
								dList <dEdgeCollapseEdgeHandle>::dNode* handleNodePtr1 = edgeHandleList.Addtop (handle);
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

	progress = ReportProgress(dFloat32 (1.0f));
	return progress;
}

bool dPolyhedra::TriangulateFace(dEdge* const face, const dFloat64* const pool, dInt32 strideInBytes)
{
	if (face->m_next->m_next->m_next != face) 
	{
		dInt32 mark = IncLRU();
		dEdge* ptr = face;
		do 
		{
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != face);
		char memPool[D_LOCAL_BUFFER_SIZE * (sizeof (dEdge*)+sizeof (dFloat64))];
		dDownHeap<dEdge*, dFloat64> heap(&memPool[0], sizeof (memPool));

		dInt32 stride = dInt32(strideInBytes / sizeof (dFloat64));
		dEdge* const edge = TriangulateFace(face, pool, stride, heap, nullptr);
		dAssert(!edge);
		return !edge;
	}
	return true;
}

dEdge* dPolyhedra::BestEdgePolygonizeFace(const dBigVector& normal, dEdge* const edge, const dFloat64* const pool, dInt32 stride, const dBigVector& point) const
{
	dBigVector p0(dBigVector::m_triplexMask & dBigVector(&pool[edge->m_incidentVertex * stride]));
	dBigVector r(point - p0);
	dEdge* e0 = edge;
	do 
	{
		dBigVector p1(dBigVector::m_triplexMask & dBigVector(&pool[e0->m_twin->m_incidentVertex * stride]));
		dBigVector p2(dBigVector::m_triplexMask & dBigVector(&pool[e0->m_prev->m_incidentVertex * stride]));
		dFloat64 test0 = r.DotProduct(normal.CrossProduct(p1 - p0)).GetScalar();
		dFloat64 test1 = r.DotProduct((p2 - p0).CrossProduct(normal)).GetScalar();
		
		if ((test0 > 0.0f) && (test1 > 0.0f)) 
		{
			break;
		}
		e0 = e0->m_prev->m_twin;
	} while (e0 != edge);
	return e0;
}

bool dPolyhedra::PolygonizeFace(dEdge* const face, const dFloat64* const pool, dInt32 strideInBytes)
{
	dPolyhedra flatFace;
	dEdge* array[D_LOCAL_BUFFER_SIZE];

	dInt32 count = 0;		
	dEdge* edge = face;
	do 
	{
		dEdge* const perimeter = flatFace.AddHalfEdge (edge->m_incidentVertex, edge->m_twin->m_incidentVertex);
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

	dInt32 i0 = count - 1;
	for(dInt32 i = 0; i < count; i ++) 
	{
		dEdge* const edge1 = array[i];
		dEdge* const prev1 = array[i0];

		edge1->m_prev = prev1;
		prev1->m_next = edge1;
		i0 = i;
	} 

	for(dInt32 i = 0; i < count; i ++) 
	{
		dEdge* const edge1 = array[i];
		dEdge* const twin1 = flatFace.FindEdge (edge1->m_next->m_incidentVertex, edge1->m_incidentVertex);
		if (twin1) 
		{
			twin1->m_twin = edge1;
			edge1->m_twin = twin1;
		} 
		else 
		{
			dEdge* const perimeter = flatFace.AddHalfEdge (edge1->m_next->m_incidentVertex, edge1->m_incidentVertex);
			perimeter->m_twin = edge1;
			edge1->m_twin = perimeter;
			perimeter->m_incidentFace = -1;
			perimeter->m_prev = nullptr;
			perimeter->m_next = nullptr;
		}
	}

	for (dInt32 i = 0; i < count; i++) 
	{
		dEdge* const edge1 = array[i];
		dEdge* const twin1 = edge1->m_twin;
		if (!twin1->m_next) 
		{
			dEdge* next = edge1->m_prev->m_twin;
			while (next->m_prev) 
			{
				next = next->m_prev->m_twin;
			}
			twin1->m_next = next;	
			next->m_prev = next;
		}
	}

	dBigVector normal (flatFace.FaceNormal(array[0], pool, strideInBytes));
	if (flatFace.TriangulateFace(array[0], pool, strideInBytes)) 
	{
		dInt32 stride = dInt32(strideInBytes / sizeof (dFloat64));
		flatFace.RefineTriangulation(pool, stride);

		//RemoveOuterColinearEdges(*this, vertex, stride);
		dInt32 polygon[D_LOCAL_BUFFER_SIZE];
		dEdge* diagonalsPool[D_LOCAL_BUFFER_SIZE];

		dInt32 diagonalCount = GetInteriorDiagonals(flatFace, diagonalsPool, sizeof (diagonalsPool) / sizeof (diagonalsPool[0]));

		if (diagonalCount) 
		{
			dEdge* edge1 = &flatFace.GetRoot()->GetInfo();
			if (edge1->m_incidentFace < 0) 
			{
				edge1 = edge1->m_twin;
			}

			dAssert(edge1->m_incidentFace > 0);

			dBigVector normal1(flatFace.FaceNormal(edge1, pool, strideInBytes));
			normal1 = normal1.Scale(dFloat64(1.0f) / sqrt(normal1.DotProduct(normal1).GetScalar()));

			edge1 = nullptr;
			dPolyhedra::Iterator iter0(flatFace);
			for (iter0.Begin(); iter0; iter0++) 
			{
				edge1 = &(*iter0);
				if (edge1->m_incidentFace < 0) 
				{
					break;
				}
			}
			dAssert(edge1);

			dInt32 isConvex = 1;
			dEdge* ptr = edge1;
			dInt32 mark = flatFace.IncLRU();

			dBigVector normal2(normal1);
			dBigVector p0(dBigVector::m_triplexMask & dBigVector(&pool[ptr->m_prev->m_incidentVertex * stride]));
			dBigVector p1(dBigVector::m_triplexMask & dBigVector(&pool[ptr->m_incidentVertex * stride]));
			dBigVector e0(p1 - p0);

			dAssert(normal2.m_w == dFloat32(0.0f));
			e0 = e0.Scale(dFloat64(1.0f) / sqrt(e0.DotProduct(e0).GetScalar() + dFloat64(1.0e-24f)));
			do 
			{
				dBigVector p2(dBigVector::m_triplexMask & dBigVector(&pool[ptr->m_next->m_incidentVertex * stride]));
				dBigVector e1(p2 - p1);
				e1 = e1.Scale(dFloat64(1.0f) / sqrt(e1.DotProduct(e1).GetScalar() + dFloat32(1.0e-24f)));
				dFloat64 dot = normal2.DotProduct(e0.CrossProduct(e1)).GetScalar();
				
				if (dot > dFloat32(5.0e-3f)) 
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
				dPolyhedra::Iterator iter(flatFace);
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
					dInt32 count1 = 0;
					ptr = edge1;
					do 
					{
						polygon[count1] = ptr->m_incidentVertex;
						count1++;
						dAssert(count1 < dInt32(sizeof (polygon) / sizeof (polygon[0])));
						ptr = ptr->m_next;
					} while (ptr != edge1);

					for (dInt32 i = 0; i < count1 - 1; i++) 
					{
						for (dInt32 j = i + 1; j < count1; j++) 
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
				for (dInt32 j = 0; j < diagonalCount; j++) 
				{
					dEdge* const diagonal = diagonalsPool[j];
					flatFace.DeleteEdge(diagonal);
				}
			} 
			else 
			{
				for (dInt32 j = 0; j < diagonalCount; j++) 
				{
					dEdge* const diagonal = diagonalsPool[j];
					if (!IsEssensialDiagonal(diagonal, normal1, pool, stride)) 
					{
						flatFace.DeleteEdge(diagonal);
					}
				}
			}
		}

		dInt32 mark = flatFace.IncLRU();
		dPolyhedra::Iterator iter0(flatFace);
		for (iter0.Begin(); iter0; iter0++) 
		{
			dEdge* const edge1 = &(*iter0);
			if ((edge1->m_mark != mark) && (edge1->m_incidentFace > 0)) 
			{
				edge1->m_mark = mark;
				edge1->m_twin->m_mark = mark;
				if (!FindEdge(edge1->m_incidentVertex, edge1->m_twin->m_incidentVertex)) 
				{
					dgPairKey key0 (edge1->m_incidentVertex, 0);
					dgPairKey key1 (edge1->m_twin->m_incidentVertex, 0);
					dNode* const node0 = FindGreater (key0.GetVal());
					dNode* const node1 = FindGreater (key1.GetVal());
					dAssert (node0);
					dAssert (node1);
					dEdge* e0 = &node0->GetInfo();
					dEdge* e1 = &node1->GetInfo();

					dBigVector p0 (dBigVector::m_triplexMask & dBigVector(&pool[e0->m_incidentVertex * stride]));
					dBigVector p1 (dBigVector::m_triplexMask & dBigVector(&pool[e1->m_incidentVertex * stride]));
					e0 = BestEdgePolygonizeFace (normal, e0, pool, stride, p1);
					e1 = BestEdgePolygonizeFace (normal, e1, pool, stride, p0);
					ConnectVertex (e0, e1);
				}
			}
		}
	}

	return true;
}

void dPolyhedra::RemoveInteriorEdges (dPolyhedra& buildConvex, const dFloat64* const vertex, dInt32 strideInBytes)
{
	dInt32 polygon[D_LOCAL_BUFFER_SIZE * 8];
	dEdge* diagonalsPool[D_LOCAL_BUFFER_SIZE * 8];

	dInt32 stride = dInt32 (strideInBytes / sizeof (dFloat64));

	buildConvex.BeginFace();
	dPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter;) 
	{
		dEdge* edge = &(*iter);
		iter++;
		if (edge->m_incidentFace > 0) 
		{

			dPolyhedra flatFace;
			MarkAdjacentCoplanarFaces(flatFace, edge, vertex, strideInBytes);
			if (flatFace.GetCount()) 
			{
				//flatFace.RefineTriangulation(vertex, stride);
				RemoveOuterColinearEdges(flatFace, vertex, stride);
				RemoveInteriorColinearEdges(flatFace, vertex, stride);
				flatFace.RefineTriangulation(vertex, stride);

				dInt32 diagonalCount = GetInteriorDiagonals(flatFace, diagonalsPool, sizeof(diagonalsPool) / sizeof(diagonalsPool[0]));
				if (diagonalCount) 
				{
					edge = &flatFace.GetRoot()->GetInfo();
					if (edge->m_incidentFace < 0) 
					{
						edge = edge->m_twin;
					}
					dAssert(edge->m_incidentFace > 0);

					dBigVector normal(FaceNormal(edge, vertex, strideInBytes));
					normal = normal.Scale(dFloat64(1.0f) / sqrt(normal.DotProduct(normal).GetScalar()));

					edge = nullptr;
					dPolyhedra::Iterator iter1(flatFace);
					for (iter1.Begin(); iter1; iter1++) 
					{
						edge = &(*iter1);
						if (edge->m_incidentFace < 0) 
						{
							break;
						}
					}
					dAssert(edge);

					dInt32 isConvex = 1;
					dEdge* ptr = edge;
					dInt32 mark = flatFace.IncLRU();

					dBigVector normal2(normal);
					dBigVector p0(dBigVector::m_triplexMask & dBigVector(&vertex[ptr->m_prev->m_incidentVertex * stride]));
					dBigVector p1(dBigVector::m_triplexMask & dBigVector(&vertex[ptr->m_incidentVertex * stride]));
					dBigVector e0(p1 - p0);
					e0 = e0.Scale(dFloat64(1.0f) / sqrt(e0.DotProduct(e0).GetScalar() + dFloat64(1.0e-24f)));
					do 
					{
						dBigVector p2(dBigVector::m_triplexMask & dBigVector(&vertex[ptr->m_next->m_incidentVertex * stride]));
						dBigVector e1(p2 - p1);
						e1 = e1.Scale(dFloat64(1.0f) / sqrt(e1.DotProduct(e1).GetScalar() + dFloat32(1.0e-24f)));
						dFloat64 dot = normal2.DotProduct(e0.CrossProduct(e1)).GetScalar();

						if (dot > dFloat32(5.0e-3f)) 
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
						dPolyhedra::Iterator iter2(flatFace);
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
							dInt32 count = 0;
							ptr = edge;
							do 
							{
								polygon[count] = ptr->m_incidentVertex;
								count++;
								dAssert(count < dInt32(sizeof(polygon) / sizeof(polygon[0])));
								ptr = ptr->m_next;
							} while (ptr != edge);

							for (dInt32 i = 0; i < count - 1; i++) 
							{
								for (dInt32 j = i + 1; j < count; j++) 
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
						for (dInt32 j = 0; j < diagonalCount; j++) 
						{
							dEdge* const diagonal = diagonalsPool[j];
							flatFace.DeleteEdge(diagonal);
						}
					} 
					else 
					{
						for (dInt32 j = 0; j < diagonalCount; j++) 
						{
							dEdge* const diagonal = diagonalsPool[j];
							if (!IsEssensialDiagonal(diagonal, normal, vertex, stride)) 
							{
								flatFace.DeleteEdge(diagonal);
							}
						}
					}
				}

				dInt32 mark = flatFace.IncLRU();
				dPolyhedra::Iterator iter1(flatFace);
				for (iter1.Begin(); iter1; iter1++) 
				{
					dEdge* const edge1 = &(*iter1);
					if (edge1->m_mark != mark) 
					{
						if (edge1->m_incidentFace > 0) 
						{
							dEdge* ptr = edge1;
							dInt32 diagonalCount1 = 0;
							do 
							{
								polygon[diagonalCount1] = ptr->m_incidentVertex;
								diagonalCount1++;
								dAssert(diagonalCount1 < dInt32(sizeof(polygon) / sizeof(polygon[0])));
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

void dPolyhedra::ConvexPartition (const dFloat64* const vertex, dInt32 strideInBytes, dPolyhedra* const leftOversOut)
{
	if (GetCount()) 
	{
		Triangulate (vertex, strideInBytes, leftOversOut);
		DeleteDegenerateFaces (vertex, strideInBytes, dFloat32 (1.0e-5f));
		Optimize (vertex, strideInBytes, dFloat32 (1.0e-3f));
		DeleteDegenerateFaces (vertex, strideInBytes, dFloat32 (1.0e-5f));

		if (GetCount()) 
		{
			dPolyhedra buildConvex;
			RemoveInteriorEdges (buildConvex, vertex, strideInBytes);
			SwapInfo(buildConvex);
		}
	}
}


dMatrix dPolyhedra::CalculateSphere(dBigVector& size, const dFloat64* const vertex, dInt32 strideInBytes) const
{
	dInt32 stride = dInt32(strideInBytes / sizeof(dFloat64));

	dInt32 vertexCount = 0;
	dInt32 mark = IncLRU();
	dPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const edge = &(*iter);
		if (edge->m_mark != mark) 
		{
			dEdge* ptr = edge;
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
	dInt32 vertexCountIndex = 0;
	dStack<dBigVector> pool(vertexCount);
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const edge = &(*iter);
		if (edge->m_mark != mark) 
		{
			dEdge* ptr = edge;
			do 
			{
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			dInt32 incidentVertex = edge->m_incidentVertex * stride;
			pool[vertexCountIndex] = dBigVector(vertex[incidentVertex + 0], vertex[incidentVertex + 1], vertex[incidentVertex + 2], dFloat32(0.0f));
			vertexCountIndex++;
		}
	}
	dAssert(vertexCountIndex <= vertexCount);

	//dMatrix axis(dGetIdentityMatrix());
	//dgObb sphere(axis);
	dConvexHull3d convexHull(&pool[0].m_x, sizeof(dBigVector), vertexCountIndex, 0.0f);

	size = dBigVector::m_zero;
	dMatrix sphere(dGetIdentityMatrix());
	if (convexHull.GetCount()) 
	{
		dStack<dInt32> triangleList(convexHull.GetCount() * 3);
		dInt32 trianglesCount = 0;
		for (dConvexHull3d::dNode* node = convexHull.GetFirst(); node; node = node->GetNext()) 
		{
			dConvexHull3dFace* const face = &node->GetInfo();
			triangleList[trianglesCount * 3 + 0] = face->m_index[0];
			triangleList[trianglesCount * 3 + 1] = face->m_index[1];
			triangleList[trianglesCount * 3 + 2] = face->m_index[2];
			trianglesCount++;
			dAssert((trianglesCount * 3) <= triangleList.GetElementsCount());
		}
		
		//dVector* const dst = (dVector*)&pool[0].m_x;
		for (dInt32 i = 0; i < convexHull.GetVertexCount(); i++) 
		{
			pool[i] = convexHull.GetVertex(i);
		}

		dVector eigen;
		dBigVector var(dBigVector::m_zero);
		dBigVector cov(dBigVector::m_zero);
		dBigVector origin(dBigVector::m_zero);

		for (dInt32 i = 0; i < vertexCount; i++) 
		{
			const dBigVector p(pool[i] & dBigVector::m_triplexMask);
			const dBigVector q(p.ShiftTripleLeft());
			origin += p;
			var += p * p;
			cov += p * q;
		}
		dSwap(cov.m_y, cov.m_z);

		dFloat64 k = dFloat64(1.0) / vertexCount;
		var = var.Scale(k);
		cov = cov.Scale(k);
		origin = origin.Scale(k);
		
		dFloat64 Ixx = var.m_x - origin.m_x * origin.m_x;
		dFloat64 Iyy = var.m_y - origin.m_y * origin.m_y;
		dFloat64 Izz = var.m_z - origin.m_z * origin.m_z;
		
		dFloat64 Ixy = cov.m_x - origin.m_x * origin.m_y;
		dFloat64 Ixz = cov.m_y - origin.m_x * origin.m_z;
		dFloat64 Iyz = cov.m_z - origin.m_y * origin.m_z;
		
		sphere.m_front = dVector(dFloat32(Ixx), dFloat32(Ixy), dFloat32(Ixz), dFloat32(0.0f));
		sphere.m_up    = dVector(dFloat32(Ixy), dFloat32(Iyy), dFloat32(Iyz), dFloat32(0.0f));
		sphere.m_right = dVector(dFloat32(Ixz), dFloat32(Iyz), dFloat32(Izz), dFloat32(0.0f));
		//dVector eigenValues(sphere.EigenVectors());

		dVector minVal(dFloat32(1e15f));
		dVector maxVal(dFloat32(-1e15f));
		for (dInt32 i = 0; i < vertexCount; i++)
		{
			dVector tmp(sphere.UnrotateVector(pool[i]));
			minVal = minVal.GetMin(tmp);
			maxVal = maxVal.GetMax(tmp);
		}

		dVector massCenter((maxVal + minVal) * dVector::m_half);
		massCenter.m_w = dFloat32(1.0f);
		sphere.m_posit = sphere.TransformVector(massCenter);
		size = dVector ((maxVal - minVal) * dVector::m_half);
	}
	return sphere;
}
