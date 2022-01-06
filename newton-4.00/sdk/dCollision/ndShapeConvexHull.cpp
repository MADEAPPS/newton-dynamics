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
#include "ndCollisionStdafx.h"
#include "ndShapeInstance.h"
#include "ndShapeConvexHull.h"

#define D_CONVEX_VERTEX_SPLITE_SIZE	48
D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndShapeConvexHull)

D_MSV_NEWTON_ALIGN_32
class ndShapeConvexHull::ndConvexBox
{
	public:
	ndVector m_box[2];
	ndInt32 m_vertexStart;
	ndInt32 m_vertexCount;
	ndInt32 m_soaVertexStart;
	ndInt32 m_soaVertexCount;
	ndInt32 m_leftBox;
	ndInt32 m_rightBox;
} D_GCC_NEWTON_ALIGN_32;


ndShapeConvexHull::ndShapeConvexHull (ndInt32 count, ndInt32 strideInBytes, ndFloat32 tolerance, const ndFloat32* const vertexArray)
	:ndShapeConvex(m_convexHull)
	,m_supportTree(nullptr)
	,m_faceArray(nullptr)
	,m_soa_x(nullptr)
	,m_soa_y(nullptr)
	,m_soa_z(nullptr)
	,m_soa_index(nullptr)
	,m_vertexToEdgeMapping(nullptr)
	,m_faceCount(0)
	,m_soaVertexCount(0)
	,m_supportTreeCount(0)
{
	m_edgeCount = 0;
	m_vertexCount = 0;
	m_vertex = nullptr;
	m_simplex = nullptr;
	Create(count, strideInBytes, vertexArray, tolerance);
}

ndShapeConvexHull::ndShapeConvexHull(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndShapeConvex(m_convexHull)
	,m_supportTree(nullptr)
	,m_faceArray(nullptr)
	,m_soa_x(nullptr)
	,m_soa_y(nullptr)
	,m_soa_z(nullptr)
	,m_soa_index(nullptr)
	,m_vertexToEdgeMapping(nullptr)
	,m_faceCount(0)
	,m_soaVertexCount(0)
	,m_supportTreeCount(0)
{
	m_edgeCount = 0;
	m_vertexCount = 0;
	m_vertex = nullptr;
	m_simplex = nullptr;

	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	ndArray<ndVector> array;
	xmlGetFloatArray3(xmlNode, "vextexArray3", array);
	Create(array.GetCount(), sizeof (ndVector), &array[0].m_x, ndFloat32 (0.0f));
}

ndShapeConvexHull::~ndShapeConvexHull()
{
	if (m_vertexToEdgeMapping) 
	{
		ndMemory::Free(m_vertexToEdgeMapping);
	}
	
	if (m_faceArray) 
	{
		ndMemory::Free(m_faceArray);
	}
	
	if (m_supportTree) 
	{
		ndMemory::Free(m_supportTree);
	}
	
	if (m_soa_index)
	{
		ndMemory::Free(m_soa_x);
		ndMemory::Free(m_soa_y);
		ndMemory::Free(m_soa_z);
		ndMemory::Free(m_soa_index);
	}
}

bool ndShapeConvexHull::Create(ndInt32 count, ndInt32 strideInBytes, const ndFloat32* const vertexArray, ndFloat32 tolerance)
{
	ndInt32 stride = strideInBytes / sizeof(ndFloat32);
	ndStack<ndBigVector> buffer(2 * count);
	for (ndInt32 i = 0; i < count; i++) 
	{
		buffer[i] = ndVector(vertexArray[i * stride + 0], vertexArray[i * stride + 1], vertexArray[i * stride + 2], ndFloat32(0.0f));
	}

	ndConvexHull3d* convexHull = new ndConvexHull3d(&buffer[0].m_x, sizeof (ndBigVector), count, tolerance);
	if (!convexHull->GetCount()) 
	{
		dAssert(0);
		//// this is a degenerated hull hull to add some thickness and for a thick plane
		//delete convexHull;
		//
		//ndStack<ndVector> tmp(3 * count);
		//for (ndInt32 i = 0; i < count; i++) 
		//{
		//	tmp[i][0] = ndFloat32(buffer[i].m_x);
		//	tmp[i][1] = ndFloat32(buffer[i].m_y);
		//	tmp[i][2] = ndFloat32(buffer[i].m_z);
		//	tmp[i][3] = ndFloat32(0.0f);
		//}
		//
		//dObb sphere;
		//sphere.SetDimensions(&tmp[0][0], sizeof(ndVector), count);
		//
		//ndInt32 index = 0;
		//ndFloat32 size = ndFloat32(1.0e10f);
		//for (ndInt32 i = 0; i < 3; i++) 
		//{
		//	if (sphere.m_size[i] < size) 
		//	{
		//		index = i;
		//		size = sphere.m_size[i];
		//	}
		//}
		//ndVector normal(ndFloat32(0.0f));
		//normal[index] = ndFloat32(1.0f);
		//ndVector step = sphere.RotateVector(normal.Scale(ndFloat32(0.05f)));
		//for (ndInt32 i = 0; i < count; i++) 
		//{
		//	ndVector p1(tmp[i] + step);
		//	ndVector p2(tmp[i] - step);
		//
		//	buffer[i * 3 + 0] = p1.m_x;
		//	buffer[i * 3 + 1] = p1.m_y;
		//	buffer[i * 3 + 2] = p1.m_z;
		//	buffer[(i + count) * 3 + 0] = p2.m_x;
		//	buffer[(i + count) * 3 + 1] = p2.m_y;
		//	buffer[(i + count) * 3 + 2] = p2.m_z;
		//}
		//count *= 2;
		//convexHull = new ndConvexHull3d(&buffer[0].m_x, sizeof(ndBigVector), count, tolerance);
		//if (!convexHull->GetCount()) 
		//{
		//	delete convexHull;
		//	return false;
		//}
	}

	// check for degenerated faces
	for (bool success = false; !success; ) 
	{
		success = true;
		const ndBigVector* const hullVertexArray = convexHull->GetVertexPool();

		ndStack<ndInt8> mask(convexHull->GetVertexCount());
		memset(&mask[0], 1, mask.GetSizeInBytes());
		for (ndConvexHull3d::ndNode* node = convexHull->GetFirst(); node; node = node->GetNext()) 
		{
			ndConvexHull3dFace& face = node->GetInfo();
			const ndBigVector& p0 = hullVertexArray[face.m_index[0]];
			const ndBigVector& p1 = hullVertexArray[face.m_index[1]];
			const ndBigVector& p2 = hullVertexArray[face.m_index[2]];
			dAssert(p0.m_w == p1.m_w);
			dAssert(p0.m_w == p2.m_w);
			ndBigVector p1p0(p1 - p0);
			ndBigVector p2p0(p2 - p0);
			ndBigVector normal(p2p0.CrossProduct(p1p0));
			ndFloat64 mag2 = normal.DotProduct(normal).GetScalar();
			if (mag2 < ndFloat64(1.0e-6f * 1.0e-6f)) 
			{
				success = false;
				ndInt32 index = -1;
				ndBigVector p2p1(p2 - p1);
				ndFloat64 dist10 = p1p0.DotProduct(p1p0).GetScalar();
				ndFloat64 dist20 = p2p0.DotProduct(p2p0).GetScalar();
				ndFloat64 dist21 = p2p1.DotProduct(p2p1).GetScalar();
				if ((dist10 >= dist20) && (dist10 >= dist21)) 
				{
					index = 2;
				}
				else if ((dist20 >= dist10) && (dist20 >= dist21)) 
				{
					index = 1;
				}
				else if ((dist21 >= dist10) && (dist21 >= dist20)) 
				{
					index = 0;
				}
				dAssert(index != -1);
				mask[face.m_index[index]] = 0;
			}
		}
		if (!success) 
		{
			ndInt32 count1 = 0;
			ndInt32 vertexCount = convexHull->GetVertexCount();
			for (ndInt32 i = 0; i < vertexCount; i++) 
			{
				if (mask[i]) 
				{
					buffer[count1] = hullVertexArray[i] & ndBigVector::m_triplexMask;
					count1++;
				}
			}
			delete convexHull;
			convexHull = new ndConvexHull3d(&buffer[0].m_x, sizeof(ndBigVector), count1, tolerance);
		}
	}

	dAssert(convexHull);
	ndInt32 vertexCount = convexHull->GetVertexCount();
	if (vertexCount < 4) 
	{
		delete convexHull;
		return false;
	}

	const ndBigVector* const hullVertexArray = convexHull->GetVertexPool();
	ndPolyhedra polyhedra;
	polyhedra.BeginFace();
	for (ndConvexHull3d::ndNode* node = convexHull->GetFirst(); node; node = node->GetNext()) 
	{
		ndConvexHull3dFace& face = node->GetInfo();
		polyhedra.AddFace(face.m_index[0], face.m_index[1], face.m_index[2]);
	}
	polyhedra.EndFace();

	if (vertexCount > 4) 
	{
		while (RemoveCoplanarEdge(polyhedra, hullVertexArray));
	}

	ndStack<ndInt32> vertexMap(vertexCount);
	memset(&vertexMap[0], -1, vertexCount * sizeof(ndInt32));

	ndInt32 mark = polyhedra.IncLRU();
	ndPolyhedra::Iterator iter(polyhedra);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_mark != mark) 
		{
			if (vertexMap[edge->m_incidentVertex] == -1) 
			{
				vertexMap[edge->m_incidentVertex] = m_vertexCount;
				m_vertexCount++;
			}
			ndEdge* ptr = edge;
			do 
			{
				ptr->m_mark = mark;
				ptr->m_userData = m_edgeCount;
				m_edgeCount++;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
		}
	}

	m_vertex = (ndVector*)ndMemory::Malloc(ndInt32(m_vertexCount * sizeof(ndVector)));
	m_simplex = (ndConvexSimplexEdge*)ndMemory::Malloc(ndInt32(m_edgeCount * sizeof(ndConvexSimplexEdge)));
	m_vertexToEdgeMapping = (const ndConvexSimplexEdge**)ndMemory::Malloc(ndInt32(m_vertexCount * sizeof(ndConvexSimplexEdge*)));

	for (ndInt32 i = 0; i < vertexCount; i++) 
	{
		if (vertexMap[i] != -1) 
		{
			m_vertex[vertexMap[i]] = hullVertexArray[i];
			m_vertex[vertexMap[i]].m_w = ndFloat32(0.0f);
		}
	}
	delete convexHull;

	vertexCount = m_vertexCount;
	mark = polyhedra.IncLRU();;
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_mark != mark) 
		{
			ndEdge *ptr = edge;
			do 
			{
				ptr->m_mark = mark;
				ndConvexSimplexEdge* const simplexPtr = &m_simplex[ptr->m_userData];
				simplexPtr->m_vertex = vertexMap[ptr->m_incidentVertex];
				simplexPtr->m_next = &m_simplex[ptr->m_next->m_userData];
				simplexPtr->m_prev = &m_simplex[ptr->m_prev->m_userData];
				simplexPtr->m_twin = &m_simplex[ptr->m_twin->m_userData];

				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
		}
	}

	m_faceCount = 0;
	ndStack<char> faceMarks(m_edgeCount);
	memset(&faceMarks[0], 0, m_edgeCount * sizeof(ndInt8));

	ndStack<ndConvexSimplexEdge*> faceArray(m_edgeCount);

	for (ndInt32 i = 0; i < m_edgeCount; i++) 
	{
		ndConvexSimplexEdge* const face = &m_simplex[i];
		if (!faceMarks[i]) 
		{
			ndConvexSimplexEdge* ptr = face;
			do 
			{
				dAssert((ptr - m_simplex) >= 0);
				faceMarks[ndInt32(ptr - m_simplex)] = '1';
				ptr = ptr->m_next;
			} while (ptr != face);

			faceArray[m_faceCount] = face;
			m_faceCount++;
		}
	}
	m_faceArray = (ndConvexSimplexEdge **)ndMemory::Malloc(ndInt32(m_faceCount * sizeof(ndConvexSimplexEdge *)));
	memcpy(m_faceArray, &faceArray[0], m_faceCount * sizeof(ndConvexSimplexEdge *));

	if (vertexCount > D_CONVEX_VERTEX_SPLITE_SIZE) 
	{
		// create a face structure for support vertex
		ndStack<ndConvexBox> boxTree(vertexCount);
		ndTree<ndVector, ndInt32> sortTree;
		ndStack<ndTree<ndVector, ndInt32>::ndNode*> vertexNodeList(vertexCount);
		
		ndVector boxP0(ndFloat32(1.0e15f));
		ndVector boxP1(-ndFloat32(1.0e15f));
		for (ndInt32 i = 0; i < vertexCount; i++) 
		{
			const ndVector& p = m_vertex[i];
			vertexNodeList[i] = sortTree.Insert(p, i);
			boxP0 = boxP0.GetMin(p);
			boxP1 = boxP1.GetMax(p);
		}
		
		boxTree[0].m_box[0] = boxP0 & ndVector::m_triplexMask;
		boxTree[0].m_box[1] = boxP1 & ndVector::m_triplexMask;
		boxTree[0].m_leftBox = -1;
		boxTree[0].m_rightBox = -1;
		boxTree[0].m_vertexStart = 0;
		boxTree[0].m_vertexCount = vertexCount;
		boxTree[0].m_soaVertexStart = -1;
		boxTree[0].m_soaVertexCount = -1;
		ndInt32 boxCount = 1;
		
		ndInt32 stack = 1;
		ndInt32 stackBoxPool[64];
		stackBoxPool[0] = 0;
		
		while (stack) 
		{
			stack--;
			ndInt32 boxIndex = stackBoxPool[stack];
			ndConvexBox& box = boxTree[boxIndex];
			if (box.m_vertexCount > D_CONVEX_VERTEX_SPLITE_SIZE/2)
			{
				ndVector median(ndVector::m_zero);
				ndVector varian(ndVector::m_zero);
				for (ndInt32 i = 0; i < box.m_vertexCount; i++) 
				{
					ndVector& p = vertexNodeList[box.m_vertexStart + i]->GetInfo();
					boxP0 = boxP0.GetMin(p);
					boxP1 = boxP1.GetMax(p);
					median += p;
					varian += p * p;
				}
		
				ndInt32 index = 0;
				ndFloat64 maxVarian = ndFloat64(-1.0e10f);
				varian = varian.Scale(ndFloat32(box.m_vertexCount)) - median * median;
				for (ndInt32 i = 0; i < 3; i++) 
				{
					if (varian[i] > maxVarian) 
					{
						index = i;
						maxVarian = varian[i];
					}
				}
				ndVector center = median.Scale(ndFloat32(1.0f) / ndFloat32(box.m_vertexCount));
				ndFloat32 test = center[index];
		
				ndInt32 i0 = 0;
				ndInt32 i1 = box.m_vertexCount - 1;
				do 
				{
					for (; i0 <= i1; i0++) 
					{
						ndFloat32 val = vertexNodeList[box.m_vertexStart + i0]->GetInfo()[index];
						if (val > test) 
						{
							break;
						}
					}
		
					for (; i1 >= i0; i1--) 
					{
						ndFloat32 val = vertexNodeList[box.m_vertexStart + i1]->GetInfo()[index];
						if (val < test) 
						{
							break;
						}
					}
		
					if (i0 < i1) 
					{
						dSwap(vertexNodeList[box.m_vertexStart + i0], vertexNodeList[box.m_vertexStart + i1]);
						i0++;
						i1--;
					}
				} while (i0 <= i1);
		
				if (i0 == 0) 
				{
					i0 = box.m_vertexCount / 2;
				}
				if (i0 >= (box.m_vertexCount - 1)) 
				{
					i0 = box.m_vertexCount / 2;
				}
		
				{
					// insert right branch AABB
					ndVector rightBoxP0(ndFloat32(1.0e15f));
					ndVector rightBoxP1(-ndFloat32(1.0e15f));
					for (ndInt32 i = i0; i < box.m_vertexCount; i++) 
					{
						const ndVector& p = vertexNodeList[box.m_vertexStart + i]->GetInfo();
						rightBoxP0 = rightBoxP0.GetMin(p);
						rightBoxP1 = rightBoxP1.GetMax(p);
					}
		
					box.m_rightBox = boxCount;
					boxTree[boxCount].m_box[0] = rightBoxP0 & ndVector::m_triplexMask;
					boxTree[boxCount].m_box[1] = rightBoxP1 & ndVector::m_triplexMask;
					boxTree[boxCount].m_leftBox = -1;
					boxTree[boxCount].m_rightBox = -1;
					boxTree[boxCount].m_vertexStart = box.m_vertexStart + i0;
					boxTree[boxCount].m_vertexCount = box.m_vertexCount - i0;
					boxTree[boxCount].m_soaVertexStart = -1;
					boxTree[boxCount].m_soaVertexCount = -1;
					stackBoxPool[stack] = boxCount;
					stack++;
					boxCount++;
				}
		
				{
					// insert left branch AABB
					ndVector leftBoxP0(ndFloat32(1.0e15f));
					ndVector leftBoxP1(-ndFloat32(1.0e15f));
					for (ndInt32 i = 0; i < i0; i++) 
					{
						const ndVector& p = vertexNodeList[box.m_vertexStart + i]->GetInfo();
						leftBoxP0 = leftBoxP0.GetMin(p);
						leftBoxP1 = leftBoxP1.GetMax(p);
					}
		
					box.m_leftBox = boxCount;
					boxTree[boxCount].m_box[0] = leftBoxP0 & ndVector::m_triplexMask;;
					boxTree[boxCount].m_box[1] = leftBoxP1 & ndVector::m_triplexMask;;
					boxTree[boxCount].m_leftBox = -1;
					boxTree[boxCount].m_rightBox = -1;
					boxTree[boxCount].m_vertexStart = box.m_vertexStart;
					boxTree[boxCount].m_vertexCount = i0;
					boxTree[boxCount].m_soaVertexStart = -1;
					boxTree[boxCount].m_soaVertexCount = -1;

					stackBoxPool[stack] = boxCount;
					stack++;
					boxCount++;
				}
			}
		}
		
		for (ndInt32 i = 0; i < m_vertexCount; i++) 
		{
			m_vertex[i] = vertexNodeList[i]->GetInfo();
			vertexNodeList[i]->GetInfo().m_w = ndFloat32(i);
		}
		
		m_supportTreeCount = boxCount;
		m_supportTree = (ndConvexBox*)ndMemory::Malloc(ndInt32(boxCount * sizeof(ndConvexBox)));
		memcpy(m_supportTree, &boxTree[0], boxCount * sizeof(ndConvexBox));
		
		for (ndInt32 i = 0; i < m_edgeCount; i++) 
		{
			ndConvexSimplexEdge* const ptr = &m_simplex[i];
			ndTree<ndVector, ndInt32>::ndNode* const node = sortTree.Find(ptr->m_vertex);
			ndInt32 index = ndInt32(node->GetInfo().m_w);
			ptr->m_vertex = ndInt16(index);
		}

		m_soaVertexCount = 0;
		for (ndInt32 i = 0; i < boxCount; i ++) 
		{
			ndConvexBox* const box = &m_supportTree[i];
			if (box->m_leftBox == -1)
			{
				dAssert(box->m_rightBox == -1);
				ndInt32 soaCount = ((box->m_vertexCount + 3) & -4) / 4;
				m_soaVertexCount += soaCount;
			}
		}

		m_soa_x = (ndVector*)ndMemory::Malloc(m_soaVertexCount * sizeof(ndVector));
		m_soa_y = (ndVector*)ndMemory::Malloc(m_soaVertexCount * sizeof(ndVector));
		m_soa_z = (ndVector*)ndMemory::Malloc(m_soaVertexCount * sizeof(ndVector));
		m_soa_index = (ndVector*)ndMemory::Malloc(m_soaVertexCount * sizeof(ndVector));

		ndInt32 startAcc = 0;
		ndVector array[D_CONVEX_VERTEX_SPLITE_SIZE];
		for (ndInt32 k = 0; k < boxCount; k++)
		{
			ndConvexBox* const box = &m_supportTree[k];
			if (box->m_leftBox == -1)
			{
				dAssert(box->m_rightBox == -1);
				const ndInt32 soaCount = ((box->m_vertexCount + 3) & -4) / 4;

				ndFloat32* const indexptr = &m_soa_index[startAcc].m_x;
				for (ndInt32 i = 0; i < soaCount * 4; i++)
				{
					array[i] = m_vertex[box->m_vertexStart];
					indexptr[i] = ndFloat32(box->m_vertexStart);
				}

				for (ndInt32 i = 0; i < box->m_vertexCount; i++)
				{
					array[i] = m_vertex[box->m_vertexStart + i];
					indexptr[i] = ndFloat32(box->m_vertexStart + i);
				}

				for (ndInt32 i = 0; i < box->m_vertexCount; i += 4)
				{
					ndVector temp;
					ndInt32 j = startAcc + i / 4;
					ndVector::Transpose4x4(
						m_soa_x[j], m_soa_y[j], m_soa_z[j], temp,
						array[i + 0], array[i + 1], array[i + 2], array[i + 3]);
				}

				box->m_soaVertexStart = startAcc;
				box->m_soaVertexCount = soaCount;
				startAcc += soaCount;
			}

		}
	}
	else 
	{
		m_soaVertexCount = ((m_vertexCount + 3) & -4) / 4;
		m_soaVertexCount = 2 * ((m_soaVertexCount + 1) / 2);
		m_soa_x = (ndVector*)ndMemory::Malloc(m_soaVertexCount * sizeof(ndVector));
		m_soa_y = (ndVector*)ndMemory::Malloc(m_soaVertexCount * sizeof(ndVector));
		m_soa_z = (ndVector*)ndMemory::Malloc(m_soaVertexCount * sizeof(ndVector));
		m_soa_index = (ndVector*)ndMemory::Malloc(m_soaVertexCount * sizeof(ndVector));

		ndVector array[D_CONVEX_VERTEX_SPLITE_SIZE];
		ndFloat32* const indexptr = &m_soa_index[0].m_x;
		for (ndInt32 i = 0; i < m_soaVertexCount * 4; i++)
		{
			array[i] = m_vertex[0];
			indexptr[i] = ndFloat32(0);
		}

		for (ndInt32 i = 0; i < m_vertexCount; i++)
		{
			array[i] = m_vertex[i];
			indexptr[i] = ndFloat32(i);
		}

		for (ndInt32 i = 0; i < m_soaVertexCount; i ++)
		{
			ndVector temp;
			ndInt32 j = i * 4;
			ndVector::Transpose4x4(
				m_soa_x[i], m_soa_y[i], m_soa_z[i], temp,
				array[j + 0], array[j + 1], array[j + 2], array[j + 3]);
		}
	}

	for (ndInt32 i = 0; i < m_edgeCount; i++) 
	{
		ndConvexSimplexEdge* const edge = &m_simplex[i];
		m_vertexToEdgeMapping[edge->m_vertex] = edge;
	}

	SetVolumeAndCG();

	return true;
}

ndBigVector ndShapeConvexHull::FaceNormal(const ndEdge *face, const ndBigVector* const pool) const
{
	const ndEdge* edge = face;
	ndBigVector p0(pool[edge->m_incidentVertex]);
	edge = edge->m_next;

	ndBigVector p1(pool[edge->m_incidentVertex]);
	ndBigVector e1(p1 - p0);

	ndBigVector normal(ndBigVector::m_zero);
	for (edge = edge->m_next; edge != face; edge = edge->m_next) 
	{
		ndBigVector p2(pool[edge->m_incidentVertex]);
		ndBigVector e2(p2 - p0);
		ndBigVector n1(e1.CrossProduct(e2));
#ifdef _DEBUG
		dAssert(n1.m_w == ndFloat32(0.0f));
		ndFloat64 mag = normal.DotProduct(n1).GetScalar();
		dAssert(mag >= -ndFloat32(0.1f));
#endif
		normal += n1;
		e1 = e2;
	}

	ndFloat64 den = sqrt(normal.DotProduct(normal).GetScalar()) + ndFloat64(1.0e-24f);
	normal = normal.Scale(ndFloat64(1.0f) / den);

#ifdef _DEBUG
	edge = face;
	ndBigVector e0(pool[edge->m_incidentVertex] - pool[edge->m_prev->m_incidentVertex]);
	do 
	{
		ndBigVector de1(pool[edge->m_next->m_incidentVertex] - pool[edge->m_incidentVertex]);
		ndBigVector dn1(e0.CrossProduct(de1));
		ndFloat64 x = normal.DotProduct(dn1).GetScalar();
		dAssert(x > -ndFloat64(0.01f));
		e0 = de1;
		edge = edge->m_next;
	} while (edge != face);
#endif
	return normal;
}

bool ndShapeConvexHull::RemoveCoplanarEdge(ndPolyhedra& polyhedra, const ndBigVector* const hullVertexArray) const
{
	bool removeEdge = false;
	// remove coplanar edges
	ndInt32 mark = polyhedra.IncLRU();
	ndPolyhedra::Iterator iter(polyhedra);
	for (iter.Begin(); iter; ) 
	{
		ndEdge* edge0 = &(*iter);
		iter++;

		if (edge0->m_incidentFace != -1) 
		{
			if (edge0->m_mark < mark) 
			{
				edge0->m_mark = mark;
				edge0->m_twin->m_mark = mark;
				ndBigVector normal0(FaceNormal(edge0, &hullVertexArray[0]));
				ndBigVector normal1(FaceNormal(edge0->m_twin, &hullVertexArray[0]));

				ndFloat64 test = normal0.DotProduct(normal1).GetScalar();
				if (test > ndFloat64(0.99995f)) 
				{
					if ((edge0->m_twin->m_next->m_twin->m_next != edge0) && (edge0->m_next->m_twin->m_next != edge0->m_twin)) 
					{
						#define DG_MAX_EDGE_ANGLE ndFloat32 (1.0e-3f)
						if (edge0->m_twin == &(*iter)) 
						{
							if (iter) 
							{
								iter++;
							}
						}

						ndBigVector e1(hullVertexArray[edge0->m_twin->m_next->m_next->m_incidentVertex] - hullVertexArray[edge0->m_incidentVertex]);
						ndBigVector e0(hullVertexArray[edge0->m_incidentVertex] - hullVertexArray[edge0->m_prev->m_incidentVertex]);

						dAssert(e0.m_w == ndFloat64(0.0f));
						dAssert(e1.m_w == ndFloat64(0.0f));
						dAssert(e0.DotProduct(e0).GetScalar() >= ndFloat64(0.0f));
						dAssert(e1.DotProduct(e1).GetScalar() >= ndFloat64(0.0f));

						e0 = e0.Scale(ndFloat64(1.0f) / sqrt(e0.DotProduct(e0).GetScalar()));
						e1 = e1.Scale(ndFloat64(1.0f) / sqrt(e1.DotProduct(e1).GetScalar()));
						ndBigVector n1(e0.CrossProduct(e1));

						ndFloat64 projection = n1.DotProduct(normal0).GetScalar();
						if (projection >= DG_MAX_EDGE_ANGLE) 
						{
							ndBigVector e11(hullVertexArray[edge0->m_next->m_next->m_incidentVertex] - hullVertexArray[edge0->m_twin->m_incidentVertex]);
							ndBigVector e00(hullVertexArray[edge0->m_twin->m_incidentVertex] - hullVertexArray[edge0->m_twin->m_prev->m_incidentVertex]);
							dAssert(e00.m_w == ndFloat64(0.0f));
							dAssert(e11.m_w == ndFloat64(0.0f));
							dAssert(e00.DotProduct(e00).GetScalar() >= ndFloat64(0.0f));
							dAssert(e11.DotProduct(e11).GetScalar() >= ndFloat64(0.0f));
							e00 = e00.Scale(ndFloat64(1.0f) / sqrt(e00.DotProduct(e00).GetScalar()));
							e11 = e11.Scale(ndFloat64(1.0f) / sqrt(e11.DotProduct(e11).GetScalar()));

							ndBigVector n11(e00.CrossProduct(e11));
							projection = n11.DotProduct(normal0).GetScalar();
							if (projection >= DG_MAX_EDGE_ANGLE) 
							{
								dAssert(&(*iter) != edge0);
								dAssert(&(*iter) != edge0->m_twin);
								polyhedra.DeleteEdge(edge0);
								removeEdge = true;
							}
						}
					}
					else 
					{
						ndEdge* next = edge0->m_next;
						ndEdge* prev = edge0->m_prev;
						polyhedra.DeleteEdge(edge0);
						for (edge0 = next; edge0->m_prev->m_twin == edge0; edge0 = next) 
						{
							next = edge0->m_next;
							polyhedra.DeleteEdge(edge0);
						}

						for (edge0 = prev; edge0->m_next->m_twin == edge0; edge0 = prev) 
						{
							prev = edge0->m_prev;
							polyhedra.DeleteEdge(edge0);
						}
						iter.Begin();
						removeEdge = true;
					}
				}
			}
		}
	}

	return removeEdge;
}

inline ndVector ndShapeConvexHull::SupportVertexBruteForce(const ndVector& dir, ndInt32* const vertexIndex) const
{
	const ndVector dirX(dir.m_x);
	const ndVector dirY(dir.m_y);
	const ndVector dirZ(dir.m_z);
	ndVector support(ndVector::m_negOne);
	ndVector maxProj(ndFloat32(-1.0e20f));
	for (ndInt32 i = 0; i < m_soaVertexCount; i += 2)
	{
		ndVector dot(m_soa_x[i] * dirX + m_soa_y[i] * dirY + m_soa_z[i] * dirZ);
		support = support.Select(m_soa_index[i], dot > maxProj);
		maxProj = maxProj.GetMax(dot);

		dot = m_soa_x[i + 1] * dirX + m_soa_y[i + 1] * dirY + m_soa_z[i + 1] * dirZ;
		support = support.Select(m_soa_index[i + 1], dot > maxProj);
		maxProj = maxProj.GetMax(dot);
	}

	ndVector dot(maxProj.ShiftRight().ShiftRight());
	ndVector support1(support.ShiftRight().ShiftRight());
	support = support.Select(support1, dot > maxProj);
	maxProj = maxProj.GetMax(dot);
		
	dot = maxProj.ShiftRight();
	support1 = support.ShiftRight();
	support = support.Select(support1, dot > maxProj);

	const ndInt32 index = ndInt32(support.GetScalar());
	if (vertexIndex)
	{
		*vertexIndex = index;
	}
	dAssert(index != -1);
	return m_vertex[index];
}

inline ndVector ndShapeConvexHull::SupportVertexhierarchical(const ndVector& dir, ndInt32* const vertexIndex) const
{
	const ndInt32 ix = (dir[0] > ndFloat64(0.0f)) ? 1 : 0;
	const ndInt32 iy = (dir[1] > ndFloat64(0.0f)) ? 1 : 0;
	const ndInt32 iz = (dir[2] > ndFloat64(0.0f)) ? 1 : 0;

	const ndConvexBox& leftBox = m_supportTree[m_supportTree[0].m_leftBox];
	const ndConvexBox& rightBox = m_supportTree[m_supportTree[0].m_rightBox];

	const ndVector leftP(leftBox.m_box[ix][0], leftBox.m_box[iy][1], leftBox.m_box[iz][2], ndFloat32(0.0f));
	const ndVector rightP(rightBox.m_box[ix][0], rightBox.m_box[iy][1], rightBox.m_box[iz][2], ndFloat32(0.0f));

	const ndFloat32 leftDist = leftP.DotProduct(dir).GetScalar();
	const ndFloat32 rightDist = rightP.DotProduct(dir).GetScalar();

	ndFloat32 distPool[32];
	const ndConvexBox* stackPool[32];
	if (rightDist >= leftDist)
	{
		distPool[0] = leftDist;
		stackPool[0] = &leftBox;

		distPool[1] = rightDist;
		stackPool[1] = &rightBox;
	}
	else
	{
		distPool[0] = rightDist;
		stackPool[0] = &rightBox;

		distPool[1] = leftDist;
		stackPool[1] = &leftBox;
	}

	const ndVector dirX(dir.m_x);
	const ndVector dirY(dir.m_y);
	const ndVector dirZ(dir.m_z);
	ndVector support(ndVector::m_negOne);
	ndVector maxProj(ndFloat32(-1.0e20f));
	ndVector maxVertexProjection(maxProj);

	ndInt32 stack = 2;
	while (stack)
	{
		stack--;
		const ndFloat32 dist = distPool[stack];
		if (dist > maxProj.GetScalar())
		{
			const ndConvexBox& box = *stackPool[stack];
			if (box.m_leftBox > 0)
			{
				dAssert(box.m_rightBox > 0);
				const ndConvexBox& leftBox1 = m_supportTree[box.m_leftBox];
				const ndConvexBox& rightBox1 = m_supportTree[box.m_rightBox];

				const ndVector leftBoxP(leftBox1.m_box[ix][0], leftBox1.m_box[iy][1], leftBox1.m_box[iz][2], ndFloat32(0.0f));
				const ndVector rightBoxP(rightBox1.m_box[ix][0], rightBox1.m_box[iy][1], rightBox1.m_box[iz][2], ndFloat32(0.0f));

				const ndFloat32 leftBoxDist = leftBoxP.DotProduct(dir).GetScalar();
				const ndFloat32 rightBoxDist = rightBoxP.DotProduct(dir).GetScalar();
				if (rightBoxDist >= leftBoxDist)
				{
					distPool[stack] = leftBoxDist;
					stackPool[stack] = &leftBox1;
					stack++;
					dAssert(stack < ndInt32 (sizeof(distPool) / sizeof(distPool[0])));

					distPool[stack] = rightBoxDist;
					stackPool[stack] = &rightBox1;
					stack++;
					dAssert(stack < ndInt32 (sizeof(distPool) / sizeof(distPool[0])));
				}
				else
				{
					distPool[stack] = rightBoxDist;
					stackPool[stack] = &rightBox1;
					stack++;
					dAssert(stack < ndInt32 (sizeof(distPool) / sizeof(distPool[0])));

					distPool[stack] = leftBoxDist;
					stackPool[stack] = &leftBox1;
					stack++;
					dAssert(stack < ndInt32 (sizeof(distPool) / sizeof(distPool[0])));
				}
			}
			else
			{
				for (ndInt32 j = 0; j < box.m_soaVertexCount; j++)
				{
					ndInt32 i = box.m_soaVertexStart + j;
					ndVector dot(m_soa_x[i] * dirX + m_soa_y[i] * dirY + m_soa_z[i] * dirZ);
					support = support.Select(m_soa_index[i], dot > maxVertexProjection);
					maxVertexProjection = maxVertexProjection.GetMax(dot);
				}
				maxProj = maxProj.GetMax(maxVertexProjection.ShiftRight().ShiftRight());
				maxProj = maxProj.GetMax(maxProj.ShiftRight());
			}
		}
	}

	ndVector support1(support.ShiftRight().ShiftRight());
	ndVector dot(maxVertexProjection.ShiftRight().ShiftRight());
	support = support.Select(support1, dot > maxVertexProjection);
	maxVertexProjection = maxVertexProjection.GetMax(dot);

	support1 = support.ShiftRight();
	dot = maxVertexProjection.ShiftRight();
	support = support.Select(support1, dot > maxVertexProjection);

	const ndInt32 index = ndInt32(support.GetScalar());
	if (vertexIndex)
	{
		*vertexIndex = index;
	}
	dAssert(index != -1);
	return m_vertex[index];
}

ndVector ndShapeConvexHull::SupportVertex(const ndVector& dir, ndInt32* const vertexIndex) const
{
	dAssert(dir.m_w == ndFloat32(0.0f));
	if (m_vertexCount > D_CONVEX_VERTEX_SPLITE_SIZE) 
	{
		return SupportVertexhierarchical(dir, vertexIndex);
	}
	else 
	{
		return SupportVertexBruteForce(dir, vertexIndex);
	}
}

ndShapeInfo ndShapeConvexHull::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeConvex::GetShapeInfo());

	info.m_convexhull.m_vertexCount = m_vertexCount;
	info.m_convexhull.m_strideInBytes = sizeof(ndVector);
	info.m_convexhull.m_faceCount = m_faceCount;
	info.m_convexhull.m_vertex = &m_vertex[0];
	return info;
}

void ndShapeConvexHull::DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const
{
	ndVector vertex[512];
	ndShapeDebugNotify::ndEdgeType edgeType[512];
	memset(edgeType, ndShapeDebugNotify::m_shared, sizeof(edgeType));
	for (ndInt32 i = 0; i < m_faceCount; i++) 
	{
		ndConvexSimplexEdge* const face = m_faceArray[i];
		ndConvexSimplexEdge* ptr = face;
		ndInt32 count = 0;
		do 
		{
			vertex[count] = m_vertex[ptr->m_vertex];
			count++;
			dAssert(count < ndInt32 (sizeof(vertex) / sizeof(vertex[0])));
			ptr = ptr->m_next;
		} while (ptr != face);
		matrix.TransformTriplex(&vertex[0].m_x, sizeof(ndVector), &vertex[0].m_x, sizeof(ndVector), count);
		debugCallback.DrawPolygon(count, vertex, edgeType);
	}
}

void ndShapeConvexHull::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndShapeConvex::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "vextexArray3", m_vertexCount, m_vertex);
}
