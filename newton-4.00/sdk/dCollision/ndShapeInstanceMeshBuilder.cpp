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
#include "ndShape.h"
#include "ndShapeInstance.h"
#include "ndShapeCompound.h"
#include "ndShapeInstanceMeshBuilder.h"

ndShapeInstanceMeshBuilder::ndShapeInstanceMeshBuilder(const ndShapeInstance& instance)
	:ndMeshEffect()
{
	class dgMeshEffectBuilder: public ndShapeDebugNotify
	{
		public:
		dgMeshEffectBuilder()
			:m_vertex(256)
			,m_faceIndexCount(256)
			,m_brush(0)
		{
		}

		virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceVertex, const ndEdgeType* const)
		{
			m_faceIndexCount.PushBack(vertexCount);
			for (ndInt32 i = 0; i < vertexCount; i++) 
			{
				ndBigVector point(faceVertex[i].m_x, faceVertex[i].m_y, faceVertex[i].m_z, ndFloat32(m_brush));
				m_vertex.PushBack(point);
			}
		}
	
		ndArray<ndBigVector> m_vertex;
		ndArray<ndInt32> m_faceIndexCount;
		ndInt32 m_brush;
	};
	dgMeshEffectBuilder builder;

	Init();
	if (((ndShape*)instance.GetShape())->GetAsShapeCompound())
	{
		ndInt32 brush = 0;
		ndMatrix matrix(instance.GetLocalMatrix());
		const ndShapeCompound* const compound = ((ndShape*)instance.GetShape())->GetAsShapeCompound();
		const ndShapeCompound::ndTreeArray& array = compound->GetTree();
		ndShapeCompound::ndTreeArray::Iterator iter(array);
		for (iter.Begin(); iter; iter++) 
		{
			ndShapeInstance* const childInstance = iter.GetNode()->GetInfo()->GetShape();
			builder.m_brush = brush;
			brush++;
			childInstance->DebugShape(matrix, builder);
		}
	}
	else 
	{
		ndMatrix matrix(dGetIdentityMatrix());
		instance.DebugShape(matrix, builder);
	}

	ndStack<ndInt32>indexListBuffer(builder.m_vertex.GetCount());
	ndInt32* const indexList = &indexListBuffer[0];
	dVertexListToIndexList(&builder.m_vertex[0].m_x, sizeof(ndBigVector), 4, builder.m_vertex.GetCount(), &indexList[0], DG_VERTEXLIST_INDEXLIST_TOL);
	
	ndMeshEffect::dMeshVertexFormat vertexFormat;
	
	vertexFormat.m_faceCount = builder.m_faceIndexCount.GetCount();
	vertexFormat.m_faceIndexCount = &builder.m_faceIndexCount[0];
	//vertexFormat.m_faceMaterial = materialIndex;
	
	vertexFormat.m_vertex.m_data = &builder.m_vertex[0].m_x;
	vertexFormat.m_vertex.m_strideInBytes = sizeof(ndBigVector);
	vertexFormat.m_vertex.m_indexList = &indexList[0];
	
	BuildFromIndexList(&vertexFormat);
	RepairTJoints();
	CalculateNormals(ndFloat32(45.0f) * ndDegreeToRad);
}
