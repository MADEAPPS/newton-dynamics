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
#include "ndCollisionStdafx.h"
#include "ndShape.h"
#include "ndShapeInstance.h"
#include "ndShapeInstanceMeshBuilder.h"

ndShapeInstanceMeshBuilder::ndShapeInstanceMeshBuilder(const ndShapeInstance& instance)
	:ndMeshEffect()
{
	class dgMeshEffectBuilder: public ndShapeDebugCallback
	{
		public:
		dgMeshEffectBuilder()
			:m_vertex(256)
			,m_faceIndexCount(256)
			,m_brush(0)
		{
		}

		virtual void DrawPolygon(dInt32 vertexCount, const dVector* const faceVertex, const ndEdgeType* const)
		{
			m_faceIndexCount.PushBack(vertexCount);
			for (dInt32 i = 0; i < vertexCount; i++) 
			{
				dBigVector point(faceVertex[i].m_x, faceVertex[i].m_y, faceVertex[i].m_z, dFloat32(m_brush));
				m_vertex.PushBack(point);
			}
		}
	
		dArray<dBigVector> m_vertex;
		dArray<dInt32> m_faceIndexCount;
		dInt32 m_brush;
	};
	dgMeshEffectBuilder builder;

	Init();
	if (((ndShape*)instance.GetShape())->GetAsShapeCompoundConvex())
	{
		dAssert(0);
		//dgCollisionInfo collisionInfo;
		//collision->GetCollisionInfo(&collisionInfo);
		//
		//dInt32 brush = 0;
		//dMatrix matrix(collisionInfo.m_offsetMatrix);
		//dgCollisionCompound* const compoundCollision = (dgCollisionCompound*)collision->GetChildShape();
		//for (dTree<dgCollisionCompound::dgNodeBase*, dInt32>::dTreeNode* node = compoundCollision->GetFirstNode(); node; node = compoundCollision->GetNextNode(node)) {
		//	builder.m_brush = brush;
		//	brush++;
		//	dgCollisionInstance* const childShape = compoundCollision->GetCollisionFromNode(node);
		//	childShape->DebugCollision(matrix, (dgCollision::OnDebugCollisionMeshCallback) dgMeshEffectBuilder::GetShapeFromCollision, &builder);
		//}
	}
	else 
	{
		dMatrix matrix(dGetIdentityMatrix());
		instance.DebugShape(matrix, builder);
	}

	dStack<dInt32>indexListBuffer(builder.m_vertex.GetCount());
	dInt32* const indexList = &indexListBuffer[0];
	dVertexListToIndexList(&builder.m_vertex[0].m_x, sizeof(dBigVector), 4, builder.m_vertex.GetCount(), &indexList[0], DG_VERTEXLIST_INDEXLIST_TOL);
	
	ndMeshEffect::dMeshVertexFormat vertexFormat;
	
	vertexFormat.m_faceCount = builder.m_faceIndexCount.GetCount();
	vertexFormat.m_faceIndexCount = &builder.m_faceIndexCount[0];
	//vertexFormat.m_faceMaterial = materialIndex;
	
	vertexFormat.m_vertex.m_data = &builder.m_vertex[0].m_x;
	vertexFormat.m_vertex.m_strideInBytes = sizeof(dBigVector);
	vertexFormat.m_vertex.m_indexList = &indexList[0];
	
	BuildFromIndexList(&vertexFormat);
	RepairTJoints();
	CalculateNormals(dFloat32(45.0f) * dDegreeToRad);
}
