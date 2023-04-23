/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#include "ndFileFormatStdafx.h"
#include "ndFileFormatSave.h"
#include "ndFileFormatShapeStaticProceduralMesh.h"

ndFileFormatShapeStaticProceduralMesh::ndFileFormatShapeStaticProceduralMesh()
	:ndFileFormatShapeStaticMesh(ndShapeStaticProceduralMesh::StaticClassName())
{
}

ndFileFormatShapeStaticProceduralMesh::ndFileFormatShapeStaticProceduralMesh(const char* const className)
	:ndFileFormatShapeStaticMesh(className)
{
}

ndInt32 ndFileFormatShapeStaticProceduralMesh::SaveShape(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndShape* const shape)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_SHAPE_CLASS, ndShapeStaticProceduralMesh::StaticClassName());
	ndFileFormatShapeStaticMesh::SaveShape(scene, classNode, shape);
	//xmlSaveParam(classNode, "size", shape->GetObbSize());
	return xmlGetNodeId(classNode);
}

ndShape* ndFileFormatShapeStaticProceduralMesh::LoadShape(const nd::TiXmlElement* const, const ndTree<ndShape*, ndInt32>&)
{
	//ndVector size (xmlGetVector3(node, "size") * ndVector::m_two);
	ndShapeStaticProceduralMesh* const staticMesh = new ndShapeStaticProceduralMesh(ndFloat32 (0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	return staticMesh;
}