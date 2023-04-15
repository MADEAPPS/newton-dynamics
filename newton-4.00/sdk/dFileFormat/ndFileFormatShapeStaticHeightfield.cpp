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
#include "ndFileFormat.h"
#include "ndFileFormatShapeStaticHeightfield.h"

ndFileFormatShapeStaticHeightfield::ndFileFormatShapeStaticHeightfield()
	:ndFileFormatShapeStaticMesh(ndShapeHeightfield::StaticClassName())
{
}

ndFileFormatShapeStaticHeightfield::ndFileFormatShapeStaticHeightfield(const char* const className)
	:ndFileFormatShapeStaticMesh(className)
{
}

ndInt32 ndFileFormatShapeStaticHeightfield::SaveShape(ndFileFormat* const scene, nd::TiXmlElement* const parentNode, const ndShape* const shape)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndShapeClass", ndShapeHeightfield::StaticClassName());
	ndFileFormatShapeStaticMesh::SaveShape(scene, classNode, shape);

	ndAssert(0);
	char fileName[1024];
	sprintf(fileName, "%s_%s_%d.bin", scene->m_assetPath.GetStr(), ndShapeHeightfield::StaticClassName(), xmlGetNodeId(classNode));
	//char* const ptr = strrchr(fileName, '.');
	//if (ptr)
	//{
	//	ndInt32 nodeId = xmlGetNodeId(classNode);
	//	sprintf(ptr, "_%d.bin", nodeId);
	//}

	ndShapeHeightfield* const staticMesh = (ndShapeHeightfield*)shape;
	xmlSaveParam(classNode, "assetName", "string", fileName);
	xmlSaveParam(classNode, "minBox", staticMesh->m_minBox);
	xmlSaveParam(classNode, "maxBox", staticMesh->m_maxBox);
	xmlSaveParam(classNode, "horizontalScale_x", staticMesh->m_horizontalScale_x);
	xmlSaveParam(classNode, "horizontalScale_z", staticMesh->m_horizontalScale_z);
	xmlSaveParam(classNode, "width", staticMesh->m_width);
	xmlSaveParam(classNode, "height", staticMesh->m_height);
	xmlSaveParam(classNode, "diagonalMode", ndInt32(staticMesh->m_diagonalMode));

	FILE* const file = fopen(fileName, "wb");
	if (file)
	{
		fwrite(&staticMesh->m_elevationMap[0], sizeof(ndReal), size_t(staticMesh->m_elevationMap.GetCount()), file);
		fwrite(&staticMesh->m_atributeMap[0], sizeof(ndInt8), size_t(staticMesh->m_atributeMap.GetCount()), file);
		fclose(file);
	}

	return xmlGetNodeId(classNode);
}
