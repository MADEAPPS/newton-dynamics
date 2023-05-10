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
#include "ndFileFormatShapeStaticHeightfield.h"

ndFileFormatShapeStaticHeightfield::ndFileFormatShapeStaticHeightfield()
	:ndFileFormatShapeStaticMesh(ndShapeHeightfield::StaticClassName())
{
}

ndFileFormatShapeStaticHeightfield::ndFileFormatShapeStaticHeightfield(const char* const className)
	:ndFileFormatShapeStaticMesh(className)
{
}

ndInt32 ndFileFormatShapeStaticHeightfield::SaveShape(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndShape* const shape)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_SHAPE_CLASS, ndShapeHeightfield::StaticClassName());
	ndFileFormatShapeStaticMesh::SaveShape(scene, classNode, shape);

	char fileName[1024];
	sprintf(fileName, "%s_%s_%d.bin", scene->m_assetPath.GetStr(), ndShapeHeightfield::StaticClassName(), xmlGetNodeId(classNode));

	ndShapeHeightfield* const staticMesh = (ndShapeHeightfield*)shape;
	xmlSaveParam(classNode, "assetName", fileName);
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

ndShape* ndFileFormatShapeStaticHeightfield::LoadShape(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>&)
{
	const char* const filename = xmlGetString(node, "assetName");
	ndVector minBox (xmlGetVector3(node, "minBox"));
	ndVector maxBox(xmlGetVector3(node, "maxBox"));
	ndInt32 width = xmlGetInt(node, "width");
	ndInt32 height = xmlGetInt(node, "height");
	ndInt32 diagonalMode = xmlGetInt(node, "diagonalMode");
	ndFloat32 horizontalScale_x = xmlGetFloat(node, "horizontalScale_x");
	ndFloat32 horizontalScale_z = xmlGetFloat(node, "horizontalScale_z");

	ndShapeHeightfield* const staticMesh = new ndShapeHeightfield(width, height, ndShapeHeightfield::ndGridConstruction (diagonalMode), horizontalScale_x, horizontalScale_z);

	FILE* const file = fopen(filename, "rb");
	if (file)
	{
		size_t ret;
		ndAssert(staticMesh->m_atributeMap.GetCount() == width * height);
		ndAssert(staticMesh->m_elevationMap.GetCount() == width * height);

		ret = fread(&staticMesh->m_elevationMap[0], sizeof(ndReal), size_t(staticMesh->m_elevationMap.GetCount()), file);
		ret = fread(&staticMesh->m_atributeMap[0], sizeof(ndInt8), size_t(staticMesh->m_atributeMap.GetCount()), file);
		fclose(file);
	}
	return staticMesh;
}