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
#include "ndFileFormatShapeInstance.h"

ndFileFormatShapeInstance::ndFileFormatShapeInstance()
	:ndFileFormatRegistrar(ndShapeInstance::StaticClassName())
{
}

ndFileFormatShapeInstance::ndFileFormatShapeInstance(const char* const className)
	:ndFileFormatRegistrar(className)
{
}

void ndFileFormatShapeInstance::SaveCollision(ndFileFormat* const scene, nd::TiXmlElement* const parentNode, const ndShapeInstance* const collision)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndShapeInstanceClass", ndShapeInstance::StaticClassName());

	const ndShape* const shape = collision->GetShape();
	ndUnsigned64 hash = shape->GetHash();
	//ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(shape->ClassName());
	//ndAssert(handler);
	//handler->SaveShape(scene, classNode, shape);
	ndTree<ndInt32, ndUnsigned64>::ndNode* const shapeNode = scene->m_uniqueShapes.Find(hash);
	ndAssert(shapeNode);

	xmlSaveParam(classNode, "shapeNodeIdRef", shapeNode->GetInfo());
	xmlSaveParam(classNode, "scale", collision->m_scale);
	xmlSaveParam(classNode, "skinMargin", collision->m_skinMargin);
	xmlSaveParam(classNode, "localMatrix", collision->m_localMatrix);
	xmlSaveParam(classNode, "alignmentMatrix", collision->m_alignmentMatrix);

	xmlSaveParam(classNode, "extra0", collision->m_shapeMaterial.m_userId);
	xmlSaveParam(classNode, "extra1", ndInt64(collision->m_shapeMaterial.m_data.m_alignPad));
	for (ndInt32 i = 0; i < sizeof(collision->m_shapeMaterial.m_userParam) / sizeof(collision->m_shapeMaterial.m_userParam[0]); ++i)
	{
		char label[64];
		sprintf(label, "extra%d", i + 2);
		xmlSaveParam(classNode, label, ndInt64(collision->m_shapeMaterial.m_userParam[i].m_intData));
	}
}
