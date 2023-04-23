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
#include "ndFileFormatShapeInstance.h"

ndFileFormatShapeInstance::ndFileFormatShapeInstance()
	:ndFileFormatRegistrar(ndShapeInstance::StaticClassName())
{
}

ndFileFormatShapeInstance::ndFileFormatShapeInstance(const char* const className)
	:ndFileFormatRegistrar(className)
{
}

void ndFileFormatShapeInstance::SaveCollision(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndShapeInstance* const collision)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_INSTANCE_CLASS, ndShapeInstance::StaticClassName());

	const ndShape* const shape = collision->GetShape();
	ndUnsigned64 hash = shape->GetHash();
	ndTree<ndInt32, ndUnsigned64>::ndNode* const shapeNode = scene->m_uniqueShapesIds.Find(hash);
	ndAssert(shapeNode);

	xmlSaveParam(classNode, "shapeNodeIdRef", shapeNode->GetInfo());
	xmlSaveParam(classNode, "scale", collision->m_scale);
	xmlSaveParam(classNode, "skinMargin", collision->m_skinMargin);
	xmlSaveParam(classNode, "localMatrix", collision->m_localMatrix);
	xmlSaveParam(classNode, "alignmentMatrix", collision->m_alignmentMatrix);
	xmlSaveParam(classNode, "mode", collision->GetCollisionMode() ? 1 : 0);
	xmlSaveParam(classNode, "scaleType", ndInt32 (collision->GetScaleType()));

	ndArray<ndInt64> material;
	material.PushBack(collision->m_shapeMaterial.m_userId);
	material.PushBack(ndInt64(collision->m_shapeMaterial.m_data.m_alignPad));
	for (ndInt32 i = 0; i < sizeof(collision->m_shapeMaterial.m_userParam) / sizeof(collision->m_shapeMaterial.m_userParam[0]); ++i)
	{
		material.PushBack(ndInt64 (collision->m_shapeMaterial.m_userParam[i].m_intData));
	}

	xmlSaveParam(classNode, "material", material);
}

//void ndFileFormatShapeInstance::LoadCollision(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>& shapeMap, ndBodyKinematic* const body)
ndShapeInstance* ndFileFormatShapeInstance::LoadCollision(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>& shapeMap)
{
	ndArray<ndInt64> materialData;
	ndVector scale(xmlGetVector3(node, "scale"));
	ndInt32 shapeRef = xmlGetInt(node, "shapeNodeIdRef");
	ndInt32 mode = xmlGetInt(node, "mode") ? true : false;
	ndShapeInstance::ndScaleType scaleType = ndShapeInstance::ndScaleType (xmlGetInt(node, "scaleType"));
	ndFloat32 skinMargin = xmlGetFloat(node, "skinMargin");
	ndMatrix localMatrix (xmlGetMatrix(node, "localMatrix"));
	ndMatrix aligmentMatrix(xmlGetMatrix(node, "alignmentMatrix"));
	xmlGetInt64(node, "material", materialData);

	ndTree<ndShape*, ndInt32>::ndNode* const shapeNode = shapeMap.Find(shapeRef);
	ndAssert(shapeNode);
	//ndShapeInstance instance(shapeNode->GetInfo());
	ndShapeInstance* const instance = new ndShapeInstance(shapeNode->GetInfo());

	instance->SetScale(scale);
	instance->SetLocalMatrix(localMatrix);
	instance->SetCollisionMode(mode ? true : false);
	instance->m_scaleType = scaleType;
	instance->m_skinMargin = skinMargin;
	instance->m_alignmentMatrix = aligmentMatrix;

	ndShapeMaterial material;
	material.m_userId = materialData[0];
	material.m_data.m_alignPad = ndUnsigned64(materialData[1]);
	for (ndInt32 i = 0; i < sizeof(material.m_userParam) / sizeof(material.m_userParam[0]); ++i)
	{
		material.m_userParam[i].m_intData = ndUnsigned64(materialData[2 + i]);
	}
	instance->SetMaterial(material);

	//body->SetCollisionShape(instance);
	return instance;
}