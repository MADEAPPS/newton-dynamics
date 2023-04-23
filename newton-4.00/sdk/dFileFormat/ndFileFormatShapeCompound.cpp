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
#include "ndFileFormatShapeCompound.h"

ndFileFormatShapeCompound::ndFileFormatShapeCompound()
	:ndFileFormatShape(ndShapeCompound::StaticClassName())
{
}

ndFileFormatShapeCompound::ndFileFormatShapeCompound(const char* const className)
	:ndFileFormatShape(className)
{
}

ndInt32 ndFileFormatShapeCompound::SaveShape(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndShape* const shape)
{
	ndShapeCompound* const compoundShape = (ndShapeCompound*)shape;
	const ndShapeCompound::ndTreeArray& shapeList = compoundShape->GetTree();

	ndShapeCompound::ndTreeArray::Iterator it(shapeList);
	for (it.Begin(); it; it++)
	{
		const ndShapeInstance* const childInstance = compoundShape->GetShapeInstance(it.GetNode());
		const ndShape* const childShape = childInstance->GetShape();
		ndUnsigned64 hash = childShape->GetHash();
		ndTree<ndInt32, ndUnsigned64>::ndNode* const node = scene->m_uniqueShapesIds.Insert(hash);
		if (node)
		{
			ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(childShape->ClassName());
			ndInt32 id = handler->SaveShape(scene, parentNode, childShape);
			node->GetInfo() = id;
		}
	}

	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_SHAPE_CLASS, ndShapeCompound::StaticClassName());
	ndFileFormatShape::SaveShape(scene, classNode, shape);
	for (it.Begin(); it; it++)
	{
		const ndShapeInstance* const childInstance = compoundShape->GetShapeInstance(it.GetNode());
		ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(childInstance->ClassName());
		ndAssert(handler);
		handler->SaveCollision(scene, classNode, childInstance);
	}
	return xmlGetNodeId(classNode);
}


ndShape* ndFileFormatShapeCompound::LoadShape(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>& shapeMap)
{
	ndShapeInstance rootInstance(new ndShapeCompound());
	ndShapeCompound* const compoundShape = (ndShapeCompound*)rootInstance.GetShape();

	compoundShape->BeginAddRemove();

	ndFileFormatRegistrar* const collisionHandler = ndFileFormatRegistrar::GetHandler(ndShapeInstance::StaticClassName());
	ndAssert(collisionHandler);
	for (const nd::TiXmlNode* childNode = node->FirstChild(D_INSTANCE_CLASS); childNode; childNode = childNode->NextSibling())
	{
		ndSharedPtr<ndShapeInstance> instance(collisionHandler->LoadCollision((nd::TiXmlElement*)childNode, shapeMap));
		compoundShape->AddCollision(*instance);
	}
	compoundShape->EndAddRemove();
	return new ndShapeCompound(*compoundShape, nullptr);
}