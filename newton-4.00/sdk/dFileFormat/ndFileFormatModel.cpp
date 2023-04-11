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
#include "ndFileFormatModel.h"

ndFileFormatModel::ndFileFormatModel()
	:ndFileFormatRegistrar(ndModel::StaticClassName())
{
}

ndFileFormatModel::ndFileFormatModel(const char* const className)
	:ndFileFormatRegistrar(className)
{
}

void ndFileFormatModel::SaveModel(ndFileFormat* const scene, nd::TiXmlElement* const parentNode, const ndModel* const model);
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndModelClass", ndModel::StaticClassName());

	ndAssert(0);
	//ndModelNotify* const notity = body->GetNotifyCallback();
	//if (notity)
	//{
	//	ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(notity->ClassName());
	//	ndAssert(handler);
	//	handler->SaveNotify(scene, classNode, notity);
	//}
	//
	//ndTree<ndInt32, ndUnsigned64>::ndNode* const node = scene->m_bodiesIds.Insert(body->GetId());
	//ndAssert(node);
	//if (node)
	//{
	//	ndInt32 id;
	//	classNode->Attribute("nodeId", &id);
	//	node->GetInfo() = id;
	//}
	//
	//xmlSaveParam(classNode, "matrix", body->GetMatrix());
	//xmlSaveParam(classNode, "omega", body->GetOmega());
	//xmlSaveParam(classNode, "velocity", body->GetVelocity());
	//xmlSaveParam(classNode, "centerOfMass", body->GetCentreOfMass());
}
