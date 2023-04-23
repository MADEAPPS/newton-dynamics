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
#include "ndFileFormatSave.h"
#include "ndFileFormatBody.h"
#include "ndFileFormatNotify.h"

ndFileFormatBody::ndFileFormatBody()
	:ndFileFormatRegistrar(ndBody::StaticClassName())
{
}

ndFileFormatBody::ndFileFormatBody(const char* const className)
	:ndFileFormatRegistrar(className)
{
}

void ndFileFormatBody::SaveBody(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndBody* const body)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_BODY_CLASS, ndBody::StaticClassName());

	ndBodyNotify* const notity = body->GetNotifyCallback();
	if (notity)
	{
		ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(notity->ClassName());
		if (handler)
		{
			handler->SaveNotify(scene, classNode, notity);
		}
		else
		{
			ndTrace(("subclass %s not found, instead saving baseclass %s", notity->ClassName(), notity->SuperClassName()));
			ndFileFormatRegistrar* const superHandler = ndFileFormatRegistrar::GetHandler(notity->SuperClassName());
			ndAssert(superHandler);
			superHandler->SaveNotify(scene, classNode, notity);
		}
	}

	ndTree<ndInt32, ndUnsigned64>::ndNode* const node = scene->m_bodiesIds.Insert(body->GetId());
	ndAssert(node);
	if (node)
	{
		ndInt32 id;
		classNode->Attribute("nodeId", &id);
		node->GetInfo() = id;
	}

	xmlSaveParam(classNode, "matrix", body->GetMatrix());
	xmlSaveParam(classNode, "omega", body->GetOmega());
	xmlSaveParam(classNode, "velocity", body->GetVelocity());
	xmlSaveParam(classNode, "centerOfMass", body->GetCentreOfMass());
}

void ndFileFormatBody::LoadBody(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>&, ndBody* const body)
{
	ndMatrix matrix(xmlGetMatrix(node, "matrix"));
	ndVector omega(xmlGetVector3(node, "omega"));
	ndVector veloc(xmlGetVector3(node, "velocity"));
	ndVector com(xmlGetVector3(node, "centerOfMass"));

	body->SetMatrix(matrix);
	body->SetOmega(omega);
	body->SetVelocity(veloc);
	body->SetCentreOfMass(com);

	nd::TiXmlNode* notifyNode = (nd::TiXmlNode*)node->FirstChild(D_NOTIFY_CLASS);
	while (notifyNode && !ndFileFormatRegistrar::GetHandler(((nd::TiXmlElement*)notifyNode)->Attribute("className")))
	{
		notifyNode = (nd::TiXmlNode*)notifyNode->FirstChild(D_NOTIFY_CLASS);
	}
	if (notifyNode)
	{
		const nd::TiXmlElement* const element = (nd::TiXmlElement*)notifyNode;
		ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(element->Attribute("className"));
		ndBodyNotify* const notify = handler->LoadNotify(element);
		body->SetNotifyCallback(notify);
	}
}