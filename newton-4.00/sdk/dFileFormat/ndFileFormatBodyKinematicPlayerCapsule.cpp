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
#include "ndFileFormatBodyKinematicPlayerCapsule.h"

ndFileFormatBodyKinematicPlayerCapsule::ndFileFormatBodyKinematicPlayerCapsule()
	:ndFileFormatBodyKinematicBase(ndBodyPlayerCapsule::StaticClassName())
{
}

ndFileFormatBodyKinematicPlayerCapsule::ndFileFormatBodyKinematicPlayerCapsule(const char* const className)
	:ndFileFormatBodyKinematicBase(className)
{
}

void ndFileFormatBodyKinematicPlayerCapsule::SaveBody(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndBody* const body)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_BODY_CLASS, ndBodyPlayerCapsule::StaticClassName());
	ndFileFormatBodyKinematicBase::SaveBody(scene, classNode, body);

	const ndBodyPlayerCapsule* const exportBody = ((ndBody*)body)->GetAsBodyPlayerCapsule();

	xmlSaveParam(classNode, "localFrame", exportBody->m_localFrame);
	xmlSaveParam(classNode, "mass", exportBody->m_mass);
	xmlSaveParam(classNode, "height", exportBody->m_height);
	xmlSaveParam(classNode, "radius", exportBody->m_radius);
	xmlSaveParam(classNode, "stepHeight", exportBody->m_stepHeight);

	//xmlSaveParam(classNode, "weistScale", exportBody->m_weistScale);
	//xmlSaveParam(classNode, "crouchScale", exportBody->m_crouchScale);
	//xmlSaveParam(classNode, "", exportBody->m_impulse);
	//xmlSaveParam(classNode, "", exportBody->m_invMass);
	//xmlSaveParam(classNode, "", exportBody->m_headingAngle);
	//xmlSaveParam(classNode, "", exportBody->m_forwardSpeed);
	//xmlSaveParam(classNode, "", exportBody->m_lateralSpeed);
	//xmlSaveParam(classNode, "", exportBody->m_contactPatch);
	//xmlSaveParam(classNode, "", exportBody->m_isAirbone);
	//xmlSaveParam(classNode, "", exportBody->m_isOnFloor);
	//xmlSaveParam(classNode, "", exportBody->m_isCrouched);
}

ndBody* ndFileFormatBodyKinematicPlayerCapsule::LoadBody(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>& shapeMap)
{
	ndBodyPlayerCapsule* const body = new ndBodyPlayerCapsule();
	LoadBody(node, shapeMap, body);
	return body;
}

void ndFileFormatBodyKinematicPlayerCapsule::LoadBody(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>& shapeMap, ndBody* const body)
{
	ndFileFormatBodyKinematicBase::LoadBody((nd::TiXmlElement*)node->FirstChild(D_BODY_CLASS), shapeMap, body);

	ndMatrix localFrame (xmlGetMatrix (node, "localFrame"));
	ndFloat32 mass = xmlGetFloat(node, "mass");
	ndFloat32 height = xmlGetFloat(node, "height");
	ndFloat32 radius = xmlGetFloat(node, "radius");
	ndFloat32 stepHeight = xmlGetFloat(node, "stepHeight");
	//ndFloat32 weistScale = xmlGetFloat(node, "weistScale");
	//ndFloat32 crouchScale = xmlGetFloat(node, "crouchScale");

	ndBodyPlayerCapsule* const kinBody = ((ndBody*)body)->GetAsBodyPlayerCapsule();
	kinBody->Init(localFrame, mass, radius, height, stepHeight);
}