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
#include "ndFileFormatNotify.h"

ndFileFormatNotify::ndFileFormatNotify()
	:ndFileFormatRegistrar(ndBodyNotify::StaticClassName())
{
}

ndFileFormatNotify::ndFileFormatNotify(const char* const className)
	:ndFileFormatRegistrar(className)
{
}

void ndFileFormatNotify::SaveNotify(ndFileFormatSave* const, nd::TiXmlElement* const parentNode, const ndBodyNotify* const notify)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_NOTIFY_CLASS, ndBodyNotify::StaticClassName());
	xmlSaveParam(classNode, "gravity", notify->GetGravity());
}

void ndFileFormatNotify::LoadNotify(const nd::TiXmlElement* const node, ndBodyNotify* const notify)
{
	ndVector gravity(xmlGetVector3(node, "gravity"));
	notify->SetGravity(gravity);
}

ndBodyNotify* ndFileFormatNotify::LoadNotify(const nd::TiXmlElement* const node)
{
	ndAssert(0);
	//ndBodyNotify* const notify = new ndBodyNotify(ndVector::m_zero);
	//LoadNotify(node, notify);
	//return notify;
	return nullptr;
}