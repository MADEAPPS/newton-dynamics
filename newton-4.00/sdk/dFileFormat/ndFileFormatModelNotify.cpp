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
#include "ndFileFormatModelNotify.h"

ndFileFormatModelNotify::ndFileFormatModelNotify()
	:ndFileFormatNotify(ndModelNotify::StaticClassName())
{
}

ndFileFormatModelNotify::ndFileFormatModelNotify(const char* const className)
	:ndFileFormatNotify(className)
{
}

void ndFileFormatModelNotify::SaveNotify(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndBodyNotify* const notify)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_NOTIFY_CLASS, ndModelNotify::StaticClassName());
	ndFileFormatNotify::SaveNotify(scene, classNode, notify);
}

ndBodyNotify* ndFileFormatModelNotify::LoadNotify(const nd::TiXmlElement* const node)
{
	ndModelNotify* const notify = new ndModelNotify();
	LoadNotify(node, notify);
	return notify;
}

void ndFileFormatModelNotify::LoadNotify(const nd::TiXmlElement* const node, ndBodyNotify* const notify)
{
	ndFileFormatNotify::LoadNotify((nd::TiXmlElement*)node->FirstChild(D_NOTIFY_CLASS), notify);
}