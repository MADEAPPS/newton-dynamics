/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndBodyTriggerVolume.h"

ndBodyTriggerVolume::ndBodyTriggerVolume()
	:ndBodyKinematic()
{
	m_contactTestOnly = 1;
}

ndBodyTriggerVolume::ndBodyTriggerVolume(const nd::TiXmlNode* const xmlNode, const dTree<const ndShape*, dUnsigned32>& shapesCache)
	:ndBodyKinematic(xmlNode->FirstChild("ndBodyKinematic"), shapesCache)
{
	// nothing was saved
	m_contactTestOnly = 1;
}

ndBodyTriggerVolume::~ndBodyTriggerVolume()
{
}

void ndBodyTriggerVolume::Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 nodeid, const dTree<dUnsigned32, const ndShape*>& shapesCache) const
{
	nd::TiXmlElement* const paramNode = CreateRootElement(rootNode, "ndBodyTriggerVolume", nodeid);
	ndBodyKinematic::Save(paramNode, assetPath, nodeid, shapesCache);
}
