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
#include "ndFileFormatShape.h"

ndFileFormatShape::ndFileFormatShape()
	:ndFileFormatRegistrar(ndShape::StaticClassName())
{
}

ndFileFormatShape::ndFileFormatShape(const char* const className)
	:ndFileFormatRegistrar(className)
{
}

void ndFileFormatShape::SaveShape(nd::TiXmlElement* const parentNode, const ndShape* const shape)
{
	nd::TiXmlElement* const classNode = new nd::TiXmlElement(ndShape::StaticClassName());
	parentNode->LinkEndChild(classNode);

	xmlSaveParam(classNode, "inertia", shape->m_inertia);
	xmlSaveParam(classNode, "crossInertia", shape->m_crossInertia);
	xmlSaveParam(classNode, "centerOfMass", shape->m_centerOfMass);
	xmlSaveParam(classNode, "boxSize", shape->m_boxSize);
	xmlSaveParam(classNode, "boxOrigin", shape->m_boxOrigin);
	xmlSaveParam(classNode, "collisionId", ndInt32 (shape->m_collisionId));
}
