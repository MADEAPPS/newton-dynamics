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
#include "ndFileFormatShapeCompound.h"

ndFileFormatShapeCompound::ndFileFormatShapeCompound()
	:ndFileFormatShape(ndShapeCompound::StaticClassName())
{
}

ndFileFormatShapeCompound::ndFileFormatShapeCompound(const char* const className)
	:ndFileFormatShape(className)
{
}

void ndFileFormatShapeCompound::SaveShape(nd::TiXmlElement* const parentNode, const ndShape* const shape)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndShape", ndShapeCompound::StaticClassName());
	ndFileFormatShape::SaveShape(classNode, shape);

	ndShapeCompound* const compoundShape = (ndShapeCompound*)shape;
	const ndShapeCompound::ndTreeArray& shapeList = compoundShape->GetTree();

	ndShapeCompound::ndTreeArray::Iterator it(shapeList);
	for (it.Begin(); it; it++)
	{
		ndShapeInstance* const childInstance = compoundShape->GetShapeInstance(it.GetNode());
		ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(childInstance->ClassName());
		ndAssert(handler);
		handler->SaveCollision(classNode, childInstance);
	}
}
