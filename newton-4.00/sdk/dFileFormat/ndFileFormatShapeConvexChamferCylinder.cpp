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
#include "ndFileFormatShapeConvexChamferCylinder.h"

ndFileFormatShapeConvexChamferCylinder::ndFileFormatShapeConvexChamferCylinder()
	:ndFileFormatShapeConvex(ndShapeChamferCylinder::StaticClassName())
{
}

ndFileFormatShapeConvexChamferCylinder::ndFileFormatShapeConvexChamferCylinder(const char* const className)
	:ndFileFormatShapeConvex(className)
{
}

ndInt32 ndFileFormatShapeConvexChamferCylinder::SaveShape(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndShape* const shape)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_SHAPE_CLASS, ndShapeChamferCylinder::StaticClassName());
	ndFileFormatShapeConvex::SaveShape(scene, classNode, shape);

	const ndShapeChamferCylinder* const cylinder = (ndShapeChamferCylinder*)shape;

	xmlSaveParam(classNode, "height", cylinder->m_height);
	xmlSaveParam(classNode, "radius", cylinder->m_radius);
	return xmlGetNodeId(classNode);
}

ndShape* ndFileFormatShapeConvexChamferCylinder::LoadShape(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>&)
{
	ndFloat32 height = xmlGetFloat(node, "height");
	ndFloat32 radius = xmlGetFloat(node, "radius");
	return new ndShapeChamferCylinder(radius, height * ndFloat32(2.0f));
}