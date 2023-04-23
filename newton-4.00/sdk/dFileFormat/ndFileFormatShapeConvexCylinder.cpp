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
#include "ndFileFormatShapeConvexCylinder.h"

ndFileFormatShapeConvexCylinder::ndFileFormatShapeConvexCylinder()
	:ndFileFormatShapeConvex(ndShapeCylinder::StaticClassName())
{
}

ndFileFormatShapeConvexCylinder::ndFileFormatShapeConvexCylinder(const char* const className)
	:ndFileFormatShapeConvex(className)
{
}

ndInt32 ndFileFormatShapeConvexCylinder::SaveShape(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndShape* const shape)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_SHAPE_CLASS, ndShapeCylinder::StaticClassName());
	ndFileFormatShapeConvex::SaveShape(scene, classNode, shape);

	const ndShapeCylinder* const cylinder = (ndShapeCylinder*)shape;

	xmlSaveParam(classNode, "height", cylinder->m_height);
	xmlSaveParam(classNode, "radius0", cylinder->m_radius0);
	xmlSaveParam(classNode, "radius1", cylinder->m_radius1);
	return xmlGetNodeId(classNode);
}

ndShape* ndFileFormatShapeConvexCylinder::LoadShape(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>&)
{
	ndFloat32 height = xmlGetFloat(node, "height");
	ndFloat32 radius0 = xmlGetFloat(node, "radius0");
	ndFloat32 radius1 = xmlGetFloat(node, "radius1");
	return new ndShapeCylinder(radius0, radius1, height * ndFloat32(2.0f));
}