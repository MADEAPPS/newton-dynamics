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
#include "ndFileFormatShapeConvexHull.h"

ndFileFormatShapeConvexHull::ndFileFormatShapeConvexHull()
	:ndFileFormatShapeConvex(ndShapeConvexHull::StaticClassName())
{
}

ndFileFormatShapeConvexHull::ndFileFormatShapeConvexHull(const char* const className)
	:ndFileFormatShapeConvex(className)
{
}

ndInt32 ndFileFormatShapeConvexHull::SaveShape(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndShape* const shape)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_SHAPE_CLASS, ndShapeConvexHull::StaticClassName());
	ndFileFormatShapeConvex::SaveShape(scene, classNode, shape);

	ndArray<ndVector> points;
	const ndShapeConvexHull* const convexShape = (ndShapeConvexHull*)shape;
	for (ndInt32 i = 0; i < convexShape->m_vertexCount; ++i)
	{
		points.PushBack(convexShape->m_vertex[i]);
	}
	xmlSaveParam(classNode, "points", points);
	return xmlGetNodeId(classNode);
}

ndShape* ndFileFormatShapeConvexHull::LoadShape(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>&)
{
	ndArray<ndVector> points;
	xmlGetFloatArray3(node, "points", points);
	return new ndShapeConvexHull(points.GetCount(), sizeof(ndVector), (0.0f), &points[0].m_x);
}