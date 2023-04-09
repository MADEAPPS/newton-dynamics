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

#ifndef _ND_FILE_FORMAT_H__
#define _ND_FILE_FORMAT_H__

#include "ndFileFormatStdafx.h"

class ndFileFormat : public ndClassAlloc
{
	public: 
	ndFileFormat();
	~ndFileFormat();

	void SaveWorld(const ndWorld* const world, const char* const path);
	void SaveBodies(const ndWorld* const world, const char* const path);
	
	private:
	void CollectScene(const ndWorld* const world);
	void SaveWorld(nd::TiXmlElement* const rootNode);
	void SaveBodies(nd::TiXmlElement* const rootNode);
	void SaveJoints(nd::TiXmlElement* const rootNode);
	void SaveCollisionShapes(nd::TiXmlElement* const rootNode);

	ndString m_fileName;
	const ndWorld* m_world;
	ndArray<ndBody*> m_bodies;
	ndArray<ndJointBilateralConstraint*> m_joints;

	ndTree<ndInt32, ndUnsigned64> m_bodiesIds;
	ndTree<ndInt32, ndUnsigned64> m_jointsIds;
	ndTree<ndInt32, ndUnsigned64> m_uniqueShapesIds;

	friend class ndFileFormatBody;
	friend class ndFileFormatJoint;
	friend class ndFileFormatShapeInstance;
	friend class ndFileFormatShapeCompound;
	friend class ndFileFormatShapeStaticMesh_bvh;
	friend class ndFileFormatShapeStaticHeightfield;
};

#endif 

