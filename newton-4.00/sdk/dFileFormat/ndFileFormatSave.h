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

#ifndef _ND_FILE_FORMAT_SAVE_H__
#define _ND_FILE_FORMAT_SAVE_H__

#include "ndFileFormatStdafx.h"
#include "ndFileFormat.h"

class ndFileFormatSave : public ndFileFormat
{
	public: 
	ndFileFormatSave();
	~ndFileFormatSave();

	void SaveWorld(const ndWorld* const world, const char* const path);
	void SaveBodies(const ndWorld* const world, const char* const path);
	void SaveModels(const ndWorld* const world, const char* const path);

	ndInt32 FindBodyId(const ndBody* const body) const;
	ndInt32 FindJointId(const ndJointBilateralConstraint* const joint) const;
	
	private:
	void BeginSave(const ndWorld* const world, const char* const path);
	void EndSave();

	void CollectScene();
	void SaveWorld(nd::TiXmlElement* const rootNode);
	void SaveBodies(nd::TiXmlElement* const rootNode);
	void SaveJoints(nd::TiXmlElement* const rootNode);
	void SaveModels(nd::TiXmlElement* const rootNode);
	void SaveShapes(nd::TiXmlElement* const rootNode);

	ndWorld* m_world;
	nd::TiXmlDocument* m_doc;

	ndArray<ndBody*> m_bodies;
	ndArray<ndModel*> m_models;
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
	friend class ndFileFormatJointVehicleTorsionBar;
};

#endif 

