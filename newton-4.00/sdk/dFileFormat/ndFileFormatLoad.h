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

#ifndef _ND_FILE_FORMAT_LOAD_H__
#define _ND_FILE_FORMAT_LOAD_H__

#include "ndFileFormatStdafx.h"
#include "ndFileFormat.h"

class ndFileFormatLoad : public ndFileFormat
{
	public: 
	ndFileFormatLoad();
	~ndFileFormatLoad();

	void Load(const char* const path);
	void AddToWorld(ndWorld* const world);

	const ndList<ndSharedPtr<ndBody>>& GetBodyList() const;
	
	private:
	void LoadShapes(const nd::TiXmlElement* const rootNode, ndTree<ndShape*, ndInt32>& shapeMap);
	void LoadBodies(const nd::TiXmlElement* const rootNode, const ndTree<ndShape*, ndInt32>& shapeMap, ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap);
	void LoadJoints(const nd::TiXmlElement* const rootNode, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>& jointMap);
	void LoadModels(const nd::TiXmlElement* const rootNode, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, const ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>& jointMap);

	ndList<ndSharedPtr<ndBody>> m_bodies;
	ndList<ndSharedPtr<ndModel>> m_models;
	ndList<ndSharedPtr<ndJointBilateralConstraint>> m_joints;

	friend class ndFileFormatBody;
	friend class ndFileFormatJoint;
	friend class ndFileFormatShapeInstance;
	friend class ndFileFormatShapeCompound;
	friend class ndFileFormatShapeStaticMesh_bvh;
	friend class ndFileFormatShapeStaticHeightfield;
	friend class ndFileFormatJointVehicleTorsionBar;
};

#endif 

