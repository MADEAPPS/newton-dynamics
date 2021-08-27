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

#ifndef __D_ND_LOAD_SAVE_H__
#define __D_ND_LOAD_SAVE_H__

#include "ndNewtonStdafx.h"

class ndWorld;
class ndWordSettings : public dClassAlloc
{
	public:
	D_CLASS_REFLECTION(ndWordSettings);

	ndWordSettings()
		:dClassAlloc()
		,m_subSteps(2)
		,m_solverIterations(4)
	{
	}

	virtual ~ndWordSettings()
	{
	}

	D_NEWTON_API ndWordSettings(const dLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API virtual void Load(const dLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API virtual void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;

	dInt32 m_subSteps;
	dInt32 m_solverIterations;
};

D_MSV_NEWTON_ALIGN_32
class ndLoadSave: public dClassAlloc
{
	public:
	ndLoadSave();
	~ndLoadSave();

	D_NEWTON_API bool LoadScene(const char* const path);
	D_NEWTON_API void SaveScene(const char* const path, const ndWorld* const world, const ndWordSettings* const setting);

	private:
	void SaveSceneSettings(nd::TiXmlNode* const rootNode, const char* const assetPath, const char* const assetName, const ndWordSettings* const setting) const;
	void SaveCollisionShapes(nd::TiXmlNode* const rootNode, const ndWorld* const world,
		const char* const assetPath, const char* const assetName,
		dTree<dUnsigned32, const ndShape*>& shapesMap);
	void SaveRigidBodies(nd::TiXmlNode* const rootNode, const ndWorld* const world,
		const char* const assetPath, const char* const assetName,
		const dTree<dUnsigned32, const ndShape*>& shapesMap,
		dTree<dUnsigned32, const ndBodyKinematic*>& bodyMap);
	void SaveJoints(nd::TiXmlNode* const rootNode, const ndWorld* const world,
		const char* const assetPath, const char* const assetName,
		const dTree<dUnsigned32, const ndBodyKinematic*>& bodyMap,
		dTree<dUnsigned32, const ndJointBilateralConstraint*>& jointMap);
	void SaveModels(nd::TiXmlNode* const rootNode, const ndWorld* const world,
		const char* const assetPath, const char* const assetName,
		const dTree<dUnsigned32, const ndBodyKinematic*>& bodyMap,
		const dTree<dUnsigned32, const ndJointBilateralConstraint*>& jointMap,
		dTree<dUnsigned32, const ndModel*>& modelMap);
	
	void LoadSceneSettings(const nd::TiXmlNode* const rootNode, const char* const assetPath);
	void LoadCollisionShapes(const nd::TiXmlNode* const rootNode, const char* const assetPath, ndShapeLoaderCache& shapesMap);
	void LoadRigidBodies(const nd::TiXmlNode* const rootNode, const char* const assetPath, const ndShapeLoaderCache& shapesMap);
	void LoadJoints(const nd::TiXmlNode* const rootNode, const char* const assetPath);
	void LoadModels(const nd::TiXmlNode* const rootNode, const char* const assetPath);

	public:
	ndWordSettings* m_setting;
	ndBodyLoaderCache m_bodyMap;
	ndJointLoaderCache m_jointMap;
	ndModelLoaderCache m_modelMap;
} D_GCC_NEWTON_ALIGN_32;


inline ndLoadSave::ndLoadSave()
	:dClassAlloc()	
	,m_setting(nullptr)
	,m_bodyMap()
	,m_jointMap()
	,m_modelMap()
{
}

inline ndLoadSave::~ndLoadSave()
{
	if (m_setting)
	{
		delete m_setting;
	}
}


#endif
