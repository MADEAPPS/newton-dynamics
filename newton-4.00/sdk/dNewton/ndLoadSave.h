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

#ifndef __ND_ND_LOAD_SAVE_H__
#define __ND_ND_LOAD_SAVE_H__

#include "ndNewtonStdafx.h"

class ndWorld;
class ndLoadSaveInfo;

class ndWordSettings : public ndClassAlloc
{
	public:
	D_CLASS_REFLECTION(ndWordSettings);

	ndWordSettings()
		:ndClassAlloc()
		,m_subSteps(2)
		,m_solverIterations(4)
	{
	}

	virtual ~ndWordSettings()
	{
	}

	D_NEWTON_API ndWordSettings(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API virtual void Load(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_NEWTON_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	dInt32 m_subSteps;
	dInt32 m_solverIterations;
};

D_MSV_NEWTON_ALIGN_32
class ndLoadSave: public ndClassAlloc
{
	public:
	ndLoadSave();
	~ndLoadSave();

	D_NEWTON_API bool LoadScene(const char* const path);
	D_NEWTON_API void SaveModel(const char* const path, const ndModel* const model);
	D_NEWTON_API void SaveScene(const char* const path, const ndWorld* const world, const ndWordSettings* const setting);

	private:
	void SaveSceneSettings(ndLoadSaveInfo& info) const;
	void SaveShapes(ndLoadSaveInfo& info);
	void SaveBodies(ndLoadSaveInfo& info);
	void SaveJoints(ndLoadSaveInfo& info);
	void SaveModels(ndLoadSaveInfo& info);
	
	void LoadSceneSettings(const nd::TiXmlNode* const rootNode, const char* const assetPath);
	void LoadShapes(const nd::TiXmlNode* const rootNode, const char* const assetPath, ndShapeLoaderCache& shapesMap);
	void LoadBodies(const nd::TiXmlNode* const rootNode, const char* const assetPath, const ndShapeLoaderCache& shapesMap);
	void LoadJoints(const nd::TiXmlNode* const rootNode, const char* const assetPath);
	void LoadModels(const nd::TiXmlNode* const rootNode, const char* const assetPath);

	public:
	ndWordSettings* m_setting;
	ndBodyLoaderCache m_bodyMap;
	ndJointLoaderCache m_jointMap;
	ndModelLoaderCache m_modelMap;
} D_GCC_NEWTON_ALIGN_32;


inline ndLoadSave::ndLoadSave()
	:ndClassAlloc()	
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
