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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndModel.h"
#include "ndLoadSave.h"
#include "ndBodyDynamic.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndWordSettings);

class ndLoadSaveInfo
{
	public:
	void ExtensionAndFilePath(const char* const path)
	{
		char fileNameExt[1024];
		strcpy(fileNameExt, path);
		// find last slash
		char* lastSlach = strrchr(fileNameExt, '/');
		if (!lastSlach)
		{
			lastSlach = strrchr(fileNameExt, '\\');
		}
		if (!lastSlach)
		{
			lastSlach = fileNameExt;
		}

		char* const ext = strrchr(lastSlach, '.');
		if (ext)
		{
			*ext = 0;
		}

		strncpy(m_assetPath, fileNameExt, sizeof(m_assetPath) - 10);
		strcat(fileNameExt, ".nd");
		
		char* namePtr = strrchr(m_assetPath, '/');
		if (!namePtr)
		{
			namePtr = strrchr(m_assetPath, '\\');
		}
		if (namePtr)
		{
			strncpy(m_assetName, namePtr + 1, sizeof(m_assetName) - 16);
		}
		else
		{
			namePtr = m_assetPath;
			strncpy(m_assetName, namePtr, sizeof(m_assetName) - 16);
		}
		namePtr[0] = 0;
		strcat(m_assetName, "_asset");
		strncpy(m_fileName, fileNameExt, sizeof(m_fileName) - 16);
	}

	char m_fileName[512];
	char m_assetPath[512];
	char m_assetName[128];

	nd::TiXmlElement* m_worldNode;
	nd::TiXmlElement* m_settingsNode;
	nd::TiXmlElement* m_shapesNode;
	nd::TiXmlElement* m_bodiesNode;
	nd::TiXmlElement* m_jointsNode;
	nd::TiXmlElement* m_modelsNode;

	const ndBodyList* m_bodyList;
	const ndJointList* m_jointList;
	const ndModelList* m_modelList;
	const ndWordSettings* m_setting;

	ndTree<dInt32, const ndShape*> m_shapeMap;
	ndTree<dInt32, const ndModel*> m_modelMap;
	ndTree<dInt32, const ndBodyKinematic*> m_bodyMap;
	ndTree<dInt32, const ndJointBilateralConstraint*> m_jointMap;
};

ndWordSettings::ndWordSettings(const ndLoadSaveBase::dLoadDescriptor&)
	:ndClassAlloc()
	,m_subSteps(2)
	,m_solverIterations(4)
{
}

void ndWordSettings::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);

	xmlSaveParam(childNode, "description", "string", "Newton Dynamics 4.00");
	xmlSaveParam(childNode, "revision", "string", "1.00");
	xmlSaveParam(childNode, "solverSubsteps", m_subSteps);
	xmlSaveParam(childNode, "solverIterations", m_solverIterations);
}

void ndWordSettings::Load(const ndLoadSaveBase::dLoadDescriptor& desc)
{
	m_subSteps = xmlGetInt(desc.m_rootNode, "solverSubsteps");
	m_solverIterations = xmlGetInt(desc.m_rootNode, "solverIterations");
}

void ndLoadSave::LoadSceneSettings(const nd::TiXmlNode* const rootNode, const char* const assetPath)
{
	const nd::TiXmlNode* const setting = rootNode->FirstChild("ndSettings");
	dAssert(setting);

	const nd::TiXmlNode* node = setting->FirstChild();
	const char* const className = node->Value();

	ndLoadSaveBase::dLoadDescriptor settingDesc;
	settingDesc.m_rootNode = node;
	settingDesc.m_assetPath = assetPath;

	m_setting = D_CLASS_REFLECTION_LOAD_NODE(ndWordSettings, className, settingDesc);
	m_setting->Load(settingDesc);
}

void ndLoadSave::LoadShapes(const nd::TiXmlNode* const rootNode, 
	const char* const assetPath, ndShapeLoaderCache& shapesMap)
{
	const nd::TiXmlNode* const shapes = rootNode->FirstChild("ndShapes");
	if (shapes)
	{
		ndLoadSaveBase::dLoadDescriptor descriptor;
		descriptor.m_assetPath = assetPath;

		class ndPendingCompounds
		{
			public:
			ndShapeInstance* m_compoundInstance;
			const nd::TiXmlNode* m_subShapeNodes;
		};

		ndArray<ndPendingCompounds> pendingCompounds;
		for (const nd::TiXmlNode* node = shapes->FirstChild(); node; node = node->NextSibling())
		{
			const char* const name = node->Value();
			descriptor.m_rootNode = node;
			ndShape* const shape = D_CLASS_REFLECTION_LOAD_NODE(ndShape, name, descriptor);
			dAssert(shape);
			if (shape)
			{
				dInt32 hashId;
				const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
				element->Attribute("hashId", &hashId);
				ndShapeLoaderCache::ndNode* const shapeMapNode = shapesMap.Insert(ndShapeInstance(shape), hashId);
				ndShapeCompound* const compound = ((ndShape*)shapeMapNode->GetInfo().GetShape())->GetAsShapeCompound();
				if (compound)
				{
					ndPendingCompounds pending;
					pending.m_compoundInstance = (ndShapeInstance*)&shapeMapNode->GetInfo();
					pending.m_subShapeNodes = element->FirstChild()->NextSibling();
					pendingCompounds.PushBack(pending);
				}
			}
		}

		for (dInt32 i = 0; i < pendingCompounds.GetCount(); i++)
		{
			ndShapeInstance* const instance = pendingCompounds[i].m_compoundInstance;
			ndShapeCompound* const compound = instance->GetShape()->GetAsShapeCompound();
			compound->BeginAddRemove();
			for (const nd::TiXmlNode* node = pendingCompounds[i].m_subShapeNodes->FirstChild("ndShapeInstance"); node; node = node->NextSibling())
			{
				ndShapeInstance subShapeInstance(node, shapesMap);
				compound->AddCollision(&subShapeInstance);
			}
			compound->EndAddRemove();
		}
	}
}

void ndLoadSave::LoadBodies(const nd::TiXmlNode* const rootNode, 
	const char* const assetPath, const ndShapeLoaderCache& shapesMap)
{
	const nd::TiXmlNode* const bodies = rootNode->FirstChild("ndBodies");
	if (bodies)
	{
		ndLoadSaveBase::dLoadDescriptor descriptor;
		descriptor.m_assetPath = assetPath;
		descriptor.m_shapeMap = &shapesMap;

		for (const nd::TiXmlNode* node = bodies->FirstChild(); node; node = node->NextSibling())
		{
			const char* const name = node->Value();
			descriptor.m_rootNode = node;
			ndBody* const body = D_CLASS_REFLECTION_LOAD_NODE(ndBody, name, descriptor);
			if (body)
			{
				dInt32 hashId;
				const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
				element->Attribute("hashId", &hashId);
				m_bodyMap.Insert(body, hashId);
			}
		}
	}
}

void ndLoadSave::LoadJoints(const nd::TiXmlNode* const rootNode, const char* const assetPath)
{
	const nd::TiXmlNode* const joints = rootNode->FirstChild("ndJoints");
	if (joints)
	{
		ndLoadSaveBase::dLoadDescriptor descriptor;
		descriptor.m_assetPath = assetPath;
		descriptor.m_bodyMap = &m_bodyMap;

		for (const nd::TiXmlNode* node = joints->FirstChild(); node; node = node->NextSibling())
		{
			const char* const name = node->Value();
			descriptor.m_rootNode = node;
			ndJointBilateralConstraint* const joint = D_CLASS_REFLECTION_LOAD_NODE(ndJointBilateralConstraint, name, descriptor);
			if (joint)
			{
				dInt32 hashId;
				const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
				element->Attribute("hashId", &hashId);
				m_jointMap.Insert(joint, hashId);
				if (joint->GetBody1()->GetAsBodySentinel())
				{
					joint->ReplaceSentinel(nullptr);
				}
			}
		}
	}
}

void ndLoadSave::LoadModels(const nd::TiXmlNode* const rootNode, const char* const assetPath)
{
	const nd::TiXmlNode* const models = rootNode->FirstChild("ndModels");
	if (models)
	{
		ndLoadSaveBase::dLoadDescriptor descriptor;
		descriptor.m_assetPath = assetPath;
		descriptor.m_bodyMap = &m_bodyMap;
		descriptor.m_jointMap = &m_jointMap;

		for (const nd::TiXmlNode* node = models->FirstChild(); node; node = node->NextSibling())
		{
			const char* const name = node->Value();
			descriptor.m_rootNode = node;
			ndModel* const model = D_CLASS_REFLECTION_LOAD_NODE(ndModel, name, descriptor);
			if (model)
			{
				dInt32 hashId;
				const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
				element->Attribute("hashId", &hashId);
				m_modelMap.Insert(model, hashId);
			}
		}
	}
}

void ndLoadSave::SaveSceneSettings(ndLoadSaveInfo& info) const
{
	ndLoadSaveBase::ndSaveDescriptor descriptor;
	descriptor.m_assetPath = info.m_assetPath;
	descriptor.m_assetName = info.m_assetName;
	descriptor.m_rootNode = info.m_settingsNode;
	info.m_setting->Save(descriptor);
}

void ndLoadSave::SaveShapes(ndLoadSaveInfo& info)
{
	ndTree<const ndShape*, dInt32> shapeList;
	ndTree<dInt32, const ndShape*>::Iterator iter (info.m_shapeMap);
	for (iter.Begin(); iter; iter++)
	{
		dInt32 hash = iter.GetNode()->GetInfo();
		const ndShape* const shape = iter.GetKey();
		shapeList.Insert(shape, hash);
	}

	ndLoadSaveBase::ndSaveDescriptor descriptor;
	descriptor.m_assetPath = info.m_assetPath;
	descriptor.m_assetName = info.m_assetName;
	descriptor.m_rootNode = info.m_shapesNode;
	descriptor.m_shapeMap = &info.m_shapeMap;

	ndTree<const ndShape*, dInt32>::Iterator shapeIter(shapeList);
	for (shapeIter.Begin(); shapeIter; shapeIter++)
	{
		descriptor.m_nodeNodeHash = shapeIter.GetKey();
		const ndShape* const shape = shapeIter.GetNode()->GetInfo();
		shape->Save(descriptor);
	}
}

void ndLoadSave::SaveBodies(ndLoadSaveInfo& info)
{
	ndLoadSaveBase::ndSaveDescriptor descriptor;
	descriptor.m_assetPath = info.m_assetPath;
	descriptor.m_assetName = info.m_assetName;
	descriptor.m_rootNode = info.m_bodiesNode;

	for (ndBodyList::ndNode* bodyNode = info.m_bodyList->GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndBodyKinematic* const body = bodyNode->GetInfo();

		ndShape* const shape = body->GetCollisionShape().GetShape();
		ndTree<dInt32, const ndShape*>::ndNode* shapeNode0 = info.m_shapeMap.Find(shape);
		if (!shapeNode0)
		{
			ndShapeCompound* const compound = shape->GetAsShapeCompound();
			if (compound)
			{
				ndShapeCompound::ndTreeArray::Iterator iter(compound->GetTree());
				for (iter.Begin(); iter; iter++)
				{
					ndShapeCompound::ndNodeBase* const node = iter.GetNode()->GetInfo();
					ndShapeInstance* const instance = node->GetShape();
					ndShape* const subShape = instance->GetShape();
					ndTree<dInt32, const ndShape*>::ndNode* subShapeNode = info.m_shapeMap.Find(subShape);
					if (!subShapeNode)
					{
						info.m_shapeMap.Insert(info.m_shapeMap.GetCount(), subShape);
					}
				}
			}
			shapeNode0 = info.m_shapeMap.Insert(info.m_shapeMap.GetCount(), shape);
		}
		descriptor.m_shapeNodeHash = shapeNode0->GetInfo();

		ndTree<dInt32, const ndBodyKinematic*>::ndNode* bodyHashNode = info.m_bodyMap.Find(body);
		if (!bodyHashNode)
		{
			bodyHashNode = info.m_bodyMap.Insert(info.m_bodyMap.GetCount() + 1, body);
		}
		descriptor.m_nodeNodeHash = bodyHashNode->GetInfo();
		body->Save(descriptor);
	}
}

void ndLoadSave::SaveJoints(ndLoadSaveInfo& info)
{
	ndLoadSaveBase::ndSaveDescriptor descriptor;
	descriptor.m_assetPath = info.m_assetPath;
	descriptor.m_assetName = info.m_assetName;
	descriptor.m_rootNode = info.m_jointsNode;

	for (ndJointList::ndNode* jointNode = info.m_jointList->GetFirst(); jointNode; jointNode = jointNode->GetNext())
	{
		ndJointBilateralConstraint* const joint = jointNode->GetInfo();
		const ndBodyKinematic* const body0 = joint->GetBody0();
		ndTree<dInt32, const ndBodyKinematic*>::ndNode* bodyNode0 = info.m_bodyMap.Find(body0);
		if (!bodyNode0)
		{
			bodyNode0 = info.m_bodyMap.Insert(info.m_bodyMap.GetCount(), body0);
		}
		const ndBodyKinematic* const body1 = joint->GetBody1()->GetAsBodySentinel() ? nullptr : joint->GetBody1();
		ndTree<dInt32, const ndBodyKinematic*>::ndNode* bodyNode1 = info.m_bodyMap.Find(body1);
		if (!bodyNode1)
		{
			bodyNode1 = info.m_bodyMap.Insert(info.m_bodyMap.GetCount(), body1);
		}

		descriptor.m_body0NodeHash = bodyNode0->GetInfo();
		descriptor.m_body1NodeHash = bodyNode1->GetInfo();

		ndTree<dInt32, const ndJointBilateralConstraint*>::ndNode* jointHashNode = info.m_jointMap.Find(joint);
		if (!jointHashNode)
		{
			jointHashNode = info.m_jointMap.Insert(info.m_jointMap.GetCount(), joint);
		}

		descriptor.m_nodeNodeHash = jointHashNode->GetInfo();
		joint->Save(descriptor);
	}
}

void ndLoadSave::SaveModels(ndLoadSaveInfo& info)
{
	ndLoadSaveBase::ndSaveDescriptor descriptor;
	descriptor.m_assetPath = info.m_assetPath;
	descriptor.m_assetName = info.m_assetName;
	descriptor.m_rootNode = info.m_modelsNode;
	descriptor.m_bodyMap = &info.m_bodyMap;
	descriptor.m_jointMap = &info.m_jointMap;
	
	for (ndModelList::ndNode* modelNode = info.m_modelList->GetFirst(); modelNode; modelNode = modelNode->GetNext())
	{
		ndModel* const model = modelNode->GetInfo();
		descriptor.m_nodeNodeHash = info.m_modelMap.GetCount();
		info.m_modelMap.Insert(descriptor.m_nodeNodeHash, model);
		model->Save(descriptor);
	}
}

bool ndLoadSave::LoadScene(const char* const path)
{
	char* const oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");

	nd::TiXmlDocument doc(path);
	doc.LoadFile();
	if (doc.Error())
	{
		setlocale(LC_ALL, oldloc);
		return false;
	}
	dAssert(!doc.Error());

	if (!doc.FirstChild("ndWorld"))
	{
		return false;
	}

	char assetPath[1024];
	strcpy(assetPath, path);

	char* namePtr = strrchr(assetPath, '/');
	if (!namePtr)
	{
		namePtr = strrchr(assetPath, '\\');
	}
	namePtr[0] = 0;

	const nd::TiXmlElement* const worldNode = doc.RootElement();
	ndShapeLoaderCache shapesMap;

	ndBodySentinel sentinel;
	m_bodyMap.Insert(&sentinel, 0);

	LoadSceneSettings(worldNode, assetPath);
	LoadShapes(worldNode, assetPath, shapesMap);
	LoadBodies(worldNode, assetPath, shapesMap);
	LoadJoints(worldNode, assetPath);
	LoadModels(worldNode, assetPath);
	setlocale(LC_ALL, oldloc);

	m_bodyMap.Remove(0);
	return true;
}

void ndLoadSave::SaveScene(const char* const path, const ndWorld* const world, const ndWordSettings* const setting)
{
	ndLoadSaveInfo info;
	info.ExtensionAndFilePath(path);

	nd::TiXmlDocument asciifile;
	nd::TiXmlDeclaration* const decl = new nd::TiXmlDeclaration("1.0", "", "");
	asciifile.LinkEndChild(decl);

	nd::TiXmlElement* const worldNode = new nd::TiXmlElement("ndWorld");
	asciifile.LinkEndChild(worldNode);

	info.m_worldNode = worldNode;
	info.m_settingsNode = new nd::TiXmlElement("ndSettings");
	info.m_shapesNode = new nd::TiXmlElement("ndShapes");
	info.m_bodiesNode = new nd::TiXmlElement("ndBodies");
	info.m_jointsNode = new nd::TiXmlElement("ndJoints");
	info.m_modelsNode = new nd::TiXmlElement("ndModels");

	worldNode->LinkEndChild(info.m_settingsNode);
	worldNode->LinkEndChild(info.m_shapesNode);
	worldNode->LinkEndChild(info.m_bodiesNode);
	worldNode->LinkEndChild(info.m_jointsNode);
	worldNode->LinkEndChild(info.m_modelsNode);

	info.m_setting = setting;
	info.m_bodyList = &world->GetBodyList();
	info.m_jointList = &world->GetJointList();
	info.m_modelList = &world->GetModelList();

	info.m_bodyMap.Insert(0, nullptr);
	SaveSceneSettings(info);
	SaveModels(info);
	SaveJoints(info);
	info.m_bodyMap.Remove((ndBodyKinematic*)nullptr);

	SaveBodies(info);
	SaveShapes(info);

	char* const oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");
	asciifile.SaveFile(info.m_fileName);
	setlocale(LC_ALL, oldloc);
}

void ndLoadSave::SaveModel(const char* const path, const ndModel* const model)
{
	ndLoadSaveInfo info;
	info.ExtensionAndFilePath(path);

	nd::TiXmlDocument asciifile;
	nd::TiXmlDeclaration* const decl = new nd::TiXmlDeclaration("1.0", "", "");
	asciifile.LinkEndChild(decl);
	
	nd::TiXmlElement* const worldNode = new nd::TiXmlElement("ndWorld");
	asciifile.LinkEndChild(worldNode);
	
	info.m_worldNode = worldNode;
	info.m_settingsNode = new nd::TiXmlElement("ndSettings");
	info.m_shapesNode = new nd::TiXmlElement("ndShapes");
	info.m_bodiesNode = new nd::TiXmlElement("ndBodies");
	info.m_jointsNode = new nd::TiXmlElement("ndJoints");
	info.m_modelsNode = new nd::TiXmlElement("ndModels");
	
	worldNode->LinkEndChild(info.m_settingsNode);
	worldNode->LinkEndChild(info.m_shapesNode);
	worldNode->LinkEndChild(info.m_bodiesNode);
	worldNode->LinkEndChild(info.m_jointsNode);
	worldNode->LinkEndChild(info.m_modelsNode);
	
	ndModelList modelList;
	ndJointList jointList;
	ndBodyList bodyList;
	modelList.Append((ndModel*)model);

	ndWordSettings settings;
	info.m_setting = &settings;
	info.m_bodyList = &bodyList;
	info.m_modelList = &modelList;
	info.m_jointList = &jointList;

	info.m_bodyMap.Insert(0, nullptr);
	SaveSceneSettings(info);
	SaveModels(info);

	ndTree<dInt32, const ndJointBilateralConstraint*>::Iterator jointIter(info.m_jointMap);
	for (jointIter.Begin(); jointIter; jointIter++)
	{
		jointList.Append((ndJointBilateralConstraint*)jointIter.GetKey());
	}
	SaveJoints(info);
	info.m_bodyMap.Remove((ndBodyKinematic*)nullptr);

	ndTree<dInt32, const ndBodyKinematic*>::Iterator bodyIter(info.m_bodyMap);
	for (bodyIter.Begin(); bodyIter; bodyIter++)
	{
		bodyList.Append((ndBodyKinematic*)bodyIter.GetKey());
	}
	SaveBodies(info);
	SaveShapes(info);
	
	char* const oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");
	asciifile.SaveFile(info.m_fileName);
	setlocale(LC_ALL, oldloc);
}

