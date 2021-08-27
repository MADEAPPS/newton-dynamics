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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndModel.h"
#include "ndLoadSave.h"
#include "ndBodyDynamic.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndWordSettings);

ndWordSettings::ndWordSettings(const dLoadSaveBase::dLoadDescriptor&)
	:dClassAlloc()
	,m_subSteps(2)
	,m_solverIterations(4)
{
}

void ndWordSettings::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);

	xmlSaveParam(childNode, "description", "string", "Newton Dynamics 4.00");
	xmlSaveParam(childNode, "revision", "string", "1.00");
	xmlSaveParam(childNode, "solverSubsteps", m_subSteps);
	xmlSaveParam(childNode, "solverIterations", m_solverIterations);
}

void ndWordSettings::Load(const dLoadSaveBase::dLoadDescriptor& desc)
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

	dLoadSaveBase::dLoadDescriptor settingDesc;
	settingDesc.m_rootNode = node;
	settingDesc.m_assetPath = assetPath;

	m_setting = D_CLASS_REFLECTION_LOAD_NODE(ndWordSettings, className, settingDesc);
	m_setting->Load(settingDesc);
}

void ndLoadSave::LoadCollisionShapes(const nd::TiXmlNode* const rootNode, 
	const char* const assetPath, ndShapeLoaderCache& shapesMap)
{
	const nd::TiXmlNode* const shapes = rootNode->FirstChild("ndShapes");
	if (shapes)
	{
		dLoadSaveBase::dLoadDescriptor descriptor;
		descriptor.m_assetPath = assetPath;

		class ndPendingCompounds
		{
			public:
			ndShapeInstance* m_compoundInstance;
			const nd::TiXmlNode* m_subShapeNodes;
		};

		dArray<ndPendingCompounds> pendingCompounds;
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
				ndShapeLoaderCache::dNode* const shapeMapNode = shapesMap.Insert(ndShapeInstance(shape), hashId);
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

void ndLoadSave::LoadRigidBodies(const nd::TiXmlNode* const rootNode, 
	const char* const assetPath, const ndShapeLoaderCache& shapesMap)
{
	const nd::TiXmlNode* const bodies = rootNode->FirstChild("ndBodies");
	if (bodies)
	{
		dLoadSaveBase::dLoadDescriptor descriptor;
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
	const nd::TiXmlNode* const joints = rootNode->FirstChild("ndBilateralJoints");
	if (joints)
	{
		ndBodyDynamic sentinelBody;
		dInt32 sentinelHash = 0;
		m_bodyMap.Insert(&sentinelBody, sentinelHash);
		dLoadSaveBase::dLoadDescriptor descriptor;
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
			}
		}
		
		m_bodyMap.Remove(sentinelHash);
	}
}

void ndLoadSave::LoadModels(const nd::TiXmlNode* const rootNode, const char* const assetPath)
{
	const nd::TiXmlNode* const models = rootNode->FirstChild("ndModels");
	if (models)
	{
		dLoadSaveBase::dLoadDescriptor descriptor;
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

void ndLoadSave::SaveSceneSettings(nd::TiXmlNode* const rootNode, const char* const assetPath, const char* const assetName, const ndWordSettings* const setting) const
{
	nd::TiXmlElement* const settingsNode = new nd::TiXmlElement("ndSettings");
	rootNode->LinkEndChild(settingsNode);

	dLoadSaveBase::dSaveDescriptor descriptor;
	descriptor.m_assetPath = assetPath;
	descriptor.m_assetName = assetName;
	descriptor.m_rootNode = settingsNode;
	setting->Save(descriptor);
}

void ndLoadSave::SaveCollisionShapes(nd::TiXmlNode* const shapeRootNode,
	const ndWorld* const world, const char* const assetPath, const char* const assetName,
	dTree<dUnsigned32, const ndShape*>& shapeMap)
{
	const ndBodyList& bodyList = world->GetBodyList();
	dArray<dTree<dUnsigned32, const ndShape*>::dNode*> shapeNodeOrder;

	for (ndBodyList::dNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndBodyKinematic* const body = bodyNode->GetInfo();
		ndShape* const shape = body->GetCollisionShape().GetShape();
		ndShapeCompound* const compound = shape->GetAsShapeCompound();
		if (compound)
		{
			ndShapeCompound::ndTreeArray::Iterator iter(compound->GetTree());
			for (iter.Begin(); iter; iter++)
			{
				ndShapeCompound::ndNodeBase* const node = iter.GetNode()->GetInfo();
				ndShapeInstance* const instance = node->GetShape();
				ndShape* const subShape = instance->GetShape();
				if (!shapeMap.Find(subShape))
				{
					shapeNodeOrder.PushBack(shapeMap.Insert(shapeNodeOrder.GetCount(), subShape));
				}
			}
		}
		else if (!shapeMap.Find(shape))
		{
			shapeNodeOrder.PushBack(shapeMap.Insert(shapeNodeOrder.GetCount(), shape));
		}
	}

	for (ndBodyList::dNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndBodyKinematic* const body = bodyNode->GetInfo();
		ndShape* const shape = body->GetCollisionShape().GetShape();
		ndShapeCompound* const compound = shape->GetAsShapeCompound();
		if (compound)
		{
			shapeNodeOrder.PushBack(shapeMap.Insert(shapeNodeOrder.GetCount(), shape));
		}
	}

	if (shapeMap.GetCount())
	{
		nd::TiXmlElement* const rootNode = new nd::TiXmlElement("ndShapes");
		shapeRootNode->LinkEndChild(rootNode);

		dLoadSaveBase::dSaveDescriptor descriptor;
		descriptor.m_assetPath = assetPath;
		descriptor.m_assetName = assetName;
		descriptor.m_rootNode = rootNode;
		descriptor.m_shapeMap = &shapeMap;
		for (dInt32 hashId = 0; hashId < shapeNodeOrder.GetCount(); hashId++)
		{
			descriptor.m_nodeNodeHash = hashId;
			const ndShape* const shape = shapeNodeOrder[hashId]->GetKey();
			shape->Save(descriptor);
		}
	}
}

void ndLoadSave::SaveRigidBodies(nd::TiXmlNode* const rootNode, const ndWorld* const world,
	const char* const assetPath, const char* const assetName,
	const dTree<dUnsigned32, const ndShape*>& shapesMap,
	dTree<dUnsigned32, const ndBodyKinematic*>& bodyMap)
{
	const ndBodyList& bodyList = world->GetBodyList();
	if (bodyList.GetCount())
	{
		dInt32 bodyIndex = 1;
		nd::TiXmlElement* const bodiesNode = new nd::TiXmlElement("ndBodies");
		rootNode->LinkEndChild(bodiesNode);
		dLoadSaveBase::dSaveDescriptor descriptor;
		descriptor.m_assetPath = assetPath;
		descriptor.m_assetName = assetName;
		descriptor.m_rootNode = bodiesNode;

		for (ndBodyList::dNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
		{
			ndBodyKinematic* const body = bodyNode->GetInfo();

			dInt32 shapeHash = shapesMap.Find(body->GetCollisionShape().GetShape())->GetInfo();
			descriptor.m_shapeNodeHash = shapeHash;
			descriptor.m_nodeNodeHash = bodyIndex;
			body->Save(descriptor);

			bodyMap.Insert(bodyIndex, body);
			bodyIndex++;
		}
	}
}

void ndLoadSave::SaveJoints(nd::TiXmlNode* const rootNode, const ndWorld* const world,
	const char* const assetPath, const char* const assetName,
	const dTree<dUnsigned32, const ndBodyKinematic*>& bodyMap,
	dTree<dUnsigned32, const ndJointBilateralConstraint*>& jointMap)
{
	const ndJointList& jointList = world->GetJointList();
	if (jointList.GetCount())
	{
		dInt32 jointIndex = 0;
		nd::TiXmlElement* const jointsNode = new nd::TiXmlElement("ndBilateralJoints");
		rootNode->LinkEndChild(jointsNode);
		dLoadSaveBase::dSaveDescriptor descriptor;
		descriptor.m_assetPath = assetPath;
		descriptor.m_assetName = assetName;
		descriptor.m_rootNode = jointsNode;

		for (ndJointList::dNode* jointNode = jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
		{
			ndJointBilateralConstraint* const joint = jointNode->GetInfo();
			dInt32 bodyHash0 = bodyMap.Find(joint->GetBody0())->GetInfo();
			dInt32 bodyHash1 = (joint->GetBody1() == world->GetSentinelBody()) ? 0 : bodyMap.Find(joint->GetBody1())->GetInfo();
			descriptor.m_body0NodeHash = bodyHash0;
			descriptor.m_body1NodeHash = bodyHash1;
			descriptor.m_nodeNodeHash = jointIndex;
			joint->Save(descriptor);
			jointMap.Insert(jointIndex, joint);
			jointIndex++;
		}
	}
}

void ndLoadSave::SaveModels(nd::TiXmlNode* const rootNode, const ndWorld* const world,
	const char* const assetPath, const char* const assetName,
	const dTree<dUnsigned32, const ndBodyKinematic*>& bodyMap,
	const dTree<dUnsigned32, const ndJointBilateralConstraint*>& jointMap,
	dTree<dUnsigned32, const ndModel*>& modelMap)
{
	const ndModelList& modelList = world->GetModelList();
	if (modelList.GetCount())
	{
		dInt32 modelIndex = 0;
		nd::TiXmlElement* const modelsNode = new nd::TiXmlElement("ndModels");
		rootNode->LinkEndChild(modelsNode);
		dLoadSaveBase::dSaveDescriptor descriptor;
		descriptor.m_assetPath = assetPath;
		descriptor.m_assetName = assetName;
		descriptor.m_rootNode = modelsNode;
		descriptor.m_bodyMap = &bodyMap;
		descriptor.m_jointMap = &jointMap;

		for (ndModelList::dNode* modelNode = modelList.GetFirst(); modelNode; modelNode = modelNode->GetNext())
		{
			ndModel* const model = modelNode->GetInfo();
			descriptor.m_nodeNodeHash = modelIndex;
			model->Save(descriptor);
			modelMap.Insert(modelIndex, model);
			modelIndex++;
		}
	}
}


void ndLoadSave::SaveScene(const char* const path, const ndWorld* const world, const ndWordSettings* const setting)
{
	char* const oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");

	nd::TiXmlDocument asciifile;
	nd::TiXmlDeclaration* const decl = new nd::TiXmlDeclaration("1.0", "", "");
	asciifile.LinkEndChild(decl);

	nd::TiXmlElement* const worldNode = new nd::TiXmlElement("ndWorld");
	asciifile.LinkEndChild(worldNode);

	char assetPath[1024];
	char assetName[1024];
	char fileNameExt[1024];

	strcpy(fileNameExt, path);
	char* const ext = strrchr(fileNameExt, '.');
	if (ext)
	{
		*ext = 0;
	}

	strcpy(assetPath, fileNameExt);
	strcat(fileNameExt, ".nd");

	char* namePtr = strrchr(assetPath, '/');
	if (!namePtr)
	{
		namePtr = strrchr(assetPath, '\\');
	}
	if (namePtr)
	{
		strcpy(assetName, namePtr + 1);
	}
	else
	{
		namePtr = assetPath;
		strcpy(assetName, namePtr);
	}
	namePtr[0] = 0;
	strcat(assetName, "_asset");

	dTree<dUnsigned32, const ndShape*> shapeMap;
	dTree<dUnsigned32, const ndModel*> modelMap;
	dTree<dUnsigned32, const ndBodyKinematic*> bodyMap;
	dTree<dUnsigned32, const ndJointBilateralConstraint*> jointMap;

	SaveSceneSettings(worldNode, assetPath, assetName, setting);
	SaveCollisionShapes(worldNode, world, assetPath, assetName, shapeMap);
	SaveRigidBodies(worldNode, world, assetPath, assetName, shapeMap, bodyMap);
	SaveJoints(worldNode, world, assetPath, assetName, bodyMap, jointMap);
	SaveModels(worldNode, world, assetPath, assetName, bodyMap, jointMap, modelMap);

	asciifile.SaveFile(path);
	setlocale(LC_ALL, oldloc);
}

bool ndLoadSave::LoadScene(const char* const path)
{
	char* const oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");

	nd::TiXmlDocument doc(path);
	doc.LoadFile();
	if (doc.Error())
	{
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

	//ndBodyLoaderCache bodyMap;
	//ndJointLoaderCache jointMap;
	//ndModelLoaderCache modelMap;
	ndShapeLoaderCache shapesMap;

	LoadSceneSettings(worldNode, assetPath);
	LoadCollisionShapes(worldNode, assetPath, shapesMap);
	LoadRigidBodies(worldNode, assetPath, shapesMap);
	LoadJoints(worldNode, assetPath);
	LoadModels(worldNode, assetPath);
	setlocale(LC_ALL, oldloc);

	return true;
}