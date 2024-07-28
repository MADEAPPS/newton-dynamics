/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndUrdfFile.h"
#include "ndModelArticulation.h"

ndUrdfFile::ndUrdfFile()
	:ndClassAlloc()
{
}

ndUrdfFile::~ndUrdfFile()
{
}

//ndModelArticulation* ndUrdfFile::Import(const char* const fileName)
ndModelArticulation* ndUrdfFile::Import(const char* const)
{
	ndAssert(0);
	return nullptr;
}

void ndUrdfFile::Export(const char* const filePathName, ndModelArticulation* const model)
{
	ndAssert(strstr(filePathName, ".urdf"));

	nd::TiXmlDocument* const doc = new nd::TiXmlDocument("");
	nd::TiXmlDeclaration* const decl = new nd::TiXmlDeclaration("1.0", "", "");
	doc->LinkEndChild(decl);
	ndString oldloc(setlocale(LC_ALL, 0));

	nd::TiXmlElement* const rootNode = new nd::TiXmlElement("robot");
	doc->LinkEndChild(rootNode);
	if (model->GetName() == "")
	{
		rootNode->SetAttribute("name", "robot_model");
	}
	else
	{
		rootNode->SetAttribute("name", model->GetName().GetStr());
	}

	CheckUniqueNames(model);
	AddLinks(rootNode, model);
	AddJoints(rootNode, model);

	doc->SaveFile(filePathName);
	setlocale(LC_ALL, oldloc.GetStr());
	delete doc;
}

void ndUrdfFile::CheckUniqueNames(ndModelArticulation* const model)
{
	ndTree < ndModelArticulation::ndNode*, ndString> filter;
	
	ndString baseName("link_");
	for (ndModelArticulation::ndNode* childNode = model->GetRoot()->GetFirstIterator(); childNode; childNode = childNode->GetNextIterator())
	{
		if (childNode->m_name == "")
		{
			childNode->m_name = baseName + "0";
		}
		ndInt32 count = 1;
		while (filter.Find(childNode->m_name))
		{
			childNode->m_name = baseName + count;
			count++;
		}
		filter.Insert(childNode, childNode->m_name);
	}
}

void ndUrdfFile::AddLinks(nd::TiXmlElement* const rootNode, const ndModelArticulation* const model)
{
	ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;
	stack.PushBack(model->GetRoot());

	while (stack.GetCount())
	{
		ndModelArticulation::ndNode* const node = stack[stack.GetCount() - 1];
		stack.SetCount(stack.GetCount() - 1);

		nd::TiXmlElement* const link = new nd::TiXmlElement("link");
		rootNode->LinkEndChild(link);

		link->SetAttribute("name", node->m_name.GetStr());

		for (ndModelArticulation::ndNode* childNode = node->GetFirstChild(); childNode; childNode = childNode->GetNext())
		{
			stack.PushBack(childNode);
		}
	}
}

void ndUrdfFile::AddJoints(nd::TiXmlElement* const rootNode, const ndModelArticulation* const model)
{
	ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;
	//stack.PushBack(model->GetRoot());

	for (ndModelArticulation::ndNode* child = model->GetRoot()->GetFirstChild(); child; child = child->GetNext())
	{
		stack.PushBack(child);
	}

	while (stack.GetCount())
	{
		ndModelArticulation::ndNode* const node = stack[stack.GetCount() - 1];
		stack.SetCount(stack.GetCount() - 1);

		nd::TiXmlElement* const joint = new nd::TiXmlElement("joint");
		rootNode->LinkEndChild(joint);

		ndString name(node->GetParent()->m_name + "_" + node->m_name);
		joint->SetAttribute("name", name.GetStr());
		joint->SetAttribute("type", "xxxxxxxx");

		nd::TiXmlElement* const parent = new nd::TiXmlElement("parent");
		joint->LinkEndChild(parent);
		parent->SetAttribute("link", node->GetParent()->m_name.GetStr());

		nd::TiXmlElement* const child = new nd::TiXmlElement("child");
		joint->LinkEndChild(child);
		child->SetAttribute("link", node->m_name.GetStr());

		for (ndModelArticulation::ndNode* childNode = node->GetFirstChild(); childNode; childNode = childNode->GetNext())
		{
			stack.PushBack(childNode);
		}
	}
}