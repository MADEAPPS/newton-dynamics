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
#include "ndCollisionStdafx.h"
#include "ndMesh.h"
#include "ndMeshEffect.h"
#include "ndMeshLoader.h"

ndMeshLoader::ndMeshLoader()
	:ndClassAlloc()
	,m_mesh(nullptr)
{
}

ndMeshLoader::~ndMeshLoader()
{
}

bool ndMeshLoader::LoadMesh(const ndString& fullPathMeshName)
{
	ndString oldloc(setlocale(LC_ALL, 0));
	nd::TiXmlDocument doc(fullPathMeshName.GetStr());
	doc.LoadFile();
	if (doc.Error())
	{
		m_mesh = ndSharedPtr<ndMesh>(nullptr);
		setlocale(LC_ALL, oldloc.GetStr());
		return false;
	}

	struct MeshXmlNodePair
	{
		MeshXmlNodePair() {}
		MeshXmlNodePair(ndMesh* const mesh, const nd::TiXmlElement* const xmlNode)
			:m_mesh(mesh)
			,m_xmlNode(xmlNode)
		{
		}
		ndMesh* m_mesh;
		const nd::TiXmlElement* m_xmlNode;
	};

	ndFixSizeArray<MeshXmlNodePair, 1024> stack;

	const nd::TiXmlElement* const rootNode = doc.RootElement();
	ndAssert(strcmp(rootNode->Value(), "ndMesh") == 0);

	m_mesh = ndSharedPtr<ndMesh>(new ndMesh());
	stack.PushBack(MeshXmlNodePair(*m_mesh, rootNode));

	while (stack.GetCount())
	{
		MeshXmlNodePair entry(stack.Pop());

		ndMesh* const mesh = entry.m_mesh;

		mesh->m_name = ndString(xmlGetString(entry.m_xmlNode, "name"));
		mesh->m_matrix = xmlGetMatrix(entry.m_xmlNode, "matrix");

		nd::TiXmlElement* const xmlNodeType = (nd::TiXmlElement*)entry.m_xmlNode->FirstChild("type");
		ndAssert(xmlNodeType);
		const char* const nodeType = xmlGetNameAttribute(xmlNodeType, "nodeType");
		if (!strcmp(nodeType, "node"))
		{
			mesh->SetNodeType(ndMesh::m_node);
		}
		else if (!strcmp(nodeType, "bone"))
		{
			mesh->SetNodeType(ndMesh::m_bone);
		}
		else if (!strcmp(nodeType, "endBone"))
		{
			mesh->SetNodeType(ndMesh::m_boneEnd);
		}
		else if (!strcmp(nodeType, "collisionShape"))
		{
			mesh->SetNodeType(ndMesh::m_collisionShape);
		}
		else
		{
			ndAssert(0);
		}
		ndTriplexReal target(xmlGetTriplexRealAttribute(xmlNodeType, "target"));

		const nd::TiXmlElement* const xmlGeometry = (nd::TiXmlElement*)entry.m_xmlNode->FirstChild("geometry");
		if (xmlGeometry)
		{
			ndSharedPtr<ndMeshEffect> geometry(new ndMeshEffect());
			geometry->DeserializeFromXml(xmlGeometry);
			mesh->SetMesh(geometry);
		}

		for (const nd::TiXmlNode* node = entry.m_xmlNode->FirstChild("ndMesh"); node; node = node->NextSibling("ndMesh"))
		{
			const nd::TiXmlElement* const linkNode = (nd::TiXmlElement*)node;
			ndAssert(strcmp(linkNode->Value(), "ndMesh") == 0);

			ndSharedPtr<ndMesh> child (new ndMesh());
			entry.m_mesh->AddChild(child);
			stack.PushBack(MeshXmlNodePair(*child, linkNode));
		}
	}

	setlocale(LC_ALL, oldloc.GetStr());
	return true;
}

void ndMeshLoader::SaveMesh(const ndString& fullPathName)
{
	ndString oldloc(setlocale(LC_ALL, 0));
	ndSharedPtr<nd::TiXmlDocument> doc(new nd::TiXmlDocument(""));
	nd::TiXmlDeclaration* const decl = new nd::TiXmlDeclaration("1.0", "", "");
	doc->LinkEndChild(decl);

	nd::TiXmlElement* const rootNode = new nd::TiXmlElement("ndMesh");
	doc->LinkEndChild(rootNode);

	// make the bone list for sikn and othe dependencies
	ndTree<ndString, ndUnsigned32> bonesMap;
	for (ndMesh* node = m_mesh->IteratorFirst(); node; node = node->IteratorNext(*m_mesh))
	{
		if (node->m_name.GetStr())
		{
			ndUnsigned32 hash = ndUnsigned32(ndCRC64(node->m_name.GetStr()) & 0xffffffff);
			bonesMap.Insert(node->m_name, hash);
		}
	}

	struct MeshXmlNodePair
	{
		const ndMesh* m_meshNode;
		nd::TiXmlElement* m_parentXml;
	};

	ndFixSizeArray<MeshXmlNodePair, 1024> stack;
	MeshXmlNodePair pair;
	pair.m_meshNode = *m_mesh;
	pair.m_parentXml = rootNode;
	stack.PushBack(pair);

	while (stack.GetCount())
	{
		MeshXmlNodePair entry(stack.Pop());
		xmlSaveParam(entry.m_parentXml, "name", entry.m_meshNode->m_name.GetStr());
		xmlSaveParam(entry.m_parentXml, "matrix", entry.m_meshNode->m_matrix);

		nd::TiXmlElement* const xmlNodeType = new nd::TiXmlElement("type");
		entry.m_parentXml->LinkEndChild(xmlNodeType);
		switch (entry.m_meshNode->m_type)
		{
			case ndMesh::m_node:
				xmlSaveAttribute(xmlNodeType, "nodeType", "node");
				break;
			case ndMesh::m_bone:
				xmlSaveAttribute(xmlNodeType, "nodeType", "bone");
				break;
			case ndMesh::m_boneEnd:
				xmlSaveAttribute(xmlNodeType, "nodeType", "endBone");
				break;

			case ndMesh::m_collisionShape:
				xmlSaveAttribute(xmlNodeType, "nodeType", "collisionShape");
				break;
		}
		ndVector boneTarget(entry.m_meshNode->GetBoneTarget());
		xmlSaveAttribute(xmlNodeType, "target", ndTriplexReal(ndReal(boneTarget.m_x), ndReal(boneTarget.m_y), ndReal(boneTarget.m_z)));

		if (*entry.m_meshNode->GetMesh())
		{
			nd::TiXmlElement* const geometry = new nd::TiXmlElement("geometry");
			entry.m_parentXml->LinkEndChild(geometry);
			entry.m_meshNode->GetMesh()->SerializeToXml(geometry, bonesMap);
		}

		for (ndList<ndSharedPtr<ndMesh>>::ndNode* node = entry.m_meshNode->m_children.GetFirst(); node; node = node->GetNext())
		{
			MeshXmlNodePair childPair;
			childPair.m_meshNode = *node->GetInfo();
			nd::TiXmlElement* const child = new nd::TiXmlElement("ndMesh");
			entry.m_parentXml->LinkEndChild(child);
			childPair.m_parentXml = child;
			stack.PushBack(childPair);
		}
	};

	doc->SaveFile(fullPathName.GetStr());
	setlocale(LC_ALL, oldloc.GetStr());
}
