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

#include "ndModelStdafx.h"
#include "ndMeshLoader.h"
#include "ndAnimationSequence.h"
#include "ndAnimationKeyframesTrack.h"

ndMeshLoader::ndMeshLoader()
	:ndClassAlloc()
	,m_mesh(nullptr)
{
}

ndMeshLoader::ndMeshLoader(const ndMeshLoader& src)
	:ndClassAlloc(src)
	,m_mesh(nullptr)
{
	ndAssert(0);
}

ndMeshLoader::~ndMeshLoader()
{
}

const ndSharedPtr<ndAnimationSequence> ndMeshLoader::FindSequence(const ndString& fbxPathAnimName) const
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::ndNode* const node = m_animationCache.Find(fbxPathAnimName);
	if (node)
	{
		return node->GetInfo();
	}
	return ndSharedPtr<ndAnimationSequence>(nullptr);
}

ndSharedPtr<ndAnimationSequence> ndMeshLoader::ImportFbxAnimation(const ndString& fbxPathAnimName)
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::ndNode* node = m_animationCache.Find(fbxPathAnimName);
	if (!node)
	{
		ndFbxMeshLoader animLoader;
		ndSharedPtr<ndAnimationSequence> sequence (animLoader.LoadAnimation(fbxPathAnimName.GetStr()));
		if (sequence)
		{
			node = m_animationCache.Insert(sequence, fbxPathAnimName);
		}
	}
	return node ? node->GetInfo() : ndSharedPtr<ndAnimationSequence>(nullptr);
}

ndSharedPtr<ndAnimationSequence> ndMeshLoader::GetAnimationSequence(const ndString& pathAnimName)
{
	//ndAssert(0);
	//return ndSharedPtr<ndAnimationSequence>(nullptr);
	return ImportFbxAnimation(pathAnimName);
}

void ndMeshLoader::SetTranslationTracks(const ndString& boneName)
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::Iterator it(m_animationCache);
	for (it.Begin(); it; it++)
	{
		const ndSharedPtr<ndAnimationSequence>& cycle = it.GetNode()->GetInfo();
		for (ndList<ndAnimationKeyFramesTrack>::ndNode* node = cycle->GetTracks().GetFirst(); node; node = node->GetNext())
		{
			ndAnimationKeyFramesTrack& track = node->GetInfo();
			ndString name(track.GetName());
			name.ToLower();
			if (name.Find(boneName) != -1)
			{
				ndAnimationKeyFramesTrack& translationTrack = cycle->GetTranslationTrack();
				ndVector translation(ndVector::m_zero);
				ndReal offset = ndReal(track.m_position[0].m_x);
				for (ndInt32 i = 0; i < track.m_position.GetCount(); ++i)
				{
					translation.m_x = track.m_position[i].m_x - offset;
					translationTrack.m_position.PushBack(translation);
					translationTrack.m_position.m_time.PushBack(track.m_position.m_time[i]);
					track.m_position[i].m_x = offset;
				}
				break;
			}
		}
	}
}

ndString ndMeshLoader::GetPath(const ndString& fullPathName) const
{
	const char* ptr = strrchr(fullPathName.GetStr(), '/');
	if (!ptr)
	{
		ptr = strrchr(fullPathName.GetStr(), '\\');
	}
	return ndString (fullPathName.GetStr(), ndInt32(fullPathName.Size() - strlen(ptr + 1)));
}

ndString ndMeshLoader::GetName(const ndString& fullPathName) const
{
	const char* ptr1 = strrchr(fullPathName.GetStr(), '.');
	const char* ptr0 = strrchr(fullPathName.GetStr(), '/');
	if (!ptr0)
	{
		ptr0 = strrchr(fullPathName.GetStr(), '\\');
	}
	ndAssert(ptr0);
	ndAssert(ptr1);
	ndInt32 start = ndInt32(fullPathName.Size() - strlen(ptr0 + 1));
	ndInt32 end = ndInt32(fullPathName.Size() - strlen(ptr1));
	return fullPathName.SubString(start, end - start);
}

bool ndMeshLoader::ImportFbx(const ndString& fbxPathMeshName)
{
	ndFbxMeshLoader loader;
	m_mesh = ndSharedPtr<ndMesh>(loader.LoadMesh(fbxPathMeshName.GetStr(), false));
#if 0
	//ndAssert(0);
	ndTrace(("exporting mesh %s\n", fbxPathMeshName.GetStr()));
	ndString tmpName(fbxPathMeshName);
	tmpName.ToLower();
	ndString exportName(tmpName.SubString(0, tmpName.Find(".fbx")) + ".nd");
	SaveMesh(exportName);
#endif
	return m_mesh;
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
		mesh->SetNodeType(ndMesh::m_node);
		if (!strcmp(nodeType, "bone"))
		{
			mesh->SetNodeType(ndMesh::m_bone);
		}
		else if (!strcmp(nodeType, "endBone"))
		{
			mesh->SetNodeType(ndMesh::m_boneEnd);
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
	for (ndMesh* node = m_mesh->IteratorFirst(); node; node = node->IteratorNext())
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
