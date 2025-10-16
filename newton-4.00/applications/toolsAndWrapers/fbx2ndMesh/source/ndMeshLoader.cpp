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

ndSharedPtr<ndAnimationSequence> ndMeshLoader::GetAnimationSequence(const ndString& fbxPathAnimName)
{
	ndAssert(0);
	return ndSharedPtr<ndAnimationSequence>(nullptr);
	//ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::ndNode* node = m_animationCache.Find(fbxPathAnimName);
	//if (!node)
	//{
	//	ndSharedPtr<ndAnimationSequence> sequence (LoadAnimation(fbxPathAnimName.GetStr()));
	//	if (sequence)
	//	{
	//		node = m_animationCache.Insert(sequence, fbxPathAnimName);
	//	}
	//}
	//return node ? node->GetInfo() : ndSharedPtr<ndAnimationSequence>(nullptr);
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

	const ndString exportName(GetPath(fbxPathMeshName) + GetName(fbxPathMeshName) + ".nd");
	ndTrace(("exporting mesh %s\n", exportName.GetStr()));
	SaveMesh(exportName);
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

		const nd::TiXmlElement* const xmlGeometry = (nd::TiXmlElement*)entry.m_xmlNode->FirstChild("ndGeometry");
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
		//xmlSaveParam(entry.m_parentXml, "meshMatrix", entry.m_meshNode->m_meshMatrix);

		if (*entry.m_meshNode->GetMesh())
		{
			nd::TiXmlElement* const geometry = new nd::TiXmlElement("ndGeometry");
			entry.m_parentXml->LinkEndChild(geometry);
			entry.m_meshNode->GetMesh()->SerializeToXml(geometry);
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
