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

#ifndef _ND_URDF_FILE_H_
#define _ND_URDF_FILE_H_

#include "ndModelArticulation.h"

class ndUrdfBodyNotify : public ndBodyNotify
{
	public:
	ndUrdfBodyNotify(ndMeshEffect* const mesh)
		:ndBodyNotify(ndVector::m_zero)
		,m_offset(ndGetIdentityMatrix())
		,m_mesh(mesh)
	{
	}

	ndUrdfBodyNotify(const ndUrdfBodyNotify& src)
		:ndBodyNotify(src)
		,m_offset(src.m_offset)
		,m_mesh(src.m_mesh)
	{
	}

	ndBodyNotify* Clone() const
	{
		return new ndUrdfBodyNotify(*this);
	}

	ndUrdfBodyNotify* GetAsUrdfBodyNotify()
	{
		return this;
	}
	
	ndMatrix m_offset;
	ndSharedPtr<ndMeshEffect> m_mesh;
};

class ndUrdfFile : public ndClassAlloc
{
	class Material
	{
		public:
		Material()
			:m_color(1.0f, 1.0f, 1.0f, 1.0f)
		{
			m_texture[0] = 0;
		}
		ndVector m_color;
		char m_texture[256];
	};

	public:
	D_NEWTON_API ndUrdfFile();
	D_NEWTON_API virtual ~ndUrdfFile();

	D_NEWTON_API virtual ndModelArticulation* Import(const char* const fileName);
	D_NEWTON_API virtual void Export(const char* const fileName, ndModelArticulation* const model);

	private:
	class Hierarchy
	{
		public:
		Hierarchy(const nd::TiXmlNode* const link)
			:m_parent(nullptr)
			,m_link(link)
			,m_joint(link)
			,m_parentLink(nullptr)
			,m_articulation(nullptr)
			,m_parentArticulation(nullptr)
		{
		}

		Hierarchy* m_parent;
		const nd::TiXmlNode* m_link;
		const nd::TiXmlNode* m_joint;
		const nd::TiXmlNode* m_parentLink;
		ndModelArticulation::ndNode* m_articulation;
		ndModelArticulation::ndNode* m_parentArticulation;
	};

	class Surrogate: public ndNodeHierarchy<Surrogate>
	{
		public:
		ndVector m_com;
		ndMatrix m_bodyMatrix;
		ndMatrix m_shapeLocalMatrix;
		ndMatrix m_bodyInertia;
		ndMatrix m_jointLocalMatrix0;
		ndMatrix m_jointLocalMatrix1;
		ndModelArticulation::ndNode* m_articulation;
	};

	void ExportMakeNamesUnique(ndModelArticulation* const model);
	Surrogate* ExportMakeSurrogate(ndModelArticulation* const model);
	void ExportCollectTransforms(Surrogate* const surrogateRoot);
	void ExportOrigin(nd::TiXmlElement* const linkNode, const ndMatrix& pose);
	void ExportLink(nd::TiXmlElement* const rootNode, const Surrogate* const link);
	void ExportJoint(nd::TiXmlElement* const rootNode, const Surrogate* const link);
	void ExportVisual(nd::TiXmlElement* const linkNode, const Surrogate* const link);
	void ExportInertia(nd::TiXmlElement* const linkNode, const Surrogate* const link);
	void ExportMaterials(nd::TiXmlElement* const rootNode, const Surrogate* const link);
	void ExportCollision(nd::TiXmlElement* const linkNode, const Surrogate* const link);
	
	void ImportMaterials(const nd::TiXmlNode* const rootNode);
	ndBodyDynamic* ImportLink(const nd::TiXmlNode* const linkNode);
	ndMatrix ImportOrigin(const nd::TiXmlNode* const parentNode) const;
	void ImportVisual(const nd::TiXmlNode* const linkNode, ndBodyDynamic* const body);
	void ImportInertia(const nd::TiXmlNode* const linkNode, ndBodyDynamic* const body);
	void ImportCollision(const nd::TiXmlNode* const linkNode, ndBodyDynamic* const body);
	void ImportStlMesh(const char* const pathName, ndMeshEffect* const meshEffect) const;
	ndJointBilateralConstraint* ImportJoint(const nd::TiXmlNode* const jointNode, ndBodyDynamic* const child, ndBodyDynamic* const parent);

	ndString m_path;
	ndArray<Material> m_materials;
	ndTree<Hierarchy, ndString> m_bodyLinks;
	ndTree<ndInt32, ndString> m_materialMap;
};

#endif