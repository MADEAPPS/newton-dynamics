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
		,m_mesh(mesh)
	{
	}

	ndUrdfBodyNotify(const ndUrdfBodyNotify& src)
		:ndBodyNotify(src)
		,m_mesh(src.m_mesh)
	{
	}

	ndBodyNotify* Clone() const
	{
		return new ndUrdfBodyNotify(*this);
	}
	
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
		{
		}

		Hierarchy* m_parent;
		const nd::TiXmlNode* m_link;
		const nd::TiXmlNode* m_joint;
		const nd::TiXmlNode* m_parentLink;
		ndModelArticulation::ndNode* m_articulation;
	};

	class Surrogate: public ndNodeHierarchy<Surrogate>
	{
		public:
		ndMatrix m_bodyMatrix;
		ndMatrix m_bodyInertia;
		ndMatrix m_shapeMatrixMatrix;
		ndMatrix m_jointBodyMatrix0;
		ndMatrix m_jointBodyMatrix1;
		ndVector m_com;
		ndModelArticulation::ndNode* m_articulation;
	};

	void MakeNamesUnique(ndModelArticulation* const model);
	void ExportOrigin(nd::TiXmlElement* const linkNode, const ndMatrix& pose);

	void ExportLink(nd::TiXmlElement* const rootNode, const Surrogate* const link);
	void ExportMaterials(nd::TiXmlElement* const rootNode, const Surrogate* const link);
	void ExportInertia(nd::TiXmlElement* const linkNode, const Surrogate* const link);
	void ExportCollision(nd::TiXmlElement* const linkNode, const Surrogate* const link);
	void ExportVisual(nd::TiXmlElement* const linkNode, const Surrogate* const link);
	void ExportJoint(nd::TiXmlElement* const rootNode, const Surrogate* const link);

	void ImportMaterials(const nd::TiXmlNode* const rootNode);
	ndBodyDynamic* ImportLink(const nd::TiXmlNode* const linkNode);
	ndMatrix ImportOrigin(const nd::TiXmlNode* const parentNode) const;
	ndJointBilateralConstraint* ImportJoint(const nd::TiXmlNode* const jointNode, ndBodyDynamic* const child, ndBodyDynamic* const parent);

	void ApplyRotation(ndModelArticulation* const model);
	Surrogate* ApplyInvRotation(ndModelArticulation* const model);
	void LoadStlMesh(const char* const pathName, ndMeshEffect* const meshEffect) const;

	ndString m_path;
	ndArray<Material> m_materials;
	ndTree<Hierarchy, ndString> m_bodyLinks;
	ndTree<ndInt32, ndString> m_materialMap;
};

#endif