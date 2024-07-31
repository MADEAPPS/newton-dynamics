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

class ndModelArticulation;
class ndUrdfFile : public ndClassAlloc
{
	public:
	ndUrdfFile();
	virtual ~ndUrdfFile();

	virtual ndModelArticulation* Import(const char* const fileName);
	virtual void Export(const char* const fileName, ndModelArticulation* const model);

	private:
	void CheckUniqueNames(ndModelArticulation* const model);
	void AddLinks(nd::TiXmlElement* const rootNode, const ndModelArticulation* const model);
	void AddJoints(nd::TiXmlElement* const rootNode, const ndModelArticulation* const model);

	void AddMaterials(nd::TiXmlElement* const rootNode, const ndModelArticulation* const model);
	void AddLink(nd::TiXmlElement* const rootNode, const ndModelArticulation::ndNode* const link);
	void AddJoint(nd::TiXmlElement* const rootNode, const ndModelArticulation::ndNode* const link);

	void AddInertia(nd::TiXmlElement* const linkNode, const ndModelArticulation::ndNode* const link);
	void AddGeometry(nd::TiXmlElement* const linkNode, const ndModelArticulation::ndNode* const link);
	void AddCollision(nd::TiXmlElement* const linkNode, const ndModelArticulation::ndNode* const link);

	void AddPose(nd::TiXmlElement* const linkNode, const ndMatrix& pose);
	void AddCollision(nd::TiXmlElement* const linkNode, const ndModelArticulation::ndNode* const link, const ndShapeInstance& collision);


	class Material
	{
		public:
		Material()
			:m_color(1.0f, 1.0f, 1.0f, 1.0f)
		{
		}
		ndVector m_color;
	};

	void LoadMaterials(const nd::TiXmlElement* const rootNode, ndTree<Material, ndString>& materials);
	void LoadLinks(const nd::TiXmlElement* const rootNode, const ndTree<Material, ndString>& materials, ndTree<ndBodyDynamic*, ndString>& bodyMap);
	void LoadJoints(const nd::TiXmlElement* const rootNode, const ndTree<ndBodyDynamic*, ndString>& bodyMap, ndTree<ndJointBilateralConstraint*, ndString>& joints);

	ndBodyDynamic* CreateBody(const nd::TiXmlElement* const rootNode, const ndTree<Material, ndString>& materials);
	ndJointBilateralConstraint* CreateJoint(const nd::TiXmlElement* const rootNode, const ndTree<ndBodyDynamic*, ndString>& bodyMap);
};


#endif