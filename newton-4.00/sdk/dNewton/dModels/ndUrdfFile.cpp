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
#include "ndBodyDynamic.h"
#include "dJoints/ndJointHinge.h"
#include "dJoints/ndJointSlider.h"
#include "dJoints/ndJointFix6dof.h"

ndUrdfFile::ndUrdfFile()
	:ndClassAlloc()
{
}

ndUrdfFile::~ndUrdfFile()
{
}

// *******************************************************************************
// 
// *******************************************************************************
void ndUrdfFile::ImportMaterials(const nd::TiXmlNode* const rootNode)
{
	m_materials.SetCount(0);
	m_materialMap.RemoveAll();
	m_materials.PushBack(Material());
	for (const nd::TiXmlNode* node = rootNode->FirstChild("material"); node; node = node->NextSibling("material"))
	{
		const nd::TiXmlElement* const materialNode = (nd::TiXmlElement*)node;
		const nd::TiXmlElement* const color = (nd::TiXmlElement*)node->FirstChild("color");
		if (color)
		{
			Material material;

			const char* const name = materialNode->Attribute("name");
			m_materialMap.Insert(ndInt32(m_materials.GetCount()), name);

			const char* const rgba = color->Attribute("rgba");

			ndFloat32 r;
			ndFloat32 g;
			ndFloat32 b;
			ndFloat32 a;
			ndInt32 ret = 0;
			ret = sscanf(rgba, "%f %f %f %f", &r, &g, &b, &a);
			material.m_color.m_x = r;
			material.m_color.m_y = g;
			material.m_color.m_z = b;
			material.m_color.m_w = a;
			m_materials.PushBack(material);
		}
	}
}

// *******************************************************************************
// 
// *******************************************************************************
ndMatrix ndUrdfFile::ImportOrigin(const nd::TiXmlNode* const parentNode) const
{
	const nd::TiXmlElement* const origin = (nd::TiXmlElement*)parentNode->FirstChild("origin");

	ndMatrix matrix(ndGetIdentityMatrix());
	if (origin)
	{
		ndFloat32 x = ndFloat32(0.0f);
		ndFloat32 y = ndFloat32(0.0f);
		ndFloat32 z = ndFloat32(0.0f);
		ndFloat32 yaw = ndFloat32(0.0f);
		ndFloat32 roll = ndFloat32(0.0f);
		ndFloat32 pitch = ndFloat32(0.0f);

		const char* const xyz = origin->Attribute("xyz");
		if (xyz)
		{
			ndInt32 ret = 0;
			ret = sscanf(xyz, "%f %f %f", &x, &y, &z);
		}
		const char* const rpy = origin->Attribute("rpy");
		if (rpy)
		{
			ndInt32 ret = 0;
			ret = sscanf(rpy, "%f %f %f", &pitch, &yaw, &roll);
		}

		matrix = ndPitchMatrix(pitch) * ndYawMatrix(yaw) * ndRollMatrix(roll);
		matrix.m_posit.m_x = x;
		matrix.m_posit.m_y = y;
		matrix.m_posit.m_z = z;
	}
	return matrix;
}

void ndUrdfFile::LoadStlMesh(const char* const pathName, ndMeshEffect* const meshEffect) const
{
	char meshFile[256];
	char meshPath[1024];

	const char* meshName = strrchr(pathName, '/');
	if (!meshName)
	{
		meshName = strrchr(pathName, '\\');
	}
	sprintf(meshFile, "%s", meshName + 1);
	char* ptr = strrchr(meshFile, '.');
	sprintf(ptr, ".stl");

	sprintf(meshPath, "%s/%s", m_path.GetStr(), meshFile);

	FILE* const file = fopen(meshPath, "rb");
	ndAssert(file);
	if (file)
	{
		char buffer[256];
		for (ndInt32 i = 0; i < 80; ++i)
		{
			buffer[i] =char(fgetc(file));
		}

		ndInt32 numberOfTriangles;
		size_t ret = 0;
		ret = fread(&numberOfTriangles, 1, 4, file);
		ndFloat32 inchToMeters = 0.0254f;

		ndReal normal[3];
		ndReal triangle[3][3];
		short flags;

		for (ndInt32 i = 0; i < numberOfTriangles; ++i)
		{
			ret = fread(normal, 1, sizeof (normal), file);
			ret = fread(triangle, 1, sizeof(triangle), file);
			ret = fread(&flags, 1, sizeof(flags), file);

			meshEffect->BeginBuildFace();
			for (ndInt32 j = 0; j < 3; ++j)
			{
				meshEffect->AddMaterial(0);
				meshEffect->AddPoint(triangle[j][0] * inchToMeters, triangle[j][1] * inchToMeters, triangle[j][2] * inchToMeters);
				meshEffect->AddNormal(normal[0], normal[1], normal[2]);
			}
			meshEffect->EndBuildFace();
		}

		fclose(file);
	}
}

ndBodyDynamic* ndUrdfFile::ImportLink(const nd::TiXmlNode* const linkNode)
{
	ndBodyDynamic* const body = new ndBodyDynamic();

	ndArray<const nd::TiXmlNode*> collisions;
	for (const nd::TiXmlNode* node = linkNode->FirstChild("collision"); node; node = node->NextSibling("collision"))
	{
		collisions.PushBack(node);
	}

	ndShapeInstance collision(new ndShapeNull());
	auto GetCollisionShape = [this, &collision](const nd::TiXmlNode* const node)
	{
		const nd::TiXmlNode* const geometryNode = node->FirstChild("geometry");
		const nd::TiXmlElement* const shapeNode = (nd::TiXmlElement*)geometryNode->FirstChild();
		const char* const name = shapeNode->Value();

		ndShape* shape = nullptr;

		if (strcmp(name, "sphere") == 0)
		{
			ndFloat64 radius;
			shapeNode->Attribute("radius", &radius);
			shape = new ndShapeSphere(ndFloat32(radius));
		}
		else if (strcmp(name, "cylinder") == 0)
		{
			ndFloat64 length;
			ndFloat64 radius;
			shapeNode->Attribute("length", &length);
			shapeNode->Attribute("radius", &radius);
			shape = new ndShapeCylinder(ndFloat32(radius), ndFloat32(radius), ndFloat32(length));
			ndMatrix matrix(ndYawMatrix(ndPi * ndFloat32(0.5f)));
			collision.SetLocalMatrix(matrix);
		}
		else if (strcmp(name, "box") == 0)
		{
			ndFloat32 x;
			ndFloat32 y;
			ndFloat32 z;
			const char* const size = shapeNode->Attribute("size");
			ndInt32 ret = 0;
			ret = sscanf(size, "%f %f %f", &x, &y, &z);
			shape = new ndShapeBox(x, y, z);
		}
		else
		{
			const char* const meshPathName = shapeNode->Attribute("filename");

			ndMeshEffect meshEffect;
			meshEffect.BeginBuild();
			LoadStlMesh(meshPathName, &meshEffect);
			meshEffect.EndBuild();

			ndArray<ndVector> hull;
			ndInt32 vertexCount = meshEffect.GetVertexCount();
			ndInt32 stride = meshEffect.GetVertexStrideInByte();
			const char* vertexPoolBytes = (char*)meshEffect.GetVertexPool();
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				const ndFloat64* const vertexPoolFloat = (ndFloat64*)vertexPoolBytes;
				ndVector point(vertexPoolFloat[0], vertexPoolFloat[1], vertexPoolFloat[2], ndFloat64(0.0f));
				hull.PushBack(point);
				vertexPoolBytes += stride;
			}

			shape = new ndShapeConvexHull(vertexCount, ndInt32 (sizeof(ndVector)), 1.0e-6f, &hull[0].m_x, 64);
		}
		return shape;
	};

	if (collisions.GetCount() == 1)
	{
		const nd::TiXmlNode* const node = collisions[0];
		collision.SetShape(GetCollisionShape(node));
		ndMatrix matrix(ImportOrigin(node));
		collision.SetLocalMatrix(collision.GetLocalMatrix() * matrix);
	}
	else
	{
		ndAssert(0);
	}

	const nd::TiXmlNode* const inertialNode = linkNode->FirstChild("inertial");
	const nd::TiXmlElement* const massNode = (nd::TiXmlElement*)inertialNode->FirstChild("mass");
	const nd::TiXmlElement* const inertiaNode = (nd::TiXmlElement*)inertialNode->FirstChild("inertia");

	ndFloat64 xx;
	ndFloat64 xy;
	ndFloat64 xz;
	ndFloat64 yy;
	ndFloat64 yz;
	ndFloat64 zz;
	ndFloat64 mass;

	massNode->Attribute("value", &mass);
	inertiaNode->Attribute("ixx", &xx);
	inertiaNode->Attribute("ixy", &xy);
	inertiaNode->Attribute("ixz", &xz);
	inertiaNode->Attribute("iyy", &yy);
	inertiaNode->Attribute("iyz", &yz);
	inertiaNode->Attribute("izz", &zz);

	ndMatrix inertiaMatrix(ndGetIdentityMatrix());
	inertiaMatrix[0][0] = ndFloat32(xx);
	inertiaMatrix[1][1] = ndFloat32(yy);
	inertiaMatrix[2][2] = ndFloat32(zz);

	inertiaMatrix[0][1] = ndFloat32(xy);
	inertiaMatrix[1][0] = ndFloat32(xy);

	inertiaMatrix[0][2] = ndFloat32(xz);
	inertiaMatrix[2][0] = ndFloat32(xz);

	inertiaMatrix[1][2] = ndFloat32(yz);
	inertiaMatrix[2][1] = ndFloat32(yz);

	// create a visual mesh
	ndMeshEffect* meshEffect = new ndMeshEffect;
	for (ndInt32 i = 0; i < ndInt32 (m_materials.GetCount()); ++i)
	{
		ndMeshEffect::ndMaterial meshMaterial;
		meshMaterial.m_diffuse = m_materials[i].m_color;
		//meshMaterial.m_ambient = materials[i].m_color;
		strcpy(meshMaterial.m_textureName, "wood_0.png");
		meshEffect->GetMaterials().PushBack(meshMaterial);
	}
	auto AddMeshShape = [this, meshEffect](const nd::TiXmlNode* const node)
	{
		const nd::TiXmlNode* const geometryNode = node->FirstChild("geometry");
		const nd::TiXmlElement* const shapeNode = (nd::TiXmlElement*)geometryNode->FirstChild();
		const char* const name = shapeNode->Value();

		ndShape* shape = nullptr;
		ndMatrix m_localMatrix(ndGetIdentityMatrix());
		if (strcmp(name, "sphere") == 0)
		{
			ndFloat64 radius;
			shapeNode->Attribute("radius", &radius);
			shape = new ndShapeSphere(ndFloat32(radius));
		}
		else if (strcmp(name, "cylinder") == 0)
		{
			ndFloat64 length;
			ndFloat64 radius;
			shapeNode->Attribute("length", &length);
			shapeNode->Attribute("radius", &radius);
			shape = new ndShapeCylinder(ndFloat32(radius), ndFloat32(radius), ndFloat32(length));
			m_localMatrix = ndYawMatrix(ndPi * ndFloat32(0.5f));
		}
		else if (strcmp(name, "box") == 0)
		{
			ndFloat32 x;
			ndFloat32 y;
			ndFloat32 z;
			const char* const size = shapeNode->Attribute("size");
			ndInt32 ret = 0;
			ret = sscanf(size, "%f %f %f", &x, &y, &z);
			shape = new ndShapeBox(x, y, z);
		}
		else
		{
			const char* const meshPathName = shapeNode->Attribute("filename");
			LoadStlMesh(meshPathName, meshEffect);
			ndMatrix matrix(ImportOrigin(node));
			meshEffect->ApplyTransform(m_localMatrix * matrix);
		}

		if (shape)
		{
			ndShapeInstance collision(shape);
			ndMatrix matrix(ImportOrigin(node));
			collision.SetLocalMatrix(m_localMatrix * matrix);

			class PolygonizeMesh : public ndShapeDebugNotify
			{
				public:
				PolygonizeMesh(ndMeshEffect* const meshEffect, ndInt32 materialIndex)
					:ndShapeDebugNotify()
					,m_meshEffect(meshEffect)
					,m_materialIndex(materialIndex)
				{
				}

				void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceArray, const ndEdgeType* const)
				{
					ndVector normal(ndVector::m_zero);
					for (ndInt32 i = 2; i < vertexCount; ++i)
					{
						ndVector e0(faceArray[i - 1] - faceArray[0]);
						ndVector e1(faceArray[i - 0] - faceArray[0]);
						normal += e0.CrossProduct(e1);
					}
					normal = normal & ndVector::m_triplexMask;
					normal = normal.Normalize();

					m_meshEffect->BeginBuildFace();
					for (ndInt32 i = 0; i < vertexCount; ++i)
					{
						m_meshEffect->AddMaterial(m_materialIndex);
						m_meshEffect->AddPoint(faceArray[i].m_x, faceArray[i].m_y, faceArray[i].m_z);
						m_meshEffect->AddNormal(normal.m_x, normal.m_y, normal.m_z);
						m_meshEffect->AddUV0(0.0f, 0.0f);
					}
					m_meshEffect->EndBuildFace();
				}

				ndMeshEffect* m_meshEffect;
				ndInt32 m_materialIndex;
			};

			ndInt32 materialIndex = 0;
			const nd::TiXmlElement* const materialNode = (nd::TiXmlElement*)node->FirstChild("material");
			if (materialNode)
			{
				const char* const materialName = materialNode->Attribute("name");
				const ndTree<ndInt32, ndString>::ndNode* const materialNameNode = m_materialMap.Find(materialName);
				materialIndex = materialNameNode->GetInfo();
			}

			PolygonizeMesh polygonize(meshEffect, materialIndex);
			collision.DebugShape(ndGetIdentityMatrix(), polygonize);
		}
	};

	meshEffect->BeginBuild();
	ndInt32 count = 0;
	for (const nd::TiXmlNode* node = linkNode->FirstChild("visual"); node; node = node->NextSibling("visual"))
	{
		count++;
		AddMeshShape(node);
	}
	meshEffect->EndBuild();

	if (count == 1)
	{
		const nd::TiXmlNode* node = linkNode->FirstChild("visual");
		const nd::TiXmlNode* const geometryNode = node->FirstChild("geometry");
		const nd::TiXmlElement* const shapeNode = (nd::TiXmlElement*)geometryNode->FirstChild();
		const char* const name = shapeNode->Value();

		ndInt32 materialIndex = 0;
		const nd::TiXmlElement* const materialNode = (nd::TiXmlElement*)node->FirstChild("material");
		if (materialNode)
		{
			const char* const materialName = materialNode->Attribute("name");
			const ndTree<ndInt32, ndString>::ndNode* const materialNameNode = m_materialMap.Find(materialName);
			materialIndex = materialNameNode->GetInfo();
		}

		ndMatrix uvMatrix(ndGetIdentityMatrix());
		if (strcmp(name, "sphere") == 0)
		{
			ndMatrix flipMatrix(ndGetIdentityMatrix());
			flipMatrix[0][0] = ndFloat32(-1.0f);
			ndMatrix aligmentUV(flipMatrix* uvMatrix);
			meshEffect->SphericalMapping(materialIndex, aligmentUV);
		}
		else if (strcmp(name, "cylinder") == 0)
		{
			ndMatrix flipMatrix(ndGetIdentityMatrix());
			flipMatrix[0][0] = ndFloat32(-1.0f);
			ndMatrix aligmentUV(flipMatrix* uvMatrix);
			//meshEffect->SphericalMapping(materialIndex, aligmentUV);
			meshEffect->BoxMapping(materialIndex, materialIndex, materialIndex, aligmentUV);
		}
		else if (strcmp(name, "box") == 0)
		{
			meshEffect->BoxMapping(materialIndex, materialIndex, materialIndex, uvMatrix);
		}
	}

	body->SetCollisionShape(collision);
	body->SetMassMatrix(ndFloat32(mass), collision);
	//body->SetMassMatrix(ndFloat32(mass), inertiaMatrix);
	body->SetNotifyCallback(new ndUrdfBodyNotify(meshEffect));

	return body;
}

ndJointBilateralConstraint* ndUrdfFile::CreateJoint(const nd::TiXmlNode* const jointNode, ndBodyDynamic* const childBody, ndBodyDynamic* const parentBody)
{
	ndJointBilateralConstraint* joint = nullptr;
	const char* const jointType = ((nd::TiXmlElement*)jointNode)->Attribute("type");

	ndMatrix childMatrix(ImportOrigin(jointNode));
	childBody->SetMatrix(childMatrix * parentBody->GetMatrix());
	ndMatrix pivotMatrix(childBody->GetMatrix());
	
	if (strcmp(jointType, "fixed") == 0)
	{
		joint = new ndJointFix6dof(pivotMatrix, childBody, parentBody);
	}
	else if (strcmp(jointType, "continuous") == 0)
	{
		const nd::TiXmlElement* const axisNode = (nd::TiXmlElement*)jointNode->FirstChild("axis");
		if (axisNode)
		{
			ndFloat32 x = ndFloat32(0.0f);
			ndFloat32 y = ndFloat32(0.0f);
			ndFloat32 z = ndFloat32(0.0f);
			const char* const axisPin = axisNode->Attribute("xyz");
			ndInt32 ret = 0;
			ret = sscanf(axisPin, "%f %f %f", &x, &y, &z);

			ndMatrix matrix(ndGramSchmidtMatrix(ndVector (x, y, z, ndFloat32 (0.0f))));
			pivotMatrix = matrix * pivotMatrix;
		}
		joint = new ndJointHinge(pivotMatrix, childBody, parentBody);
	}
	else if (strcmp(jointType, "prismatic") == 0)
	{
		const nd::TiXmlElement* const axisNode = (nd::TiXmlElement*)jointNode->FirstChild("axis");
		if (axisNode)
		{
			ndFloat32 x = ndFloat32(0.0f);
			ndFloat32 y = ndFloat32(0.0f);
			ndFloat32 z = ndFloat32(0.0f);
			const char* const axisPin = axisNode->Attribute("xyz");
			ndInt32 ret = 0;
			ret = sscanf(axisPin, "%f %f %f", &x, &y, &z);

			ndMatrix matrix(ndGramSchmidtMatrix(ndVector(x, y, z, ndFloat32(0.0f))));
			pivotMatrix = matrix * pivotMatrix;
		}
		joint = new ndJointSlider(pivotMatrix, childBody, parentBody);

		const nd::TiXmlElement* const limit = (nd::TiXmlElement*)jointNode->FirstChild("limit");
		if (limit)
		{
			ndFloat64 effort = 0.0f;
			ndFloat64 lower = 0.0f;
			ndFloat64 upper = 0.0f;
			limit->Attribute("effort", &effort);
			limit->Attribute("lower", &lower);
			limit->Attribute("upper", &upper);

			ndJointSlider* const slider = (ndJointSlider*)joint;
			slider->SetLimits(ndFloat32 (lower), ndFloat32 (upper));
			slider->SetLimitState(true);
		}
	}
	else if (strcmp(jointType, "revolute") == 0)
	{
		const nd::TiXmlElement* const axisNode = (nd::TiXmlElement*)jointNode->FirstChild("axis");
		if (axisNode)
		{
			ndFloat32 x = ndFloat32(0.0f);
			ndFloat32 y = ndFloat32(0.0f);
			ndFloat32 z = ndFloat32(0.0f);
			const char* const axisPin = axisNode->Attribute("xyz");
			ndInt32 ret = 0;
			ret = sscanf(axisPin, "%f %f %f", &x, &y, &z);

			ndMatrix matrix(ndGramSchmidtMatrix(ndVector(x, y, z, ndFloat32(0.0f))));
			pivotMatrix = matrix * pivotMatrix;
		}
		joint = new ndJointHinge(pivotMatrix, childBody, parentBody);

		const nd::TiXmlElement* const limit = (nd::TiXmlElement*)jointNode->FirstChild("limit");
		if (limit)
		{
			ndFloat64 effort = 0.0f;
			ndFloat64 lower = 0.0f;
			ndFloat64 upper = 0.0f;
			limit->Attribute("effort", &effort);
			limit->Attribute("lower", &lower);
			limit->Attribute("upper", &upper);

			ndJointHinge* const hinge = (ndJointHinge*)joint;
			hinge->SetLimits(ndFloat32(lower), ndFloat32(upper));
			hinge->SetLimitState(true);
		}
	}
	else
	{
		ndTrace(("FIX ME urdf load jointType %s\n", jointType));
	}

	return joint;
}

ndModelArticulation* ndUrdfFile::Import(const char* const filePathName)
{
	ndAssert(strstr(filePathName, ".urdf"));

	m_path = filePathName;
	const char* ptr = strrchr(filePathName, '/');
	if (!ptr)
	{
		ptr = strrchr(filePathName, '\\');
	}
	m_path = m_path.SubString(0, ndInt32 (ptr - filePathName));

	ndString oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");

	nd::TiXmlDocument doc(filePathName);
	doc.LoadFile();
	if (doc.Error())
	{
		setlocale(LC_ALL, oldloc.GetStr());
		return nullptr;
	}

	m_materials.SetCount(0);
	m_bodyLinks.RemoveAll();
	m_materialMap.RemoveAll();

	const nd::TiXmlElement* const rootNode = doc.RootElement();

	for (const nd::TiXmlNode* node = rootNode->FirstChild("link"); node; node = node->NextSibling("link"))
	{
		const nd::TiXmlElement* const linkNode = (nd::TiXmlElement*)node;
		const char* const name = linkNode->Attribute("name");
		m_bodyLinks.Insert(Hierarchy(node), name);
	}

	for (const nd::TiXmlNode* node = rootNode->FirstChild("joint"); node; node = node->NextSibling("joint"))
	{
		const nd::TiXmlElement* const jointNode = (nd::TiXmlElement*)node;
		const nd::TiXmlElement* const childNode = (nd::TiXmlElement*)jointNode->FirstChild("child");
		const nd::TiXmlElement* const parentNode = (nd::TiXmlElement*)jointNode->FirstChild("parent");

		const char* const childName = childNode->Attribute("link");
		const char* const parentName = parentNode->Attribute("link");

		ndTree<Hierarchy, ndString>::ndNode* const hierarchyChildNode = m_bodyLinks.Find(childName);
		ndTree<Hierarchy, ndString>::ndNode* const hierarchyParentNode = m_bodyLinks.Find(parentName);

		Hierarchy& hierachyChild = hierarchyChildNode->GetInfo();
		Hierarchy& hierachyParent = hierarchyParentNode->GetInfo();

		hierachyChild.m_joint = node;
		hierachyChild.m_parent = &hierachyParent;
		hierachyChild.m_parentLink = hierachyParent.m_link;
	}	

	Hierarchy* root = nullptr;
	ndTree<Hierarchy, ndString>::Iterator iter(m_bodyLinks);
	for (iter.Begin(); iter; iter++)
	{
		Hierarchy& link = iter.GetNode()->GetInfo();
		if (link.m_parentLink == nullptr)
		{
			root = &link;
			break;
		}
	}

	ImportMaterials(rootNode);
	ndBodyDynamic* const rootBody = ImportLink(root->m_link);

	ndModelArticulation* const model = new ndModelArticulation;
	root->m_articulation = model->AddRootBody(rootBody);

	ndFixSizeArray<Hierarchy*, 256> stack;
	stack.PushBack(root);

	while (stack.GetCount())
	{
		Hierarchy* const parent = stack[stack.GetCount() - 1];
		stack.SetCount(stack.GetCount() - 1);

		for (iter.Begin(); iter; iter++)
		{
			Hierarchy& link = iter.GetNode()->GetInfo();
			if (link.m_parentLink == parent->m_link)
			{
				ndBodyDynamic* const childBody = ImportLink(link.m_link);
				ndJointBilateralConstraint* const joint = CreateJoint(link.m_joint, childBody, parent->m_articulation->m_body->GetAsBodyDynamic());
				link.m_articulation = model->AddLimb(parent->m_articulation, joint->GetBody0(), joint);
				stack.PushBack(&link);
			}
		}
	}
	setlocale(LC_ALL, oldloc.GetStr());

	ApplyRotation(model);
	return model;
}


// *******************************************************************************
// 
// *******************************************************************************
void ndUrdfFile::MakeNamesUnique(ndModelArticulation* const model)
{
	ndTree < ndModelArticulation::ndNode*, ndString> filter;

	if (model->GetName() == "")
	{
		model->SetName("robot_model");
	}
	model->GetRoot()->m_name = "base";

	ndString baseName("node_");
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

void ndUrdfFile::ExportMaterials(nd::TiXmlElement* const rootNode, const Surrogate* const)
{
	nd::TiXmlElement* const material = new nd::TiXmlElement("material");
	rootNode->LinkEndChild(material);
	material->SetAttribute("name", "material_0");

	nd::TiXmlElement* const color = new nd::TiXmlElement("color");
	material->LinkEndChild(color);
	color->SetAttribute("rgba", "0.5 0.6 0.2 1.0");
}

void ndUrdfFile::ExportOrigin(nd::TiXmlElement* const parentNode, const ndMatrix& pose)
{
	char buffer[256];
	nd::TiXmlElement* const origin = new nd::TiXmlElement("origin");
	parentNode->LinkEndChild(origin);

	ndVector euler1;
	ndVector euler(pose.CalcPitchYawRoll(euler1));
	//ret = sscanf(rpy, "%f %f %f", &pitch, &yaw, &roll);
	sprintf(buffer, "%g %g %g", euler.m_x, euler.m_y, euler.m_z);
	origin->SetAttribute("rpy", buffer);

	sprintf(buffer, "%g %g %g", pose.m_posit.m_x, pose.m_posit.m_y, pose.m_posit.m_z);
	origin->SetAttribute("xyz", buffer);
}

void ndUrdfFile::ExportVisual(nd::TiXmlElement* const linkNode, const Surrogate* const surroratelink)
{
	char buffer[256];
	
	nd::TiXmlElement* const visualNode = new nd::TiXmlElement("visual");
	linkNode->LinkEndChild(visualNode);
	const ndModelArticulation::ndNode* const link = surroratelink->m_articulation;
	
	// add position and orientation
	const ndBodyKinematic* const body = link->m_body->GetAsBodyKinematic();
	const ndShapeInstance& collision = body->GetCollisionShape();
	
	// add geometry node
	nd::TiXmlElement* const geometry = new nd::TiXmlElement("geometry");
	visualNode->LinkEndChild(geometry);
	const ndShape* const collisionShape = collision.GetShape();
	const char* const className = collisionShape->ClassName();

	ndMatrix aligment(ndGetIdentityMatrix());
	if (!strcmp(className, "ndShapeBox"))
	{
		ndShapeInfo info(collisionShape->GetShapeInfo());
		sprintf(buffer, "%g %g %g", info.m_box.m_x, info.m_box.m_y, info.m_box.m_z);
	
		nd::TiXmlElement* const shape = new nd::TiXmlElement("box");
		geometry->LinkEndChild(shape);
		shape->SetAttribute("size", buffer);
	}
	else if (!strcmp(className, "ndShapeCapsule"))
	{
		ndAssert(0);
		ndShapeInfo info(collisionShape->GetShapeInfo());
	
		nd::TiXmlElement* const shape = new nd::TiXmlElement("cylinder");
		geometry->LinkEndChild(shape);
	
		sprintf(buffer, "%g", info.m_capsule.m_height + info.m_capsule.m_radio0 * 2.0f);
		shape->SetAttribute("length", buffer);
	
		sprintf(buffer, "%g", info.m_capsule.m_radio0);
		shape->SetAttribute("radius", buffer);
		aligment = ndYawMatrix(-ndPi * ndFloat32(0.5f));
	}
	else if (!strcmp(className, "ndShapeCylinder"))
	{
		ndShapeInfo info(collisionShape->GetShapeInfo());
	
		nd::TiXmlElement* const shape = new nd::TiXmlElement("cylinder");
		geometry->LinkEndChild(shape);
	
		sprintf(buffer, "%g", info.m_cylinder.m_height);
		shape->SetAttribute("length", buffer);
	
		sprintf(buffer, "%g", info.m_cylinder.m_radio0);
		shape->SetAttribute("radius", buffer);
		aligment = ndYawMatrix(-ndPi * ndFloat32(0.5f));
	}
	else if (!strcmp(className, "ndShapeSphere"))
	{
		ndShapeInfo info(collisionShape->GetShapeInfo());
	
		nd::TiXmlElement* const shape = new nd::TiXmlElement("sphere");
		geometry->LinkEndChild(shape);
	
		sprintf(buffer, "%g", info.m_sphere.m_radius);
		shape->SetAttribute("radius", buffer);
	}
	else
	{
		ndAssert(0);
	}

	ndMatrix matrix(aligment * surroratelink->m_shapeMatrixMatrix);
	ExportOrigin(visualNode, matrix);

	// add material node
	nd::TiXmlElement* const material = new nd::TiXmlElement("material");
	visualNode->LinkEndChild(material);
	material->SetAttribute("name", "material_0");
}

void ndUrdfFile::ExportCollision(nd::TiXmlElement* const linkNode, const Surrogate* const surroratelink)
{
	char buffer[256];
	const ndModelArticulation::ndNode* const link = surroratelink->m_articulation;
	const ndBodyKinematic* const body = link->m_body->GetAsBodyKinematic();
	
	const ndShapeInstance& collision = body->GetCollisionShape();
	const ndShape* const collisionShape = collision.GetShape();
	
	nd::TiXmlElement* const collisionNode = new nd::TiXmlElement("collision");
	linkNode->LinkEndChild(collisionNode);
	
	nd::TiXmlElement* const geometry = new nd::TiXmlElement("geometry");
	collisionNode->LinkEndChild(geometry);
	
	ndMatrix aligment(ndGetIdentityMatrix());
	const char* const className = collisionShape->ClassName();
	if (!strcmp(className, "ndShapeBox"))
	{
		ndShapeInfo info(collisionShape->GetShapeInfo());
		sprintf(buffer, "%g %g %g", info.m_box.m_x, info.m_box.m_y, info.m_box.m_z);
	
		nd::TiXmlElement* const shape = new nd::TiXmlElement("box");
		geometry->LinkEndChild(shape);
		shape->SetAttribute("size", buffer);
	}
	else if (!strcmp(className, "ndShapeCapsule"))
	{
		ndAssert(0);
		ndShapeInfo info(collisionShape->GetShapeInfo());
	
		nd::TiXmlElement* const shape = new nd::TiXmlElement("cylinder");
		geometry->LinkEndChild(shape);
	
		sprintf(buffer, "%g", info.m_capsule.m_height + info.m_capsule.m_radio0 * 2.0f);
		shape->SetAttribute("length", buffer);
	
		sprintf(buffer, "%g", info.m_capsule.m_radio0);
		shape->SetAttribute("radius", buffer);
		aligment = ndYawMatrix(-ndPi * ndFloat32(0.5f));
	}
	else if (!strcmp(className, "ndShapeCylinder"))
	{
		ndShapeInfo info(collisionShape->GetShapeInfo());
	
		nd::TiXmlElement* const shape = new nd::TiXmlElement("cylinder");
		geometry->LinkEndChild(shape);
	
		sprintf(buffer, "%g", info.m_cylinder.m_height);
		shape->SetAttribute("length", buffer);
	
		sprintf(buffer, "%g", info.m_cylinder.m_radio0);
		shape->SetAttribute("radius", buffer);
		aligment = ndYawMatrix(-ndPi * ndFloat32(0.5f));
	}
	else if (!strcmp(className, "ndShapeSphere"))
	{
		ndShapeInfo info(collisionShape->GetShapeInfo());
	
		nd::TiXmlElement* const shape = new nd::TiXmlElement("sphere");
		geometry->LinkEndChild(shape);
	
		sprintf(buffer, "%g", info.m_sphere.m_radius);
		shape->SetAttribute("radius", buffer);
	}
	else
	{
		ndAssert(0);
	}

	ndMatrix matrix(aligment * surroratelink->m_shapeMatrixMatrix);
	ExportOrigin(collisionNode, matrix);
}

void ndUrdfFile::ExportInertia(nd::TiXmlElement* const linkNode, const Surrogate* const surroratelink)
{
	char buffer[256];
	nd::TiXmlElement* const inertial = new nd::TiXmlElement("inertial");
	linkNode->LinkEndChild(inertial);
	const ndModelArticulation::ndNode* const link = surroratelink->m_articulation;
	
	const ndBodyKinematic* const body = link->m_body->GetAsBodyKinematic();
	//ndMatrix matrix(body->GetMatrix());
	//ndVector com(matrix.TransformVector(body->GetCentreOfMass()));
	ndMatrix comMatrix(ndGetIdentityMatrix());
	comMatrix.m_posit = surroratelink->m_com;
	ExportOrigin(inertial, comMatrix);
	
	nd::TiXmlElement* const mass = new nd::TiXmlElement("mass");
	inertial->LinkEndChild(mass);
	ndVector massMatrix(body->GetMassMatrix());
	sprintf(buffer, "%g", massMatrix.m_w);
	mass->SetAttribute("value", buffer);
	
	nd::TiXmlElement* const inertia = new nd::TiXmlElement("inertia");
	inertial->LinkEndChild(inertia);
	
	//ndMatrix inertiaMatrix(body->CalculateInertiaMatrix());
	//inertiaMatrix = rotation.OrthoInverse() * inertiaMatrix * rotation;
	ndMatrix inertiaMatrix(surroratelink->m_bodyInertia);
	
	sprintf(buffer, "%g", inertiaMatrix[0][0]);
	inertia->SetAttribute("xx", buffer);
	
	sprintf(buffer, "%g", inertiaMatrix[0][1]);
	inertia->SetAttribute("xy", buffer);
	
	sprintf(buffer, "%g", inertiaMatrix[0][2]);
	inertia->SetAttribute("xz", buffer);
	
	sprintf(buffer, "%g", inertiaMatrix[1][1]);
	inertia->SetAttribute("yy", buffer);
	
	sprintf(buffer, "%g", inertiaMatrix[1][2]);
	inertia->SetAttribute("yz", buffer);
	
	sprintf(buffer, "%g", inertiaMatrix[2][2]);
	inertia->SetAttribute("zz", buffer);
}

void ndUrdfFile::ExportLink(nd::TiXmlElement* const rootNode, const Surrogate* const surroratelink)
{
	char name[256];
	nd::TiXmlElement* const linkNode = new nd::TiXmlElement("link");
	rootNode->LinkEndChild(linkNode);
	const ndModelArticulation::ndNode* const link = surroratelink->m_articulation;
	
	sprintf(name, "%s_link", link->m_name.GetStr());
	linkNode->SetAttribute("name", name);

	ExportVisual(linkNode, surroratelink);
	ExportCollision(linkNode, surroratelink);
	ExportInertia(linkNode, surroratelink);
}

void ndUrdfFile::ExportJoint(nd::TiXmlElement* const rootNode, const Surrogate* const surroratelink)
{
	char buffer[256];
	const ndModelArticulation::ndNode* const link = surroratelink->m_articulation;

	sprintf(buffer, "%s_link_to_%s_link", link->m_name.GetStr(), link->GetParent()->m_name.GetStr());

	nd::TiXmlElement* const jointNode = new nd::TiXmlElement("joint");
	rootNode->LinkEndChild(jointNode);
	jointNode->SetAttribute("name", buffer);

	nd::TiXmlElement* const parent = new nd::TiXmlElement("parent");
	jointNode->LinkEndChild(parent);
	sprintf(buffer, "%s_link", link->GetParent()->m_name.GetStr());
	parent->SetAttribute("link", buffer);

	nd::TiXmlElement* const child = new nd::TiXmlElement("child");
	jointNode->LinkEndChild(child);
	sprintf(buffer, "%s_link", link->m_name.GetStr());
	child->SetAttribute("link", buffer);

	jointNode->SetAttribute("type", "fixed");
	ExportOrigin(jointNode, surroratelink->m_jointBodyMatrix1);
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

	MakeNamesUnique(model);
	rootNode->SetAttribute("name", model->GetName().GetStr());

	Surrogate* const surrogate = ApplyInvRotation(model);
	ndAssert(surrogate);

	ExportMaterials(rootNode, surrogate);

	ndFixSizeArray<Surrogate*, 256> stack;
	stack.PushBack(surrogate);

	while (stack.GetCount())
	{
		const Surrogate* const node = stack.Pop();
		ExportLink(rootNode, node);
		if (node->GetParent())
		{
			ExportJoint(rootNode, node);
		}

		for (Surrogate* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			stack.PushBack(child);
		}
	}
	 
	delete surrogate;
	doc->SaveFile(filePathName);
	setlocale(LC_ALL, oldloc.GetStr());
	delete doc;
}

void ndUrdfFile::ApplyRotation(ndModelArticulation* const model)
{
	const ndMatrix rotation(ndPitchMatrix(-ndPi * 0.5f));
	const ndMatrix rotationInv(rotation.OrthoInverse());

	ndArray<ndMatrix> matrix0;
	ndArray<ndMatrix> matrix1;
	ndArray<ndJointBilateralConstraint*> joints;
	ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;

	stack.PushBack(model->GetRoot());
	while (stack.GetCount())
	{
		ndModelArticulation::ndNode* node = stack.Pop();
		ndJointBilateralConstraint* const joint = *node->m_joint;
		if (joint)
		{
			ndMatrix pivotMatrix0;
			ndMatrix pivotMatrix1;
			joint->CalculateGlobalMatrix(pivotMatrix0, pivotMatrix1);

			joints.PushBack(joint);
			matrix0.PushBack(pivotMatrix0 * rotation);
			matrix1.PushBack(pivotMatrix1 * rotation);
		}

		for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			stack.PushBack(child);
		}
	}

	// rotate bodies.
	stack.PushBack(model->GetRoot());
	while (stack.GetCount())
	{
		ndModelArticulation::ndNode* node = stack.Pop();
		ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();

		ndShapeInstance& shape = body->GetCollisionShape();
		ndMatrix shapeMatrix(shape.GetLocalMatrix() * body->GetMatrix() * rotation);

		body->SetCentreOfMass(rotation.RotateVector(body->GetCentreOfMass()));
		ndMatrix bodyMatrix(rotationInv * body->GetMatrix() * rotation);

		body->SetMatrix(bodyMatrix);
		shape.SetLocalMatrix(shapeMatrix * bodyMatrix.OrthoInverse());

		ndUrdfBodyNotify* const notify = (ndUrdfBodyNotify*)body->GetNotifyCallback();
		notify->m_mesh->ApplyTransform(rotation);

		for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			stack.PushBack(child);
		}
	}

	// rotate joints.
	for (ndInt32 i = 0; i < joints.GetCount(); ++i)
	{
		ndBodyKinematic* const body0 = joints[i]->GetBody0();
		ndBodyKinematic* const body1 = joints[i]->GetBody1();
		ndMatrix localMatrix0(matrix0[i] * body0->GetMatrix().OrthoInverse());
		ndMatrix localMatrix1(matrix1[i] * body1->GetMatrix().OrthoInverse());
		joints[i]->SetLocalMatrix0(localMatrix0);
		joints[i]->SetLocalMatrix1(localMatrix1);
	}
}

ndUrdfFile::Surrogate* ndUrdfFile::ApplyInvRotation(ndModelArticulation* const model)
{
	Surrogate* surrogateRoot = nullptr;
	ndFixSizeArray<Surrogate*, 256> surrogateStack;
	ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;

	stack.PushBack(model->GetRoot());
	surrogateStack.PushBack(nullptr);

	while (stack.GetCount())
	{
		Surrogate* const surrogateParent = surrogateStack.Pop();
		ndModelArticulation::ndNode* const node = stack.Pop();

		Surrogate* const surrogateNode = new Surrogate;
		if (surrogateParent)
		{
			surrogateNode->Attach(surrogateParent);
		}
		surrogateNode->m_articulation = node;

		if (surrogateRoot == nullptr)
		{
			surrogateRoot = surrogateNode;
		}

		for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			stack.PushBack(child);
			surrogateStack.PushBack(surrogateNode);
		}
	}

	const ndMatrix rotation(ndPitchMatrix(ndPi * 0.5f));
	const ndMatrix rotationInv(rotation.OrthoInverse());

	surrogateStack.PushBack(surrogateRoot);
	while (surrogateStack.GetCount())
	{
		Surrogate* const node = surrogateStack.Pop();

		ndBodyKinematic* const body = node->m_articulation->m_body->GetAsBodyKinematic();

		ndShapeInstance& shape = body->GetCollisionShape();
		ndMatrix shapeMatrix(shape.GetLocalMatrix() * body->GetMatrix() * rotation);
		node->m_com = rotation.RotateVector(body->GetCentreOfMass());

		node->m_bodyMatrix = rotationInv * body->GetMatrix() * rotation;
		node->m_shapeMatrixMatrix = shapeMatrix * node->m_bodyMatrix.OrthoInverse();

		ndMatrix inertia(body->CalculateInertiaMatrix());
		node->m_bodyInertia = rotationInv * inertia * rotation;

		//ndUrdfBodyNotify* const notify = (ndUrdfBodyNotify*)body->GetNotifyCallback();
		//notify->m_mesh->ApplyTransform(rotation);

		for (Surrogate* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			surrogateStack.PushBack(child);
		}
	}

	surrogateStack.PushBack(surrogateRoot);
	while (surrogateStack.GetCount())
	{
		Surrogate* const node = surrogateStack.Pop();
		if (*node->m_articulation->m_joint)
		{
			ndMatrix matrix0;
			ndMatrix matrix1;
			node->m_articulation->m_joint->CalculateGlobalMatrix(matrix0, matrix1);

			node->m_jointBodyMatrix0 = node->m_articulation->m_joint->GetLocalMatrix0() * rotation;
			node->m_jointBodyMatrix1 = node->m_articulation->m_joint->GetLocalMatrix1() * rotation;
			//node->m_jointBodyMatrix0 = matrix0 * node->m_bodyMatrix.OrthoInverse();
			//node->m_jointBodyMatrix1 = matrix1 * node->GetParent()->m_bodyMatrix.OrthoInverse();
			//node->m_jointBodyMatrix1 = matrix1 * node->GetParent()->m_bodyMatrix.OrthoInverse();
		}

		for (Surrogate* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			surrogateStack.PushBack(child);
		}
	}

	return surrogateRoot;
}