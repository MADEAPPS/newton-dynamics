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
#include "ndMesh.h"
#include "ndUrdfFile.h"

ndUrdfFile::ndUrdfFile()
	:ndClassAlloc()
{
}

ndUrdfFile::~ndUrdfFile()
{
}

void ndUrdfFile::Export(const char* const filePathName, ndModelArticulation* const model)
{
	ndAssert(0);
	//ndAssert(strstr(filePathName, ".urdf"));
	//
	//nd::TiXmlDocument* const doc = new nd::TiXmlDocument("");
	//nd::TiXmlDeclaration* const decl = new nd::TiXmlDeclaration("1.0", "", "");
	//doc->LinkEndChild(decl);
	//ndString oldloc(setlocale(LC_ALL, 0));
	//
	//nd::TiXmlElement* const rootNode = new nd::TiXmlElement("robot");
	//doc->LinkEndChild(rootNode);
	//if (model->GetName() == "")
	//{
	//	rootNode->SetAttribute("name", "robot_model");
	//}
	//else
	//{
	//	rootNode->SetAttribute("name", model->GetName().GetStr());
	//}
	//
	//CheckUniqueNames(model);
	////AddMaterials(rootNode, model);
	//AddLinks(rootNode, model);
	////AddJoints(rootNode, model);
	//
	//doc->SaveFile(filePathName);
	//setlocale(LC_ALL, oldloc.GetStr());
	//delete doc;
}

#if 0
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

//void ndUrdfFile::AddMaterials(nd::TiXmlElement* const rootNode, const ndModelArticulation* const model)
void ndUrdfFile::AddMaterials(nd::TiXmlElement* const rootNode, const ndModelArticulation* const)
{
	nd::TiXmlElement* const material = new nd::TiXmlElement("material");
	rootNode->LinkEndChild(material);
	material->SetAttribute("name", "material_0");

	nd::TiXmlElement* const color = new nd::TiXmlElement("color");
	material->LinkEndChild(color);
	color->SetAttribute("rgba", "0.5 0.6 0.2, 1.0");
}

void ndUrdfFile::AddLinks(nd::TiXmlElement* const rootNode, const ndModelArticulation* const model)
{
	ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;
	stack.PushBack(model->GetRoot());

	while (stack.GetCount())
	{
		ndModelArticulation::ndNode* const node = stack[stack.GetCount() - 1];
		stack.SetCount(stack.GetCount() - 1);

		AddLink(rootNode, node);

		for (ndModelArticulation::ndNode* childNode = node->GetFirstChild(); childNode; childNode = childNode->GetNext())
		{
			stack.PushBack(childNode);
		}
	}
}

void ndUrdfFile::AddLink(nd::TiXmlElement* const rootNode, const ndModelArticulation::ndNode* const link)
{
	nd::TiXmlElement* const linkNode = new nd::TiXmlElement("link");
	rootNode->LinkEndChild(linkNode);

	linkNode->SetAttribute("name", link->m_name.GetStr());

	AddInertia(linkNode, link);
	AddGeometry(linkNode, link);
	AddCollision(linkNode, link);
}

void ndUrdfFile::AddInertia(nd::TiXmlElement* const linkNode, const ndModelArticulation::ndNode* const link)
{
	char buffer[256];
	nd::TiXmlElement* const inertial = new nd::TiXmlElement("inertial");
	linkNode->LinkEndChild(inertial);

	const ndBodyKinematic* const body = link->m_body->GetAsBodyKinematic();
	ndMatrix matrix(body->GetMatrix());
	ndVector com(matrix.TransformVector(body->GetCentreOfMass()));
	AddPose(inertial, matrix);

	nd::TiXmlElement* const mass = new nd::TiXmlElement("mass");
	inertial->LinkEndChild(mass);
	ndVector massMatrix(body->GetMassMatrix());
	sprintf(buffer, "%g", massMatrix.m_w);
	mass->SetAttribute("value", buffer);

	nd::TiXmlElement* const inertia = new nd::TiXmlElement("inertia");
	inertial->LinkEndChild(inertia);

	ndMatrix inertiaMatrix(body->CalculateInertiaMatrix());
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

void ndUrdfFile::AddPose(nd::TiXmlElement* const parentNode, const ndMatrix& pose)
{
	char buffer[256];
	nd::TiXmlElement* const origin = new nd::TiXmlElement("origin");
	parentNode->LinkEndChild(origin);
	sprintf(buffer, "%g %g %g", pose.m_posit.m_x, pose.m_posit.m_y, pose.m_posit.m_z);
	origin->SetAttribute("xyz", buffer);

	ndVector euler1;
	ndVector euler(pose.CalcPitchYawRoll(euler1));
	sprintf(buffer, "%g %g %g", euler.m_z, euler.m_x, euler.m_y);
	origin->SetAttribute("rpy", buffer);
}

void ndUrdfFile::AddGeometry(nd::TiXmlElement* const linkNode, const ndModelArticulation::ndNode* const link)
{
	char buffer[256];

	nd::TiXmlElement* const visualNode = new nd::TiXmlElement("visual");
	linkNode->LinkEndChild(visualNode);

	// add position and orientation
	const ndBodyKinematic* const body = link->m_body->GetAsBodyKinematic();
	const ndShapeInstance& collision = body->GetCollisionShape();
	ndMatrix matrix(collision.GetScaledTransform(body->GetMatrix()));

	AddPose(visualNode, matrix);

	// add geometry node
	nd::TiXmlElement* const geometry = new nd::TiXmlElement("geometry");
	visualNode->LinkEndChild(geometry);
	const ndShape* const collisionShape = collision.GetShape();
	const char* const className = collisionShape->ClassName();
	if (!strcmp(className, "ndShapeBox"))
	{
		ndShapeInfo info (collisionShape->GetShapeInfo());
		sprintf(buffer, "%g %g %g", info.m_box.m_x, info.m_box.m_y, info.m_box.m_z);

		nd::TiXmlElement* const shape = new nd::TiXmlElement("box");
		geometry->LinkEndChild(shape);
		shape->SetAttribute("size", buffer);
	}
	else if (!strcmp(className, "ndShapeCapsule"))
	{
		ndShapeInfo info(collisionShape->GetShapeInfo());

		nd::TiXmlElement* const shape = new nd::TiXmlElement("cylinder");
		geometry->LinkEndChild(shape);

		sprintf(buffer, "%g", info.m_capsule.m_radio0);
		shape->SetAttribute("radius", buffer);

		sprintf(buffer, "%g", info.m_capsule.m_height + info.m_capsule.m_radio0 * 2.0f);
		shape->SetAttribute("length", buffer);
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

	// add material node
	nd::TiXmlElement* const material = new nd::TiXmlElement("material");
	visualNode->LinkEndChild(material);
	material->SetAttribute("name", "material_0");

	nd::TiXmlElement* const color = new nd::TiXmlElement("color");
	material->LinkEndChild(color);
	color->SetAttribute("rgba", "0.5 0.6 0.2, 1.0");
}

void ndUrdfFile::AddCollision(nd::TiXmlElement* const linkNode, const ndModelArticulation::ndNode* const link, const ndShapeInstance& collision)
{
	char buffer[256];
	const ndShape* const collisionShape = collision.GetShape();
	const char* const className = collisionShape->ClassName();

	nd::TiXmlElement* const collisionNode = new nd::TiXmlElement("collision");
	linkNode->LinkEndChild(collisionNode);

	const ndBodyKinematic* const body = link->m_body->GetAsBodyKinematic();
	ndMatrix matrix(collision.GetScaledTransform(body->GetMatrix()));
	AddPose(collisionNode, matrix);

	nd::TiXmlElement* const geometry = new nd::TiXmlElement("geometry");
	collisionNode->LinkEndChild(geometry);

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
		ndShapeInfo info(collisionShape->GetShapeInfo());
		
		nd::TiXmlElement* const shape = new nd::TiXmlElement("cylinder");
		geometry->LinkEndChild(shape);
		
		sprintf(buffer, "%g", info.m_capsule.m_radio0);
		shape->SetAttribute("radius", buffer);
		
		sprintf(buffer, "%g", info.m_capsule.m_height + info.m_capsule.m_radio0 * 2.0f);
		shape->SetAttribute("length", buffer);
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
}

void ndUrdfFile::AddCollision(nd::TiXmlElement* const linkNode, const ndModelArticulation::ndNode* const link)
{
	const ndBodyKinematic* const body = link->m_body->GetAsBodyKinematic();
	const ndShapeInstance& collision = body->GetCollisionShape();

	const ndShape* const collisionShape = collision.GetShape();
	const char* const className = collisionShape->ClassName();

	if (!strcmp(className, "ndShapeCompound"))
	{
		ndAssert(0);
	}
	else
	{
		AddCollision(linkNode, link, collision);
	}
}

void ndUrdfFile::AddJoints(nd::TiXmlElement* const rootNode, const ndModelArticulation* const model)
{
	ndAssert(0);
	ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;
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


		nd::TiXmlElement* const child = new nd::TiXmlElement("child");
		joint->LinkEndChild(child);
		child->SetAttribute("link", node->m_name.GetStr());

		for (ndModelArticulation::ndNode* childNode = node->GetFirstChild(); childNode; childNode = childNode->GetNext())
		{
			stack.PushBack(childNode);
		}
	}
}

void ndUrdfFile::AddJoint(nd::TiXmlElement* const joint, const ndModelArticulation::ndNode* const link)
{
	ndString name(link->GetParent()->m_name + "_" + link->m_name);
	joint->SetAttribute("name", name.GetStr());
	joint->SetAttribute("type", "xxxxxxxx");

	nd::TiXmlElement* const parent = new nd::TiXmlElement("parent");
	joint->LinkEndChild(parent);
	parent->SetAttribute("link", link->GetParent()->m_name.GetStr());
}

#endif

// *******************************************************************************
// 
// *******************************************************************************
void ndUrdfFile::LoadMaterials(const nd::TiXmlNode* const rootNode, ndTree<ndInt32, ndString>& materialMap, ndArray<Material>& materials)
{
	materials.PushBack(Material());
	for (const nd::TiXmlNode* node = rootNode->FirstChild("material"); node; node = node->NextSibling("material"))
	{
		const nd::TiXmlElement* const materialNode = (nd::TiXmlElement*)node;
		const nd::TiXmlElement* const color = (nd::TiXmlElement*)node->FirstChild("color");
		if (color)
		{
			Material material;

			const char* const name = materialNode->Attribute("name");
			materialMap.Insert(ndInt32(materials.GetCount()), name);

			const char* const rgba = color->Attribute("rgba");

			ndFloat32 r;
			ndFloat32 g;
			ndFloat32 b;
			ndFloat32 a;
			sscanf(rgba, "%f %f %f", &r, &g, &b, &a);
			material.m_color.m_x = r;
			material.m_color.m_y = g;
			material.m_color.m_z = b;
			material.m_color.m_w = a;
			materials.PushBack(material);
		}
	}
}


// *******************************************************************************
// 
// *******************************************************************************
ndMatrix ndUrdfFile::GetMatrix(const nd::TiXmlNode* const parentNode) const
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
			sscanf(xyz, "%f %f %f", &x, &y, &z);
		}
		const char* const rpy = origin->Attribute("rpy");
		if (rpy)
		{
			sscanf(rpy, "%f %f %f", &pitch, &yaw, &roll);
		}

		matrix = ndPitchMatrix(pitch) * ndYawMatrix(yaw) * ndRollMatrix(roll);
		matrix.m_posit.m_x = x;
		matrix.m_posit.m_y = y;
		matrix.m_posit.m_z = z;
	}
	return matrix;
}

ndBodyDynamic* ndUrdfFile::CreateBody(const nd::TiXmlNode* const linkNode, const ndTree<ndInt32, ndString>& materialMap, const ndArray<Material>& materials)
{
	ndBodyDynamic* const body = new ndBodyDynamic();

	ndArray<const nd::TiXmlNode*> collisions;
	for (const nd::TiXmlNode* node = linkNode->FirstChild("collision"); node; node = node->NextSibling("collision"))
	{
		collisions.PushBack(node);
	}

	ndShapeInstance collision(new ndShapeNull());
	auto GetCollisionShape = [&collision](const nd::TiXmlNode* const node)
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
			sscanf(size, "%f %f %f", &x, &y, &z);
			shape = new ndShapeBox(x, y, z);
		}
		else
		{
			ndTrace(("FIX ME urdf load collision %s\n", name));
			shape = new ndShapeNull();
		}
		return shape;
	};

	if (collisions.GetCount() == 1)
	{
		const nd::TiXmlNode* const node = collisions[0];
		collision.SetShape(GetCollisionShape(node));
		ndMatrix matrix(GetMatrix(node));
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
	for (ndInt32 i = 0; i < ndInt32 (materials.GetCount()); ++i)
	{
		ndMeshEffect::ndMaterial meshMaterial;
		meshMaterial.m_diffuse = materials[i].m_color;
		//meshMaterial.m_ambient = materials[i].m_color;
		meshEffect->GetMaterials().PushBack(meshMaterial);
	}
	auto AddMeshShape = [this, meshEffect, &materialMap](const nd::TiXmlNode* const node)
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
			sscanf(size, "%f %f %f", &x, &y, &z);
			shape = new ndShapeBox(x, y, z);
		}
		else
		{
			ndTrace(("FIX ME urdf load geometry mesh %s\n", name));
			//shape = new ndShapeNull();
			shape = new ndShapeSphere(0.01f);
		}

		ndShapeInstance collision(shape);
		ndMatrix matrix(GetMatrix(node));
		collision.SetLocalMatrix(m_localMatrix * matrix);

		class PolygonizeMesh: public ndShapeDebugNotify
		{
			public:
			PolygonizeMesh(ndMeshEffect* const meshEffect, ndInt32 materialIndex)
				:ndShapeDebugNotify()
				,m_meshEffect(meshEffect)
				,m_materialIndex(materialIndex)
			{
			}

			void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceArray, const ndEdgeType* const edgeType)
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
			const char* const name = materialNode->Attribute("name");
			const ndTree<ndInt32, ndString>::ndNode* const materialNode = materialMap.Find(name);
			materialIndex = materialNode->GetInfo();
		}

		PolygonizeMesh polygonize(meshEffect, materialIndex);
		collision.DebugShape(ndGetIdentityMatrix(), polygonize);
	};

	meshEffect->BeginBuild();
	for (const nd::TiXmlNode* node = linkNode->FirstChild("visual"); node; node = node->NextSibling("visual"))
	{
		AddMeshShape(node);
	}
	meshEffect->EndBuild();

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
	const nd::TiXmlElement* const childNode = (nd::TiXmlElement*)jointNode->FirstChild("child");
	const nd::TiXmlElement* const parentNode = (nd::TiXmlElement*)jointNode->FirstChild("parent");

	ndMatrix childMatrix(GetMatrix(jointNode));
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
			sscanf(axisPin, "%f %f %f", &x, &y, &z);

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
			sscanf(axisPin, "%f %f %f", &x, &y, &z);

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
			sscanf(axisPin, "%f %f %f", &x, &y, &z);

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

void ndUrdfFile::ApplyRotation(const ndMatrix& rotation, ndModelArticulation* const model)
{
	ndMatrix rotationInv(rotation.OrthoInverse());

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

ndModelArticulation* ndUrdfFile::Import(const char* const filePathName)
{
	ndAssert(strstr(filePathName, ".urdf"));

	ndString oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");

	nd::TiXmlDocument doc(filePathName);
	doc.LoadFile();
	if (doc.Error())
	{
		setlocale(LC_ALL, oldloc.GetStr());
		return nullptr;
	}

	ndArray<Material> materials;
	ndTree<ndInt32, ndString> materialMap;
	ndTree<Hierarchy, ndString> bodyLinks;

	const nd::TiXmlElement* const rootNode = doc.RootElement();

	for (const nd::TiXmlNode* node = rootNode->FirstChild("link"); node; node = node->NextSibling("link"))
	{
		const nd::TiXmlElement* const linkNode = (nd::TiXmlElement*)node;
		const char* const name = linkNode->Attribute("name");
		bodyLinks.Insert(Hierarchy(node), name);
	}

	for (const nd::TiXmlNode* node = rootNode->FirstChild("joint"); node; node = node->NextSibling("joint"))
	{
		const nd::TiXmlElement* const jointNode = (nd::TiXmlElement*)node;
		const nd::TiXmlElement* const childNode = (nd::TiXmlElement*)jointNode->FirstChild("child");
		const nd::TiXmlElement* const parentNode = (nd::TiXmlElement*)jointNode->FirstChild("parent");

		const char* const childName = childNode->Attribute("link");
		const char* const parentName = parentNode->Attribute("link");

		ndTree<Hierarchy, ndString>::ndNode* const hierarchyChildNode = bodyLinks.Find(childName);
		ndTree<Hierarchy, ndString>::ndNode* const hierarchyParentNode = bodyLinks.Find(parentName);

		Hierarchy& hierachyChild = hierarchyChildNode->GetInfo();
		Hierarchy& hierachyParent = hierarchyParentNode->GetInfo();

		hierachyChild.m_joint = node;
		hierachyChild.m_parent = &hierachyParent;
		hierachyChild.m_parentLink = hierachyParent.m_link;
	}	

	Hierarchy* root = nullptr;
	ndTree<Hierarchy, ndString>::Iterator iter(bodyLinks);
	for (iter.Begin(); iter; iter++)
	{
		Hierarchy& link = iter.GetNode()->GetInfo();
		if (link.m_parentLink == nullptr)
		{
			root = &link;
			break;
		}
	}

	LoadMaterials(rootNode, materialMap, materials);
	ndBodyDynamic* const rootBody = CreateBody(root->m_link, materialMap, materials);

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
				ndBodyDynamic* const childBody = CreateBody(link.m_link, materialMap, materials);
				ndJointBilateralConstraint* const joint = CreateJoint(link.m_joint, childBody, parent->m_articulation->m_body->GetAsBodyDynamic());
				link.m_articulation = model->AddLimb(parent->m_articulation, joint->GetBody0(), joint);
				stack.PushBack(&link);
			}
		}
	}
	setlocale(LC_ALL, oldloc.GetStr());

	ndMatrix rotation(ndPitchMatrix(-ndPi * 0.5f));
	ApplyRotation(rotation, model);
	return model;
}
