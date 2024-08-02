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
	//AddMaterials(rootNode, model);
	AddLinks(rootNode, model);
	//AddJoints(rootNode, model);

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

// *******************************************************************************
// 
// *******************************************************************************
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

	const nd::TiXmlElement* const rootNode = doc.RootElement();

	ndTree<Material, ndString> materials;
	ndTree<ndBodyDynamic*, ndString> bodies;
	ndTree<ndJointBilateralConstraint*, ndString> joints;

	LoadMaterials(rootNode, materials);
	LoadLinks(rootNode, materials, bodies);
	LoadJoints(rootNode, bodies, joints);
	ndModelArticulation* const model = BuildModel(bodies, joints);

	setlocale(LC_ALL, oldloc.GetStr());
	return model;
}

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
			//sscanf(rpy, "%f %f %f", &roll, &pitch, &yaw);
			sscanf(rpy, "%f %f %f", &roll, &yaw, &pitch);
		}

		matrix = ndPitchMatrix(pitch) * ndYawMatrix(yaw) * ndRollMatrix(roll);
		matrix.m_posit.m_x = x;
		matrix.m_posit.m_y = y;
		matrix.m_posit.m_z = z;
	}
	return matrix;
}

void ndUrdfFile::LoadMaterials(const nd::TiXmlElement* const rootNode, ndTree<Material, ndString>& materials)
{
	for (const nd::TiXmlNode* node = rootNode->FirstChild("material"); node; node = node->NextSibling("material"))
	{
		Material material;

		const nd::TiXmlElement* const materialNode = (nd::TiXmlElement*)node;
		const char* const name = materialNode->Attribute("name");
		materials.Insert(material, name);
	}
}

void ndUrdfFile::LoadLinks(const nd::TiXmlElement* const rootNode, const ndTree<Material, ndString>& materials, ndTree<ndBodyDynamic*, ndString>& bodyMap)
{
	for (const nd::TiXmlNode* node = rootNode->FirstChild("link"); node; node = node->NextSibling("link"))
	{
		const nd::TiXmlElement* const linkNode = (nd::TiXmlElement*)node;
		const char* const name = linkNode->Attribute("name");

		ndBodyDynamic* const body = CreateBody(linkNode, materials);
		bodyMap.Insert(body, name);
	}
}

void ndUrdfFile::LoadJoints(const nd::TiXmlElement* const rootNode, const ndTree<ndBodyDynamic*, ndString>& bodyMap, ndTree<ndJointBilateralConstraint*, ndString>& jointMap)
{
	for (const nd::TiXmlNode* node = rootNode->FirstChild("joint"); node; node = node->NextSibling("joint"))
	{
		const nd::TiXmlElement* const jointNode = (nd::TiXmlElement*)node;
		const char* const name = jointNode->Attribute("name");
		ndJointBilateralConstraint* const joint = CreateJoint(jointNode, bodyMap);
		jointMap.Insert(joint, name);
	}
}

ndBodyDynamic* ndUrdfFile::CreateBody(const nd::TiXmlElement* const linkNode, const ndTree<Material, ndString>& materials)
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
		collision.SetShape (GetCollisionShape(node));
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

	body->SetCollisionShape(collision);
	body->SetMassMatrix(ndFloat32(mass), inertiaMatrix);

	return body;
}

ndJointBilateralConstraint* ndUrdfFile::CreateJoint(const nd::TiXmlElement* const jointNode, const ndTree<ndBodyDynamic*, ndString>& bodyMap)
{
	const char* const jointType = jointNode->Attribute("type");

	ndJointBilateralConstraint* joint = nullptr;

	const nd::TiXmlElement* const childNode = (nd::TiXmlElement*)jointNode->FirstChild("child");
	const nd::TiXmlElement* const parentNode = (nd::TiXmlElement*)jointNode->FirstChild("parent");
	
	const char* const childName = childNode->Attribute("link");
	const char* const parentName = parentNode->Attribute("link");
	ndBodyDynamic* const childBody = bodyMap.Find(childName)->GetInfo();
	ndBodyDynamic* const parentBody = bodyMap.Find(parentName)->GetInfo();

	ndMatrix childMatrix(GetMatrix(jointNode));
	childBody->SetMatrix(childMatrix);

	ndMatrix pivotMatrix(childMatrix);
	if (strcmp(jointType, "fixed") == 0)
	{
		joint = new ndJointFix6dof(pivotMatrix, childBody, parentBody);
	}
	else if (strcmp(jointType, "continuous") == 0)
	{
		ndFloat32 x = ndFloat32(0.0f);
		ndFloat32 y = ndFloat32(0.0f);
		ndFloat32 z = ndFloat32(0.0f);

		ndFloat32 x_did = ndFloat32(0.0f);
		ndFloat32 y_did = ndFloat32(0.0f);
		ndFloat32 z_did = ndFloat32(0.0f);

		ndAssert(0);
		const nd::TiXmlElement* const axisNode = (nd::TiXmlElement*)jointNode->FirstChild("axis");
		const nd::TiXmlElement* const originNode = (nd::TiXmlElement*)jointNode->FirstChild("origin");
		
		ndMatrix rotationMatrix(ndGetIdentityMatrix());
		//const char* const axisRot = axisNode->Attribute("rpy");
		//const char* const axisPosit = originNode->Attribute("xyz");

		//const char* const originRot = originNode->Attribute("rpy");
		//const char* const originPosit = originNode->Attribute("xyz");

		//sscanf(originPosit, "%f %f %f", &x, &y, &z);
		//sscanf(originRot, "%f %f %f", &x_did, &y_did, &z_did);
		//ndMatrix matrix(ndGramSchmidtMatrix(ndVector (x_did, y_did, z_did, ndFloat32 (0.0f))));
		//matrix.m_posit.m_x = x;
		//matrix.m_posit.m_y = y;
		//matrix.m_posit.m_z = z;

		ndMatrix matrix(rotationMatrix);
		joint = new ndJointHinge(matrix, childBody, parentBody);
	}
	else if (strcmp(jointType, "revolute") == 0)
	{
		ndAssert(0);
		ndFloat32 x = ndFloat32(0.0f);
		ndFloat32 y = ndFloat32(0.0f);
		ndFloat32 z = ndFloat32(0.0f);

		ndFloat32 x_did = ndFloat32(0.0f);
		ndFloat32 y_did = ndFloat32(0.0f);
		ndFloat32 z_did = ndFloat32(0.0f);

		const nd::TiXmlElement* const axisNode = (nd::TiXmlElement*)jointNode->FirstChild("axis");
		const nd::TiXmlElement* const originNode = (nd::TiXmlElement*)jointNode->FirstChild("origin");

		ndMatrix rotationMatrix(ndGetIdentityMatrix());
		//const char* const axisRot = axisNode->Attribute("rpy");
		//const char* const axisPosit = originNode->Attribute("xyz");

		//const char* const originRot = originNode->Attribute("rpy");
		//const char* const originPosit = originNode->Attribute("xyz");

		//sscanf(originPosit, "%f %f %f", &x, &y, &z);
		//sscanf(originRot, "%f %f %f", &x_did, &y_did, &z_did);
		//ndMatrix matrix(ndGramSchmidtMatrix(ndVector (x_did, y_did, z_did, ndFloat32 (0.0f))));
		//matrix.m_posit.m_x = x;
		//matrix.m_posit.m_y = y;
		//matrix.m_posit.m_z = z;

		ndMatrix matrix(rotationMatrix);
		joint = new ndJointHinge(matrix, childBody, parentBody);
	}

	else if (strcmp(jointType, "prismatic") == 0)
	{
		ndAssert(0);
		const nd::TiXmlElement* const limitNode = (nd::TiXmlElement*)jointNode->FirstChild("limit");
		const nd::TiXmlElement* const originNode = (nd::TiXmlElement*)jointNode->FirstChild("origin");

		ndMatrix matrix(ndGetIdentityMatrix());
		joint = new ndJointSlider (matrix, childBody, parentBody);
	}
	else
	{
		ndTrace(("FIX ME urdf load jointType %s\n", jointType));
	}

	return joint;
}

ndModelArticulation* ndUrdfFile::BuildModel(const ndTree<ndBodyDynamic*, ndString>& bodyMap, ndTree<ndJointBilateralConstraint*, ndString>& joints)
{
	ndTree<ndInt32, ndBodyDynamic*> parentBody;
	ndTree<ndBodyDynamic*, ndString>::Iterator bodyIter(bodyMap);
	for (bodyIter.Begin(); bodyIter; bodyIter++)
	{
		ndBodyDynamic* const body = bodyIter.GetNode()->GetInfo();
		parentBody.Insert(0, body);
	}

	ndTree<ndJointBilateralConstraint*, ndString>::Iterator jointIter(joints);
	for (jointIter.Begin(); jointIter; jointIter++)
	{
		ndJointBilateralConstraint* const joint = jointIter.GetNode()->GetInfo();
		//ndTrace(("%d %d\n", joint->GetBody0()->GetId(), joint->GetBody1()->GetId()));
		ndTree<ndInt32, ndBodyDynamic*>::ndNode* bodyNode = parentBody.Find(joint->GetBody0()->GetAsBodyDynamic());
		bodyNode->GetInfo() = 1;
	}

	ndBodyDynamic* root = nullptr;
	ndTree<ndInt32, ndBodyDynamic*>::Iterator parentBodyIter(parentBody);
	for (parentBodyIter.Begin(); parentBodyIter; parentBodyIter++)
	{
		ndTree<ndInt32, ndBodyDynamic*>::ndNode* const bodyNode = parentBodyIter.GetNode();
		if (bodyNode->GetInfo() == 0)
		{
			root = bodyNode->GetKey();
			break;
		}
	}
	ndAssert(root);

	ndModelArticulation* const model = new ndModelArticulation;
	ndModelArticulation::ndNode* const node = model->AddRootBody(root);

	ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;
	stack.PushBack(node);
	while (stack.GetCount())
	{
		ndModelArticulation::ndNode* const parentNode = stack[stack.GetCount() - 1];
		stack.SetCount(stack.GetCount() - 1);

		ndBodyDynamic* const parentBody = parentNode->m_body->GetAsBodyDynamic();
		for (jointIter.Begin(); jointIter; jointIter++)
		{
			ndJointBilateralConstraint* const joint = jointIter.GetNode()->GetInfo();
			if (joint->GetBody1() == parentBody)
			{
				ndModelArticulation::ndNode* const limb = model->AddLimb(parentNode, joint->GetBody0(), joint);
				//ndTrace(("%d %d\n", joint->GetBody0()->GetId(), joint->GetBody1()->GetId()));
				stack.PushBack(limb);
			}
		}
	}

	return model;
}