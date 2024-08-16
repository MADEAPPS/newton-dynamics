/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndCoreStdafx.h"
#include "ndTinyXmlGlue.h"

static char* FloatToString(char* const buffer, ndInt32 size, ndFloat32 value)
{
	snprintf(buffer, size_t(size), "%g", value);
	char* ptr = buffer + strlen(buffer);
	*ptr = ' ';
	ptr++;
	*ptr = 0;
	return ptr;
}

static void CleanWhiteSpace(const char* const value)
{
	size_t size = strlen(value) - 1;
	if (value[size] == ' ')
	{
		char* ptr = (char*)value;
		ptr[size] = 0;
	}
}

#if 0

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const char* const value)
{
	xmlSaveParam(rootNode, name, "string", value);
}

//void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, ndInt32 count, const ndVector* const array)

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndFloat32>& array)
{
	char* const buffer = ndAlloca(char, array.GetCount() * 12 + 256);
	char* ptr = buffer;
	for (ndInt32 i = 0; i < array.GetCount(); ++i)
	{
		ptr = FloatToString(ptr, array[i]);
	}
	CleanWhiteSpace(buffer);

	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);

	node->SetAttribute("count", array.GetCount());
	node->SetAttribute("floatArray", buffer);
}

#ifdef D_NEWTON_USE_DOUBLE
void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndReal>& array)
{
	char* const buffer = ndAlloca(char, array.GetCount() * 12 + 256);
	char* ptr = buffer;
	for (ndInt32 i = 0; i < array.GetCount(); ++i)
	{
		ptr = FloatToString(ptr, array[i]);
	}
	CleanWhiteSpace(buffer);

	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);

	node->SetAttribute("count", array.GetCount());
	node->SetAttribute("realArray", buffer);
}
#endif



#ifdef D_NEWTON_USE_DOUBLE
void xmlGetFloatArray(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndReal>& array)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*) rootNode->FirstChild(name);
	ndAssert(element);
	ndInt32 count;
	element->Attribute("count", &count);
	array.Resize(count);
	array.SetCount(count);

	const char* const data = element->Attribute("floatArray");

	size_t start = 0;
	for (ndInt32 i = 0; i < count; ++i)
	{
		char x[64];
		sscanf(&data[start], "%[^ ]", x);
		start += strlen(x) + 1;

		ndFloat64 fx;
		sscanf(x, "%lf", &fx);
		array[i] = ndReal(fx);
	}
}
#endif

void xmlGetFloatArray(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndFloat32>& array)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);
	ndInt32 count;
	element->Attribute("count", &count);
	array.Resize(count);
	array.SetCount(count);

	const char* const data = element->Attribute("floatArray");

	size_t start = 0;
	for (ndInt32 i = 0; i < count; ++i)
	{
		char x[64];
		sscanf(&data[start], "%[^ ]", x);
		start += strlen(x) + 1;

		ndFloat64 fx;
		sscanf(x, "%lf", &fx);
		array[i] = ndFloat32(fx);
	}
}

const nd::TiXmlNode* xmlFind(const nd::TiXmlNode* const rootNode, const char* const name)
{
	//for (const nd::TiXmlElement* node = (nd::TiXmlElement*) rootNode->FirstChild(name); node; node = (nd::TiXmlElement*) node->NextSibling())
	//{
	//	const char* const text = node->GetText();
	//	if (!strcmp(text, name))
	//	{
	//		return node;
	//	}
	//}
	return rootNode->FirstChild(name);
}

#endif


static ndInt32 g_classId = 0;
void xmlResetClassId()
{
	g_classId = 0;
}

nd::TiXmlElement* xmlCreateClassNode(nd::TiXmlElement* const parent, const char* const className, const char* const name)
{
	nd::TiXmlElement* const node = new nd::TiXmlElement(className);
	parent->LinkEndChild(node);
	node->SetAttribute("className", name);
	node->SetAttribute("nodeId", g_classId);
	g_classId++;
	return node;
}

static void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const char* const type, const char* const value)
{
	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);
	CleanWhiteSpace(value);
	node->SetAttribute(type, value);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const char* const value)
{
	xmlSaveParam(rootNode, name, "string", value);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndMatrix& value)
{
	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);

	ndVector euler1;
	ndVector euler0 (value.CalcPitchYawRoll(euler1));
	euler0 = euler0.Scale(ndRadToDegree);

	xmlSaveParam(node, "posit", value.m_posit);
	xmlSaveParam(node, "angles", euler0);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, ndInt32 value)
{
	char buffer[1024];
	snprintf(buffer, sizeof (buffer), "%d", value);
	xmlSaveParam(rootNode, name, "int32", buffer);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, ndInt64 value)
{
	char buffer[1024];
	long long x = value;
	snprintf(buffer, sizeof (buffer), "%llu", x);
	xmlSaveParam(rootNode, name, "int64", buffer);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, ndFloat32 value)
{
	char buffer[1024];
	FloatToString(buffer, sizeof (buffer), value);
	xmlSaveParam(rootNode, name, "float", buffer);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndVector& value)
{
	char buffer[1024];
	char* ptr0 = FloatToString(buffer, sizeof (buffer), value.m_x);
	char* ptr1 = FloatToString(ptr0, sizeof (buffer) - 256, value.m_y);
	FloatToString(ptr1, sizeof(buffer) - 256, value.m_z);
	xmlSaveParam(rootNode, name, "float3", buffer);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndInt32>& array)
{
	char* const buffer = ndAlloca(char, array.GetCount() * 24 + 256);
	char* ptr = buffer;
	for (ndInt32 i = 0; i < array.GetCount(); ++i)
	{
		snprintf(ptr, 256, "%d ", array[i]);
		ptr += strlen(ptr);
	}
	CleanWhiteSpace(buffer);

	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);

	node->SetAttribute("count", ndInt32(array.GetCount()));
	node->SetAttribute("intArray", buffer);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndInt64>& array)
{
	char* const buffer = ndAlloca(char, array.GetCount() * 24 + 256);
	char* ptr = buffer;
	for (ndInt32 i = 0; i < array.GetCount(); ++i)
	{
		long long int value = array[i];
		snprintf(ptr, 256, "%lld ", value);
		ptr += strlen(ptr);
	}
	CleanWhiteSpace(buffer);

	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);

	node->SetAttribute("count", ndInt32(array.GetCount()));
	node->SetAttribute("int64Array", buffer);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndVector>& array)
{
	char* const buffer = ndAlloca(char, array.GetCount() * 4 * 12 + 256);
	char* ptr = buffer;
	for (ndInt32 i = 0; i < array.GetCount(); ++i)
	{
		for (ndInt32 j = 0; j < 3; ++j)
		{
			ptr = FloatToString(ptr, 256, array[i][j]);
		}
	}
	CleanWhiteSpace(buffer);

	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);

	node->SetAttribute("count", ndInt32(array.GetCount()));
	node->SetAttribute("float3Array", buffer);
}

ndInt32 xmlGetNodeId(const nd::TiXmlNode* const rootNode)
{
	ndInt32 id;
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode;
	element->Attribute("nodeId", &id);
	return id;
}

ndInt32 xmlGetInt(const nd::TiXmlNode* const rootNode, const char* const name)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);
	ndInt32 value;
	element->Attribute("int32", &value);
	return value;
}

ndInt64 xmlGetInt64(const nd::TiXmlNode* const rootNode, const char* const name)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);
	const char* const data = element->Attribute("int64");

	long long int value;
	sscanf(data, "%lld", &value);
	return ndInt64(value);
}

const char* xmlGetString(const nd::TiXmlNode* const rootNode, const char* const name)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);
	return element->Attribute("string");
}

ndFloat32 xmlGetFloat(const nd::TiXmlNode* const rootNode, const char* const name)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);
	ndFloat64 value;
	element->Attribute("float", &value);
	return ndFloat32(value);
}

ndVector xmlGetVector3(const nd::TiXmlNode* const rootNode, const char* const name)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);

	const char* const positData = element->Attribute("float3");

	ndFloat64 fx;
	ndFloat64 fy;
	ndFloat64 fz;
	sscanf(positData, "%lf %lf %lf", &fx, &fy, &fz);

	ndVector posit(ndVector::m_zero);
	posit.m_x = ndFloat32(fx);
	posit.m_y = ndFloat32(fy);
	posit.m_z = ndFloat32(fz);
	return posit;
}

ndMatrix xmlGetMatrix(const nd::TiXmlNode* const rootNode, const char* const name)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);

	ndVector posit(xmlGetVector3(element, "posit"));
	ndVector euler(xmlGetVector3(element, "angles"));
	ndMatrix matrix(ndPitchMatrix(euler.m_x * ndDegreeToRad) * ndYawMatrix(euler.m_y * ndDegreeToRad) * ndRollMatrix(euler.m_z * ndDegreeToRad));

	matrix.m_posit = posit;
	matrix.m_posit.m_w = ndFloat32(1.0f);
	return matrix;
}

void xmlGetInt64(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndInt64>& array)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);
	ndInt32 count;
	element->Attribute("count", &count);
	array.Resize(count);
	array.SetCount(count);

	const char* const data = element->Attribute("int64Array");

	size_t start = 0;
	ndVector point(ndVector::m_zero);
	for (ndInt32 i = 0; i < count; ++i)
	{
		char x[64];
		sscanf(&data[start], "%[^ ]", x);
		start += strlen(x) + 1;

		long long int fx;
		sscanf(x, "%lld", &fx);
		array[i] = ndInt64 (fx);
	}
}

void xmlGetFloatArray3(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndVector>& array)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);
	ndInt32 count;
	element->Attribute("count", &count);
	array.Resize(count);
	array.SetCount(count);

	//const char* const data = element->Attribute("float4Array");
	const char* const data = element->Attribute("float3Array");

	size_t start = 0;
	ndVector point(ndVector::m_zero);
	for (ndInt32 i = 0; i < count; ++i)
	{
		char x[64];
		char y[64];
		char z[64];
		//char w[64];
		//sscanf(&data[start], "%[^ ] %[^ ] %[^ ] %[^ ]", x, y, z, w);
		//start += strlen(x) + strlen(y) + strlen(z) + strlen(w) + 4;

		sscanf(&data[start], "%[^ ] %[^ ] %[^ ]", x, y, z);
		start += strlen(x) + strlen(y) + strlen(z) + 3;

		ndFloat64 fx;
		ndFloat64 fy;
		ndFloat64 fz;
		//ndFloat64 fw;

		sscanf(x, "%lf", &fx);
		sscanf(y, "%lf", &fy);
		sscanf(z, "%lf", &fz);
		//sscanf(w, "%lf", &fw);

		point.m_x = ndFloat32(fx);
		point.m_y = ndFloat32(fy);
		point.m_z = ndFloat32(fz);
		//point.m_w = ndFloat32(fw);
		point.m_w = ndFloat32(0.0f);

		array[i] = point;
	}
}
