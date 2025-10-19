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

void xmlSaveAtribute(nd::TiXmlElement* const rootNode, const char* const name, ndInt32 value)
{
	rootNode->SetAttribute(name, value);
}

void xmlSaveAtribute(nd::TiXmlElement* const rootNode, const char* const name, ndReal value)
{
	rootNode->SetDoubleAttribute(name, value);
}

void xmlSaveAtribute(nd::TiXmlElement* const rootNode, const char* const name, const char* const value)
{
	rootNode->SetAttribute(name, value);
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

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndReal>& array)
{
	ndStack<char> buffer(ndInt32(array.GetCount() * 4 * 12 + 256));
	char* ptr = &buffer[0];
	for (ndInt32 i = 0; i < array.GetCount(); ++i)
	{
		ptr = FloatToString(ptr, 256, array[i]);
	}
	CleanWhiteSpace(&buffer[0]);

	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);

	node->SetAttribute("count", ndInt32(array.GetCount()));
	node->SetAttribute("floatArray", &buffer[0]);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndInt32>& array)
{
	ndStack<char> buffer(ndInt32 (array.GetCount() * 24 + 256));
	char* ptr = &buffer[0];
	for (ndInt32 i = 0; i < array.GetCount(); ++i)
	{
		snprintf(ptr, 256, "%d ", array[i]);
		ptr += strlen(ptr);
	}
	CleanWhiteSpace(&buffer[0]);

	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);

	node->SetAttribute("count", ndInt32(array.GetCount()));
	node->SetAttribute("intArray", &buffer[0]);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndInt64>& array)
{
	ndStack<char> buffer(ndInt32(array.GetCount() * 24 + 256));
	char* ptr = &buffer[0];
	for (ndInt32 i = 0; i < array.GetCount(); ++i)
	{
		long long int value = array[i];
		snprintf(ptr, 256, "%lld ", value);
		ptr += strlen(ptr);
	}
	CleanWhiteSpace(&buffer[0]);

	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);

	node->SetAttribute("count", ndInt32(array.GetCount()));
	node->SetAttribute("int64Array", &buffer[0]);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndVector>& array)
{
	ndStack<char> buffer(ndInt32(array.GetCount() * 4 * 12 + 256));
	char* ptr = &buffer[0];
	for (ndInt32 i = 0; i < array.GetCount(); ++i)
	{
		for (ndInt32 j = 0; j < 3; ++j)
		{
			ptr = FloatToString(ptr, 256, array[i][j]);
		}
	}
	CleanWhiteSpace(&buffer[0]);

	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);

	node->SetAttribute("count", ndInt32(array.GetCount()));
	node->SetAttribute("float3Array", &buffer[0]);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndTriplexReal>& array)
{
	ndStack<char> buffer(ndInt32(array.GetCount() * 4 * 12 + 256));
	char* ptr = &buffer[0];
	for (ndInt32 i = 0; i < array.GetCount(); ++i)
	{
		ptr = FloatToString(ptr, 256, array[i].m_x);
		ptr = FloatToString(ptr, 256, array[i].m_y);
		ptr = FloatToString(ptr, 256, array[i].m_z);
	}
	CleanWhiteSpace(&buffer[0]);

	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);

	node->SetAttribute("count", ndInt32(array.GetCount()));
	node->SetAttribute("float3Array", &buffer[0]);
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

	ndInt32 ret = 0;
	long long int value;
	ret = sscanf(data, "%lld", &value);
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
	ndInt32 ret = 0;
	ret = sscanf(positData, "%lf %lf %lf", &fx, &fy, &fz);

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

void xmlGetInt(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndInt32>& array)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);
	ndInt32 count;
	element->Attribute("count", &count);
	const char* const data = element->Attribute("intArray");
	ndAssert(data);

	char x[128];
	x[127] = 0;
	size_t start = 0;
	ndVector point(ndVector::m_zero);
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndInt32 ret = sscanf(&data[start], "%[^ ]", x);
		start += strlen(x) + 1;

		long long int fx;
		ret = sscanf(x, "%lld", &fx);
		array.PushBack(ndInt32(fx));
	}
}

void xmlGetInt64(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndInt64>& array)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);
	ndInt32 count;
	element->Attribute("count", &count);
	const char* const data = element->Attribute("intArray");
	ndAssert(data);

	char x[128];
	x[127] = 0;
	size_t start = 0;
	ndVector point(ndVector::m_zero);
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndInt32 ret = sscanf(&data[start], "%[^ ]", x);
		start += strlen(x) + 1;

		long long int fx;
		ret = sscanf(x, "%lld", &fx);
		array.PushBack(ndInt64 (fx));
	}
}

void xmlGetFloatArray3(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndTriplexReal>& array)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);
	ndInt32 count;
	element->Attribute("count", &count);

	const char* const data = element->Attribute("float3Array");
	ndAssert(data);

	size_t start = 0;
	char x[128];
	char y[128];
	char z[128];

	x[127] = 0;
	y[127] = 0;
	z[127] = 0;
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndInt32 ret = sscanf(&data[start], "%[^ ] %[^ ] %[^ ]", x, y, z);
		start += strlen(x) + strlen(y) + strlen(z) + 3;

		ndFloat64 fx;
		ndFloat64 fy;
		ndFloat64 fz;
		ndTriplexReal tuple;

		ret = sscanf(x, "%lf", &fx);
		ret = sscanf(y, "%lf", &fy);
		ret = sscanf(z, "%lf", &fz);
		
		tuple.m_x = ndFloat32(fx);
		tuple.m_y = ndFloat32(fy);
		tuple.m_z = ndFloat32(fz);
		array.PushBack(tuple);
	}
}

void xmlGetFloatArray3(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndVector>& array)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);
	ndInt32 count;
	element->Attribute("count", &count);

	const char* const data = element->Attribute("float3Array");
	ndAssert(data);

	size_t start = 0;
	for (ndInt32 i = 0; i < count; ++i)
	{
		char x[128];
		char y[128];
		char z[128];

		x[127] = 0;
		y[127] = 0;
		z[127] = 0;

		ndInt32 ret = sscanf(&data[start], "%[^ ] %[^ ] %[^ ]", x, y, z);
		start += strlen(x) + strlen(y) + strlen(z) + 3;

		ndFloat64 fx;
		ndFloat64 fy;
		ndFloat64 fz;

		ret = sscanf(x, "%lf", &fx);
		ret = sscanf(y, "%lf", &fy);
		ret = sscanf(z, "%lf", &fz);
		array.PushBack (ndVector(ndFloat32(fx), ndFloat32(fy), ndFloat32(fz), ndFloat32(0.0f)));
	}
}

void xmlGetFloat64Array3(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndBigVector>& array)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*)rootNode->FirstChild(name);
	ndAssert(element);
	ndInt32 count;
	element->Attribute("count", &count);

	const char* const data = element->Attribute("float3Array");
	ndAssert(data);

	size_t start = 0;
	for (ndInt32 i = 0; i < count; ++i)
	{
		char x[128];
		char y[128];
		char z[128];

		x[127] = 0;
		y[127] = 0;
		z[127] = 0;

		ndInt32 ret = sscanf(&data[start], "%[^ ] %[^ ] %[^ ]", x, y, z);
		start += strlen(x) + strlen(y) + strlen(z) + 3;

		ndFloat64 fx;
		ndFloat64 fy;
		ndFloat64 fz;

		ret = sscanf(x, "%lf", &fx);
		ret = sscanf(y, "%lf", &fy);
		ret = sscanf(z, "%lf", &fz);
		array.PushBack(ndBigVector(fx, fy, fz, ndFloat64(0.0f)));
	}
}

