/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "dCoreStdafx.h"
#include "dTinyXmlGlue.h"

static char* FloatToString(char* const buffer, dFloat32 value)
{
	sprintf(buffer, "%f", value);
	char* ptr = buffer + strlen(buffer);
	while (*(--ptr) == '0')
	{
		*ptr = ' ';
	}
	if (*ptr == '.')
	{
		ptr++;
		*ptr = '0';
	}
	ptr++;
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

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const char* const type, const char* const value)
{
	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);
	CleanWhiteSpace(value);
	node->SetAttribute(type, value);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, dInt32 value)
{
	char buffer[1024];
	sprintf(buffer, "%d", value);
	xmlSaveParam(rootNode, name, "int32", buffer);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, dInt64 value)
{
	char buffer[1024];
	sprintf(buffer, "%lld", value);
	xmlSaveParam(rootNode, name, "int64", buffer);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, dFloat32 value)
{
	char buffer[1024];
	FloatToString(buffer, value);
	xmlSaveParam(rootNode, name, "float", buffer);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const dVector& value)
{
	char buffer[1024];
	char* ptr0 = FloatToString(buffer, value.m_x);
	char* ptr1 = FloatToString(ptr0, value.m_y);
	FloatToString(ptr1, value.m_z);
	xmlSaveParam(rootNode, name, "float3", buffer);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const dMatrix& value)
{
	dVector euler0;
	dVector euler1;
	value.CalcPitchYawRoll(euler0, euler1);
	euler0 = euler0.Scale(dRadToDegree);

	char buffer[256];
	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);

	sprintf(buffer, "%f %f %f", value.m_posit.m_x, value.m_posit.m_y, value.m_posit.m_z);
	node->SetAttribute("position", buffer);
	sprintf(buffer, "%f %f %f", euler0.m_x, euler0.m_y, euler0.m_z);
	node->SetAttribute("angles", buffer);
}

void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, dInt32 count, const dVector* const array)
{
	char* const buffer = dAlloca(char, count * 4 * 12);

	char* ptr = buffer;
	for (dInt32 i = 0; i < count; i++)
	{
		for (dInt32 j = 0; j < 3; j++)
		{
			ptr = FloatToString(ptr, array[i][j]);
		}
	}
	CleanWhiteSpace(buffer);

	nd::TiXmlElement* const node = new nd::TiXmlElement(name);
	rootNode->LinkEndChild(node);
	
	node->SetAttribute("count", count);
	node->SetAttribute("floatArray", buffer);
}

D_CORE_API dInt32 xmlGetInt(const nd::TiXmlNode* const rootNode, const char* const name)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*) rootNode->FirstChild(name);
	dAssert(element);
	dInt32 value;
	element->Attribute("int32", &value);
	return value;
}

dInt64 xmlGetInt64(const nd::TiXmlNode* const rootNode, const char* const name)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*) rootNode->FirstChild(name);
	dAssert(element);
	const char* const data = element->Attribute("int64");

	dInt64 value;
	sscanf(data, "%lld", &value);
	return value;
}

dFloat32 xmlGetFloat(const nd::TiXmlNode* const rootNode, const char* const name)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*) rootNode->FirstChild(name);
	dAssert(element);
	dFloat64 value;
	element->Attribute("float", &value);
	return dFloat32 (value);
}

void xmlGetFloatArray3(const nd::TiXmlNode* const rootNode, const char* const name, dArray<dVector>& array)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*) rootNode->FirstChild(name);
	dAssert(element);
	dInt32 count;
	element->Attribute("count", &count);
	array.Resize(count);
	array.SetCount(count);

	const char* const data = element->Attribute("floatArray");

	size_t start = 0;
	dVector point(dVector::m_zero);
	for (dInt32 i = 0; i < count; i++)
	{
		char x[64];
		char y[64];
		char z[64];
		sscanf(&data[start], "%[^ ] %[^ ] %[^ ]", x, y, z);
		start += strlen(x) + strlen(y) + strlen(z) + 3;

		dFloat64 fx;
		dFloat64 fy;
		dFloat64 fz;
		
		sscanf(x, "%lf", &fx);
		sscanf(y, "%lf", &fy);
		sscanf(z, "%lf", &fz);

		point.m_x = dFloat32(fx);
		point.m_y = dFloat32(fy);
		point.m_z = dFloat32(fz);

		array[i] = point;
	}
}

D_CORE_API dVector xmlGetVector3(const nd::TiXmlNode* const rootNode, const char* const name)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*) rootNode->FirstChild(name);
	dAssert(element);

	const char* const positData = element->Attribute("float3");

	dFloat64 fx;
	dFloat64 fy;
	dFloat64 fz;
	sscanf(positData, "%lf %lf %lf", &fx, &fy, &fz);

	dVector posit(dVector::m_zero);
	posit.m_x = dFloat32(fx);
	posit.m_y = dFloat32(fy);
	posit.m_z = dFloat32(fz);
	return posit;
}

dMatrix xmlGetMatrix(const nd::TiXmlNode* const rootNode, const char* const name)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*) rootNode->FirstChild(name);
	dAssert(element);
	
	const char* const positData = element->Attribute("position");
	const char* const angleData = element->Attribute("angles");

	dVector posit(dVector::m_one);
	dVector euler(dVector::m_zero);

	dFloat64 fx;
	dFloat64 fy;
	dFloat64 fz;
	//sscanf(positData, "%f %f %f", &posit.m_x, &posit.m_y, &posit.m_z);
	sscanf(positData, "%lf %lf %lf", &fx, &fy, &fz);
	posit.m_x = dFloat32(fx);
	posit.m_y = dFloat32(fy);
	posit.m_z = dFloat32(fz);

	//sscanf(angleData, "%f %f %f", &euler.m_x, &euler.m_y, &euler.m_z);
	sscanf(angleData, "%lf %lf %lf", &fx, &fy, &fz);
	euler.m_x = dFloat32(fx);
	euler.m_y = dFloat32(fy);
	euler.m_z = dFloat32(fz);
	euler = euler.Scale(dDegreeToRad);

	dMatrix matrix (dPitchMatrix(euler.m_x) * dYawMatrix(euler.m_y) * dRollMatrix(euler.m_z));
	matrix.m_posit = posit;
	return matrix;
}

const char* xmlGetString(const nd::TiXmlNode* const rootNode, const char* const name)
{
	const nd::TiXmlElement* const element = (nd::TiXmlElement*) rootNode->FirstChild(name);
	dAssert(element);
	return element->Attribute("string");
}