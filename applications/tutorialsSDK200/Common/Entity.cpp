/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "StdAfx.h"
#include "Entity.h"
#include "tinyxml.h"
#include "OpenGlUtil.h"


//#define USE_XML_FILE

// This function or variable may be unsafe. Consider using fscanf_s instead. 
// To disable deprecation, use _CRT_SECURE_NO_WARNINGS. See online help for details.
#pragma warning (disable: 4996) 

Entity::Entity(void) :
	m_matrix (GetIdentityMatrix()),
	m_curPosition (0.0f, 0.0f, 0.0f, 1.0f),
	m_prevPosition (0.0f, 0.0f, 0.0f, 1.0f),
	m_curRotation (1.0f, 0.0f, 0.0f, 0.0f),
	m_prevRotation (1.0f, 0.0f, 0.0f, 0.0f)
{
	m_displayList = 0;
	m_subMeshCount = 0;
	m_vertexCount = 0;
	m_uv = NULL;
	m_vertex = NULL;
	m_normal = NULL;
	m_subMeshes = NULL;
}

Entity::~Entity(void)
{
	if (m_displayList) {
		glDeleteLists (m_displayList, 1);
	}

	if (m_vertex) {
		delete[] m_uv;
		delete[] m_vertex;
		delete[] m_normal;
	}

	if (m_subMeshes) {
		for (int i = 0; i < m_subMeshCount; i ++) {
			delete[] m_subMeshes[i].m_indexArray;
		}
		delete[] m_subMeshes;
	}
}

void Entity::GetBBox (dVector& minBox, dVector& maxBox)
{
	// initialize the Box bound to ridicules values
	minBox = dVector ( 1.0e10f,  1.0e10f,  1.0e10f, 1.0f);
	maxBox = dVector (-1.0e10f, -1.0e10f, -1.0e10f, 1.0f);
	for (int i = 0; i < m_vertexCount; i ++) {
		dFloat val;

		// adjust to a better bound for x
		val = m_vertex[i * 3 + 0];
		minBox.m_x = (val < minBox.m_x) ? val : minBox.m_x;
		maxBox.m_x = (val > maxBox.m_x) ? val : maxBox.m_x;

		// adjust to a better bound for y
		val = m_vertex[i * 3 + 1];
		minBox.m_y = (val < minBox.m_y) ? val : minBox.m_y;
		maxBox.m_y = (val > maxBox.m_y) ? val : maxBox.m_y;

		// adjust to a better bound for z
		val = m_vertex[i * 3 + 2];
		minBox.m_z = (val < minBox.m_z) ? val : minBox.m_z;
		maxBox.m_z = (val > maxBox.m_z) ? val : maxBox.m_z;
	}
}



void Entity::OptimizeMesh()
{
	// create a compiled display list for faster rendering
	dMatrix matrix (GetIdentityMatrix());

	glPushMatrix();
	glMultMatrixf(&matrix[0][0]);

	dFloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	glEnableClientState (GL_VERTEX_ARRAY);
	glEnableClientState (GL_NORMAL_ARRAY);
	glEnableClientState (GL_TEXTURE_COORD_ARRAY);

	glVertexPointer (3, GL_FLOAT, 0, m_vertex);
	glNormalPointer (GL_FLOAT, 0, m_normal);
	glTexCoordPointer (2, GL_FLOAT, 0, m_uv);


	m_displayList = GLuint (glGenLists(1));
	glNewList(m_displayList, GL_COMPILE);
	for (int i = 0; i < m_subMeshCount; i ++) {

		SubMesh& segment = m_subMeshes[i];

		//      glMaterialfv(GL_FRONT, GL_SPECULAR, &segment.m_specular.m_x);
		//      glMaterialfv(GL_FRONT, GL_AMBIENT, &segment.m_ambient.m_x);
		//      glMaterialfv(GL_FRONT, GL_DIFFUSE, &segment.m_diffuse.m_x);
		//      glMaterialf(GL_FRONT, GL_SHININESS, segment.m_shiness);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		if (segment.m_textureHandle) {
			glEnable(GL_TEXTURE_2D);            
			glBindTexture(GL_TEXTURE_2D, GLuint (segment.m_textureHandle));
		} else {
			glDisable(GL_TEXTURE_2D);
		}

		glDrawElements (GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_SHORT, segment.m_indexArray);
	}
	glEndList();

	glDisableClientState(GL_VERTEX_ARRAY);    // disable vertex arrays
	glDisableClientState(GL_NORMAL_ARRAY);    // disable normal arrays
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);   // disable normal arrays

	glPopMatrix();
}

void Entity::Render (dFloat interpolationParam)
{
	interpolationParam;
	// Calculate visual Transform by Interpolating between prev and curr State
	dVector posit (m_prevPosition + (m_curPosition - m_prevPosition).Scale (interpolationParam));
	dQuaternion rotation (m_prevRotation.Slerp(m_curRotation, interpolationParam));

	m_matrix = dMatrix (rotation, posit);

	glPushMatrix();
	glMultMatrixf(&m_matrix[0][0]);
	if (m_displayList) {
		glCallList(m_displayList);
	} else {
		dFloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
		glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
		glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

		glEnableClientState (GL_VERTEX_ARRAY);
		glEnableClientState (GL_NORMAL_ARRAY);
		glEnableClientState (GL_TEXTURE_COORD_ARRAY);

		glVertexPointer (3, GL_FLOAT, 0, m_vertex);
		glNormalPointer (GL_FLOAT, 0, m_normal);
		glTexCoordPointer (2, GL_FLOAT, 0, m_uv);



		for (int i = 0; i < m_subMeshCount; i ++) {

			SubMesh& segment = m_subMeshes[i];

			//          glMaterialfv(GL_FRONT, GL_SPECULAR, &segment.m_specular.m_x);
			//          glMaterialfv(GL_FRONT, GL_AMBIENT, &segment.m_ambient.m_x);
			//          glMaterialfv(GL_FRONT, GL_DIFFUSE, &segment.m_diffuse.m_x);
			//          glMaterialf(GL_FRONT, GL_SHININESS, segment.m_shiness);
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			if (segment.m_textureHandle) {
				glEnable(GL_TEXTURE_2D);            
				glBindTexture(GL_TEXTURE_2D, GLuint (segment.m_textureHandle));
			} else {
				glDisable(GL_TEXTURE_2D);
			}

			glDrawElements (GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_SHORT, segment.m_indexArray);
		}

		glDisableClientState(GL_VERTEX_ARRAY);    // disable vertex arrays
		glDisableClientState(GL_NORMAL_ARRAY);    // disable normal arrays
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);   // disable normal arrays
	}

	glPopMatrix();
}



/*
void Entity::LoadMesh (const char* name)
{
	int vertexCount;
	int subMeshCount;
	char outPathName[2048];
	FILE* file;

	// get the full path and Name
	GetWorkingFileName (name, outPathName);   

	file = fopen (outPathName, "rb");


	// read all of the vertices's, normal and UV
	fscanf (file, "%s %d/n", outPathName, &vertexCount);
	m_vertexCount = vertexCount;
	m_vertex = (dFloat*) malloc (3 * vertexCount * sizeof (dFloat));
	m_normal = (dFloat*) malloc (3 * vertexCount * sizeof (dFloat));
	m_uv = (dFloat*) malloc (2 * vertexCount * sizeof (dFloat));
	for (int i = 0; i < vertexCount; i ++) {
		fscanf (file, "%f %f %f ", &m_vertex[i * 3], &m_vertex[i * 3 + 1], &m_vertex[i * 3 + 2]);
		fscanf (file, "%f %f %f ", &m_normal[i * 3], &m_normal[i * 3 + 1], &m_normal[i * 3 + 2]);
		fscanf (file, "%f %f\n", &m_uv[i * 2], &m_uv[i * 2 + 1]);
	}

	// read all of the  mesh segments, 
	fscanf (file, "%s %d/n", outPathName, &subMeshCount);
	m_subMeshCount = subMeshCount;
	m_subMeshes = (SubMesh*) malloc (subMeshCount * sizeof (SubMesh));
	for (int i = 0; i < subMeshCount; i ++) { 
		int indexCount;
		char texName[256];
		fscanf (file, "%s %s/n", outPathName, texName);
		fscanf (file, "%s %d/n", outPathName, &indexCount);

		// load the texture for this submesh
		m_subMeshes[i].m_textureHandle = LoadTexture (texName);
		m_subMeshes[i].m_indexCount = indexCount;
		m_subMeshes[i].m_indexArray = (unsigned short*) malloc (indexCount * sizeof (unsigned short));
		for (int j = 0; j < indexCount; j ++) {
			int index;
			fscanf (file, "%d ", &index);
			m_subMeshes[i].m_indexArray[j] = (unsigned short) index;
		}
		fscanf (file, "\n");
	}

	// close file after reading
	fclose (file);

	// create a complied DislpalyList
	OptimizeMesh();
}
*/


#if USE_XML_FILE
static int StringToInts (const char* string, int* ints)
{
	int count;
	count = 0;

	do {
		ints[count] = atoi (string);
		count ++;
		while (string[0] && !isspace(string[0])) string ++;
		while (string[0] && isspace(string[0])) string ++;
	} while (string[0]);

	return count;
}


static int StringToFloats (const char* string, dFloat* floats)
{
	int count;
	count = 0;

	do {
		floats[count] = dFloat (atof (string));
		count ++;
		while (string[0] && !isspace(string[0])) string ++;
		while (string[0] && isspace(string[0])) string ++;
	} while (string[0]);

	return count;
}
#endif


void Entity::LoadMesh (const char* name)
{
#if USE_XML_FILE

	char outPathName[2048];
	const TiXmlElement* root;

	GetWorkingFileName (name, outPathName);   
	TiXmlDocument doc (outPathName);
	doc.LoadFile();

	root = doc.RootElement();
	if (root && !strcmp (root->GetText (), "newton 2.0 file format")){
		//		for (const TiXmlElement* meshNode = (TiXmlElement*)root->FirstChild("mesh"); meshNode; meshNode = (TiXmlElement*)meshNode->NextSibling()) {
		{

			//			const char* type;
			//			dMesh* mesh;
			//			mesh = NULL;
			const TiXmlElement* meshNode = (TiXmlElement*)root->FirstChild("mesh");

			//			type = meshNode->Attribute ("type");
			//			mesh = context.CreateMesh (strcmp (type, "static") ? D_SKIN_MESH : D_STATIC_MESH);
			//			meshNode->Attribute ("boneID", &mesh->m_boneID);
			//			strcpy (mesh->m_name, meshNode->Attribute ("name"));

			int vertexCount;
			const TiXmlElement* vertex;
			vertex = (TiXmlElement*)meshNode->FirstChild("Vertex");
			_ASSERTE (vertex);
			vertex->Attribute ("count", &vertexCount);  

			m_vertexCount = vertexCount;
			m_vertex = (dFloat*) malloc (3 * vertexCount * sizeof (dFloat));
			m_normal = (dFloat*) malloc (3 * vertexCount * sizeof (dFloat));
			m_uv = (dFloat*) malloc (2 * vertexCount * sizeof (dFloat));
			{
				int count;
				dFloat* data;
				int* indices;
				const TiXmlElement* posit;
				const TiXmlElement* positIndex;

				posit = (TiXmlElement*)vertex->FirstChild("posit");
				posit->Attribute ("count", &count);
				data = (dFloat*) malloc (3 * count * sizeof (dFloat));
				StringToFloats (posit->Attribute ("float3"), data);

				positIndex = (TiXmlElement*)posit->FirstChild("indices");
				indices = (int*) malloc (vertexCount * sizeof (int));;
				StringToInts (positIndex->GetText(), indices);
				for (int i = 0; i < vertexCount; i ++) {
					int index;
					index = indices[i];
					m_vertex[i * 3 + 0] = data[index * 3 + 0];
					m_vertex[i * 3 + 1] = data[index * 3 + 1];
					m_vertex[i * 3 + 2] = data[index * 3 + 2];
				}
				free (indices);
				free (data);
			}

			{
				int count;
				dFloat* data;
				int* indices;
				const TiXmlElement* posit;
				const TiXmlElement* positIndex;

				posit = (TiXmlElement*)vertex->FirstChild("normal");
				posit->Attribute ("count", &count);
				data = (dFloat*) malloc (3 * count * sizeof (dFloat));
				StringToFloats (posit->Attribute ("float3"), data);

				positIndex = (TiXmlElement*)posit->FirstChild("indices");
				indices = (int*) malloc (vertexCount * sizeof (int));
				StringToInts (positIndex->GetText(), indices);
				for (int i = 0; i < vertexCount; i ++) {
					int index;
					index = indices[i];
					m_normal[i * 3 + 0] = data[index * 3 + 0];
					m_normal[i * 3 + 1] = data[index * 3 + 1];
					m_normal[i * 3 + 2] = data[index * 3 + 2];
				}
				free (indices);
				free (data);
			}

			{
				int count;
				dFloat* data;
				int* indices;
				const TiXmlElement* posit;
				const TiXmlElement* positIndex;

				posit = (TiXmlElement*)vertex->FirstChild("uv0");
				posit->Attribute ("count", &count);
				data = (dFloat*) malloc (2 * count * sizeof (dFloat));
				StringToFloats (posit->Attribute ("float2"), data);

				positIndex = (TiXmlElement*)posit->FirstChild("indices");
				indices = (int*) malloc (vertexCount * sizeof (int));
				StringToInts (positIndex->GetText(), indices);
				for (int i = 0; i < vertexCount; i ++) {
					int index;
					index = indices[i];
					m_uv[i * 2 + 0] = data[index * 2 + 0];
					m_uv[i * 2 + 1] = data[index * 2 + 1];
				}
				free (indices);
				free (data);
			}
#if 0
			if (mesh->GetType() == dMesh::D_SKIN_MESH) {
				int weightCount;
				int* boneIndex;
				int* vertexIndex;
				dFloat* vertexWeigh;
				const TiXmlElement* bones;
				const TiXmlElement* vertexs;
				const TiXmlElement* weights;
				const TiXmlElement* weightsMap;
				dMesh::dWeightList::dBoneWeightIndex* boneIndexWeight;
				dVector*  weightsPtr;

				weightsPtr = mesh->m_weighList->m_vertexWeight;
				boneIndexWeight = mesh->m_weighList->m_boneWeightIndex;

				weightsMap = (TiXmlElement*)vertex->FirstChild("boneWeightMap");
				weightsMap->Attribute ("weightCount", &weightCount);

				bones = (TiXmlElement*)weightsMap->FirstChild("boneIndex");
				vertexs = (TiXmlElement*)weightsMap->FirstChild("vertexIndex");
				weights = (TiXmlElement*)weightsMap->FirstChild("weightValue");

				boneIndex = (int*) malloc (weightCount * sizeof (int));
				vertexIndex = (int*) malloc (weightCount * sizeof (int));
				vertexWeigh = (dFloat*) malloc (weightCount * sizeof (dFloat));

				dModel::StringToFloats (weights->Attribute ("float"), vertexWeigh);
				dModel::StringToInts (bones->Attribute ("indices"), boneIndex);
				dModel::StringToInts (vertexs->Attribute ("indices"), vertexIndex);
				for (int i = 0; i < weightCount; i ++) {
					int bone; 
					int index; 
					dFloat weight; 

					bone = boneIndex[i];
					index = vertexIndex[i];
					weight = vertexWeigh[i];
					for (int j = 0; j < 4; j ++) {
						if (weightsPtr[index][j] == dFloat (0.0f)) {
							boneIndexWeight[index].m_index[j] = bone;
							weightsPtr[index][j] = weight;
							break;
						}
					}
				}
				free (vertexWeigh);
				free (vertexIndex);
				free (boneIndex);
			}
#endif

			// add the triangles to the mesh 
			const TiXmlElement* segments;
			segments = (TiXmlElement*)meshNode->FirstChild("faces");
			_ASSERTE (segments);

			m_subMeshCount = 0;
			for (const TiXmlElement* seg = (TiXmlElement*)segments->FirstChild("triangles"); seg; seg = (TiXmlElement*)seg->NextSibling()) {
				m_subMeshCount ++;
			}
			m_subMeshes = (SubMesh*) malloc (m_subMeshCount * sizeof (SubMesh));

			int index = 0;
			for (const TiXmlElement* seg = (TiXmlElement*)segments->FirstChild("triangles"); seg; seg = (TiXmlElement*)seg->NextSibling()) {

				int indexCount;
				int materialID;
				int* indices;

				SubMesh* segment;
				TiXmlElement* material;

				segment = &m_subMeshes[index];
				index ++;

				materialID = 0;
				seg->Attribute ("count", &indexCount);

				material = (TiXmlElement*)seg->FirstChild("material");
				if (material) {
					const char* texName;
					//					StringToFloats (material->Attribute("ambient"), &segment->m_ambient.m_x);
					//					StringToFloats (material->Attribute("diffuse"), &segment->m_diffuse.m_x);
					//					StringToFloats (material->Attribute("specular"), &segment->m_specular.m_x);
					//					StringToFloats (material->Attribute("shiness"), &segment->m_shiness);

					texName = material->Attribute ("texture");
					if (texName) {
						segment->m_textureHandle = LoadTexture (texName);
					}
				}

				segment->m_indexCount = indexCount;
				segment->m_indexArray = (unsigned short*) malloc (indexCount * sizeof (unsigned short));

				indices = (int*) malloc (indexCount * sizeof (int)); 
				StringToInts (seg->Attribute ("indices"), indices);
				for (int i = 0; i < indexCount; i ++) {
					segment->m_indexArray[i] = short (indices[i]);
				}
				free (indices);
			}
		} 

		// create a complied DislpalyList
		OptimizeMesh();
	}

#else


	int vertexCount;
	int subMeshCount;
	char outPathName[2048];
	FILE* file;

	// get the full path and Name
	GetWorkingFileName (name, outPathName);   

	file = fopen (outPathName, "rb");

	// read all of the vertices's, normal and UV
	fscanf (file, "%s %d/n", outPathName, &vertexCount);
	m_vertexCount = vertexCount;
	m_vertex = (dFloat*) malloc (3 * vertexCount * sizeof (dFloat));
	m_normal = (dFloat*) malloc (3 * vertexCount * sizeof (dFloat));
	m_uv = (dFloat*) malloc (2 * vertexCount * sizeof (dFloat));
	for (int i = 0; i < vertexCount; i ++) {
		fscanf (file, "%f %f %f ", &m_vertex[i * 3], &m_vertex[i * 3 + 1], &m_vertex[i * 3 + 2]);
		fscanf (file, "%f %f %f ", &m_normal[i * 3], &m_normal[i * 3 + 1], &m_normal[i * 3 + 2]);
		fscanf (file, "%f %f\n", &m_uv[i * 2], &m_uv[i * 2 + 1]);
	}

	// read all of the  mesh segments, 
	fscanf (file, "%s %d/n", outPathName, &subMeshCount);
	m_subMeshCount = subMeshCount;
	m_subMeshes = (SubMesh*) malloc (subMeshCount * sizeof (SubMesh));
	for (int i = 0; i < subMeshCount; i ++) { 
		int indexCount;
		char texName[256];
		fscanf (file, "%s %s/n", outPathName, texName);
		fscanf (file, "%s %d/n", outPathName, &indexCount);

		// load the texture for this submesh
		m_subMeshes[i].m_textureHandle = LoadTexture (texName);
		m_subMeshes[i].m_indexCount = indexCount;
		m_subMeshes[i].m_indexArray = (unsigned short*) malloc (indexCount * sizeof (unsigned short));
		for (int j = 0; j < indexCount; j ++) {
			int index;
			fscanf (file, "%d ", &index);
			m_subMeshes[i].m_indexArray[j] = (unsigned short) index;
		}
		fscanf (file, "\n");
	}

	// close file after reading
	fclose (file);

	// create a complied DislpalyList
	OptimizeMesh();
#endif
}

