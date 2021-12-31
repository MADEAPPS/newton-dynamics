/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndDemoEntity.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoEntityManager.h"

ndDemoMesh::ndDemoMesh(const char* const name)
	:ndDemoMeshInterface()
	,ndList<ndDemoSubMesh>()
	,m_indexCount(0)
	,m_vertexCount(0)
	,m_textureLocation(0)
	,m_transparencyLocation(0)
	,m_normalMatrixLocation(0)
	,m_projectMatrixLocation(0)
	,m_viewModelMatrixLocation(0)
	,m_directionalLightDirLocation(0)
	,m_materialAmbientLocation(0)
	,m_materialDiffuseLocation(0)
	,m_materialSpecularLocation(0)
	,m_shader(0)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_hasTransparency(false)
{
	m_name = name;
}

ndDemoMesh::ndDemoMesh(const ndDemoMesh&, const ndShaderPrograms&)
	:ndDemoMeshInterface()
	,ndList<ndDemoSubMesh>()
	,m_indexCount(0)
	,m_vertexCount(0)
	,m_shader(0)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_hasTransparency(false)
{
	dAssert(0);
	//AllocVertexData(mesh.m_vertexCount);
	//memcpy (m_points, mesh.m_points, m_vertexCount * sizeof (glPositionNormalUV));
	//
	//for (ndNode* nodes = mesh.GetFirst(); nodes; nodes = nodes->GetNext()) 
	//{
	//	ndDemoSubMesh* const segment = AddSubMesh();
	//	ndDemoSubMesh& srcSegment = nodes->GetInfo();
	//
	//	segment->AllocIndexData (srcSegment.m_indexCount);
	//	memcpy (segment->m_indexes, srcSegment.m_indexes, srcSegment.m_indexCount * sizeof (unsigned));
	//
	//	segment->m_shiness = srcSegment.m_shiness;
	//	segment->m_ambient = srcSegment.m_ambient;
	//	segment->m_diffuse = srcSegment.m_diffuse;
	//	segment->m_specular = srcSegment.m_specular;
	//	segment->m_textureHandle = srcSegment.m_textureHandle;
	//	segment->m_textureName = srcSegment.m_textureName;
	//	segment->m_shader = srcSegment.m_shader;
	//	if (segment->m_textureHandle) 
	//	{
	//		AddTextureRef (srcSegment.m_textureHandle);
	//	}
	//}
	//
	//// see if this mesh can be optimized
	//OptimizeForRender ();
}

ndDemoMesh::ndDemoMesh(const char* const name, const ndShaderPrograms& shaderCache, const ndShapeInstance* const collision, const char* const texture0, const char* const, const char* const, ndFloat32 opacity, const ndMatrix& uvMatrix)
	:ndDemoMeshInterface()
	,ndList<ndDemoSubMesh>()
	,m_indexCount(0)
	,m_vertexCount(0)
	,m_shader(0)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_hasTransparency(false)
{
	m_name = name;
	ndShapeInstanceMeshBuilder mesh(*collision);

	//mesh.CalculateNormals(30.0f * dDegreeToRad);

	ndMatrix aligmentUV(uvMatrix);
	//ndMatrix aligmentUV (collision->GetLocalMatrix());
	//aligmentUV = aligmentUV.Inverse();

	m_shader = shaderCache.m_diffuseEffect;

	// apply uv projections
	ndShapeInfo info(collision->GetShapeInfo());
	switch (info.m_collisionType)
	{
		case ndShapeID::m_sphere:
		case ndShapeID::m_capsule:
		{
			mesh.SphericalMapping(LoadTexture(texture0), &aligmentUV[0][0]);
			break;
		}

		//case SERIALIZE_ID_CONE:
		//case SERIALIZE_ID_CAPSULE:
		//case SERIALIZE_ID_CYLINDER:
		//case SERIALIZE_ID_CHAMFERCYLINDER:
		//{
		//	//NewtonMeshApplySphericalMapping(mesh, LoadTexture(texture0));
		//	NewtonMeshApplyCylindricalMapping(mesh, LoadTexture(texture0), LoadTexture(texture1), &aligmentUV[0][0]);
		//	break;
		//}

		case ndShapeID::m_box:
		{
			ndInt32 tex0 = LoadTexture(texture0);
			//ndInt32 tex1 = LoadTexture(texture1);
			//ndInt32 tex2 = LoadTexture(texture2);
			//mesh.BoxMapping(tex0, tex1, tex2, aligmentUV);
			mesh.UniformBoxMapping(tex0, aligmentUV);
			break;
		}

		default:
		{
			ndInt32 tex0 = LoadTexture(texture0);
			//ndInt32 tex0 = LoadTexture(texture0);
			//ndInt32 tex1 = LoadTexture(texture1);
			//ndInt32 tex2 = LoadTexture(texture2);
			//NewtonMeshApplyBoxMapping(mesh, tex0, tex1, tex2, &aligmentUV[0][0]);
			mesh.UniformBoxMapping(tex0, aligmentUV);
		}
	}

	// extract the materials index array for mesh
	ndIndexArray* const geometryHandle = mesh.MaterialGeometryBegin();

	// extract vertex data  from the newton mesh
	ndInt32 indexCount = 0;
	ndInt32 vertexCount = mesh.GetPropertiesCount();
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		indexCount += mesh.GetMaterialIndexCount(geometryHandle, handle);
	}

	struct dTmpData
	{
		ndFloat32 m_posit[3];
		ndFloat32 m_normal[3];
		ndFloat32 m_uv[2];
	};

	ndArray<dTmpData> tmp;
	ndArray<ndInt32> indices;
	ndArray<glPositionNormalUV> points;

	tmp.SetCount(vertexCount);
	points.SetCount(vertexCount);
	indices.SetCount(indexCount);

	mesh.GetVertexChannel(sizeof(dTmpData), &tmp[0].m_posit[0]);
	mesh.GetNormalChannel(sizeof(dTmpData), &tmp[0].m_normal[0]);
	mesh.GetUV0Channel(sizeof(dTmpData), &tmp[0].m_uv[0]);

	for (ndInt32 i = 0; i < vertexCount; i++)
	{
		points[i].m_posit.m_x = GLfloat(tmp[i].m_posit[0]);
		points[i].m_posit.m_y = GLfloat(tmp[i].m_posit[1]);
		points[i].m_posit.m_z = GLfloat(tmp[i].m_posit[2]);
		points[i].m_normal.m_x = GLfloat(tmp[i].m_normal[0]);
		points[i].m_normal.m_y = GLfloat(tmp[i].m_normal[1]);
		points[i].m_normal.m_z = GLfloat(tmp[i].m_normal[2]);
		points[i].m_uv.m_u = GLfloat(tmp[i].m_uv[0]);
		points[i].m_uv.m_v = GLfloat(tmp[i].m_uv[1]);
	}

	ndInt32 segmentStart = 0;
	bool hasTransparency = false;
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		ndInt32 material = mesh.GetMaterialID(geometryHandle, handle);
		ndDemoSubMesh* const segment = AddSubMesh();

		segment->m_material.m_textureHandle = (GLuint)material;
		segment->SetOpacity(opacity);
		hasTransparency = hasTransparency | segment->m_hasTranparency;

		segment->m_indexCount = mesh.GetMaterialIndexCount(geometryHandle, handle);

		segment->m_segmentStart = segmentStart;
		mesh.GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment->m_indexCount;
	}

	mesh.MaterialGeometryEnd(geometryHandle);

	m_hasTransparency = hasTransparency;

	// optimize this mesh for hardware buffers if possible
	OptimizeForRender(&points[0], vertexCount, &indices[0], indexCount);
}

ndDemoMesh::ndDemoMesh(const char* const name, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache)
	:ndDemoMeshInterface()
	,ndList<ndDemoSubMesh>()
	,m_indexCount(0)
	,m_vertexCount(0)
	,m_shader(0)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_hasTransparency(false)
{
	m_name = name;
	m_shader = shaderCache.m_diffuseEffect;

	// extract the materials index array for mesh
	ndIndexArray* const geometryHandle = meshNode->MaterialGeometryBegin();

	// extract vertex data  from the newton mesh		
	ndInt32 indexCount = 0;
	ndInt32 vertexCount = meshNode->GetPropertiesCount();
	for (ndInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		indexCount += meshNode->GetMaterialIndexCount(geometryHandle, handle);
	}

	struct dTmpData
	{
		ndFloat32 m_posit[3];
		ndFloat32 m_normal[3];
		ndFloat32 m_uv[2];
	};

	ndArray<dTmpData> tmp;
	ndArray<ndInt32> indices;
	ndArray<glPositionNormalUV> points;

	indices.SetCount(indexCount);
	points.SetCount(vertexCount);
	tmp.SetCount(vertexCount);

	meshNode->GetVertexChannel(sizeof(dTmpData), &tmp[0].m_posit[0]);
	meshNode->GetNormalChannel(sizeof(dTmpData), &tmp[0].m_normal[0]);
	meshNode->GetUV0Channel(sizeof(dTmpData), &tmp[0].m_uv[0]);

	for (ndInt32 i = 0; i < vertexCount; i++)
	{
		points[i].m_posit.m_x = GLfloat(tmp[i].m_posit[0]);
		points[i].m_posit.m_y = GLfloat(tmp[i].m_posit[1]);
		points[i].m_posit.m_z = GLfloat(tmp[i].m_posit[2]);
		points[i].m_normal.m_x = GLfloat(tmp[i].m_normal[0]);
		points[i].m_normal.m_y = GLfloat(tmp[i].m_normal[1]);
		points[i].m_normal.m_z = GLfloat(tmp[i].m_normal[2]);
		points[i].m_uv.m_u = GLfloat(tmp[i].m_uv[0]);
		points[i].m_uv.m_v = GLfloat(tmp[i].m_uv[1]);
	}

	ndInt32 segmentStart = 0;
	bool hasTransparency = false;
	const ndArray<ndMeshEffect::dMaterial>& materialArray = meshNode->GetMaterials();
	for (ndInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		ndInt32 materialIndex = meshNode->GetMaterialID(geometryHandle, handle);
		ndDemoSubMesh* const segment = AddSubMesh();
		
		const ndMeshEffect::dMaterial& material = materialArray[materialIndex];
		segment->m_material.m_ambient = glVector4(material.m_ambient);
		segment->m_material.m_diffuse = glVector4(material.m_diffuse);
		segment->m_material.m_specular = glVector4(material.m_specular);
		segment->m_material.m_opacity = GLfloat(material.m_opacity);
		segment->m_material.m_shiness = GLfloat(material.m_shiness);
		strcpy(segment->m_material.m_textureName, material.m_textureName);
		//segment->m_material.m_textureHandle = (GLuint)material.m_textureHandle;
		segment->m_material.m_textureHandle = LoadTexture(material.m_textureName);
		segment->SetOpacity(material.m_opacity);
		hasTransparency = hasTransparency | segment->m_hasTranparency;
		
		segment->m_indexCount = meshNode->GetMaterialIndexCount(geometryHandle, handle);
		
		segment->m_segmentStart = segmentStart;
		meshNode->GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment->m_indexCount;
	}

	meshNode->MaterialGeometryEnd(geometryHandle);

	m_hasTransparency = hasTransparency;

	// optimize this mesh for hardware buffers if possible
	OptimizeForRender(&points[0], vertexCount, &indices[0], indexCount);
}

ndDemoMesh::~ndDemoMesh()
{
	ResetOptimization();
}

const char* ndDemoMesh::GetTextureName(const ndDemoSubMesh* const subMesh) const
{
	return subMesh->m_material.m_textureName;
}

ndDemoSubMesh* ndDemoMesh::AddSubMesh()
{
	return &Append()->GetInfo();
}

void ndDemoMesh::RenderNormals()
{
	dAssert(0);
/*
	glDisable(GL_TEXTURE_2D);

	glColor3f(1.0f, 1.0f, 1.0f);

	ndFloat32 length = 0.1f;
	glBegin(GL_LINES);

	for (ndInt32 i = 0; i < m_vertexCount; i ++)
	{
		glVertex3f (GLfloat(m_vertex[i * 3 + 0]), GLfloat(m_vertex[i * 3 + 1]), GLfloat(m_vertex[i * 3 + 2]));
		glVertex3f (GLfloat(m_vertex[i * 3 + 0] + m_normal[i * 3 + 0] * length), GLfloat(m_vertex[i * 3 + 1] + m_normal[i * 3 + 1] * length), GLfloat(m_vertex[i * 3 + 2] + m_normal[i * 3 + 2] * length));
	}

	glEnd();
*/
}

void ndDemoMesh::OptimizeForRender(
	const glPositionNormalUV* const points, ndInt32 pointCount,
	const ndInt32* const indices, ndInt32 indexCount)
{
	// first make sure the previous optimization is removed
	ResetOptimization();

	glGenVertexArrays(1, &m_vertextArrayBuffer);
	glBindVertexArray(m_vertextArrayBuffer);

	glGenBuffers(1, &m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, pointCount * sizeof(glPositionNormalUV), &points[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_posit));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_normal));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_uv));
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindVertexArray(0);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(0);

	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexCount * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glUseProgram(m_shader);
	m_textureLocation = glGetUniformLocation(m_shader, "texture");
	m_transparencyLocation = glGetUniformLocation(m_shader, "transparency");
	m_normalMatrixLocation = glGetUniformLocation(m_shader, "normalMatrix");
	m_projectMatrixLocation = glGetUniformLocation(m_shader, "projectionMatrix");
	m_viewModelMatrixLocation = glGetUniformLocation(m_shader, "viewModelMatrix");
	m_directionalLightDirLocation = glGetUniformLocation(m_shader, "directionalLightDir");

	m_materialAmbientLocation = glGetUniformLocation(m_shader, "material_ambient");
	m_materialDiffuseLocation = glGetUniformLocation(m_shader, "material_diffuse");
	m_materialSpecularLocation = glGetUniformLocation(m_shader, "material_specular");

	glUseProgram(0);
	m_vertexCount = pointCount;
	m_indexCount = indexCount;
}

void  ndDemoMesh::ResetOptimization()
{
	if (m_vertextArrayBuffer)
	{
		glDeleteBuffers(1, &m_indexBuffer);
		glDeleteBuffers(1, &m_vertexBuffer);
		glDeleteVertexArrays(1, &m_vertextArrayBuffer);
	}
}

void ndDemoMesh::Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix)
{
	if (m_isVisible)
	{
		bool hasTransparency = m_hasTransparency;
		if (hasTransparency) 
		{
			scene->PushTransparentMesh(this, modelMatrix);
		}

		if (hasTransparency)
		{
			for (ndNode* node = GetFirst(); node; node = node->GetNext())
			{
				ndDemoSubMesh& segment = node->GetInfo();
				hasTransparency = hasTransparency & segment.m_hasTranparency;
			}
		}

		if (!hasTransparency)
		{
			//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glUseProgram(m_shader);

			ndDemoCamera* const camera = scene->GetCamera();

			const ndMatrix& viewMatrix = camera->GetViewMatrix();
			const glMatrix& projectionMatrix (camera->GetProjectionMatrix());
			const glMatrix viewModelMatrix(modelMatrix * viewMatrix);
			const glVector4 directionaLight(viewMatrix.RotateVector(ndVector(-1.0f, 1.0f, 0.0f, 0.0f)).Normalize());

			glUniform1i(m_textureLocation, 0);
			glUniform1f(m_transparencyLocation, 1.0f);
			glUniform4fv(m_directionalLightDirLocation, 1, &directionaLight[0]);
			glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &viewModelMatrix[0][0]);
			glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &projectionMatrix[0][0]);
			glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &viewModelMatrix[0][0]);

			//float k1 = 7.0 / 120.0;
			//float k2 = 1.0 / 240.0;
			//float d2 = viewModelMatrix.m_posit.DotProduct(viewModelMatrix.m_posit & ndVector::m_triplexMask).GetScalar();
			//float d1 = sqrt(d2);
			//float attenuation = 1.0 / (1.0 + k1 * d1 + k2 * d2);
			//dAssert(attenuation > 0.0f);
			//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

			glBindVertexArray(m_vertextArrayBuffer);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);

			glActiveTexture(GL_TEXTURE0);
			for (ndNode* node = GetFirst(); node; node = node->GetNext())
			{
				ndDemoSubMesh& segment = node->GetInfo();
				if (!segment.m_hasTranparency)
				{
					glUniform3fv(m_materialAmbientLocation, 1, &segment.m_material.m_ambient[0]);
					glUniform3fv(m_materialDiffuseLocation, 1, &segment.m_material.m_diffuse[0]);
					glUniform3fv(m_materialSpecularLocation, 1, &segment.m_material.m_specular[0]);

					glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
					glBindTexture(GL_TEXTURE_2D, segment.m_material.m_textureHandle);
					glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
				}
			}

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
			glBindVertexArray(0);
			glUseProgram(0);

			//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}
	}
}

void ndDemoMesh::RenderGeometry(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix)
{
	glUseProgram(m_shader);

	ndDemoCamera* const camera = scene->GetCamera();

	const ndMatrix& viewMatrix = camera->GetViewMatrix();
	const glMatrix& projectionMatrix (camera->GetProjectionMatrix());
	const glMatrix viewModelMatrix(modelMatrix * viewMatrix);
	const glVector4 directionaLight(viewMatrix.RotateVector(ndVector(-1.0f, 1.0f, 0.0f, 0.0f)).Normalize());

	glUniform1i(m_textureLocation, 0);
	glUniform4fv(m_directionalLightDirLocation, 1, &directionaLight[0]);
	glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &viewModelMatrix[0][0]);
	glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &projectionMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &viewModelMatrix[0][0]);

	glBindVertexArray(m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);

	glActiveTexture(GL_TEXTURE0);
	for (ndNode* node = GetFirst(); node; node = node->GetNext())
	{
		ndDemoSubMesh& segment = node->GetInfo();
		if (segment.m_hasTranparency)
		{
			glUniform1f(m_transparencyLocation, segment.m_material.m_opacity);
			//glMaterialParam(GL_FRONT, GL_SPECULAR, &segment.m_material.m_specular.m_x);
			//glMaterialParam(GL_FRONT, GL_AMBIENT, &segment.m_material.m_ambient.m_x);
			//glMaterialParam(GL_FRONT, GL_DIFFUSE, &segment.m_material.m_diffuse.m_x);
			//glMaterialf(GL_FRONT, GL_SHININESS, GLfloat(segment.m_material.m_shiness));

			glUniform3fv(m_materialDiffuseLocation, 1, &segment.m_material.m_diffuse[0]);
			glUniform3fv(m_materialAmbientLocation, 1, &segment.m_material.m_ambient[0]);
			glUniform3fv(m_materialSpecularLocation, 1, &segment.m_material.m_ambient[0]);

			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

			glBindTexture(GL_TEXTURE_2D, segment.m_material.m_textureHandle);
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

void ndDemoMesh::RenderTransparency(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix)
{
	if (m_isVisible)
	{
		glDepthMask(GL_FALSE);
		glEnable(GL_BLEND);
		glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glFrontFace(GL_CW);
		RenderGeometry(scene, modelMatrix);

		glFrontFace(GL_CCW);
		RenderGeometry(scene, modelMatrix);
		
		glEnable(GL_CULL_FACE);
		glDisable(GL_BLEND);
		glDepthMask(GL_TRUE);
	}
}

void ndDemoMesh::GetVertexArray(ndArray<ndVector>& points) const
{
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	const glPositionNormalUV* const data = (glPositionNormalUV*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_ONLY);

	points.Resize(m_vertexCount);
	points.SetCount(m_vertexCount);
	for (ndInt32 i = 0; i < m_vertexCount; i++)
	{
		points[i] = ndVector(data[i].m_posit.m_x, data[i].m_posit.m_y, data[i].m_posit.m_z, ndFloat32 (0.0f));
	}

	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

