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
#ifndef __ND_RENDER_PRIMITIVE_MESH_IMPLEMENT_H__
#define __ND_RENDER_PRIMITIVE_MESH_IMPLEMENT_H__

#include "ndRenderStdafx.h"
#include "ndRenderContext.h"
#include "ndRenderPrimitiveMesh.h"

class glPositionNormalUV;
class ndRenderPrimitiveMeshSegment;

class ndRenderPrimitiveMeshImplement : public ndContainersFreeListAlloc<ndRenderPrimitiveMeshImplement>
{
	public:
	ndRenderPrimitiveMeshImplement(
		const ndRender* const render, const ndShapeInstance* const collision, 
		const ndRenderPrimitiveMeshMaterial& material, ndRenderPrimitiveMesh::ndUvMapingMode mapping,
		const ndMatrix& uvMatrix, bool stretchMaping);
	~ndRenderPrimitiveMeshImplement();

	void Render(const ndRender* const render, const ndMatrix& modelViewMatrix) const;
	void RenderShadowMap(ndRenderPassShadowsImplement* const owner, const ndMatrix& lightMatrix) const;

	private:
	void ResetOptimization();
	void OptimizeForRender(
		const glPositionNormalUV* const points, ndInt32 pointCount,
		const ndInt32* const indices, ndInt32 indexCount);

	const ndRenderContext* m_context;
	ndList<ndRenderPrimitiveMeshSegment> m_segments;

	GLint m_indexCount;
	GLint m_vertexCount;

	GLuint m_indexBuffer;
	GLuint m_vertexBuffer;
	GLuint m_vertextArrayBuffer;

	//GLint m_textureLocation;
	//GLint m_transparencyLocation;

	GLint m_diffuseColor;
	GLint m_specularColor;
	GLint m_directionalLightAmbient;
	GLint m_directionalLightIntesity;
	GLint m_directionalLightDirection;
	GLint m_specularAlpha;

	GLint m_normalMatrixLocation;
	GLint m_projectMatrixLocation;
	GLint m_viewModelMatrixLocation;
	//GLint m_materialAmbientLocation;
	//GLint m_materialDiffuseLocation;
	//GLint m_materialSpecularLocation;
	//GLint m_directionalLightDirLocation;
};

#endif