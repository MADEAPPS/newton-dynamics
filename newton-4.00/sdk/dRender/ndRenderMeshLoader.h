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

#ifndef _ND_RENDER_MESH_LOADER_H_
#define _ND_RENDER_MESH_LOADER_H_


class ndRender;
class ndRenderMeshLoader : public ndMeshLoader
{
	public:
	ndRenderMeshLoader(ndRender* const renderer);
	//ndRenderMeshLoader(const ndRenderMeshLoader& src) {};
	virtual ~ndRenderMeshLoader();

	virtual bool LoadMesh(const ndString& fullPathMeshName) override;
	virtual bool ImportFbx(const ndString& fbxPathMeshName) override;

	private:
	bool MeshToRenderSceneNode(const ndString& materialBasePath);
	

	public:
	ndRender* m_owner;
	ndSharedPtr<ndRenderSceneNode> m_renderMesh;
};

#endif