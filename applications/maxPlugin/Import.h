
/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __NEWTON_MAX_IMPORT_H__
#define __NEWTON_MAX_IMPORT_H__

class Mtl;
class INode;
class dMatrix;
class Interface;
class dMeshInstance;
class ImpInterface;
class GeometryCache;
class MaterialCache;
class MaxNodeChache;



class Import
{
	public:
	Import(const char* name, Interface* ip, ImpInterface* impip);
	~Import(void);

	void SetSceneParameters();
	void SetSmoothingGroups (Mesh& mesh);
	INode* CreateMaxHelperNode ();
	INode* CreateMaxMeshNode (dScene& scene, Mtl *mtl, dScene::dTreeNode* meshNode, const GeometryCache& meshCache);

	void LoadMaterials (dScene& scene, MaterialCache& materialCache);
	void ApplyModifiers (dScene& scene, const MaxNodeChache& maxNodeCache);
	void LoadGeometries (dScene& scene, GeometryCache& meshCache, const MaterialCache& materialCache);
	void LoadNodes (dScene& scene, const GeometryCache& meshCache, Mtl *mtl, MaxNodeChache& maxNodeCache);
	void LoadNode (dScene& scene, INode* maxParent, dScene::dTreeNode* node, const dMatrix& parentMatrix, const GeometryCache& meshCache, Mtl *mtl, MaxNodeChache& maxNodeCache);


	int m_succes;
	Interface* m_ip;
	ImpInterface* m_impip;
	char m_path[256];
};


#endif