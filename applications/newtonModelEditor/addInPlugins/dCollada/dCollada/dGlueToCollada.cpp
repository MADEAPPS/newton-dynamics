//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// basic Hierarchical Scene Node Class
//********************************************************************


#include "dColladaStdafx.h"
#include "dGlueToCollada.h"

#include <dae.h>
#include <dae/domAny.h>
#include <dom/domCOLLADA.h>
#include <dom/domConstants.h>
#include <dom/domProfile_COMMON.h>


#define D_SKIN_POS_FIX					"_skin"
#define D_ANIMATION_PREFIX_NAME			"anim_"
#define D_TEXTURE_PREFIX_NAME			"texture_"
#define D_MATERIAL_PREFIX_NAME			"material_"
#define D_MATERIAL_PREFIX_NAME_EFFECT	"materialFx_"
#define	D_ROOT_NODE_NAME				"root_"
#define	D_NODE_NAME						"node_"



#ifdef _POSIX_VER 
//#define cdom::strcasecmp strcasecmp
#endif


class dColladaFileParcel
{

	class MeshPoint
	{
		public: 
		dFloat m_vertex[3];
		dFloat m_normal[3];
		dFloat m_uv[2];
		dFloat m_originalIndex;
	};


	class SourceBuffer
	{
		public: 
		int m_count;
		int m_stride;
		const char* m_id;
		domFloat *m_data;
	};


	class CollImage
	{
		public: 
		char m_textureName[D_NAME_STRING_LENGTH];
		char m_texturePathName[D_NAME_STRING_LENGTH * 4];
	};

	class CollImageCache: public dTree<CollImage, int>
	{
		public: 
		CollImageCache()
			:dTree<CollImage, int>()
		{
		}

		void AddTexture (const char* name)
		{
			CollImage texture;
			const char* ptr;

			ptr = strrchr (name, '/');
			if (ptr) {
				ptr ++;
			} else {
				ptr = strrchr (name, '\\');
				if (ptr) {
					ptr ++;
				} else {
					ptr = name;
				}
			}

//			LoadTexture(ptr, texture.m_texture);
			sprintf (texture.m_textureName, "%s", ptr);
			sprintf (texture.m_texturePathName, "%s", name);
			Insert (texture, dCRC(name));
		}

		const CollImage& GetTexture (const char* name)  const
		{
			dTreeNode* node;

			node = Find (dCRC(name));
			_ASSERTE (node);
			return node->GetInfo();
		}
	};


	class CollMaterial
	{
		public:
		//	GLuint m_texture;
		//	char m_name[64];
		CollImage m_texture;
	};

	class CollMaterialCache: public dTree<CollMaterial, int>
	{
		public: 
		CollMaterialCache()
			:dTree<CollMaterial, int>()
		{
		}

		void AddMaterial (const CollMaterial& material, const char* name)
		{
			Insert (material, dCRC(name));
		}

		CollMaterial* GetMaterial (const char* name) const
		{
			dTreeNode* node;

			node = Find (dCRC(name));
			return node ? &node->GetInfo() : NULL;
		}
	};

	class VerterMap
	{
		public:
		int m_vertexIndexInColladaMesh;
		int m_vertexIndexInDMesh;
	};

	class ModifierVertexCache: public dTree<VerterMap*, dMesh*>
	{
		public:
		ModifierVertexCache()
		{
		}

		~ModifierVertexCache()
		{	
			Iterator iter (*this);
			for (iter.Begin(); iter; iter ++) {
				VerterMap* map = (*iter);
				delete[] map;
			}
		}
	};

	class CollGeoCache: public dTree<dMesh*, domGeometry*>
	{
		public:
		dMesh* FindByName (const char* name) const
		{
			domGeometry* key;
			Iterator iter (*this);
			for (iter.Begin(); iter; iter ++) {
				key = iter.GetKey();
				if (!cdom::strcasecmp (name, key->getId())) {
					return iter.GetNode()->GetInfo();
				}
			}
			return NULL;
		}

		~CollGeoCache ()
		{
			dMesh* geometry;

			while (GetRoot()) {
				geometry = GetRoot()->GetInfo();
				Remove(GetRoot());
				_ASSERTE (geometry);
				geometry->Release();
			}
		}
	};


	public:
	dColladaFileParcel (DAE* collada, const dMatrix& globalRotation, dFloat scale)
		:m_globalRotation (globalRotation), m_scale (scale) 
	{
		m_nodeIndex = 0;
		m_collada = collada; 
	}

	~dColladaFileParcel ()
	{
	}

	void SceneToCollada (dModel* model)
	{
		daeInt error;
		daeDatabase* database;
		daeDocument *document;
		dTree<domNode*, dModel*> nodeMap; 
		dTree<unsigned, unsigned> uniqueNameFilter;

		document = NULL;
		database = m_collada->getDatabase();
		error = database->insertDocument ("mainDoc", &document);

		_ASSERTE (error == DAE_OK);
		_ASSERTE (document);

		document = database->getDocument(daeUInt (0));

		// save graphical part
		AddAsset (document);
		AddCameraLibrary (document, model);
		AddImageLibrary (document, model);
		AddMaterialLibrary (document, model);
		AddGeometryLibrary (document, model);
		AddControllerLibrary(document, model);

		AddAnimationLibrary(document, model);
		
		AddVisualSceneLibrary (document, model, nodeMap, uniqueNameFilter);

		// add the scene instance
		AddScene (document);
	}


	void SceneToCollada (dSceneModelList& sceneList)
	{
		daeInt error;
		daeDatabase* database;
		daeDocument *document;
		dTree<domNode*, dModel*> nodeMap; 

		document = NULL;
		database = m_collada->getDatabase();
		error = database->insertDocument ("mainDoc", &document);

		_ASSERTE (error == DAE_OK);
		_ASSERTE (document);

		document = database->getDocument(daeUInt (0));

		// save graphical part
		AddAsset (document);

		dTree<unsigned, unsigned> uniqueNameFilter;
		for (dSceneModelList::dListNode* node = sceneList.GetFirst(); node; node = node->GetNext()) {
			dModel* model = node->GetInfo();			
			AddCameraLibrary (document, model);
			AddImageLibrary (document, model);
			AddMaterialLibrary (document, model);
			AddGeometryLibrary (document, model);
			AddControllerLibrary(document, model);
			AddAnimationLibrary(document, model);

			AddVisualSceneLibrary (document, model, nodeMap, uniqueNameFilter);
		}

		// add the scene instance
		AddScene (document);
	}


	void SceneToCollada (NewtonWorld* world)
	{
		daeInt error;
		daeDatabase* database;
		daeDocument *document;

		document = NULL;
		database = m_collada->getDatabase();
		error = database->insertDocument ("mainDoc", &document);

		_ASSERTE (error == DAE_OK);
		_ASSERTE (document);

		document = database->getDocument(daeUInt (0));

		// save graphical part
		AddAsset (document);


		dTree<domNode*, dModel*> nodeMap;
		dTree<unsigned, unsigned> uniqueNameFilter;
		for (const NewtonBody* body = NewtonWorldGetFirstBody (world); body; body = NewtonWorldGetNextBody(world, body)) {
			dModel* model = (dModel*) NewtonBodyGetUserData(body);
			if (model) {
				_ASSERTE (model->IsType (dModel::GetRttiType()));

				AddCameraLibrary (document, model);
				AddImageLibrary (document, model);
				AddMaterialLibrary (document, model);
				AddGeometryLibrary (document, model);
				AddControllerLibrary(document, model);
				AddAnimationLibrary(document, model);

				AddVisualSceneLibrary (document, model, nodeMap, uniqueNameFilter);
			}
		}


		// find all bodies forming any kind of contraction
		dTree<const NewtonBody*, const NewtonBody*> bodyFilter;

		dTree<domGeometry *, const NewtonCollision*> shapeFilter;

		int modelID = 0;
		for (const NewtonBody* body = NewtonWorldGetFirstBody (world); body; body = NewtonWorldGetNextBody(world, body)) {
			if (!bodyFilter.Find(body)) {
				dList<const NewtonBody*> bodyList;
				int count;
				const NewtonBody* stack[2048];

				stack[0] = body;
				count = 1;
				while (count) {
					count --;
					const NewtonBody* modelBody = stack[count];

					bodyFilter.Insert(body, modelBody);
					bodyList.Append(modelBody);
					for (NewtonJoint* joint = NewtonBodyGetFirstJoint (modelBody); joint; joint = NewtonBodyGetNextJoint (body, joint)) {
						NewtonBody* body0 = NewtonJointGetBody0(joint);
						if (bodyFilter.Find(body0)) {
							stack[count] = body0;
							count ++;
						}

						NewtonBody* body1 = NewtonJointGetBody1(joint);
						if (bodyFilter.Find(body1)) {
							stack[count] = body1;
							count ++;
						}
					}
				}

				char name[256];
				dModel* model = (dModel*) NewtonBodyGetUserData (bodyList.GetFirst()->GetInfo());
				if (model) {               
					_ASSERTE (model->IsType(dModel::GetRttiType()));
					sprintf (name, "%s_%03d", model->m_name, modelID);
				} else {
					sprintf (name, "%model_%x", bodyList.GetFirst()->GetInfo());
				}
				AddPhysicsModelLibrary (document, bodyList, name, shapeFilter, nodeMap);

				modelID ++;
			}
		}

		// add the scene instance
		AddScene (document);
	}


	void ColladatToScene (dModel* model, dLoaderContext* context)
	{
		domAsset *asset;
		domCOLLADA *domRoot;
		daeDatabase* database;
		daeDocument *document;
		CollGeoCache meshCache;
		CollImageCache imageCache;
		domAsset::domUnit* unit;
		domAsset::domUp_axis* axis;
		CollMaterialCache materialCache;
		ModifierVertexCache modifierVertexMapCache;

		database = m_collada->getDatabase();
		_ASSERTE (database);
		document = database->getDocument(daeUInt (0));
		_ASSERTE (document);

		domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		_ASSERTE (domRoot);

		asset = domRoot->getAsset();
		_ASSERTE (domRoot);

		unit = asset->getUnit();
		if (unit) {
			m_scale *= dFloat (unit->getMeter());
		}

		axis = asset->getUp_axis();
		if (axis) {
			const domUpAxisType& value = axis->getValue();
			switch (value)
			{
				case UPAXISTYPE_X_UP:
				{
					m_globalRotation = dYawMatrix(3.14159265f * 0.5f) * m_globalRotation;
					break;
				}

				case UPAXISTYPE_Z_UP:
				{
					m_globalRotation = dPitchMatrix(-3.14159265f * 0.5f) * m_globalRotation;
					break;
				}

				case UPAXISTYPE_Y_UP:
				default:
				{
				}
			}
		}

		LoadVisualScene(document, model, context, meshCache, modifierVertexMapCache, materialCache, imageCache);
		context->LoaderFixup (model);
	}


	void ColladatToScene (dSceneModelList& sceneList, dLoaderContext* context)
	{
		domAsset *asset;
		domCOLLADA *domRoot;
		daeDatabase* database;
		daeDocument *document;
		CollGeoCache meshCache;
		CollImageCache imageCache;
		domAsset::domUnit* unit;
		domAsset::domUp_axis* axis;
		CollMaterialCache materialCache;
//		ColladaNodeTodSceneNodeMap nodeMap;
		ModifierVertexCache modifierVertexMapCache;

		database = m_collada->getDatabase();
		_ASSERTE (database);
		document = database->getDocument(daeUInt (0));
		_ASSERTE (document);

		domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		_ASSERTE (domRoot);

		asset = domRoot->getAsset();
		_ASSERTE (domRoot);

		unit = asset->getUnit();
		if (unit) {
			m_scale *= dFloat (unit->getMeter());
		}

		axis = asset->getUp_axis();
		if (axis) {
			const domUpAxisType& value = axis->getValue();
			switch (value)
			{
				case UPAXISTYPE_X_UP:
				{
					m_globalRotation = dYawMatrix(3.14159265f * 0.5f) * m_globalRotation;
					break;
				}

				case UPAXISTYPE_Z_UP:
				{
					m_globalRotation = dPitchMatrix(-3.14159265f * 0.5f) * m_globalRotation;
					break;
				}

				case UPAXISTYPE_Y_UP:
				default:
				{
				}
			}
		}

		LoadVisualScene(document, sceneList, context, meshCache, modifierVertexMapCache, materialCache, imageCache);
		for (dSceneModelList::dListNode* node = sceneList.GetFirst(); node; node = node->GetNext()) {
			dModel* model = node->GetInfo();
			context->LoaderFixup (model);
		}
	}


	void ColladatToScene (dSceneModelList& sceneList, NewtonWorld* world, dLoaderContext* context)
	{
		domAsset *asset;
		domCOLLADA *domRoot;
		daeDatabase* database;
		daeDocument *document;
		CollGeoCache meshCache;
		CollImageCache imageCache;
		domAsset::domUnit* unit;
		domAsset::domUp_axis* axis;
		CollMaterialCache materialCache;
		ModifierVertexCache modifierVertexMapCache;

		database = m_collada->getDatabase();
		_ASSERTE (database);
		document = database->getDocument(daeUInt (0));
		_ASSERTE (document);

		domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		_ASSERTE (domRoot);

		asset = domRoot->getAsset();
		_ASSERTE (domRoot);

		unit = asset->getUnit();
		if (unit) {
			m_scale *= dFloat (unit->getMeter());
		}

		axis = asset->getUp_axis();
		if (axis) {
			const domUpAxisType& value = axis->getValue();
			switch (value)
			{
				case UPAXISTYPE_X_UP:
				{
					m_globalRotation = dYawMatrix(3.14159265f * 0.5f) * m_globalRotation;
					break;
				}

				case UPAXISTYPE_Z_UP:
				{
					m_globalRotation = dPitchMatrix(-3.14159265f * 0.5f) * m_globalRotation;
					break;
				}

				case UPAXISTYPE_Y_UP:
				default:
				{
				}
			}
		}

		LoadPhysicScene(document, world, sceneList, context, meshCache, modifierVertexMapCache, materialCache, imageCache);
		for (dSceneModelList::dListNode* node = sceneList.GetFirst(); node; node = node->GetNext()) {
			dModel* model = node->GetInfo();
			context->LoaderFixup (model);
		}
	}




	private:

	dMatrix GetMatrix (domNode* node) const
	{
		dMatrix matrix (GetIdentityMatrix());
		domMatrix_Array matrixArray = node->getMatrix_array();
		if (matrixArray.getCount()) {
			// parce the matrix by concatenating each of the individual components	
			for (int i = 0; i < int (matrixArray.getCount()); i ++) {
				const domFloat4x4& data = matrixArray[i]->getValue();
				dMatrix colladaMatrix; 
				for (int j = 0; j < 4; j ++) {
					for (int k = 0; k < 4; k ++) {
						colladaMatrix[k][j] = dFloat (data[j * 4 + k]);
					}
				}
				matrix = colladaMatrix * matrix;
			}
		} else {
			// or parse the matrix by concatenating each of the individual components
			domTranslate_Array &translationArray = node->getTranslate_array();
			for (int i = 0; i < int (translationArray.getCount()); i ++) {
				const domFloat3& data = translationArray[i]->getValue();
				matrix[3][0] += dFloat (data[0]);
				matrix[3][1] += dFloat (data[1]);
				matrix[3][2] += dFloat (data[2]);
			}

			domRotate_Array &rotationArray = node->getRotate_array();
			for (int i = 0; i < int (rotationArray.getCount()); i ++) {
				dFloat angle;
				const domFloat4& data = rotationArray[i]->getValue();
				angle = dFloat (data[3]) * 3.1316f / 180.0f;
				dFloat x = dFloat (data[0]);
				dFloat y = dFloat (data[1]);
				dFloat z = dFloat (data[2]);
				dVector axis (x, y, z, 0.0f);
				axis = axis.Scale (1.0f / sqrtf (axis % axis));
				dQuaternion rot (axis, angle);
				dMatrix colladaMatrix (rot, dVector (0.0f, 0.0f, 0.0f, 1.0f));
				matrix = colladaMatrix * matrix;
			}
		}
		return matrix;
	}


	void AddAsset (daeDocument *document)
	{
		domAsset *asset;
		domCOLLADA *domRoot;
		domAsset::domCreated *created;
		domAsset::domModified *modified;
		domAsset::domUnit *unit;
		domAsset::domUp_axis* axis;
		domAsset::domContributor* contributor;
		domAsset::domContributor::domAuthor* autor;

		domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		asset = daeSafeCast<domAsset>(domRoot->createAndPlace(COLLADA_ELEMENT_ASSET));


		contributor = daeSafeCast<domAsset::domContributor>(asset->createAndPlace(COLLADA_ELEMENT_CONTRIBUTOR));
		autor = daeSafeCast<domAsset::domContributor::domAuthor>(contributor->createAndPlace(COLLADA_ELEMENT_AUTHOR));
		autor->setValue("Newton Game Dynamics 2.00 Collada Export");

		created = daeSafeCast<domAsset::domCreated>(asset->createAndPlace(COLLADA_ELEMENT_CREATED));

		time_t rawtime;
		tm * ptm;
		time ( &rawtime );
		ptm = gmtime ( &rawtime );

		char date[256];
		sprintf (date, "%d-%02d-%02dT%02d:%02d:%02dZ", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

		//created->setValue("2010-01-04T15:56:43Z");
		created->setValue(date);

		modified = daeSafeCast<domAsset::domModified>(asset->createAndPlace(COLLADA_ELEMENT_MODIFIED));
		modified->setValue(date);

		unit = daeSafeCast<domAsset::domUnit>(asset->createAndPlace(COLLADA_ELEMENT_UNIT));
		unit->setName("meter");
		unit->setMeter (1.0);

		axis = daeSafeCast<domAsset::domUp_axis>(asset->createAndPlace(COLLADA_ELEMENT_UP_AXIS));
		axis->setValue(UPAXISTYPE_Y_UP);
//		axis->setValue(UPAXISTYPE_Z_UP);
	}


	void AddCameraLibrary (daeDocument* document, const dModel* model)
	{
/*
		domCOLLADA *domRoot;
		CameraNode* camera;
		domLibrary_cameras *library;
		domCamera::domOptics* optics;
		domCamera::domOptics::domTechnique_common* technique;
		domCamera::domOptics::domTechnique_common::domPerspective* perpective;

		camera = (CameraNode*) rootNode->Find(CAMERA_NAME);
		//		_ASSERTE (camera);
		if (camera) {
			domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());

			library = daeSafeCast<domLibrary_cameras>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_CAMERAS));

			m_collCamera = daeSafeCast<domCamera>(library->createAndPlace(COLLADA_ELEMENT_CAMERA));
			m_collCamera->setId(CAMERA_NAME);
			optics = daeSafeCast<domCamera::domOptics>(m_collCamera->createAndPlace(COLLADA_ELEMENT_OPTICS));
			technique = daeSafeCast<domCamera::domOptics::domTechnique_common>(optics->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
			perpective = daeSafeCast<domCamera::domOptics::domTechnique_common::domPerspective>(technique->createAndPlace(COLLADA_ELEMENT_PERSPECTIVE));

			domTargetableFloat* fov; 
			domTargetableFloat* aspect; 
			domTargetableFloat* farPlane;
			domTargetableFloat* nearPlane;

			fov = daeSafeCast<domTargetableFloat>(perpective->createAndPlace(COLLADA_ELEMENT_YFOV));
			aspect = daeSafeCast<domTargetableFloat>(perpective->createAndPlace(COLLADA_ELEMENT_ASPECT_RATIO));
			nearPlane = daeSafeCast<domTargetableFloat>(perpective->createAndPlace(COLLADA_ELEMENT_ZNEAR));
			farPlane = daeSafeCast<domTargetableFloat>(perpective->createAndPlace(COLLADA_ELEMENT_ZFAR));

			fov->setValue(camera->m_fov);
			aspect->setValue(camera->m_aspect);
			nearPlane->setValue(camera->m_nearPlane);
			farPlane->setValue(camera->m_farPlane);
		}
*/
	}


	void AddImageLibrary (daeDocument* document, const dModel* model)
	{
		domCOLLADA *domRoot;
		domLibrary_images* library;

		domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		if (domRoot->getLibrary_images_array().getCount()) {
			library = domRoot->getLibrary_images_array()[0];
		} else {
			library = daeSafeCast<domLibrary_images>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_IMAGES));
		}

		for (dList<dMeshInstance>::dListNode* node = model->m_meshList.GetFirst(); node; node = node->GetNext()) { 
			dMeshInstance& intance = node->GetInfo();
			if (!intance.IsIntance()) {
				dMesh* mesh = intance.m_mesh;
				for (dMesh::dListNode* subMeshNode = mesh->GetFirst(); subMeshNode; subMeshNode = subMeshNode->GetNext()) {
					int crcCode;
					char name[256];
					char texName[256];
					dSubMesh* subMesh = &subMeshNode->GetInfo();
					
					mesh->GetTextureName(subMesh, texName);
					crcCode = dCRC (texName);

					sprintf (name, "%s%x", D_TEXTURE_PREFIX_NAME, crcCode);
					if (!FindImage (library, name)) {
						domImage* image = daeSafeCast<domImage>(library->createAndPlace(COLLADA_ELEMENT_IMAGE));
						image->setId (name);

						domImage::domInit_from* imagefrom = daeSafeCast<domImage::domInit_from>(image->createAndPlace(COLLADA_ELEMENT_INIT_FROM));
						imagefrom->setValue (texName);
					}
				}
			}
		}
	}


	void AddDefaultMaterial (domLibrary_effects* effectLibrary, domLibrary_materials* materialLibrary)
	{
		domEffect *effect;
		domMaterial* material;
		domInstance_effect *instanceEffect;
		char materialName[128];

		sprintf (materialName, "%s%x", D_MATERIAL_PREFIX_NAME, dCRC (0));

		material = daeSafeCast<domMaterial>(materialLibrary->createAndPlace(COLLADA_ELEMENT_MATERIAL));
		material->setId (materialName);

		// create an effect and add instance to the material
		effect = daeSafeCast<domEffect>(effectLibrary->createAndPlace(COLLADA_ELEMENT_EFFECT));
		sprintf (materialName, "%s%x", D_MATERIAL_PREFIX_NAME_EFFECT, dCRC (0));
		effect->setId (materialName);

		instanceEffect = daeSafeCast<domInstance_effect>(material->createAndPlace(COLLADA_ELEMENT_INSTANCE_EFFECT ));
		daeURI uri (*effect);
		uri.set("", "", "", "", effect->getId());
		//uri.setElement (effect);
		//uri.resolveURI();
		instanceEffect->setUrl (uri);

		// add properties to the effect
		// need to copy the material properties into the effect, 
		//const dgMaxMaterial& myMaterial = *iter;
		domProfile_COMMON *profile = daeSafeCast<domProfile_COMMON>(effect->createAndPlace(COLLADA_ELEMENT_PROFILE_COMMON));

		domProfile_COMMON::domTechnique *technique = daeSafeCast<domProfile_COMMON::domTechnique>(profile->createAndPlace(COLLADA_ELEMENT_TECHNIQUE));
		technique->setSid("common");
		domProfile_COMMON::domTechnique::domLambert *lambert = daeSafeCast<domProfile_COMMON::domTechnique::domLambert>(technique->createAndPlace(COLLADA_ELEMENT_LAMBERT));

		domCommon_color_or_texture_type *diffuse = daeSafeCast<domCommon_color_or_texture_type>(lambert->createAndPlace(COLLADA_ELEMENT_DIFFUSE));
		domCommon_color_or_texture_type::domColor *color = daeSafeCast<domCommon_color_or_texture_type::domColor>(diffuse->createAndPlace(COLLADA_ELEMENT_COLOR));
		domFx_color_common& value = color->getValue();
		color->getValue().append(0.7f);
		color->getValue().append(0.7f);
		color->getValue().append(0.7f);
		color->getValue().append(1.0f);
		value.setCount(4);
		value[0] = 0.7f;
		value[1] = 0.7f;
		value[2] = 0.7f;
		value[3] = 1.0f;
	}


	void AddMaterialLibrary (daeDocument* document, const dModel* model)
	{
		domCOLLADA *domRoot;
		domLibrary_effects* effectLibrary;
		domLibrary_materials* materialLibrary;

		domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot() );

		domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());

		// create an material and effect library
		if (domRoot->getLibrary_effects_array().getCount()) {
			effectLibrary = domRoot->getLibrary_effects_array()[0];
		} else {
			effectLibrary = daeSafeCast<domLibrary_effects>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_EFFECTS));
		}

		if (domRoot->getLibrary_materials_array().getCount()) {
			materialLibrary = domRoot->getLibrary_materials_array()[0];
		} else {
			materialLibrary = daeSafeCast<domLibrary_materials>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_MATERIALS));
			AddDefaultMaterial (effectLibrary, materialLibrary);
		}

		for (dList<dMeshInstance>::dListNode* node = model->m_meshList.GetFirst(); node; node = node->GetNext()) { 
			dMeshInstance& instance = node->GetInfo();
			if (!instance.IsIntance()) {
				dMesh* mesh = instance.m_mesh;
				for (dMesh::dListNode* subMeshNode = mesh->GetFirst(); subMeshNode; subMeshNode = subMeshNode->GetNext()) {
					dMesh* mesh = node->GetInfo().m_mesh;
					int crcCode;
					dSubMesh* subMesh = &subMeshNode->GetInfo();

					char name[256];
					char materialName[256];

					mesh->GetTextureName(subMesh, name);
					crcCode = dCRC (name);
					sprintf (materialName, "%s%x", D_MATERIAL_PREFIX_NAME, crcCode);

					if (!FindMaterial(materialLibrary, materialName)) {
						domEffect *effect;
						domMaterial* material;
						domInstance_effect *instanceEffect;

						material = daeSafeCast<domMaterial>(materialLibrary->createAndPlace(COLLADA_ELEMENT_MATERIAL));
						material->setId (materialName);

						// create an effect and add instance to the material
						effect = daeSafeCast<domEffect>(effectLibrary->createAndPlace(COLLADA_ELEMENT_EFFECT));
						sprintf (materialName, "%s%x", D_MATERIAL_PREFIX_NAME_EFFECT, crcCode);
						effect->setId (materialName);

						instanceEffect = daeSafeCast<domInstance_effect>(material->createAndPlace(COLLADA_ELEMENT_INSTANCE_EFFECT ));
						daeURI uri (*effect);
						uri.set("", "", "", "", effect->getId());
						//						uri.resolveURI();
						instanceEffect->setUrl( uri );

						// add properties to the effect
						// need to copy the material properties into the effect, 
						//const dgMaxMaterial& myMaterial = *iter;
						domProfile_COMMON *profile = daeSafeCast<domProfile_COMMON>(effect->createAndPlace(COLLADA_ELEMENT_PROFILE_COMMON));

						domProfile_COMMON::domTechnique *technique = daeSafeCast<domProfile_COMMON::domTechnique>(profile->createAndPlace(COLLADA_ELEMENT_TECHNIQUE));
						technique->setSid("common");
						domProfile_COMMON::domTechnique::domLambert *lambert = daeSafeCast<domProfile_COMMON::domTechnique::domLambert>(technique->createAndPlace(COLLADA_ELEMENT_LAMBERT));

						domCommon_color_or_texture_type *diffuse = daeSafeCast<domCommon_color_or_texture_type>(lambert->createAndPlace(COLLADA_ELEMENT_DIFFUSE));
						if (!crcCode) {
							domCommon_color_or_texture_type::domColor *color = daeSafeCast<domCommon_color_or_texture_type::domColor>(diffuse->createAndPlace(COLLADA_ELEMENT_COLOR));
							domFx_color_common& value = color->getValue();
							value.setCount(4);
							value[0] = 0.7f;
							value[1] = 0.7f;
							value[2] = 0.7f;
							value[3] = 1.0f;
//							domCommon_color_or_texture_typeRef elemEmission;
//							domCommon_color_or_texture_typeRef elemAmbient;
//							domCommon_color_or_texture_typeRef elemDiffuse;
//							domCommon_color_or_texture_typeRef elemReflective;
//							domCommon_float_or_param_typeRef elemReflectivity;
//							domCommon_transparent_typeRef elemTransparent;
//							domCommon_float_or_param_typeRef elemTransparency;
//							domCommon_float_or_param_typeRef elemIndex_of_refraction;

						} else {
							char nameID [128];
							char nameIDimage [128];
							domCommon_newparam_type *np;

							sprintf (nameID, "%s%x", D_TEXTURE_PREFIX_NAME, crcCode);							
							sprintf (nameIDimage, "%s_image", nameID);

							np = daeSafeCast<domCommon_newparam_type>(profile->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
							np->setSid (nameIDimage);
							domFx_surface_common *surface = daeSafeCast<domFx_surface_common>(np->createAndPlace(COLLADA_ELEMENT_SURFACE));
							surface->setType( FX_SURFACE_TYPE_ENUM_2D );
							domFx_surface_init_from_common *surfIF = daeSafeCast<domFx_surface_init_from_common>(surface->createAndPlace(COLLADA_ELEMENT_INIT_FROM));
							surfIF->setValue (nameID);
							domFx_surface_common_complexType::domFormat *format = daeSafeCast<domFx_surface_common_complexType::domFormat>(surface->createAndPlace(COLLADA_ELEMENT_FORMAT));
							format->setValue("A8R8G8B8");

							np = daeSafeCast<domCommon_newparam_type>(profile->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
							//np->setSid( "sampler" );
							np->setSid (nameID);
							domFx_sampler2D_common *sampler = daeSafeCast<domFx_sampler2D_common>(np->createAndPlace(COLLADA_ELEMENT_SAMPLER2D));
							domFx_sampler2D_common::domSource *sampSrc = daeSafeCast<domFx_sampler2D_common::domSource>(sampler->createAndPlace(COLLADA_ELEMENT_SOURCE));
							sampSrc->setValue( nameIDimage );

							domCommon_color_or_texture_type::domTexture *texture = daeSafeCast<domCommon_color_or_texture_type::domTexture>(diffuse->createAndPlace(COLLADA_ELEMENT_TEXTURE));
							texture->setTexture( nameID );
							texture->setTexcoord ("CHANNEL1");
						}
					}
				}
			}
		}
	}



	domGeometry *AddConvexGeometry (daeDocument* document, const dMesh* geometry)
	{
		domLibrary_geometries *library;

		domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		if (domRoot->getLibrary_geometries_array().getCount()) {
			library = domRoot->getLibrary_geometries_array()[0];
		} else {
			library = daeSafeCast<domLibrary_geometries>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_GEOMETRIES));
		}
		_ASSERTE (library);

		domGeometry *collGeometry = daeSafeCast<domGeometry>(library->createAndPlace(COLLADA_ELEMENT_GEOMETRY ));


		char meshName[256];
		geometry->GetName(meshName);
		collGeometry->setId(meshName);
		domConvex_mesh *colladaMesh = daeSafeCast<domConvex_mesh>(collGeometry->createAndPlace(COLLADA_ELEMENT_CONVEX_MESH));


		domVertices *verts;
		int* vertexArrayIndexList = new int[geometry->m_vertexCount];
		{
			// add the vertices
			char text[256];
			int vertexCount;
			domSource *posSource;
			float* floatPool = new float[geometry->m_vertexCount * 3];

			posSource = daeSafeCast<domSource>(colladaMesh->createAndPlace(COLLADA_ELEMENT_SOURCE));
			sprintf (text, "%s_position", meshName);
			posSource->setId (text);

			for (int i = 0; i < geometry->m_vertexCount; i ++) {
				floatPool[i * 3 + 0] = m_scale * geometry->m_vertex[i * 3 + 0];
				floatPool[i * 3 + 1] = m_scale * geometry->m_vertex[i * 3 + 1];
				floatPool[i * 3 + 2] = m_scale * geometry->m_vertex[i * 3 + 2];
			}

			vertexCount = dModel::dPackVertexArray (floatPool, 3, 3 * sizeof (dFloat), geometry->m_vertexCount, vertexArrayIndexList);
			m_globalRotation.TransformTriplex(floatPool, sizeof (float) * 3, floatPool, sizeof (float) * 3, geometry->m_vertexCount);

			domFloat_array *fa = daeSafeCast<domFloat_array>(posSource->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
			strcat (text, "Array");
			fa->setId( text );
			fa->setCount (vertexCount * 3);

			domListOfFloats &posSrcArray = fa->getValue();
			for (int i = 0; i < vertexCount; i ++) {
				posSrcArray.append3( floatPool[i * 3 + 0], floatPool[i * 3 + 1], floatPool[i * 3 + 2]);
			}

			//create the accessor
			domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(posSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
			domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
			acc->setCount (vertexCount);
			acc->setStride (3);
			daeURI uri(*fa);
			uri.set("", "", "", "", fa->getId());
			acc->setSource( uri );

			domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName( "x" );
			param->setType( "float" );
			param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName( "y" );
			param->setType( "float" );
			param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName( "z" );
			param->setType( "float" );

			verts = daeSafeCast<domVertices>(colladaMesh->createAndPlace(COLLADA_ELEMENT_VERTICES));
			sprintf (text, "%sVertices", geometry->m_name);
			verts->setId (text);
			domInputLocal *inputLocal = daeSafeCast<domInputLocal>(verts->createAndPlace(COLLADA_ELEMENT_INPUT));
			inputLocal->setSemantic (COMMON_PROFILE_INPUT_POSITION);
			daeURI uri1(*posSource);
			uri1.set("", "", "", "", posSource->getId());
			inputLocal->setSource (uri1);

			delete[] floatPool;
		}


		for (dMesh::dListNode* node = geometry->GetFirst(); node; node = node->GetNext()) {
			char texName[256];
			char materialName[256];
			domTriangles *polys1;
			domInputLocalOffset *input;

			const dSubMesh& subMesh = node->GetInfo();
			geometry->GetTextureName(&subMesh, texName);
			sprintf (materialName, "%s%x", D_MATERIAL_PREFIX_NAME, dCRC (texName));

			polys1 = daeSafeCast<domTriangles>(colladaMesh->createAndPlace(COLLADA_ELEMENT_TRIANGLES));
			polys1->setCount (subMesh.m_indexCount / 3);
			polys1->setMaterial (materialName);

			input = daeSafeCast<domInputLocalOffset>(polys1->createAndPlace(COLLADA_ELEMENT_INPUT ));
			input->setSemantic( COMMON_PROFILE_INPUT_VERTEX);
			input->setOffset (0);
			daeURI uri (*verts);
			uri.set("", "", "", "", verts->getId());
			input->setSource (uri);

//			input = daeSafeCast<domInputLocalOffset>(polys1->createAndPlace(COLLADA_ELEMENT_INPUT));
//			input->setSemantic( COMMON_PROFILE_INPUT_NORMAL );
//			input->setOffset (1);
//			daeURI uri1 (*normalSrc);
//			uri1.set("", "", "", "", normalSrc->getId());
//			input->setSource (uri1);

//			input = daeSafeCast<domInputLocalOffset>(polys1->createAndPlace(COLLADA_ELEMENT_INPUT));
//			input->setSemantic (COMMON_PROFILE_INPUT_TEXCOORD);
//			input->setOffset (2);
//			input->setSet (0);
//			daeURI uri2 (*uvSource);
//			uri2.set("", "", "", "", uvSource->getId());
//			input->setSource (uri2);

			domP *p = daeSafeCast<domP>(polys1->createAndPlace(COLLADA_ELEMENT_P));
			domListOfUInts &indices = p->getValue();
			for (int i = 0; i < subMesh.m_indexCount; i ++) {
				int index = subMesh.m_indexes[i];
				indices.append (vertexArrayIndexList[index]);
			}
		}

		delete[] vertexArrayIndexList;

		return collGeometry;
	}



	domGeometry *AddGeometry (daeDocument* document, const dMesh* geometry)
	{
		domLibrary_geometries *library;

		domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		if (domRoot->getLibrary_geometries_array().getCount()) {
			library = domRoot->getLibrary_geometries_array()[0];
		} else {
			library = daeSafeCast<domLibrary_geometries>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_GEOMETRIES));
		}
		_ASSERTE (library);

		domGeometry *collGeometry = daeSafeCast<domGeometry>(library->createAndPlace(COLLADA_ELEMENT_GEOMETRY ));

		char meshName[256];
		geometry->GetName(meshName);
		collGeometry->setId(meshName);
		domMesh *colladaMesh = daeSafeCast<domMesh>(collGeometry->createAndPlace(COLLADA_ELEMENT_MESH ));

		domVertices *verts;
		int* vertexArrayIndexList = new int[geometry->m_vertexCount];
		{
			// add the vertices
			char text[256];
			int vertexCount;
			domSource *posSource;
			float* floatPool = new float[geometry->m_vertexCount * 3];

			posSource = daeSafeCast<domSource>(colladaMesh->createAndPlace(COLLADA_ELEMENT_SOURCE));
			sprintf (text, "%s_position", meshName);
			posSource->setId (text);

			for (int i = 0; i < geometry->m_vertexCount; i ++) {
				floatPool[i * 3 + 0] = m_scale * geometry->m_vertex[i * 3 + 0];
				floatPool[i * 3 + 1] = m_scale * geometry->m_vertex[i * 3 + 1];
				floatPool[i * 3 + 2] = m_scale * geometry->m_vertex[i * 3 + 2];
			}

			vertexCount = dModel::dPackVertexArray (floatPool, 3, 3 * sizeof (dFloat), geometry->m_vertexCount, vertexArrayIndexList);
			m_globalRotation.TransformTriplex(floatPool, sizeof (float) * 3, floatPool, sizeof (float) * 3, geometry->m_vertexCount);

			domFloat_array *fa = daeSafeCast<domFloat_array>(posSource->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
			strcat (text, "Array");
			fa->setId( text );
			fa->setCount (vertexCount * 3);

			domListOfFloats &posSrcArray = fa->getValue();
			for (int i = 0; i < vertexCount; i ++) {
				posSrcArray.append3( floatPool[i * 3 + 0], floatPool[i * 3 + 1], floatPool[i * 3 + 2]);
			}

			//create the accessor
			domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(posSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
			domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
			acc->setCount (vertexCount);
			acc->setStride (3);
			daeURI uri(*fa);
			uri.set("", "", "", "", fa->getId());
			//uri.setElement (fa);
			//uri.resolveURI();
			acc->setSource( uri );

			domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName( "x" );
			param->setType( "float" );
			param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName( "y" );
			param->setType( "float" );
			param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName( "z" );
			param->setType( "float" );


			verts = daeSafeCast<domVertices>(colladaMesh->createAndPlace(COLLADA_ELEMENT_VERTICES));
			sprintf (text, "%sVertices", geometry->m_name);
			verts->setId (text);
			domInputLocal *inputLocal = daeSafeCast<domInputLocal>(verts->createAndPlace(COLLADA_ELEMENT_INPUT));
			inputLocal->setSemantic (COMMON_PROFILE_INPUT_POSITION);
			//uri.setElement (posSource);
			daeURI uri1(*posSource);
			uri1.set("", "", "", "", posSource->getId());
			//uri.resolveURI();
			inputLocal->setSource (uri1);

			delete[] floatPool;
		}


		domSource *normalSrc;
		int* normalArrayIndexList = new int[geometry->m_vertexCount];
		{
			char text[256];
			int vertexCount;
			float* floatPool = new float[geometry->m_vertexCount * 3];

			normalSrc = daeSafeCast<domSource>(colladaMesh->createAndPlace(COLLADA_ELEMENT_SOURCE));
			sprintf (text, "%s_normal", meshName);
			normalSrc->setId (text);

			for (int i = 0; i < geometry->m_vertexCount; i ++) {
				floatPool[i * 3 + 0] = geometry->m_normal[i * 3 + 0];
				floatPool[i * 3 + 1] = geometry->m_normal[i * 3 + 1];
				floatPool[i * 3 + 2] = geometry->m_normal[i * 3 + 2];
			}
			vertexCount = dModel::dPackVertexArray (floatPool, 3, 3 * sizeof (dFloat), geometry->m_vertexCount, normalArrayIndexList);
			dMatrix rot (m_globalRotation);
			rot.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
			rot.TransformTriplex(floatPool, sizeof (float) * 3, floatPool, sizeof (float) * 3, geometry->m_vertexCount);

			domFloat_array *fa = daeSafeCast<domFloat_array>(normalSrc->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
			strcat (text, "Array");
			fa->setId (text);
			fa->setCount (vertexCount * 3);

			domListOfFloats &posSrcArray = fa->getValue();
			for (int i = 0; i < vertexCount; i ++) {
				posSrcArray.append3( floatPool[i * 3 + 0], floatPool[i * 3 + 1], floatPool[i * 3 + 2]);
			}

			//create the accessors
			domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(normalSrc->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
			domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
			acc->setCount (vertexCount);
			acc->setStride (3);
			daeURI uri(*fa);
			uri.set("", "", "", "", fa->getId());
			//uri.setElement (fa);
			//uri.resolveURI();
			acc->setSource( uri );

			domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName( "x" );
			param->setType( "float" );
			param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName( "y" );
			param->setType( "float" );
			param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName( "z" );
			param->setType( "float" );

			delete[] floatPool;
		}

		domSource *uvSource;
		int* uv0ArrayIndexList = new int[geometry->m_vertexCount];
		{
			char text[256];
			int vertexCount;
			float* floatPool = new float[geometry->m_vertexCount * 3];

			uvSource = daeSafeCast<domSource>(colladaMesh->createAndPlace(COLLADA_ELEMENT_SOURCE));
			sprintf (text, "%s_uv", meshName);
			uvSource->setId (text);
			for (int i = 0; i < geometry->m_vertexCount; i ++) {
				floatPool[i * 3 + 0] = geometry->m_uv[i * 2 + 0];
				floatPool[i * 3 + 1] = geometry->m_uv[i * 2 + 1];
				floatPool[i * 3 + 2] = 0.0f;
			}
			vertexCount = dModel::dPackVertexArray (floatPool, 3, 3 * sizeof (dFloat), geometry->m_vertexCount, uv0ArrayIndexList);

			domFloat_array *fa = daeSafeCast<domFloat_array>(uvSource->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
			strcat (text, "Array");
			fa->setId (text);
			fa->setCount(vertexCount * 2);
			domListOfFloats &posSrcArray = fa->getValue();
			for (int i = 0; i < vertexCount; i ++) {
				posSrcArray.append2( floatPool[i * 3 + 0], floatPool[i * 3 + 1]);
			}

			domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(uvSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
			domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
			acc->setCount (vertexCount);
			acc->setStride( 2 );
			daeURI uri(*fa);
			uri.set("", "", "", "", fa->getId());
			//uri.setElement( fa );
			//uri.resolveURI();
			acc->setSource( uri );

			domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName( "s" );
			param->setType( "float" );
			param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName( "t" );
			param->setType( "float" );

			delete[] floatPool;
		}

		for (dMesh::dListNode* node = geometry->GetFirst(); node; node = node->GetNext()) {
			char texName[256];
			char materialName[256];
			domTriangles *polys1;
			domInputLocalOffset *input;


			const dSubMesh& subMesh = node->GetInfo();
//			sprintf (materialName, "%s%x", D_MATERIAL_PREFIX_NAME, dCRC (subMesh.m_textureName));
			geometry->GetTextureName(&subMesh, texName);
			sprintf (materialName, "%s%x", D_MATERIAL_PREFIX_NAME, dCRC (texName));

			polys1 = daeSafeCast<domTriangles>(colladaMesh->createAndPlace(COLLADA_ELEMENT_TRIANGLES));
			polys1->setCount (subMesh.m_indexCount / 3);
			polys1->setMaterial (materialName);


			input = daeSafeCast<domInputLocalOffset>(polys1->createAndPlace(COLLADA_ELEMENT_INPUT ));
			input->setSemantic( COMMON_PROFILE_INPUT_VERTEX );
			input->setOffset( 0 );
			daeURI uri (*verts);
			uri.set("", "", "", "", verts->getId());
			//uri.setElement (verts);
			//uri.resolveURI();
			input->setSource (uri);


			input = daeSafeCast<domInputLocalOffset>(polys1->createAndPlace(COLLADA_ELEMENT_INPUT));
			input->setSemantic( COMMON_PROFILE_INPUT_NORMAL );
			input->setOffset (1);
			daeURI uri1 (*normalSrc);
			uri1.set("", "", "", "", normalSrc->getId());
			//uri.setElement (normalSrc);
			//uri.resolveURI();
			input->setSource (uri1);

			input = daeSafeCast<domInputLocalOffset>(polys1->createAndPlace(COLLADA_ELEMENT_INPUT));
			input->setSemantic (COMMON_PROFILE_INPUT_TEXCOORD);
			input->setOffset (2);
			//			input->setSet (1);
			input->setSet (0);
			daeURI uri2 (*uvSource);
			uri2.set("", "", "", "", uvSource->getId());
			//uri.setElement (uvSource);
			//uri.resolveURI();
			input->setSource (uri2);

			domP *p = daeSafeCast<domP>(polys1->createAndPlace(COLLADA_ELEMENT_P));
			domListOfUInts &indices = p->getValue();
			for (int i = 0; i < subMesh.m_indexCount; i ++) {
				int index = subMesh.m_indexes[i];
				indices.append3 (vertexArrayIndexList[index], normalArrayIndexList[index], uv0ArrayIndexList[index]);
			}
		}

		delete[] uv0ArrayIndexList;
		delete[] normalArrayIndexList;
		delete[] vertexArrayIndexList;

		return collGeometry;
	}


	void AddGeometryLibrary (daeDocument* document, const dModel* model)
	{
		for (dList<dMeshInstance>::dListNode* node = model->m_meshList.GetFirst(); node; node = node->GetNext()) { 
			dMeshInstance& instance = node->GetInfo();
			if (!instance.IsIntance()) {
				dMesh* mesh = node->GetInfo().m_mesh;
				domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
				if (domRoot->getLibrary_geometries_array().getCount()) {
					domLibrary_geometries *library = domRoot->getLibrary_geometries_array()[0];
					char meshName[256];
					//sprintf (meshName, "%s", mesh->GetName());
					mesh->GetName(meshName);
					if (!FindGeometry (library, meshName)) {
						AddGeometry (document, mesh);
					}

				} else {
					AddGeometry (document, mesh);
				}
			}
		}
	}


	void AddAnimation (daeDocument* document, dAnimationClip* clip, const dModel* model)
	{
		domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());

		if (!domRoot->getLibrary_animations_array().getCount()) {
			daeSafeCast<domLibrary_animations>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_ANIMATIONS));
		}

		domLibrary_animations *library = domRoot->getLibrary_animations_array()[0];
		_ASSERTE (library);

		domAnimation *animation = daeSafeCast<domAnimation>(library->createAndPlace(COLLADA_ELEMENT_ANIMATION ));

		char nameID[256];
		sprintf (nameID, "%s_anim", clip->m_name);							
		animation->setId(nameID);
		animation->setName(clip->m_name);

		for (dAnimationClip::dListNode* node = clip->GetFirst(); node; node = node->GetNext()) {
			char text[256];

			dKeyFrames* track = &node->GetInfo();
			domSource *timeSource = daeSafeCast<domSource>(animation->createAndPlace(COLLADA_ELEMENT_SOURCE));
			{
				sprintf (text, "%s_anim_time", track->m_bindName);
				timeSource->setId (text);
				domFloat_array *fa = daeSafeCast<domFloat_array>(timeSource->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
				strcat (text, "Array");
				fa->setId (text);
				fa->setCount (track->m_keyFrameCounts);
				domListOfFloats &posSrcArray = fa->getValue();
				for (int i = 0; i < track->m_keyFrameCounts; i ++) {
					posSrcArray.append(track->m_keys[i]);
				}

				//create the accessor
				domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(timeSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
				domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
				acc->setCount (track->m_keyFrameCounts);
				acc->setStride (1);
				daeURI uri(*fa);
				uri.set("", "", "", "", fa->getId());
				acc->setSource( uri );

				domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
				param->setName( "key" );
				param->setType( "float" );
			}


			domSource *transfromSource = daeSafeCast<domSource>(animation->createAndPlace(COLLADA_ELEMENT_SOURCE));
			{
				sprintf (text, "%s_anim_transform", track->m_bindName);
				transfromSource->setId (text);
				domFloat_array *fa = daeSafeCast<domFloat_array>(transfromSource->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
				strcat (text, "Array");
				fa->setId (text);
				fa->setCount (track->m_keyFrameCounts * 16);
				domListOfFloats &posSrcArray = fa->getValue();
				for (int i = 0; i < track->m_keyFrameCounts; i ++) {
					dMatrix matrix (track->m_rotation[i], track->m_posit[i]);
					matrix.m_posit = matrix.m_posit.Scale (m_scale);
					matrix.m_posit.m_w = 1.0f;
					matrix = (m_globalRotation.Inverse() * matrix * m_globalRotation);
					for (int j = 0; j < 4; j ++) {
						for (int k = 0; k < 4; k ++) {
							posSrcArray.append(matrix[k][j]);
						}
					}
				}

				//create the accessor
				domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(transfromSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
				domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
				acc->setCount (track->m_keyFrameCounts);
				acc->setStride (16);
				daeURI uri(*fa);
				uri.set("", "", "", "", fa->getId());
				acc->setSource( uri );

				domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
				param->setName( "matrix" );
				param->setType( "float4x4" );
			}

			domSource *interpSource = daeSafeCast<domSource>(animation->createAndPlace(COLLADA_ELEMENT_SOURCE));
			{
				sprintf (text, "%s_anim_transform_intepolation", track->m_bindName);
				interpSource->setId (text);

				domName_array *interpNames = daeSafeCast<domName_array>(interpSource->createAndPlace(COLLADA_ELEMENT_NAME_ARRAY));
				strcat (text, "Array");
				interpNames->setId (text);
				interpNames->setCount (track->m_keyFrameCounts);
				domListOfNames &srcArray = interpNames->getValue();
				for (int i = 0; i < track->m_keyFrameCounts; i ++) {
					const char* name = "LINEAR";
					srcArray.append(name);
				}

				//create the accessor
				domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(interpSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
				domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
				acc->setCount (track->m_keyFrameCounts);
				acc->setStride (1);
				daeURI uri (*interpNames);
				uri.set("", "", "", "", interpNames->getId());
				acc->setSource (uri);
				domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
				param->setName( "interpolation" );
				param->setType( "name" );
			}

			domSampler *sampler = daeSafeCast<domSampler>(animation->createAndPlace(COLLADA_ELEMENT_SAMPLER));
			sprintf (text, "%s_anim_sample", track->m_bindName);
			sampler->setId (text);

			domInputLocal *inputLocal = daeSafeCast<domInputLocal>(sampler->createAndPlace(COLLADA_ELEMENT_INPUT));
			inputLocal->setSemantic (COMMON_PROFILE_INPUT_INPUT);
			daeURI uri0 (*timeSource);
			uri0.set("", "", "", "", timeSource->getId());
			inputLocal->setSource (uri0);

			inputLocal = daeSafeCast<domInputLocal>(sampler->createAndPlace(COLLADA_ELEMENT_INPUT));
			inputLocal->setSemantic (COMMON_PROFILE_INPUT_OUTPUT);
			daeURI uri1 (*transfromSource);
			uri1.set("", "", "", "", transfromSource->getId());
			inputLocal->setSource (uri1);

			inputLocal = daeSafeCast<domInputLocal>(sampler->createAndPlace(COLLADA_ELEMENT_INPUT));
			inputLocal->setSemantic (COMMON_PROFILE_INPUT_INTERPOLATION);
			daeURI uri2 (*interpSource);
			uri2.set("", "", "", "", interpSource->getId());
			inputLocal->setSource (uri2);


			domChannel *channel = daeSafeCast<domChannel>(animation->createAndPlace(COLLADA_ELEMENT_CHANNEL));
			daeURI uri3 (*sampler);
			uri3.set("", "", "", "", sampler->getId());
			channel->setSource (uri3);
			sprintf (text, "%s%s", D_NODE_NAME,track->m_bindName);
			channel->setTarget (text);
		}
	}


	void AddAnimationLibrary(daeDocument* document, const dModel* model)
	{
		for (dList<dAnimationClip*>::dListNode* node = model->m_animations.GetFirst(); node; node = node->GetNext()) { 
			dAnimationClip* clip = node->GetInfo();
			domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
			if (domRoot->getLibrary_animations_array().getCount()) {
				domLibrary_animations *library = domRoot->getLibrary_animations_array()[0];
				if (!FindAnimation (library, clip->m_name)) {
					AddAnimation (document, clip, model);
				}

			} else {
				AddAnimation (document, clip, model);
			}
		}
	}



	void AddController (daeDocument* document, dMeshInstance* meshInstance, const dModel* model)
	{
		domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		if (!domRoot->getLibrary_controllers_array().getCount()) {
			daeSafeCast<domLibrary_geometries>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_CONTROLLERS));
		}
		domLibrary_controllers *controllerLibrary = domRoot->getLibrary_controllers_array()[0];
		_ASSERTE (controllerLibrary);

		domLibrary_geometries *geometryLibrary = domRoot->getLibrary_geometries_array()[0];
		_ASSERTE (geometryLibrary);
		
		const dMesh* mesh = meshInstance->m_mesh;
		dSkinModifier* skinData = (dSkinModifier*) meshInstance->GetModifier();
		const dBone** boneList = skinData->m_skinnedBones;
		dMatrix* bindingMatrices = skinData->m_bindingMatrices;

		domController *controller = daeSafeCast<domController>(controllerLibrary->createAndPlace(COLLADA_ELEMENT_CONTROLLER ));

		
		char text[256];
		char meshName[256];
//		const char* meshName = meshInstance->m_mesh->GetName(mesh);
		mesh->GetName(meshName);
		sprintf (text, "%s%s", meshName, D_SKIN_POS_FIX);
		controller->setId(text);

		domSkin* skin = daeSafeCast<domSkin>(controller->createAndPlace(COLLADA_TYPE_SKIN));

		domGeometry* geoSource = NULL;
		const domGeometry_Array& array = geometryLibrary->getGeometry_array();
		for (int i = 0; i < int (array.getCount()); i ++) {
			geoSource = array[i];
			if (!cdom::strcasecmp (meshName, geoSource->getId())) {
				break;
			}
		}

		daeURI uri (*geoSource);
		uri.set("", "", "", "", geoSource->getId());
		skin->setSource (uri);

		domSkin::domBind_shape_matrix* bindMatrix;
		bindMatrix = daeSafeCast<domSkin::domBind_shape_matrix>(skin->createAndPlace(COLLADA_TYPE_BIND_SHAPE_MATRIX));
		domFloat4x4& value = bindMatrix->getValue();
		value.setCount(16);
		dMatrix matrix (skinData->m_shapeBindMatrix);
		matrix = m_globalRotation.Inverse() * matrix * m_globalRotation;
		matrix.m_posit = matrix.m_posit.Scale (m_scale);
		for (int j = 0; j < 4; j ++) {
			for (int i = 0; i < 4; i ++) {
				value[j * 4 + i] = matrix[i][j];
			}
		}

		domSource *boneSource;
		domSource *bindSource;
		domSource *weightsSource;
		int* weightsIndexMap = new int[mesh->m_vertexCount * 4];
		{
			char text[256];
			bindSource = daeSafeCast<domSource>(skin->createAndPlace(COLLADA_ELEMENT_SOURCE));
			sprintf (text, "%s_bindpose", meshName);
			bindSource->setId (text);

			domFloat_array *boneMatrix = daeSafeCast<domFloat_array>(bindSource->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
			strcat (text, "Array");
			boneMatrix->setId (text);
			boneMatrix->setCount (skinData->m_bonesCount * 16);
			domListOfFloats &boneSrcArray = boneMatrix->getValue();
			for (int i = 0; i < skinData->m_bonesCount; i ++) {
				dMatrix matrix (bindingMatrices[i]);
//				matrix = matrix.Transpose4X4();
				matrix = m_globalRotation.Inverse() * matrix * m_globalRotation;
				matrix.m_posit = matrix.m_posit.Scale (m_scale);
//				float* data = &matrix[0][0];
//				for (int j = 0; j < 16; j ++) {
//					boneSrcArray.append(data[j]);
//				}
				for (int j = 0; j < 4; j ++) {
					for (int i = 0; i < 4; i ++) {
						boneSrcArray.append (matrix[i][j]);
					}
				}
			}

			//create the accessor
			domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(bindSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
			domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
			acc->setCount (skinData->m_bonesCount);
			acc->setStride (16);
//			uri.setElement (bindSource);
			daeURI uri (*boneMatrix);
//			uri.resolveURI();
			uri.set("", "", "", "", boneMatrix->getId());
			acc->setSource (uri);
			domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName ("bindMatrix");
			param->setType ("float4x4");
		}


		{
			char text[256];

			boneSource = daeSafeCast<domSource>(skin->createAndPlace(COLLADA_ELEMENT_SOURCE));
			sprintf (text, "%s_bones", meshName);
			boneSource->setId (text);

			domName_array *boneNames = daeSafeCast<domName_array>(boneSource->createAndPlace(COLLADA_ELEMENT_NAME_ARRAY));
			strcat (text, "Array");
			boneNames->setId (text);
			boneNames->setCount (skinData->m_bonesCount);
			domListOfNames &boneSrcArray = boneNames->getValue();
			for (int i = 0; i < skinData->m_bonesCount; i ++) {
				const char* name = boneList[i]->GetName();
				boneSrcArray.append(name);
			}

			//create the accessor
			domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(boneSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
			domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
			acc->setCount (skinData->m_bonesCount);
			acc->setStride (1);
//			uri.setElement (boneNames);
			daeURI uri (*boneNames);
//			uri.resolveURI();
			uri.set("", "", "", "", boneNames->getId());
			acc->setSource (uri);
			domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName ("bone");
			param->setType ("Name");
		}

		{
			char text[256];
			weightsSource = daeSafeCast<domSource>(skin->createAndPlace(COLLADA_ELEMENT_SOURCE));
			sprintf (text, "%s_weights", meshName);
			weightsSource->setId (text);

			dVector* weightData = new dVector [mesh->m_vertexCount * 4];
			for (int i = 0; i < mesh->m_vertexCount; i ++) {
				for (int j = 0; j < 4; j ++) {
					weightData[i * 4 + j].m_x = skinData->m_vertexWeight[i][j];
					weightData[i * 4 + j].m_y = 0.0f;
					weightData[i * 4 + j].m_z = 0.0f;
				}
			}
			int indexCount = dModel::dPackVertexArray (&weightData[0].m_x, 3, sizeof (dVector), mesh->m_vertexCount * 4, weightsIndexMap);

			domFloat_array *weights = daeSafeCast<domFloat_array>(weightsSource->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
			strcat (text, "Array");
			weights->setId (text);
			weights->setCount (indexCount);
			domListOfFloats &weightsSrcArray = weights->getValue();
			for (int i = 0; i < indexCount; i ++) {
				weightsSrcArray.append(weightData[i].m_x);
			}
			delete[] weightData;

			//create the accessor
			domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(weightsSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
			domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
			acc->setCount (indexCount);
			acc->setStride (1);
//			uri.setElement (weightsSource);
			daeURI uri (*weights);
//			uri.resolveURI();
			uri.set("", "", "", "", weights->getId());
			acc->setSource (uri);
			domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
			param->setName ("weight");
			param->setType ("float");
		}


		domSkin::domJoints *joints = daeSafeCast<domSkin::domJoints>(skin->createAndPlace(COLLADA_ELEMENT_JOINTS));
		domInputLocal *inputLocal = daeSafeCast<domInputLocal>(joints->createAndPlace(COLLADA_ELEMENT_INPUT));
		inputLocal->setSemantic (COMMON_PROFILE_INPUT_JOINT);
//		uri.setElement (boneSource);
		daeURI uri0 (*boneSource);
//		uri.resolveURI();
		uri0.set("", "", "", "", boneSource->getId());
		inputLocal->setSource (uri0);

		inputLocal = daeSafeCast<domInputLocal>(joints->createAndPlace(COLLADA_ELEMENT_INPUT));
		inputLocal->setSemantic (COMMON_PROFILE_INPUT_INV_BIND_MATRIX);
//		uri.setElement (bindSource);
		daeURI uri1 (*bindSource);
//		uri.resolveURI();
		uri1.set("", "", "", "", bindSource->getId());
		inputLocal->setSource (uri1);


		domSkin::domVertex_weights *vertexWeights = daeSafeCast<domSkin::domVertex_weights>(skin->createAndPlace(COLLADA_ELEMENT_VERTEX_WEIGHTS));
		domInputLocalOffset* localInputOffset = daeSafeCast<domInputLocalOffset>(vertexWeights->createAndPlace(COLLADA_ELEMENT_INPUT));
		localInputOffset->setSemantic (COMMON_PROFILE_INPUT_JOINT);
		localInputOffset->setOffset (0);
//		uri.setElement (boneSource);
		daeURI uri2 (*boneSource);
//		uri.resolveURI();
		uri2.set("", "", "", "", boneSource->getId());
		localInputOffset->setSource (uri2);

		localInputOffset = daeSafeCast<domInputLocalOffset>(vertexWeights->createAndPlace(COLLADA_ELEMENT_INPUT));
		localInputOffset->setSemantic (COMMON_PROFILE_INPUT_WEIGHT);
		localInputOffset->setOffset (1);
//		uri.setElement (weightsSource);
		daeURI uri3 (*weightsSource);
//		uri.resolveURI();
		uri3.set("", "", "", "", weightsSource->getId());
		localInputOffset->setSource (uri3);


		int* vertexArrayIndexList = new int[mesh->m_vertexCount];
		dVector* floatPool = new dVector[mesh->m_vertexCount];
		for (int i = 0; i < mesh->m_vertexCount; i ++) {
			floatPool[i] = dVector (mesh->m_vertex[i * 3 + 0], mesh->m_vertex[i * 3 + 1], mesh->m_vertex[i * 3 + 2], dFloat(i));
		}
		int vertexCount = dModel::dPackVertexArray (&floatPool[0].m_x, 3, sizeof (dVector), mesh->m_vertexCount, vertexArrayIndexList);

		vertexWeights->setCount(vertexCount);
		domSkin::domVertex_weights::domV *v;
		domSkin::domVertex_weights::domVcount *vCount;
		v = daeSafeCast<domSkin::domVertex_weights::domV>(vertexWeights->createAndPlace(COLLADA_ELEMENT_V));
		vCount = daeSafeCast<domSkin::domVertex_weights::domVcount>(vertexWeights->createAndPlace(COLLADA_ELEMENT_VCOUNT));

		domListOfInts &vIndices = v->getValue();
		domListOfUInts &vCountIndices = vCount->getValue();
//		for (int i = 0; i < mesh->m_vertexCount; i ++) {
		for (int i = 0; i < vertexCount; i ++) {
			int index;
			index = int (floatPool[i].m_w);
			int count = 0;
			for (int j = 0; j < 4; j ++) {
				count += (skinData->m_vertexWeight[index][j] > 0.0f) ? 1 : 0;
			}
			vCountIndices.append(count);

			for (int j = 0; j < count; j ++) {
				int boneIndex = skinData->m_boneWeightIndex[index].m_index[j];
				vIndices.append(boneIndex);
				vIndices.append(weightsIndexMap[index * 4 + j]);
			}
		}

		delete[] floatPool; 
		delete[] vertexArrayIndexList;
		delete[] weightsIndexMap;
	}


	void AddControllerLibrary(daeDocument* document, const dModel* model)
	{
		for (dList<dMeshInstance>::dListNode* node = model->m_meshList.GetFirst(); node; node = node->GetNext()) { 
			dMeshInstance& instance = node->GetInfo();
			if (instance.m_modifier) {
//				dMesh* mesh = node->GetInfo().m_mesh;
				AddController (document, &instance, model);
			}
		}
	}


	

	domNode *AddNodes(daeDocument* document, dBone* rootNode, daeElement *parent, const dModel* model, dTree<unsigned, unsigned>& uniqueNameFilter)
	{
		char name[256];

		if (rootNode->GetName()[0]) {
			sprintf (name, "%s%s", D_NODE_NAME, rootNode->GetName());
		} else {
			sprintf (name, "%s%04d", D_NODE_NAME, m_nodeIndex);
			m_nodeIndex ++;
		}


		domNode *tmChild = daeSafeCast<domNode>(parent->createAndPlace(COLLADA_ELEMENT_NODE));

		char uniqueName[256];
		strcpy (uniqueName, name);
		for (int i = 0; uniqueNameFilter.Find(dCRC (uniqueName)); i ++) {
			sprintf (uniqueName, "%s%03d", name, i);
		}
		int n = 0;
		uniqueNameFilter.Insert(n, dCRC (uniqueName));

//		dTree<domNode *, domNode *>& uniqueNameFilter
		tmChild->setId (uniqueName);
		tmChild->setSid(rootNode->GetName());
		tmChild->setType ((rootNode->GetType() == dBone::m_bone) ? NODETYPE_JOINT : NODETYPE_NODE);


	// add this node to the map
	//		domInstance_camera* cameraIntance;
	//		m_visualNodeMap.Insert (tmChild, mesh);
	//	if this is a camera node add the camera instance
	//		if ((mesh == m_cameraNode) && m_collCamera) {
	//			cameraIntance = daeSafeCast<domInstance_camera>(tmChild->createAndPlace(COLLADA_ELEMENT_INSTANCE_CAMERA));
	//			uri.setElement (m_collCamera);
	//			uri.resolveURI();
	//			cameraIntance->setUrl (uri);
	//		}

		dMatrix matrix (m_globalRotation.Inverse() * rootNode->GetMatrix() * m_globalRotation);

		domTranslate* position = daeSafeCast<domTranslate>(tmChild->createAndPlace(COLLADA_ELEMENT_TRANSLATE));
		domFloat3& positValue = position->getValue();
		positValue.setCount(3);
		positValue[0] = matrix.m_posit.m_x * m_scale;
		positValue[1] = matrix.m_posit.m_y * m_scale;
		positValue[2] = matrix.m_posit.m_z * m_scale;

		domRotate* rotation = daeSafeCast<domRotate>(tmChild->createAndPlace(COLLADA_ELEMENT_ROTATE));
		domFloat4& rotateValue = rotation->getValue();
		rotateValue.setCount(4);
		dQuaternion rot (matrix);
		dVector dir (rot.m_q1, rot.m_q2, rot.m_q3, 0.0f);
		float mag2 = dir % dir;
		if (mag2 > 1.0e-6f) {
			dir = dir.Scale (1.0f / dSqrt (mag2));
		} else {
			dir = dVector (1.0f, 0.0f, 0.0f, 0.0f);
		}
		rotateValue[0] = dir.m_x;
		rotateValue[1] = dir.m_y;
		rotateValue[2] = dir.m_z;
		rotateValue[3] = (2.0f * dAcos (rot.m_q0)) * (180.0f / 3.14159265f);

//		dMesh* mesh;
//		mesh = model->FindMesh(rootNode->GetName());
//		if (mesh) {
//			if (mesh->GetType() == dMesh::D_STATIC_MESH) {
//				AddMeshInstance (document, tmChild, mesh);
//			} else {
//				AddControllerInstance (document, tmChild, mesh);
//			}
//		}

		for (dBone* ptr = rootNode->GetChild(); ptr; ptr = ptr->GetSibling()) {
			AddNodes(document, ptr, tmChild, model, uniqueNameFilter);
		}

		return tmChild;
	}


	domNode* FindNodeNodeByName (domNode *root, const char* name)
	{
		int count;
		char nodeName[256];
		domNode* array[1024];

		
		sprintf (nodeName, "%s%s", D_NODE_NAME, name);
//		int lenght = int (strlen (nodeName));
		count = 1;
		array[0] = root;
		while (count) {
			domNode* node;
			count --;

			node = array[count];
//			if (!strncmp (nodeName, node->getId(), lenght)) {
			if (!strcmp (nodeName, node->getId())) {
				return node;
			}

			daeTArray<daeSmartRef<daeElement> > nodes;
			node->getChildren (nodes);
			for (int i = 0; i < int (nodes.getCount()); i ++) {
				daeElement* data = nodes[i];
				if (data->getElementType() == COLLADA_TYPE::NODE) {
					array[count] = (domNode*) data;
					count ++;
				}
			}
		}
		return NULL;
	}


	void AddMeshInstance (daeDocument* document, daeElement *node, dMesh* mesh)
	{
		int geometryCount;
		domCOLLADA *domRoot;
		domLibrary_materials *materialLibrary;
		domLibrary_geometries *geometryLibrary;

		domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		materialLibrary = domRoot->getLibrary_materials_array()[0];
		geometryLibrary = domRoot->getLibrary_geometries_array()[0];
		const domGeometry_Array &geometryArray =  geometryLibrary->getGeometry_array();
		geometryCount = int (geometryArray.getCount());

		for (int i = 0; i < geometryCount; i ++) {
			char name[256];
			domGeometry *meshGeometry;
			meshGeometry = geometryArray[i];

			mesh->GetName(name);
			if (!cdom::strcasecmp (meshGeometry->getId(), name)) {

				domInstance_geometry *meshInstance = daeSafeCast<domInstance_geometry>(node->createAndPlace(COLLADA_ELEMENT_INSTANCE_GEOMETRY));
				daeURI uri(*meshGeometry);
				uri.set("", "", "", "", meshGeometry->getId());
				meshInstance->setUrl (uri);

				domMesh *colladaMesh = meshGeometry->getMesh();
				const domTriangles_Array &trianglesArray = colladaMesh->getTriangles_array();
				int polyCount = int (trianglesArray.getCount());

				domBind_material *bindMat = daeSafeCast<domBind_material>(meshInstance->createAndPlace(COLLADA_ELEMENT_BIND_MATERIAL));
				domBind_material::domTechnique_common *bmtc = daeSafeCast<domBind_material::domTechnique_common>(bindMat->createAndPlace (COLLADA_ELEMENT_TECHNIQUE_COMMON));

				for (int j = 0; j < polyCount; j ++) {
					domMaterial* material;
					domTriangles* triangles; 
					domInstance_material *instMat;

					triangles = trianglesArray[j];
					instMat = daeSafeCast<domInstance_material>(bmtc->createAndPlace(COLLADA_ELEMENT_INSTANCE_MATERIAL ));
					material = FindMaterial (materialLibrary, triangles->getMaterial());
					_ASSERTE (material);
					daeURI uri(*material);
					uri.set("", "", "", "", material->getId());
					//uri.setElement (material);
					//uri.resolveURI();
					instMat->setSymbol(triangles->getMaterial());
					instMat->setTarget (uri);
				}
				break;
			}
		}
	}



	void AddControllerInstance (daeDocument* document, domNode *root, domNode *node, dMeshInstance* meshInstance)
	{
		char name[256];
		char meshName[256];
		dMesh* mesh = meshInstance->m_mesh;
		mesh->GetName(meshName);		
//		sprintf (name, "%s_skin", context->GetMeshName (mesh));
		sprintf (name, "%s%s", meshName, D_SKIN_POS_FIX);

		domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		domLibrary_controllers *controllerLibrary = domRoot->getLibrary_controllers_array()[0];
		const domController_Array &controllerArray = controllerLibrary->getController_array();

		int controllersCount = int (controllerArray.getCount());
		for (int i = 0; i < controllersCount; i ++) {
			domController *meshController = controllerArray[i];
			if (!cdom::strcasecmp (meshController->getId(), name)) {
				domInstance_controller *controllerInstance = daeSafeCast<domInstance_controller>(node->createAndPlace(COLLADA_ELEMENT_INSTANCE_CONTROLLER));

				daeURI uri (*meshController);
				uri.set("", "", "", "", meshController->getId());
				controllerInstance->setUrl (uri);

				domInstance_controller::domSkeleton* skeleton = daeSafeCast<domInstance_controller::domSkeleton>(controllerInstance->createAndPlace(COLLADA_ELEMENT_SKELETON));

				dBone* modelRootBone = ((dSkinModifier*)meshInstance->GetModifier())->m_skinnedBones[0]->GetRoot();
				domNode* rootBone = FindNodeNodeByName (root, modelRootBone->GetName());
				daeURI uri1 (*rootBone);
				uri1.set("", "", "", "", rootBone->getId());
				skeleton->setValue(uri1);
				
				domLibrary_materials *materialLibrary = domRoot->getLibrary_materials_array()[0];
				domLibrary_geometries *geometryLibrary = domRoot->getLibrary_geometries_array()[0];
				const domGeometry_Array &geometryArray = geometryLibrary->getGeometry_array();
				int geometryCount = int (geometryArray.getCount());

				for (int i = 0; i < geometryCount; i ++) {
					domGeometry *meshGeometry = geometryArray[i];

					if (!cdom::strcasecmp (meshGeometry->getId(), meshName)) {
						domMesh *colladaMesh = meshGeometry->getMesh();
						const domTriangles_Array &trianglesArray = colladaMesh->getTriangles_array();
						int polyCount = int (trianglesArray.getCount());

						domBind_material *bindMat = daeSafeCast<domBind_material>(controllerInstance->createAndPlace(COLLADA_ELEMENT_BIND_MATERIAL));
						domBind_material::domTechnique_common *bmtc = daeSafeCast<domBind_material::domTechnique_common>(bindMat->createAndPlace (COLLADA_ELEMENT_TECHNIQUE_COMMON));

						for (int j = 0; j < polyCount; j ++) {
							domTriangles* triangles = trianglesArray[j];
							domInstance_material *instMat = daeSafeCast<domInstance_material>(bmtc->createAndPlace(COLLADA_ELEMENT_INSTANCE_MATERIAL ));
							domMaterial* material = FindMaterial (materialLibrary, triangles->getMaterial());
							_ASSERTE (material);
							daeURI uri2 (*material);
							uri2.set("", "", "", "", material->getId());
							instMat->setSymbol(triangles->getMaterial());
							instMat->setTarget (uri2);
						}
						break;
					}
				}

				break;
			}
		}
	}



	void AddMeshInstaceToScene(daeDocument* document, daeElement *parent, const dModel* model, dMeshInstance* meshInstance)
	{
//		if (meshInstance->m_mesh->m_hasBone) {
			dBone* bone;
			domNode* node;
			bone = model->FindBone(meshInstance->m_boneID);
			_ASSERTE (bone);
			node = FindNodeNodeByName ((domNode *)parent, bone->GetName());
			_ASSERTE (node);
			if (meshInstance->m_modifier) {
				AddControllerInstance (document, (domNode*) parent, node, meshInstance);
			} else {
				AddMeshInstance (document, node, meshInstance->m_mesh);
			}
//		} else {
//			_ASSERTE (0);
//			dBone tmpBone(NULL);
//			tmpBone.SetMatrix(model->m_matrix);
//			tmpBone.SetNameID(context->GetMeshName(mesh));
//			AddNodes(document, &tmpBone, parent, model, context);
//			node = FindNodeNodeByName ((domNode *)parent, context->GetMeshName(mesh));
//		}
	}


	void AddVisualSceneLibrary (daeDocument* document, dModel* model, 
								dTree<domNode*, dModel*>& nodeMap, dTree<unsigned, unsigned>& uniqueNameFilter)
	{
		domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		
			// save skeleton hierarchy 
		if (model->m_skeleton.GetCount()) {
			domLibrary_visual_scenes *library;
			if (!domRoot->getLibrary_visual_scenes_array().getCount()) {
				library = daeSafeCast<domLibrary_visual_scenes>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_VISUAL_SCENES ));
			} else {
				library = domRoot->getLibrary_visual_scenes_array()[0];
			}

			int sceneNumber = int (library->getVisual_scene_array().getCount());
			char sceneName [256];
			sprintf (sceneName, "%svisual%03d", D_ROOT_NODE_NAME, sceneNumber);
			domVisual_scene *scene = daeSafeCast<domVisual_scene>(library->createAndPlace(COLLADA_ELEMENT_VISUAL_SCENE));
			scene->setId (sceneName);

			for (dList<dBone*>::dListNode* node = model->m_skeleton.GetFirst(); node; node = node->GetNext()) {
				dBone* rootNode = node->GetInfo();
				domNode* colNode = AddNodes(document, rootNode, scene, model, uniqueNameFilter);
				nodeMap.Insert (colNode, model);
			}

			// save all mesh referenced by each node
			for (dList<dMeshInstance>::dListNode* node = model->m_meshList.GetFirst(); node; node = node->GetNext()) { 
				AddMeshInstaceToScene(document, scene, model, &node->GetInfo());
			}
		} else {
			if (model->m_meshList.GetCount()) {

				// this is a special case of a model with no bones
				domLibrary_visual_scenes *library;
				if (!domRoot->getLibrary_visual_scenes_array().getCount()) {
					library = daeSafeCast<domLibrary_visual_scenes>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_VISUAL_SCENES ));
				} else {
					library = domRoot->getLibrary_visual_scenes_array()[0];
				}

				int sceneNumber = int (library->getVisual_scene_array().getCount());
				char sceneName [256];
				sprintf (sceneName, "%sVisual%03d", D_ROOT_NODE_NAME, sceneNumber);
				domVisual_scene *scene = daeSafeCast<domVisual_scene>(library->createAndPlace(COLLADA_ELEMENT_VISUAL_SCENE));
				scene->setId (sceneName);

				for (dList<dMeshInstance>::dListNode* node = model->m_meshList.GetFirst(); node; node = node->GetNext()) {

					char name[256];

					node->GetInfo().m_mesh->GetName(name);

					dBone* rootNode = new dBone(NULL);

					int hasBoneState = node->GetInfo().m_mesh->m_hasBone;
					node->GetInfo().m_mesh->m_hasBone = 1;
					//rootNode->SetMatrix(model->m_matrix);
					rootNode->SetMatrix(GetIdentityMatrix());
					rootNode->SetNameID (name);
					rootNode->m_type = dBone::m_sceneNode;
					rootNode->m_boneID = node->GetInfo().m_mesh->m_boneID;

					model->AddSkeleton(rootNode);
					rootNode->Release();

					domNode* colNode = AddNodes(document, rootNode, scene, model, uniqueNameFilter);
					nodeMap.Insert (colNode, model);

					AddMeshInstaceToScene(document, scene, model, &node->GetInfo());
					model->RemoveSkeleton(rootNode);
					node->GetInfo().m_mesh->m_hasBone = hasBoneState;
				}
			}
		}
	}


	void AddOffssetMatrix  (daeElement* parent, const dMatrix& offsetMatrix)
	{
		dMatrix matrix (m_globalRotation * offsetMatrix * m_globalRotation);
		matrix.m_posit = matrix.m_posit.Scale (m_scale);

		dQuaternion rot (matrix);
		dFloat angle = 2.0f * acosf (rot.m_q0);
		dVector dir (rot.m_q1, rot.m_q2, rot.m_q3, 0.0f);

		dFloat mag2 = dir % dir;
		if (mag2 > 1.0e-6f) {
			dir = dir.Scale(1.0f / sqrtf (dir % dir));
		} else {
			dir = dVector (1.0f, 0.0f, 0.0f, 0.0f);
		}

		domTranslate* translation = daeSafeCast<domTranslate>(parent->createAndPlace(COLLADA_ELEMENT_TRANSLATE)); 
		translation->getValue().append (matrix.m_posit.m_x);
		translation->getValue().append (matrix.m_posit.m_y);
		translation->getValue().append (matrix.m_posit.m_z);

		domRotate* rotation = daeSafeCast<domRotate>(parent->createAndPlace(COLLADA_ELEMENT_ROTATE)); 
		rotation->getValue().append (dir.m_x);
		rotation->getValue().append (dir.m_y);
		rotation->getValue().append (dir.m_z);
		rotation->getValue().append (angle * 180.0f / 3.14159265f);
	}

	void ApplyMatrixAligment (dMatrix& matrix)
	{
		matrix = dRollMatrix(-0.5f * 3.14159265f) * matrix;
	}

	void UnapplyMatrixAligment (dMatrix& matrix)
	{
		matrix = dRollMatrix(0.5f * 3.14159265f) * matrix;
	}




//	void AddShape (daeDocument* document, domRigid_body::domTechnique_common* technique, const NewtonBody* newtonBody)
	void AddShape (daeDocument* document, domRigid_body::domTechnique_common* technique, 
				   const NewtonCollision* collision, dTree<domGeometry *, const NewtonCollision*>& shapeCache)
	{
		// get the collision information
		NewtonCollisionInfoRecord collisionInfo;
		NewtonCollisionGetInfo (collision, &collisionInfo);

		domRigid_body::domTechnique_common::domShape *shape = NULL;

		dMatrix& matrix = *((dMatrix*) &collisionInfo.m_offsetMatrix[0][0]);
		switch (collisionInfo.m_collisionType)
		{
			case SERIALIZE_ID_BOX:
			{
				shape = daeSafeCast<domRigid_body::domTechnique_common::domShape>(technique->createAndPlace(COLLADA_ELEMENT_SHAPE));
				domBox* box = daeSafeCast<domBox>(shape->createAndPlace(COLLADA_ELEMENT_BOX)); 
				domBox::domHalf_extents* halfExtend = daeSafeCast<domBox::domHalf_extents>(box->createAndPlace(COLLADA_ELEMENT_HALF_EXTENTS)); 

				dVector size (collisionInfo.m_box.m_x, collisionInfo.m_box.m_y, collisionInfo.m_box.m_z, 0.0f);
				size = size.Scale (m_scale * 0.5f);
				halfExtend->getValue().append (size.m_x);
				halfExtend->getValue().append (size.m_y);
				halfExtend->getValue().append (size.m_z);
				break;
			}

			case SERIALIZE_ID_SPHERE:
			{
				shape = daeSafeCast<domRigid_body::domTechnique_common::domShape>(technique->createAndPlace(COLLADA_ELEMENT_SHAPE));
				domSphere* sphere = daeSafeCast<domSphere>(shape->createAndPlace(COLLADA_ELEMENT_SPHERE)); 
				domSphere::domRadius* halfExtend = daeSafeCast<domSphere::domRadius>(sphere->createAndPlace(COLLADA_ELEMENT_RADIUS)); 
				halfExtend->setValue(collisionInfo.m_sphere.m_r0 * m_scale);

				domExtra* extra = daeSafeCast<domExtra>(sphere->createAndPlace(COLLADA_ELEMENT_EXTRA));
				domTechnique *extraTechnique = daeSafeCast<domTechnique>(extra->createAndPlace(COLLADA_ELEMENT_TECHNIQUE));
				extraTechnique->setProfile("ellipse");

				daeElement* element = (daeElement*)extraTechnique->createAndPlace( "radios" ); 
				domAny *domExtraExtension = (domAny*)element; 
				char radios[256];
				sprintf (radios, "%f %f %f", collisionInfo.m_sphere.m_r0 * m_scale, collisionInfo.m_sphere.m_r1 * m_scale, collisionInfo.m_sphere.m_r2 * m_scale);
				domExtraExtension->setValue (radios);
				break;
			}

			case SERIALIZE_ID_CAPSULE:
			{
				shape = daeSafeCast<domRigid_body::domTechnique_common::domShape>(technique->createAndPlace(COLLADA_ELEMENT_SHAPE));
				domCapsule* capsule = daeSafeCast<domCapsule>(shape->createAndPlace(COLLADA_ELEMENT_CAPSULE)); 

				domCapsule::domHeight* height = daeSafeCast<domCapsule::domHeight>(capsule->createAndPlace(COLLADA_ELEMENT_HEIGHT)); 
				dFloat h = (collisionInfo.m_capsule.m_height * 0.5f - collisionInfo.m_capsule.m_r0) * m_scale;
				height->setValue(h);

				domCapsule::domRadius* radius = daeSafeCast<domCapsule::domRadius>(capsule->createAndPlace(COLLADA_ELEMENT_RADIUS)); 
				radius->getValue().append(collisionInfo.m_capsule.m_r0 * m_scale);
				radius->getValue().append(collisionInfo.m_capsule.m_r1 * m_scale);

				ApplyMatrixAligment (matrix);
				break;
			}


			case SERIALIZE_ID_CONE:
			{
				shape = daeSafeCast<domRigid_body::domTechnique_common::domShape>(technique->createAndPlace(COLLADA_ELEMENT_SHAPE));
				domTapered_cylinder* cone = daeSafeCast<domTapered_cylinder>(shape->createAndPlace(COLLADA_ELEMENT_TAPERED_CYLINDER)); 

				domTapered_cylinder::domHeight* height = daeSafeCast<domTapered_cylinder::domHeight>(cone->createAndPlace(COLLADA_ELEMENT_HEIGHT)); 
				height->setValue(collisionInfo.m_cone.m_height * m_scale * 0.5f);

				domTapered_cylinder::domRadius1* radius1 = daeSafeCast<domTapered_cylinder::domRadius1>(cone->createAndPlace(COLLADA_ELEMENT_RADIUS1)); 
				radius1->getValue().append(collisionInfo.m_cone.m_r * m_scale);
				radius1->getValue().append(collisionInfo.m_cone.m_r * m_scale);

				domTapered_cylinder::domRadius2* radius2 = daeSafeCast<domTapered_cylinder::domRadius2>(cone->createAndPlace(COLLADA_ELEMENT_RADIUS2)); 
				radius2->getValue().append(0.0f);
				radius2->getValue().append(0.0f);

				ApplyMatrixAligment (matrix);
				break;
			}

			case SERIALIZE_ID_CYLINDER:
			{
				shape = daeSafeCast<domRigid_body::domTechnique_common::domShape>(technique->createAndPlace(COLLADA_ELEMENT_SHAPE));
				domCylinder* cylinder = daeSafeCast<domCylinder>(shape->createAndPlace(COLLADA_ELEMENT_CYLINDER)); 

				domCylinder::domHeight* height = daeSafeCast<domCylinder::domHeight>(cylinder->createAndPlace(COLLADA_ELEMENT_HEIGHT)); 
				height->setValue(collisionInfo.m_cylinder.m_height * m_scale * 0.5f);

				domCylinder::domRadius* radius = daeSafeCast<domCylinder::domRadius>(cylinder->createAndPlace(COLLADA_ELEMENT_RADIUS)); 
				radius->getValue().append(collisionInfo.m_cylinder.m_r0 * m_scale);
				radius->getValue().append(collisionInfo.m_cylinder.m_r0 * m_scale);

				ApplyMatrixAligment (matrix);
				break;
			}

			case SERIALIZE_ID_CHAMFERCYLINDER:
			{
				shape = daeSafeCast<domRigid_body::domTechnique_common::domShape>(technique->createAndPlace(COLLADA_ELEMENT_SHAPE));
				domCylinder* cylinder = daeSafeCast<domCylinder>(shape->createAndPlace(COLLADA_ELEMENT_CYLINDER)); 

				domCylinder::domHeight* height = daeSafeCast<domCylinder::domHeight>(cylinder->createAndPlace(COLLADA_ELEMENT_HEIGHT)); 
				height->setValue(collisionInfo.m_chamferCylinder.m_height * m_scale * 0.5);

				domCylinder::domRadius* radius = daeSafeCast<domCylinder::domRadius>(cylinder->createAndPlace(COLLADA_ELEMENT_RADIUS)); 
				radius->getValue().append(collisionInfo.m_chamferCylinder.m_r * m_scale);
				radius->getValue().append(collisionInfo.m_chamferCylinder.m_r * m_scale);

				domExtra* extra = daeSafeCast<domExtra>(cylinder->createAndPlace(COLLADA_ELEMENT_EXTRA));
				domTechnique *extraTechnique = daeSafeCast<domTechnique>(extra->createAndPlace(COLLADA_ELEMENT_TECHNIQUE));
				extraTechnique->setProfile("chamferCylinder");

				ApplyMatrixAligment (matrix);
				break;
			}


			case SERIALIZE_ID_COMPOUND:
			{
				for (int i = 0; i < collisionInfo.m_compoundCollision.m_chidrenCount; i ++) {
					AddShape (document, technique, collisionInfo.m_compoundCollision.m_chidren[i], shapeCache);
				}
				break;
			}


			case SERIALIZE_ID_CONVEXHULL:
			{
				dTree<domGeometry *, const NewtonCollision*>::dTreeNode* node = shapeCache.Find(collision);
				if (!node) {
					// create a helper mesh from the collision collision
					NewtonMesh* newtMesh = NewtonMeshCreateFromCollision(collision);

					char name[256];
					sprintf (name, "convexShape%x", collision);
					dMesh* mesh = new dMesh(name);

					// extract vertex data  from the newton mesh		
					int vertexCount = NewtonMeshGetPointCount (newtMesh); 
					mesh->AllocVertexData(vertexCount);
					NewtonMeshGetVertexStreams (newtMesh, 3 * sizeof (dFloat), (dFloat*) mesh->m_vertex,
														  3 * sizeof (dFloat), (dFloat*) mesh->m_normal,
														  2 * sizeof (dFloat), (dFloat*) mesh->m_uv, 
														  2 * sizeof (dFloat), (dFloat*) mesh->m_uv);

					// extract the materials index array for mesh
					void* geometryHandle = NewtonMeshBeginHandle (newtMesh); 
					for (int handle = NewtonMeshFirstMaterial (newtMesh, geometryHandle); handle != -1; handle = NewtonMeshNextMaterial (newtMesh, geometryHandle, handle)) {
						//int material = NewtonMeshMaterialGetMaterial (newtMesh, geometryHandle, handle); 
						int indexCount = NewtonMeshMaterialGetIndexCount (newtMesh, geometryHandle, handle); 

						dSubMesh* segment = mesh->AddSubMesh();
						segment->m_textureHandle = 0;

						segment->AllocIndexData (indexCount);
						NewtonMeshMaterialGetIndexStream (newtMesh, geometryHandle, handle, (int*)segment->m_indexes); 
					}
					NewtonMeshEndHandle (newtMesh, geometryHandle); 
					NewtonMeshDestroy(newtMesh);

		//			collGeometry = AddGeometry (document, geometry);
					domGeometry *collGeometry = AddConvexGeometry (document, mesh);

					node = shapeCache.Insert(collGeometry, collision);
					mesh->Release();
				}

				domGeometry *collGeometry = node->GetInfo();

				shape = daeSafeCast<domRigid_body::domTechnique_common::domShape>(technique->createAndPlace(COLLADA_ELEMENT_SHAPE));
				domInstance_geometry *meshInstance = daeSafeCast<domInstance_geometry>(shape->createAndPlace(COLLADA_ELEMENT_INSTANCE_GEOMETRY));
				daeURI uri(*collGeometry);
				uri.set("", "", "", "", collGeometry->getId());
				meshInstance->setUrl (uri);

//				domBind_material *bindMat = daeSafeCast<domBind_material>(instance->createAndPlace(COLLADA_ELEMENT_BIND_MATERIAL));
//				domBind_material::domTechnique_common *bmtc = daeSafeCast<domBind_material::domTechnique_common>(bindMat->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON ));
//				domInstance_material *instMat = daeSafeCast<domInstance_material>(bmtc->createAndPlace(COLLADA_ELEMENT_INSTANCE_MATERIAL ));
//				sprintf (materialName, "%s%x", D_MATERIAL_PREFIX_NAME, dCRC (0));
//				domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot() );
//				materialLibrary = domRoot->getLibrary_materials_array()[0];
//				material = FindMaterial (materialLibrary, materialName);
//				_ASSERTE (material);
//				uri.setElement (material);
//				uri.resolveURI();
//				instMat->setSymbol(materialName);
//				instMat->setTarget (uri);

				break;
			}

			case SERIALIZE_ID_TREE:
			{
				dTree<domGeometry *, const NewtonCollision*>::dTreeNode* node = shapeCache.Find(collision);
				if (!node) {
					const dFloat* vertexArray;
					int vertexCount;
					int vertexStride;

					int* indexArray = new int [collisionInfo.m_collisionTree.m_indexCount];
					int* faceAttribute = new int [collisionInfo.m_collisionTree.m_indexCount / 3];
					dVector minBox (-1.0e10f, -1.0e10f, -1.0e10f);
					dVector maxBox ( 1.0e10f,  1.0e10f,  1.0e10f);

					int triangleCount = NewtonTreeCollisionGetVertexListIndexListInAABB (collision, &minBox.m_x, &maxBox.m_x,
																					  &vertexArray, &vertexCount, &vertexStride, 
																					  indexArray, collisionInfo.m_collisionTree.m_indexCount,
																					  faceAttribute); 

					_ASSERTE (vertexCount == collisionInfo.m_collisionTree.m_vertexCount);

					char name[256];
					sprintf (name, "collisionTree%x", collision);
					dMesh* mesh = new dMesh (name);

					// pack the vertex list
					int *vertexMask = new int [vertexCount];
					memset (vertexMask, -1, vertexCount * sizeof (int));

					vertexStride = vertexStride / sizeof (dFloat);
					mesh->AllocVertexData(vertexCount);

					int newVertexCount = 0;
					for (int i = 0; i < triangleCount * 3; i ++) {
						int j = indexArray[i];
						if (vertexMask[j] == -1) {
							vertexMask[j] = newVertexCount;
							mesh->m_vertex[newVertexCount * 3 + 0] = dFloat (vertexArray[j * vertexStride + 0]);
							mesh->m_vertex[newVertexCount * 3 + 1] = dFloat (vertexArray[j * vertexStride + 1]);
							mesh->m_vertex[newVertexCount * 3 + 2] = dFloat (vertexArray[j * vertexStride + 2]);
							mesh->m_normal[newVertexCount * 3 + 0] = dFloat (0.0f);
							mesh->m_normal[newVertexCount * 3 + 1] = dFloat (1.0f);
							mesh->m_normal[newVertexCount * 3 + 2] = dFloat (0.0f);
							mesh->m_normal[newVertexCount * 2 + 0] = dFloat (0.0f);
							mesh->m_normal[newVertexCount * 2 + 1] = dFloat (0.0f);
							newVertexCount ++;
						}
					}

					mesh->m_vertexCount = newVertexCount;

					int count;
					do {
						int id = 0;
						count = 0;
						for (int i = 0; i < triangleCount; i ++) {
							if (faceAttribute[i] != -1) {
								if (faceAttribute[i] != id) {
									id = faceAttribute[i];
								}
								count ++;
							}
						}

						if (count) {
							dSubMesh* segment = mesh->AddSubMesh();
							segment->AllocIndexData (count * 3);

							int index = 0;
							for (int i = 0; i < triangleCount; i ++) {
								if (faceAttribute[i] == id) {
									faceAttribute[i] = -1;
									segment->m_indexes[index * 3 + 0] = vertexMask[indexArray[i * 3 + 0]];
									segment->m_indexes[index * 3 + 1] = vertexMask[indexArray[i * 3 + 1]];
									segment->m_indexes[index * 3 + 2] = vertexMask[indexArray[i * 3 + 2]];
									index ++;
								}
							}
						}
					} while (count);

					domGeometry *collGeometry = AddGeometry (document, mesh);
					node = shapeCache.Insert(collGeometry, collision);
					mesh->Release();

					delete[] vertexMask;
					delete[] indexArray;
					delete[] faceAttribute;
				}

				domGeometry *collGeometry = node->GetInfo();

				shape = daeSafeCast<domRigid_body::domTechnique_common::domShape>(technique->createAndPlace(COLLADA_ELEMENT_SHAPE));
				domInstance_geometry *meshInstance = daeSafeCast<domInstance_geometry>(shape->createAndPlace(COLLADA_ELEMENT_INSTANCE_GEOMETRY));
				daeURI uri(*collGeometry);
				uri.set("", "", "", "", collGeometry->getId());
				meshInstance->setUrl (uri);

//				bindMat = daeSafeCast<domBind_material>(instance->createAndPlace(COLLADA_ELEMENT_BIND_MATERIAL));
//				bmtc = daeSafeCast<domBind_material::domTechnique_common>(bindMat->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON ));
//				instMat = daeSafeCast<domInstance_material>(bmtc->createAndPlace (COLLADA_ELEMENT_INSTANCE_MATERIAL));
//				sprintf (materialName, "%s%x", D_MATERIAL_PREFIX_NAME, dCRC (0));
//				domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot() );
//				materialLibrary = domRoot->getLibrary_materials_array()[0];
//				material = FindMaterial (materialLibrary, materialName);
//				_ASSERTE (material);
//				uri.setElement (material);
//				uri.resolveURI();
//				instMat->setSymbol(materialName);
//				instMat->setTarget (uri);
				break;
			}


			case D_PLANE_COLLISON_ID:
			{
				shape = daeSafeCast<domRigid_body::domTechnique_common::domShape>(technique->createAndPlace(COLLADA_ELEMENT_SHAPE));
				domPlane* plane = daeSafeCast<domPlane>(shape->createAndPlace(COLLADA_ELEMENT_PLANE)); 
				domPlane::domEquation* equation = daeSafeCast<domPlane::domEquation>(plane->createAndPlace(COLLADA_ELEMENT_EQUATION)); 

				dVector surface (collisionInfo.m_paramArray[0], collisionInfo.m_paramArray[1], collisionInfo.m_paramArray[2], collisionInfo.m_paramArray[3]);
				surface = m_globalRotation.TransformPlane (surface);
				equation->getValue().append (surface.m_x);
				equation->getValue().append (surface.m_y);
				equation->getValue().append (surface.m_z);
				equation->getValue().append (surface.m_w * m_scale);
				break;
			}


			default: 
				_ASSERTE (0);
		}

		AddOffssetMatrix (shape, matrix);

	}



//	domRigid_body* AddRigidBody (daeDocument* document, const NewtonBody* newtonBody)
	domRigid_body* AddRigidBody (daeDocument* document, domPhysics_model* model, const NewtonBody* newtonBody, const char* name, dTree<domGeometry *, const NewtonCollision*>& shapeFilter)
	{
//		daeURI uri;
//		domCOLLADA *domRoot;
//		domPhysics_material* material;
//		domInstance_physics_material* intanceMaterial;
//		domLibrary_physics_materials* materialLibrary;

		// create the image library
//		domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot() );

		// add a body to this model
		domRigid_body* rigidBody = daeSafeCast<domRigid_body>(model->createAndPlace(COLLADA_ELEMENT_RIGID_BODY));
		rigidBody->setSid(name);

		// add rigid body info
		domRigid_body::domTechnique_common* techniqueCommon = daeSafeCast<domRigid_body::domTechnique_common>(rigidBody->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
		_ASSERTE (techniqueCommon);


		// set the mass and the body type
		dFloat mass;
		dVector inertia;
		NewtonBodyGetMassMatrix (newtonBody, &mass, &inertia.m_x, &inertia.m_y, &inertia.m_z);

		bool isDynamic = (mass > 1.0e-3f) ? true : false; 

		domRigid_body::domTechnique_common::domDynamic* dynamicBody = daeSafeCast<domRigid_body::domTechnique_common::domDynamic>(techniqueCommon->createAndPlace(COLLADA_ELEMENT_DYNAMIC));
		dynamicBody->setValue(isDynamic);

		domTargetableFloat* bodyMass = daeSafeCast<domTargetableFloat>(techniqueCommon->createAndPlace(COLLADA_ELEMENT_MASS));
		bodyMass->setValue(mass);

		dVector com;
		NewtonBodyGetCentreOfMass (newtonBody, &com.m_x);
		domRigid_body::domTechnique_common::domMass_frame* massFrame = daeSafeCast<domRigid_body::domTechnique_common::domMass_frame>(techniqueCommon->createAndPlace(COLLADA_ELEMENT_MASS_FRAME));

		com.m_w = 0.0f;
		com = m_globalRotation.RotateVector(com);
		com = com.Scale (m_scale);
		domTranslate* translation = daeSafeCast<domTranslate>(massFrame->createAndPlace(COLLADA_ELEMENT_TRANSLATE)); 
		translation->getValue().append (com.m_x);
		translation->getValue().append (com.m_y);
		translation->getValue().append (com.m_z);

		domRotate* rotation = daeSafeCast<domRotate>(massFrame->createAndPlace(COLLADA_ELEMENT_ROTATE)); 
		rotation->getValue().append (0.0f);
		rotation->getValue().append (0.0f);
		rotation->getValue().append (1.0f);
		rotation->getValue().append (0.0f);

		inertia.m_w = 0.0f;
		inertia = m_globalRotation.RotateVector(inertia);
		inertia = inertia.Scale (m_scale* m_scale);
		domTargetableFloat3* bodyInertia = daeSafeCast<domTargetableFloat3>(techniqueCommon->createAndPlace(COLLADA_ELEMENT_INERTIA));
		bodyInertia->getValue().append(inertia.m_x);
		bodyInertia->getValue().append(inertia.m_y);
		bodyInertia->getValue().append(inertia.m_z);


		// material for this body
//		intanceMaterial = daeSafeCast<domInstance_physics_material>(techniqueCommon->createAndPlace(COLLADA_ELEMENT_INSTANCE_PHYSICS_MATERIAL));
//		_ASSERTE (intanceMaterial);
//		uri.setElement (material);
//		uri.setElement (AddPhysicsMaterialLibrary(document, newtonBody, enumID));
//		uri.resolveURI();
//		intanceMaterial->setUrl(uri);

		// add collision shape for this body
		AddShape (document, techniqueCommon, NewtonBodyGetCollision (newtonBody), shapeFilter);

		domExtra* extra = daeSafeCast<domExtra>(rigidBody->createAndPlace(COLLADA_ELEMENT_EXTRA));
		domTechnique *technique = daeSafeCast<domTechnique>(extra->createAndPlace(COLLADA_ELEMENT_TECHNIQUE));
		technique->setProfile("Newton");

		daeElement* element = (daeElement*)technique->createAndPlace( "BindingUserData" ); 
		domAny *domExtraExtension = (domAny*)element; 
		domExtraExtension->setValue ("default");

		element = (daeElement*)technique->createAndPlace( "BindingSetTransformFunction" ); 
		domExtraExtension = (domAny*)element; 
		domExtraExtension->setValue ("default");

		element = (daeElement*)technique->createAndPlace( "BindingExternalForceFunction" ); 
		domExtraExtension = (domAny*)element; 
		domExtraExtension->setValue ("default");

		element = (daeElement*)technique->createAndPlace( "BindingBodyDestructorCallback" ); 
		domExtraExtension = (domAny*)element; 
		domExtraExtension->setValue ("default");



//		element = (daeElement*)technique->createAndPlace( "AutoSleepMode" ); 
//		domExtraExtension = (domAny*)element; 
//		//autoSleepMode = NewtonBodyGetAutoFreeze (newtonBody);
//		autoSleepMode = false;
//		sprintf (tmpName, "%s", autoSleepMode ? "true" : "false");
//		domExtraExtension->setValue (tmpName);

		char tmpName[256];
		element = (daeElement*)technique->createAndPlace( "InternalLinearDrag" ); 
		domExtraExtension = (domAny*)element; 
		float linearDragValue = NewtonBodyGetLinearDamping (newtonBody);
		sprintf (tmpName, "%f", linearDragValue);
		domExtraExtension->setValue (tmpName);

		float angularDragValue[4];
		element = (daeElement*)technique->createAndPlace( "InternalAngularDrag" ); 
		domExtraExtension = (domAny*)element; 
		NewtonBodyGetAngularDamping (newtonBody, angularDragValue);
		sprintf (tmpName, "%f", angularDragValue[0]);
		domExtraExtension->setValue (tmpName);

//		element = (daeElement*)technique->createAndPlace( "InternalGyroscopicForces" ); 
//		domExtraExtension = (domAny*)element; 
//		//gyroscopyForces = NewtonBodyGetGyroscopicForcesMode (newtonBody);
//		gyroscopyForces = false;
//		sprintf (tmpName, "%s", gyroscopyForces ? "true" : "false");
//		domExtraExtension->setValue (tmpName);

		element = (daeElement*)technique->createAndPlace( "InternalContinueCollision" ); 
		domExtraExtension = (domAny*)element; 
		int continueCollision = NewtonBodyGetContinuousCollisionMode (newtonBody);
//		sprintf (tmpName, "%s", continueCollision ? "true" : "false");
		domExtraExtension->setValue (continueCollision ? "true" : "false");

		element = (daeElement*)technique->createAndPlace( "RecursivelyCollideWithLinkedBodies" ); 
		domExtraExtension = (domAny*)element; 
		int collideWithCollision = NewtonBodyGetJointRecursiveCollision (newtonBody);
//		sprintf (tmpName, "%s", collideWithCollision ? "true" : "false");
		domExtraExtension->setValue (collideWithCollision ? "true" : "false");

		element = (daeElement*)technique->createAndPlace( "UnilateralMaterialID" ); 
		domExtraExtension = (domAny*)element; 
		int materialGroudlID = NewtonBodyGetMaterialGroupID (newtonBody);
		sprintf (tmpName, "%d", materialGroudlID);
		domExtraExtension->setValue (tmpName);

		return rigidBody;
	}


	void AddPhysicsModelLibrary (daeDocument* document, const dList<const NewtonBody*>& bodyList, const char* modelName, 
							     dTree<domGeometry *, const NewtonCollision*>& shapeFilter, dTree<domNode*, dModel*>& nodeMap)
	{
//		int bodyEnumeration; 
//		int jointEnumeration; 
//		void* oldUserData;
//		domCOLLADA *domRoot;
//		domPhysics_model* model;
//		domRigid_body* sentinel;
//		domRigid_body* rigidBody;
//		domRigid_constraint* rigidConstraint;
//		const NewtonBody* body;
//		const NewtonBody* newtonBody;
//		const NewtonJoint* newtonJoint;
//		const NewtonCollision* collision;
//		domLibrary_physics_models* modelLibrary;
//		domLibrary_physics_materials* materialLibrary;
//		NewtonBodyList::dTreeNode* bodyNode;


		// create the image library
//		domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot() );
//		modelLibrary = daeSafeCast<domLibrary_physics_models>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_PHYSICS_MODELS));
//		materialLibrary = daeSafeCast<domLibrary_physics_materials>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_PHYSICS_MATERIALS));

		domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		if (!domRoot->getLibrary_physics_models_array().getCount()) {
			daeSafeCast<domLibrary_physics_models>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_PHYSICS_MODELS));

//			domLibrary_physics_scenes *library = daeSafeCast<domLibrary_physics_scenes>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_PHYSICS_SCENES));
			char sceneName[256];
//			sprintf (sceneName, "%sphysic", D_ROOT_NODE_NAME);

			domLibrary_physics_scenes *library = daeSafeCast<domLibrary_physics_scenes>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_PHYSICS_SCENES));
			domPhysics_scene *scene = daeSafeCast<domPhysics_scene>(library->createAndPlace(COLLADA_ELEMENT_PHYSICS_SCENE));
			sprintf (sceneName, "%sphysic", D_ROOT_NODE_NAME);
			scene->setId (sceneName);
			domPhysics_scene::domTechnique_common* technique = daeSafeCast<domPhysics_scene::domTechnique_common>(scene->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
			domTargetableFloat3* gravity = daeSafeCast<domTargetableFloat3>(technique->createAndPlace(COLLADA_ELEMENT_GRAVITY));
			dVector g (m_globalRotation.RotateVector(dVector (dFloat (0.0f), dFloat (-10.0f), dFloat (0.0f)))); 
			g = g.Scale (m_scale);
			gravity->getValue().append(g.m_x);
			gravity->getValue().append(g.m_y);
			gravity->getValue().append(g.m_z);

			domTargetableFloat* tiemStep = daeSafeCast<domTargetableFloat>(technique->createAndPlace(COLLADA_ELEMENT_TIME_STEP));
			tiemStep->setValue(1.0f / 60.0f);
		}

		domLibrary_physics_models *modelLibrary = domRoot->getLibrary_physics_models_array()[0];
		_ASSERTE (modelLibrary);

		// add the physics models to the scene.
		domPhysics_model* model = daeSafeCast<domPhysics_model>(modelLibrary->createAndPlace(COLLADA_ELEMENT_PHYSICS_MODEL));
		model->setId (modelName);

		domLibrary_physics_scenes *library = domRoot->getLibrary_physics_scenes_array()[0];
		domPhysics_scene *scene = library->getPhysics_scene_array()[0];
		domInstance_physics_model* instanceModel = daeSafeCast<domInstance_physics_model>(scene->createAndPlace(COLLADA_TYPE_INSTANCE_PHYSICS_MODEL));
		daeURI uri (*model);
		uri.set("", "", "", "", model->getId());
		instanceModel->setUrl(uri);

		for (dList<const NewtonBody*>::dListNode* node = bodyList.GetFirst(); node; node = node->GetNext()) {
			const NewtonBody* body = node->GetInfo();
			char name[256];
			sprintf (name, "rigidBody_%s", modelName);
			AddRigidBody (document, model, body, name, shapeFilter);

			domInstance_rigid_body* instanceBody = daeSafeCast<domInstance_rigid_body>(instanceModel->createAndPlace(COLLADA_TYPE_INSTANCE_RIGID_BODY));
			instanceBody->setBody(name);

			dModel* visualModel = (dModel*) NewtonBodyGetUserData(body);
			if (visualModel) {
				_ASSERTE (visualModel->IsType(dModel::GetRttiType()));
				domNode* visualNode = nodeMap.Find (visualModel)->GetInfo();
				_ASSERTE (visualNode);
				daeURI uri (*visualNode);
				uri.set("", "", "", "", visualNode->getID());
				instanceBody->setTarget(uri);
			} else {

			}
			
			domInstance_rigid_body::domTechnique_common* techniqueCommon = daeSafeCast<domInstance_rigid_body::domTechnique_common>(instanceBody->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));

			dVector veloc;
			dVector omega;
			NewtonBodyGetOmega(body, &omega[0]);
			NewtonBodyGetVelocity(body, &veloc[0]);
			domInstance_rigid_body::domTechnique_common::domVelocity* initVeloc = daeSafeCast<domInstance_rigid_body::domTechnique_common::domVelocity>(techniqueCommon->createAndPlace(COLLADA_ELEMENT_VELOCITY));
			domInstance_rigid_body::domTechnique_common::domAngular_velocity* initOmega = daeSafeCast<domInstance_rigid_body::domTechnique_common::domAngular_velocity>(techniqueCommon->createAndPlace(COLLADA_ELEMENT_ANGULAR_VELOCITY));

			omega = m_globalRotation.RotateVector(omega);
			veloc = m_globalRotation.RotateVector(veloc.Scale (m_scale));

			initVeloc->getValue().append(veloc[0]);
			initVeloc->getValue().append(veloc[1]);
			initVeloc->getValue().append(veloc[2]);

			initOmega->getValue().append(omega[0]);
			initOmega->getValue().append(omega[1]);
			initOmega->getValue().append(omega[2]);
		}

/*
		// now add all constraints
		NewtonWorldSetUserData(m_world, &m_jointList);
		NewtonWorldForEachJointDo (m_world, NewtonJointList::JointIterator);

		jointEnumeration = 0;
		NewtonJointList::Iterator constIter (m_jointList);
		for (constIter.Begin(); constIter; constIter ++) {
			newtonJoint = constIter.GetNode()->GetKey();
			rigidConstraint = AddRigidConstraint (document, sentinel, model, newtonJoint, jointEnumeration);
			constIter.GetNode()->GetInfo() = rigidConstraint;
			jointEnumeration ++;
		}

		// restore the original user data
		NewtonWorldSetUserData(m_world, oldUserData);

		// destroy the semtinelBody
		NewtonDestroyBody(m_world, body);
*/
	}


	void AddScene (daeDocument* document)
	{
		domCOLLADA *domRoot;
		domInstanceWithExtra *ivs;
		domCOLLADA::domScene *sceneInstance;

		domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		sceneInstance = daeSafeCast<domCOLLADA::domScene>(domRoot->createAndPlace(COLLADA_ELEMENT_SCENE));

		if (domRoot->getLibrary_visual_scenes_array().getCount()) {
			domLibrary_visual_scenes *visualLibrary;
			visualLibrary = domRoot->getLibrary_visual_scenes_array()[0];
			domVisual_scene_Array& visualScenes = visualLibrary->getVisual_scene_array();
//			for (int i = 0; i <  int (visualScenes.getCount()); i ++ ) {
				int i = 0;

				domVisual_scene *visualScene;
				visualScene = visualScenes[i];
				ivs = daeSafeCast<domInstanceWithExtra>(sceneInstance->createAndPlace(COLLADA_ELEMENT_INSTANCE_VISUAL_SCENE));
				daeURI uri (*visualScene);
				uri.set("", "", "", "", visualScene->getId());
				ivs->setUrl (uri);
//				break;
//			}
		}

		if (domRoot->getLibrary_physics_scenes_array().getCount()) {
			domLibrary_physics_scenes *physicsLibrary;
			physicsLibrary = domRoot->getLibrary_physics_scenes_array()[0];
			domPhysics_scene_Array& physicsScenes = physicsLibrary->getPhysics_scene_array();
			for (int i = 0; i <  int (physicsScenes.getCount()); i ++ ) {
				domPhysics_scene *physicsScene;
				physicsScene = physicsScenes[0];
				ivs = daeSafeCast<domInstanceWithExtra>(sceneInstance->createAndPlace(COLLADA_ELEMENT_INSTANCE_PHYSICS_SCENE));
				daeURI uri (*physicsScene);
				uri.set("", "", "", "", physicsScene->getId());
				//uri.setElement (physicsScene);
				//uri.resolveURI();
				ivs->setUrl (uri);
			}
		}
	}


	void LoadVisualScene(daeDocument* document, dModel* model, dLoaderContext* context, CollGeoCache& meshCache,
						 ModifierVertexCache& modifierVertexMapCache, CollMaterialCache& materialCache, CollImageCache& imageCache)
	{
		domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		_ASSERTE (domRoot );

		domCOLLADA::domScene *scene = domRoot->getScene();
		_ASSERTE (scene);

		// find the visual scene instance that is store in the scene node; 
		domInstanceWithExtra *instanceVisualScene = scene->getInstance_visual_scene();
		if (instanceVisualScene) {

			// if there is a instance to a Scene then Get teh Scene from the uri
			daeURI uri (instanceVisualScene->getUrl());

			daeElement* element = uri.getElement();
			_ASSERTE (element);

			// at this point we know that the collada element is a visual scene, we can cast it;
			//LoadScene (model, visualScene, nodeMap);
			LoadScene (document, model, (domVisual_scene*) element, context, meshCache, modifierVertexMapCache, materialCache, imageCache);
		}
	}



	void LoadVisualScene(daeDocument* document, dSceneModelList& sceneList, dLoaderContext* context, CollGeoCache& meshCache,
						 ModifierVertexCache& modifierVertexMapCache, CollMaterialCache& materialCache, CollImageCache& imageCache)
	{
		domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		_ASSERTE (domRoot );

//		domCOLLADA::domScene *scene = domRoot->getScene();
//		_ASSERTE (scene);

		const domLibrary_visual_scenes_Array &libraryArrays = domRoot->getLibrary_visual_scenes_array ();
		for (int j = 0; j < int (libraryArrays.getCount()); j ++) {
			const domVisual_scene_Array& libraryArray = libraryArrays[j]->getVisual_scene_array();
			for (int i = 0; i < int (libraryArray.getCount()); i ++) {
				dModel* model = context->CreateModel();
				domVisual_scene* visualScene = libraryArray[i];
				_ASSERTE (visualScene);
				LoadScene (document, model, visualScene, context, meshCache, modifierVertexMapCache, materialCache, imageCache);
				sceneList.AddModel(model);
				model->Release();
			}
		}
	}


	void LoadPhysicScene(daeDocument* document, NewtonWorld* world, dSceneModelList& sceneList, dLoaderContext* context, CollGeoCache& meshCache,
		ModifierVertexCache& modifierVertexMapCache, CollMaterialCache& materialCache, CollImageCache& imageCache)
	{
		domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
		_ASSERTE (domRoot );

		dTree<dModel*, domNode*> modelCache; 
		const domLibrary_visual_scenes_Array &libraryArrays = domRoot->getLibrary_visual_scenes_array ();
		for (int j = 0; j < int (libraryArrays.getCount()); j ++) {
			const domVisual_scene_Array& libraryArray = libraryArrays[j]->getVisual_scene_array();
			for (int i = 0; i < int (libraryArray.getCount()); i ++) {
				dModel* model = context->CreateModel();
				domVisual_scene* visualScene = libraryArray[i];
				_ASSERTE (visualScene);
				domNode* node = LoadScene (document, model, visualScene, context, meshCache, modifierVertexMapCache, materialCache, imageCache);
				sceneList.AddModel(model);
				model->Release();
				modelCache.Insert(model, node);
			}
		}



		domCOLLADA::domScene *scene = domRoot->getScene();
		_ASSERTE (scene);

		// find the visual scene instance that is store in the scene node; 
		domInstanceWithExtra_Array& physicIntanceArray = scene->getInstance_physics_scene_array();

		for (int i = 0; i < int (physicIntanceArray.getCount()); i ++) {
			domInstanceWithExtra *physicIntanceScene = physicIntanceArray[i];

			// if there is a instance to a Scene then Get the Scene from the uri
			daeURI uri (physicIntanceScene->getUrl());

			daeElement* element = uri.getElement();
			_ASSERTE (element);

			domPhysics_scene* physicScene = (domPhysics_scene*) element;
			domInstance_physics_model_Array& instancePhysicsModelArray = physicScene->getInstance_physics_model_array();
			for (int j = 0; j < int (instancePhysicsModelArray.getCount()); j ++) {
				domInstance_physics_model* instancePhysicsModel = instancePhysicsModelArray[j];

				daeURI bodyUri (instancePhysicsModel->getUrl());
				element = bodyUri.getElement();
				_ASSERTE (element);
				domPhysics_model* physicModel = (domPhysics_model*) element;
				domRigid_body_Array& bodyArray = physicModel->getRigid_body_array();

				domInstance_rigid_body_Array& instanceRigidBodyArray = instancePhysicsModel->getInstance_rigid_body_array();
				for (int k = 0; k < int (instanceRigidBodyArray.getCount()); k ++) {
					domInstance_rigid_body* instanceRigidBody = instanceRigidBodyArray[k];

					domRigid_body* rigidBody = NULL;
					for (int m = 0; m < int (bodyArray.getCount()); m ++) {
						if (!strcmp (instanceRigidBody->getBody(), bodyArray[m]->getSid())) {
							rigidBody = bodyArray[m];
							break;
						}
					}

					daeURI nodeUri (instanceRigidBody->getTarget());
					element = nodeUri.getElement();
					dModel *model = NULL;
					dMatrix matrix (GetIdentityMatrix());
					if (element) {
						domNode* node = (domNode*) element;

						matrix = GetMatrix (node);
						model = modelCache.Find(node)->GetInfo();
						//model = modelCache.Find((domNode*) element)->GetInfo();
					}
				
					NewtonBody* body = LoadRigidBody (world, rigidBody, model, context, meshCache, modifierVertexMapCache, materialCache, imageCache);

					matrix = m_globalRotation.Inverse() * matrix * m_globalRotation;
					matrix.m_posit = matrix.m_posit.Scale (m_scale);
					NewtonBodySetMatrix(body, &matrix[0][0]);

					domInstance_rigid_body::domTechnique_common* technique = instanceRigidBody->getTechnique_common();
					domInstance_rigid_body::domTechnique_common::domVelocity* initVeloc = technique->getVelocity();
					domInstance_rigid_body::domTechnique_common::domAngular_velocity* initOmega = technique->getAngular_velocity();

					dVector omega;
					dVector veloc;

					omega[0] = dFloat(initOmega->getValue()[0]);
					omega[1] = dFloat(initOmega->getValue()[1]);
					omega[2] = dFloat(initOmega->getValue()[2]);

					veloc[0] = dFloat(initVeloc->getValue()[0]);
					veloc[1] = dFloat(initVeloc->getValue()[1]);
					veloc[2] = dFloat(initVeloc->getValue()[2]);

					omega = m_globalRotation.RotateVector(omega);
					veloc = m_globalRotation.RotateVector(veloc.Scale (m_scale));

					NewtonBodySetOmega(body, &omega[0]);
					NewtonBodySetVelocity(body, &veloc[0]);
				}
			}
		}
	}


	struct MaterialTrianglePair
	{
		int m_faceIndex;
		CollMaterial* m_material;
	};

	static int SortMatrialTriangles (const void *A, const void *B) 
	{
		const MaterialTrianglePair* vertexA = (MaterialTrianglePair*) A;
		const MaterialTrianglePair* vertexB = (MaterialTrianglePair*) B;

		if (vertexA[0].m_material < vertexB[0].m_material) {
			return -1;
		} else if (vertexA[0].m_material > vertexB[0].m_material) {
			return 1;
		} else {
			return 0;
		}
	}


	void GetOffssets (const domInputLocalOffset_Array& inputArray, SourceBuffer* sourceBuffers, 
					  SourceBuffer** posSource, int& vertexIndicesOffset, 
					  SourceBuffer** normalSrc, int& normalIndicesOffset, 
					  SourceBuffer** uvSource, int& uvIndicesOffset) 
	{
		int index = 0;
		*uvSource = NULL;
		*posSource = NULL;
		*normalSrc = NULL;
		uvIndicesOffset = 0;
		vertexIndicesOffset = 0;
		normalIndicesOffset = 0;
		for (int k = 0; k < int (inputArray.getCount()); k ++) {

			domInputLocalOffset* offset;
			offset = inputArray[k];

			daeURI uri (offset->getSource());
			daeElement* element = uri.getElement();
			_ASSERTE (element);
			if (!cdom::strcasecmp (offset->getSemantic(), COMMON_PROFILE_INPUT_VERTEX)) {
				vertexIndicesOffset = int (offset->getOffset());

				domVertices* vertices = (domVertices*) element;
				domInputLocal_Array &inputArray = vertices->getInput_array();
				for (int i = 0; i < int (inputArray.getCount()); i ++) {
					domInputLocal* input;
					domInputLocal* offset;
					offset = inputArray[i];
					input = inputArray[i];
					if (!cdom::strcasecmp (offset->getSemantic(), COMMON_PROFILE_INPUT_POSITION)) {
						domAccessor *accessor;
						domFloat_array *floatArray;
						domSource::domTechnique_common *technique;


						daeURI uri1 (input->getSource());
						element = uri1.getElement();
						_ASSERTE (element);
						domSource *source = (domSource *) element;

						floatArray = source->getFloat_array();
						domListOfFloats &srcArray = floatArray->getValue();

						technique = source->getTechnique_common();
						_ASSERTE (technique);

						accessor = technique->getAccessor();
						_ASSERTE (accessor);
						sourceBuffers[index].m_id = source->getId();
						sourceBuffers[index].m_stride = int (accessor->getStride());
						sourceBuffers[index].m_count = int (floatArray->getCount());
						sourceBuffers[index].m_data = &srcArray[0];
						*posSource = &sourceBuffers[index];
						index ++;
						break;
					}
				}

			} else if (!cdom::strcasecmp (offset->getSemantic(), COMMON_PROFILE_INPUT_NORMAL)) {
				normalIndicesOffset = int (offset->getOffset());

				domSource *source = (domSource*) element;

				domFloat_array *floatArray = source->getFloat_array();
				domListOfFloats &srcArray = floatArray->getValue();

				domSource::domTechnique_common *technique = source->getTechnique_common();
				_ASSERTE (technique);

				domAccessor *accessor = technique->getAccessor();
				_ASSERTE (accessor);
				sourceBuffers[index].m_id = source->getId();
				sourceBuffers[index].m_stride = int (accessor->getStride());
				sourceBuffers[index].m_count = int (floatArray->getCount());
				sourceBuffers[index].m_data = &srcArray[0];
				*normalSrc = &sourceBuffers[index];
				index ++;

			} else if (!cdom::strcasecmp (offset->getSemantic(), COMMON_PROFILE_INPUT_TEXCOORD)) {
				uvIndicesOffset = int (offset->getOffset());

				domSource *source = (domSource*) element;
				domFloat_array *floatArray = source->getFloat_array();
				domListOfFloats &srcArray = floatArray->getValue();

				domSource::domTechnique_common *technique = source->getTechnique_common();
				_ASSERTE (technique);

				domAccessor *accessor = technique->getAccessor();
				_ASSERTE (accessor);
				sourceBuffers[index].m_id = source->getId();
				sourceBuffers[index].m_stride = int (accessor->getStride());
				sourceBuffers[index].m_count = int (floatArray->getCount());
				sourceBuffers[index].m_data = &srcArray[0];
				*uvSource = &sourceBuffers[index];
				index ++;
			}
		}
		_ASSERTE (posSource);
	}


	// check image duplicates in the library
	domImage* FindImage (domLibrary_images* library, const char*name) const
	{
		if (name && library) {
			const domImage_Array& array = library->getImage_array();
			for (unsigned i = 0; i < array.getCount(); i ++) {
				domImage* image = array[i];
				if (!strcmp (name, image->getId())) {
					return image;
				}
			}
		}
		return NULL;
	}

	domGeometry* FindGeometry (domLibrary_geometries* library, const char*name) const
	{
		if (name && library) {
			const domGeometry_Array& array = library->getGeometry_array();
			for (unsigned i = 0; i < array.getCount(); i ++) {
				domGeometry* image = array[i];
				if (!strcmp (name, image->getId())) {
					return image;
				}
			}
		}
		return NULL;
	}

	domAnimation* FindAnimation (domLibrary_animations* library, const char*name) const
	{
		if (name && library) {
			const domAnimation_Array& array = library->getAnimation_array();
			for (unsigned i = 0; i < array.getCount(); i ++) {
				domAnimation* image = array[i];
				if (!strcmp (name, image->getId())) {
					return image;
				}
			}
		}
		return NULL;
	}

	domMaterial* FindMaterial (domLibrary_materials* library, const char* name) const
	{
		if (name && library) {
			const domMaterial_Array& array = library->getMaterial_array();
			for (unsigned i = 0; i < array.getCount(); i ++) {
				domMaterial* material = array[i];
				if (!cdom::strcasecmp (name, material->getId())) {
					return material;
				}
			}
		}
		return NULL;
	}


	domTechnique *FindProfileExtra (domExtra_Array& extraArray, const char* keyword)
	{
		// if the geometry have extras, the search the Veter format option 
		for (int j = 0; j < int (extraArray.getCount()); j ++) {
			domExtra* extra = extraArray[j];
			const domTechnique_Array& techniqueArray = extra->getTechnique_array();
			for (int k = 0; k < int (techniqueArray.getCount()); k ++) {
				if (!cdom::strcasecmp (techniqueArray[k]->getProfile(), keyword)) {
					return techniqueArray[k];
				}
			}
		}

		return NULL;
	}




	CollMaterial* GetMaterial (const char* matName, CollMaterialCache& materialCache, CollImageCache& imageCache)
	{
		daeDatabase* database = m_collada->getDatabase();
		_ASSERTE (database);

		daeDocument *document = database->getDocument(daeUInt (0));
		_ASSERTE (document);

		domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());

		domLibrary_materials* library = NULL;
		if (domRoot->getLibrary_materials_array().getCount()) {
			library = domRoot->getLibrary_materials_array()[0];
		}


		CollMaterial* matId = NULL;
		domMaterial* material = FindMaterial (library, matName);
		if (material) {
			domInstance_effect* instanceEffect;
			instanceEffect = material->getInstance_effect();
			_ASSERTE (instanceEffect);
			if (instanceEffect) {
				daeURI uri (instanceEffect->getUrl());
				daeElement* element = uri.getElement();
				_ASSERTE (element);

	//				effect = FindEffect (effectLibrary, uri.getID());
				domEffect* effect = (domEffect*) element;
				if (effect) {
					matId = materialCache.GetMaterial (effect->getId());
					if (!matId) {
						domLibrary_images* imageLibrary;

						imageLibrary = NULL;
						if (domRoot->getLibrary_images_array().getCount()) {
							imageLibrary = domRoot->getLibrary_images_array()[0];
							_ASSERTE (imageLibrary);
						}

						const domFx_profile_abstract_Array &profileArray = effect->getFx_profile_abstract_array();

						domProfile_COMMON *profile = daeSafeCast<domProfile_COMMON> (profileArray[0]);
						if (profile) {
							// and the diffuse color from the type form the material type
							domProfile_COMMON::domTechnique* technique = profile->getTechnique();
							_ASSERTE (technique);

							domCommon_color_or_texture_type* diffuse = NULL;
							if (technique->getLambert()) {
								domProfile_COMMON::domTechnique::domLambert* parameters;
								parameters = technique->getLambert();
								_ASSERTE (parameters);
								diffuse = parameters->getDiffuse();
							} else if (technique->getPhong()) {
								domProfile_COMMON::domTechnique::domPhong* parameters;
								parameters = technique->getPhong();
								_ASSERTE (parameters);
								diffuse = parameters->getDiffuse();
							} else if (technique->getBlinn()) {
								domProfile_COMMON::domTechnique::domBlinn* parameters;
								parameters = technique->getBlinn();
								_ASSERTE (parameters);
								diffuse = parameters->getDiffuse();
							} else {
								_ASSERTE (0);
							}

							// since this is a prove of concept we only support the diffuse color texture feature
							// more complex material are unsupported at these time
							if (diffuse) {
								domCommon_color_or_texture_type::domTexture *texture;
								texture = diffuse->getTexture();
								if (texture) {
									const char* textName;
									textName = NULL;
									const char* textSample = texture->getTexture();
									domCommon_newparam_type_Array& paramaArray = profile->getNewparam_array();

									// find the texture sample 
									for (int j = 0; !textName && (j < int (paramaArray.getCount())); j ++) {
										const char* name = paramaArray[j]->getSid();
										if (!cdom::strcasecmp (name, textSample)) {
											domFx_sampler2D_common* sampler2d;
											domCommon_newparam_type_complexType* type;
											domFx_sampler2D_common_complexType::domSource* source;

											type = paramaArray[j];
											sampler2d = type->getSampler2D();
											source = sampler2d->getSource();
											const char* textSample2d = source->getValue();

											// find the image name for this diffuse effect
											for (int k = 0; !textName && (k < int (paramaArray.getCount())); k ++) {
												const char* name = paramaArray[k]->getSid(); 
												if (!cdom::strcasecmp (name, textSample2d)) {
													domFx_surface_common* surface;

													type = paramaArray[k];
													surface = type->getSurface();
													//domFx_surface_common::domInit_from_Array& initFromArray = surface->getInit_from_array();

													domFx_surface_init_commonRef surfInit = surface->getFx_surface_init_common();
													domFx_surface_init_from_common_Array initFromArray = surfInit->getInit_from_array();

													// finally get the name of the texture and bail out from form all loops
													for (int m = 0; !textName && (m < int (paramaArray.getCount())); m ++) {
														const xsIDREF& id = initFromArray[m]->getValue();
														textName = id.getID();
													}
												}
											}
										}
									}

									// if we have a texture name save this material as one of the material used by the geometries
									if (textName) {
										domImage* image = FindImage (imageLibrary, textName);
										_ASSERTE (image);

										domImage::domInit_from* initfrom = image->getInit_from();
										_ASSERTE (initfrom);

										if (initfrom->getValue().getOriginalURI()) {
											CollMaterial collMaterial;
											xsAnyURI uri (initfrom->getValue());
											textName = uri.getURI();

											// add the full path texture name to the cache
											imageCache.AddTexture (textName);

											// now get a filtered texture name
											collMaterial.m_texture = imageCache.GetTexture (textName);

											materialCache.AddMaterial(collMaterial, effect->getId());
											matId = materialCache.GetMaterial (effect->getId());
										}
									}

								} else {
									// no technique for this segment only color
									//_ASSERTE (0);
								}
							}
						}

					}
				}
			}
		}


		return matId;
	}


	void LoadTriangles(domTriangles_Array &trianglesArray, CollMaterialCache& materialCache, CollImageCache& imageCache,
					   MeshPoint* points, MaterialTrianglePair* materialTriangle, int& vertexCount)
	{
		// load the triangles
		for (int j = 0; j < int (trianglesArray.getCount()); j ++) {
			domTriangles* triangles = trianglesArray[j];;

			CollMaterial* matId = NULL;
			if (triangles->getMaterial()) {
				matId = GetMaterial (triangles->getMaterial(), materialCache, imageCache);
			}

			int triangleCount = int (triangles->getCount());
			domP *p = triangles->getP();
			domListOfUInts& indices = p->getValue();

			int uvIndicesOffset = 0;
			int vertexIndicesOffset = 0;
			int normalIndicesOffset = 0;
			SourceBuffer *uvSource = NULL;
			SourceBuffer *posSource = NULL;
			SourceBuffer *normalSrc = NULL;
			SourceBuffer sourceBuffers[6];

			GetOffssets (triangles->getInput_array(), sourceBuffers, &posSource, vertexIndicesOffset, &normalSrc, normalIndicesOffset, &uvSource, uvIndicesOffset);

			int index = vertexCount;
			vertexCount += triangleCount * 3;
			int indexCount = int (indices.getCount());
			for (int k = 0; k < indexCount; k += int (triangles->getInput_array().getCount())) {

				int t;
				int m;
				int stride;

				t = index/3;
				materialTriangle[t].m_faceIndex = t;
				materialTriangle[t].m_material = matId;

				//m = int (vertexIndices[k]);
				m = int (indices[k + vertexIndicesOffset]);

				stride = int (posSource->m_stride);
				points[index].m_vertex[0] = dFloat (posSource->m_data[m * stride + 0]);
				points[index].m_vertex[1] = dFloat (posSource->m_data[m * stride + 1]);
				points[index].m_vertex[2] = dFloat (posSource->m_data[m * stride + 2]);
				points[index].m_originalIndex = dFloat(m);

//				if (normalIndices && normalSrc) {
				if (normalSrc) {
//					m = int (indnormalIndices[k]);
					m = int (indices [k + normalIndicesOffset]);
					stride = int (posSource->m_stride);
					points[index].m_normal[0] = dFloat (normalSrc->m_data[m * stride + 0]);
					points[index].m_normal[1] = dFloat (normalSrc->m_data[m * stride + 1]);
					points[index].m_normal[2] = dFloat (normalSrc->m_data[m * stride + 2]);
				} else {
					points[index].m_normal[0] = 0.0f;
					points[index].m_normal[1] = 1.0f;
					points[index].m_normal[2] = 0.0f;
				}


				//				if (uvIndices && uvSource) {
				if (uvSource) {
					//					m = int (uvIndices[k]);
					m = int (indices[k + uvIndicesOffset]);
					stride = int (uvSource->m_stride);
					points[index].m_uv[0] = dFloat (uvSource->m_data[m * stride + 0]);
					points[index].m_uv[1] = dFloat (uvSource->m_data[m * stride + 1]);
				} else {
					points[index].m_uv[0] = 0.0f;
					points[index].m_uv[1] = 0.0f;
				}
				index ++;
			}
		}
	}


	void LoadPolygons(domPolylist_Array &polygonListArray, CollMaterialCache& materialCache, CollImageCache& imageCache, 
					  MeshPoint* points, MaterialTrianglePair* materialTriangle, int& vertexCount)
	{
		// load the polygonList array
		for (int j = 0; j < int (polygonListArray.getCount()); j ++) {

			domPolylist* polygon = polygonListArray[j];
			CollMaterial* matId = NULL;
			if (polygon->getMaterial()) {
				matId = GetMaterial (polygon->getMaterial(), materialCache, imageCache);
			}

			int uvIndicesOffset = 0;
			int vertexIndicesOffset = 0;
			int normalIndicesOffset = 0;
			SourceBuffer *uvSource = NULL;
			SourceBuffer *posSource = NULL;
			SourceBuffer *normalSrc = NULL;
			SourceBuffer sourceBuffers[6];
			GetOffssets (polygon->getInput_array(), sourceBuffers, &posSource, vertexIndicesOffset, &normalSrc, normalIndicesOffset, &uvSource, uvIndicesOffset);

			int stride = int (polygon->getInput_array().getCount());

			domPolylist::domVcount* vcount = polygon->getVcount();
			const domListOfUInts& vIndices = vcount->getValue();

			domPRef elemP = polygon->getP();
			const domListOfUInts& dataIndices = elemP->getValue();
			const domUint* indexP = &dataIndices[0];

			int pointsStride = int (posSource->m_stride);
			int pointcount = int (vIndices.getCount());
			for (int k = 0; k < pointcount; k ++) {
				int count = int (vIndices[k]);
				for (int i = 2; i < count; i ++) {
					int t;
					int m;

					t = vertexCount/3;
					materialTriangle[t].m_faceIndex = t;
					materialTriangle[t].m_material = matId;

					m = int (indexP[0 + vertexIndicesOffset]);
					points[vertexCount].m_vertex[0] = dFloat (posSource->m_data[m * pointsStride + 0]);
					points[vertexCount].m_vertex[1] = dFloat (posSource->m_data[m * pointsStride + 1]);
					points[vertexCount].m_vertex[2] = dFloat (posSource->m_data[m * pointsStride + 2]);
					points[vertexCount].m_originalIndex = dFloat(m);

					if (normalSrc) {
						int normalStride;
						normalStride = int (normalSrc->m_stride);
						m = int (indexP[0 + normalIndicesOffset]);
						normalStride = int (normalSrc->m_stride);
						points[vertexCount].m_normal[0] = dFloat (normalSrc->m_data[m * normalStride + 0]);
						points[vertexCount].m_normal[1] = dFloat (normalSrc->m_data[m * normalStride + 1]);
						points[vertexCount].m_normal[2] = dFloat (normalSrc->m_data[m * normalStride + 2]);
					} else {
						points[vertexCount].m_normal[0] = 0.0f;
						points[vertexCount].m_normal[1] = 1.0f;
						points[vertexCount].m_normal[2] = 0.0f;
					}

					if (uvSource) {
						int uvStride;
						m = int (indexP[0 + uvIndicesOffset]);
						uvStride = int (uvSource->m_stride);
						points[vertexCount].m_uv[0] = dFloat (uvSource->m_data[m * uvStride + 0]);
						points[vertexCount].m_uv[1] = dFloat (uvSource->m_data[m * uvStride + 1]);
					} else {
						points[vertexCount].m_uv[0] = 0.0f;
						points[vertexCount].m_uv[1] = 0.0f;
					}
					vertexCount ++;


					m = int (indexP[(i - 1) * stride + vertexIndicesOffset]);
					points[vertexCount].m_vertex[0] = dFloat (posSource->m_data[m * pointsStride + 0]);
					points[vertexCount].m_vertex[1] = dFloat (posSource->m_data[m * pointsStride + 1]);
					points[vertexCount].m_vertex[2] = dFloat (posSource->m_data[m * pointsStride + 2]);
					points[vertexCount].m_originalIndex = dFloat(m);
					if (normalSrc) {
						int normalStride;
						normalStride = int (normalSrc->m_stride);
						m = int (indexP[(i - 1) * stride + normalIndicesOffset]);
						normalStride = int (normalSrc->m_stride);
						points[vertexCount].m_normal[0] = dFloat (normalSrc->m_data[m * normalStride + 0]);
						points[vertexCount].m_normal[1] = dFloat (normalSrc->m_data[m * normalStride + 1]);
						points[vertexCount].m_normal[2] = dFloat (normalSrc->m_data[m * normalStride + 2]);
					} else {
						points[vertexCount].m_normal[0] = 0.0f;
						points[vertexCount].m_normal[1] = 1.0f;
						points[vertexCount].m_normal[2] = 0.0f;
					}
					if (uvSource) {
						int uvStride;
						m = int (indexP[(i - 1) * stride  + uvIndicesOffset]);
						uvStride = int (uvSource->m_stride);
						points[vertexCount].m_uv[0] = dFloat (uvSource->m_data[m * uvStride + 0]);
						points[vertexCount].m_uv[1] = dFloat (uvSource->m_data[m * uvStride + 1]);
					} else {
						points[vertexCount].m_uv[0] = 0.0f;
						points[vertexCount].m_uv[1] = 0.0f;
					}
					vertexCount ++;


					m = int (indexP[i * stride + vertexIndicesOffset]);
					points[vertexCount].m_vertex[0] = dFloat (posSource->m_data[m * pointsStride + 0]);
					points[vertexCount].m_vertex[1] = dFloat (posSource->m_data[m * pointsStride + 1]);
					points[vertexCount].m_vertex[2] = dFloat (posSource->m_data[m * pointsStride + 2]);
					points[vertexCount].m_originalIndex = dFloat(m);
					if (normalSrc) {
						int normalStride;
						normalStride = int (normalSrc->m_stride);
						m = int (indexP[i * stride + normalIndicesOffset]);
						normalStride = int (normalSrc->m_stride);
						points[vertexCount].m_normal[0] = dFloat (normalSrc->m_data[m * normalStride + 0]);
						points[vertexCount].m_normal[1] = dFloat (normalSrc->m_data[m * normalStride + 1]);
						points[vertexCount].m_normal[2] = dFloat (normalSrc->m_data[m * normalStride + 2]);
					} else {
						points[vertexCount].m_normal[0] = 0.0f;
						points[vertexCount].m_normal[1] = 1.0f;
						points[vertexCount].m_normal[2] = 0.0f;
					}
					if (uvSource) {
						int uvStride;
						m = int (indexP[i * stride + uvIndicesOffset]);
						uvStride = int (uvSource->m_stride);
						points[vertexCount].m_uv[0] = dFloat (uvSource->m_data[m * uvStride + 0]);
						points[vertexCount].m_uv[1] = dFloat (uvSource->m_data[m * uvStride + 1]);
					} else {
						points[vertexCount].m_uv[0] = 0.0f;
						points[vertexCount].m_uv[1] = 0.0f;
					}
					vertexCount ++;
				}
				indexP += count * stride;
			}
		}
	}

	void LoadPolygons(domPolygons_Array &polygonArray, CollMaterialCache& materialCache, CollImageCache& imageCache, 
					  MeshPoint* points, MaterialTrianglePair* materialTriangle, int& vertexCount)
	{
		// load the polygons
		for (int j = 0; j < int (polygonArray.getCount()); j ++) {
			domPolygons* polygons = polygonArray[j];

			CollMaterial* matId = NULL;
			if (polygons->getMaterial()) {
				matId = GetMaterial (polygons->getMaterial(), materialCache, imageCache);
			}

			int polygonCount = int (polygons->getCount());
			int stride = int (polygons->getInput_array().getCount());

			int uvIndicesOffset = 0;
			int vertexIndicesOffset = 0;
			int normalIndicesOffset = 0;
			SourceBuffer *uvSource = NULL;
			SourceBuffer *posSource = NULL;
			SourceBuffer *normalSrc = NULL;
			SourceBuffer sourceBuffers[6];
			GetOffssets (polygons->getInput_array(), sourceBuffers, &posSource, vertexIndicesOffset, &normalSrc, normalIndicesOffset, &uvSource, uvIndicesOffset);

			domP_Array& indexArray = polygons->getP_array();
			_ASSERTE (int (indexArray.getCount()) == polygonCount);
			for (int k = 0; k < polygonCount; k ++) {
				int vCount;
				int pointsStride;

				domPRef pIndices = indexArray[k];
				domListOfUInts& dataIndices = pIndices->getValue();

				vCount = int (dataIndices.getCount() / stride);
				pointsStride = int (posSource->m_stride);

				for (int i = 2; i < vCount; i ++) {
					int t;
					int m;

					t = vertexCount/3;
					materialTriangle[t].m_faceIndex = t;
					materialTriangle[t].m_material = matId;

					m = int (dataIndices[0 + vertexIndicesOffset]);
					points[vertexCount].m_vertex[0] = dFloat (posSource->m_data[m * pointsStride + 0]);
					points[vertexCount].m_vertex[1] = dFloat (posSource->m_data[m * pointsStride + 1]);
					points[vertexCount].m_vertex[2] = dFloat (posSource->m_data[m * pointsStride + 2]);
					points[vertexCount].m_originalIndex = dFloat(m);
					if (normalSrc) {
						int normalStride;
						normalStride = int (normalSrc->m_stride);
						m = int (dataIndices[0 + normalIndicesOffset]);
						normalStride = int (normalSrc->m_stride);
						points[vertexCount].m_normal[0] = dFloat (normalSrc->m_data[m * normalStride + 0]);
						points[vertexCount].m_normal[1] = dFloat (normalSrc->m_data[m * normalStride + 1]);
						points[vertexCount].m_normal[2] = dFloat (normalSrc->m_data[m * normalStride + 2]);
					} else {
						points[vertexCount].m_normal[0] = 0.0f;
						points[vertexCount].m_normal[1] = 1.0f;
						points[vertexCount].m_normal[2] = 0.0f;
					}
					if (uvSource) {
						int uvStride;
						m = int (dataIndices[0 + uvIndicesOffset]);
						uvStride = int (uvSource->m_stride);
						points[vertexCount].m_uv[0] = dFloat (uvSource->m_data[m * uvStride + 0]);
						points[vertexCount].m_uv[1] = dFloat (uvSource->m_data[m * uvStride + 1]);
					} else {
						points[vertexCount].m_uv[0] = 0.0f;
						points[vertexCount].m_uv[1] = 0.0f;
					}
					vertexCount ++;


					m = int (dataIndices[(i - 1) * stride + vertexIndicesOffset]);
					points[vertexCount].m_vertex[0] = dFloat (posSource->m_data[m * pointsStride + 0]);
					points[vertexCount].m_vertex[1] = dFloat (posSource->m_data[m * pointsStride + 1]);
					points[vertexCount].m_vertex[2] = dFloat (posSource->m_data[m * pointsStride + 2]);
					points[vertexCount].m_originalIndex = dFloat(m);
					if (normalSrc) {
						int normalStride;
						normalStride = int (normalSrc->m_stride);
						m = int (dataIndices[(i - 1) * stride + normalIndicesOffset]);
						normalStride = int (normalSrc->m_stride);
						points[vertexCount].m_normal[0] = dFloat (normalSrc->m_data[m * normalStride + 0]);
						points[vertexCount].m_normal[1] = dFloat (normalSrc->m_data[m * normalStride + 1]);
						points[vertexCount].m_normal[2] = dFloat (normalSrc->m_data[m * normalStride + 2]);
					} else {
						points[vertexCount].m_normal[0] = 0.0f;
						points[vertexCount].m_normal[1] = 1.0f;
						points[vertexCount].m_normal[2] = 0.0f;
					}
					if (uvSource) {
						int uvStride;
						m = int (dataIndices[(i - 1) * stride  + uvIndicesOffset]);
						uvStride = int (uvSource->m_stride);
						points[vertexCount].m_uv[0] = dFloat (uvSource->m_data[m * uvStride + 0]);
						points[vertexCount].m_uv[1] = dFloat (uvSource->m_data[m * uvStride + 1]);
					} else {
						points[vertexCount].m_uv[0] = 0.0f;
						points[vertexCount].m_uv[1] = 0.0f;
					}
					vertexCount ++;


					m = int (dataIndices[i * stride + vertexIndicesOffset]);
					points[vertexCount].m_vertex[0] = dFloat (posSource->m_data[m * pointsStride + 0]);
					points[vertexCount].m_vertex[1] = dFloat (posSource->m_data[m * pointsStride + 1]);
					points[vertexCount].m_vertex[2] = dFloat (posSource->m_data[m * pointsStride + 2]);
					points[vertexCount].m_originalIndex = dFloat(m);
					if (normalSrc) {
						int normalStride;
						normalStride = int (normalSrc->m_stride);
						m = int (dataIndices[i * stride + normalIndicesOffset]);
						normalStride = int (normalSrc->m_stride);
						points[vertexCount].m_normal[0] = dFloat (normalSrc->m_data[m * normalStride + 0]);
						points[vertexCount].m_normal[1] = dFloat (normalSrc->m_data[m * normalStride + 1]);
						points[vertexCount].m_normal[2] = dFloat (normalSrc->m_data[m * normalStride + 2]);
					} else {
						points[vertexCount].m_normal[0] = 0.0f;
						points[vertexCount].m_normal[1] = 1.0f;
						points[vertexCount].m_normal[2] = 0.0f;
					}
					if (uvSource) {
						int uvStride;
						m = int (dataIndices[i * stride + uvIndicesOffset]);
						uvStride = int (uvSource->m_stride);
						points[vertexCount].m_uv[0] = dFloat (uvSource->m_data[m * uvStride + 0]);
						points[vertexCount].m_uv[1] = dFloat (uvSource->m_data[m * uvStride + 1]);
					} else {
						points[vertexCount].m_uv[0] = 0.0f;
						points[vertexCount].m_uv[1] = 0.0f;
					}
					vertexCount ++;
				}
			}
		}
	}


	dMesh* LoadMesh (domMesh* colladaMesh, dLoaderContext* context, //domGeometry *colladaGeometry,
					 ModifierVertexCache& modifierVertexMapCache, CollMaterialCache& materialCache, CollImageCache& imageCache)
	{
		domGeometry* collGeometry = (domGeometry*) colladaMesh->getParent();
		dMesh* geometry = context->CreateMesh(collGeometry->getId());

		// calculate the total triangle count
		int vertexCount = 0;

		// count polygons in polygons list array
		domPolylist_Array &polygonListArray = colladaMesh->getPolylist_array();
		for (int j = 0; j < int (polygonListArray.getCount()); j ++) {
			domPolylist* polygon;
			domPolylist::domVcount* vcount;

			polygon = polygonListArray[j];
			vcount = polygon->getVcount();
			const domListOfUInts& vIndices = vcount->getValue();
			for (int k = 0; k < int (vIndices.getCount()); k ++) {
				int count = int (vIndices[k]);
				vertexCount += (count - 2) * 3;
			}
		}


		// count polygons
		domPolygons_Array &polygonArray = colladaMesh->getPolygons_array();
		for (int j = 0; j < int (polygonArray.getCount()); j ++) {
			int stride;
			int polygonCount;
			domPolygons* polygons;
			polygons = polygonArray[j];

			polygonCount = int (polygons->getCount());

			stride = int (polygons->getInput_array().getCount());
			domP_Array& indexArray = polygons->getP_array();

			_ASSERTE (int (indexArray.getCount()) == polygonCount);
			for (int k = 0; k < polygonCount; k ++) {
				int vCount;
				domPRef pIndices = indexArray[k];
				domListOfUInts& dataIndices = pIndices->getValue();
				vCount = int (dataIndices.getCount() / stride);
				vertexCount += (vCount - 2) * 3;
			}
		}

		// count triangles
		domTriangles_Array &trianglesArray = colladaMesh->getTriangles_array();
		for (int j = 0; j < int (trianglesArray.getCount()); j ++) {
			domTriangles* triangles;
			triangles = trianglesArray[j];
			vertexCount += int (triangles->getCount()) * 3;
		}

		int totalTriangleCount = vertexCount / 3;
		MaterialTrianglePair* materialTriangle = new MaterialTrianglePair[totalTriangleCount + 1];
		int* indexList = new int[vertexCount];
		MeshPoint* points = new MeshPoint[vertexCount];

		vertexCount = 0;

		LoadTriangles(trianglesArray, materialCache, imageCache, points, materialTriangle, vertexCount);
		LoadPolygons (polygonListArray, materialCache, imageCache, points, materialTriangle, vertexCount);
		LoadPolygons(polygonArray, materialCache, imageCache, points, materialTriangle, vertexCount);


		vertexCount = dModel::dPackVertexArray (&points[0].m_vertex[0], 8, sizeof (MeshPoint), vertexCount, indexList);
		// create space to store the vertices, normals and uvs
		geometry->AllocVertexData (vertexCount);
		VerterMap* vertexMap = new VerterMap[vertexCount + 1];

		vertexMap[vertexCount].m_vertexIndexInDMesh = 0x7fffffff;
		vertexMap[vertexCount].m_vertexIndexInColladaMesh = 0x7fffffff;
		modifierVertexMapCache.Insert(vertexMap, geometry);

		for (int j = 0; j < vertexCount; j ++) {
			dVector p(points[j].m_vertex[0], points[j].m_vertex[1], points[j].m_vertex[2], 0.0f);
			geometry->m_vertex[j * 3 + 0] = p.m_x;
			geometry->m_vertex[j * 3 + 1] = p.m_y;
			geometry->m_vertex[j * 3 + 2] = p.m_z;

			dVector n(points[j].m_normal[0], points[j].m_normal[1], points[j].m_normal[2], 0.0f);
			geometry->m_normal[j * 3 + 0] = n.m_x;
			geometry->m_normal[j * 3 + 1] = n.m_y;
			geometry->m_normal[j * 3 + 2] = n.m_z;

			geometry->m_uv[j * 2 + 0] = points[j].m_uv[0];
			geometry->m_uv[j * 2 + 1] = points[j].m_uv[1];

			vertexMap[j].m_vertexIndexInDMesh = j;
			vertexMap[j].m_vertexIndexInColladaMesh = int (points[j].m_originalIndex);
		}


		materialTriangle[totalTriangleCount].m_material = (CollMaterial*) 0x7fffffff;
		qsort(materialTriangle, totalTriangleCount, sizeof (MaterialTrianglePair), SortMatrialTriangles);
		for (int j = 0; j < totalTriangleCount; ) {
			int i ;
			int triangleCount;
			CollMaterial* material;
			i = j;
			material = materialTriangle[i].m_material;
			while (material == materialTriangle[j].m_material) j ++;

			triangleCount = j - i;

			dSubMesh& subMesh = geometry->Append()->GetInfo() ;
			subMesh.AllocIndexData(triangleCount * 3);

			if (material) {
				strcpy (subMesh.m_textureName, material->m_texture.m_textureName);
			}
			for (int k = 0; k < triangleCount; k ++) {
				int index;
				index = materialTriangle[k + i].m_faceIndex;
				subMesh.m_indexes[k * 3 + 0] = indexList[index * 3 + 0];
				subMesh.m_indexes[k * 3 + 1] = indexList[index * 3 + 1];
				subMesh.m_indexes[k * 3 + 2] = indexList[index * 3 + 2];
			}
		}

		delete[] indexList;
		delete[] points;
		delete[] materialTriangle;

		return geometry;
	}


	dMesh* LoadConvexMesh (domConvex_mesh* colladaMesh, dLoaderContext* context, //domGeometry *colladaGeometry,
		ModifierVertexCache& modifierVertexMapCache, CollMaterialCache& materialCache, CollImageCache& imageCache)
	{
		domGeometry* collGeometry = (domGeometry*) colladaMesh->getParent();
//		sprintf (geometry->m_name, "%s", collGeometry->getId());

		dMesh* geometry = context->CreateMesh(collGeometry->getId());

		// calculate the total triangle count
		int vertexCount = 0;

		// count polygons in polygons list array
		domPolylist_Array &polygonListArray = colladaMesh->getPolylist_array();
		for (int j = 0; j < int (polygonListArray.getCount()); j ++) {
			domPolylist* polygon;
			domPolylist::domVcount* vcount;

			polygon = polygonListArray[j];
			vcount = polygon->getVcount();
			const domListOfUInts& vIndices = vcount->getValue();
			for (int k = 0; k < int (vIndices.getCount()); k ++) {
				int count = int (vIndices[k]);
				vertexCount += (count - 2) * 3;
			}
		}


		// count polygons
		domPolygons_Array &polygonArray = colladaMesh->getPolygons_array();
		for (int j = 0; j < int (polygonArray.getCount()); j ++) {
			int stride;
			int polygonCount;
			domPolygons* polygons;
			polygons = polygonArray[j];

			polygonCount = int (polygons->getCount());

			stride = int (polygons->getInput_array().getCount());
			domP_Array& indexArray = polygons->getP_array();

			_ASSERTE (int (indexArray.getCount()) == polygonCount);
			for (int k = 0; k < polygonCount; k ++) {
				int vCount;
				domPRef pIndices = indexArray[k];
				domListOfUInts& dataIndices = pIndices->getValue();
				vCount = int (dataIndices.getCount() / stride);
				vertexCount += (vCount - 2) * 3;
			}
		}

		// count triangles
		domTriangles_Array &trianglesArray = colladaMesh->getTriangles_array();
		for (int j = 0; j < int (trianglesArray.getCount()); j ++) {
			domTriangles* triangles;
			triangles = trianglesArray[j];
			vertexCount += int (triangles->getCount()) * 3;
		}

		int totalTriangleCount = vertexCount / 3;
		MaterialTrianglePair* materialTriangle = new MaterialTrianglePair[totalTriangleCount + 1];
		int* indexList = new int[vertexCount];
		MeshPoint* points = new MeshPoint[vertexCount];

		vertexCount = 0;

		LoadTriangles(trianglesArray, materialCache, imageCache, points, materialTriangle, vertexCount);
		LoadPolygons (polygonListArray, materialCache, imageCache, points, materialTriangle, vertexCount);
		LoadPolygons(polygonArray, materialCache, imageCache, points, materialTriangle, vertexCount);


		vertexCount = dModel::dPackVertexArray (&points[0].m_vertex[0], 8, sizeof (MeshPoint), vertexCount, indexList);
		// create space to store the vertices, normals and uvs
		geometry->AllocVertexData (vertexCount);
		VerterMap* vertexMap = new VerterMap[vertexCount + 1];

		vertexMap[vertexCount].m_vertexIndexInDMesh = 0x7fffffff;
		vertexMap[vertexCount].m_vertexIndexInColladaMesh = 0x7fffffff;
		modifierVertexMapCache.Insert(vertexMap, geometry);

		for (int j = 0; j < vertexCount; j ++) {
			dVector p(points[j].m_vertex[0], points[j].m_vertex[1], points[j].m_vertex[2], 0.0f);
			geometry->m_vertex[j * 3 + 0] = p.m_x;
			geometry->m_vertex[j * 3 + 1] = p.m_y;
			geometry->m_vertex[j * 3 + 2] = p.m_z;

			dVector n(points[j].m_normal[0], points[j].m_normal[1], points[j].m_normal[2], 0.0f);
			geometry->m_normal[j * 3 + 0] = n.m_x;
			geometry->m_normal[j * 3 + 1] = n.m_y;
			geometry->m_normal[j * 3 + 2] = n.m_z;

			geometry->m_uv[j * 2 + 0] = points[j].m_uv[0];
			geometry->m_uv[j * 2 + 1] = points[j].m_uv[1];

			vertexMap[j].m_vertexIndexInDMesh = j;
			vertexMap[j].m_vertexIndexInColladaMesh = int (points[j].m_originalIndex);
		}


		materialTriangle[totalTriangleCount].m_material = (CollMaterial*) 0x7fffffff;
		qsort(materialTriangle, totalTriangleCount, sizeof (MaterialTrianglePair), SortMatrialTriangles);
		for (int j = 0; j < totalTriangleCount; ) {
			int i ;
			int triangleCount;
			CollMaterial* material;
			i = j;
			material = materialTriangle[i].m_material;
			while (material == materialTriangle[j].m_material) j ++;

			triangleCount = j - i;

			dSubMesh& subMesh = geometry->Append()->GetInfo() ;
			subMesh.AllocIndexData(triangleCount * 3);

			if (material) {
				strcpy (subMesh.m_textureName, material->m_texture.m_textureName);
			}
			for (int k = 0; k < triangleCount; k ++) {
				int index;
				index = materialTriangle[k + i].m_faceIndex;
				subMesh.m_indexes[k * 3 + 0] = indexList[index * 3 + 0];
				subMesh.m_indexes[k * 3 + 1] = indexList[index * 3 + 1];
				subMesh.m_indexes[k * 3 + 2] = indexList[index * 3 + 2];
			}
		}

		delete[] indexList;
		delete[] points;
		delete[] materialTriangle;

		return geometry;
	}

	static int SortVertexMap (const void *A, const void *B) 
	{
		const VerterMap* vertexA = (VerterMap*) A;
		const VerterMap* vertexB = (VerterMap*) B;

		if (vertexA[0].m_vertexIndexInColladaMesh < vertexB[0].m_vertexIndexInColladaMesh) {
			return -1;
		} else if (vertexA[0].m_vertexIndexInColladaMesh > vertexB[0].m_vertexIndexInColladaMesh) {
			return 1;
		} else {
			return 0;
		}
	}


	dSkinModifier* LoadSkinController (daeDocument* document, domSkin* skin, dMesh* mesh, const dModel* model, ModifierVertexCache& modifierVertexMapCache)
	{
		dSkinModifier* skinModifier = new dSkinModifier (mesh);

		domSkin::domVertex_weights *vertexWeights = skin->getVertex_weights();
		domInputLocalOffset* weights = vertexWeights->getInput_array()[0];
		domInputLocalOffset* jointInputs = vertexWeights->getInput_array()[1];
		if (strcmp (jointInputs->getSemantic(), COMMON_PROFILE_INPUT_JOINT)) {
			domInputLocalOffset *tmp = weights;
			weights = jointInputs;
			jointInputs = tmp;
		}
		_ASSERTE (!strcmp (weights->getSemantic(), COMMON_PROFILE_INPUT_WEIGHT));
		_ASSERTE (!strcmp (jointInputs->getSemantic(), COMMON_PROFILE_INPUT_JOINT));


		int BoneIndices[1024];
		daeURI uri (jointInputs->getSource());
		const daeElement* element = uri.getElement ();
		domSource *boneSource = (domSource *)element;
		domName_array *boneNames = boneSource->getName_array();
		domListOfNames &boneSrcArray = boneNames->getValue();
		for (int i = 0; i < int (boneSrcArray.getCount()); i ++) {
			dBone* bone;
			bone = model->FindBone(boneSrcArray[i]);
			BoneIndices[i] = bone->GetBoneID();
		}


		daeURI uri1 (weights->getSource());
		element = uri1.getElement ();
		domSource *weightSource = (domSource *)element;
		domFloat_array *weightValue = weightSource->getFloat_array();
		domListOfFloats &weightValueArray = weightValue->getValue();

		dSkinModifier::dBoneVertexWeightData* skinData = new dSkinModifier::dBoneVertexWeightData[mesh->m_vertexCount * 4];
		domSkin::domVertex_weights::domV *v = vertexWeights->getV();
		domSkin::domVertex_weights::domVcount *vCount = vertexWeights->getVcount();

		domListOfInts &vIndices = v->getValue();
		domListOfUInts &vCountIndices = vCount->getValue();
		int jointOffset = int (jointInputs->getOffset());
		int weightOffset = int (weights->getOffset());

		VerterMap* vertexMap = modifierVertexMapCache.Find(mesh)->GetInfo();
		qsort(vertexMap, mesh->m_vertexCount, sizeof (VerterMap), SortVertexMap);


		int vertexIndex = 0;
		int weightsCount = 0;
		int weightStartIndex = 0;
		for (int collVertexIndex = 0; collVertexIndex < (int) vCountIndices.getCount(); collVertexIndex ++) {
			int count = int (vCountIndices[collVertexIndex]);
			for (int j = 0; j < count; j ++) {
				int boneIndex = int (vIndices[weightStartIndex * 2 + jointOffset]);
				int weightIndex = int (vIndices[weightStartIndex * 2 + weightOffset]);
				int boneId = BoneIndices[boneIndex];
				dFloat weightValue = dFloat (weightValueArray[weightIndex]);
				weightStartIndex ++;

				if (weightValue > 1.0e-3f) {
					for (int k = vertexIndex; vertexMap[k].m_vertexIndexInColladaMesh <= collVertexIndex; k ++) {
						skinData[weightsCount].m_boneId = boneId;
						skinData[weightsCount].m_weight = weightValue;
						skinData[weightsCount].m_vertexIndex = vertexMap[k].m_vertexIndexInDMesh;
						weightsCount ++;
					}
				}
			}

			do {
				vertexIndex ++;
			} while (vertexMap[vertexIndex].m_vertexIndexInColladaMesh == collVertexIndex);
		}

		skinModifier->SetBindingPose(mesh, *model, skinData, weightsCount);
		delete[] skinData;

		return skinModifier;
	}


	dMatrix GetOffssetMatrix (const domTranslate_Array &translationArray, const domRotate_Array &rotationArray)
	{
		dMatrix matrix (GetIdentityMatrix());

		for (int i = 0; i < int (translationArray.getCount()); i ++) {
			const domFloat3& data = translationArray[i]->getValue();
			matrix[3][0] += dFloat (data[0]);
			matrix[3][1] += dFloat (data[1]);
			matrix[3][2] += dFloat (data[2]);
		}

		for (int i = 0; i < int (rotationArray.getCount()); i ++) {
			const domFloat4& data = rotationArray[i]->getValue();
			dFloat angle = dFloat (data[3]) * 3.1316f / 180.0f;
			dFloat x = dFloat (data[0]);
			dFloat y = dFloat (data[1]);
			dFloat z = dFloat (data[2]);
			dVector axis (x, y, z, 0.0f);
			axis = axis.Scale (1.0f / sqrtf (axis % axis));
			dQuaternion rot (axis, angle);
			dMatrix colladaMatrix (rot, dVector (0.0f, 0.0f, 0.0f, 1.0f));
			matrix = colladaMatrix * matrix;
		}
		matrix = m_globalRotation.Inverse() * matrix * m_globalRotation;
		matrix.m_posit = matrix.m_posit.Scale(m_scale);
		return matrix;
	}


	NewtonCollision* CreateCollision(NewtonWorld* world, domRigid_body::domTechnique_common::domShape* shape,
									 CollGeoCache& meshCache, ModifierVertexCache& modifierVertexMapCache, 
									 CollMaterialCache& materialCache, CollImageCache& imageCache)
	{
		NewtonCollision* collision = NULL;

		int shapeID = 0;

		dMatrix matrix (GetOffssetMatrix (shape->getTranslate_array(), shape->getRotate_array()));
		if (shape->getBox()) {
			domBox* box = shape->getBox(); 
			domBox::domHalf_extents* halfExtend = box->getHalf_extents(); 
			dVector size (dFloat(halfExtend->getValue()[0]), dFloat(halfExtend->getValue()[1]), dFloat(halfExtend->getValue()[2]), 0.0f);
			size = size.Scale (m_scale * 2.0f);
			collision = NewtonCreateBox (world, size.m_x, size.m_y, size.m_z, shapeID, &matrix[0][0]);

		} else if (shape->getSphere()) {
			domSphere* sphere = shape->getSphere(); 

			dFloat radius0 = dFloat (sphere->getRadius()->getValue());
			dFloat radius1 = radius0;
			dFloat radius2 = radius0;
			
			domTechnique* technique = FindProfileExtra (sphere->getExtra_array(), "ellipse");
			if (technique) {
				const daeElementRefArray& array = technique->getContents();
				daeElement* element = array[0];
				domAny *domExtraExtension = (domAny*)element; 
				_ASSERTE (!strcmp (domExtraExtension->getElementName(), "radios"));

				const char* radios = domExtraExtension->getValue();
				sscanf (radios, "%f %f %f", &radius0, &radius1, &radius2);
			}
		
			collision = NewtonCreateSphere (world, radius0 * m_scale, radius1 * m_scale, radius2 * m_scale, shapeID, &matrix[0][0]);

		} else if (shape->getCylinder()) {

			domCylinder* cylinder = shape->getCylinder();
			dFloat height = dFloat (cylinder->getHeight()->getValue()) * m_scale * 2.0f; 
			dFloat radius = dFloat (cylinder->getRadius()->getValue()[0]) * m_scale;

			UnapplyMatrixAligment (matrix);
			domTechnique* technique = FindProfileExtra (cylinder->getExtra_array(), "chamferCylinder");
			if (technique) {
				collision = NewtonCreateChamferCylinder (world, radius, height, shapeID, &matrix[0][0]);
			} else {
				collision = NewtonCreateCylinder (world, radius, height, shapeID, &matrix[0][0]);
			}

		} else if (shape->getCapsule()) {

			domCapsule* capsule = shape->getCapsule(); 
			dFloat radius = dFloat (capsule->getRadius()->getValue()[0]); 
			dFloat height = (dFloat (capsule->getHeight()->getValue()) + radius) * 2.0f;

			radius *= m_scale;
			height *= m_scale;

			UnapplyMatrixAligment (matrix);
			collision = NewtonCreateCapsule (world, radius, height, shapeID, &matrix[0][0]);

		} else if (shape->getTapered_cylinder()) {

			domTapered_cylinder* cone = shape->getTapered_cylinder();
			dFloat height = dFloat (cone->getHeight()->getValue())  * m_scale * 2.0f;
			dFloat radius1 = dFloat (cone->getRadius1()->getValue()[0]) * m_scale;
			dFloat radius2 = dFloat (cone->getRadius2()->getValue()[0]) * m_scale;

			UnapplyMatrixAligment (matrix);
			if (radius1 >= radius2) {
				collision = NewtonCreateCone (world, radius1, height, shapeID, &matrix[0][0]);
			} else {
				matrix = dPitchMatrix (3.141592f) * matrix;
				collision = NewtonCreateCone (world, radius2, height, shapeID, &matrix[0][0]);
			}

		} else if (shape->getInstance_geometry()) {

			domInstance_geometry* instance = shape->getInstance_geometry(); 
			daeURI uri (instance->getUrl());
			const daeElement* element = uri.getElement();
			domGeometry *collGeometry = (domGeometry*) element;

			if (collGeometry->getConvex_mesh()) {
				CollGeoCache::dTreeNode* cacheNode = meshCache.Find(collGeometry);
				if (!cacheNode) {
					dLoaderContext context;
					dMesh* mesh = LoadConvexMesh (collGeometry->getConvex_mesh(), &context, modifierVertexMapCache, materialCache, imageCache);

					mesh->ApplyGlobalScale(m_scale);
					mesh->ApplyGlobalTransform (m_globalRotation);
					cacheNode = meshCache.Insert(mesh, collGeometry);
				}
				dMesh* mesh = cacheNode->GetInfo();
				collision = NewtonCreateConvexHull (world, mesh->m_vertexCount, mesh->m_vertex, 3 * sizeof (dFloat), 0.0f, shapeID, &matrix[0][0]);

			} else {

				CollGeoCache::dTreeNode* cacheNode = meshCache.Find(collGeometry);
				if (!cacheNode) {
					dLoaderContext context;
					dMesh* mesh = LoadMesh (collGeometry->getMesh(), &context, modifierVertexMapCache, materialCache, imageCache);

					mesh->ApplyGlobalScale(m_scale);
					mesh->ApplyGlobalTransform (m_globalRotation);
					cacheNode = meshCache.Insert(mesh, collGeometry);
				}
				dMesh* mesh = cacheNode->GetInfo();

				collision = NewtonCreateTreeCollision (world, NULL);
				NewtonTreeCollisionBeginBuild (collision);

				dFloat* const vertex = mesh->m_vertex;
				for (dMesh::dListNode* node = mesh->GetFirst(); node; node = node->GetNext()) {
					const dSubMesh& segment = node->GetInfo();
					for (int j = 0; j < segment.m_indexCount; j += 3) {
						dVector face[3];

						int index = segment.m_indexes[j + 0] * 3;
						face[0] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2], 0.0f);

						index = segment.m_indexes[j + 1] * 3;
						face[1] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2], 0.0f);

						index = segment.m_indexes[j + 2] * 3;
						face[2] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2], 0.0f);

						NewtonTreeCollisionAddFace (collision, 3, &face[0].m_x, sizeof (dVector), 0);
					}
				}

				NewtonTreeCollisionEndBuild (collision, 1);	

			}

		} else if (shape->getPlane()) {
			_ASSERTE (0);
/*
			domPlane* plane;
			domPlane::domEquation* equation;

			plane = shape->getPlane(); 
			equation = plane->getEquation(); 

			dVector surface (equation->getValue()[0], equation->getValue()[1], equation->getValue()[2], equation->getValue()[2]);
			surface = m_axisRotation.TransformPlane(surface);
			surface = matrix.TransformPlane (surface);
			collision = CreatePlaneCollidion (m_world, surface );
*/
		} else {
			_ASSERTE (0);
		}

		return collision;
	}


	NewtonCollision* CreateCollision(NewtonWorld* world, const domRigid_body::domTechnique_common::domShape_Array& shapeArray, int isDynamics,
									CollGeoCache& meshCache, ModifierVertexCache& modifierVertexMapCache, CollMaterialCache& materialCache, CollImageCache& imageCache)
	{
		NewtonCollision* collision = NULL;
		int count = int (shapeArray.getCount());
		if (count == 1)	{
			domRigid_body::domTechnique_common::domShape* shape = shapeArray[0];
			collision = CreateCollision(world, shape, meshCache, modifierVertexMapCache, materialCache, imageCache);
		} else {
			if (isDynamics) {
				NewtonCollision** collsionArray = new NewtonCollision*[count];
				_ASSERTE (count < sizeof (collsionArray) / sizeof (collsionArray[0]));
				for (int i = 0; i < count; i ++) {
					domRigid_body::domTechnique_common::domShape* shape = shapeArray[i];
					collsionArray[i] = CreateCollision(world, shape, meshCache, modifierVertexMapCache, materialCache, imageCache);
				}
				collision = NewtonCreateCompoundCollision (world, count, collsionArray, 0);
				
				for (int i = 0; i < count; i ++) {
					NewtonReleaseCollision (world, collsionArray[i]);
				}

				delete[] collsionArray;
			} else {
				domRigid_body::domTechnique_common::domShape* shape = shapeArray[0];
				collision = CreateCollision(world, shape, meshCache, modifierVertexMapCache, materialCache, imageCache);
			}
		}
		return collision;
	}



	NewtonBody* LoadRigidBody (NewtonWorld* world, domRigid_body* rigidBody, dModel* model, dLoaderContext* context,
							  CollGeoCache& meshCache, ModifierVertexCache& modifierVertexMapCache, 
							  CollMaterialCache& materialCache, CollImageCache& imageCache)
	{
//		int i;
//		int j;
//		int k;
//		int isDynamicsBody;
//		dFloat Ixx;
//		dFloat Iyy;
//		dFloat Izz;
//		dFloat mass;
//		dVector com;
//		domNode* target;
//		daeElement* element;
//		NewtonBody* body;
//		dSceneNode* viualNodePtr;
//		domRigid_body* rigidBody;
//		domTargetableFloat* bodyMass;
//		domTargetableFloat3* bodyInertia;
//		dNodePhysicsRigidBody::dTreeNode* rigidBodyNode;
//		ColladaNodeTodSceneNodeMap::dTreeNode* visualNode;
//		domRigid_body::domTechnique_common::domDynamic* dynamicBody;
//		domRigid_body::domTechnique_common::domMass_frame* massFrame;
//		domInstance_rigid_body* instanceRigidBody

//		element = instanceRigidBody->getTarget().getElement();
//		target = (domNode*) element;
//		visualNode = m_nodeTodNodeMap.Find(target);
//		rigidBodyNode = m_rigidBodyCache.Find (dCRC (instanceRigidBody->getBody()));
//		_ASSERTE (rigidBodyNode);
//		rigidBody = rigidBodyNode->GetInfo();


		domRigid_body::domTechnique_common* techniqueCommon = rigidBody->getTechnique_common();
		_ASSERTE (techniqueCommon);

		int isDynamicsBody = 1;
		if (techniqueCommon->getMass()) {
			if (techniqueCommon->getMass()->getValue() == 0.0f) {
				isDynamicsBody = 0;
			}
		}

		if (isDynamicsBody && techniqueCommon->getDynamic()) {
			if (techniqueCommon->getDynamic()->getValue() == false) {
				isDynamicsBody = 0;
			}
		}

		NewtonCollision* collision = CreateCollision(world, techniqueCommon->getShape_array(), isDynamicsBody,
													 meshCache, modifierVertexMapCache, materialCache, imageCache);


		NewtonBody* body = NewtonCreateBody (world, collision);
		NewtonReleaseCollision (world, collision);

//		m_rigidBodyMap.Insert(body, rigidBody);

		// set user data;
//		NewtonBodySetUserData (body, visualNode ? visualNode->GetInfo() : NULL);
//		NewtonBodySetTransformCallback (body, DefaultColladaPhysicsSetTransform);
//		NewtonBodySetForceAndTorqueCallback (body, DefaultColladaApplyGravityForce);

		// by default disable collision with jointed objects
//		NewtonBodySetJointRecursiveCollision (body, 0);

		// set a destructor for this rigid body
//		NewtonBodySetDestructorCallback (body, PhysicsBodyDestructor);

		// all body have default Material ID		
//		NewtonBodySetMaterialGroupID (body, colladaMaterialIndex);

		dFloat Ixx = 0.0f;
		dFloat Iyy = 0.0f;
		dFloat Izz = 0.0f;
		dFloat mass = 0.0f;
		dVector com (0.0f, 0.0f, 0.0f, 0.0f);
		domTargetableFloat* bodyMass = techniqueCommon->getMass();
		if (bodyMass) {
			domRigid_body::domTechnique_common::domDynamic* dynamicBody = techniqueCommon->getDynamic();
			if (dynamicBody) {
				if (dynamicBody->getValue() == true) {
					mass = dFloat (bodyMass->getValue());
				}
			}
			if (mass > 0.0f) {
				domRigid_body::domTechnique_common::domMass_frame* massFrame = techniqueCommon->getMass_frame();
				domTargetableFloat3* bodyInertia = techniqueCommon->getInertia();
				if (massFrame && bodyInertia) {
					Ixx = dFloat (bodyInertia->getValue()[0]);
					Iyy = dFloat (bodyInertia->getValue()[1]);
					Izz = dFloat (bodyInertia->getValue()[2]);
					if (massFrame->getTranslate_array().getCount()) {
						domTranslate* translation = massFrame->getTranslate_array()[0];
						com.m_x = dFloat (translation->getValue()[0]);
						com.m_y = dFloat (translation->getValue()[1]);
						com.m_z = dFloat (translation->getValue()[2]);
						com = m_globalRotation.RotateVector(com.Scale (m_scale));
					}
				} else {
					dVector inertia;
					NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &com[0]);	
					Ixx = inertia.m_x * mass;
					Iyy = inertia.m_y * mass;
					Izz = inertia.m_z * mass;
				}
			}
		}

		NewtonBodySetCentreOfMass (body, &com[0]);
		if (mass > 0.0f) {
			NewtonBodySetMassMatrix (body, mass, Ixx, Iyy, Izz);
		}

//		dMatrix matrix (GetIdentityMatrix());
//		if (visualNode) {
//			for (viualNodePtr = visualNode->GetInfo(); viualNodePtr; viualNodePtr = viualNodePtr->GetParent()) {
//				matrix = matrix * viualNodePtr->GetMatrix();
//			}
//			NewtonBodySetMatrix (body, &matrix[0][0]);
//		} else {
//			matrix.m_posit.m_y = - 1000.0f;
//		}


		// add a moronic callada style material;
//		if (visualNode) {
//			ColladaFrictionRestituionMaterial* frictionRestituionMaterial;
//			frictionRestituionMaterial = new ColladaFrictionRestituionMaterial;
//			visualNode->GetInfo()->SetUserData (frictionRestituionMaterial);

			// material for this body
//			domPhysics_material* physicMaterial;
//			domInstance_physics_material* intanceMaterial;
//			intanceMaterial = techniqueCommon->getInstance_physics_material();
//			if (intanceMaterial) {
//				const daeElement* element;
//				domPhysics_material::domTechnique_common* technique;
//				const daeURI& uri = intanceMaterial->getUrl();
//				element = uri.getElement();
//				if (element) {
//					physicMaterial = (domPhysics_material*) element;
//					technique = physicMaterial->getTechnique_common();
//					frictionRestituionMaterial->m_staticFriction = technique->getStatic_friction()->getValue();
//					frictionRestituionMaterial->m_restitution = technique->getRestitution()->getValue();
//					frictionRestituionMaterial->m_dynamicFriction = technique->getDynamic_friction()->getValue();
//				}
//			}
//		}


		domTechnique *technique = FindProfileExtra (rigidBody->getExtra_array(), "Newton");
		if (technique) {
			// it is a newton collada set specific parameters 
			const daeElementRefArray& array = technique->getContents();
			for (int i = 0; i < int (array.getCount()); i ++) {
				daeElement* element = array[i];
				domAny *domExtraExtension = (domAny*)element; 

				if (!cdom::strcasecmp (domExtraExtension->getElementName(), "BindingUserData")) {
					context->SetUserData(body, model, domExtraExtension->getValue());

				} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "BindingSetTransformFunction")) {
					context->SetTransformCallback(body, domExtraExtension->getValue());

				} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "BindingExternalForceFunction")) {
					context->SetForceAndTorqueCallback(body, domExtraExtension->getValue());

				} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "BindingBodyDestructorCallback")) {
					context->SetDestructorCallback(body, domExtraExtension->getValue());

				} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "AutoSleepMode")) {
					if (!cdom::strcasecmp (domExtraExtension->getValue(), "true")) {
						NewtonBodySetAutoSleep (body, 1);
					} else {
						NewtonBodySetAutoSleep (body, 0);
					}

				} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "InternalLinearDrag")) {
					dFloat drag;
					sscanf (domExtraExtension->getValue(), "%f", &drag);
					NewtonBodySetLinearDamping(body, drag);

				} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "InternalAngularDrag")) {
					dFloat drag[4];
					sscanf (domExtraExtension->getValue(), "%f", &drag[0]);
					drag[1] = drag[0];
					drag[2] = drag[0];
					NewtonBodySetAngularDamping (body, &drag[0]);

				} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "InternalContinueCollision")) {
					if (!cdom::strcasecmp (domExtraExtension->getValue(), "true")) {
						NewtonBodySetContinuousCollisionMode(body, 1);
					} else {
						NewtonBodySetContinuousCollisionMode (body, 0);
					}

				} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "RecursivelyCollideWithLinkedBodies")) {
					if (!cdom::strcasecmp (domExtraExtension->getValue(), "true")) {
						NewtonBodySetJointRecursiveCollision(body, 1);
					} else {
						NewtonBodySetJointRecursiveCollision (body, 0);
					}

				} else if (!cdom::strcasecmp (domExtraExtension->getElementName(), "UnilateralMaterialID")) {
					int materiaID;
					sscanf (domExtraExtension->getValue(), "%d", &materiaID);

					// for now since collada do not support realistic manifold materials 
					materiaID = 0;
					NewtonBodySetMaterialGroupID (body, materiaID);
				}
			}
		} else {
			// it is not a Newton Collada set the default parameters
			_ASSERTE (0);
			context->SetUserData(body, model, "default");
			context->SetTransformCallback(body, "default");
			context->SetForceAndTorqueCallback(body, "default");
			context->SetDestructorCallback(body, "default");
			NewtonBodySetAutoSleep (body, 1);

			//NewtonBodySetLinearDamping(body, drag);
			//NewtonBodySetAngularDamping (body, &drag[0]);
			//NewtonBodySetContinuousCollisionMode(body, 1);
			//NewtonBodySetContinuousCollisionMode (body, 0);
			//NewtonBodySetJointRecursiveCollision(body, 1);
			//NewtonBodySetJointRecursiveCollision (body, 0);
			//NewtonBodySetMaterialGroupID (body, materiaID);
		}

		return body;
	}




	void LoadNodeData (dModel* model, dBone* bone, domNode* colNode, dLoaderContext* context, CollGeoCache& meshCache, 
					   ModifierVertexCache& modifierVertexMapCache, CollMaterialCache& materialCache, CollImageCache& imageCache)	
	{
		// get the name ID (note: many exporters do not obey Collada specifications and use the Id instead of the Name) 
		if (colNode->getSid()) {
			bone->SetNameID(colNode->getSid());
		} else if (colNode->getName()) {
			bone->SetNameID(colNode->getName());
		} else {
			bone->SetNameID(colNode->getId());
		}

		dMatrix matrix ( m_globalRotation.Inverse() * GetMatrix (colNode) * m_globalRotation);
		matrix.m_posit = matrix.m_posit.Scale (m_scale);
		bone->SetMatrix(matrix);

		if (colNode->getType() == NODETYPE_JOINT) {
			bone->SetType(dBone::m_bone);
		}


		// instance controller may make reference to nodes that may not being read yet, wee need to read them in a second pass.
		const domInstance_controller_Array& controllerArray = colNode->getInstance_controller_array();
		if (controllerArray.getCount()) {
			domInstance_controller* instance = controllerArray[0];
			daeURI uri (instance->getUrl());
			const daeElement* element = uri.getElement();
			domController* controller = (domController*) element;
			domSkin* skin = controller->getSkin();

			daeURI uriMesh (skin->getSource());
			element = uriMesh.getElement();
			domGeometry* collGeometry = (domGeometry*) element;

			CollGeoCache::dTreeNode* cacheNode = meshCache.Find(collGeometry);
			dMesh* mesh = NULL;
			if (!cacheNode) {
				// Load this Mesh for the firseTime time
				if (collGeometry->getMesh()) {
					mesh = LoadMesh (collGeometry->getMesh(), context, modifierVertexMapCache, materialCache, imageCache);
				} else if (collGeometry->getConvex_mesh()){
					mesh = LoadConvexMesh (collGeometry->getConvex_mesh(), context, modifierVertexMapCache, materialCache, imageCache);
				} else {
					_ASSERTE (0);
					//geometry = new dGeometry;
				}
				mesh->ApplyGlobalScale(m_scale);
				mesh->ApplyGlobalTransform (m_globalRotation);
				cacheNode = meshCache.Insert(mesh, collGeometry);
			}

			_ASSERTE (cacheNode);
			mesh = cacheNode->GetInfo();
			if (!mesh->m_hasBone) {
				// if this mesh is already use by a bone, just add a reference to the mesh 
				mesh->m_hasBone = 1;
				mesh->m_boneID = bone->m_boneID;
			}

			// add a reference to this mesh to this model.
			model->AddMesh(mesh);
			model->m_meshList.GetLast()->GetInfo().m_boneID = bone->m_boneID;
		}


		// read the Mesh instantiated by this node if there are not read yet
		const domInstance_geometry_Array& gemeortyArray = colNode->getInstance_geometry_array();
		if (gemeortyArray.getCount()) {
			// for know we will only allow one mesh per node, as the dModel format does no support array of dMesh per nodes yet
			// it will be in the future
			dMesh* mesh = NULL;
			domInstance_geometry* instance = gemeortyArray[0];

			// get the pointer to the Mesh instance from the uri
			const daeURI& uri = instance->getUrl();
			const daeElement* element = uri.getElement();
			if (element) {
				
				domGeometry* collGeometry = (domGeometry*) element;

				// if the mesh is not in the model, we must load this new mesh instance
				CollGeoCache::dTreeNode* cacheNode = meshCache.Find(collGeometry);
				if (!cacheNode) {
					// Load this Mesh for the firseTime time
					if (collGeometry->getMesh()) {
						mesh = LoadMesh (collGeometry->getMesh(), context, modifierVertexMapCache, materialCache, imageCache);
					} else if (collGeometry->getConvex_mesh()){
						mesh = LoadConvexMesh (collGeometry->getConvex_mesh(), context, modifierVertexMapCache, materialCache, imageCache);
					} else {
						_ASSERTE (0);
		//				geometry = new dGeometry;
					}
					mesh->ApplyGlobalScale(m_scale);
					mesh->ApplyGlobalTransform (m_globalRotation);
					cacheNode = meshCache.Insert(mesh, collGeometry);
				}
				_ASSERTE (cacheNode);
				mesh = cacheNode->GetInfo();
				if (!mesh->m_hasBone) {
					// if this mesh is already use by a bone, just add a reference to the mesh 
					mesh->m_hasBone = 1;
					mesh->m_boneID = bone->m_boneID;
				}
				// add a reference to this mesh to this model.
				model->AddMesh(mesh);
			}

//			for (dList<dMeshInstance>::dListNode* node = model->m_meshList.GetFirst(); node; node = node->GetNext()) { 
//				if (node->GetInfo().m_mesh == mesh) {
//					node->GetInfo().m_boneID = bone->m_boneID;
//				}
//			}

			model->m_meshList.GetLast()->GetInfo().m_boneID = bone->m_boneID;
		}
	}


	domNode* LoadScene (daeDocument* document, dModel* model, domVisual_scene* visualScene, dLoaderContext* context,
						CollGeoCache& meshCache, ModifierVertexCache& modifierVertexMapCache, CollMaterialCache& materialCache, CollImageCache& imageCache)
	{
		int stack;
		int boneId;
		dBone* rootBone;
		domNode* colladaRoot;
		domNode* nodePool[1024];
		dBone* parentBones[1024];
		dTree<dBone*, domNode*> controllersIntances; 

		// create a dummy root bode;
		colladaRoot = NULL;
		rootBone = new dBone (NULL);

		stack = 0;
		const domNode_Array& nodeArray = visualScene->getNode_array();
		for (int i = 0; i < int (nodeArray.getCount()); i ++) {
			parentBones[stack] = rootBone;
			nodePool[stack] = nodeArray[i];
			stack ++;
		}

		boneId = 0;
		while (stack) {
			dBone* node;
			dBone* parent;
			domNode* colNode;

			stack --;
			colNode = nodePool[stack];
			parent = parentBones[stack]; 

			node = new dBone(parent);

			if (!colladaRoot) {
				colladaRoot = colNode;
			}

			node->m_boneID = boneId;
			boneId ++;

			if (colNode->getInstance_controller_array().getCount()) {
				controllersIntances.Insert(node, colNode);
			}
			LoadNodeData (model, node, colNode, context, meshCache, modifierVertexMapCache, materialCache, imageCache);

			// check if this collada file has Node instances, a feature support by some modelers but not all  
			domInstance_node_Array& instansceNodeArray = colNode->getInstance_node_array();
			for (int i = 0; i < int (instansceNodeArray.getCount()); i ++) {
				daeElement* element;
				domNode* childNode;

				// if there are node instances we will just read them as real nodes copies.
				domInstance_node* instansceNode = instansceNodeArray[0];
				daeURI uri (instansceNode->getUrl());
				element = uri.getElement ();
				childNode = (domNode *)element;

				parentBones[stack] = node;
				nodePool[stack] = childNode;
				stack ++;
			}

			const domNode_Array &nodeArray = colNode->getNode_array();
			for (int i = 0; i < int (nodeArray.getCount()); i ++) {
				parentBones[stack] = node;
				nodePool[stack] = nodeArray[i];
				stack ++;
			}
		}


		// detach all children nodes from the root bone and add them to the Model as root bones.
//		dList<dBone*>& boneList = model->m_skeleton.Append()->GetInfo();
		while (rootBone->GetChild()) {
			dBone *bone;
			bone = rootBone->GetChild();
			bone->Detach();
//			boneList.m_data.Append (bone);
			model->AddSkeleton (bone);
			bone->Release();
		}

		// now load all meshes controllers
		while (controllersIntances.GetCount()) {
			dBone* bone = controllersIntances.GetRoot()->GetInfo();
			domNode* colNode = controllersIntances.GetRoot()->GetKey();
			controllersIntances.Remove(controllersIntances.GetRoot());

			domInstance_controller* instanceController = colNode->getInstance_controller_array()[0];
			daeURI controllerUri (instanceController->getUrl());
			const daeElement* element = controllerUri.getElement();
			domController* controller = (domController*) element;
			_ASSERTE (controller);
			domSkin* skin = controller->getSkin();

			const daeURI& uriMesh = skin->getSource();
			element = uriMesh.getElement();
			domGeometry* collGeometry = (domGeometry*) element;

			_ASSERTE (meshCache.Find(collGeometry));
			dMesh* mesh = meshCache.Find(collGeometry)->GetInfo();

			dSkinModifier* skinModifier = LoadSkinController (document, skin, mesh, model, modifierVertexMapCache);
			for (dList<dMeshInstance>::dListNode* node = model->m_meshList.GetFirst(); node; node = node->GetNext()) {
				dMeshInstance& instance = node->GetInfo();
				if ((instance.m_mesh == mesh) && instance.m_boneID == bone->m_boneID) {
					instance.SetModifier (skinModifier);
					break;
				}
			}
		}

		rootBone->Release();

		return colladaRoot;
	}


	dMatrix m_globalRotation;
	dFloat m_scale;
	int m_nodeIndex;
	DAE* m_collada; 
};




void ExportColladaScene (const char* fileName, const NewtonWorld* world)
{
	_ASSERTE (0);
/*
	DAE* dae;
	daeInt error;
	daeURI* uriName;
	daeDatabase* database;
	daeDocument* document;

	// create a new collada object
	dae = CreateDAE ();
	_ASSERTE (dae);

	dColladaFileParcel parcel(dae, );
	parcel.SceneToCollada (world, context);

	database = dae->getDatabase();
	if (database) {
		document = database->getDocument(daeUInt (0));
		if (document) {
			uriName = document->getDocumentURI();
			error = dae->saveAs(fileName, uriName->getURI());
			_ASSERTE (error == DAE_OK);
		}
	}

	delete dae;
*/
}


void ExportColladaScene (
	const char* name, 
	const NewtonWorld* world, 
	dColladaSceneExportContext* context, 
	const dMatrix& globalRotation, 
	dFloat scale)
{

}


dSceneModelList::dSceneModelList()
	:dList<dModel*>()
{
}

dSceneModelList::~dSceneModelList()
{
	while (GetFirst()) {
		RemoveModel (GetFirst()->GetInfo());
	}
}

void dSceneModelList::AddModel (dModel* model)
{
	Append (model);
	model->AddRef();
}

void dSceneModelList::RemoveModel (dModel* model)
{
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		if (node->GetInfo() == model) {
			node->GetInfo()->Release();
			Remove (node);
			break;
		}
	}
}


void dSceneModelList::ExportVisualScene (const char* fileName, const dMatrix& globalRotation, dFloat scale)
{
	DAE* dae;
	daeInt error;
	daeURI* uriName;
	daeDatabase* database;
	daeDocument* document;

	// create a new collada object
	dae = new DAE;
	_ASSERTE (dae);

	dColladaFileParcel parcel(dae, globalRotation, scale);
	parcel.SceneToCollada (*this);

	database = dae->getDatabase();
	if (database) {
		document = database->getDocument(daeUInt (0));
		if (document) {
			uriName = document->getDocumentURI();
			error = dae->saveAs(fileName, uriName->getURI());
			_ASSERTE (error == DAE_OK);
		}
	}
	delete dae;
}

void dSceneModelList::ExportPhysicsScene (const char* fileName, NewtonWorld* world, const dMatrix& globalRotation, dFloat scale)
{
	DAE* dae;
	daeInt error;
	daeURI* uriName;
	daeDatabase* database;
	daeDocument* document;

	// create a new collada object
	dae = new DAE;
	_ASSERTE (dae);

	dColladaFileParcel parcel(dae, globalRotation, scale);
	parcel.SceneToCollada (world);

	database = dae->getDatabase();
	if (database) {
		document = database->getDocument(daeUInt (0));
		if (document) {
			uriName = document->getDocumentURI();
			error = dae->saveAs(fileName, uriName->getURI());
			_ASSERTE (error == DAE_OK);
		}
	}
	delete dae;
}


void dSceneModelList::ImportVisualScene  (const char* fileName, dLoaderContext& context, const dMatrix& globalRotation, dFloat scale)
{
	DAE* dae;
	daeInt error;

	dae = new DAE;
	_ASSERTE (dae);

	error = dae->load(fileName);
	_ASSERTE (error == DAE_OK);

	if (error == DAE_OK) {
		dColladaFileParcel parcel(dae, globalRotation, scale);
		parcel.ColladatToScene (*this, &context);
	}
	delete dae;
}



void dSceneModelList::ImportPhysicsScene  (const char* fileName, NewtonWorld* world, dLoaderContext& context, const dMatrix& globalRotation, dFloat scale)
{
	DAE* dae;
	daeInt error;

	dae = new DAE;
	_ASSERTE (dae);

	error = dae->load(fileName);
	_ASSERTE (error == DAE_OK);

	if (error == DAE_OK) {
		dColladaFileParcel parcel(dae, globalRotation, scale);
		parcel.ColladatToScene (*this, world, &context);
	}
	delete dae;
}

