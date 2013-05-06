/////////////////////////////////////////////////////////////////////////////
// Name:        ColladaExport.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 07:45:05
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////

#include "StdAfx.h"
#include "ColladaExport.h"
#include <time.h>

#define D_CAMERA_NAME					"camera"
#define D_TEXTURE_PREFIX_NAME			"texture_"
#define D_MATERIAL_PREFIX_NAME			"material_"
#define D_MATERIAL_PREFIX_NAME_EFFECT	"effect_"
#define D_SKIN_POSFIX_NAME				"_skin"

ColladaExport::ColladaExport()
	:dExportPlugin()
{
}


ColladaExport::~ColladaExport()
{
}


ColladaExport* ColladaExport::GetPlugin()
{
	static ColladaExport gImporter;
	return &gImporter;
}


//void ColladaExport::Export (const char* fileName, const dScene* world)
void ColladaExport::Export (const char* const fileName, dPluginInterface* const interface)
{
	_ASSERTE (0);
/*
	// create a new collada object
	DAE* dae = new DAE;
	_ASSERTE (dae);

	m_globalRotation = dYawMatrix (90.0f * 3.141592f / 180.0f);

	UniqueNameFilter uniqueNames;
	SceneToCollada (dae, world, uniqueNames);

	daeDatabase* database = dae->getDatabase();
	if (database) {
		daeDocument* document = database->getDocument(daeUInt (0));
		if (document) {
			daeURI* uriName = document->getDocumentURI();
			daeInt error;
			error = dae->saveAs(fileName, uriName->getURI());
			_ASSERTE (error == DAE_OK);
		}
	}
	delete dae;
*/
}


void ColladaExport::FixName (char* const name) const
{
	for (int i = 0; name[i]; i ++) {
		if (isspace(name[i])) {
			name[i] = '_';
		}
	}
}


void ColladaExport::AddAsset (daeDocument *document)
{
	domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
	domAsset *asset = daeSafeCast<domAsset>(domRoot->createAndPlace(COLLADA_ELEMENT_ASSET));

	domAsset::domContributor* contributor = daeSafeCast<domAsset::domContributor>(asset->createAndPlace(COLLADA_ELEMENT_CONTRIBUTOR));
	domAsset::domContributor::domAuthor* autor = daeSafeCast<domAsset::domContributor::domAuthor>(contributor->createAndPlace(COLLADA_ELEMENT_AUTHOR));
	autor->setValue("Newton Game Dynamics 2.00 Collada Export");

	domAsset::domCreated *created = daeSafeCast<domAsset::domCreated>(asset->createAndPlace(COLLADA_ELEMENT_CREATED));

	time_t rawtime;
	tm * ptm;
	time ( &rawtime );
	ptm = gmtime ( &rawtime );

	char date[256];
	sprintf (date, "%d-%02d-%02dT%02d:%02d:%02dZ", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

	//created->setValue("2010-01-04T15:56:43Z");
	created->setValue(date);

	domAsset::domModified* modified = daeSafeCast<domAsset::domModified>(asset->createAndPlace(COLLADA_ELEMENT_MODIFIED));
	modified->setValue(date);

	domAsset::domUnit* unit = daeSafeCast<domAsset::domUnit>(asset->createAndPlace(COLLADA_ELEMENT_UNIT));
	unit->setName("meter");
	unit->setMeter (1.0);

	domAsset::domUp_axis* axis = daeSafeCast<domAsset::domUp_axis>(asset->createAndPlace(COLLADA_ELEMENT_UP_AXIS));
	axis->setValue(UPAXISTYPE_Y_UP);
//	axis->setValue(UPAXISTYPE_Z_UP);
}



void ColladaExport::AddCameraLibrary (daeDocument *document, const dScene* scene)
{
/*
	dScene::Iterator iter (*scene);
	for (iter.Begin(); iter; iter ++) {
		dScene::dTreeNode* node = iter.GetNode();
		dNodeInfo* info = scene->GetInfoFromNode(node);
		if (info->GetTypeId() == dCameraNodeInfo::GetRttiType()) {
			dCameraNodeInfo* camera = (dCameraNodeInfo*) scene->GetInfoFromNode(node);
			if (camera->GetViewMode() == dCameraNodeInfo::m_perspective) {

				domCOLLADA* domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());

				domLibrary_cameras* library = daeSafeCast<domLibrary_cameras>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_CAMERAS));

				domCamera* collCamera = daeSafeCast<domCamera>(library->createAndPlace(COLLADA_ELEMENT_CAMERA));

				collCamera->setId(D_CAMERA_NAME);

				domCamera::domOptics* optics = daeSafeCast<domCamera::domOptics>(collCamera->createAndPlace(COLLADA_ELEMENT_OPTICS));
				domCamera::domOptics::domTechnique_common* technique = daeSafeCast<domCamera::domOptics::domTechnique_common>(optics->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
				domCamera::domOptics::domTechnique_common::domPerspective* perpective = daeSafeCast<domCamera::domOptics::domTechnique_common::domPerspective>(technique->createAndPlace(COLLADA_ELEMENT_PERSPECTIVE));

				domTargetableFloat* fov = daeSafeCast<domTargetableFloat>(perpective->createAndPlace(COLLADA_ELEMENT_YFOV));
				domTargetableFloat* aspect = daeSafeCast<domTargetableFloat>(perpective->createAndPlace(COLLADA_ELEMENT_ASPECT_RATIO));
				domTargetableFloat* nearPlane = daeSafeCast<domTargetableFloat>(perpective->createAndPlace(COLLADA_ELEMENT_ZNEAR));
				domTargetableFloat* farPlane = daeSafeCast<domTargetableFloat>(perpective->createAndPlace(COLLADA_ELEMENT_ZFAR));


				fov->setValue(60.0f);
				aspect->setValue(1.0f);
				nearPlane->setValue(0.1f);
				farPlane->setValue(500.0f);
				break;
			}
		}
	}
*/
}


// check image duplicates in the library
domImage* ColladaExport::FindImage (domLibrary_images* library, const char*name) const
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

domMaterial* ColladaExport::FindMaterial (domLibrary_materials* library, const char* name) const
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



void ColladaExport::AddImageLibrary (daeDocument *document, const dScene* scene)
{
	_ASSERTE (0);
/*
	domLibrary_images* library = NULL;
	domCOLLADA* domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
	if (domRoot->getLibrary_images_array().getCount()) {
		library = domRoot->getLibrary_images_array()[0];
	} else {
		library = daeSafeCast<domLibrary_images>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_IMAGES));
	}


	dScene::Iterator iter (*scene);
	for (iter.Begin(); iter; iter ++) {
		dScene::dTreeNode* node = iter.GetNode();
		dNodeInfo* info = scene->GetInfoFromNode(node);
		if (info->GetTypeId() == dTextureNodeInfo::GetRttiType()) {
			dTextureNodeInfo* texture = (dTextureNodeInfo*) scene->GetInfoFromNode(node);
			char name[256];
			int crcCode = dCRC (texture->GetPathName());
			sprintf (name, "%s%x", D_TEXTURE_PREFIX_NAME, crcCode);
			FixName (name);
			if (!FindImage (library, name)) {
				domImage* image = daeSafeCast<domImage>(library->createAndPlace(COLLADA_ELEMENT_IMAGE));
				image->setId (name);

				domImage::domInit_from* imagefrom = daeSafeCast<domImage::domInit_from>(image->createAndPlace(COLLADA_ELEMENT_INIT_FROM));
				imagefrom->setValue (texture->GetPathName());
			}
		}
	}
*/
}


void ColladaExport::AddDefaultMaterial (domLibrary_effects* effectLibrary, domLibrary_materials* materialLibrary)
{
	domEffect *effect;
	domMaterial* material;
	domInstance_effect *instanceEffect;
	char materialName[128];

	sprintf (materialName, "defaultMaterial");

	material = daeSafeCast<domMaterial>(materialLibrary->createAndPlace(COLLADA_ELEMENT_MATERIAL));
	material->setId (materialName);

	// create an effect and add instance to the material
	effect = daeSafeCast<domEffect>(effectLibrary->createAndPlace(COLLADA_ELEMENT_EFFECT));
	sprintf (materialName, "defaultEffect");
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





void ColladaExport::AddMaterialLibrary (daeDocument *document, const dScene* scene)
{
	_ASSERTE (0);
/*
	domCOLLADA* domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());

	// create an material and effect library
	domLibrary_effects* effectLibrary = NULL;
	if (domRoot->getLibrary_effects_array().getCount()) {
		effectLibrary = domRoot->getLibrary_effects_array()[0];
	} else {
		effectLibrary = daeSafeCast<domLibrary_effects>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_EFFECTS));
	}

	domLibrary_materials* materialLibrary = NULL;
	if (domRoot->getLibrary_materials_array().getCount()) {
		materialLibrary = domRoot->getLibrary_materials_array()[0];
	} else {
		materialLibrary = daeSafeCast<domLibrary_materials>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_MATERIALS));
		AddDefaultMaterial (effectLibrary, materialLibrary);
	}


	dScene::Iterator iter (*scene);
	for (iter.Begin(); iter; iter ++) {
		dScene::dTreeNode* node = iter.GetNode();
		dNodeInfo* info = scene->GetInfoFromNode(node);
		if (info->GetTypeId() == dMaterialNodeInfo::GetRttiType()) {
			char materialName[256];

			dMaterialNodeInfo* material = (dMaterialNodeInfo*) scene->GetInfoFromNode(node);
			sprintf (materialName, "%s_%d", material->GetName(), material->GetId());
			FixName (materialName);

			if (!FindMaterial(materialLibrary, materialName)) {

				domMaterial* colladaMaterial = daeSafeCast<domMaterial>(materialLibrary->createAndPlace(COLLADA_ELEMENT_MATERIAL));
				colladaMaterial->setId (materialName);

				// create an effect and add instance to the material
				domEffect* effect = daeSafeCast<domEffect>(effectLibrary->createAndPlace(COLLADA_ELEMENT_EFFECT));
				//sprintf (materialName, "%s%x", D_MATERIAL_PREFIX_NAME_EFFECT, crcCode);
				sprintf (materialName, "%s%s_%d", D_MATERIAL_PREFIX_NAME_EFFECT, material->GetName(), material->GetId());
				FixName (materialName);
				effect->setId (materialName);

				domInstance_effect* instanceEffect = daeSafeCast<domInstance_effect>(colladaMaterial->createAndPlace(COLLADA_ELEMENT_INSTANCE_EFFECT ));
				daeURI uri (*effect);
				uri.set("", "", "", "", effect->getId());
				instanceEffect->setUrl (uri);

				// add properties to the effect
				// need to copy the material properties into the effect, 
				//const dgMaxMaterial& myMaterial = *iter;
				domProfile_COMMON *profile = daeSafeCast<domProfile_COMMON>(effect->createAndPlace(COLLADA_ELEMENT_PROFILE_COMMON));

				domProfile_COMMON::domTechnique *technique = daeSafeCast<domProfile_COMMON::domTechnique>(profile->createAndPlace(COLLADA_ELEMENT_TECHNIQUE));
				technique->setSid("common");
				domProfile_COMMON::domTechnique::domLambert *lambert = daeSafeCast<domProfile_COMMON::domTechnique::domLambert>(technique->createAndPlace(COLLADA_ELEMENT_LAMBERT));
				domCommon_color_or_texture_type *diffuse = daeSafeCast<domCommon_color_or_texture_type>(lambert->createAndPlace(COLLADA_ELEMENT_DIFFUSE));

				dScene::dTreeNode* textureNode = NULL; 
				for (void* ptr = scene->GetFirstChild(node); ptr; ptr = scene->GetNextChild(node, ptr)) {
					dScene::dTreeNode* texNode = scene->GetNodeFromLink(ptr);
					dTextureNodeInfo* texture = (dTextureNodeInfo*) scene->GetInfoFromNode(texNode);
					if (texture->GetTypeId() == dTextureNodeInfo::GetRttiType()) {
						textureNode = texNode;
						break;
					}
				}

				if (textureNode) {
					char nameID [128];
					char nameIDimage [128];

					dTextureNodeInfo* texture = (dTextureNodeInfo*) scene->GetInfoFromNode(textureNode);
					int crcCode = dCRC (texture->GetPathName());
					//sprintf (name, "%s%x", D_TEXTURE_PREFIX_NAME, crcCode);

					sprintf (nameID, "%s%x", D_TEXTURE_PREFIX_NAME, crcCode);							
					sprintf (nameIDimage, "%s_image", nameID);
					FixName (nameIDimage);

					domCommon_newparam_type* np = daeSafeCast<domCommon_newparam_type>(profile->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
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

					domCommon_color_or_texture_type::domTexture *colladaTexture = daeSafeCast<domCommon_color_or_texture_type::domTexture>(diffuse->createAndPlace(COLLADA_ELEMENT_TEXTURE));
					colladaTexture->setTexture (nameID);
					colladaTexture->setTexcoord ("CHANNEL1");

				} else {
					domCommon_color_or_texture_type::domColor *color = daeSafeCast<domCommon_color_or_texture_type::domColor>(diffuse->createAndPlace(COLLADA_ELEMENT_COLOR));
					domFx_color_common& value = color->getValue();
					value.setCount(4);
					value[0] = 0.7f;
					value[1] = 0.7f;
					value[2] = 0.7f;
					value[3] = 1.0f;
//					domCommon_color_or_texture_typeRef elemEmission;
//					domCommon_color_or_texture_typeRef elemAmbient;
//					domCommon_color_or_texture_typeRef elemDiffuse;
//					domCommon_color_or_texture_typeRef elemReflective;
//					domCommon_float_or_param_typeRef elemReflectivity;
//					domCommon_transparent_typeRef elemTransparent;
//					domCommon_float_or_param_typeRef elemTransparency;
//					domCommon_float_or_param_typeRef elemIndex_of_refraction;
				}
			}
		}
	}
*/
}


void ColladaExport::AddGeometryLibrary (daeDocument *document, const dScene* scene, UniqueNameFilter& uniqueNames)
{
	domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());

	domLibrary_geometries *library = NULL;
	if (domRoot->getLibrary_geometries_array().getCount()) {
		library = domRoot->getLibrary_geometries_array()[0];
	} else {
		library = daeSafeCast<domLibrary_geometries>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_GEOMETRIES));
	}
	_ASSERTE (library);

	dScene::Iterator iter (*scene);
	for (iter.Begin(); iter; iter ++) {
		dScene::dTreeNode* node = iter.GetNode();
		dNodeInfo* info = scene->GetInfoFromNode(node);
		if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
			if (info->GetTypeId() == dMeshNodeInfo::GetRttiType()) {
				AddMesh (document, scene, node, uniqueNames);
			} else {
				_ASSERTE (0);
			}
		}
	}
}


void ColladaExport::AddMesh (daeDocument *document, const dScene* scene, dScene::dTreeNode* meshNode, UniqueNameFilter& uniqueNames)
{
_ASSERTE (0);
/*
	domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
	domLibrary_geometries* library = domRoot->getLibrary_geometries_array()[0];
	_ASSERTE (library);

	domGeometry *collGeometry = daeSafeCast<domGeometry>(library->createAndPlace (COLLADA_ELEMENT_GEOMETRY));

	dMeshNodeInfo* geometry = (dMeshNodeInfo*) scene->GetInfoFromNode(meshNode);
	NewtonMesh* mesh = geometry->GetMesh ();

//	char meshName[256];
//	sprintf (meshName, "%s", geometry->GetName());
	const char* meshName = uniqueNames.CreateUniqueName(scene, meshNode);
	collGeometry->setId(meshName);

	domMesh *colladaMesh = daeSafeCast<domMesh>(collGeometry->createAndPlace(COLLADA_ELEMENT_MESH ));

	domVertices *verts = NULL;
	{
		// add the vertices
		char text[256];

		int vertexCount = NewtonMeshGetVertexCount(mesh);
		int strideInByte = NewtonMeshGetVertexStrideInByte(mesh);
		dFloat* vertex = NewtonMeshGetVertexArray (mesh); 

		float* floatPool = new float[vertexCount * 3];
		dMatrix matrix (geometry->GetPivotMatrix() * m_globalRotation);
		matrix.TransformTriplex(floatPool, sizeof (float) * 3, vertex, strideInByte, vertexCount);

		domSource* posSource = daeSafeCast<domSource>(colladaMesh->createAndPlace(COLLADA_ELEMENT_SOURCE));
		sprintf (text, "%s_position", meshName);
		posSource->setId (text);

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
		sprintf (text, "%sVertices", meshName);
		verts->setId (text);
		domInputLocal *inputLocal = daeSafeCast<domInputLocal>(verts->createAndPlace(COLLADA_ELEMENT_INPUT));
		inputLocal->setSemantic (COMMON_PROFILE_INPUT_POSITION);
		daeURI uri1(*posSource);
		uri1.set("", "", "", "", posSource->getId());
		inputLocal->setSource (uri1);

		delete[] floatPool;
	}

	int pointCount = NewtonMeshGetPointCount(mesh);
	domSource *normalSrc = NULL;
	int* normalArrayIndexList = new int[pointCount];
	{
		char text[256];
		float* floatPool = new float[pointCount * 3];

		normalSrc = daeSafeCast<domSource>(colladaMesh->createAndPlace(COLLADA_ELEMENT_SOURCE));
		sprintf (text, "%s_normal", meshName);
		normalSrc->setId (text);

		//int vertexStride = NewtonMeshGetVertexStrideInByte(mesh) / sizeof (dFloat);
		int pointStride = NewtonMeshGetPointStrideInByte(mesh) / sizeof (dFloat);
		dFloat* points = NewtonMeshGetNormalArray(mesh); 

		for (int i = 0; i < pointCount; i ++) {
			floatPool[i * 3 + 0] = points[i * pointStride + 0];
			floatPool[i * 3 + 1] = points[i * pointStride + 1];
			floatPool[i * 3 + 2] = points[i * pointStride + 2];
		}

		//vertexCount = dModel::dPackVertexArray (floatPool, 3, 3 * sizeof (dFloat), geometry->m_vertexCount, normalArrayIndexList);
		int normalCount = dPackVertexArray (floatPool, 3, 3 * sizeof (dFloat), pointCount, normalArrayIndexList);

		dMatrix rot (geometry->GetPivotMatrix() * m_globalRotation);
		rot.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
		rot.TransformTriplex(floatPool, sizeof (float) * 3, floatPool, sizeof (float) * 3, normalCount);


		domFloat_array *fa = daeSafeCast<domFloat_array>(normalSrc->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
		strcat (text, "Array");
		fa->setId (text);
		fa->setCount (normalCount * 3);

		domListOfFloats &posSrcArray = fa->getValue();
		for (int i = 0; i < normalCount; i ++) {
			posSrcArray.append3( floatPool[i * 3 + 0], floatPool[i * 3 + 1], floatPool[i * 3 + 2]);
		}

		//create the accessors
		domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(normalSrc->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
		domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
		acc->setCount (normalCount);
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


	domSource *uvSource = NULL;
	int* uv0ArrayIndexList = new int[pointCount];
	{
		char text[256];
		float* floatPool = new float[pointCount * 3];

		uvSource = daeSafeCast<domSource>(colladaMesh->createAndPlace(COLLADA_ELEMENT_SOURCE));
		sprintf (text, "%s_uv", meshName);
		uvSource->setId (text);

		int pointStride = NewtonMeshGetPointStrideInByte(mesh) / sizeof (dFloat);
		dFloat* points = NewtonMeshGetUV0Array(mesh); 
		for (int i = 0; i < pointCount; i ++) {
			floatPool[i * 3 + 0] = points[i * pointStride + 0];
			floatPool[i * 3 + 1] = points[i * pointStride + 1];
			floatPool[i * 3 + 2] = 0.0f;
		}
		int uvCount = dPackVertexArray (floatPool, 3, 3 * sizeof (dFloat), pointCount, uv0ArrayIndexList);

		domFloat_array *fa = daeSafeCast<domFloat_array>(uvSource->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
		strcat (text, "Array");
		fa->setId (text);
		fa->setCount(uvCount * 2);
		domListOfFloats &posSrcArray = fa->getValue();
		for (int i = 0; i < uvCount; i ++) {
			posSrcArray.append2( floatPool[i * 3 + 0], floatPool[i * 3 + 1]);
		}

		domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(uvSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
		domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
		acc->setCount (uvCount);
		acc->setStride( 2 );
		daeURI uri(*fa);
		uri.set("", "", "", "", fa->getId());
		acc->setSource( uri );

		domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
		param->setName( "s" );
		param->setType( "float" );
		param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
		param->setName( "t" );
		param->setType( "float" );

		delete[] floatPool;
	}


	// calculate face count and index count;
	int faceCount = 0;
	int indexCount = 0;
	for (void* face = NewtonMeshGetFirstFace(mesh); face; face = NewtonMeshGetNextFace(mesh, face)) {
		if (!NewtonMeshIsFaceOpen (mesh, face)) {
			faceCount ++;
			indexCount += NewtonMeshGetFaceIndexCount (mesh, face);
		}
	}

	int* faceIndexCount = new int [faceCount];
	int* faceVertexArray = new int [indexCount];
	int* faceAttributeArray = new int [indexCount];

	//domLibrary_materials* materialLibrary = domRoot->getLibrary_materials_array()[0];

	for (void* ptr = scene->GetFirstChild(meshNode); ptr; ptr = scene->GetNextChild(meshNode, ptr)) {
		dScene::dTreeNode* materialNode = scene->GetNodeFromLink(ptr);
		dNodeInfo* info = scene->GetInfoFromNode(materialNode);
		if (info->GetTypeId() == dMaterialNodeInfo::GetRttiType()) {
			char materialName[256];
			dMaterialNodeInfo* material = (dMaterialNodeInfo*) info;
			sprintf (materialName, "%s_%d", material->GetName(), material->GetId());
			FixName (materialName);

			//domMaterial* colladaMaterial = FindMaterial(materialLibrary, materialName);
			//_ASSERTE (FindMaterial(materialLibrary, materialName));

			int indexCount = 0;
			int polygonCount = 0;
			for (void* face = NewtonMeshGetFirstFace(mesh); face; face = NewtonMeshGetNextFace(mesh, face)) {
				if (!NewtonMeshIsFaceOpen (mesh, face)) {
					if (NewtonMeshGetFaceMaterial (mesh, face) == material->GetId()) {
						faceIndexCount[polygonCount] = NewtonMeshGetFaceIndexCount (mesh, face);

						NewtonMeshGetFaceIndices (mesh, face, &faceVertexArray[indexCount]);
						NewtonMeshGetFacePointIndices (mesh, face, &faceAttributeArray[indexCount]);

						indexCount += NewtonMeshGetFaceIndexCount (mesh, face);
						polygonCount ++;
					}
				}
			}

			if (polygonCount) {
//				domPolylist* polyList = daeSafeCast<domPolylist>(colladaMesh->createAndPlace(COLLADA_ELEMENT_POLYLIST));
				domPolygons* polyList = daeSafeCast<domPolygons>(colladaMesh->createAndPlace(COLLADA_ELEMENT_POLYGONS));

				polyList->setCount (polygonCount);
				polyList->setMaterial (materialName);

				domInputLocalOffset* input = daeSafeCast<domInputLocalOffset>(polyList->createAndPlace (COLLADA_ELEMENT_INPUT));
				input->setSemantic (COMMON_PROFILE_INPUT_VERTEX);
				input->setOffset( 0 );
				daeURI uri (*verts);
				uri.set("", "", "", "", verts->getId());
				input->setSource (uri);

				input = daeSafeCast<domInputLocalOffset>(polyList->createAndPlace(COLLADA_ELEMENT_INPUT));
				input->setSemantic (COMMON_PROFILE_INPUT_NORMAL);
				input->setOffset (1);
				daeURI uri1 (*normalSrc);
				uri1.set("", "", "", "", normalSrc->getId());
				input->setSource (uri1);

				input = daeSafeCast<domInputLocalOffset>(polyList->createAndPlace(COLLADA_ELEMENT_INPUT));
				input->setSemantic (COMMON_PROFILE_INPUT_TEXCOORD);
				input->setOffset (2);
				//input->setSet (1);
				input->setSet (0);
				daeURI uri2 (*uvSource);
				uri2.set("", "", "", "", uvSource->getId());
				input->setSource (uri2);


				int indexAcc = 0;
				for (int j = 0; j < polygonCount; j ++) {

					domP *p = daeSafeCast<domP>(polyList->createAndPlace(COLLADA_ELEMENT_P));
					domListOfUInts &indices = p->getValue();
					int count = faceIndexCount[j];
					for (int i = 0; i < count; i ++) {
						int vIndex = faceVertexArray[indexAcc + i];
						int attIndex = faceAttributeArray[indexAcc + i];
						indices.append3 (vIndex, normalArrayIndexList[attIndex], uv0ArrayIndexList[attIndex]);
					}
					indexAcc += count;
				}
			}
		}
	}


	delete[] faceAttributeArray;
	delete[] faceVertexArray;
	delete[] faceIndexCount;
	delete[] uv0ArrayIndexList;
	delete[] normalArrayIndexList;
*/
}


void ColladaExport::AddControllerLibrary (daeDocument *document, const dScene* scene, UniqueNameFilter& uniqueNames)
{
	domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());

	dScene::Iterator iter (*scene);
	for (iter.Begin(); iter; iter ++) {
		dScene::dTreeNode* meshNode = iter.GetNode();
		dNodeInfo* info = scene->GetInfoFromNode(meshNode);
		if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
			dScene::dTreeNode* skinModifierNode = NULL;	
			for (void* ptr = scene->GetFirstChild(meshNode); ptr; ptr = scene->GetNextChild(meshNode, ptr)) {
				dScene::dTreeNode* node = scene->GetNodeFromLink(ptr);
				dNodeInfo* info = scene->GetInfoFromNode(node);
				if (info->GetTypeId() == dGeometryNodeSkinModifierInfo::GetRttiType()) {
					skinModifierNode = node;
					break;
				}
			}

			if (skinModifierNode) {
				if (!domRoot->getLibrary_controllers_array().getCount()) {
					daeSafeCast<domLibrary_geometries>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_CONTROLLERS));
				}
				domLibrary_controllers *controllerLibrary = domRoot->getLibrary_controllers_array()[0];
				_ASSERTE (controllerLibrary);

				domLibrary_geometries *geometryLibrary = domRoot->getLibrary_geometries_array()[0];
				_ASSERTE (geometryLibrary);

				domController *controller = daeSafeCast<domController>(controllerLibrary->createAndPlace(COLLADA_ELEMENT_CONTROLLER ));

				dGeometryNodeSkinModifierInfo* skinModifier = (dGeometryNodeSkinModifierInfo*) scene->GetInfoFromNode(skinModifierNode);

//				const dMesh* mesh = meshInstance->m_mesh;
//				dSkinModifier* skinData = (dSkinModifier*) meshInstance->GetModifier();
//				const dBone** boneList = skinData->m_skinnedBones;
//				dMatrix* bindingMatrices = skinData->m_bindingMatrices;

				char exName[256];
//				char meshName[256];
//				mesh->GetName(meshName);
				const char* meshName = uniqueNames.FindUniqueName(scene, meshNode);
				sprintf (exName, "%s%s", meshName, D_SKIN_POSFIX_NAME);
				controller->setId(exName);

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
				dMatrix matrix (skinModifier->m_shapeBindMatrix);
				matrix = m_globalRotation.Inverse() * matrix * m_globalRotation;
//				matrix.m_posit = matrix.m_posit.Scale (m_scale);
				for (int j = 0; j < 4; j ++) {
					for (int i = 0; i < 4; i ++) {
						value[j * 4 + i] = matrix[i][j];
					}
				}

				domSource *boneSource = NULL;
				domSource *bindSource = NULL;
				domSource *weightsSource = NULL;
				int* weightsIndexMap = new int[skinModifier->m_vertexCount * 4];
				{
					char text[256];
					bindSource = daeSafeCast<domSource>(skin->createAndPlace(COLLADA_ELEMENT_SOURCE));
					sprintf (text, "%s_bindpose", meshName);
					bindSource->setId (text);

					const dMatrix* const bindingMatrices = skinModifier->m_boneBindingMatrix;

					domFloat_array *boneMatrix = daeSafeCast<domFloat_array>(bindSource->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
					strcat (text, "_Array");
					boneMatrix->setId (text);
					boneMatrix->setCount (skinModifier->m_boneCount * 16);
					domListOfFloats &boneSrcArray = boneMatrix->getValue();
					for (int i = 0; i < skinModifier->m_boneCount; i ++) {
						dMatrix matrix (bindingMatrices[i]);
						matrix = m_globalRotation.Inverse() * matrix * m_globalRotation;
						//matrix.m_posit = matrix.m_posit.Scale (m_scale);
						for (int j = 0; j < 4; j ++) {
							for (int i = 0; i < 4; i ++) {
								boneSrcArray.append (matrix[i][j]);
							}
						}
					}

					//create the accessors
					domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(bindSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
					domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
					acc->setCount (skinModifier->m_boneCount);
					acc->setStride (16);
					daeURI uri (*boneMatrix);
					uri.set("", "", "", "", boneMatrix->getId());
					acc->setSource (uri);
					domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
					param->setName ("bindMatrix");
					param->setType ("float4x4");
				}


				{
					char text[256];
					boneSource = daeSafeCast<domSource>(skin->createAndPlace(COLLADA_ELEMENT_SOURCE));
					sprintf (text, "%s_bones", exName);
					boneSource->setId (text);

					domName_array *boneNames = daeSafeCast<domName_array>(boneSource->createAndPlace(COLLADA_ELEMENT_NAME_ARRAY));
					strcat (text, "_Array");
					boneNames->setId (text);
					boneNames->setCount (skinModifier->m_boneCount);
					domListOfNames &boneSrcArray = boneNames->getValue();

					for (void* ptr = scene->GetFirstChild(skinModifierNode); ptr; ptr = scene->GetNextChild(skinModifierNode, ptr)) {
						dScene::dTreeNode* node = scene->GetNodeFromLink(ptr);
						dNodeInfo* info = scene->GetInfoFromNode(node);
						if (info->IsType(dSceneNodeInfo::GetRttiType())) {
							boneSrcArray.append(uniqueNames.FindUniqueName(scene, node));
							_ASSERTE (int (boneSrcArray.getCount()) <= skinModifier->m_boneCount);
						}
					}


					//create the accessors
					domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(boneSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
					domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
					acc->setCount (skinModifier->m_boneCount);
					acc->setStride (1);
					daeURI uri (*boneNames);
					uri.set("", "", "", "", boneNames->getId());
					acc->setSource (uri);
					domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
					param->setName ("bone");
					param->setType ("Name");
				}


				{
					char text[256];
					weightsSource = daeSafeCast<domSource>(skin->createAndPlace(COLLADA_ELEMENT_SOURCE));
					sprintf (text, "%s_weights", exName);
					weightsSource->setId (text);

					dVector* weightData = new dVector [skinModifier->m_vertexCount * 4];
					for (int i = 0; i < skinModifier->m_vertexCount; i ++) {
						for (int j = 0; j < 4; j ++) {
							weightData[i * 4 + j].m_x = skinModifier->m_vertexWeights[i][j];
							weightData[i * 4 + j].m_y = 0.0f;
							weightData[i * 4 + j].m_z = 0.0f;
							weightData[i * 4 + j].m_w = 0.0f;
						}
					}
					int indexCount = dPackVertexArray (&weightData[0].m_x, 3, sizeof (dVector), skinModifier->m_vertexCount * 4, weightsIndexMap);

					domFloat_array *weights = daeSafeCast<domFloat_array>(weightsSource->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
					strcat (text, "_Array");
					weights->setId (text);
					weights->setCount (indexCount);
					domListOfFloats &weightsSrcArray = weights->getValue();
					for (int i = 0; i < indexCount; i ++) {
						weightsSrcArray.append(weightData[i].m_x);
					}
					delete[] weightData;

					//create the accessors
					domSource::domTechnique_common *srcTeqC = daeSafeCast<domSource::domTechnique_common>(weightsSource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
					domAccessor *acc = daeSafeCast<domAccessor>(srcTeqC->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
					acc->setCount (indexCount);
					acc->setStride (1);
					daeURI uri (*weights);
					uri.set("", "", "", "", weights->getId());
					acc->setSource (uri);
					domParam *param = daeSafeCast<domParam>(acc->createAndPlace(COLLADA_ELEMENT_PARAM));
					param->setName ("weight");
					param->setType ("float");
				}


				domSkin::domJoints *joints = daeSafeCast<domSkin::domJoints>(skin->createAndPlace(COLLADA_ELEMENT_JOINTS));
				domInputLocal *inputLocal = daeSafeCast<domInputLocal>(joints->createAndPlace(COLLADA_ELEMENT_INPUT));
				inputLocal->setSemantic (COMMON_PROFILE_INPUT_JOINT);
				daeURI uri0 (*boneSource);
				uri0.set("", "", "", "", boneSource->getId());
				inputLocal->setSource (uri0);

				inputLocal = daeSafeCast<domInputLocal>(joints->createAndPlace(COLLADA_ELEMENT_INPUT));
				inputLocal->setSemantic (COMMON_PROFILE_INPUT_INV_BIND_MATRIX);
				daeURI uri1 (*bindSource);
				uri1.set("", "", "", "", bindSource->getId());
				inputLocal->setSource (uri1);


				domSkin::domVertex_weights *vertexWeights = daeSafeCast<domSkin::domVertex_weights>(skin->createAndPlace(COLLADA_ELEMENT_VERTEX_WEIGHTS));
				domInputLocalOffset* localInputOffset = daeSafeCast<domInputLocalOffset>(vertexWeights->createAndPlace(COLLADA_ELEMENT_INPUT));
				localInputOffset->setSemantic (COMMON_PROFILE_INPUT_JOINT);
				localInputOffset->setOffset (0);
				daeURI uri2 (*boneSource);
				uri2.set("", "", "", "", boneSource->getId());
				localInputOffset->setSource (uri2);

				localInputOffset = daeSafeCast<domInputLocalOffset>(vertexWeights->createAndPlace(COLLADA_ELEMENT_INPUT));
				localInputOffset->setSemantic (COMMON_PROFILE_INPUT_WEIGHT);
				localInputOffset->setOffset (1);
				daeURI uri3 (*weightsSource);
				uri3.set("", "", "", "", weightsSource->getId());
				localInputOffset->setSource (uri3);


//				int* vertexArrayIndexList = new int[mesh->m_vertexCount];
//				dVector* floatPool = new dVector[mesh->m_vertexCount];
//				for (int i = 0; i < mesh->m_vertexCount; i ++) {
//					floatPool[i] = dVector (mesh->m_vertex[i * 3 + 0], mesh->m_vertex[i * 3 + 1], mesh->m_vertex[i * 3 + 2], dFloat(i));
//				}
//				int vertexCount = dModel::dPackVertexArray (&floatPool[0].m_x, 3, sizeof (dVector), mesh->m_vertexCount, vertexArrayIndexList);

				vertexWeights->setCount(skinModifier->m_vertexCount);
				domSkin::domVertex_weights::domV* v = daeSafeCast<domSkin::domVertex_weights::domV>(vertexWeights->createAndPlace(COLLADA_ELEMENT_V));
				domSkin::domVertex_weights::domVcount* vCount = daeSafeCast<domSkin::domVertex_weights::domVcount>(vertexWeights->createAndPlace(COLLADA_ELEMENT_VCOUNT));

				domListOfInts &vIndices = v->getValue();
				domListOfUInts &vCountIndices = vCount->getValue();
				for (int i = 0; i < skinModifier->m_vertexCount; i ++) {

//					int index;
//					index = int (floatPool[i].m_w);
					int count = 0;
					for (int j = 0; j < 4; j ++) {
						count += (skinModifier->m_vertexWeights[i][j] > 0.0f) ? 1 : 0;
					}
					vCountIndices.append(count);

					for (int j = 0; j < count; j ++) {
						int boneIndex = skinModifier->m_boneWeightIndex[i].m_index[j];
						vIndices.append(boneIndex);
						vIndices.append(weightsIndexMap[i * 4 + j]);
					}
				}

//				delete[] floatPool; 
//				delete[] vertexArrayIndexList;

				delete[] weightsIndexMap;
			}
		}
	}
}


//void AddMeshInstance (daeDocument* document, daeElement *node, dMesh* mesh)
void ColladaExport::AddMeshInstance (daeDocument *document, const dScene* scene, daeElement *node, dScene::dTreeNode* meshNode, UniqueNameFilter& uniqueNames)
{
	domCOLLADA* domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
	domLibrary_materials* materialLibrary = domRoot->getLibrary_materials_array()[0];
	domLibrary_geometries* geometryLibrary = domRoot->getLibrary_geometries_array()[0];
	const domGeometry_Array &geometryArray =  geometryLibrary->getGeometry_array();
	int geometryCount = int (geometryArray.getCount());
	
	const char* name = uniqueNames.FindUniqueName(scene, meshNode);
	for (int i = 0; i < geometryCount; i ++) {

		domGeometry *meshGeometry = geometryArray[i];
		if (!cdom::strcasecmp (meshGeometry->getId(), name)) {

			domInstance_geometry *meshInstance = daeSafeCast<domInstance_geometry>(node->createAndPlace(COLLADA_ELEMENT_INSTANCE_GEOMETRY));
			daeURI uri(*meshGeometry);
			uri.set("", "", "", "", meshGeometry->getId());
			meshInstance->setUrl (uri);

			domMesh *colladaMesh = meshGeometry->getMesh();
//			const domTriangles_Array &trianglesArray = colladaMesh->getTriangles_array();
			const domPolygons_Array &polygonsArray = colladaMesh->getPolygons_array();
			int polyCount = int (polygonsArray.getCount());

			domBind_material *bindMat = daeSafeCast<domBind_material>(meshInstance->createAndPlace(COLLADA_ELEMENT_BIND_MATERIAL));
			domBind_material::domTechnique_common *bmtc = daeSafeCast<domBind_material::domTechnique_common>(bindMat->createAndPlace (COLLADA_ELEMENT_TECHNIQUE_COMMON));

			for (int j = 0; j < polyCount; j ++) {
				domPolygons* polygons = polygonsArray[j];

				domInstance_material* instMat = daeSafeCast<domInstance_material>(bmtc->createAndPlace(COLLADA_ELEMENT_INSTANCE_MATERIAL ));
				domMaterial* material = FindMaterial (materialLibrary, polygons->getMaterial());

				_ASSERTE (material);
				daeURI uri(*material);
				uri.set("", "", "", "", material->getId());
				instMat->setSymbol(polygons->getMaterial());
				instMat->setTarget (uri);
			}
			break;
		}
	}
}



void ColladaExport::AddVisualSceneLibrary (daeDocument *document, const dScene* scene, UniqueNameFilter& uniqueNames, ModifierNodes& modifierList)
{
	struct ParentChilePair
	{
		domNode *m_parent;
		dScene::dTreeNode* m_node;
		dScene::dTreeNode* m_parentNode;
	};
	dList<ParentChilePair> stack; 
	domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());


	domLibrary_visual_scenes *library = daeSafeCast<domLibrary_visual_scenes>(domRoot->createAndPlace(COLLADA_ELEMENT_LIBRARY_VISUAL_SCENES));

	//	int sceneNumber = int (library->getVisual_scene_array().getCount());
	//	sprintf (sceneName, "%svisual%03d", D_ROOT_NODE_NAME, sceneNumber);
	domVisual_scene *colladaRootNode = daeSafeCast<domVisual_scene>(library->createAndPlace(COLLADA_ELEMENT_VISUAL_SCENE));
	colladaRootNode->setId ("visualScene");

//	dScene::Iterator iter (*scene);
//	for (iter.Begin(); iter; iter ++) {
	dScene::dTreeNode* rootBone = scene->GetRootNode();
	for (void* ptr = scene->GetFirstChild(rootBone); ptr; ptr = scene->GetNextChild(rootBone, ptr)) {
		dScene::dTreeNode* node = scene->GetNodeFromLink(ptr);
		dNodeInfo* info = scene->GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			ParentChilePair item;
			item.m_node = node; 
			item.m_parentNode = NULL;
			item.m_parent = (domNode*) colladaRootNode;
			stack.Append(item);
		}
	}


	while (stack.GetCount()) {
		ParentChilePair item (stack.GetLast()->GetInfo());
		stack.Remove(stack.GetLast());

		
		const char* name = uniqueNames.CreateUniqueName(scene, item.m_node);
		domNode *tmChild = daeSafeCast<domNode>(item.m_parent->createAndPlace(COLLADA_ELEMENT_NODE));

		dSceneNodeInfo* sceneInfo = (dSceneNodeInfo*) scene->GetInfoFromNode(item.m_node);

		tmChild->setId (name);
		tmChild->setSid(sceneInfo->GetName());
//		tmChild->setType ((rootNode->GetType() == dBone::m_bone) ? NODETYPE_JOINT : NODETYPE_NODE);

//		add this node to the map
//		domInstance_camera* cameraIntance;
//		m_visualNodeMap.Insert (tmChild, mesh);
//		if this is a camera node add the camera instance
//			if ((mesh == m_cameraNode) && m_collCamera) {
//				cameraIntance = daeSafeCast<domInstance_camera>(tmChild->createAndPlace(COLLADA_ELEMENT_INSTANCE_CAMERA));
//				uri.setElement (m_collCamera);
//				uri.resolveURI();
//				cameraIntance->setUrl (uri);
//			}
//		}
//		dMatrix matrix (m_globalRotation.Inverse() * rootNode->GetMatrix() * m_globalRotation);

		dMatrix matrix (sceneInfo->GetTransform());
		if (item.m_parentNode) {
			dSceneNodeInfo* parentNodeInfo = (dSceneNodeInfo*) scene->GetInfoFromNode(item.m_parentNode);
			matrix = matrix * parentNodeInfo->GetTransform().Inverse4x4();
		}

		matrix = m_globalRotation.Inverse() * matrix * m_globalRotation;

		// check if the node has scale			

//		domTranslate* position = daeSafeCast<domTranslate>(tmChild->createAndPlace(COLLADA_ELEMENT_TRANSLATE));
//		domFloat3& positValue = position->getValue();
//		positValue.setCount(3);
//		positValue[0] = matrix.m_posit.m_x;
//		positValue[1] = matrix.m_posit.m_y;
//		positValue[2] = matrix.m_posit.m_z;

//		domRotate* rotation = daeSafeCast<domRotate>(tmChild->createAndPlace(COLLADA_ELEMENT_ROTATE));
//		domFloat4& rotateValue = rotation->getValue();
//		rotateValue.setCount(4);
//		dQuaternion rot (matrix);
//		dVector dir (rot.m_q1, rot.m_q2, rot.m_q3, 0.0f);
//		float mag2 = dir % dir;
//		if (mag2 > 1.0e-6f) {
//			dir = dir.Scale (1.0f / dSqrt (mag2));
//		} else {
//			dir = dVector (1.0f, 0.0f, 0.0f, 0.0f);
//		}
//		rotateValue[0] = dir.m_x;
//		rotateValue[1] = dir.m_y;
//		rotateValue[2] = dir.m_z;
//		rotateValue[3] = (2.0f * dAcos (rot.m_q0)) * (180.0f / 3.14159265f);

		domMatrix* colladaMatrix = daeSafeCast<domMatrix>(tmChild->createAndPlace(COLLADA_ELEMENT_MATRIX));
		domFloat4x4& data = colladaMatrix->getValue();
		data.setCount(16);
		for (int j = 0; j < 4; j ++) {
			for (int k = 0; k < 4; k ++) {
				data[j * 4 + k] = matrix[k][j];
			}
		}
		
		for (void* ptr = scene->GetFirstChild(item.m_node); ptr; ptr = scene->GetNextChild(item.m_node, ptr)) {
			dScene::dTreeNode* meshNode = scene->GetNodeFromLink(ptr);
			dNodeInfo* info = scene->GetInfoFromNode(meshNode);
			if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
				dScene::dTreeNode* skinModifierNode = NULL;	
				for (void* ptr = scene->GetFirstChild(meshNode); ptr; ptr = scene->GetNextChild(meshNode, ptr)) {
					dScene::dTreeNode* skinNode = scene->GetNodeFromLink(ptr);
					dNodeInfo* info = scene->GetInfoFromNode(skinNode);
					if (info->GetTypeId() == dGeometryNodeSkinModifierInfo::GetRttiType()) {
						skinModifierNode = skinNode;
						break;
					}
				}
				if (!skinModifierNode) {
					_ASSERTE (info->GetTypeId() == dMeshNodeInfo::GetRttiType());
					AddMeshInstance (document, scene, tmChild, meshNode, uniqueNames);
				} else {
					modifierList.Insert(tmChild, meshNode);
				}
				break;
			}
		}

		for (void* ptr = scene->GetFirstChild(item.m_node); ptr; ptr = scene->GetNextChild(item.m_node, ptr)) {
			dScene::dTreeNode* node = scene->GetNodeFromLink(ptr);
			dNodeInfo* info = scene->GetInfoFromNode(node);
			if (info->IsType(dSceneNodeInfo::GetRttiType())) {
				ParentChilePair newItem;
				newItem.m_node = node; 
				newItem.m_parentNode = item.m_node;
				newItem.m_parent = tmChild;
				stack.Append(newItem);
			}
		}
	}
}



void ColladaExport::AddControllerInstances(daeDocument *document, const dScene* scene, UniqueNameFilter& uniqueNames, ModifierNodes& modifierList)
{
	domCOLLADA *domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());

	ModifierNodes::Iterator iter (modifierList);
	for (iter.Begin(); iter; iter ++) {
		domNode *node = iter.GetNode()->GetInfo();
		dScene::dTreeNode* meshNode = iter.GetKey(); 

		char name[256];
		const char* meshName = uniqueNames.FindUniqueName(scene, meshNode);
		sprintf (name, "%s%s", meshName, D_SKIN_POSFIX_NAME);

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

				//dBone* modelRootBone = ((dSkinModifier*)meshInstance->GetModifier())->m_skinnedBones[0]->GetRoot();
				//domNode* rootBone = FindNodeNodeByName (root, modelRootBone->GetName());
				domNode* rootBone = node;
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
						const domPolygons_Array &polygonArray = colladaMesh->getPolygons_array();
						int polyCount = int (polygonArray.getCount());
						domBind_material *bindMat = daeSafeCast<domBind_material>(controllerInstance->createAndPlace(COLLADA_ELEMENT_BIND_MATERIAL));
						domBind_material::domTechnique_common *bmtc = daeSafeCast<domBind_material::domTechnique_common>(bindMat->createAndPlace (COLLADA_ELEMENT_TECHNIQUE_COMMON));

						for (int j = 0; j < polyCount; j ++) {
							domPolygons* polygons = polygonArray[j];
							domInstance_material *instMat = daeSafeCast<domInstance_material>(bmtc->createAndPlace(COLLADA_ELEMENT_INSTANCE_MATERIAL ));
							domMaterial* material = FindMaterial (materialLibrary, polygons->getMaterial());
							_ASSERTE (material);
							daeURI uri2 (*material);
							uri2.set("", "", "", "", material->getId());
							instMat->setSymbol(polygons->getMaterial());
							instMat->setTarget (uri2);
						}
						break;
					}
				}
				break;
			}
		}
	}
}



void ColladaExport::AddScene (daeDocument* document)
{
	domCOLLADA* domRoot = daeSafeCast<domCOLLADA>(document->getDomRoot());
	domCOLLADA::domScene* sceneInstance = daeSafeCast<domCOLLADA::domScene>(domRoot->createAndPlace(COLLADA_ELEMENT_SCENE));

	_ASSERTE (domRoot->getLibrary_visual_scenes_array().getCount());

	domLibrary_visual_scenes* visualLibrary = domRoot->getLibrary_visual_scenes_array()[0];
	domVisual_scene_Array& visualScenes = visualLibrary->getVisual_scene_array();

	domVisual_scene* visualScene = visualScenes[0];
	domInstanceWithExtra* ivs = daeSafeCast<domInstanceWithExtra>(sceneInstance->createAndPlace(COLLADA_ELEMENT_INSTANCE_VISUAL_SCENE));
	daeURI uri (*visualScene);
	uri.set("", "", "", "", visualScene->getId());
	ivs->setUrl (uri);

	if (domRoot->getLibrary_physics_scenes_array().getCount()) {
		_ASSERTE (0);
/*
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
*/
	}
}



void ColladaExport::SceneToCollada (DAE* collada, const dScene* scene, UniqueNameFilter& uniqueNames)
{
//	daeInt error;
//	daeDatabase* database;
//	daeDocument *document;
//	dTree<domNode*, dModel*> nodeMap; 

	daeDocument *document = NULL;
	daeDatabase* database = collada->getDatabase();
	daeInt error;
	error = database->insertDocument ("mainDoc", &document);

	_ASSERTE (error == DAE_OK);
	_ASSERTE (document);

	document = database->getDocument(daeUInt (0));

	// save graphical part
	AddAsset (document);


	

/*
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
*/

	ModifierNodes modifierList;
	AddCameraLibrary (document, scene);
	AddImageLibrary (document, scene);
	AddMaterialLibrary (document, scene);

	AddGeometryLibrary (document, scene, uniqueNames);
	AddVisualSceneLibrary (document, scene, uniqueNames, modifierList);

	AddControllerLibrary(document, scene, uniqueNames);
	AddControllerInstances(document, scene, uniqueNames, modifierList);
//	AddAnimationLibrary(document, model);
	
	AddScene (document);
}