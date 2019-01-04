/* Copyright (c) <2003-2018> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

// FbxtoNgd.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#ifdef IOS_REF
#undef  IOS_REF
#define IOS_REF (*(pManager->GetIOSettings()))
#endif

#define DEFUALT_MATERIAL_ID -1

class CheckMemoryLeaks
{
	public:
	CheckMemoryLeaks()
	{
		atexit(CheckMemoryLeaksCallback);
		// Track all memory leaks at the operating system level.
		// make sure no Newton tool or utility leaves leaks behind.
		_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF));
		//_CrtSetBreakAlloc (34092);
	}

	static void CheckMemoryLeaksCallback()
	{
		_CrtDumpMemoryLeaks();
	}
};
static CheckMemoryLeaks checkLeaks;


class ImportStackData
{
	public:
	ImportStackData(const dMatrix& parentMatrix, FbxNode* const fbxNode, dScene::dTreeNode* parentNode)
		:m_parentMatrix(parentMatrix)
		,m_fbxNode(fbxNode)
		,m_parentNode(parentNode)
	{
	}
	dMatrix m_parentMatrix;
	FbxNode* m_fbxNode;
	dScene::dTreeNode* m_parentNode;
};

class GlobalNodeMap: public dTree<dScene::dTreeNode*, FbxNode*>
{
};

class UsedMaterials: public dTree<int, dScene::dTreeNode*>
{
};

class GlobalMeshMap: public dTree<dScene::dTreeNode*, FbxNodeAttribute*>
{
};

class GlobalTextureMap: public dTree<dScene::dTreeNode*, FbxTexture*>
{
};

class GlobalMaterialMap: public dTree<dScene::dTreeNode*, FbxSurfaceMaterial*>
{
};

class LocalMaterialMap: public dTree<dScene::dTreeNode*, int>
{
};

static int g_materialId = 0;
static int InitializeSdkObjects(FbxManager*& pManager, FbxScene*& pScene);
static bool LoadScene(FbxManager* pManager, FbxDocument* pScene, const char* pFilename);
static bool ConvertToNgd(dScene* const ngdScene, FbxScene* fbxScene);
static void PopulateScene(dScene* const ngdScene, FbxScene* const fbxScene);
static void LoadHierarchy(FbxScene* const fbxScene, dScene* const ngdScene, GlobalNodeMap& nodeMap);
static void ImportMeshNode(FbxScene* const fbxScene, dScene* const ngdScene, FbxNode* const fbxMeshNode, dScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials, GlobalNodeMap& nodeMap);
static void ImportMaterials(FbxScene* const fbxScene, dScene* const ngdScene, FbxNode* const fbxMeshNode, dScene::dTreeNode* const meshNode, GlobalMaterialMap& materialCache, LocalMaterialMap& localMaterilIndex, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials);
static void ImportTexture(dScene* const ngdScene, FbxProperty pProperty, dScene::dTreeNode* const materialNode, GlobalTextureMap& textureCache);
static void ImportSkeleton(dScene* const ngdScene, FbxScene* const fbxScene, FbxNode* const fbxNode, dScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials, int boneId);
static void ImportAnimations(FbxScene* const fbxScene, dScene* const ngdScene, GlobalNodeMap& nodeMap);
static void ImportAnimationLayer(FbxAnimLayer* const animLayer, dScene* const ngdScene, GlobalNodeMap& nodeMap, FbxNode* const pNode, dScene::dTreeNode* const animationLayers);


int InitializeSdkObjects(FbxManager*& pManager, FbxScene*& pScene)
{
	//The first thing to do is to create the FBX Manager which is the object allocator for almost all the classes in the SDK
	pManager = FbxManager::Create();
	if (!pManager)
	{
		FBXSDK_printf("Error: Unable to create FBX Manager!\n");
		exit(1);
	} else FBXSDK_printf("Autodesk FBX SDK version %s\n", pManager->GetVersion());

	//Create an IOSettings object. This object holds all import/export settings.
	FbxIOSettings* ios = FbxIOSettings::Create(pManager, IOSROOT);
	pManager->SetIOSettings(ios);

	//Load plugins from the executable directory (optional)
	FbxString lPath = FbxGetApplicationDirectory();
	pManager->LoadPluginsDirectory(lPath.Buffer());

	//Create an FBX scene. This object holds most objects imported/exported from/to files.
	pScene = FbxScene::Create(pManager, "My Scene");
	if (!pScene)
	{
		FBXSDK_printf("Error: Unable to create FBX scene!\n");
		return 0;
	}
	return 1;
}

bool LoadScene(FbxManager* pManager, FbxDocument* pScene, const char* pFilename)
{
	int lFileMajor, lFileMinor, lFileRevision;
	int lSDKMajor, lSDKMinor, lSDKRevision;
	//int lFileFormat = -1;
	bool lStatus;
	char lPassword[1024];

	// Get the file version number generate by the FBX SDK.
	FbxManager::GetFileFormatVersion(lSDKMajor, lSDKMinor, lSDKRevision);

	// Create an importer.
	FbxImporter* lImporter = FbxImporter::Create(pManager, "");

	// Initialize the importer by providing a filename.
	const bool lImportStatus = lImporter->Initialize(pFilename, -1, pManager->GetIOSettings());
	lImporter->GetFileVersion(lFileMajor, lFileMinor, lFileRevision);

	if (!lImportStatus)
	{
		FbxString error = lImporter->GetStatus().GetErrorString();
		FBXSDK_printf("Call to FbxImporter::Initialize() failed.\n");
		FBXSDK_printf("Error returned: %s\n\n", error.Buffer());

		if (lImporter->GetStatus().GetCode() == FbxStatus::eInvalidFileVersion)
		{
			FBXSDK_printf("FBX file format version for this FBX SDK is %d.%d.%d\n", lSDKMajor, lSDKMinor, lSDKRevision);
			FBXSDK_printf("FBX file format version for file '%s' is %d.%d.%d\n\n", pFilename, lFileMajor, lFileMinor, lFileRevision);
		}

		return false;
	}

	FBXSDK_printf("FBX file format version for this FBX SDK is %d.%d.%d\n", lSDKMajor, lSDKMinor, lSDKRevision);

	if (lImporter->IsFBX())
	{
		FBXSDK_printf("FBX file format version for file '%s' is %d.%d.%d\n\n", pFilename, lFileMajor, lFileMinor, lFileRevision);

		// Set the import states. By default, the import states are always set to 
		// true. The code below shows how to change these states.
		IOS_REF.SetBoolProp(IMP_FBX_MATERIAL, true);
		IOS_REF.SetBoolProp(IMP_FBX_TEXTURE, true);
		IOS_REF.SetBoolProp(IMP_FBX_LINK, true);
		IOS_REF.SetBoolProp(IMP_FBX_SHAPE, true);
		IOS_REF.SetBoolProp(IMP_FBX_GOBO, true);
		IOS_REF.SetBoolProp(IMP_FBX_ANIMATION, true);
		IOS_REF.SetBoolProp(IMP_FBX_GLOBAL_SETTINGS, true);
	}

	// Import the scene.
	lStatus = lImporter->Import(pScene);

	if (lStatus == false && lImporter->GetStatus().GetCode() == FbxStatus::ePasswordError)
	{
		FBXSDK_printf("Please enter password: ");

		lPassword[0] = '\0';

		FBXSDK_CRT_SECURE_NO_WARNING_BEGIN
			scanf("%s", lPassword);
		FBXSDK_CRT_SECURE_NO_WARNING_END

			FbxString lString(lPassword);

		IOS_REF.SetStringProp(IMP_FBX_PASSWORD, lString);
		IOS_REF.SetBoolProp(IMP_FBX_PASSWORD_ENABLE, true);

		lStatus = lImporter->Import(pScene);

		if (lStatus == false && lImporter->GetStatus().GetCode() == FbxStatus::ePasswordError)
		{
			FBXSDK_printf("\nPassword is wrong, import aborted.\n");
		}
	}

	// Destroy the importer.
	lImporter->Destroy();

	return lStatus;
}


bool ConvertToNgd(dScene* const ngdScene, FbxScene* fbxScene)
{
	FbxGlobalSettings& settings = fbxScene->GetGlobalSettings();

	const FbxSystemUnit& systemUnit = settings.GetSystemUnit();
	dFloat scaleFactor = dFloat(systemUnit.GetScaleFactor());

	dMatrix convertMatrix(dGetIdentityMatrix());
	convertMatrix[0][0] = dFloat(scaleFactor / 100.0f);
	convertMatrix[1][1] = dFloat(scaleFactor / 100.0f);
	convertMatrix[2][2] = dFloat(scaleFactor / 100.0f);


	int sign;
	dVector upVector(0.0f, 1.0f, 0.0f, 0.0f);
	dVector frontVector(1.0f, 0.0f, 0.0f, 0.0f);
	FbxAxisSystem axisSystem = settings.GetAxisSystem();
	if (axisSystem.GetUpVector(sign) == FbxAxisSystem::eXAxis) {
		dAssert(0);
	} else if (axisSystem.GetUpVector(sign) == FbxAxisSystem::eYAxis) {
		upVector = dVector(0.0f, 1.0f * sign, 0.0f, 0.0f);
		if (axisSystem.GetFrontVector(sign) == FbxAxisSystem::eParityEven) {
			frontVector = dVector(1.0f * sign, 0.0f, 0.0f, 0.0f);
		} else {
			frontVector = dVector(0.0f, 0.0f, 1.0f * sign, 0.0f);
		}
	} else {
		upVector = dVector(1.0f * sign, 0.0f, 0.0f, 0.0f);
		if (axisSystem.GetFrontVector(sign) == FbxAxisSystem::eParityEven) {
			dAssert(0);
			frontVector = dVector(1.0f * sign, 0.0f, 0.0f, 0.0f);
		} else {
			frontVector = dVector(0.0f, 0.0f, -1.0f * sign, 0.0f);
		}
	}

	dMatrix axisMatrix(dGetIdentityMatrix());
	axisMatrix.m_front = frontVector;
	axisMatrix.m_up = upVector;
	axisMatrix.m_right = frontVector.CrossProduct(upVector);
	convertMatrix = axisMatrix * convertMatrix;

	PopulateScene(ngdScene, fbxScene);

	ngdScene->RemoveUnusedMaterials();
	ngdScene->BakeTransform(convertMatrix);

	return true;
}

void PopulateScene(dScene* const ngdScene, FbxScene* const fbxScene)
{
	GlobalNodeMap nodeMap;
	GlobalMeshMap meshCache;
	GlobalTextureMap textureCache;
	GlobalMaterialMap materialCache;
	UsedMaterials usedMaterials;

	dScene::dTreeNode* const defaulMaterialNode = ngdScene->CreateMaterialNode(DEFUALT_MATERIAL_ID);
	dMaterialNodeInfo* const defaulMaterial = (dMaterialNodeInfo*)ngdScene->GetInfoFromNode(defaulMaterialNode);
	defaulMaterial->SetName("default_material");
	usedMaterials.Insert(0, defaulMaterialNode);

	g_materialId = 0;
	LoadHierarchy(fbxScene, ngdScene, nodeMap);

	int boneId = 0;
	GlobalNodeMap::Iterator iter(nodeMap);
	for (iter.Begin(); iter; iter++) {
		FbxNode* const fbxNode = iter.GetKey();
		dScene::dTreeNode* const ngdNode = iter.GetNode()->GetInfo();
		FbxNodeAttribute* const attribute = fbxNode->GetNodeAttribute();

		if (attribute) {
			FbxNodeAttribute::EType attributeType = attribute->GetAttributeType();
			if (attributeType == FbxNodeAttribute::eSkeleton)
			{
				ImportSkeleton(ngdScene, fbxScene, fbxNode, ngdNode, meshCache, materialCache, textureCache, usedMaterials, boneId);
				boneId++;
			}
		}
	}

	for (iter.Begin(); iter; iter++) {
		FbxNode* const fbxNode = iter.GetKey();
		dScene::dTreeNode* const ngdNode = iter.GetNode()->GetInfo();
		FbxNodeAttribute* const attribute = fbxNode->GetNodeAttribute();

		if (attribute) {
			FbxNodeAttribute::EType attributeType = attribute->GetAttributeType();

			switch (attributeType)
			{
				case FbxNodeAttribute::eMesh:
				{
					ImportMeshNode(fbxScene, ngdScene, fbxNode, ngdNode, meshCache, materialCache, textureCache, usedMaterials, nodeMap);
					break;
				}

				case FbxNodeAttribute::eSkeleton:
				{
					//ImportSkeleton(ngdScene, fbxScene, fbxNode, ngdNode, meshCache, materialCache, textureCache, usedMaterials, boneId);
					//boneId++;
					break;
				}

				case FbxNodeAttribute::eLine:
				{
					dAssert(0);
					//ImportLineShape(fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials);
					break;
				}

				case FbxNodeAttribute::eNurbsCurve:
				{
					dAssert(0);
					//ImportNurbCurveShape(fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials);
					break;
				}

				case FbxNodeAttribute::eNull:
				{
					break;
				}

				case FbxNodeAttribute::eMarker:
				case FbxNodeAttribute::eNurbs:
				case FbxNodeAttribute::ePatch:
				case FbxNodeAttribute::eCamera:
				case FbxNodeAttribute::eCameraStereo:
				case FbxNodeAttribute::eCameraSwitcher:
				case FbxNodeAttribute::eLight:
				case FbxNodeAttribute::eOpticalReference:
				case FbxNodeAttribute::eOpticalMarker:

				case FbxNodeAttribute::eTrimNurbsSurface:
				case FbxNodeAttribute::eBoundary:
				case FbxNodeAttribute::eNurbsSurface:
				case FbxNodeAttribute::eShape:
				case FbxNodeAttribute::eLODGroup:
				case FbxNodeAttribute::eSubDiv:
				case FbxNodeAttribute::eCachedEffect:
				case FbxNodeAttribute::eUnknown:
				default:
					dAssert(0);
					break;
			}
		}
	}

	UsedMaterials::Iterator iter1(usedMaterials);
	for (iter1.Begin(); iter1; iter1++) {
		int count = iter1.GetNode()->GetInfo();
		if (!count) {
			dScene::dTreeNode* const materiaCacheNode = ngdScene->FindGetMaterialCacheNode();
			dScene::dTreeNode* const materialNode = iter1.GetKey();
			void* nextLink;
			for (void* link = ngdScene->GetFirstParentLink(materialNode); link; link = nextLink) {
				nextLink = ngdScene->GetNextParentLink(materialNode, link);
				dScene::dTreeNode* const parentNode = ngdScene->GetNodeFromLink(link);
				if (parentNode != materiaCacheNode) {
					ngdScene->RemoveReference(parentNode, materialNode);
				}
			}
		}
	}

	ImportAnimations(fbxScene, ngdScene, nodeMap);
}

void ImportAnimationLayer(FbxAnimLayer* const animLayer, dScene* const ngdScene, GlobalNodeMap& nodeMap, FbxNode* const rootNode, dScene::dTreeNode* const animationTakeNode)
{
	int stack = 1;
	FbxNode* fbxNodes[32];
	fbxNodes[0] = rootNode;

	while (stack) {
		stack--;
		FbxNode* const fbxNode = fbxNodes[stack];

		FbxAnimCurve* const animCurveX = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X);
		FbxAnimCurve* const animCurveY = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y);
		FbxAnimCurve* const animCurveZ = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z);

		FbxAnimCurve* const animCurveRotX = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X);
		FbxAnimCurve* const animCurveRotY = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y);
		FbxAnimCurve* const animCurveRotZ = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z);

		if (animCurveX || animCurveRotX) {
			dScene::dTreeNode* const trackNode = ngdScene->CreateAnimationTrack(animationTakeNode);
			dAnimationTrack* const animationTrack = (dAnimationTrack*)ngdScene->GetInfoFromNode(trackNode);
			dScene::dTreeNode* const ngdNode = nodeMap.Find(fbxNode)->GetInfo();
			dSceneNodeInfo* const ngdInfo = (dSceneNodeInfo*)ngdScene->GetInfoFromNode(ngdNode);
			animationTrack->SetName(ngdInfo->GetName());
			ngdScene->AddReference(ngdNode, trackNode);

			if (animCurveX) {
				dAssert(animCurveY);
				dAssert(animCurveZ);

				int keyCount = animCurveX->KeyGetCount();
				dAssert(keyCount == animCurveY->KeyGetCount());
				dAssert(keyCount == animCurveZ->KeyGetCount());
				for (int i = 0; i < keyCount; i++) {
					FbxTime keyTime = animCurveX->KeyGetTime(i);
					dAssert(keyTime == animCurveY->KeyGetTime(i));
					dAssert(keyTime == animCurveZ->KeyGetTime(i));

					dFloat x = animCurveX->KeyGetValue(i);
					dFloat y = animCurveY->KeyGetValue(i);
					dFloat z = animCurveZ->KeyGetValue(i);
					animationTrack->AddPosition(dFloat (keyTime.GetSecondDouble()), x, y, z);
				}
			}
			if (animCurveRotX) {
				dAssert(animCurveRotY);
				dAssert(animCurveRotZ);

				int keyCount = animCurveRotX->KeyGetCount();
				dAssert(keyCount == animCurveRotY->KeyGetCount());
				dAssert(keyCount == animCurveRotZ->KeyGetCount());
				for (int i = 0; i < keyCount; i++) {
					FbxTime keyTime = animCurveRotX->KeyGetTime(i);
					dAssert(keyTime == animCurveRotY->KeyGetTime(i));
					dAssert(keyTime == animCurveRotZ->KeyGetTime(i));

					dFloat x = animCurveRotX->KeyGetValue(i);
					dFloat y = animCurveRotY->KeyGetValue(i);
					dFloat z = animCurveRotZ->KeyGetValue(i);
					animationTrack->AddRotation(dFloat(keyTime.GetSecondDouble()), x, y, z);
				}
			}

			animationTrack->OptimizeCurves();
		}

		for (int i = 0; i < fbxNode->GetChildCount(); i++) {
			fbxNodes[stack] = fbxNode->GetChild(i);
			stack++;
		}
	}
}

void ImportAnimations(FbxScene* const fbxScene, dScene* const ngdScene, GlobalNodeMap& nodeMap)
{
	const int numStacks = fbxScene->GetSrcObjectCount<FbxAnimStack>();
	if (numStacks) {
		FbxAnimStack* const animStack = fbxScene->GetSrcObject<FbxAnimStack>(0);

		FbxString lOutputString = "Animation Stack Name: ";
		lOutputString += animStack->GetName();
		lOutputString += "\n";
		FBXSDK_printf(lOutputString);
		int nbAnimLayers = animStack->GetMemberCount<FbxAnimLayer>();

		dScene::dTreeNode* const animationLayers = ngdScene->CreateAnimationLayers();

		for (int j = 0; j < nbAnimLayers; j++) {
			FbxAnimLayer* const animLayer = animStack->GetMember<FbxAnimLayer>(j);

			FbxString takeName(animLayer->GetName());

			dScene::dTreeNode* const animationTakeNode = ngdScene->CreateAnimationTake();
			dAnimationTake* const animationTake = (dAnimationTake*)ngdScene->GetInfoFromNode(animationTakeNode);
			animationTake->SetName(animLayer->GetName());
			ImportAnimationLayer(animLayer, ngdScene, nodeMap, fbxScene->GetRootNode(), animationTakeNode);
		}
	}
}

void LoadHierarchy(FbxScene* const fbxScene, dScene* const ngdScene, GlobalNodeMap& nodeMap)
{
	dList <ImportStackData> nodeStack;

	FbxAnimEvaluator* const evaluator = fbxScene->GetAnimationEvaluator();

	// Print the nodes of the scene and their attributes recursively.
	FbxNode* const rootNode = fbxScene->GetRootNode();
	if (rootNode) {
		int count = rootNode->GetChildCount();
		for (int i = 0; i < count; i++) {
			nodeStack.Append(ImportStackData(dGetIdentityMatrix(), rootNode->GetChild(count - i - 1), ngdScene->GetRootNode()));
		}
	}

	while (nodeStack.GetCount()) {

		ImportStackData data(nodeStack.GetLast()->GetInfo());
		nodeStack.Remove(nodeStack.GetLast());

		dScene::dTreeNode* const node = ngdScene->CreateSceneNode(data.m_parentNode);
		dSceneNodeInfo* const info = (dSceneNodeInfo*)ngdScene->GetInfoFromNode(node);

		dMatrix localMatrix(evaluator->GetNodeLocalTransform(data.m_fbxNode));
/*
dMatrix globalMatrix(evaluator->GetNodeGlobalTransform(data.m_fbxNode));
dVector euler1;
dVector euler2;
dVector scale;
dMatrix stretchAxis;
globalMatrix.PolarDecomposition(globalMatrix, scale, stretchAxis);
globalMatrix.GetEulerAngles(euler1, euler2);
euler1 = euler1.Scale(dRadToDegree);
euler2 = euler2.Scale(dRadToDegree);
*/

		info->SetName(data.m_fbxNode->GetName());
		info->SetTransform(localMatrix);

		FbxVector4 fbxPivotScaling(data.m_fbxNode->GetGeometricScaling(FbxNode::eSourcePivot));
		FbxVector4 fbxPivotRotation(data.m_fbxNode->GetGeometricRotation(FbxNode::eSourcePivot));
		FbxVector4 fbxPivotTranslation(data.m_fbxNode->GetGeometricTranslation(FbxNode::eSourcePivot));

		dVector pivotTranslation(dFloat(fbxPivotTranslation[0]), dFloat(fbxPivotTranslation[1]), dFloat(fbxPivotTranslation[2]), 1.0f);
		dVector pivotRotation(dFloat(fbxPivotRotation[0] * 3.14159265359 / 180.0), dFloat(fbxPivotRotation[1] * 3.14159265359 / 180.0), dFloat(fbxPivotRotation[2] * 3.14159265359 / 180.0), 0.0f);

		dMatrix pivotScale(dGetIdentityMatrix());
		pivotScale[0][0] = dFloat(fbxPivotScaling[0]);
		pivotScale[1][1] = dFloat(fbxPivotScaling[1]);
		pivotScale[2][2] = dFloat(fbxPivotScaling[2]);
		dMatrix pivotMatrix(pivotScale * dMatrix(pivotRotation[0], pivotRotation[1], pivotRotation[2], pivotTranslation));
		info->SetGeometryTransform(pivotMatrix);

		nodeMap.Insert(node, data.m_fbxNode);

		int count = data.m_fbxNode->GetChildCount();
		for (int i = 0; i < count; i++) {
			nodeStack.Append(ImportStackData(dGetIdentityMatrix(), data.m_fbxNode->GetChild(count - i - 1), node));
		}
	}
}

void ImportTexture(dScene* const ngdScene, FbxProperty pProperty, dScene::dTreeNode* const materialNode, GlobalTextureMap& textureCache)
{
	int lTextureCount = pProperty.GetSrcObjectCount<FbxTexture>();
	for (int j = 0; j < lTextureCount; ++j) {
		//Here we have to check if it's layered textures, or just textures:
		FbxLayeredTexture *lLayeredTexture = pProperty.GetSrcObject<FbxLayeredTexture>(j);
		if (lLayeredTexture)
		{
			dAssert(0);
			/*
			DisplayInt("    Layered Texture: ", j);
			FbxLayeredTexture *lLayeredTexture = pProperty.GetSrcObject<FbxLayeredTexture>(j);
			int lNbTextures = lLayeredTexture->GetSrcObjectCount<FbxTexture>();
			for(int k =0; k<lNbTextures; ++k)
			{
			FbxTexture* lTexture = lLayeredTexture->GetSrcObject<FbxTexture>(k);
			if(lTexture)
			{

			if(pDisplayHeader){
			DisplayInt("    Textures connected to Material ", pMaterialIndex);
			pDisplayHeader = false;
			}

			//NOTE the blend mode is ALWAYS on the LayeredTexture and NOT the one on the texture.
			//Why is that?  because one texture can be shared on different layered textures and might
			//have different blend modes.

			FbxLayeredTexture::EBlendMode lBlendMode;
			lLayeredTexture->GetTextureBlendMode(k, lBlendMode);
			DisplayString("    Textures for ", pProperty.GetName());
			DisplayInt("        Texture ", k);
			DisplayTextureInfo(lTexture, (int) lBlendMode);
			}
			}
			*/
		} else {
			//no layered texture simply get on the property
			FbxTexture* const texture = pProperty.GetSrcObject<FbxTexture>(j);
			if (texture) {
				GlobalTextureMap::dTreeNode* textCacheNode = textureCache.Find(texture);
				if (!textCacheNode) {

					//const FbxString& name = pProperty.GetName();
					//const char* const texPathName = name.Buffer();
					dAssert(pProperty.GetName() == "DiffuseColor");

					FbxFileTexture* const fileTexture = FbxCast<FbxFileTexture>(texture);
					//FbxProceduralTexture* const proceduralTexture = FbxCast<FbxProceduralTexture>(texture);
					dAssert(!FbxCast<FbxProceduralTexture>(texture));

					const char* const texPathName = fileTexture->GetFileName();

					/*
					DisplayDouble("            Scale U: ", pTexture->GetScaleU());
					DisplayDouble("            Scale V: ", pTexture->GetScaleV());
					DisplayDouble("            Translation U: ", pTexture->GetTranslationU());
					DisplayDouble("            Translation V: ", pTexture->GetTranslationV());
					DisplayBool("            Swap UV: ", pTexture->GetSwapUV());
					DisplayDouble("            Rotation U: ", pTexture->GetRotationU());
					DisplayDouble("            Rotation V: ", pTexture->GetRotationV());
					DisplayDouble("            Rotation W: ", pTexture->GetRotationW());
					const char* lAlphaSources[] = { "None", "RGB Intensity", "Black" };
					DisplayString("            Alpha Source: ", lAlphaSources[pTexture->GetAlphaSource()]);
					DisplayDouble("            Cropping Left: ", pTexture->GetCroppingLeft());
					DisplayDouble("            Cropping Top: ", pTexture->GetCroppingTop());
					DisplayDouble("            Cropping Right: ", pTexture->GetCroppingRight());
					DisplayDouble("            Cropping Bottom: ", pTexture->GetCroppingBottom());
					const char* lMappingTypes[] = { "Null", "Planar", "Spherical", "Cylindrical", "Box", "Face", "UV", "Environment" };
					DisplayString("            Mapping Type: ", lMappingTypes[pTexture->GetMappingType()]);
					if (pTexture->GetMappingType() == FbxTexture::ePlanar) {
					const char* lPlanarMappingNormals[] = { "X", "Y", "Z" };

					DisplayString("            Planar Mapping Normal: ", lPlanarMappingNormals[pTexture->GetPlanarMappingNormal()]);
					}

					const char* lBlendModes[]   = { "Translucent", "Add", "Modulate", "Modulate2" };
					if(pBlendMode >= 0)
					DisplayString("            Blend Mode: ", lBlendModes[pBlendMode]);
					DisplayDouble("            Alpha: ", pTexture->GetDefaultAlpha());

					if (lFileTexture) {
					const char* lMaterialUses[] = { "Model Material", "Default Material" };
					DisplayString("            Material Use: ", lMaterialUses[lFileTexture->GetMaterialUse()]);
					}

					const char* pTextureUses[] = { "Standard", "Shadow Map", "Light Map", "Spherical Reflexion Map", "Sphere Reflexion Map", "Bump Normal Map" };
					DisplayString("            Texture Use: ", pTextureUses[pTexture->GetTextureUse()]);
					DisplayString("");

					*/
					char path[2048];
					const char* const texName = dGetNameFromPath(texPathName);
					dExtractPathFromFullName(texPathName, path);

					dScene::dTreeNode* const textNode = ngdScene->CreateTextureNode(texName);
					dTextureNodeInfo* const textureInfo = (dTextureNodeInfo*)ngdScene->GetInfoFromNode(textNode);
					textureInfo->SetName(texName);
					textureInfo->SetPathName(texName);
					textCacheNode = textureCache.Insert(textNode, texture);
				}

				// for now only set the Diffuse texture
				dScene::dTreeNode* const textNode = textCacheNode->GetInfo();
				dTextureNodeInfo* const textureInfo = (dTextureNodeInfo*)ngdScene->GetInfoFromNode(textNode);
				dMaterialNodeInfo* const materialInfo = (dMaterialNodeInfo*)ngdScene->GetInfoFromNode(materialNode);
				ngdScene->AddReference(materialNode, textNode);

				materialInfo->SetDiffuseTextId(textureInfo->GetId());
				materialInfo->SetAmbientTextId(textureInfo->GetId());
				materialInfo->SetSpecularTextId(textureInfo->GetId());
			}
		}
	}
}

void ImportMaterials(FbxScene* const fbxScene, dScene* const ngdScene, FbxNode* const fbxMeshNode, dScene::dTreeNode* const meshNode, GlobalMaterialMap& materialCache, LocalMaterialMap& localMaterilIndex, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials)
{
	dScene::dTreeNode* const materialNode = ngdScene->FindMaterialById(DEFUALT_MATERIAL_ID);
	localMaterilIndex.Insert(materialNode, DEFUALT_MATERIAL_ID);
	ngdScene->AddReference(meshNode, materialNode);

	int materialCount = fbxMeshNode->GetMaterialCount();
	if (materialCount) {
		for (int i = 0; i < materialCount; i++) {
			FbxSurfaceMaterial* const fbxMaterial = fbxMeshNode->GetMaterial(i);
			GlobalMaterialMap::dTreeNode* globalMaterialCacheNode = materialCache.Find(fbxMaterial);
			if (!globalMaterialCacheNode) {
				dScene::dTreeNode* const materialNode = ngdScene->CreateMaterialNode(g_materialId);
				globalMaterialCacheNode = materialCache.Insert(materialNode, fbxMaterial);
				usedMaterials.Insert(0, materialNode);

				dMaterialNodeInfo* const materialInfo = (dMaterialNodeInfo*)ngdScene->GetInfoFromNode(materialNode);
				materialInfo->SetName(fbxMaterial->GetName());

				dVector ambient(materialInfo->GetAmbientColor());
				dVector difusse(materialInfo->GetDiffuseColor());
				dVector specular(materialInfo->GetSpecularColor());
				dFloat opacity = materialInfo->GetOpacity();
				dFloat shininess = materialInfo->GetShininess();
				if (fbxMaterial->GetClassId().Is(FbxSurfacePhong::ClassId)) {
					FbxSurfacePhong* const phongMaterial = (FbxSurfacePhong *)fbxMaterial;

					// Get the ambient Color
					ambient[0] = dFloat(phongMaterial->Ambient.Get()[0]);
					ambient[1] = dFloat(phongMaterial->Ambient.Get()[1]);
					ambient[2] = dFloat(phongMaterial->Ambient.Get()[2]);
					ambient[3] = 1.0f;

					// get the Diffuse Color
					difusse[0] = dFloat(phongMaterial->Diffuse.Get()[0]);
					difusse[1] = dFloat(phongMaterial->Diffuse.Get()[1]);
					difusse[2] = dFloat(phongMaterial->Diffuse.Get()[2]);
					difusse[3] = 1.0f;

					// Get specular Color (unique to Phong materials)
					specular[0] = dFloat(phongMaterial->Specular.Get()[0]);
					specular[1] = dFloat(phongMaterial->Specular.Get()[1]);
					specular[2] = dFloat(phongMaterial->Specular.Get()[2]);
					specular[3] = 1.0f;

					// Display the Shininess
					shininess = dFloat(phongMaterial->Shininess.Get());

					// Display the Emissive Color
					//lKFbxDouble3 =((FbxSurfacePhong *) lMaterial)->Emissive;
					//theColor.Set(lKFbxDouble3.Get()[0], lKFbxDouble3.Get()[1], lKFbxDouble3.Get()[2]);
					//DisplayColor("            Emissive: ", theColor);

					//Opacity is Transparency factor now
					opacity = dFloat(1.0 - phongMaterial->TransparencyFactor.Get());

					// Display the Reflectivity
					//lKFbxDouble1 =((FbxSurfacePhong *) lMaterial)->ReflectionFactor;
					//DisplayDouble("            Reflectivity: ", lKFbxDouble1.Get());
				} else {
					dAssert(0);
				}

				materialInfo->SetAmbientColor(ambient);
				materialInfo->SetDiffuseColor(difusse);
				materialInfo->SetSpecularColor(specular);
				materialInfo->SetShininess(shininess);
				materialInfo->SetOpacity(opacity);

				g_materialId++;
			}
			dScene::dTreeNode* const materialNode = globalMaterialCacheNode->GetInfo();
			localMaterilIndex.Insert(materialNode, i);
			ngdScene->AddReference(meshNode, materialNode);

			// assume layer 0 for now
			FbxProperty textureProperty(fbxMaterial->FindProperty(FbxLayerElement::sTextureChannelNames[0]));
			if (textureProperty.IsValid()) {
				ImportTexture(ngdScene, textureProperty, materialNode, textureCache);
			}
		}
	}
}


void ImportMeshNode(FbxScene* const fbxScene, dScene* const ngdScene, FbxNode* const fbxMeshNode, dScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials, GlobalNodeMap& nodeMap)
{
	GlobalMeshMap::dTreeNode* instanceNode = meshCache.Find(fbxMeshNode->GetMesh());
	if (instanceNode) {
		dScene::dTreeNode* const meshInstance = instanceNode->GetInfo();
		ngdScene->AddReference(node, meshInstance);
	} else {
		FbxMesh* const fbxMesh = fbxMeshNode->GetMesh();
		dScene::dTreeNode* const meshNode = ngdScene->CreateMeshNode(node);
		meshCache.Insert(meshNode, fbxMesh);

		dMeshNodeInfo* const instance = (dMeshNodeInfo*)ngdScene->GetInfoFromNode(meshNode);
		char name[256];
		sprintf(name, "%s_mesh", fbxMeshNode->GetName());
		instance->SetName(name);

		FbxAMatrix pivotMatrix;
		fbxMesh->GetPivot(pivotMatrix);

		dMatrix matrix(pivotMatrix);
		instance->SetPivotMatrix(matrix);
		dAssert(matrix[1][1] == 1.0f);

		LocalMaterialMap localMaterialIndex;
		ImportMaterials(fbxScene, ngdScene, fbxMeshNode, meshNode, materialCache, localMaterialIndex, textureCache, usedMaterials);

		//int materialID = 0;		
		//dScene::dTreeNode* const matNode = ngdScene->CreateMaterialNode(materialID);
		//dMaterialNodeInfo* const material = (dMaterialNodeInfo*) ngdScene->GetInfoFromNode(matNode);
		//material->SetName("default material");
		//materialID ++;

		int indexCount = 0;
		int faceCount = fbxMesh->GetPolygonCount();
		for (int i = 0; i < faceCount; i++) {
			indexCount += fbxMesh->GetPolygonSize(i);
		}

		int* const faceIndexList = new int[faceCount];
		int* const materialIndex = new int[faceCount];
		int* const vertexIndex = new int[indexCount];
		int* const normalIndex = new int[indexCount];
		int* const uv0Index = new int[indexCount];
		int* const uv1Index = new int[indexCount];

		dBigVector* const vertexArray = new dBigVector[fbxMesh->GetControlPointsCount()];
		dVector* const normalArray = new dVector[indexCount];
		dVector* const uv0Array = new dVector[indexCount];
		dVector* const uv1Array = new dVector[indexCount];

		int controlCount = fbxMesh->GetControlPointsCount();
		const FbxVector4* const controlPoints = fbxMesh->GetControlPoints();
		for (int i = 0; i < controlCount; i++) {
			const FbxVector4& p = controlPoints[i];
			vertexArray[i] = dBigVector(p[0], p[1], p[2], 0.0);
		}

		FbxGeometryElementUV* const uvArray = fbxMesh->GetElementUV();
		FbxLayerElement::EMappingMode uvMapingMode = uvArray ? uvArray->GetMappingMode() : FbxGeometryElement::eNone;
		FbxLayerElement::EReferenceMode uvRefMode = uvArray ? uvArray->GetReferenceMode() : FbxGeometryElement::eIndex;

		FbxGeometryElementMaterial* const materialArray = fbxMesh->GetElementMaterial();
		FbxLayerElement::EMappingMode materialMapingMode = materialArray ? materialArray->GetMappingMode() : FbxGeometryElement::eNone;
		FbxLayerElement::EReferenceMode materialRefMode = materialArray ? materialArray->GetReferenceMode() : FbxGeometryElement::eIndex;

		int index = 0;
		for (int i = 0; i < faceCount; i++) {
			int polygonIndexCount = fbxMesh->GetPolygonSize(i);

			int materialID = DEFUALT_MATERIAL_ID;
			if (materialArray) {
				if (materialMapingMode == FbxGeometryElement::eByPolygon) {
					materialID = (materialRefMode == FbxGeometryElement::eDirect) ? i : materialArray->GetIndexArray().GetAt(i);
				} else {
					materialID = (materialRefMode == FbxGeometryElement::eDirect) ? 0 : materialArray->GetIndexArray().GetAt(0);
				}
			}
			LocalMaterialMap::dTreeNode* const matNode = localMaterialIndex.Find(materialID);
			dAssert(matNode);
			dMaterialNodeInfo* const material = (dMaterialNodeInfo*)ngdScene->GetInfoFromNode(matNode->GetInfo());
			materialIndex[i] = material->GetId();

			dAssert(usedMaterials.Find(matNode->GetInfo()));
			usedMaterials.Find(matNode->GetInfo())->GetInfo() += 1;

			faceIndexList[i] = polygonIndexCount;
			for (int j = 0; j < polygonIndexCount; j++) {
				vertexIndex[index] = fbxMesh->GetPolygonVertex(i, j);
				FbxVector4 n(0, 1, 0, 0);
				fbxMesh->GetPolygonVertexNormal(i, j, n);
				normalArray[index] = dVector(dFloat(n[0]), dFloat(n[1]), dFloat(n[2]), 0.0f);
				normalIndex[index] = index;

				FbxVector2 uv(0, 0);
				if (uvMapingMode == FbxGeometryElement::eByPolygonVertex) {
					int textIndex = (uvRefMode == FbxGeometryElement::eDirect) ? index : uvArray->GetIndexArray().GetAt(index);
					uv = uvArray->GetDirectArray().GetAt(textIndex);
				}
				uv0Index[index] = index;
				uv0Array[index] = dVector(dFloat(uv[0]), dFloat(uv[1]), 0.0f, 0.0f);

				index++;
				dAssert(index <= indexCount);
			}
		}


		// import skin if there is any
		NewtonMeshVertexWeightData::dgWeights* weightArray = NULL;
		int deformerCount = fbxMesh->GetDeformerCount(FbxDeformer::eSkin);
		if (deformerCount) {
			dAssert(deformerCount == 1);
			FbxSkin* const skin = (FbxSkin*)fbxMesh->GetDeformer(0, FbxDeformer::eSkin);
			dAssert(skin);

			// count the number of weights
			int skinVertexDataCount = 0;
			int clusterCount = skin->GetClusterCount();
			for (int i = 0; i < clusterCount; i++) {
				FbxCluster* cluster = skin->GetCluster(i);
				skinVertexDataCount += cluster->GetControlPointIndicesCount();
			}

			const int vertexCount = fbxMesh->GetControlPointsCount();
			dFloat* const tempWeight = new dFloat [16 * vertexCount];
			int* const tempBones = new int[16 * vertexCount];
			memset(tempWeight, 0, 16 * vertexCount * sizeof(dFloat));
			memset(tempBones, 0, 16 * vertexCount * sizeof(int));

			int maxBoneWeightCount = 0;
			for (int i = 0; i < clusterCount; i++) {
				FbxCluster* const cluster = skin->GetCluster(i);
				FbxNode* const fbxBone = cluster->GetLink();

				GlobalNodeMap::dTreeNode* const boneNode = nodeMap.Find(fbxBone);
				if (boneNode) {
					dScene::dTreeNode* const parentbone = boneNode->GetInfo();
					dScene::dTreeNode* const bone = ngdScene->FindChildByType(parentbone, dBoneNodeInfo::GetRttiType());
					dAssert(bone);

					dBoneNodeInfo* const boneInfo = (dBoneNodeInfo*)ngdScene->GetInfoFromNode(bone);
					
					const int* const boneVertexIndices = cluster->GetControlPointIndices();
					const double* const boneVertexWeights = cluster->GetControlPointWeights();
					// Iterate through all the vertices, which are affected by the bone
					int numBoneVertexIndices = cluster->GetControlPointIndicesCount();
					for (int j = 0; j < numBoneVertexIndices; j++) {
						int boneVertexIndex = boneVertexIndices[j];
						float boneWeight = (float)boneVertexWeights[j];

						for (int k = 0; k < 16; k++) {
							if (tempWeight[16 * boneVertexIndex + k] == 0.0f) {
								tempWeight[16 * boneVertexIndex + k] = boneWeight;
								tempBones[16 * boneVertexIndex + k] = boneInfo->GetId();
								maxBoneWeightCount = dMax(maxBoneWeightCount, k + 1);
								dAssert(maxBoneWeightCount <= 16);
								break;
							}
						}
					}
				}
			}

			for (int i = 0; i < vertexCount; i++) {
				dFloat* const weighPtr = &tempWeight[i * 16];
				int n = 0;
				for (int j = 0; j < 16; j++) {
					n += (weighPtr[j] > 0.0f) ? 1 : 0;
				}
				if (n > 4) {
					int* const bonePtr = &tempBones[i * 16];
					for (int j = 0; j < n - 1; j++) {
						for (int k = j + 1; k < n; k++) {
							if (weighPtr[k] > weighPtr[j]) {
								dSwap(bonePtr[k], bonePtr[j]);
								dSwap(weighPtr[k], weighPtr[j]);
							}
						}
					}
				}

				dFloat normalize = 0.0f;
				for (int j = 0; j < 4; j++) {
					normalize += weighPtr[j];
				}
				normalize = 1.0f / normalize;
				for (int j = 0; j < 4; j++) {
					weighPtr[j] *= normalize;
				}
			}

			weightArray = new NewtonMeshVertexWeightData::dgWeights[fbxMesh->GetControlPointsCount()];
			memset(weightArray, 0, vertexCount * sizeof(NewtonMeshVertexWeightData::dgWeights));

			for (int i = 0; i < vertexCount; i++) {
				const dFloat* const weighPtr = &tempWeight[i * 16];
				const int* const bonePtr = &tempBones[i * 16];
				for (int j = 0; j < 4; j++) {
					weightArray[i].m_weightBlends[j] = weighPtr[j];
					weightArray[i].m_controlIndex[j] = bonePtr[j];
				}
				dAssert((weightArray[i].m_weightBlends[3] > 0.0f) || (weightArray[i].m_controlIndex[3] == 0));
			}

			delete[] tempWeight;
			delete[] tempBones;
		}

		NewtonMeshVertexFormat format;
		NewtonMeshClearVertexFormat(&format);
		format.m_faceCount = faceCount;
		format.m_faceIndexCount = faceIndexList;
		format.m_faceMaterial = materialIndex;

		format.m_vertex.m_data = &vertexArray[0].m_x;
		format.m_vertex.m_indexList = vertexIndex;
		format.m_vertex.m_strideInBytes = sizeof(dBigVector);

		format.m_weight.m_data = weightArray;
		format.m_weight.m_strideInBytes = sizeof(NewtonMeshVertexWeightData::dgWeights);

		format.m_normal.m_data = &normalArray[0].m_x;
		format.m_normal.m_indexList = normalIndex;
		format.m_normal.m_strideInBytes = sizeof(dVector);

		format.m_uv0.m_data = &uv0Array[0].m_x;
		format.m_uv0.m_indexList = uv0Index;
		format.m_uv0.m_strideInBytes = sizeof(dVector);

		instance->BuildFromVertexListIndexList(&format);

		// some meshes has degenerated faces we must repair them to be legal manifold
		instance->RepairTJoints();

		instance->SmoothNormals(45.0f * dDegreeToRad);

		if (weightArray) {
			delete[] weightArray;
		}
		delete[] uv1Array;
		delete[] uv0Array;
		delete[] normalArray;
		delete[] vertexArray;
		delete[] uv1Index;
		delete[] uv0Index;
		delete[] normalIndex;
		delete[] vertexIndex;
		delete[] materialIndex;
		delete[] faceIndexList;
	}
}

void ImportSkeleton(dScene* const ngdScene, FbxScene* const fbxScene, FbxNode* const fbxNode, dScene::dTreeNode* const ngdNode, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials, int boneId)
{
	FbxSkeleton* fbxBone = (FbxSkeleton*)fbxNode->GetNodeAttribute();

	dScene::dTreeNode* const boneNode = ngdScene->CreateBoneNode(ngdNode);
	dBoneNodeInfo* const bone = (dBoneNodeInfo*)ngdScene->GetInfoFromNode(boneNode);
	bone->SetName(fbxNode->GetName());

	FbxSkeleton::EType type = fbxBone->GetSkeletonType();

	bone->SetLength (dFloat (fbxBone->GetLimbLengthDefaultValue()));
	if (type == FbxSkeleton::eRoot) {
		bone->SetType(dBoneNodeInfo::m_root);
	} else if (type == FbxSkeleton::eLimb || FbxSkeleton::eLimbNode) {
		bone->SetType(dBoneNodeInfo::m_limb);
	} else {
		bone->SetType(dBoneNodeInfo::m_effector);
	}
	bone->SetId(boneId);
}

int main(int argc, char** argv)
{
	if (argc != 2) {
		printf("fbxToNgd [fbx_file_name]\n");
		return 0;
	}

	FbxManager* fbxManager = NULL;
	FbxScene* fbxScene = NULL;

	if (!InitializeSdkObjects(fbxManager, fbxScene)) {
		return 0;
	}

	if (!fbxManager || !fbxScene) {
		FBXSDK_printf("failed to load file: %s\n", argv[1]);
	}
	FbxString filePath(argv[1]);
	bool lResult = LoadScene(fbxManager, fbxScene, filePath.Buffer());
	if (!lResult) {
		FBXSDK_printf("failed to load file: %s\n", argv[1]);
		return 0;
	}

	NewtonWorld* newton = NewtonCreate();
	dScene* ngdScene = new dScene(newton);

	if (ConvertToNgd(ngdScene, fbxScene)) {
		char name[1024];
		strcpy(name, argv[1]);
		_strlwr(name);
		char* ptr = strstr(name, ".fbx");
		ptr[0] = 0;
		strcat(name, ".ngd");
		ngdScene->Serialize(name);
	}

	delete ngdScene;
	NewtonDestroy(newton);

	fbxManager->Destroy();
	FBXSDK_printf("Conversion successful!\n");
	return 0;
}
