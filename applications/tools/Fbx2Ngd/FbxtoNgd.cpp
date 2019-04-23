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

#define ANIMATION_RESAMPLING	(1.0f/60.0f)

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
static bool LoadScene(FbxManager* pManager, FbxDocument* pScene, const char* const pFilename);
static void LoadHierarchy(dScene* const ngdScene, FbxScene* const fbxScene, GlobalNodeMap& nodeMap);
static bool ConvertToNgd(dScene* const ngdScene, FbxScene* const fbxScene, bool importMesh, bool importAnimations);
static void PopulateScene(dScene* const ngdScene, FbxScene* const fbxScene, bool importMesh, bool importAnimations);
static void ImportMeshNode(dScene* const ngdScene, FbxScene* const fbxScene, GlobalNodeMap& nodeMap, FbxNode* const fbxMeshNode, dScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials);
static void ImportMaterials(FbxScene* const fbxScene, dScene* const ngdScene, FbxNode* const fbxMeshNode, dScene::dTreeNode* const meshNode, GlobalMaterialMap& materialCache, LocalMaterialMap& localMaterilIndex, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials);
static void ImportSkinModifier(dScene* const ngdScene, FbxScene* const fbxScene, GlobalNodeMap& nodeMap, FbxNode* const fbxMeshNode, dScene::dTreeNode* const node, GlobalMeshMap& meshCache);
static void ImportTexture(FbxScene* const fbxScene, dScene* const ngdScene, FbxProperty pProperty, dScene::dTreeNode* const materialNode, GlobalTextureMap& textureCache);
static void ImportSkeleton(dScene* const ngdScene, FbxScene* const fbxScene, FbxNode* const fbxNode, dScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials, int boneId);
static void PrepareSkeleton(FbxScene* const fbxScene);
static dFloat CalculateAnimationPeriod(FbxScene* const fbxScene, FbxAnimLayer* const animLayer);
static void ImportAnimations(dScene* const ngdScene, FbxScene* const fbxScene, GlobalNodeMap& nodeMap);
static void RemoveReflexionMatrices(dScene* const ngdScene);


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

bool LoadScene(FbxManager* pManager, FbxDocument* pScene, const char* const pFilename)
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

bool ConvertToNgd(dScene* const ngdScene, FbxScene* const fbxScene, bool importMesh, bool importAnimations)
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
	axisMatrix = axisMatrix * dYawMatrix(dPi);
	convertMatrix = axisMatrix * convertMatrix;

	PopulateScene(ngdScene, fbxScene, importMesh, importAnimations);

	ngdScene->RemoveUnusedMaterials();
	ngdScene->BakeTransform(convertMatrix);
	ngdScene->FreezeScale();
	RemoveReflexionMatrices(ngdScene);

	return true;
}

void PopulateScene(dScene* const ngdScene, FbxScene* const fbxScene, bool importMesh, bool importAnimations)
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
	LoadHierarchy(ngdScene, fbxScene, nodeMap);

	if (importMesh) {
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
						ImportMeshNode(ngdScene, fbxScene, nodeMap, fbxNode, ngdNode, meshCache, materialCache, textureCache, usedMaterials);
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

					case FbxNodeAttribute::eSkeleton:
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
	}

	if (importAnimations) {
		ImportAnimations(ngdScene, fbxScene, nodeMap);
	}
}

void PrepareSkeleton(FbxScene* const fbxScene)
{
	int stack = 1;
	FbxNode* fbxNodes[32];
	fbxNodes[0] = fbxScene->GetRootNode();

	FbxAnimEvaluator* const evaluator = fbxScene->GetAnimationEvaluator();
	evaluator->Reset();

	while (stack) {
		stack--;
		FbxNode* const fbxNode = fbxNodes[stack];

		fbxNode->SetPivotState(FbxNode::eSourcePivot, FbxNode::ePivotActive);
		fbxNode->SetPivotState(FbxNode::eDestinationPivot, FbxNode::ePivotActive);

		EFbxRotationOrder RotationOrder;
		fbxNode->GetRotationOrder(FbxNode::eSourcePivot, RotationOrder);
		fbxNode->SetRotationOrder(FbxNode::eDestinationPivot, RotationOrder);

		dAssert(RotationOrder == FbxEuler::eEulerXYZ);
		fbxNode->ConvertPivotAnimationRecursive(NULL, FbxNode::eDestinationPivot, FbxTime::GetFrameRate(fbxScene->GetGlobalSettings().GetTimeMode()), false);

		for (int i = 0; i < fbxNode->GetChildCount(); i++) {
			fbxNodes[stack] = fbxNode->GetChild(i);
			stack++;
		}
	}
}

void RemoveReflexionMatrices(dScene* const ngdScene)
{
//	ngdScene->FreezeScale();
/*
	dList<dScene::dTreeNode*> nodeStack;
	dList<dMatrix> matrixStack;

	for (void* link = ngdScene->GetFirstChildLink(ngdScene->GetRootNode()); link; link = ngdScene->GetNextChildLink(ngdScene->GetRootNode(), link)) {
		dScene::dTreeNode* const node = ngdScene->GetNodeFromLink(link);
		dNodeInfo* const nodeInfo = ngdScene->GetInfoFromNode(node);
		if (nodeInfo->IsType(dSceneNodeInfo::GetRttiType())) {
			nodeStack.Append(node);
			matrixStack.Append(dGetIdentityMatrix());
		}
	}

	while (nodeStack.GetCount()) {
		dScene::dTreeNode* const rootNode = nodeStack.GetLast()->GetInfo();
		dMatrix parentMatrix(matrixStack.GetLast()->GetInfo());

		nodeStack.Remove(nodeStack.GetLast());
		matrixStack.Remove(matrixStack.GetLast());

		dSceneNodeInfo* const sceneNodeInfo = (dSceneNodeInfo*)ngdScene->GetInfoFromNode(rootNode);
		dAssert(sceneNodeInfo->IsType(dSceneNodeInfo::GetRttiType()));
		dMatrix transform(sceneNodeInfo->GetTransform());

		dFloat det = transform[0].DotProduct3(transform[1].CrossProduct(transform[2]));
		if (det < 0.0f) {
			dAssert(0);
		}

		for (void* link = ngdScene->GetFirstChildLink(rootNode); link; link = ngdScene->GetNextChildLink(rootNode, link)) {
			dScene::dTreeNode* const node = ngdScene->GetNodeFromLink(link);
			dNodeInfo* const nodeInfo = ngdScene->GetInfoFromNode(node);

			if (nodeInfo->IsType(dSceneNodeInfo::GetRttiType())) {
				nodeStack.Append(node);
				matrixStack.Append(dGetIdentityMatrix());
			}
		}
	}
*/

	dMatrix reflexionMatrix(dGetIdentityMatrix());
	reflexionMatrix[2] = reflexionMatrix[2].Scale(-1.0f);
	dScene::Iterator iter(*ngdScene);
	for (iter.Begin(); iter; iter++) {
		dScene::dTreeNode* const node = iter.GetNode();
		dNodeInfo* const nodeInfo = ngdScene->GetInfoFromNode(node);
		if (nodeInfo->IsType(dMeshNodeInfo::GetRttiType())) {
			dScene::dTreeNode* const parentNode = ngdScene->FindParentByType(node, dSceneNodeInfo::GetRttiType());
			dAssert(parentNode);
			dSceneNodeInfo* const sceneNodeInfo = (dSceneNodeInfo*)ngdScene->GetInfoFromNode(parentNode);
			dAssert(sceneNodeInfo->IsType(dSceneNodeInfo::GetRttiType()));
			dMatrix matrix(sceneNodeInfo->GetGeometryTransform());
			dFloat det = matrix[0].DotProduct3(matrix[1].CrossProduct(matrix[2]));
			if (det < 0.0f) {
				matrix = reflexionMatrix * matrix;
				sceneNodeInfo->SetGeometryTransform(matrix);
				nodeInfo->BakeTransform(reflexionMatrix);

				dMeshNodeInfo* const meshNodeInfo = (dMeshNodeInfo*)nodeInfo;
				NewtonMeshFlipWinding(meshNodeInfo->GetMesh());
			}
		}
	}
}

dFloat CalculateAnimationPeriod(FbxScene* const fbxScene, FbxAnimLayer* const animLayer)
{
	FbxNode* const fbxRootnode = fbxScene->GetRootNode();

	int stack = 1;
	FbxNode* fbxNodes[32];
	fbxNodes[0] = fbxRootnode;

	dFloat t0 = 1.0e10f;
	dFloat t1 = -1.0e10f;
	FbxAnimCurve* curvexArray[6];
	while (stack) {
		stack--;
		FbxNode* const fbxNode = fbxNodes[stack];

		curvexArray[0] = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X);;
		curvexArray[1] = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y);;
		curvexArray[2] = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z);;
		curvexArray[3] = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X);;
		curvexArray[4] = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y);;
		curvexArray[5] = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z);;

		for (int i = 0; i < 6; i++) {
			FbxTimeSpan interval;
			if (curvexArray[i]) {
				curvexArray[i]->GetTimeInterval(interval);
				t0 = dFloat(dMin(t0, dFloat(interval.GetStart().GetSecondDouble())));
				t1 = dFloat(dMax(t1, dFloat(interval.GetStop().GetSecondDouble())));
				}
			}

		for (int i = 0; i < fbxNode->GetChildCount(); i++) {
			fbxNodes[stack] = fbxNode->GetChild(i);
			stack++;
		}
	}
	return t1 - t0;
}


void ImportAnimations(dScene* const ngdScene, FbxScene* const fbxScene, GlobalNodeMap& nodeMap)
{
	const int animationCount = fbxScene->GetSrcObjectCount<FbxAnimStack>();

	const int nodeCount = fbxScene->GetNodeCount();
	for (int i = 0; i < animationCount; i++) {
		FbxAnimStack* const animStack = fbxScene->GetSrcObject<FbxAnimStack>(i);
		const char* const animStackName = animStack->GetName();

		fbxScene->SetCurrentAnimationStack(animStack);
		FbxTakeInfo* const takeInfo = fbxScene->GetTakeInfo(animStackName);

		dScene::dTreeNode* const animationTakeNode = ngdScene->CreateAnimationTake();
		dAnimationTake* const animationTake = (dAnimationTake*)ngdScene->GetInfoFromNode(animationTakeNode);
		animationTake->SetName(animStackName);

		FbxTime start (takeInfo->mLocalTimeSpan.GetStart());
		FbxTime end (takeInfo->mLocalTimeSpan.GetStop());

		dFloat t0 = dFloat(start.GetSecondDouble());
		dFloat t1 = dFloat(end.GetSecondDouble());
		dFloat period = t1 - t0;
		animationTake->SetPeriod(period);

		const FbxLongLong firstFrameIndex = start.GetFrameCount(FbxTime::eDefaultMode);
		const FbxLongLong lastFrameIndex = end.GetFrameCount(FbxTime::eDefaultMode);

		dList <ImportStackData> nodeStack;
		FbxNode* const rootNode = fbxScene->GetRootNode();
		if (rootNode) {
			int count = rootNode->GetChildCount();
			for (int i = 0; i < count; i++) {
				nodeStack.Append(ImportStackData(dGetIdentityMatrix(), rootNode->GetChild(count - i - 1), NULL));
			}
		}

		while (nodeStack.GetCount()) {
			ImportStackData data(nodeStack.GetLast()->GetInfo());
			nodeStack.Remove(nodeStack.GetLast());

			dScene::dTreeNode* const ngdNode = nodeMap.Find(data.m_fbxNode)->GetInfo();
			dSceneNodeInfo* const ngdInfo = (dSceneNodeInfo*)ngdScene->GetInfoFromNode(ngdNode);

			dScene::dTreeNode* const trackNode = ngdScene->CreateAnimationTrack(animationTakeNode);
			dAnimationTrack* const animationTrack = (dAnimationTrack*)ngdScene->GetInfoFromNode(trackNode);

			animationTrack->SetName(ngdInfo->GetName());
			ngdScene->AddReference(ngdNode, trackNode);

			FbxLongLong j = firstFrameIndex;
			do {
				FbxTime fbxTime;
				fbxTime.SetFrame(j, FbxTime::eDefaultMode);
				dMatrix localMatrix(data.m_fbxNode->EvaluateLocalTransform(fbxTime));

				dFloat t = dFloat(fbxTime.GetSecondDouble());
				animationTrack->AddKeyframe(t, localMatrix);

				j++;
			} while (j <= lastFrameIndex);

			animationTrack->OptimizeCurves();

			int count = data.m_fbxNode->GetChildCount();
			for (int i = 0; i < count; i++) {
				nodeStack.Append(ImportStackData(dGetIdentityMatrix(), data.m_fbxNode->GetChild(count - i - 1), NULL));
			}
		}
	}
}

void ImportTexture(FbxScene* const fbxScene, dScene* const ngdScene, FbxProperty pProperty, dScene::dTreeNode* const materialNode, GlobalTextureMap& textureCache)
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
				ImportTexture(fbxScene, ngdScene, textureProperty, materialNode, textureCache);
			}
		}
	}
}

void ImportSkinModifier(dScene* const ngdScene, FbxScene* const fbxScene, GlobalNodeMap& nodeMap, FbxNode* const fbxMeshNode, dScene::dTreeNode* const ngdNode, GlobalMeshMap& meshCache)
{
	FbxMesh* const fbxMesh = fbxMeshNode->GetMesh();
	int deformerCount = fbxMesh->GetDeformerCount(FbxDeformer::eSkin);
	if (!deformerCount) {
		return;
	}

	dAssert(deformerCount == 1);
	FbxSkin* const skin = (FbxSkin*)fbxMesh->GetDeformer(0, FbxDeformer::eSkin);
	dAssert(skin);
	dAssert(meshCache.Find(fbxMesh));
	dScene::dTreeNode* const meshNode = meshCache.Find(fbxMesh)->GetInfo();

	const int clusterCount = skin->GetClusterCount();
	dTree <FbxCluster*, FbxNode*> clusterBoneMap;
	for (int i = 0; i < clusterCount; i++) {
		FbxCluster* const cluster = skin->GetCluster(i);
		FbxNode* const fbxBone = cluster->GetLink();
		clusterBoneMap.Insert(cluster, fbxBone);
	}
	
	dMeshNodeInfo* const ngdMeshInfo = (dMeshNodeInfo*)ngdScene->GetInfoFromNode(meshNode);
	for (int i = 0; i < clusterCount; i++) {
		FbxCluster* const cluster = skin->GetCluster(i);
		FbxNode* const fbxBone = cluster->GetLink();
		GlobalNodeMap::dTreeNode* const boneNode = nodeMap.Find(fbxBone);
		if (boneNode) {
			char skinName[256];

			dScene::dTreeNode* const ngdNode = boneNode->GetInfo();
			sprintf(skinName, "%s_skinCluster", fbxBone->GetName());
			dScene::dTreeNode* const skinNode = ngdScene->CreateSkinModifierNode(ngdNode);
			ngdScene->AddReference(meshNode, skinNode);

			dGeometryNodeSkinClusterInfo* const info = (dGeometryNodeSkinClusterInfo*)ngdScene->GetInfoFromNode(skinNode);
			info->SetName(skinName);
			
			dMatrix parentBoneMatrix(dGetIdentityMatrix());
			if (clusterBoneMap.Find(fbxBone->GetParent())) {
				FbxAMatrix parentTransformMatrix;
				FbxCluster* const parentCluster = clusterBoneMap.Find(fbxBone->GetParent())->GetInfo();
				parentCluster->GetTransformLinkMatrix(parentTransformMatrix);
				parentBoneMatrix = dMatrix(parentTransformMatrix);
			}
			FbxAMatrix transformMatrix;
			cluster->GetTransformLinkMatrix(transformMatrix);
			dMatrix boneMatrix(transformMatrix);

//dString xxx (fbxBone->GetName());
//if (xxx == "mixamorig:Hips")
//dString xxx1(fbxBone->GetName());

			//ngdCluster.m_basePoseMatrix = ngdSceneNodeInfo->GetTransform();
			//dMatrix xxxx (ngdSceneNodeInfo->GetTransform());
			info->m_basePoseMatrix = boneMatrix * parentBoneMatrix.Inverse4x4();

			const int* const boneVertexIndices = cluster->GetControlPointIndices();
			const double* const boneVertexWeights = cluster->GetControlPointWeights();
			const int numBoneVertexIndices = cluster->GetControlPointIndicesCount();
			dAssert(numBoneVertexIndices);

			info->m_vertexIndex.Resize(numBoneVertexIndices);
			info->m_vertexWeight.Resize(numBoneVertexIndices);
			for (int j = 0; j < numBoneVertexIndices; j++) {
				int boneVertexIndex = boneVertexIndices[j];
				dFloat boneWeight = (dFloat)boneVertexWeights[j];
				info->m_vertexWeight[j] = boneWeight;
				info->m_vertexIndex[j] = boneVertexIndex;
			}
		}
	}
}

void ImportMeshNode(dScene* const ngdScene, FbxScene* const fbxScene, GlobalNodeMap& nodeMap, FbxNode* const fbxMeshNode, dScene::dTreeNode* const ngdNode, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials)
{
	GlobalMeshMap::dTreeNode* instanceNode = meshCache.Find(fbxMeshNode->GetMesh());
	if (instanceNode) {
		dScene::dTreeNode* const meshInstance = instanceNode->GetInfo();
		ngdScene->AddReference(ngdNode, meshInstance);
	} else {
		FbxMesh* const fbxMesh = fbxMeshNode->GetMesh();
		dScene::dTreeNode* const meshNode = ngdScene->CreateMeshNode(ngdNode);
		meshCache.Insert(meshNode, fbxMesh);

		dMeshNodeInfo* const ngdMeshInfo = (dMeshNodeInfo*)ngdScene->GetInfoFromNode(meshNode);
		dSceneNodeInfo* const ngdNodeInfo = (dSceneNodeInfo*)ngdScene->GetInfoFromNode(ngdNode);

		char name[256];
		sprintf(name, "%s_mesh", fbxMeshNode->GetName());
		ngdMeshInfo->SetName(name);

		FbxVector4 fbxPivotScaling(fbxMeshNode->GetGeometricScaling(FbxNode::eSourcePivot));
		FbxVector4 fbxPivotRotation(fbxMeshNode->GetGeometricRotation(FbxNode::eSourcePivot));
		FbxVector4 fbxPivotTranslation(fbxMeshNode->GetGeometricTranslation(FbxNode::eSourcePivot));
		dVector pivotTranslation(dFloat(fbxPivotTranslation[0]), dFloat(fbxPivotTranslation[1]), dFloat(fbxPivotTranslation[2]), 1.0f);
		dVector pivotRotation(dFloat(fbxPivotRotation[0] * dDegreeToRad), dFloat(fbxPivotRotation[1] * dDegreeToRad), dFloat(fbxPivotRotation[2] * dDegreeToRad), 0.0f);
		dMatrix pivotScale(dGetIdentityMatrix());
		pivotScale[0][0] = dFloat(fbxPivotScaling[0]);
		pivotScale[1][1] = dFloat(fbxPivotScaling[1]);
		pivotScale[2][2] = dFloat(fbxPivotScaling[2]);
		dMatrix pivotMatrix(pivotScale * dMatrix(pivotRotation[0], pivotRotation[1], pivotRotation[2], pivotTranslation));
		ngdNodeInfo->SetGeometryTransform(pivotMatrix);

		LocalMaterialMap localMaterialIndex;
		ImportMaterials(fbxScene, ngdScene, fbxMeshNode, meshNode, materialCache, localMaterialIndex, textureCache, usedMaterials);

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
		const char* const uvLayerName = uvArray ? uvArray->GetName() : NULL;

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
				bool pUnmapped = false;
				FbxVector4 n(0.0, 1.0, 0.0, 0.0);
				FbxVector2 uv(0.0, 0.0);
				
				fbxMesh->GetPolygonVertexNormal(i, j, n);
				fbxMesh->GetPolygonVertexUV(i, j, uvLayerName, uv, pUnmapped);

				vertexIndex[index] = fbxMesh->GetPolygonVertex(i, j);
				uv0Array[index] = dVector(dFloat(uv[0]), dFloat(uv[1]), 0.0f, 0.0f);
				normalArray[index] = dVector(dFloat(n[0]), dFloat(n[1]), dFloat(n[2]), dFloat(0.0f));

				uv0Index[index] = index;
				normalIndex[index] = index;

				index++;
				dAssert(index <= indexCount);
			}
		}

		NewtonMeshVertexFormat format;
		NewtonMeshClearVertexFormat(&format);
		format.m_faceCount = faceCount;
		format.m_faceIndexCount = faceIndexList;
		format.m_faceMaterial = materialIndex;

		format.m_vertex.m_data = &vertexArray[0].m_x;
		format.m_vertex.m_indexList = vertexIndex;
		format.m_vertex.m_strideInBytes = sizeof(dBigVector);

		format.m_normal.m_data = &normalArray[0].m_x;
		format.m_normal.m_indexList = normalIndex;
		format.m_normal.m_strideInBytes = sizeof(dVector);

		format.m_uv0.m_data = &uv0Array[0].m_x;
		format.m_uv0.m_indexList = uv0Index;
		format.m_uv0.m_strideInBytes = sizeof(dVector);

		ngdMeshInfo->BuildFromVertexListIndexList(&format);

		// some meshes has degenerated faces we must repair them to be legal manifold
		ngdMeshInfo->RepairTJoints();
//		ngdMeshInfo->SmoothNormals(45.0f * dDegreeToRad);

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

		ImportSkinModifier(ngdScene, fbxScene, nodeMap, fbxMeshNode, ngdNode, meshCache);
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

void LoadHierarchy(dScene* const ngdScene, FbxScene* const fbxScene, GlobalNodeMap& nodeMap)
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

		FbxNode* const fbxNode = data.m_fbxNode;
		dMatrix localMatrix(fbxNode->EvaluateLocalTransform());

		info->SetName(fbxNode->GetName());
		info->SetTransform(localMatrix);
		nodeMap.Insert(node, fbxNode);
		dAssert (nodeMap.Find(fbxNode));

		int count = fbxNode->GetChildCount();
		for (int i = 0; i < count; i++) {
			nodeStack.Append(ImportStackData(dGetIdentityMatrix(), fbxNode->GetChild(i), node));
		}
	}
}

void ImportAnimationLayer(dScene* const ngdScene, FbxScene* const fbxScene, GlobalNodeMap& nodeMap, FbxAnimLayer* const animLayer, dScene::dTreeNode* const animationTakeNode)
{
	dFloat period = CalculateAnimationPeriod(fbxScene, animLayer);

	dAnimationTake* const animationTake = (dAnimationTake*)ngdScene->GetInfoFromNode(animationTakeNode);
	animationTake->SetPeriod(period);

	FbxAnimEvaluator* const evaluator = fbxScene->GetAnimationEvaluator();
	evaluator->Reset();

	dList <ImportStackData> nodeStack;
	FbxNode* const rootNode = fbxScene->GetRootNode();
	if (rootNode) {
		int count = rootNode->GetChildCount();
		for (int i = 0; i < count; i++) {
			nodeStack.Append(ImportStackData(dGetIdentityMatrix(), rootNode->GetChild(count - i - 1), NULL));
		}
	}

	while (nodeStack.GetCount()) {
		ImportStackData data(nodeStack.GetLast()->GetInfo());
		nodeStack.Remove(nodeStack.GetLast());

		dScene::dTreeNode* const trackNode = ngdScene->CreateAnimationTrack(animationTakeNode);
		dAnimationTrack* const animationTrack = (dAnimationTrack*)ngdScene->GetInfoFromNode(trackNode);
		dScene::dTreeNode* const ngdNode = nodeMap.Find(data.m_fbxNode)->GetInfo();
		dSceneNodeInfo* const ngdInfo = (dSceneNodeInfo*)ngdScene->GetInfoFromNode(ngdNode);
		animationTrack->SetName(ngdInfo->GetName());
		ngdScene->AddReference(ngdNode, trackNode);

		dFloat timeAcc = 0.0f;
		do {
			FbxTime fbxTime;
			fbxTime.SetSecondDouble(timeAcc);
			// not fact to back this but function does not seems to work 
			//dMatrix localMatrix(evaluator->GetNodeLocalTransform(data.m_fbxNode));
			dMatrix parentMatrix(data.m_fbxNode->GetParent()->EvaluateGlobalTransform(fbxTime));
			dMatrix nodeMatrix(data.m_fbxNode->EvaluateGlobalTransform(fbxTime));
			dMatrix localMatrix(nodeMatrix * parentMatrix.Inverse4x4());

			animationTrack->AddKeyframe(timeAcc, localMatrix);
			timeAcc += ANIMATION_RESAMPLING;
		} while (timeAcc < period);

		animationTrack->OptimizeCurves();

		int count = data.m_fbxNode->GetChildCount();
		for (int i = 0; i < count; i++) {
			nodeStack.Append(ImportStackData(dGetIdentityMatrix(), data.m_fbxNode->GetChild(count - i - 1), NULL));
		}
	}
}

int main(int argc, char** argv)
{
	bool importMesh = true;
	bool importAnimations = true;
	const char* name = NULL;
	for (int i = 1; i < argc; i ++) {
		if (argv[i][0] == '-') {
			if (argv[i][1] == 'm') {
				importMesh = true;
				importAnimations = false;
			} else if (argv[i][1] == 'a') {
				importMesh = false;
				importAnimations = true;
			}
		} else {
			name = argv[i];
		}
	}

	if (!name) {
		printf("fbxToNgd [fbx_file_name] -m -a\n");
		printf("[fbx_file_name] = fbx file name\n");
		printf("-m = export mesh only\n");
		printf("-a = export animation only\n");
	}

	FbxScene* fbxScene = NULL;
	FbxManager* fbxManager = NULL;
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

	NewtonWorld* const newton = NewtonCreate();
	dScene* const ngdScene = new dScene(newton);

	if (ConvertToNgd(ngdScene, fbxScene, importMesh, importAnimations)) {
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


