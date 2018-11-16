// FbxtoNgd.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#ifdef IOS_REF
#undef  IOS_REF
#define IOS_REF (*(pManager->GetIOSettings()))
#endif

#define DEFUALT_MATERIAL_ID -1

class ImportStackData
{
	public:
	ImportStackData(const dMatrix& parentMatrix, FbxNode* const fbxNode, dScene::dTreeNode* parentNode)
		:m_parentMatrix(parentMatrix)
		, m_fbxNode(fbxNode)
		, m_parentNode(parentNode)
	{
	}
	dMatrix m_parentMatrix;
	FbxNode* m_fbxNode;
	dScene::dTreeNode* m_parentNode;
};


class GlobalNodeMap: public dTree<dScene::dTreeNode*, FbxNode*>
{
};

class UsedMaterials : public dTree<int, dScene::dTreeNode*>
{
};


static bool ConvertToNgd(dScene* const ngdScene, FbxScene* fbxScene);
static void PopulateScene(dScene* const ngdScene, FbxScene* const fbxScene);
static void LoadHierarchy(FbxScene* const fbxScene, dScene* const ngdScene, GlobalNodeMap& nodeMap);

static int InitializeSdkObjects(FbxManager*& pManager, FbxScene*& pScene)
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


static bool LoadScene(FbxManager* pManager, FbxDocument* pScene, const char* pFilename)
{
	int lFileMajor, lFileMinor, lFileRevision;
	int lSDKMajor, lSDKMinor, lSDKRevision;
	//int lFileFormat = -1;
	int i, lAnimStackCount;
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

		// From this point, it is possible to access animation stack information without
		// the expense of loading the entire file.

		FBXSDK_printf("Animation Stack Information\n");

		lAnimStackCount = lImporter->GetAnimStackCount();

		FBXSDK_printf("    Number of Animation Stacks: %d\n", lAnimStackCount);
		FBXSDK_printf("    Current Animation Stack: \"%s\"\n", lImporter->GetActiveAnimStackName().Buffer());
		FBXSDK_printf("\n");

		for (i = 0; i < lAnimStackCount; i++)
		{
			FbxTakeInfo* lTakeInfo = lImporter->GetTakeInfo(i);

			FBXSDK_printf("    Animation Stack %d\n", i);
			FBXSDK_printf("         Name: \"%s\"\n", lTakeInfo->mName.Buffer());
			FBXSDK_printf("         Description: \"%s\"\n", lTakeInfo->mDescription.Buffer());

			// Change the value of the import name if the animation stack should be imported 
			// under a different name.
			FBXSDK_printf("         Import Name: \"%s\"\n", lTakeInfo->mImportName.Buffer());

			// Set the value of the import state to false if the animation stack should be not
			// be imported. 
			FBXSDK_printf("         Import State: %s\n", lTakeInfo->mSelect ? "true" : "false");
			FBXSDK_printf("\n");
		}

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
	}

	NewtonWorld* newton = NewtonCreate();
	dScene* ngdScene = new dScene(newton);

	if (ConvertToNgd(ngdScene, fbxScene)) {

	}

	delete ngdScene;
	NewtonDestroy(newton);

	fbxManager->Destroy();
	FBXSDK_printf("Conversion successful!\n");
    return 0;
}


static bool ConvertToNgd(dScene* const ngdScene, FbxScene* fbxScene)
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

	return true;
}


void PopulateScene(dScene* const ngdScene, FbxScene* const fbxScene)
{
	GlobalNodeMap nodeMap;
//	GlobalMeshMap meshCache;
//	GlobalTextureMap textureCache;
//	GlobalMaterialMap materialCache;
	UsedMaterials usedMaterials;

	dScene::dTreeNode* const defaulMaterialNode = ngdScene->CreateMaterialNode(DEFUALT_MATERIAL_ID);
	dMaterialNodeInfo* const defaulMaterial = (dMaterialNodeInfo*)ngdScene->GetInfoFromNode(defaulMaterialNode);
	defaulMaterial->SetName("default_material");
	usedMaterials.Insert(0, defaulMaterialNode);

	int m_materialId = 0;
	LoadHierarchy(fbxScene, ngdScene, nodeMap);

	GlobalNodeMap::Iterator iter(nodeMap);
	for (iter.Begin(); iter; iter++) {
		FbxNode* const fbxNode = iter.GetKey();
		dScene::dTreeNode* const node = iter.GetNode()->GetInfo();
		FbxNodeAttribute* const attribute = fbxNode->GetNodeAttribute();

		if (attribute) {
			FbxNodeAttribute::EType attributeType = attribute->GetAttributeType();

			switch (attributeType)
			{
				case FbxNodeAttribute::eMesh:
				{
					dAssert(0);
//					ImportMeshNode(fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials, nodeMap);
					break;
				}

				case FbxNodeAttribute::eSkeleton:
				{
					dAssert(0);
					//ImportSkeleton(fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials);
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

/*
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
*/
}


static void LoadHierarchy(FbxScene* const fbxScene, dScene* const ngdScene, GlobalNodeMap& nodeMap)
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
