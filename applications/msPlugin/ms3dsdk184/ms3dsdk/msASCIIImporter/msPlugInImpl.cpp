#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "msPlugInImpl.h"
#include "msLib.h"
#include "DlgOptions.h"
#include "DlgMessage.h"



#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif



/////////////////////////////////////////////////////////////////////////////
// CMsPlugInApp

BEGIN_MESSAGE_MAP(CMsPlugInApp, CWinApp)
	//{{AFX_MSG_MAP(CMsPlugInApp)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
//

CMsPlugInApp::CMsPlugInApp()
{
}

/////////////////////////////////////////////////////////////////////////////
//

CMsPlugInApp theApp;



cMsPlugIn*
CreatePlugIn ()
{
    return new cPlugIn ();
}



cPlugIn::cPlugIn ()
{
    strcpy (szTitle, "MilkShape 3D ASCII...");
}



cPlugIn::~cPlugIn ()
{
}



int
cPlugIn::GetType ()
{
    return cMsPlugIn::eTypeImport;
}



const char*
cPlugIn::GetTitle ()
{
    return szTitle;
}


int
cPlugIn::Execute (msModel *pModel)
{
    if (!pModel)
        return -1;

    //
    // switch the module state for MFC Dlls
    //
    AFX_MANAGE_STATE(AfxGetStaticModuleState());

    //
    // ask for deletion
    //
    if (msModel_GetBoneCount (pModel) > 0)
    {
        int RC = ::AfxMessageBox (IDS_WARNING_MODEL_DELETE, MB_YESNOCANCEL);
        if (RC != IDYES)
            return 0;
    }

    //
    // options dialog
    //
    cDlgOptions dlgOptions (NULL);
    if (dlgOptions.DoModal () != IDOK)
        return 0;

    CString sPath = dlgOptions.GetPathName ();
    if (sPath.IsEmpty ())
        return 0;

    int nOptionFlags = dlgOptions.GetOptionFlags ();

    CString s;
    FILE *file = fopen ((LPCTSTR) sPath, "rt");
    if (!file)
    {
        s.Format(IDS_ERROR_OPENING_FILE_P, sPath);
        ::AfxMessageBox (s);
        return -1;
    }

    cDlgMessage dlgMessage (NULL);
    dlgMessage.Create (IDD_MESSAGE, NULL);
    dlgMessage.SetTitle ("Importing...");

    //
    // destroy the bones
    //
    if (pModel->pBones)
    {
        free (pModel->pBones);
        pModel->pBones = 0;
        pModel->nNumBones = 0;
        pModel->nNumAllocedBones = 0;
    }
    int i, j;

    // detach the vertices from the bones
    for (i = 0; i < msModel_GetMeshCount (pModel); i++)
    {
        msMesh *pMesh = msModel_GetMeshAt (pModel, i);
        for (j = 0; j < msMesh_GetVertexCount (pMesh); j++)
        {
            msVertex *pVertex = msMesh_GetVertexAt (pMesh, j);
            msVertex_SetBoneIndex (pVertex, -1);
        }
    }

    bool bError = false;
    char szLine[256];
    char szName[MS_MAX_NAME];
    int nFlags, nIndex;

    while (fgets (szLine, 256, file) != NULL  && !bError)
    {
        if (!strncmp (szLine, "//", 2))
            continue;

        int nFrame = 0;
        int nNumMeshes = 0;
        int nNumMaterials = 0;
        int nNumBones = 0;
		int nNumComments = 0;

		if (sscanf (szLine, "Frames: %d", &nFrame) == 1)
        {
            msModel_SetTotalFrames (pModel, nFrame);
        }

        else if (sscanf (szLine, "Frame: %d", &nFrame) == 1)
        {
            msModel_SetFrame (pModel, nFrame);
        }

         else if (sscanf (szLine, "Meshes: %d", &nNumMeshes) == 1)
        {
            dlgMessage.SetMessage ("Importing Meshes...");
            dlgMessage.SetRange (nNumMeshes);
            dlgMessage.SetPosition (0);

            for (i = 0; i < nNumMeshes && !bError; i++)
            {
                int nMesh = -1;
                msMesh *pMesh = NULL;

				if (nOptionFlags & eMeshes)
				{
					nMesh = msModel_AddMesh (pModel);
					pMesh = msModel_GetMeshAt (pModel, nMesh);
				}

                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }

                // mesh: name, flags, material index
                if (sscanf (szLine, "\"%[^\"]\" %d %d",szName, &nFlags, &nIndex) != 3)
                {
                    bError = true;
                    break;
                }

				if (nOptionFlags & eMeshes)
				{
					msMesh_SetName (pMesh, szName);
					msMesh_SetFlags (pMesh, nFlags);
					if (nOptionFlags & eMaterials)
						msMesh_SetMaterialIndex (pMesh, nIndex);
					else
						msMesh_SetMaterialIndex (pMesh, -1);
				}

                //
                // vertices
                //
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }

                int nNumVertices = 0;
                if (sscanf (szLine, "%d", &nNumVertices) != 1)
                {
                    bError = true;
                    break;
                }

                for (j = 0; j < nNumVertices; j++)
                {
                    if (!fgets (szLine, 256, file))
                    {
                        bError = true;
                        break;
                    }

                    msVec3 Vertex;
                    msVec2 uv;
                    if (sscanf (szLine, "%d %f %f %f %f %f %d",
                        &nFlags,
                        &Vertex[0], &Vertex[1], &Vertex[2],
                        &uv[0], &uv[1],
                        &nIndex
                        ) != 7)
                    {
                        bError = true;
                        break;
                    }

					if (nOptionFlags & eMeshes)
					{
						int nVertex = msMesh_AddVertex (pMesh);
						msVertex *pVertex = msMesh_GetVertexAt (pMesh, nVertex);
						msVertex_SetFlags (pVertex, nFlags);
						msVertex_SetVertex (pVertex, Vertex);
						msVertex_SetTexCoords (pVertex, uv);
						if (nOptionFlags & eBones)
							msVertex_SetBoneIndex (pVertex, nIndex);
						else
							msVertex_SetBoneIndex (pVertex, -1);
					}
                }

                //
                // normals
                //
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }

                int nNumNormals = 0;
                if (sscanf (szLine, "%d", &nNumNormals) != 1)
                {
                    bError = true;
                    break;
                }

                for (j = 0; j < nNumNormals; j++)
                {
                    if (!fgets (szLine, 256, file))
                    {
                        bError = true;
                        break;
                    }

                    msVec3 Normal;
                    if (sscanf (szLine, "%f %f %f", &Normal[0], &Normal[1], &Normal[2]) != 3)
                    {
                        bError = true;
                        break;
                    }

					if (nOptionFlags & eMeshes)
					{
						int nNormal = msMesh_AddVertexNormal (pMesh);
						msMesh_SetVertexNormalAt (pMesh, nNormal, Normal);
					}
                }
                //
                // triangles
                //
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }

                int nNumTriangles = 0;
                if (sscanf (szLine, "%d", &nNumTriangles) != 1)
                {
                    bError = true;
                    break;
                }

                for (j = 0; j < nNumTriangles; j++)
                {
                    if (!fgets (szLine, 256, file))
                    {
                        bError = true;
                        break;
                    }

                    int nTempIndices[3];
                    int nTempNormalIndices[3];
                    if (sscanf (szLine, "%d %d %d %d %d %d %d %d",
                        &nFlags,
                        &nTempIndices[0], &nTempIndices[1], &nTempIndices[2],
                        &nTempNormalIndices[0], &nTempNormalIndices[1], &nTempNormalIndices[2],
                        &nIndex
                        ) != 8)
                    {
                        bError = true;
                        break;
                    }

					if (nOptionFlags & eMeshes)
					{
						int nTriangle = msMesh_AddTriangle (pMesh);
						msTriangle *pTriangle = msMesh_GetTriangleAt (pMesh, nTriangle);
						msTriangle_SetFlags (pTriangle, nFlags);
						word nIndices[3] = { nTempIndices[0], nTempIndices[1], nTempIndices[2] };
						msTriangle_SetVertexIndices (pTriangle, nIndices);
						word nNormalIndices[3] = { nTempNormalIndices[0], nTempNormalIndices[1], nTempNormalIndices[2] };
						msTriangle_SetNormalIndices (pTriangle, nNormalIndices);
						msTriangle_SetSmoothingGroup (pTriangle, nIndex);
					}
                }

                dlgMessage.SetPosition (i + 1);
            }
        }

        //
        // materials
        //
        else if (sscanf (szLine, "Materials: %d", &nNumMaterials) == 1)
        {
            int i;
            char szName[MS_MAX_PATH];

            dlgMessage.SetMessage ("Importing Materials...");
            dlgMessage.SetRange (nNumMaterials);
            dlgMessage.SetPosition (0);

            for (i = 0; i < nNumMaterials && !bError; i++)
            {
                int nMaterial = -1;
                msMaterial *pMaterial = NULL;
				if (nOptionFlags & eMaterials)
				{
					nMaterial = msModel_AddMaterial (pModel);
					pMaterial = msModel_GetMaterialAt (pModel, nMaterial);
				}

                // name
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
                if (sscanf (szLine, "\"%[^\"]\"", szName) != 1)
                {
                    bError = true;
                    break;
                }

                // ambient
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
                msVec4 Ambient;
                if (sscanf (szLine, "%f %f %f %f", &Ambient[0], &Ambient[1], &Ambient[2], &Ambient[3]) != 4)
                {
                    bError = true;
                    break;
                }

                // diffuse
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
                msVec4 Diffuse;
                if (sscanf (szLine, "%f %f %f %f", &Diffuse[0], &Diffuse[1], &Diffuse[2], &Diffuse[3]) != 4)
                {
                    bError = true;
                    break;
                }

                // specular
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
                msVec4 Specular;
                if (sscanf (szLine, "%f %f %f %f", &Specular[0], &Specular[1], &Specular[2], &Specular[3]) != 4)
                {
                    bError = true;
                    break;
                }

                // emissive
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
                msVec4 Emissive;
                if (sscanf (szLine, "%f %f %f %f", &Emissive[0], &Emissive[1], &Emissive[2], &Emissive[3]) != 4)
                {
                    bError = true;
                    break;
                }
 
                // shininess
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
                float fShininess;
                if (sscanf (szLine, "%f", &fShininess) != 1)
                {
                    bError = true;
                    break;
                }

                // transparency
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
                float fTransparency;
                if (sscanf (szLine, "%f", &fTransparency) != 1)
                {
                    bError = true;
                    break;
                }

                // diffuse texture
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
				char szDiffuseTexture[MS_MAX_PATH];
				strcpy(szDiffuseTexture, "");
                sscanf (szLine, "\"%[^\"]\"", szDiffuseTexture);

                // alpha texture
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
				char szAlphaTexture[MS_MAX_PATH];
                strcpy (szAlphaTexture, "");
                sscanf (szLine, "\"%[^\"]\"", szAlphaTexture);

				if (nOptionFlags & eMaterials)
				{
					msMaterial_SetName (pMaterial, szName);
					msMaterial_SetAmbient (pMaterial, Ambient);
					msMaterial_SetDiffuse (pMaterial, Diffuse);
					msMaterial_SetSpecular (pMaterial, Specular);
					msMaterial_SetEmissive (pMaterial, Emissive);
					msMaterial_SetShininess (pMaterial, fShininess);
					if (nOptionFlags & eNoTransparency)
						msMaterial_SetTransparency (pMaterial, 1.0f);
					else
						msMaterial_SetTransparency (pMaterial, fTransparency);
					msMaterial_SetDiffuseTexture (pMaterial, szDiffuseTexture);
					msMaterial_SetAlphaTexture (pMaterial, szAlphaTexture);
				}

                dlgMessage.SetPosition (i + 1);
            }
        }

        //
        // bones
        //
        else if (sscanf (szLine, "Bones: %d", &nNumBones) == 1)
        {
            int i;
            char szName[MS_MAX_NAME];
            char szParentName[MS_MAX_NAME];

            dlgMessage.SetMessage ("Importing Bones...");
            dlgMessage.SetRange (nNumBones);
            dlgMessage.SetPosition (0);

            for (i = 0; i < nNumBones && !bError; i++)
            {
                int nBone = -1;
                msBone *pBone = NULL;
				if (nOptionFlags & eBones)
				{
					nBone = msModel_AddBone (pModel);
					pBone = msModel_GetBoneAt (pModel, nBone);
				}

                // name
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
                if (sscanf (szLine, "\"%[^\"]\"", szName) != 1)
                {
                    bError = true;
                    break;
                }

                // parent name
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
                strcpy (szParentName, "");
                sscanf (szLine, "\"%[^\"]\"", szParentName);

                // flags, position, rotation
                msVec3 Position, Rotation;
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
                if (sscanf (szLine, "%d %f %f %f %f %f %f",
                    &nFlags,
                    &Position[0], &Position[1], &Position[2],
                    &Rotation[0], &Rotation[1], &Rotation[2]) != 7)
                {
                    bError = true;
                    break;
                }
 				if (nOptionFlags & eBones)
				{
					msBone_SetName (pBone, szName);
					msBone_SetParentName (pBone, szParentName);
					msBone_SetFlags (pBone, nFlags);
					msBone_SetPosition (pBone, Position);
					msBone_SetRotation (pBone, Rotation);
				}

                float fTime;

                // position key count
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
                int nNumPositionKeys = 0;
                if (sscanf (szLine, "%d", &nNumPositionKeys) != 1)
                {
                    bError = true;
                    break;
                }

                for (j = 0; j < nNumPositionKeys; j++)
                {
                    if (!fgets (szLine, 256, file))
                    {
                        bError = true;
                        break;
                    }
                    if (sscanf (szLine, "%f %f %f %f", &fTime, &Position[0], &Position[1], &Position[2]) != 4)
                    {
                        bError = true;
                        break;
                    }

                    if (nOptionFlags & (eKeyFrames|eBones))
                        msBone_AddPositionKey (pBone, fTime, Position);
                }

                // rotation key count
                if (!fgets (szLine, 256, file))
                {
                    bError = true;
                    break;
                }
                int nNumRotationKeys = 0;
                if (sscanf (szLine, "%d", &nNumRotationKeys) != 1)
                {
                    bError = true;
                    break;
                }

                for (j = 0; j < nNumRotationKeys; j++)
                {
                    if (!fgets (szLine, 256, file))
                    {
                        bError = true;
                        break;
                    }
                    if (sscanf (szLine, "%f %f %f %f", &fTime, &Rotation[0], &Rotation[1], &Rotation[2]) != 4)
                    {
                        bError = true;
                        break;
                    }

                    if (nOptionFlags & (eKeyFrames|eBones))
                        msBone_AddRotationKey (pBone, fTime, Rotation);
                }

                dlgMessage.SetPosition (i + 1);
            }
        }

        //
        // GroupComments
        //
        else if (sscanf (szLine, "GroupComments: %d", &nNumComments) == 1)
        {
            dlgMessage.SetMessage ("Importing Group Comments...");
            dlgMessage.SetRange (nNumComments);
            dlgMessage.SetPosition (0);

            for (i = 0; i < nNumComments && !bError; i++)
            {
				int nIndex;
				char szComment[256];

				// group index
				if (!fgets (szLine, 256, file))
				{
					bError = true;
					break;
				}
				if (sscanf (szLine, "%d", &nIndex) != 1)
				{
					bError = true;
					break;
				}

				// comment
				if (!fgets (szLine, 256, file))
				{
					bError = true;
					break;
				}
				strcpy(szComment, szLine);
				szComment[strlen(szComment) - 1] = '\0';
				while (true)
				{
					char *p = strstr(szComment, "\\n");
					if (p)
					{
						p[0] = '\r';
						p[1] = '\n';
					}
					else
						break;
				}
				msMesh *pMesh = msModel_GetMeshAt(pModel, nIndex);
				if (pMesh)
					msMesh_SetComment(pMesh, szComment);

               dlgMessage.SetPosition (i + 1);
			}
		}

        //
        // MaterialComments
        //
        else if (sscanf (szLine, "MaterialComments: %d", &nNumComments) == 1)
        {
            dlgMessage.SetMessage ("Importing Material Comments...");
            dlgMessage.SetRange (nNumComments);
            dlgMessage.SetPosition (0);

            for (i = 0; i < nNumComments && !bError; i++)
            {
				int nIndex;
				char szComment[256];

				// group index
				if (!fgets (szLine, 256, file))
				{
					bError = true;
					break;
				}
				if (sscanf (szLine, "%d", &nIndex) != 1)
				{
					bError = true;
					break;
				}

				// comment
				if (!fgets (szLine, 256, file))
				{
					bError = true;
					break;
				}
				strcpy(szComment, szLine);
				szComment[strlen(szComment) - 1] = '\0';
				while (true)
				{
					char *p = strstr(szComment, "\\n");
					if (p)
					{
						p[0] = '\r';
						p[1] = '\n';
					}
					else
						break;
				}
				msMaterial *pMaterial = msModel_GetMaterialAt(pModel, nIndex);
				if (pMaterial)
					msMaterial_SetComment(pMaterial, szComment);

               dlgMessage.SetPosition (i + 1);
			}
		}

        //
        // BoneComments
        //
        else if (sscanf (szLine, "BoneComments: %d", &nNumComments) == 1)
        {
            dlgMessage.SetMessage ("Importing Bone Comments...");
            dlgMessage.SetRange (nNumComments);
            dlgMessage.SetPosition (0);

            for (i = 0; i < nNumComments && !bError; i++)
            {
				int nIndex;
				char szComment[256];

				// group index
				if (!fgets (szLine, 256, file))
				{
					bError = true;
					break;
				}
				if (sscanf (szLine, "%d", &nIndex) != 1)
				{
					bError = true;
					break;
				}

				// comment
				if (!fgets (szLine, 256, file))
				{
					bError = true;
					break;
				}
				strcpy(szComment, szLine);
				szComment[strlen(szComment) - 1] = '\0';
				while (true)
				{
					char *p = strstr(szComment, "\\n");
					if (p)
					{
						p[0] = '\r';
						p[1] = '\n';
					}
					else
						break;
				}
				msBone *pBone = msModel_GetBoneAt(pModel, nIndex);
				if (pBone)
					msBone_SetComment(pBone, szComment);

               dlgMessage.SetPosition (i + 1);
			}
		}

        //
        // ModelComment
        //
        else if (sscanf (szLine, "ModelComment: %d", &nNumComments) == 1)
        {
            dlgMessage.SetMessage ("Importing Model Comment...");
            dlgMessage.SetRange (nNumComments);
            dlgMessage.SetPosition (0);

            if (nNumComments > 0)
            {
				char szComment[256];

				// comment
				if (!fgets (szLine, 256, file))
				{
					bError = true;
					break;
				}
				strcpy(szComment, szLine);
				szComment[strlen(szComment) - 1] = '\0';
				while (true)
				{
					char *p = strstr(szComment, "\\n");
					if (p)
					{
						p[0] = '\r';
						p[1] = '\n';
					}
					else
						break;
				}
				msModel_SetComment(pModel, szComment);

               dlgMessage.SetPosition (i + 1);
			}
		}
    }

    fclose (file);

	if (bError)
	{
		AfxMessageBox("Error importing MilkShape 3D ASCII file.");
	}

    return 0;
}

