#include "stdafx.h"
#include "msPlugInImpl.h"
#include "msLib.h"



BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{
    switch (ul_reason_for_call)
	{
		case DLL_PROCESS_ATTACH:
		case DLL_THREAD_ATTACH:
		case DLL_THREAD_DETACH:
		case DLL_PROCESS_DETACH:
			break;
    }
    return TRUE;
}



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
    return cMsPlugIn::eTypeExport;
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
    // check, if we have something to export
    //
    if (msModel_GetMeshCount (pModel) == 0)
    {
        ::MessageBox (NULL, "The model is empty!  Nothing exported!", "MilkShape 3D ASCII Export", MB_OK | MB_ICONWARNING);
        return 0;
    }

    //
    // choose filename
    //
    OPENFILENAME ofn;
    memset (&ofn, 0, sizeof (OPENFILENAME));
    
    char szFile[MS_MAX_PATH];
    char szFileTitle[MS_MAX_PATH];
    char szDefExt[32] = "txt";
    char szFilter[128] = "MilkShape 3D ASCII Files (*.txt)\0*.txt\0All Files (*.*)\0*.*\0\0";
    szFile[0] = '\0';
    szFileTitle[0] = '\0';

    ofn.lStructSize = sizeof (OPENFILENAME);
    ofn.lpstrDefExt = szDefExt;
    ofn.lpstrFilter = szFilter;
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = MS_MAX_PATH;
    ofn.lpstrFileTitle = szFileTitle;
    ofn.nMaxFileTitle = MS_MAX_PATH;
    ofn.Flags = OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST;
    ofn.lpstrTitle = "Export MilkShape 3D ASCII";

    if (!::GetSaveFileName (&ofn))
        return 0;

    //
    // export
    //
    FILE *file = fopen (szFile, "wt");
    if (!file)
        return -1;

    int i, j;
    char szName[MS_MAX_NAME];

    fprintf (file, "// MilkShape 3D ASCII\n\n");
    fprintf (file, "Frames: %d\n", msModel_GetTotalFrames (pModel));
    fprintf (file, "Frame: %d\n\n", msModel_GetFrame (pModel));
    fprintf (file, "Meshes: %d\n", msModel_GetMeshCount (pModel));
    for (i = 0; i < msModel_GetMeshCount (pModel); i++)
    {
        msMesh *pMesh = msModel_GetMeshAt (pModel, i);
        msMesh_GetName (pMesh, szName, MS_MAX_NAME);
		if (strlen(szName) == 0)
			strcpy(szName, " ");

        fprintf (file, "\"%s\" %d %d\n", szName, msMesh_GetFlags (pMesh), msMesh_GetMaterialIndex (pMesh));

        //
        // vertices
        //
        fprintf (file, "%d\n", msMesh_GetVertexCount (pMesh));
        for (j = 0; j < msMesh_GetVertexCount (pMesh); j++)
        {
            msVertex *pVertex = msMesh_GetVertexAt (pMesh, j);
            msVec3 Vertex;
            msVec2 uv;

            msVertex_GetVertex (pVertex, Vertex);
            msVertex_GetTexCoords (pVertex, uv);

            fprintf (file, "%d %f %f %f %f %f %d\n",
                msVertex_GetFlags (pVertex),
                Vertex[0], Vertex[1], Vertex[2],
                uv[0], uv[1],
                msVertex_GetBoneIndex (pVertex)
                );
        }

        //
        // vertex normals
        //
        fprintf (file, "%d\n", msMesh_GetVertexNormalCount (pMesh));
        for (j = 0; j < msMesh_GetVertexNormalCount (pMesh); j++)
        {
            msVec3 Normal;
            msMesh_GetVertexNormalAt (pMesh, j, Normal);
            fprintf (file, "%f %f %f\n", Normal[0], Normal[1], Normal[2]);
        }

        //
        // triangles
        //
        fprintf (file, "%d\n", msMesh_GetTriangleCount (pMesh));
        for (j = 0; j < msMesh_GetTriangleCount (pMesh); j++)
        {
            msTriangle *pTriangle = msMesh_GetTriangleAt (pMesh, j);
            
            word nIndices[3];
            msTriangle_GetVertexIndices (pTriangle, nIndices);
            
            fprintf (file, "%d %d %d %d %d %d %d %d\n",
                msTriangle_GetFlags (pTriangle),
                nIndices[0], nIndices[1], nIndices[2],
                pTriangle->nNormalIndices[0], pTriangle->nNormalIndices[1], pTriangle->nNormalIndices[2],
                msTriangle_GetSmoothingGroup (pTriangle)
                );
        }
    }

    //
    // materials
    //
    fprintf (file, "\nMaterials: %d\n", msModel_GetMaterialCount (pModel));
    for (i = 0; i < msModel_GetMaterialCount (pModel); i++)
    {
        msMaterial *pMaterial = msModel_GetMaterialAt (pModel, i);
        msMaterial_GetName (pMaterial, szName, MS_MAX_NAME);
		if (strlen(szName) == 0)
			strcpy(szName, " ");
        fprintf (file, "\"%s\"\n", szName);

        msVec4 vec4;
        msMaterial_GetAmbient (pMaterial, vec4);
        fprintf (file, "%f %f %f %f\n", vec4[0], vec4[1], vec4[2], vec4[3]);
        msMaterial_GetDiffuse (pMaterial, vec4);
        fprintf (file, "%f %f %f %f\n", vec4[0], vec4[1], vec4[2], vec4[3]);
        msMaterial_GetSpecular (pMaterial, vec4);
        fprintf (file, "%f %f %f %f\n", vec4[0], vec4[1], vec4[2], vec4[3]);
        msMaterial_GetEmissive (pMaterial, vec4);
        fprintf (file, "%f %f %f %f\n", vec4[0], vec4[1], vec4[2], vec4[3]);
        fprintf (file, "%f\n", msMaterial_GetShininess (pMaterial));
        fprintf (file, "%f\n", msMaterial_GetTransparency (pMaterial));

        char szTexture[MS_MAX_PATH];
        msMaterial_GetDiffuseTexture (pMaterial, szTexture, MS_MAX_PATH);
        fprintf (file, "\"%s\"\n", szTexture);
        msMaterial_GetAlphaTexture (pMaterial, szTexture, MS_MAX_PATH);
        fprintf (file, "\"%s\"\n", szTexture);
    }

    //
    // bones
    //
    fprintf (file, "\nBones: %d\n", msModel_GetBoneCount (pModel));
    for (i = 0; i < msModel_GetBoneCount (pModel); i++)
    {
        msBone *pBone = msModel_GetBoneAt (pModel, i);
        msBone_GetName (pBone, szName, MS_MAX_NAME);
		if (strlen(szName) == 0)
			strcpy(szName, " ");
        fprintf (file, "\"%s\"\n", szName);
        msBone_GetParentName (pBone, szName, MS_MAX_NAME);
        fprintf (file, "\"%s\"\n", szName);
        msVec3 Position, Rotation;
        msBone_GetPosition (pBone, Position);
        msBone_GetRotation (pBone, Rotation);
        fprintf (file, "%d %f %f %f %f %f %f\n",
            msBone_GetFlags (pBone),
            Position[0], Position[1], Position[2],
            Rotation[0], Rotation[1], Rotation[2]
            );

        fprintf (file, "%d\n", msBone_GetPositionKeyCount (pBone));
        for (j = 0; j < msBone_GetPositionKeyCount (pBone); j++)
        {
            msPositionKey *pKey = msBone_GetPositionKeyAt (pBone, j);
            //fprintf (file, "Time: %f, Position Key: %f %f %f\n",
            fprintf (file, "%f %f %f %f\n",
                pKey->fTime, pKey->Position[0], pKey->Position[1], pKey->Position[2]
                );
        }

        fprintf (file, "%d\n", msBone_GetRotationKeyCount (pBone));
        for (j = 0; j < msBone_GetRotationKeyCount (pBone); j++)
        {
            msRotationKey *pKey = msBone_GetRotationKeyAt (pBone, j);
            //fprintf (file, "Time: %f, Rotation Key: %f %f %f\n",
            fprintf (file, "%f %f %f %f\n",
                pKey->fTime, pKey->Rotation[0], pKey->Rotation[1], pKey->Rotation[2]
                );
        }
    }

	unsigned int nNumComments;

	// group comments
	{
		nNumComments = 0;
		for (i = 0; i < msModel_GetMeshCount (pModel); i++)
		{
			msMesh *pMesh = msModel_GetMeshAt (pModel, i);
			int nCommentLength = msMesh_GetComment(pMesh, NULL, 0);
			if (nCommentLength > 0)
				++nNumComments;
		}

		fprintf(file, "GroupComments: %d\n", nNumComments);

		for (i = 0; i < msModel_GetMeshCount (pModel); i++)
		{
			msMesh *pMesh = msModel_GetMeshAt (pModel, i);
			int nCommentLength = msMesh_GetComment(pMesh, NULL, 0);
			if (nCommentLength > 0)
			{
				fprintf(file, "%d\n", i);
				char *pszComment = new char[nCommentLength + 1];
				msMesh_GetComment(pMesh, pszComment, nCommentLength);
				pszComment[nCommentLength] = '\0';
				while (true)
				{
					char * p = strstr(pszComment, "\r\n");
					if (p)
					{
						p[0] = '\\';
						p[1] = 'n';
					}
					else
						break;
				}
				fprintf(file, "%s\n", pszComment);
				delete[] pszComment;
			}
		}
	}

	// material comments
	{
		nNumComments = 0;
		for (i = 0; i < msModel_GetMaterialCount (pModel); i++)
		{
			msMaterial *pMaterial = msModel_GetMaterialAt (pModel, i);
			int nCommentLength = msMaterial_GetComment(pMaterial, NULL, 0);
			if (nCommentLength > 0)
				++nNumComments;
		}

		fprintf(file, "MaterialComments: %d\n", nNumComments);

		for (i = 0; i < msModel_GetMaterialCount (pModel); i++)
		{
			msMaterial *pMaterial = msModel_GetMaterialAt (pModel, i);
			int nCommentLength = msMaterial_GetComment(pMaterial, NULL, 0);
			if (nCommentLength > 0)
			{
				fprintf(file, "%d\n", i);
				char *pszComment = new char[nCommentLength + 1];
				msMaterial_GetComment(pMaterial, pszComment, nCommentLength);
				pszComment[nCommentLength] = '\0';
				while (true)
				{
					char * p = strstr(pszComment, "\r\n");
					if (p)
					{
						p[0] = '\\';
						p[1] = 'n';
					}
					else
						break;
				}
				fprintf(file, "%s\n", pszComment);
				delete[] pszComment;
			}
		}
	}

	// joint comments
	{
		nNumComments = 0;
		for (i = 0; i < msModel_GetBoneCount (pModel); i++)
		{
			msBone *pBone = msModel_GetBoneAt (pModel, i);
			int nCommentLength = msBone_GetComment(pBone, NULL, 0);
			if (nCommentLength > 0)
				++nNumComments;
		}

		fprintf(file, "BoneComments: %d\n", nNumComments);

		for (i = 0; i < msModel_GetBoneCount (pModel); i++)
		{
			msBone *pBone = msModel_GetBoneAt (pModel, i);
			int nCommentLength = msBone_GetComment(pBone, NULL, 0);
			if (nCommentLength > 0)
			{
				fprintf(file, "%d\n", i);
				char *pszComment = new char[nCommentLength + 1];
				msBone_GetComment(pBone, pszComment, nCommentLength);
				pszComment[nCommentLength] = '\0';
				while (true)
				{
					char * p = strstr(pszComment, "\r\n");
					if (p)
					{
						p[0] = '\\';
						p[1] = 'n';
					}
					else
						break;
				}
				fprintf(file, "%s\n", pszComment);
				delete[] pszComment;
			}
		}
	}

	// model comments
	{
		nNumComments = 0;
		int nCommentLength = msModel_GetComment(pModel, NULL, 0);
		if (nCommentLength > 0)
			nNumComments = 1;
		fprintf(file, "ModelComment: %d\n", nNumComments);

		if (nCommentLength > 0)
		{
			char *pszComment = new char[nCommentLength + 1];
			msModel_GetComment(pModel, pszComment, nCommentLength);
			pszComment[nCommentLength] = '\0';
			while (true)
			{
				char * p = strstr(pszComment, "\r\n");
				if (p)
				{
					p[0] = '\\';
					p[1] = 'n';
				}
				else
					break;
			}
			fprintf(file, "%s\n", pszComment);
			delete[] pszComment;
		}
	}

    fclose (file);

    // dont' forget to destroy the model
    msModel_Destroy (pModel);

    return 0;
}
