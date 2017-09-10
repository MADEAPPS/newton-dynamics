/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#include "Common.h"

dMatrix GetMatrixFromMaxMatrix (const Matrix3& pivot)
{
	dMatrix matrix (GetIdentityMatrix());
	for (int i = 0; i < 4; i ++) {
		Point3 p (pivot.GetRow(i));
		for (int j = 0; j < 3; j ++) {
			matrix[i][j] = p[j];
		}
	}
	return matrix;
}

Matrix3 GetMatrixFromdMatrix (const dMatrix& matrix)
{
	Matrix3 maxMatrix;
	maxMatrix.SetRow (0, *((Point3*) &matrix[0]));
	maxMatrix.SetRow (1, *((Point3*) &matrix[1]));
	maxMatrix.SetRow (2, *((Point3*) &matrix[2]));
	maxMatrix.SetRow (3, *((Point3*) &matrix[3]));
	return maxMatrix;
}


void GetNodeName (INode* node, char* name)
{
	sprintf (name, "%s", node->GetName());
	for (int i = 0; name[i]; i ++) {
		if (name[i] == ' ') {
			name[i] = '_';
		}
	}
}

int FindFilePath (const char *name, const char *path,  char *fullPathName)
{
	intptr_t handle;
	FILE *tmpFile;
	const char *ptr;
	char tmpPath[1024];
	_finddata_t data;

	ptr = strrchr (name, '\\');
	if (!ptr) {
		ptr = name;
	} else {
		ptr ++;
	}
	sprintf (fullPathName, "%s\\%s", path, ptr);

	tmpFile = fopen (fullPathName, "r");
	if (tmpFile) {
		fclose (tmpFile);
		return 1;
	}

	sprintf (fullPathName, "%s\\*.", path);
	handle = _findfirst (fullPathName, &data);
	if (handle != -1) {
		do {
			if ((data.attrib & _A_SUBDIR) && (data.name[0] != '.')) {
				sprintf (tmpPath, "%s\\%s", path, data.name);
				if (FindFilePath (name, tmpPath, fullPathName)) {
					_findclose (handle);
					return 1;
				}
			}
		} while (_findnext (handle, &data) == 0);
		_findclose (handle);
	}
	return 0;
}

void GetNameFromPath (const char* pathName, char* name)
{
	const char* ptr;
	ptr = strrchr (pathName, '/');
	if (!ptr) {
		ptr = strrchr (pathName, '\\');
	}
	strcpy (name, ptr + 1);
	*strrchr (name, '.') = 0;
}


TCHAR *GetString(int id)
{
	static TCHAR buf[256];

	if (hInstance)
		return LoadString(hInstance, id, buf, sizeof(buf)) ? buf : NULL;
	return NULL;
}

