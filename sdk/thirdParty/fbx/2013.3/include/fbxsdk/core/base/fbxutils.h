/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbxutils.h
#ifndef _FBXSDK_CORE_BASE_UTILITIES_H_
#define _FBXSDK_CORE_BASE_UTILITIES_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/core/base/fbxstring.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

/** Retrieve the environment variable value.
  * \return A new string containing the environment variable value.
  */
FBXSDK_DLL FbxString FbxGetEnv(const char* pEnvVar);

/** Get the application directory
  * \return The application directory.  
  */
FBXSDK_DLL FbxString FbxGetApplicationDirectory();

/** Retrieve the system temporary folder path name.
  * \return A new string containing the system temporary folder path name.
  */
FBXSDK_DLL FbxString FbxGetSystemTempPath();

/** Change the working directory of the system.
  */
FBXSDK_DLL void FbxSetCurrentWorkPath(const char* pPath_UTF8);

/** Retrieve the working directory of the system in UTF8 format.
  * \return A string that contain the current working directory of the system.
  */
FBXSDK_DLL FbxString FbxGetCurrentWorkPath();

class FBXSDK_DLL FbxPathUtils
{
public:
	/** Bind together a root path with a file path.
	  * \param pRootPath The root path that will get binded to the file path.
	  * \param pFilePath The file path to bind to the root path.
	  * \param pCleanPath If true, the resulting path will be cleaned via FbxPathUtils::Clean().
	  * \return Both paths binded together forming a new file path.
	  * \remark If the file path is already a full valid path, pFilePath is returned.
	  */
	static FbxString Bind(const char* pRootPath, const char* pFilePath, bool pCleanPath=true);

	/** Extract the folder name from the given file path.
	  * \param pFilePath The given file path.
	  * \return The folder name. If there isn't any '\\' or '/' in  the given file path, it will return pFilePath.
	  */
	static FbxString GetFolderName(const char* pFilePath);

	/** Extract file name from the given file path.
	  * \param pFilePath The given file path.
	  * \param pWithExtension Decide the file name with extension or without extension.
	  * If it is true, return the file name with extension;
	  * if it is false, return the file name without extension.
	  */
	static FbxString GetFileName(const char* pFilePath, bool pWithExtension=true);

	/** Extract the file extension in the given file path.
	  * \param pFilePath The file path to extract the extension.
	  * \return The file extension without the '.' character.
	  * \remark Return empty string if the file path doesn't contain a valid extension.
	  */
	static FbxString GetExtensionName(const char* pFilePath);

	/** Change or append a file extension to the specified file path.
	  * \param pFilePath The file path to change the file extension
	  * \param pExtension The extension to change or append to the file path.
	  * \return The file path with the file extension changed/added.
	  * \remark If the file path doesn't end with a valid file name, pFilePath is returned.
	  */
	static FbxString ChangeExtension(const char* pFilePath, const char* pExtension);

	//! Test if the given path is relative path, if it is return true.
	static bool IsRelative(const char* pPath);

	/** Get the given new path's relative path to the given root path.
	  * \param pRootPath The given root path
	  * \param pNewPath The given new path. If it is only file name, the default directory is work directory.
	  * \return The relative path.
	  * \remarks If the given two paths have the same drive, the function will turn  '\\' in the relative path to  '/'.
	  */
	static FbxString GetRelativePath(const char* pRootPath, const char* pNewPath);

	//! Get the given new path's relative path to the given root path.
	static FbxString GetRelativeFilePath(const char* pRootPath, const char* pNewFilePath);

	/** Get the full path of given path (if the given path is relative path,
	  * it will take current directory as default root path.)
	  */
	static FbxString Resolve(const char* pRelPath);

	//! Clean the redundant and useless denotations in given path name.
	static FbxString Clean(const char* pPath);

	/** Generate full safe file path name you can use to create new file.
	  * \param pFolder The folder where the file name should be attempted to be created.
	  * \param pPrefix The prefix of generated file name.
	  * \return A valid file path that can safely be used to create a new file.
	  */
	static FbxString GenerateFileName(const char* pFolder, const char* pPrefix);
};

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_CORE_BASE_UTILITIES_H_ */
