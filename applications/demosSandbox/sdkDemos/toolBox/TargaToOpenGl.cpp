/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#include <toolbox_stdafx.h>
#include "TargaToOpenGl.h"




struct TextureEntry
{
	GLuint m_textureID;
	dString m_textureName;
};

class TextureCache: public dTree<TextureEntry, dCRCTYPE>
{
public: 

	GLuint GetTexture(const char* const texName)
	{
		GLuint texID = 0;
		dAssert (texName);

		TextureEntry entry;
		//strcpy (entry.m_textureName, texName);
		//strlwr (entry.m_textureName.GetStr());
		entry.m_textureName = texName;
		entry.m_textureName.ToLower();
		dCRCTYPE crc = dCRC64 (entry.m_textureName.GetStr());

		dTreeNode* node = Find(crc);
		if (node) {
			texID = node->GetInfo().m_textureID;
		}
		return texID;
	}

	void InsertText (const char* const texName, GLuint id) 
	{
		TextureEntry entry;
		entry.m_textureID = id;
		//strcpy (entry.m_textureName, texName);
		//strlwr (entry.m_textureName);
		entry.m_textureName = texName;
		entry.m_textureName.ToLower();
		dCRCTYPE crc = dCRC64 (entry.m_textureName.GetStr());
		Insert(entry, crc);
	}


	~TextureCache ()
	{
		Iterator iter (*this);
		for (iter.Begin(); iter; iter ++) {
//			UnloadTexture (iter.GetNode()->GetInfo().m_textureID);
			glDeleteTextures(1, &iter.GetNode()->GetInfo().m_textureID);
		}
	}

	void RemoveById (GLuint id)
	{
		Iterator iter (*this);
		for (iter.Begin(); iter; iter ++) {
			if (iter.GetNode()->GetInfo().m_textureID == id) {
				Remove (iter.GetNode());
				break;
			}
		}
	}

	const char* FindById (GLuint id) const
	{
		Iterator iter (*this);
		for (iter.Begin(); iter; iter ++) {
			if (iter.GetNode()->GetInfo().m_textureID == id) {
				return iter.GetNode()->GetInfo().m_textureName.GetStr();
			}
		}
		return NULL;
	}


	
	static TextureCache& GetChache()
	{
		static TextureCache texCache;
		return texCache;
	}
};



//	Loads the texture from the specified file and stores it in iTexture. Note
//	that we're using the GLAUX library here, which is generally discouraged,
//	but in this case spares us having to write a bitmap loading routine.
GLuint LoadTexture(const char* const filename)
{
	#pragma pack(1)
	struct TGAHEADER
	{
		GLbyte	identsize;              // Size of ID field that follows header (0)
		GLbyte	colorMapType;           // 0 = None, 1 = palette
		GLbyte	imageType;              // 0 = none, 1 = indexed, 2 = rgb, 3 = grey, +8=rle
		unsigned short	colorMapStart;  // First color map entry
		unsigned short	colorMapLength; // Number of colors
		unsigned char 	colorMapBits;   // bits per palette entry
		unsigned short	xstart;         // image x origin
		unsigned short	ystart;         // image y origin
		unsigned short	width;          // width in pixels
		unsigned short	height;         // height in pixels
		GLbyte	bits;                   // bits per pixel (8 16, 24, 32)
		GLbyte	descriptor;             // image descriptor
	};
	#pragma pack(8)


	char fullPathName[2048];
	GetWorkingFileName (filename, fullPathName);
	TextureCache& cache = TextureCache::GetChache();
	GLuint texture = cache.GetTexture(fullPathName);
	if (!texture) {
	
		FILE* const pFile = fopen (fullPathName, "rb");
   		if(pFile == NULL) {
			return 0;
		}
	    
		//dAssert (sizeof (TGAHEADER) == 18);
		// Read in header (binary) sizeof(TGAHEADER) = 18
		TGAHEADER tgaHeader;		// TGA file header
		fread(&tgaHeader, 18, 1, pFile);

	    
		// Do byte swap for big vs little Indian
		tgaHeader.colorMapStart = SWAP_INT16(tgaHeader.colorMapStart);
		tgaHeader.colorMapLength = SWAP_INT16(tgaHeader.colorMapLength);
		tgaHeader.xstart = SWAP_INT16(tgaHeader.xstart);
		tgaHeader.ystart = SWAP_INT16(tgaHeader.ystart);
		tgaHeader.width = SWAP_INT16(tgaHeader.width);
		tgaHeader.height = SWAP_INT16(tgaHeader.height);
		        
		// Get width, height, and depth of texture
		GLint iWidth = tgaHeader.width;
		GLint iHeight = tgaHeader.height;
		short sDepth = tgaHeader.bits / 8;
		dAssert ((sDepth == 3) || (sDepth == 4));

	    
		// Put some validity checks here. Very simply, I only understand
		// or care about 8, 24, or 32 bit targa's.
		if(tgaHeader.bits != 8 && tgaHeader.bits != 24 && tgaHeader.bits != 32) {
			fclose(pFile);
			return 0;
		}


		// Calculate size of image buffer
		unsigned lImageSize = tgaHeader.width * tgaHeader.height * sDepth;
	    
		// Allocate memory and check for success
		GLbyte* const pBits = new GLbyte [tgaHeader.width * tgaHeader.height * 4];
		if(pBits == NULL) {
			fclose(pFile);
			return 0;
		}
	    
		// Read in the bits
		// Check for read error. This should catch RLE or other 
		// weird formats that I don't want to recognize
		if(fread(pBits, lImageSize, 1, pFile) != 1)  {
			fclose(pFile);
			delete[] pBits;
			return 0; 
	   }


		GLenum eFormat = GL_RGBA;
		GLint iComponents = 4;
		switch(sDepth)
		{
			case 3:     
				// Most likely case
				eFormat = GL_BGR;
				iComponents = 4;
				break;

			case 4:
				eFormat = GL_BGRA;
				iComponents = 4;
			break;

			case 1:
				eFormat = GL_LUMINANCE;
				iComponents = 1;
				break;
		};


		texture = 0;
		glGenTextures(1, &texture);
		if (texture) {
			//GLenum errr = glGetError ();
			glBindTexture(GL_TEXTURE_2D, texture);

		   // select modulate to mix texture with color for shading
			glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
			

			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		//	glTexImage2D(GL_TEXTURE_2D, 0, iComponents, iWidth, iHeight, 0, eFormat, GL_UNSIGNED_BYTE, pBits);

			// when texture area is small, bilinear filter the closest mipmap
		//  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST );

			// when texture area is small, trilinear filter mipmaped
			glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

			// when texture area is large, bilinear filter the first mipmap
			glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

			// build our texture mipmaps
			gluBuild2DMipmaps (GL_TEXTURE_2D, iComponents, iWidth, iHeight, eFormat, GL_UNSIGNED_BYTE, pBits);
			
			// Done with File
			fclose(pFile);
			delete[] pBits;

			cache.InsertText (fullPathName, texture);
		}
	}
	return texture;
} 


void UnloadTexture (GLuint texture)
{
	 glDeleteTextures(1, &texture);
	 TextureCache::GetChache().RemoveById (texture);
}

const char* FindTextureById (GLuint textureID)
{
	return TextureCache::GetChache().FindById (textureID);	
}