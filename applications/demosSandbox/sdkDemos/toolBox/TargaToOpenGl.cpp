/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "TargaToOpenGl.h"

struct TextureEntry: public dRefCounter
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
		entry.m_textureName = texName;
		entry.m_textureName.ToLower();
		dCRCTYPE crc = dCRC64 (entry.m_textureName.GetStr());

		dTreeNode* node = Find(crc);
		if (node) {
			 node->GetInfo().AddRef();
			texID = node->GetInfo().m_textureID;
		}
		return texID;
	}

	void InsertText (const char* const texName, GLuint id) 
	{
		TextureEntry entry;
		entry.m_textureID = id;
		entry.m_textureName = texName;
		entry.m_textureName.ToLower();
		dCRCTYPE crc = dCRC64 (entry.m_textureName.GetStr());
		Insert(entry, crc);
	}


	~TextureCache ()
	{
		Iterator iter (*this);
		for (iter.Begin(); iter; iter ++) {
			glDeleteTextures(1, &iter.GetNode()->GetInfo().m_textureID);
		}
	}

	void RemoveById (GLuint id)
	{
		Iterator iter (*this);
		for (iter.Begin(); iter; iter ++) {
			TextureEntry& entry = iter.GetNode()->GetInfo();
			if (entry.m_textureID == id) {
				if (entry.GetRef() == 1) {
					glDeleteTextures(1, &id);
					Remove (iter.GetNode());
				}
				break;
			}
		}
	}

	dTreeNode* FindById (GLuint id) const
	{
		Iterator iter (*this);
		for (iter.Begin(); iter; iter ++) {
			if (iter.GetNode()->GetInfo().m_textureID == id) {
				//return iter.GetNode()->GetInfo().m_textureName.GetStr();
				return iter.GetNode();
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
		char identsize;					// Size of ID field that follows header (0)
		char colorMapType;				// 0 = None, 1 = palette
		char imageType;					// 0 = none, 1 = indexed, 2 = rgb, 3 = grey, +8=rle
		unsigned short colorMapStart;	// First color map entry
		unsigned short colorMapLength;	// Number of colors
		unsigned char colorMapBits;		// bits per palette entry
		unsigned short xstart;			// image x origin
		unsigned short ystart;			// image y origin
		unsigned short width;			// width in pixels
		unsigned short height;			// height in pixels
		char bits;						// bits per pixel (8 16, 24, 32)
		char descriptor;				// image descriptor
	};
	#pragma pack(8)

	char fullPathName[2048];
	dGetWorkingFileName (filename, fullPathName);
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
		int width = tgaHeader.width;
		int height = tgaHeader.height;
		short sDepth = tgaHeader.bits / 8;
		dAssert ((sDepth == 3) || (sDepth == 4));

		// Put some validity checks here. Very simply, I only understand
		// or care about 8, 24, or 32 bit targa's.
		if(tgaHeader.bits != 8 && tgaHeader.bits != 24 && tgaHeader.bits != 32) {
			fclose(pFile);
			return 0;
		}


		// Calculate size of image buffer
		unsigned lImageSize = width * height * sDepth;

		// Allocate memory and check for success
		char* const pBits = new char [width * height * 4];
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

		TextureImageFormat format = m_rgb;
		switch(sDepth)
		{
			case 1:
				format = m_luminace;
				break;

			case 3:     
				format = m_rgb;
				break;

			case 4:
				format = m_rgba;
				break;
		};

		texture = LoadImage(fullPathName, pBits, tgaHeader.width, tgaHeader.height, format);

		// Done with File
		fclose(pFile);
		delete[] pBits;
	}
	return texture;
} 

#ifndef GL_BGR
#define GL_BGR 0x80E0
#define GL_BGRA 0x80E1
#endif

GLuint LoadImage(const char* const cacheName, const char* const buffer, int width, int hight, TextureImageFormat format)
{
	// Get width, height, and depth of texture
	GLint iWidth = width;
	GLint iHeight = hight;

	//GL_COLOR_INDEX, GL_RED, GL_GREEN, GL_BLUE, GL_ALPHA, GL_RGB, GL_RGBA, GL_BGR_EXT, GL_BGRA_EXT, GL_LUMINANCE, or GL_LUMINANCE_ALPHA.

	GLenum eFormat = GL_RGBA;
	GLint iComponents = 4;
	switch(format)
	{
		case m_rgb:     
			// Most likely case
			eFormat = GL_BGR;
			//eFormat = GL_RGB;
			iComponents = 4;
			break;

		case m_rgba:
			eFormat = GL_BGRA;
			//eFormat = GL_RGBA;
			iComponents = 4;
			break;

		case m_luminace:
			//eFormat = GL_LUMINANCE;
			eFormat = GL_LUMINANCE_ALPHA;
			//eFormat = GL_ALPHA;
			iComponents = 4;
			break;
	};

	GLuint texture = 0;
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
		gluBuild2DMipmaps (GL_TEXTURE_2D, iComponents, iWidth, iHeight, eFormat, GL_UNSIGNED_BYTE, buffer);

		// Done with File
		TextureCache& cache = TextureCache::GetChache();
		cache.InsertText (cacheName, texture);
	}

	return texture;
}


void ReleaseTexture (GLuint texture)
{
	TextureCache::GetChache().RemoveById (texture);
}

const char* FindTextureById (GLuint textureID)
{
	TextureCache& cache = TextureCache::GetChache();	
	TextureCache::dTreeNode* const node = cache.FindById (textureID);
	if (node) {
		return node->GetInfo().m_textureName.GetStr();
	}
	return NULL;
}


GLuint AddTextureRef (GLuint texture)
{
	TextureCache& cache = TextureCache::GetChache();	
	TextureCache::dTreeNode* const node = cache.FindById (texture);
	if (node) {
		node->GetInfo().AddRef();
	}
	return texture;
}
