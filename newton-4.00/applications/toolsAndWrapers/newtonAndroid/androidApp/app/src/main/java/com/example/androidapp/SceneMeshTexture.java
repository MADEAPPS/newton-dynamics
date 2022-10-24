/* Copyright (c) <2003-2022> <Newton Game Dynamics>
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely
 */

package com.example.androidapp;

import android.util.Log;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteOrder;
import java.nio.IntBuffer;
import java.nio.ByteBuffer;
import android.opengl.GLES30;
import android.content.res.AssetManager;

public class SceneMeshTexture
{
    public SceneMeshTexture(String name, AssetManager assetManager)
    {
        m_id = 0;
        m_name = new String (name);
        String pathName = new String("textures/");
        pathName = pathName + name;
        ByteBuffer image = LoadAsset(pathName, assetManager);
        ParseTargaImage(image);
    }

    private ByteBuffer LoadAsset(String name, AssetManager assetManager)
    {
        try
        {
            InputStream stream = assetManager.open(name);
            int size = 0;
            for (int data = stream.read(); data != -1; data = stream.read())
            {
                size ++;
            }
            stream.reset();

            ByteBuffer buffer = ByteBuffer.allocateDirect(size);
            for (int data = stream.read(); data != -1; data = stream.read())
            {
                buffer.put((byte)data);
            }
            buffer.rewind();

            return buffer;
        }
        catch (IOException e)
        {
            Log.e("LoadTextureAsset", e.getMessage());
        }

        return null;
    }

    private int SwapOrder(short bigIndianValue)
    {
        int x = bigIndianValue;
        if (ByteOrder.nativeOrder() == ByteOrder.LITTLE_ENDIAN)
        {
            x = ((bigIndianValue & 0xff) << 8) + ((bigIndianValue >> 8) & 0xff);
        }
        return x;
    }

    private void ParseTargaImage(ByteBuffer image)
    {
        image.get();
        image.get();

        int imageType = image.get();
        int colorMapStart = SwapOrder(image.getShort());
        int colorMapLength = SwapOrder(image.getShort());
        int colorMapBits = image.get();
        int xstart = SwapOrder(image.getShort());
        int ystart = SwapOrder(image.getShort());
        int width = SwapOrder(image.getShort());
        int height = SwapOrder(image.getShort());
        int bits = image.get();
        int descriptor = image.get();

        if (!((bits == 24) || (bits == 32)))
        {
            throw new RuntimeException("invalid texture format");
        }
        int imageSize = width * height * bits / 8;

        int iComponents = 4;
        int eFormat = GLES30.GL_RGB;
        if (bits == 32)
        {
            eFormat = GLES30.GL_RGBA;
        }

        ByteBuffer data = ByteBuffer.allocateDirect(imageSize);
        for (int i = 0; i < imageSize; i ++)
        {
            //data.put(image.get());
            if (i % 3 == 1)
                data.put((byte)128);
            else
                data.put((byte)0);
        }
        data.rewind();

        IntBuffer textureId = IntBuffer.allocate(1);
        GLES30.glGenTextures(1, textureId);
        m_id = textureId.get(0);

        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, m_id);
        GLES30.glTexParameterf(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_S, GLES30.GL_REPEAT);
        GLES30.glTexParameterf(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_T, GLES30.GL_REPEAT);
        GLES30.glTexParameterf(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_LINEAR_MIPMAP_LINEAR);
        GLES30.glTexParameterf(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_LINEAR);

        //float anisotropic = 0.0f;
        //GLES30.glGetFloatv(GLES30.GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, textureId);
        //GLES30.glTexParameterf(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAX_ANISOTROPY_EXT, anisotropic);

        GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D,0, eFormat, width, height,0, eFormat, GLES30.GL_UNSIGNED_BYTE, data );
        GLES30.glGenerateMipmap(GLES30.GL_TEXTURE_2D);
    }

    String m_name;
    int m_id;
}
