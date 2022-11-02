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

    public SceneMeshTexture(AssetManager assetManager,
        String filename_x0, String filename_x1,
        String filename_y0, String filename_y1,
        String filename_z0, String filename_z1)
    {
        m_id = 0;
        m_name = filename_x0;

        int faceArray[] = new int[6];
	    String namesArray[] = new String[6];

        namesArray[0] = filename_x0;
        namesArray[1] = filename_x1;
        faceArray[0] = GLES30.GL_TEXTURE_CUBE_MAP_POSITIVE_X;
        faceArray[1] = GLES30.GL_TEXTURE_CUBE_MAP_NEGATIVE_X;

        namesArray[2] = filename_y0;
        namesArray[3] = filename_y1;
        faceArray[2] = GLES30.GL_TEXTURE_CUBE_MAP_POSITIVE_Y;
        faceArray[3] = GLES30.GL_TEXTURE_CUBE_MAP_NEGATIVE_Y;

        namesArray[4] = filename_z0;
        namesArray[5] = filename_z1;
        faceArray[4] = GLES30.GL_TEXTURE_CUBE_MAP_POSITIVE_Z;
        faceArray[5] = GLES30.GL_TEXTURE_CUBE_MAP_NEGATIVE_Z;

        IntBuffer textureId = IntBuffer.allocate(1);
        GLES30.glActiveTexture(GLES30.GL_TEXTURE0);
        GLES30.glGenTextures(1, textureId);
        m_id = textureId.get(0);
        GLES30.glBindTexture(GLES30.GL_TEXTURE_CUBE_MAP, m_id);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_CUBE_MAP, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_LINEAR);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_CUBE_MAP, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_LINEAR);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_CUBE_MAP, GLES30.GL_TEXTURE_WRAP_S, GLES30.GL_CLAMP_TO_EDGE);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_CUBE_MAP, GLES30.GL_TEXTURE_WRAP_T, GLES30.GL_CLAMP_TO_EDGE);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_CUBE_MAP, GLES30.GL_TEXTURE_WRAP_R, GLES30.GL_CLAMP_TO_EDGE);

        for (int i = 0; i < 6; i ++)
        {
            String pathName = new String("textures/");
            pathName = pathName + namesArray[i];
            ByteBuffer image = LoadAsset(pathName, assetManager);
            ParseCubeFaceImage(image, faceArray[i]);
        }
    }

    public void Clear()
    {
        int[] textureId = new int[1];
        textureId[0] = m_id;
        GLES30.glDeleteTextures(1, textureId, 0);
    }

    private ByteBuffer LoadAsset(String name, AssetManager assetManager)
    {
        try
        {
            InputStream stream = assetManager.open(name);
            int size = stream.available();

            byte[] srcDdata = new byte[size];
            stream.read(srcDdata, 0, size);

            ByteBuffer buffer = ByteBuffer.allocateDirect(size);
            buffer.put(srcDdata);
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

        //if (!((bits == 24) || (bits == 32)))
        if (bits != 24)
        {
            throw new RuntimeException("invalid texture format");
        }

        int eFormat = GLES30.GL_RGB;
        int imageSize = width * height;
        ByteBuffer data = ByteBuffer.allocateDirect(imageSize * 3);
        for (int i = 0; i < imageSize; i ++)
        {
            byte red = image.get();
            byte green = image.get();
            byte blue = image.get();
            data.put(blue);
            data.put(green);
            data.put(red);
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

        GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D,0, eFormat, width, height,0, eFormat, GLES30.GL_UNSIGNED_BYTE, data );
        GLES30.glGenerateMipmap(GLES30.GL_TEXTURE_2D);
    }

    private void ParseCubeFaceImage(ByteBuffer image, int face)
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

        if (bits != 24)
        {
            throw new RuntimeException("invalid texture format");
        }

        int eFormat = GLES30.GL_RGB;
       int imageSize = width * height;
        ByteBuffer data = ByteBuffer.allocateDirect(imageSize * 3);
        for (int i = 0; i < imageSize; i ++)
        {
            byte red = image.get();
            byte green = image.get();
            byte blue = image.get();
            data.put(blue);
            data.put(green);
            data.put(red);
        }
        data.rewind();

        GLES30.glTexImage2D(face, 0, GLES30.GL_RGB, width, height, 0,  GLES30.GL_RGB, GLES30.GL_UNSIGNED_BYTE, data);
    }

    String m_name;
    int m_id;
}
