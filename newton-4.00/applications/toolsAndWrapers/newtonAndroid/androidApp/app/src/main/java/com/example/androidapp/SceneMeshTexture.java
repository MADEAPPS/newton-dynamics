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
import java.nio.Buffer;
import java.nio.ByteBuffer;
import android.content.res.AssetManager;

public class SceneMeshTexture
{
    public SceneMeshTexture(String name, AssetManager assetManager)
    {
        m_name = new String (name);
        String pathName = new String("textures/");
        pathName = pathName + name;
        ByteBuffer buffer = LoadAsset(pathName, assetManager);
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
            Log.e("LoadShaderCode", e.getMessage());
        }

        return null;
    }


    String m_name;
    int m_id;
}
