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

import java.util.HashMap;
import android.content.res.AssetManager;

public class SceneMeshTextureCache
{
    public SceneMeshTextureCache(AssetManager assetManager)
    {
        m_assetManager = assetManager;
        m_textureMap = new HashMap<>();
    }

    public void Clear()
    {
        m_textureMap.clear();
    }

    public SceneMeshTexture GetTexture (String name)
    {
        int hash = name.hashCode();
        SceneMeshTexture texture = m_textureMap.get(hash);
        if (texture == null)
        {
            texture = new SceneMeshTexture(name, m_assetManager);
            m_textureMap.put(hash, texture);
        }
        return texture;
    }

    AssetManager m_assetManager;
    private HashMap<Integer, SceneMeshTexture> m_textureMap = new HashMap<>();
}
