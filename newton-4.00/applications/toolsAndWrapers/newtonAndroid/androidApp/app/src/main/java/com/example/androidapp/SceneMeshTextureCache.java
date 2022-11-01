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

import java.util.Map;
import java.util.HashMap;
import java.util.Iterator;
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
        Iterator<Map.Entry<Integer, SceneMeshTexture>> it = m_textureMap.entrySet().iterator();
        while (it.hasNext())
        {
            Map.Entry<Integer, SceneMeshTexture> entry = (Map.Entry<Integer, SceneMeshTexture>) it.next();
            SceneMeshTexture texture = entry.getValue();
            texture.Clear();
        }
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

    public SceneMeshTexture GetCubeTexture (
            String name_x0, String name_x1,
            String name_y0, String name_y1,
            String name_z0, String name_z1)
    {
        int hash = name_x0.hashCode();
        SceneMeshTexture texture = m_textureMap.get(hash);
        if (texture == null)
        {
            texture = new SceneMeshTexture(m_assetManager,
                name_x0, name_x1,
                name_y0, name_y1,
                name_z0, name_z1);
            m_textureMap.put(hash, texture);
        }
        return texture;
    }

    AssetManager m_assetManager;
    private HashMap<Integer, SceneMeshTexture> m_textureMap = new HashMap<>();
}
