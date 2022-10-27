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

import com.javaNewton.nMatrix;

public class SceneObjectSkyBox extends SceneObject
{
    public SceneObjectSkyBox(RenderScene scene)
    {
        super ();
        SceneMeshTextureCache textCache = scene.GetTextureCache();
        m_cubeTexture = textCache.GetCubeTexture(
          "NewtonSky0003.tga", "NewtonSky0001.tga",
                "NewtonSky0006.tga", "NewtonSky0005.tga",
                "NewtonSky0002.tga", "NewtonSky0004.tga");
    }

    @Override
    public void Render (RenderScene scene, nMatrix parentMatrix)
    {
        // sky box should not have any children
        //super.Render(scene, parentMatrix);
    }

    private SceneMeshTexture m_cubeTexture;
}
