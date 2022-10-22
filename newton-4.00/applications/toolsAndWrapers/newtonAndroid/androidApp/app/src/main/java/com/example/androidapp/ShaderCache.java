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

import android.opengl.GLES30;

import android.util.Log;
import java.io.IOException;
import java.io.InputStream;
import android.content.res.AssetManager;

public class ShaderCache
{
    ShaderCache(RenderScene scene)
    {
        m_scene = scene;

        m_solidColor = LoadShaderProgram("solidColor");
        m_directionalDiffuse = LoadShaderProgram("directionalDiffuse");

        // check for open gles errors
        GLES30.glUseProgram(m_solidColor);
        scene.checkGlError("ShaderCache");
        GLES30.glUseProgram(m_directionalDiffuse);
        scene.checkGlError("ShaderCache");

        m_scene = null;
    }

    private int LoadShader(int type, String shaderCode)
    {
        // create a vertex shader type (GLES30.GL_VERTEX_SHADER)
        // or a fragment shader type (GLES30.GL_FRAGMENT_SHADER)
        int shader = GLES30.glCreateShader(type);

        // add the source code to the shader and compile it
        GLES30.glShaderSource(shader, shaderCode);
        GLES30.glCompileShader(shader);

        return shader;
    }

    private int CompileProgram(String vertexShaderCode, String fragmentShaderCode)
    {
        int vertexShader = LoadShader(GLES30.GL_VERTEX_SHADER, vertexShaderCode);
        int fragmentShader = LoadShader(GLES30.GL_FRAGMENT_SHADER, fragmentShaderCode);

        int program = GLES30.glCreateProgram();             // create empty OpenGL Program
        GLES30.glAttachShader(program, vertexShader);   // add the vertex shader to program
        GLES30.glAttachShader(program, fragmentShader); // add the fragment shader to program
        GLES30.glLinkProgram(program);

        m_scene.checkGlError("compiling shaders");
        return program;
    }

    private String LoadShaderCode(String name)
    {
        String sourceCode = new String();

        AssetManager assetManager = m_scene.GetAssetManager();
        try
        {
            InputStream stream = assetManager.open(name);
            for (int data = stream.read(); data != -1; data = stream.read())
            {
                sourceCode = sourceCode + Character.toString((char)data );
            }
        }
        catch (IOException e)
        {
            Log.e("LoadShaderCode", e.getMessage());
        }

        return sourceCode;
    }

    private int LoadShaderProgram(String shaderName)
    {
        String vertexShaderName = new String ("shaders/");
        vertexShaderName = vertexShaderName + shaderName + ".ver";
        String vertexShaderSource = LoadShaderCode (vertexShaderName);

        String pixelShaderName = new String ("shaders/");
        pixelShaderName = pixelShaderName + shaderName + ".pix";
        String pixelShaderSource = LoadShaderCode (pixelShaderName);

        int program = CompileProgram(vertexShaderSource, pixelShaderSource);
        return program;
    }

    private RenderScene m_scene = null;

    public int m_solidColor = 0;
    public int m_directionalDiffuse = 0;
}
