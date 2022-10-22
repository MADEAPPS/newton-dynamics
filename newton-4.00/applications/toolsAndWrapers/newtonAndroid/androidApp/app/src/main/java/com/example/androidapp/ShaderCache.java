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
    private class VertexShaders
    {
            //"layout(location = 0) in vec3 in_position;" +
            //"layout(location = 1) in vec3 in_normal;" +
            //"layout(location = 2) in vec2 in_uv;" +

            //"attribute vec3 in_position;" +
            //"attribute vec3 in_normal;" +
            //"attribute vec2 in_uv;" +

        static final String directionalDiffuse =
            "layout(location = 0) in vec3 in_position;" +
            "layout(location = 1) in vec3 in_normal;" +
            "layout(location = 2) in vec2 in_uv;" +

            "uniform mat4 normalMatrix;" +
            "uniform mat4 viewModelMatrix;" +
            "uniform mat4 projectionMatrix;" +
            "uniform vec4 directionalLightDir;" +

            "out vec2 uv;" +
            "out vec3 posit;" +
            "out vec3 normal;" +
            "out vec3 lightDir;" +

            "void main()" +
            "{" +
                "lightDir = vec3(directionalLightDir.x, directionalLightDir.y, directionalLightDir.z);" +
                "posit = vec3 (viewModelMatrix * vec4(in_position, 1.0));" +
                "normal = vec3 (normalize (normalMatrix * vec4(in_normal, 0.0)));" +
                "uv = in_uv;" +
                "gl_Position = projectionMatrix * vec4(posit, 1.0);" +
            "}";
    }

    private class PixelShaders
    {
        static final String directionalDiffuse =
            "uniform sampler2D texture;" +
            "uniform float transparency;" +
            "uniform vec3 material_ambient;" +
            "uniform vec3 material_diffuse;" +
            "uniform vec3 material_specular;" +

            "in vec2 uv;" +
            "in vec3 posit;" +
            "in vec3 normal;" +
            "in vec3 lightDir;" +

            "out vec4 pixelColor;" +

            "vec3 PhongDirectionalShading(vec3 normalDir)" +
            "{" +
                "vec3 specularDir = normalize (-posit);" +

                "vec3 reflectionDir = -reflect (lightDir, normalDir);" +

                "vec3 ambientCoeff = vec3(0.0f, 0.0f, 0.0f);" +
                "vec3 diffuseCoeff = vec3(material_diffuse);" +
                "vec3 specularCoeff = vec3(0.0f, 0.0f, 0.0f);" +

                "vec3 emission = vec3(0.3f, 0.3f, 0.3f);" +
                "float shininess = 20.0f;" +

                "vec3 ambientColor = ambientCoeff + emission;" +
                "vec3 diffuseColor = diffuseCoeff * max (dot(normalDir, lightDir), 0.0f);" +
                "vec3 specularColor = specularCoeff * pow (max (dot (reflectionDir, specularDir), 0.1), shininess);" +

                "return ambientColor + diffuseColor + specularColor;" +
            "}" +

            "void main()" +
            "{" +
                "vec3 normalDir = normalize (normal);" +

                "vec3 lightIntensity = PhongDirectionalShading(normalDir);" +

                "vec3 texColor = lightIntensity * vec3 (texture2D(texture, uv));" +
                "pixelColor = vec4(texColor, transparency);" +
            "}";
    }


    public int m_solidColor = 0;
    public int m_directionalDiffuse = 0;

    ShaderCache(RenderScene scene)
    {
        m_scene = scene;

        m_solidColor = LoadShaderProgram("solidColor");
        m_directionalDiffuse = CompileProgram(VertexShaders.directionalDiffuse, PixelShaders.directionalDiffuse);

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
        String code = new String();

        AssetManager assetManager = m_scene.GetAssetManager();
        try
        {
            InputStream stream = assetManager.open(name);
            for (int data = stream.read(); data != -1; data = stream.read())
            {
                code = code + Character.toString((char)data );
            }
        }
        catch (IOException e)
        {
            Log.e("LoadShaderCode", e.getMessage());
        }

        return code;
    }

    private int LoadShaderProgram(String shaderName)
    {
        String vertexShaderName = new String ("shaders/");
        vertexShaderName = vertexShaderName + shaderName + ".vs";
        String vertexShader = LoadShaderCode (vertexShaderName);

        String pixelShaderName = new String ("shaders/");
        pixelShaderName = pixelShaderName + shaderName + ".ps";
        String pixelShader = LoadShaderCode (pixelShaderName);

        int program = CompileProgram(vertexShader, pixelShader);
        return program;
    }

    private RenderScene m_scene;
}
