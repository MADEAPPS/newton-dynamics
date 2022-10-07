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

public class ShaderCache
{
    private class VertexShaders
    {
        static final String passPosition =
            "uniform mat4 uMVPMatrix;" +
            "attribute vec4 vPosition;" +

            "void main() {" +
            "  gl_Position = uMVPMatrix * vPosition;" +
            "}";

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
        static final String simpleColor =
            "precision mediump float;" +
            "uniform vec4 vColor;" +

            "void main() " +
            "{" +
            "  gl_FragColor = vColor;" +
            "}";

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

            "vec3 FlashLightShading(vec3 normalDir)" +
            "{" +
                "vec3 lightDir = -normalize (posit);" +

                "vec3 diffuseCoeff = vec3(0.7f, 0.7f, 0.7f);" +

                "float k1 = 7.0/120.0;" +
                "float k2 = 1.0/240.0;" +
                "float d2 = dot(posit, posit);" +
                "float d1 = sqrt(d2);" +
                "float attenuation = 1.0 / (1.0 + k1 * d1 + k2 * d2);" +

                "return diffuseCoeff * max (dot(normalDir, lightDir), 0.0) * attenuation;" +
            "}" +

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

    ShaderCache()
    {
        m_solidColor = CompileProgram(VertexShaders.passPosition, PixelShaders.simpleColor);
        m_directionalDiffuse = CompileProgram(VertexShaders.directionalDiffuse, PixelShaders.directionalDiffuse);
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
        return program;
    }
}
