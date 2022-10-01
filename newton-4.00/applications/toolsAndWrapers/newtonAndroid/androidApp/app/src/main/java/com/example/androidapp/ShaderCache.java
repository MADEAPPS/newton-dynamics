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
    }


    public int m_solidColor = 0;

    ShaderCache()
    {
        m_solidColor = CompileProgram(VertexShaders.passPosition, PixelShaders.simpleColor);
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
