package com.example.androidapp;


import android.opengl.GLES30;

public class ShaderCache
{
    private class TriangleProgran {
        static final String vertexShaderCode =
                "uniform mat4 uMVPMatrix;" +
                "attribute vec4 vPosition;" +
                "void main() {" +
                "  gl_Position = uMVPMatrix * vPosition;" +
                "}";

        static final String fragmentShaderCode =
                "precision mediump float;" +
                "uniform vec4 vColor;" +
                "void main() " +
                "{" +
                "  gl_FragColor = vColor;" +
                "}";
    }

    private class SquareProgram
    {
        static final String vertexShaderCode =
                "uniform mat4 uMVPMatrix;" +
                "attribute vec4 vPosition;" +
                "void main() {" +
                "  gl_Position = uMVPMatrix * vPosition;" +
                "}";

        static final String fragmentShaderCode =
                "precision mediump float;" +
                "uniform vec4 vColor;" +
                "void main() {" +
                "  gl_FragColor = vColor;" +
                "}";
    }

    public int m_squareShader = 0;
    public int m_triangleShader = 0;

    ShaderCache()
    {
        m_squareShader = ShaderCache(SquareProgram.vertexShaderCode, SquareProgram.fragmentShaderCode);
        m_triangleShader = ShaderCache(TriangleProgran.vertexShaderCode, TriangleProgran.fragmentShaderCode);
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

    private int ShaderCache(String vertexShaderCode, String fragmentShaderCode)
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
