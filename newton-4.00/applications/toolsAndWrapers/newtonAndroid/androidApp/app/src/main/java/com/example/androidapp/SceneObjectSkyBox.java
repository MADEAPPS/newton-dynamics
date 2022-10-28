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

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import android.opengl.GLES30;

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

        float size = 200.0f;
        float vertices[] =
        {
                size, size, size,  -size, size, size,  -size,-size, size,  size,-size, size, // v0,v1,v2,v3 (front)
                size, size, size,   size,-size, size,   size,-size,-size,  size, size,-size, // v0,v3,v4,v5 (right)
                size, size, size,   size, size,-size,  -size, size,-size, -size, size, size, // v0,v5,v6,v1 (top)
                -size, size, size,  -size, size,-size,  -size,-size,-size, -size,-size, size, // v1,v6,v7,v2 (left)
                -size,-size,-size,   size,-size,-size,   size,-size, size, -size,-size, size, // v7,v4,v3,v2 (bottom)
                size,-size,-size,  -size,-size,-size,  -size, size,-size,  size, size,-size  // v4,v7,v6,v5 (back)
        };

        short indices[] =
        {
                0, 1, 2,   2, 3, 0,    // v0-v1-v2, v2-v3-v0 (front)
                4, 5, 6,   6, 7, 4,    // v0-v3-v4, v4-v5-v0 (right)
                8, 9,10,  10,11, 8,    // v0-v5-v6, v6-v1-v0 (top)
                12,13,14,  14,15,12,    // v1-v6-v7, v7-v2-v1 (left)
                16,17,18,  18,19,16,    // v7-v4-v3, v3-v2-v7 (bottom)
                20,21,22,  22,23,20     // v4-v7-v6, v6-v5-v4 (back)
        };

        m_textureMatrix = new nMatrix();
        m_textureMatrix.m_data[1].m_data[1] = -1.0f;
        m_textureMatrix.m_data[1].m_data[3] = size;

        //glGenBuffers(1, &m_vertexBuffer); //m_vbo
        //glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);

        int vertexSizeInBytes = 3 * 4;
        ByteBuffer bb = ByteBuffer.allocateDirect(24 * vertexSizeInBytes);
        bb.order(ByteOrder.nativeOrder());
        FloatBuffer gpuReadyBuffer = bb.asFloatBuffer();
        gpuReadyBuffer.put(vertices);
        gpuReadyBuffer.position(0);

        IntBuffer vertexBuffer = IntBuffer.allocate(1);
        GLES30.glGenBuffers(1, vertexBuffer);
        m_vertexBuffer = vertexBuffer.get(0);
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, m_vertexBuffer);
        GLES30.glBufferData(GLES30.GL_ARRAY_BUFFER, 24 * vertexSizeInBytes, gpuReadyBuffer, GLES30.GL_STATIC_DRAW);
        scene.checkGlError("SceneMeshPrimitive");

        //glGenVertexArrays(1, &m_vertextArrayBuffer);
        //glBindVertexArray(m_vertextArrayBuffer);
        IntBuffer vaoIdBuffer = IntBuffer.allocate(1);
        GLES30.glGenVertexArrays(1, vaoIdBuffer);
        m_vertextArrayBuffer = vaoIdBuffer.get(0);
        GLES30.glBindVertexArray(m_vertextArrayBuffer);

        GLES30.glEnableVertexAttribArray(0);
        GLES30.glVertexAttribPointer(0, 3, GLES30.GL_FLOAT, false, 3 * 4, 0);

        //GLES30.glGenBuffers(1, &m_indexBuffer);
        //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
        //glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);
        ByteBuffer gpuReadyIndexBuffer = ByteBuffer.allocateDirect(36 * 2);
        gpuReadyIndexBuffer.order(ByteOrder.nativeOrder());
        ShortBuffer drawListBuffer = gpuReadyIndexBuffer.asShortBuffer();
        drawListBuffer.put(indices);
        drawListBuffer.position(0);

        // upload the index buffer data to a gpu index buffer
        IntBuffer indexBuffer = IntBuffer.allocate(1); ;
        GLES30.glGenBuffers(1, indexBuffer);
        m_indexBuffer = indexBuffer.get(0);
        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
        GLES30.glBufferData(GLES30.GL_ELEMENT_ARRAY_BUFFER, 36 * 2, gpuReadyIndexBuffer, GLES30.GL_STATIC_DRAW);
        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, 0);
        scene.checkGlError("SceneMeshPrimitive");

        GLES30.glBindVertexArray(0);
        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, 0);
        GLES30.glDisableVertexAttribArray(1);
        GLES30.glDisableVertexAttribArray(0);
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, 0);

        m_shader = 0;
        m_matrixUniformLocation = 0;
        m_textureMatrixLocation = 0;
    }

    @Override
    public void CleanUp (RenderScene scene)
    {
        int[] buffer = new int[1];
        if (m_indexBuffer != 0)
        {
            buffer[0] = m_indexBuffer;
            GLES30.glDeleteBuffers(1, buffer, 0);
        }

        if (m_vertexBuffer != 0)
        {
            buffer[0] = m_vertexBuffer;
            GLES30.glDeleteBuffers(1, buffer, 0);
        }

        if (m_vertextArrayBuffer != 0)
        {
            buffer[0] = m_vertextArrayBuffer;
            GLES30.glDeleteVertexArrays(1, buffer, 0);
        }
    }

    @Override
    public void Render (RenderScene scene, nMatrix parentMatrix)
    {
        // sky box should not have any children
        //super.Render(scene, parentMatrix);
    }

    private nMatrix m_textureMatrix;
    private int m_shader;
    private int m_indexBuffer;
    private int m_vertexBuffer;
    private int m_vertextArrayBuffer;
    private int m_matrixUniformLocation;
    private int m_textureMatrixLocation;
    private SceneMeshTexture m_cubeTexture;
}
