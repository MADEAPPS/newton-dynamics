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

import com.javaNewton.nVector;
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
                0, 2, 1,   2, 0, 3,     // v0-v1-v2, v2-v3-v0 (front)
                4, 6, 5,   6, 4, 7,    // v0-v3-v4, v4-v5-v0 (right)
                8, 10,9,   10, 8,11,    // v0-v5-v6, v6-v1-v0 (top)
                12,14,13,  14,12,15,    // v1-v6-v7, v7-v2-v1 (left)
                16,18,17,  18,16,19,    // v7-v4-v3, v3-v2-v7 (bottom)
                20,22,21,  22,20,23     // v4-v7-v6, v6-v5-v4 (back)
        };

        SceneMeshTextureCache textCache = scene.GetTextureCache();
        m_cubeTexture = textCache.GetCubeTexture(
                "NewtonSky0003.tga", "NewtonSky0001.tga",
                "NewtonSky0006.tga", "NewtonSky0005.tga",
                "NewtonSky0002.tga", "NewtonSky0004.tga");

        m_skyMatrix = new nMatrix();
        m_textureMatrix = new nMatrix();
        m_textureMatrix.m_data[1].m_data[1] = -1.0f;
        m_textureMatrix.m_data[1].m_data[3] = size;

        m_eyeOffset = new nVector (0.0f, 0.25f, 0.0f, 1.0f);

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

        IntBuffer vaoIdBuffer = IntBuffer.allocate(1);
        GLES30.glGenVertexArrays(1, vaoIdBuffer);
        m_vertexArrayBuffer = vaoIdBuffer.get(0);
        GLES30.glBindVertexArray(m_vertexArrayBuffer);

        GLES30.glEnableVertexAttribArray(0);
        GLES30.glVertexAttribPointer(0, 3, GLES30.GL_FLOAT, false, 3 * 4, 0);

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

        m_shader = scene.GetShaderCache().m_skyBox;

        GLES30.glUseProgram(m_shader);
        m_textureMatrixLocation = GLES30.glGetUniformLocation(m_shader, "textureMatrix");
        m_matrixUniformLocation = GLES30.glGetUniformLocation(m_shader, "projectionViewModelMatrix");
        GLES30.glUseProgram(0);

        m_glTextureMatrix = new float[16];
        m_glModelViewProjectionMatrix = new float[16];
        m_textureMatrix.GetFlatArray(m_glTextureMatrix);
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

        if (m_vertexArrayBuffer != 0)
        {
            buffer[0] = m_vertexArrayBuffer;
            GLES30.glDeleteVertexArrays(1, buffer, 0);
        }
    }

    @Override
    public void Render (RenderScene scene, nMatrix parentMatrix)
    {
        GLES30.glDepthMask(false);

        SceneCamera camera = scene.GetCamera();

        nMatrix viewMatrix = camera.GetViewMatrix();
        m_skyMatrix.SetPosition(viewMatrix.UntransformVector(m_eyeOffset));
        nMatrix projectionViewModelMatrix = m_skyMatrix.Mul(viewMatrix).Mul(camera.GetProjectionMatrix());
        projectionViewModelMatrix.GetFlatArray(m_glModelViewProjectionMatrix);

        GLES30.glUseProgram(m_shader);
        GLES30.glUniformMatrix4fv(m_textureMatrixLocation, 1, false, m_glTextureMatrix, 0);
        GLES30.glUniformMatrix4fv(m_matrixUniformLocation, 1, false, m_glModelViewProjectionMatrix,0);

        GLES30.glActiveTexture(GLES30.GL_TEXTURE0);
        GLES30.glBindTexture(GLES30.GL_TEXTURE_CUBE_MAP, m_cubeTexture.m_id);
        GLES30.glBindVertexArray(m_vertexArrayBuffer);
        GLES30.glEnableVertexAttribArray(0);
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, m_vertexBuffer);
        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);

        //some how the cube map is no set properlly.
        //if I draw the box the background is black.
        GLES30.glDrawElements(GLES30.GL_TRIANGLES, 36, GLES30.GL_UNSIGNED_SHORT, 0);

        GLES30.glDepthMask(true);
    }

    private nVector m_eyeOffset;
    private nMatrix m_skyMatrix;
    private nMatrix m_textureMatrix;
    private float[] m_glTextureMatrix;
    private float[] m_glModelViewProjectionMatrix;
    private int m_shader;
    private int m_indexBuffer;
    private int m_vertexBuffer;
    private int m_vertexArrayBuffer;
    private int m_matrixUniformLocation;
    private int m_textureMatrixLocation;
    private SceneMeshTexture m_cubeTexture;
}
