package com.example.androidapp;

public class SceneMeshSegment
{
    public SceneMeshSegment(int indexOffset, int indexCount)
    {
        m_indexCount = indexCount;
        m_indexOffset = indexOffset;
    }


    private int m_indexCount;
    private int m_indexOffset;
}
