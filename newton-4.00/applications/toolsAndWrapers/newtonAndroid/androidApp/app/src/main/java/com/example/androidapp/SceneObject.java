package com.example.androidapp;

public class SceneObject
{
    SceneObject ()
    {
        m_next = null;
        m_prev = null;
        m_parent = null;
        m_firstChild = null;
    }

    SceneObject (SceneObject parent)
    {
        m_parent = parent;
        m_next = parent.m_firstChild;
        if (parent.m_firstChild != null)
        {
            parent.m_firstChild.m_prev = this;
        }
        m_prev = null;
        m_firstChild = null;
        parent.m_firstChild = this;
    }

    private SceneObject m_next;
    private SceneObject m_prev;
    private SceneObject m_parent;
    private SceneObject m_firstChild;

}
