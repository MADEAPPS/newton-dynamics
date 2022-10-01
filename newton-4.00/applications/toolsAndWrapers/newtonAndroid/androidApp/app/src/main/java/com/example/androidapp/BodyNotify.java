package com.example.androidapp;

import com.javaNewton.nMatrix;
import com.javaNewton.nBodyNotify;

public class BodyNotify extends nBodyNotify
{
    BodyNotify(SceneObject object)
    {
        super();
        m_object = object;
    }

    @Override
    public void OnTransform(nMatrix matrix)
    {
        m_object.SetMatrix(matrix);
    }

    SceneObject m_object;
}
