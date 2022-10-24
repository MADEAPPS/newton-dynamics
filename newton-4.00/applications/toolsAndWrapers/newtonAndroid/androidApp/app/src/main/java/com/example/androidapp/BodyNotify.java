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
import com.javaNewton.nBodyNotify;

public class BodyNotify extends nBodyNotify
{
    public BodyNotify(SceneObject object)
    {
        super();
        m_object = object;
    }

    @Override
    public void OnTransform(nMatrix matrix)
    {
        m_object.SetMatrix(matrix);
    }

    private SceneObject m_object;
}
