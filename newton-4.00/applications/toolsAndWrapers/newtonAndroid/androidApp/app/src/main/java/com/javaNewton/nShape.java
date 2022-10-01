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

package com.javaNewton;

import com.newton.ndShape;
import com.newton.ndShape;

public class nShape
{
    protected nShape(ndShape nativeObjecty)
    {
        m_nativeObject = nativeObjecty;
    }

    public ndShape GetNativeObject()
    {
        return m_nativeObject;
    }

    private ndShape m_nativeObject;
}
