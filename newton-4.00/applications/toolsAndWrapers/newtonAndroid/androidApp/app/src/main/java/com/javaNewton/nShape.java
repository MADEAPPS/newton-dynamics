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
