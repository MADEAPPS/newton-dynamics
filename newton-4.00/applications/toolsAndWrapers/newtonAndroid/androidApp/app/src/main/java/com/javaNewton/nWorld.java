package com.javaNewton;

import com.newton.ndWorldGlue;
import com.newton.ndRigidBodyGlue;

import java.util.HashMap;

public class nWorld extends ndWorldGlue
{
    private HashMap<Integer, ndRigidBodyGlue> bodyMap = new HashMap<>();

    @Override
    public void AddBody(ndRigidBodyGlue body)
    {
        super.AddBody(body);
        bodyMap.put (body.GetId(), body);
    }

    @Override
    public void RemoveBody(ndRigidBodyGlue body)
    {
        bodyMap.remove (body.GetId());
        super.RemoveBody(body);
    }
}
