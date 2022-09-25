package com.javaNewton;


import com.newton.nWorld;
import com.newton.nRigidBody;
import com.newton.newtonJNI;

import java.util.HashMap;

public class NewtonWorld extends nWorld
{
    private HashMap<Integer, nRigidBody> bodyMap = new HashMap<>();

    @Override
    public void AddBody(nRigidBody body)
    {
        super.AddBody(body);
        bodyMap.put (body.GetId(), body);
    }

    @Override
    public void RemoveBody(nRigidBody body)
    {
		bodyMap.remove (body.GetId());
        super.RemoveBody(body);
    }
}
