package com.example.androidapp;


import com.javaNewton.nVector;
import com.javaNewton.nMatrix;
import com.javaNewton.nShapeBox;
import com.javaNewton.nRigidBody;
import com.javaNewton.nBodyNotify;
import com.javaNewton.nShapeInstance;

import com.newton.nRigidBodyType;

public class DemosBase_BasicRigidBodies extends DemosBase
{
    DemosBase_BasicRigidBodies(MyGLRenderer renderer)
    {
        super(renderer);

        AddFloor(renderer);
        AddBox(renderer);
    }

    private void AddFloor(MyGLRenderer renderer)
    {
        nMatrix location = new nMatrix();
        location.SetPosition(new nVector(0.0f, -0.5f, 0.0f, 1.0f));
        nShapeInstance boxShape = new nShapeInstance(new nShapeBox(200.0f, 1.0f, 200.0f));
        SceneObject floorObject = new SceneObject();
        SceneMeshPrimitive mesh = new SceneMeshPrimitive(boxShape);
        floorObject.SetMesh(mesh);
        nRigidBody floor = new nRigidBody(nRigidBodyType.m_dynamic);
        floor.SetMatrix(location);
        floor.SetCollisionShape(boxShape);
        floor.SetNotify(new BodyNotify(floorObject));

        renderer.GetWorld().AddBody(floor);
        renderer.AddSceneObject(floorObject);
    }

    private void AddBox(MyGLRenderer renderer)
    {
        nMatrix location = new nMatrix();
        location.SetPosition(new nVector(0.0f, 5.0f, 0.0f, 1.0f));
        nRigidBody box = new nRigidBody(nRigidBodyType.m_dynamic);
        nShapeInstance boxShape = new nShapeInstance(new nShapeBox(0.5f, 0.5f, 0.5f));
        SceneObject boxObject = new SceneObject();
        SceneMeshPrimitive mesh = new SceneMeshPrimitive(boxShape);
        boxObject.SetMesh(mesh);
        nBodyNotify notify = new BodyNotify(boxObject);
        notify.SetGravity(new nVector(0.0f, -10.0f, 0.0f, 0.0f));
        box.SetNotify(notify);
        box.SetMatrix(location);
        box.SetCollisionShape(boxShape);
        box.SetMassMatrix(1.0f, boxShape);

        renderer.GetWorld().AddBody(box);
        renderer.AddSceneObject(boxObject);
    }
}
