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

package com.example.androidapp.Demos;

import com.example.androidapp.BodyNotify;
import com.example.androidapp.RenderScene;
import com.example.androidapp.SceneCamera;
import com.example.androidapp.SceneMeshPrimitive;
import com.example.androidapp.SceneObject;
import com.javaNewton.nVector;
import com.javaNewton.nMatrix;
import com.javaNewton.nRigidBody;
import com.javaNewton.nBodyNotify;
import com.javaNewton.nShapeBoxInstance;
import com.example.androidapp.Demos.DemoBase;

import com.newton.nRigidBodyType;

public class BasicRigidBodies extends DemoBase
{
    public BasicRigidBodies(RenderScene renderer)
    {
        super();

        AddFloor(renderer);
        AddBox(renderer);

        nMatrix matrix = new nMatrix();
        matrix.SetPosition(new nVector (-10.0f, 0.25f, 0.0f, 1.0f));
        SceneCamera camera = renderer.GetCamera();
        camera.SetMatrix(matrix);
    }

    private void AddFloor(RenderScene renderer)
    {
        nMatrix location = new nMatrix();
        location.SetPosition(new nVector(0.0f, -0.5f, 0.0f, 1.0f));

        nShapeBoxInstance shapeInstance = new nShapeBoxInstance(200.0f, 1.0f, 200.0f);
        SceneMeshPrimitive mesh = new SceneMeshPrimitive(shapeInstance, renderer, "default.tga");
        SceneObject floorObject = new SceneObject();
        floorObject.SetMesh(mesh);
        nRigidBody floor = new nRigidBody(nRigidBodyType.m_dynamic);
        floor.SetMatrix(location);
        floor.SetCollisionShape(shapeInstance);
        floor.SetNotify(new BodyNotify(floorObject));

        renderer.GetWorld().AddBody(floor);
        renderer.AddSceneObject(floorObject);
    }

    private void AddBox(RenderScene renderer)
    {
        nMatrix location = new nMatrix();
        location.SetPosition(new nVector(0.0f, 5.0f, 0.0f, 1.0f));

        nRigidBody box = new nRigidBody(nRigidBodyType.m_dynamic);
        nShapeBoxInstance shapeInstance = new nShapeBoxInstance(0.5f, 0.5f, 0.5f);
        SceneObject boxObject = new SceneObject();
        SceneMeshPrimitive mesh = new SceneMeshPrimitive(shapeInstance, renderer, "default.tga");
        boxObject.SetMesh(mesh);
        nBodyNotify notify = new BodyNotify(boxObject);
        notify.SetGravity(new nVector(0.0f, -10.0f, 0.0f, 0.0f));
        box.SetNotify(notify);
        box.SetMatrix(location);
        box.SetCollisionShape(shapeInstance);
        box.SetMassMatrix(1.0f, shapeInstance);
        renderer.GetWorld().AddBody(box);
        renderer.AddSceneObject(boxObject);
    }
}
