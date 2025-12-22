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

#include "ndSandboxStdafx.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCameraNodeFollow.h"
#include "ndHeightFieldPrimitive.h"

#if 1

// Material ragdoll : désactive les collisions internes
class DGRagdollMaterial : public ndApplicationMaterial
{
    public:
    DGRagdollMaterial() : ndApplicationMaterial() {}

    ndApplicationMaterial* Clone() const override
    {
        return new DGRagdollMaterial(*this);
    }

    virtual bool OnAabbOverlap(const ndBodyKinematic* const, const ndBodyKinematic* const) const override
    {
        return false;
    }
};

static ndSharedPtr<ndBody> MakePrimitive(ndDemoEntityManager* const scene, const ndMatrix& matrix, const ndShapeInstance& shape, ndSharedPtr<ndRenderPrimitive> mesh, ndFloat32 mass)
{
    //ndPhysicsWorld* const world = scene->GetWorld();

    ndSharedPtr<ndRenderSceneNode> entity(new ndRenderSceneNode(matrix));
    entity->SetPrimitive(mesh);

    ndSharedPtr<ndBody> body(new ndBodyDynamic());
    body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
    body->SetMatrix(matrix);
    body->GetAsBodyDynamic()->SetCollisionShape(shape);
    body->GetAsBodyDynamic()->SetMassMatrix(mass, shape);

    //world->AddBody(body);
    scene->AddEntity(entity);
    return body;
}

class ndDGController : public ndModelNotify
{
    public:
    ndDGController(ndDemoEntityManager* const scene, ndModelArticulation* const model)
        :ndModelNotify()
        ,m_model(model)
    {
        ndMatrix location = ndGetIdentityMatrix();
        location.m_posit.m_y = 10.0f;
        AddRagdollHuman(scene, location, 1.0f);
    }

    void NormalizeMassDistribution(const std::vector<ndFloat32>& bodypartMassweigh,ndFloat32 totalMass) const
    {
        ndFloat32 totalVolume = 0.0f;
        //for (const auto& part : m_bodypartlist)
        for (size_t i = 0; i < m_bodypartlist.size(); ++i)
        {
            const auto& part = m_bodypartlist[i];
            ndFloat32 volume = part->GetAsBodyKinematic()->GetCollisionShape().GetVolume();
            totalVolume = volume * bodypartMassweigh[i];
        }
        totalVolume = ndMax(totalVolume, ndFloat32(1.0f));

        ndFloat32 density = totalMass / totalVolume;

        for (size_t i = 0; i < m_bodypartlist.size(); ++i)
        {
            const auto& part = m_bodypartlist[i];
            ndBodyKinematic* body = part->GetAsBodyKinematic();
            const ndShapeInstance& shape = body->GetCollisionShape();

            ndFloat32 volume = shape.GetVolume() * bodypartMassweigh[i];
            ndFloat32 mass = density * volume;

            ndVector massMatrix (body->GetMassMatrix());
            ndAssert(massMatrix.m_w = 1.0f);
            //ndMatrix inertia = shape.CalculateInertia();
            //ndFloat32 Ixx = inertia[0][0] * mass;
            //ndFloat32 Iyy = inertia[1][1] * mass;
            //ndFloat32 Izz = inertia[2][2] * mass;
            //ndFloat32 minInertia = mass * 0.04f;
            //Ixx = ndMax(Ixx, minInertia);
            //Iyy = ndMax(Iyy, minInertia);
            //Izz = ndMax(Izz, minInertia);
            //body->SetMassMatrix(ndVector(mass, Ixx, Iyy, Izz));

            massMatrix = massMatrix.Scale(mass);
            body->SetMassMatrix(massMatrix);
        }
    }

    private:
    void AddRagdollHuman(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 massPerPart)
    {
        ndPhysicsWorld* const world = scene->GetWorld();
        ndRender* const render = *scene->GetRenderer();

        ndFloat32 capsuleRadius = 0.125f;

        // === Tes matrices locales exactement comme tu les avais ===
        ndMatrix m_dummy_matrixLocal(ndGetIdentityMatrix());
        m_dummy_matrixLocal = ndYawMatrix(180.0f * ndDegreeToRad) * m_dummy_matrixLocal;

        ndMatrix m_bassin_matrixLocal(ndGetIdentityMatrix());
        m_bassin_matrixLocal.m_front = ndVector(1.49011612e-08f, 1.0f, 0.0f, 0.0f);
        m_bassin_matrixLocal.m_up = ndVector(-1.0f, 1.49011612e-08f, 0.0f, 0.0f);
        m_bassin_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_bassin_matrixLocal.m_posit = ndVector(0.0f, 0.0f, 0.0f, 1.0f);

        ndMatrix m_colonne_matrixLocal(ndGetIdentityMatrix());
        m_colonne_matrixLocal.m_front = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
        m_colonne_matrixLocal.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
        m_colonne_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_colonne_matrixLocal.m_posit = ndVector(1.125f, 0.0f, 0.0f, 1.0f);

        ndMatrix m_head_matrixLocal(ndGetIdentityMatrix());
        m_head_matrixLocal.m_front = ndVector(1.00000012f, 0.0f, -2.98023224e-08f, 0.0f);
        m_head_matrixLocal.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
        m_head_matrixLocal.m_right = ndVector(2.98023224e-08f, 0.0f, 1.00000012f, 0.0f);
        m_head_matrixLocal.m_posit = ndVector(1.875f, 0.0f, 0.0f, 1.0f);

        ndMatrix m_epaule_L_matrixLocal(ndGetIdentityMatrix());
        m_epaule_L_matrixLocal.m_front = ndVector(-0.453990579f, 0.891006589f, -8.01680784e-08f, 0.0f);
        m_epaule_L_matrixLocal.m_up = ndVector(-0.891006589f, -0.453990579f, -2.60481876e-08f, 0.0f);
        m_epaule_L_matrixLocal.m_right = ndVector(-5.96046377e-08f, 5.96046341e-08f, 1.00000012f, 0.0f);
        m_epaule_L_matrixLocal.m_posit = ndVector(0.949999988f, -0.699999988f, 0.0f, 1.0f);

        ndMatrix m_epaule_R_matrixLocal(ndGetIdentityMatrix());
        m_epaule_R_matrixLocal.m_front = ndVector(-0.453990459f, -0.891006589f, 0.0f, 0.0f);
        m_epaule_R_matrixLocal.m_up = ndVector(0.891006589f, -0.453990459f, 0.0f, 0.0f);
        m_epaule_R_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_epaule_R_matrixLocal.m_posit = ndVector(0.949999988f, 0.699999988f, 0.0f, 1.0f);

        ndMatrix m_bras_L_matrixLocal(ndGetIdentityMatrix());
        m_bras_L_matrixLocal.m_front = ndVector(-0.42261824f, -0.906307876f, 5.8389503e-08f, 0.0f);
        m_bras_L_matrixLocal.m_up = ndVector(0.906307936f, -0.42261827f, -5.10841858e-09f, 0.0f);
        m_bras_L_matrixLocal.m_right = ndVector(4.42074324e-08f, 3.78551732e-08f, 0.999999881f, 0.0f);
        m_bras_L_matrixLocal.m_posit = ndVector(-0.25f, 0.850000024f, 0.0f, 1.0f);

        ndMatrix m_bras_R_matrixLocal(ndGetIdentityMatrix());
        m_bras_R_matrixLocal.m_front = ndVector(-0.422618419f, 0.906307757f, 0.0f, 0.0f);
        m_bras_R_matrixLocal.m_up = ndVector(-0.906307757f, -0.422618419f, 0.0f, 0.0f);
        m_bras_R_matrixLocal.m_right = ndVector(0.0f, 0.0f, 0.99999994f, 0.0f);
        m_bras_R_matrixLocal.m_posit = ndVector(-0.25f, -0.850000024f, 0.0f, 1.0f);

        ndMatrix m_avant_bras_L_matrixLocal(ndGetIdentityMatrix());
        m_avant_bras_L_matrixLocal.m_front = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
        m_avant_bras_L_matrixLocal.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
        m_avant_bras_L_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_avant_bras_L_matrixLocal.m_posit = ndVector(-1.64999998f, 0.0f, 0.0f, 1.0f);

        ndMatrix m_avant_bras_R_matrixLocal(ndGetIdentityMatrix());
        m_avant_bras_R_matrixLocal.m_front = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
        m_avant_bras_R_matrixLocal.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
        m_avant_bras_R_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_avant_bras_R_matrixLocal.m_posit = ndVector(-1.64999998f, 0.0f, 0.0f, 1.0f);

        ndMatrix m_hand_L_matrixLocal(ndGetIdentityMatrix());
        m_hand_L_matrixLocal.m_front = ndVector(2.22044605e-16f, 1.49011612e-08f, -1.0f, 0.0f);
        m_hand_L_matrixLocal.m_up = ndVector(-1.0f, 1.49011612e-08f, 0.0f, 0.0f);
        m_hand_L_matrixLocal.m_right = ndVector(1.49011612e-08f, 1.0f, 1.49011612e-08f, 0.0f);
        m_hand_L_matrixLocal.m_posit = ndVector(-0.949999988f, 0.0f, 0.0f, 1.0f);

        ndMatrix m_hand_R_matrixLocal(ndGetIdentityMatrix());
        m_hand_R_matrixLocal.m_front = ndVector(-8.74227766e-08f, 1.02323938e-07f, -1.0f, 0.0f);
        m_hand_R_matrixLocal.m_up = ndVector(1.0f, -1.49011523e-08f, -8.74227766e-08f, 0.0f);
        m_hand_R_matrixLocal.m_right = ndVector(-1.49011612e-08f, -1.0f, -1.02323938e-07f, 0.0f);
        m_hand_R_matrixLocal.m_posit = ndVector(-0.949999988f, 0.0f, 0.0f, 1.0f);

        ndMatrix m_hip_L_matrixLocal(ndGetIdentityMatrix());
        m_hip_L_matrixLocal.m_front = ndVector(-0.173648104f, -0.98480773f, 0.0f, 0.0f);
        m_hip_L_matrixLocal.m_up = ndVector(0.98480773f, -0.173648104f, 0.0f, 0.0f);
        m_hip_L_matrixLocal.m_right = ndVector(8.88178367e-16f, 0.0f, 0.999999881f, 0.0f);
        m_hip_L_matrixLocal.m_posit = ndVector(-0.5f, -0.5f, 0.0f, 1.0f);

        ndMatrix m_hip_R_matrixLocal(ndGetIdentityMatrix());
        m_hip_R_matrixLocal.m_front = ndVector(-0.173648253f, 0.98480767f, 0.0f, 0.0f);
        m_hip_R_matrixLocal.m_up = ndVector(-0.98480767f, -0.173648253f, 0.0f, 0.0f);
        m_hip_R_matrixLocal.m_right = ndVector(0.0f, 0.0f, 0.999999881f, 0.0f);
        m_hip_R_matrixLocal.m_posit = ndVector(-0.5f, 0.5f, 0.0f, 1.0f);

        ndMatrix m_cuisse_L_matrixLocal(ndGetIdentityMatrix());
        m_cuisse_L_matrixLocal.m_front = ndVector(-0.173648193f, 0.984807789f, -6.79000536e-08f, 0.0f);
        m_cuisse_L_matrixLocal.m_up = ndVector(-0.984807789f, -0.173648193f, -4.75441304e-08f, 0.0f);
        m_cuisse_L_matrixLocal.m_right = ndVector(-4.37113812e-08f, 5.86125495e-08f, 0.99999994f, 0.0f);
        m_cuisse_L_matrixLocal.m_posit = ndVector(0.600000024f, -1.0f, 0.0f, 1.0f);

        ndMatrix m_cuisse_R_matrixLocal(ndGetIdentityMatrix());
        m_cuisse_R_matrixLocal.m_front = ndVector(-0.173648164f, -0.98480773f, 0.0f, 0.0f);
        m_cuisse_R_matrixLocal.m_up = ndVector(0.98480773f, -0.173648164f, 0.0f, 0.0f);
        m_cuisse_R_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_cuisse_R_matrixLocal.m_posit = ndVector(0.600000024f, 1.0f, 0.0f, 1.0f);

        ndMatrix m_tibia_L_matrixLocal(ndGetIdentityMatrix());
        m_tibia_L_matrixLocal.m_front = ndVector(0.99999994f, 0.0f, 0.0f, 0.0f);
        m_tibia_L_matrixLocal.m_up = ndVector(0.0f, 0.99999994f, 0.0f, 0.0f);
        m_tibia_L_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_tibia_L_matrixLocal.m_posit = ndVector(-1.72500002f, 0.0f, 0.0f, 1.0f);

        ndMatrix m_tibia_R_matrixLocal(ndGetIdentityMatrix());
        m_tibia_R_matrixLocal.m_front = ndVector(0.99999994f, 0.0f, 0.0f, 0.0f);
        m_tibia_R_matrixLocal.m_up = ndVector(0.0f, 0.99999994f, 0.0f, 0.0f);
        m_tibia_R_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_tibia_R_matrixLocal.m_posit = ndVector(-1.72500002f, 0.0f, 0.0f, 1.0f);

        ndMatrix m_pied_L_matrixLocal(ndGetIdentityMatrix());
        m_pied_L_matrixLocal.m_front = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
        m_pied_L_matrixLocal.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
        m_pied_L_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_pied_L_matrixLocal.m_posit = ndVector(-0.75f, 0.0f, 0.150000006f, 1.0f);

        ndMatrix m_pied_R_matrixLocal(ndGetIdentityMatrix());
        m_pied_R_matrixLocal.m_front = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
        m_pied_R_matrixLocal.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
        m_pied_R_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_pied_R_matrixLocal.m_posit = ndVector(-0.75f, 0.0f, 0.150000006f, 1.0f);

        ndMatrix m_orteille_L_matrixLocal(ndGetIdentityMatrix());
        m_orteille_L_matrixLocal.m_front = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
        m_orteille_L_matrixLocal.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
        m_orteille_L_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_orteille_L_matrixLocal.m_posit = ndVector(0.0f, 0.0f, 0.5f, 1.0f);

        ndMatrix m_orteille_R_matrixLocal(ndGetIdentityMatrix());
        m_orteille_R_matrixLocal.m_front = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
        m_orteille_R_matrixLocal.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
        m_orteille_R_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_orteille_R_matrixLocal.m_posit = ndVector(0.0f, 0.0f, 0.5f, 1.0f);

        auto CreateCapsule = [&](ndFloat32 height, const ndMatrix& localMatrix, const char* texture = "smilli.png") -> ndSharedPtr<ndBody>
        {
            ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeCapsule(capsuleRadius, capsuleRadius, height)));

            ndRenderPrimitive::ndDescriptor desc(render);
            desc.m_collision = shape;
            desc.m_mapping = ndRenderPrimitive::m_capsule;
            desc.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName(texture)));
            ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(desc));

            ndMatrix global = localMatrix * location;
            return MakePrimitive(scene, global, **shape, mesh, massPerPart);
        };

        auto CreateBox = [&](ndFloat32 sx, ndFloat32 sy, ndFloat32 sz, const ndMatrix& localMatrix, const char* texture = "wood_0.png") -> ndSharedPtr<ndBody>
        {
            ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeBox(sx, sy, sz)));

            ndRenderPrimitive::ndDescriptor desc(render);
            desc.m_collision = shape;
            desc.m_mapping = ndRenderPrimitive::m_box;
            desc.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName(texture)));
            ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(desc));

            ndMatrix global = localMatrix * location;
            return MakePrimitive(scene, global, **shape, mesh, massPerPart);
        };

        ndMatrix globalRoot = m_dummy_matrixLocal;

        ndMatrix bassinMatrix = m_bassin_matrixLocal * globalRoot;
        ndSharedPtr<ndBody> bassinBody = CreateCapsule(0.75f, bassinMatrix, "wood_0.png");

        ndMatrix colonneMatrix = m_colonne_matrixLocal * bassinMatrix;
        ndSharedPtr<ndBody> colonneBody = CreateCapsule(1.25f, colonneMatrix);

        ndMatrix headMatrix = m_head_matrixLocal * colonneMatrix;
        ndSharedPtr<ndBody> headBody = CreateCapsule(2.0f, headMatrix, "wood_0.png");

        ndMatrix epauleLMatrix = m_epaule_L_matrixLocal * colonneMatrix;
        ndSharedPtr<ndBody> epauleLBody = CreateCapsule(1.25f, epauleLMatrix);

        ndMatrix brasLMatrix = m_bras_L_matrixLocal * epauleLMatrix;
        ndSharedPtr<ndBody> brasLBody = CreateCapsule(1.85f, brasLMatrix);

        ndMatrix avantbrasLMatrix = m_avant_bras_L_matrixLocal * brasLMatrix;
        ndSharedPtr<ndBody> avantbrasLBody = CreateCapsule(1.25f, avantbrasLMatrix);

        ndMatrix handLMatrix = m_hand_L_matrixLocal * avantbrasLMatrix;
        ndSharedPtr<ndBody> handLBody = CreateBox(0.125f, 0.5f, 0.25f, handLMatrix);

        ndMatrix epauleRMatrix = m_epaule_R_matrixLocal * colonneMatrix;
        ndSharedPtr<ndBody> epauleRBody = CreateCapsule(1.25f, epauleRMatrix);

        ndMatrix brasRMatrix = m_bras_R_matrixLocal * epauleRMatrix;
        ndSharedPtr<ndBody> brasRBody = CreateCapsule(1.85f, brasRMatrix);

        ndMatrix avantbrasRMatrix = m_avant_bras_R_matrixLocal * brasRMatrix;
        ndSharedPtr<ndBody> avantbrasRBody = CreateCapsule(1.25f, avantbrasRMatrix);

        ndMatrix handRMatrix = m_hand_R_matrixLocal * avantbrasRMatrix;
        ndSharedPtr<ndBody> handRBody = CreateBox(0.125f, 0.5f, 0.25f, handRMatrix);

        ndMatrix hipLMatrix = m_hip_L_matrixLocal * bassinMatrix;
        ndSharedPtr<ndBody> hipLBody = CreateCapsule(0.75f, hipLMatrix);

        ndMatrix cuisseLMatrix = m_cuisse_L_matrixLocal * hipLMatrix;
        ndSharedPtr<ndBody> cuisseLBody = CreateCapsule(2.0f, cuisseLMatrix);

        ndMatrix tibiaLMatrix = m_tibia_L_matrixLocal * cuisseLMatrix;
        ndSharedPtr<ndBody> tibiaLBody = CreateCapsule(1.25f, tibiaLMatrix);

        ndMatrix piedLMatrix = m_pied_L_matrixLocal * tibiaLMatrix;
        ndSharedPtr<ndBody> piedLBody = CreateBox(0.13f, 0.4f, 0.75f, piedLMatrix);

        ndMatrix orteilLMatrix = m_orteille_L_matrixLocal * piedLMatrix;
        ndSharedPtr<ndBody> orteilLBody = CreateBox(0.125f, 0.4f, 0.25f, orteilLMatrix);

        ndMatrix hipRMatrix = m_hip_R_matrixLocal * bassinMatrix;
        ndSharedPtr<ndBody> hipRBody = CreateCapsule(0.75f, hipRMatrix);

        ndMatrix cuisseRMatrix = m_cuisse_R_matrixLocal * hipRMatrix;
        ndSharedPtr<ndBody> cuisseRBody = CreateCapsule(2.0f, cuisseRMatrix);

        ndMatrix tibiaRMatrix = m_tibia_R_matrixLocal * cuisseRMatrix;
        ndSharedPtr<ndBody> tibiaRBody = CreateCapsule(1.25f, tibiaRMatrix);

        ndMatrix piedRMatrix = m_pied_R_matrixLocal * tibiaRMatrix;
        ndSharedPtr<ndBody> piedRBody = CreateBox(0.13f, 0.4f, 0.75f, piedRMatrix);

        ndMatrix orteilRMatrix = m_orteille_R_matrixLocal * piedRMatrix;
        ndSharedPtr<ndBody> orteilRBody = CreateBox(0.125f, 0.4f, 0.25f, orteilRMatrix);

        // === Liste des bodies ===
        m_bodypartlist = 
        {
            bassinBody, colonneBody, headBody,
            epauleLBody, brasLBody, avantbrasLBody, handLBody,
            epauleRBody, brasRBody, avantbrasRBody, handRBody,
            hipLBody, cuisseLBody, tibiaLBody, piedLBody, orteilLBody,
            hipRBody, cuisseRBody, tibiaRBody, piedRBody, orteilRBody
        };

        std::vector<ndFloat32> bodypartMassweigh =
        {
            3.0f, //bassinBody, 
            2.0f, //colonneBody, 
            0.5f, //headBody,
            1.0f, //epauleLBody, 
            1.0f, //brasLBody, 
            1.0f, //avantbrasLBody, 
            1.0f, //handLBody,
            1.0f, //epauleRBody, 
            1.0f, //brasRBody, 
            1.0f, //avantbrasRBody, 
            1.0f, //handRBody,
            2.0f, //hipLBody, 
            1.0f, //cuisseLBody, 
            1.0f, //tibiaLBody, 
            1.0f, //piedLBody, 
            1.0f, //orteilLBody,
            2.0f, //hipRBody, 
            1.0f, //cuisseRBody, 
            1.0f, //tibiaRBody, 
            1.0f, //piedRBody, 
            1.0f, //orteilRBody
        };

        // === Material ragdoll ===
        ndContactCallback* callback = (ndContactCallback*)world->GetContactNotify();
        DGRagdollMaterial ragdollMat;
        callback->RegisterMaterial(ragdollMat, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);

        for (auto& body : m_bodypartlist)
        {
            ndShapeMaterial mat = body->GetAsBodyDynamic()->GetCollisionShape().GetMaterial();
            mat.m_userId = ndDemoContactCallback::m_modelPart;
            body->GetAsBodyDynamic()->GetCollisionShape().SetMaterial(mat);
        }

        NormalizeMassDistribution(bodypartMassweigh, 80.0f);

        // === Tes joints (je les laisse tels quels, ils étaient déjà bons) ===
        // (tu peux les remettre exactement comme dans ton code original)
        ndJointHinge* joint1 = nullptr;
        ndJointHinge* joint2 = nullptr;
        ndJointHinge* joint3 = nullptr;
        ndJointHinge* joint4 = nullptr;
        ndJointSpherical* joint5 = nullptr;
        ndJointSpherical* joint6 = nullptr;
        ndJointHinge* joint7 = nullptr;
        ndJointHinge* joint8 = nullptr;
        ndJointHinge* joint9 = nullptr;
        ndJointHinge* joint10 = nullptr;
        ndJointHinge* joint11 = nullptr;
        ndJointHinge* joint12 = nullptr;
        ndJointSpherical* joint13 = nullptr;
        ndJointSpherical* joint14 = nullptr;
        ndJointSpherical* joint15 = nullptr;
        ndJointSpherical* joint16 = nullptr;
        ndJointHinge* joint17 = nullptr;
        ndJointHinge* joint18 = nullptr;
        ndJointHinge* joint19 = nullptr;
        ndJointHinge* joint20 = nullptr;

        ndModelArticulation::ndNode* nextRootTemp = nullptr;
        ndModelArticulation::ndNode* nextRootTemp1 = nullptr;
        ndModelArticulation::ndNode* nextRootTemp2 = nullptr;
        ndModelArticulation::ndNode* nextRootTemp3 = nullptr;

        ndModelArticulation::ndNode* modelRootNode = m_model->AddRootBody(bassinBody);

        { // colonne
            ndMatrix tmp1(colonneBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1] - 0.675f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //
            joint1 = new ndJointHinge(tmp1, colonneBody->GetAsBodyKinematic(), bassinBody->GetAsBodyKinematic());
            joint1->SetLimitState(true);
            joint1->SetLimits(-25.0f * ndDegreeToRad, 2.0f * ndDegreeToRad);
            joint1->SetAsSpringDamper(0.01f, 10.0f, 0.5f);
            //
            //m_jointlist.push_back(joint1);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint1);
            nextRootTemp = m_model->AddLimb(modelRootNode, colonneBody, jointPtr);
            nextRootTemp3 = nextRootTemp;
        }

        //{ // head
        //    ndMatrix tmp1(headBody->GetMatrix());
        //    tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1] - 1.05f, tmp1[3][2], 1.0f); // offset
        //    tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
        //
        //    joint2 = new ndJointHinge(tmp1, headBody->GetAsBodyKinematic(), colonneBody->GetAsBodyKinematic());
        //    joint2->SetLimitState(true);
        //    joint2->SetLimits(-120.0f * ndDegreeToRad, 2.0f * ndDegreeToRad);
        //    joint2->SetAsSpringDamper(0.01f, 25.0f, 0.5f);
        //    //
        //    //m_jointlist.push_back(joint2);
        //    //
        //    ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint2);
        //    nextRootTemp = m_model->AddLimb(nextRootTemp, headBody, jointPtr);
        //}

#if 1
        // upper bodi parts
        { // epaule_R
            ndMatrix tmp1(epauleRBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0] - 0.65f, tmp1[3][1] - 0.325f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            tmp1 = ndPitchMatrix(-180.0f * ndDegreeToRad) * tmp1;
            tmp1 = ndYawMatrix(-180.0f * ndDegreeToRad) * tmp1;
            //
            joint14 = new ndJointSpherical(tmp1, epauleRBody->GetAsBodyKinematic(), colonneBody->GetAsBodyKinematic());
            //joint14->SetLimitState(true);
            //joint14->SetLimits(-15.0f * ndDegreeToRad, 15.0f * ndDegreeToRad);
            joint14->SetConeLimit(24.0f * ndDegreeToRad);
            joint14->SetTwistLimits(-12.5f * ndDegreeToRad, 12.5f * ndDegreeToRad);
            joint14->SetAsSpringDamper(0.005f, 50.0f, 10.0f);
            //
            //m_jointlist.push_back(joint14);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint14);
            nextRootTemp2 = m_model->AddLimb(nextRootTemp3, epauleRBody, jointPtr);
        }

        { // epaule_L
            ndMatrix tmp1(epauleLBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0] + 0.65f, tmp1[3][1] - 0.325f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //
            joint13 = new ndJointSpherical(tmp1, epauleLBody->GetAsBodyKinematic(), colonneBody->GetAsBodyKinematic());
            //joint13->SetLimitState(true);
            //joint13->SetLimits(-15.0f * ndDegreeToRad, 15.0f * ndDegreeToRad);
            joint13->SetConeLimit(24.0f * ndDegreeToRad);
            joint13->SetTwistLimits(-12.5f * ndDegreeToRad, 12.5f * ndDegreeToRad);
            joint13->SetAsSpringDamper(0.005f, 50.0f, 10.0f);
            //
            //m_jointlist.push_back(joint13);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint13);
            nextRootTemp1 = m_model->AddLimb(nextRootTemp3, epauleLBody, jointPtr);
        }

        { // bras_R
            ndMatrix tmp1(brasRBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0] + 0.025f, tmp1[3][1] + 0.95f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            tmp1 = ndPitchMatrix(-180.0f * ndDegreeToRad) * tmp1;
            tmp1 = ndYawMatrix(-180.0f * ndDegreeToRad) * tmp1;

            joint16 = new ndJointSpherical(tmp1, brasRBody->GetAsBodyKinematic(), epauleRBody->GetAsBodyKinematic());
            //joint16->SetLimitState(true);
            //joint16->SetLimits(-165.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint16->SetConeLimit(175.0f * ndDegreeToRad);
            joint16->SetTwistLimits(-175.0f * ndDegreeToRad, 175.0f * ndDegreeToRad);
            joint16->SetAsSpringDamper(0.005f, 50.0f, 10.0f);
            //
            //m_jointlist.push_back(joint16);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint16);
            nextRootTemp2 = m_model->AddLimb(nextRootTemp2, brasRBody, jointPtr);
        }

        { // bras_L
            ndMatrix tmp1(brasLBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0] - 0.025f, tmp1[3][1] + 0.95f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //
            joint15 = new ndJointSpherical(tmp1, brasLBody->GetAsBodyKinematic(), epauleLBody->GetAsBodyKinematic());
            //joint15->SetLimitState(true);
            //joint15->SetLimits(-90.0f * ndDegreeToRad, 165.0f * ndDegreeToRad);
            joint15->SetConeLimit(175.0f * ndDegreeToRad);
            joint15->SetTwistLimits(-175.0f * ndDegreeToRad, 175.0f * ndDegreeToRad);
            joint15->SetAsSpringDamper(0.005f, 50.0f, 10.0f);
            //
            //m_jointlist.push_back(joint15);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint15);
            nextRootTemp1 = m_model->AddLimb(nextRootTemp1, brasLBody, jointPtr);
        }

        { // avantbras_R arm
            ndMatrix tmp1(avantbrasRBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1] + 0.7f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            tmp1 = ndPitchMatrix(-180.0f * ndDegreeToRad) * tmp1;
            tmp1 = ndYawMatrix(-180.0f * ndDegreeToRad) * tmp1;
            //
            joint18 = new ndJointHinge(tmp1, avantbrasRBody->GetAsBodyKinematic(), brasRBody->GetAsBodyKinematic());
            joint18->SetLimitState(true);
            joint18->SetLimits(-165.0f * ndDegreeToRad, 25.0f * ndDegreeToRad);
            joint18->SetAsSpringDamper(0.005f, 50.0f, 10.0f);
            //
            //m_jointlist.push_back(joint18);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint18);
            nextRootTemp2 = m_model->AddLimb(nextRootTemp2, avantbrasRBody, jointPtr);
        }

        { // avantbras_L = arm
            ndMatrix tmp1(avantbrasLBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1] + 0.7f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //
            joint17 = new ndJointHinge(tmp1, avantbrasLBody->GetAsBodyKinematic(), brasLBody->GetAsBodyKinematic());
            joint17->SetLimitState(true);
            joint17->SetLimits(-25.0f * ndDegreeToRad, 165.0f * ndDegreeToRad);
            joint17->SetAsSpringDamper(0.005f, 50.0f, 10.0f);
            //
            //m_jointlist.push_back(joint17);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint17);
            nextRootTemp1 = m_model->AddLimb(nextRootTemp1, avantbrasLBody, jointPtr);
        }

        { // hand_R
            ndMatrix tmp1(handRBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1] + 0.25f, tmp1[3][2], 1.0f); // offset
            //tmp1 = ndRollMatrix(180.0f * ndDegreeToRad) * tmp1;
            tmp1 = ndPitchMatrix(-180.0f * ndDegreeToRad) * tmp1;
            tmp1 = ndYawMatrix(-90.0f * ndDegreeToRad) * tmp1;
            //
            joint20 = new ndJointHinge(tmp1, handRBody->GetAsBodyKinematic(), avantbrasRBody->GetAsBodyKinematic());
            joint20->SetLimitState(true);
            joint20->SetLimits(-45.0f * ndDegreeToRad, 65.0f * ndDegreeToRad);
            ////joint20->SetAsSpringDamper(0.01f, 5.0f, 0.5f);
            //m_jointlist.push_back(joint20);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint20);
            nextRootTemp2 = m_model->AddLimb(nextRootTemp2, handRBody, jointPtr);
        }

        { // hand_L
            ndMatrix tmp1(handLBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1] + 0.25f, tmp1[3][2], 1.0f); // offset
            //tmp1 = ndRollMatrix(180.0f * ndDegreeToRad) * tmp1;
            tmp1 = ndPitchMatrix(-180.0f * ndDegreeToRad) * tmp1;
            tmp1 = ndYawMatrix(-90.0f * ndDegreeToRad) * tmp1;
            //
            joint19 = new ndJointHinge(tmp1, handLBody->GetAsBodyKinematic(), avantbrasLBody->GetAsBodyKinematic());
            joint19->SetLimitState(true);
            joint19->SetLimits(-45.0f * ndDegreeToRad, 65.0f * ndDegreeToRad);
            ////joint19->SetAsSpringDamper(0.01f, 5.0f, 0.5f);
            //m_jointlist.push_back(joint19);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint19);
            nextRootTemp1 = m_model->AddLimb(nextRootTemp1, handLBody, jointPtr);
        }
#endif

#if 0
        // lower body parts
        { // hip_L
            ndMatrix tmp1(hipLBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0] + 0.4f, tmp1[3][1] + 0.05f, tmp1[3][2], 1.0f); // offset
            //
            joint3 = new ndJointHinge(tmp1, hipLBody->GetAsBodyKinematic(), bassinBody->GetAsBodyKinematic());
            joint3->SetLimitState(true);
            joint3->SetLimits(-165.0f * ndDegreeToRad, 1.0f * ndDegreeToRad);
            joint3->SetAsSpringDamper(0.01f, 25.0f, 1.0f);
            //
            //m_jointlist.push_back(joint3);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint3);
            nextRootTemp1 = m_model->AddLimb(modelRootNode, hipLBody, jointPtr);
        }

        { // hip_R
            ndMatrix tmp1(hipRBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0] - 0.4f, tmp1[3][1] + 0.05f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndPitchMatrix(-180.0f * ndDegreeToRad) * tmp1;
            tmp1 = ndYawMatrix(-180.0f * ndDegreeToRad) * tmp1;
            //
            joint4 = new ndJointHinge(tmp1, hipRBody->GetAsBodyKinematic(), bassinBody->GetAsBodyKinematic());
            joint4->SetLimitState(true);
            joint4->SetLimits(-165.0f * ndDegreeToRad, 1.0f * ndDegreeToRad);
            joint4->SetAsSpringDamper(0.01f, 25.0f, 1.0f);
            //
            //m_jointlist.push_back(joint4);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint4);
            nextRootTemp2 = m_model->AddLimb(modelRootNode, hipRBody, jointPtr);
        }

        { // cuisse_L
            ndMatrix tmp1(cuisseLBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0] + 0.0125f, tmp1[3][1] + 1.0f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //
            joint5 = new ndJointSpherical(tmp1, cuisseLBody->GetAsBodyKinematic(), hipLBody->GetAsBodyKinematic());
            joint5->SetConeLimit(145.0f * ndDegreeToRad);
            joint5->SetTwistLimits(-145.0f * ndDegreeToRad, 145.0f * ndDegreeToRad);
            joint5->SetAsSpringDamper(0.005f, 50.0f, 10.0f);
            //
            //m_jointlist.push_back(joint5);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint5);
            nextRootTemp1 = m_model->AddLimb(nextRootTemp1, cuisseLBody, jointPtr);
        }

        { // cuisse_R
            ndMatrix tmp1(cuisseRBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0] - 0.0125f, tmp1[3][1] + 1.0f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //tmp1 = ndYawMatrix(-180.0f * ndDegreeToRad) * tmp1;
            //
            joint6 = new ndJointSpherical(tmp1, cuisseRBody->GetAsBodyKinematic(), hipRBody->GetAsBodyKinematic());
            //joint6->SetLimitState(true);
            //joint6->SetLimits(-10.0f * ndDegreeToRad, 10.0f * ndDegreeToRad);
            joint6->SetConeLimit(145.0f * ndDegreeToRad);
            joint6->SetTwistLimits(-145.0f * ndDegreeToRad, 145.0f * ndDegreeToRad);
            joint6->SetAsSpringDamper(0.005f, 50.0f, 10.0f);
            //
            //m_jointlist.push_back(joint6);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint6);
            nextRootTemp2 = m_model->AddLimb(nextRootTemp2, cuisseRBody, jointPtr);
        }

        { // tibia_L
            ndMatrix tmp1(tibiaLBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1] + 0.7f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //
            joint7 = new ndJointHinge(tmp1, tibiaLBody->GetAsBodyKinematic(), cuisseLBody->GetAsBodyKinematic());
            joint7->SetLimitState(true);
            joint7->SetLimits(-165.0f * ndDegreeToRad, 2.0f * ndDegreeToRad);
            joint7->SetAsSpringDamper(0.01f, 2.5f, 0.25f);
            //
            //m_jointlist.push_back(joint7);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint7);
            nextRootTemp1 = m_model->AddLimb(nextRootTemp1, tibiaLBody, jointPtr);
        }

        { // tibia_R
            ndMatrix tmp1(tibiaRBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1] + 0.7f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //tmp1 = ndYawMatrix(-180.0f * ndDegreeToRad) * tmp1;
            //
            joint8 = new ndJointHinge(tmp1, tibiaRBody->GetAsBodyKinematic(), cuisseRBody->GetAsBodyKinematic());
            joint8->SetLimitState(true);
            joint8->SetLimits(-165.0f * ndDegreeToRad, 2.0f * ndDegreeToRad);
            joint8->SetAsSpringDamper(0.01f, 2.5f, 0.25f);
            //
            //m_jointlist.push_back(joint8);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint8);
            nextRootTemp2 = m_model->AddLimb(nextRootTemp2, tibiaRBody, jointPtr);
        }

        { // pied_L
            ndMatrix tmp1(piedLBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1], tmp1[3][2] - 0.15f, 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //
            joint9 = new ndJointHinge(tmp1, piedLBody->GetAsBodyKinematic(), tibiaLBody->GetAsBodyKinematic());
            joint9->SetLimitState(true);
            joint9->SetLimits(-2.0f * ndDegreeToRad, 65.0f * ndDegreeToRad);
            joint9->SetAsSpringDamper(0.01f, 25.0f, 1.0f);
            //
            //m_jointlist.push_back(joint9);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint9);
            nextRootTemp1 = m_model->AddLimb(nextRootTemp1, piedLBody, jointPtr);
        }

        { // orteille_R
            ndMatrix tmp1(orteilRBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1], tmp1[3][2] - 0.125f, 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //tmp1 = ndYawMatrix(-180.0f * ndDegreeToRad) * tmp1;
            //
            joint12 = new ndJointHinge(tmp1, orteilRBody->GetAsBodyKinematic(), piedRBody->GetAsBodyKinematic());
            joint12->SetLimitState(true);
            joint12->SetLimits(-15.0f * ndDegreeToRad, 45.0f * ndDegreeToRad);
            joint12->SetAsSpringDamper(0.01f, 100.0f, 5.0f);
            //
            //m_jointlist.push_back(joint12);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint12);
            nextRootTemp2 = m_model->AddLimb(nextRootTemp2, orteilRBody, jointPtr);
        }

        { // orteille_L
            ndMatrix tmp1(orteilLBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1], tmp1[3][2] - 0.125f, 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //
            joint11 = new ndJointHinge(tmp1, orteilLBody->GetAsBodyKinematic(), piedLBody->GetAsBodyKinematic());
            joint11->SetLimitState(true);
            joint11->SetLimits(-15.0f * ndDegreeToRad, 45.0f * ndDegreeToRad);
            joint11->SetAsSpringDamper(0.01f, 100.0f, 5.0f);
            //
            //m_jointlist.push_back(joint11);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint11);
            nextRootTemp1 = m_model->AddLimb(nextRootTemp1, orteilLBody, jointPtr);
        }

        { // pied_R
            ndMatrix tmp1(piedRBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1], tmp1[3][2] - 0.15f, 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            tmp1 = ndYawMatrix(-180.0f * ndDegreeToRad) * tmp1;
            //
            joint10 = new ndJointHinge(tmp1, piedRBody->GetAsBodyKinematic(), tibiaRBody->GetAsBodyKinematic());
            joint10->SetLimitState(true);
            joint10->SetLimits(-2.0f * ndDegreeToRad, 65.0f * ndDegreeToRad);
            joint10->SetAsSpringDamper(0.01f, 25.0f, 1.0f);
            //
            //m_jointlist.push_back(joint10);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint10);
            nextRootTemp2 = m_model->AddLimb(nextRootTemp2, piedRBody, jointPtr);
        }

#endif
    }

    std::vector<ndSharedPtr<ndBody>> m_bodypartlist;
    ndModelArticulation* const m_model;
};

//void ndBasicBipedDG(ndDemoEntityManager* const scene)
void ndBasicRagdoll(ndDemoEntityManager* const scene)
{
    ndSharedPtr<ndBody> floor(BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));

    ndModelArticulation* model = new ndModelArticulation();
    ndSharedPtr<ndModelNotify> controller(new ndDGController(scene, model));
    model->SetNotifyCallback(controller);

    ndPhysicsWorld* world = scene->GetWorld();
    world->AddModel(model);
    model->AddBodiesAndJointsToWorld();

    ndQuaternion rot;
    ndVector origin(-20.0f, 10.0f, 0.0f, 1.0f);
    scene->SetCameraMatrix(rot, origin);
}

#else

namespace ndRagdoll
{
	class ndDefinition
	{
		public:
		enum ndjointType
		{
			m_root,
			m_hinge,
			m_spherical,
			m_doubleHinge,
			m_effector
		};

		struct ndDampData
		{
			ndDampData()
				:m_spring(0.0f)
				,m_damper(0.25f)
				,m_regularizer(0.025f)
			{
			}

			ndDampData(ndFloat32 spring, ndFloat32 damper, ndFloat32 regularizer)
				:m_spring(spring)
				,m_damper(damper)
				,m_regularizer(regularizer)
			{
			}

			ndFloat32 m_spring;
			ndFloat32 m_damper;
			ndFloat32 m_regularizer;
		};

		struct ndJointLimits
		{
			ndFloat32 m_minTwistAngle;
			ndFloat32 m_maxTwistAngle;
			ndFloat32 m_coneAngle;
		};

		struct ndOffsetFrameMatrix
		{
			ndFloat32 m_pitch;
			ndFloat32 m_yaw;
			ndFloat32 m_roll;
		};

		char m_boneName[32];
		ndjointType m_limbType;
		ndFloat32 m_massWeight;
		ndJointLimits m_jointLimits;
		ndOffsetFrameMatrix m_frameBasics;
		ndDampData m_coneSpringData;
		ndDampData m_twistSpringData;
	};

	static ndDefinition ragdollDefinition[] =
	{
		{ "root", ndDefinition::m_root, 1.0f, {}, {} },

		{ "lowerback", ndDefinition::m_spherical, 1.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 0.0f } },
		{ "upperback", ndDefinition::m_spherical, 1.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		{ "lowerneck", ndDefinition::m_spherical, 1.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		{ "upperneck", ndDefinition::m_spherical, 1.0f,{ -60.0f, 60.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		
		{ "lclavicle", ndDefinition::m_spherical, 1.0f, { -60.0f, 60.0f, 80.0f }, { 0.0f, -60.0f, 0.0f } },
		{ "lhumerus", ndDefinition::m_hinge, 1.0f, { -0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "lradius", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
		
		{ "rclavicle", ndDefinition::m_spherical, 1.0f, { -60.0f, 60.0f, 80.0f }, { 0.0f, 60.0f, 0.0f } },
		{ "rhumerus", ndDefinition::m_hinge, 1.0f, { -0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "rradius", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
		
		{ "rhipjoint", ndDefinition::m_spherical, 1.0f, { -45.0f, 45.0f, 80.0f }, { 0.0f, -60.0f, 0.0f } },
		{ "rfemur", ndDefinition::m_hinge, 1.0f, { -0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "rtibia", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
		
		{ "lhipjoint", ndDefinition::m_spherical, 1.0f, { -45.0f, 45.0f, 80.0f }, { 0.0f, 60.0f, 0.0f } },
		{ "lfemur", ndDefinition::m_hinge, 1.0f, { -0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "ltibia", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },

		{ "", ndDefinition::m_root,{},{} },
	};

	class ndRagDollController : public ndModelNotify
	{ 
		public:
		ndRagDollController()
			:ndModelNotify()
		{
		}

		bool OnContactGeneration(const ndBodyKinematic* const, const ndBodyKinematic* const) override
		{
			// here the application can use filter to determine what body parts should collide.
			// this greatly improves performance because since articulated models in general 
			// do not self collide, but occationaly some parts do collide. 
			// for now we just return false (no collision)
			return false;
		}

		ndSharedPtr<ndBody> CreateBodyPart(
			ndDemoEntityManager* const scene,
			const ndSharedPtr<ndRenderSceneNode>& rootMesh,
			const ndRenderMeshLoader& loader, 
			const ndDefinition& definition,
			ndBodyDynamic* const parentBody)
		{
			ndMesh* const mesh(loader.m_mesh->FindByName(definition.m_boneName));
			ndSharedPtr<ndShapeInstance> shape(mesh->CreateCollisionFromChildren());
		
			ndAssert(rootMesh->FindByName(definition.m_boneName));
			ndSharedPtr<ndRenderSceneNode> bonePart(rootMesh->FindByName(definition.m_boneName)->GetSharedPtr());
			// create the rigid body that will make this body
			ndMatrix matrix(bonePart->CalculateGlobalTransform());
		
			ndSharedPtr<ndBody> body (new ndBodyDynamic());
			body->SetMatrix(matrix);
			body->GetAsBodyDynamic()->SetCollisionShape(**shape);
			body->GetAsBodyDynamic()->SetMassMatrix(1.0f, **shape);
			body->SetNotifyCallback(new ndDemoEntityNotify(scene, bonePart, parentBody));
			return body;
		}

		ndJointBilateralConstraint* ConnectBodyParts(ndBodyDynamic* const childBody, ndBodyDynamic* const parentBody, const ndDefinition& definition)
		{
			ndMatrix matrix(childBody->GetMatrix());
			ndDefinition::ndOffsetFrameMatrix frameAngle(definition.m_frameBasics);
			ndMatrix pinAndPivotInGlobalSpace(ndPitchMatrix(frameAngle.m_pitch * ndDegreeToRad) * ndYawMatrix(frameAngle.m_yaw * ndDegreeToRad) * ndRollMatrix(frameAngle.m_roll * ndDegreeToRad) * matrix);

			ndDefinition::ndJointLimits jointLimits(definition.m_jointLimits);

			switch (definition.m_limbType)
			{
				case ndDefinition::m_spherical:
				{
					ndJointSpherical* const joint = new ndJointSpherical(pinAndPivotInGlobalSpace, childBody, parentBody);
					joint->SetConeLimit(jointLimits.m_coneAngle * ndDegreeToRad);
					joint->SetTwistLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
					joint->SetAsSpringDamper(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
					return joint;
				}

				case ndDefinition::m_hinge:
				{
					ndJointHinge* const joint = new ndJointHinge(pinAndPivotInGlobalSpace, childBody, parentBody);
					joint->SetLimitState(true);
					joint->SetLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
					joint->SetAsSpringDamper(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
					return joint;
				}

				case ndDefinition::m_doubleHinge:
				{
					ndJointDoubleHinge* const joint = new ndJointDoubleHinge(pinAndPivotInGlobalSpace, childBody, parentBody);
					joint->SetLimitState0(true);
					joint->SetLimitState1(true);
					joint->SetLimits0(-30.0f * ndDegreeToRad, 30.0f * ndDegreeToRad);
					joint->SetLimits1(-45.0f * ndDegreeToRad, 45.0f * ndDegreeToRad);
					joint->SetAsSpringDamper0(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
					joint->SetAsSpringDamper1(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
					return joint;
				}

				default:
					ndAssert(0);
			}
			return nullptr;
		}

		void CalculateMassDistribution(ndModelArticulation* const ragdoll, ndFloat32 totalMass)
		{
			ndFixSizeArray<ndBodyDynamic*, 256> bodyArray;
			ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;
			if (ragdoll->GetRoot())
			{
				stack.PushBack(ragdoll->GetRoot());
				while (stack.GetCount())
				{
					ndInt32 index = stack.GetCount() - 1;
					ndModelArticulation::ndNode* const node = stack[index];
					stack.SetCount(index);

					bodyArray.PushBack(node->m_body->GetAsBodyDynamic());
					for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
					{
						stack.PushBack(child);
					}
				}
			}

			ndFloat32 totalVolume = 0.0f;
			for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
			{
				totalVolume += bodyArray[i]->GetCollisionShape().GetVolume();
			}
			ndFloat32 density = totalMass / totalVolume;

			for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
			{
				ndBodyDynamic* const body = bodyArray[i];
				ndFloat32 volume = bodyArray[i]->GetCollisionShape().GetVolume();
				ndFloat32 mass = density * volume;
				//ndTrace(("mass=%f  volume=%f\n", mass, volume));

				ndVector inertia(body->GetMassMatrix().Scale(mass));
				body->SetMassMatrix(inertia);
			}
		}

		void RagdollBuildScript(ndDemoEntityManager* const scene, const ndRenderMeshLoader& loader, const ndMatrix& location)
		{
			ndSharedPtr<ndRenderSceneNode> entityDuplicate(loader.m_renderMesh->Clone());
			entityDuplicate->SetTransform(location);
			entityDuplicate->SetTransform(location);
			scene->AddEntity(entityDuplicate);

			ndSharedPtr<ndBody> rootBody(CreateBodyPart(scene, entityDuplicate, loader, ragdollDefinition[0], nullptr));

			ndModelArticulation* const ragdoll = (ndModelArticulation*)GetModel();
			ndModelArticulation::ndNode* const modelRootNode = ragdoll->AddRootBody(rootBody);
			ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)modelRootNode->m_body->GetAsBodyKinematic()->GetNotifyCallback();

			struct StackData
			{
				ndModelArticulation::ndNode* parentBone;
				ndSharedPtr<ndRenderSceneNode> childEntity;
			};
			ndFixSizeArray<StackData, 256> stack;

			for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = notify->GetUserData()->GetChildren().GetFirst(); node; node = node->GetNext())
			{
				StackData data;
				data.parentBone = modelRootNode;
				data.childEntity = node->GetInfo();
				stack.PushBack(data);
			}

			while (stack.GetCount())
			{
				StackData data(stack.Pop());

				const char* const name = data.childEntity->m_name.GetStr();
				//ndTrace(("name: %s\n", name));
				for (ndInt32 i = 0; ragdollDefinition[i].m_boneName[0]; ++i)
				{
					const ndDefinition& definition = ragdollDefinition[i];

					if (!strcmp(definition.m_boneName, name))
					{
						ndBodyDynamic* const parentBody = data.parentBone->m_body->GetAsBodyDynamic();
						ndSharedPtr<ndBody> childBody(CreateBodyPart(scene, entityDuplicate, loader, definition, parentBody));

						//connect this body part to its parentBody with a rag doll joint
						ndSharedPtr<ndJointBilateralConstraint> joint(ConnectBodyParts(childBody->GetAsBodyDynamic(), parentBody, definition));

						// add this child body to the rad doll model.
						data.parentBone = ragdoll->AddLimb(data.parentBone, childBody, joint);
						break;
					}
				}
			
				for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = data.childEntity->GetChildren().GetFirst(); node; node = node->GetNext())
				{
					StackData childData;
					childData.parentBone = data.parentBone;
					childData.childEntity = node->GetInfo();
					stack.PushBack(childData);
				}
			}

			CalculateMassDistribution(ragdoll, ndFloat32(100.0f));
		}
	};

	ndSharedPtr<ndModelNotify> CreateRagdoll(ndDemoEntityManager* const scene, const ndRenderMeshLoader& loader, const ndMatrix& location)
	{
		// make a hierchical atriculate model
		ndSharedPtr<ndModel> model(new ndModelArticulation());
		
		// create a ragdoll controller 
		ndSharedPtr<ndModelNotify> controller(new ndRagDollController());
		model->SetNotifyCallback(controller);
		
		ndRagDollController* const ragdollController = (ndRagDollController*)*controller;
		ragdollController->RagdollBuildScript(scene, loader, location);
		
		ndWorld* const world = scene->GetWorld();
		world->AddModel(model);
		model->AddBodiesAndJointsToWorld();
		return controller;
	}
}

using namespace ndRagdoll;
void ndBasicRagdoll (ndDemoEntityManager* const scene)
{
	// build a floor
	//ndSharedPtr<ndBody> bodyFloor(BuildPlayground(scene));
	ndSharedPtr<ndBody> bodyFloor(BuildCompoundScene(scene, ndGetIdentityMatrix()));
	//ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marblecheckboard.png", 0.1f, true));

	class PlaceMatrix : public ndMatrix
	{
		public:
		PlaceMatrix(ndDemoEntityManager* const scene, ndFloat32 x, ndFloat32 y, ndFloat32 z)
			:ndMatrix(ndGetIdentityMatrix())
		{
			m_posit = FindFloor(*scene->GetWorld(), ndVector(x, y, z, ndFloat32 (1.0f)), 200.0f);
			m_posit.m_y += ndFloat32(10.0f);
		}
	};

	ndRenderMeshLoader loader(*scene->GetRenderer());
	//loader.ImportFbx(ndGetWorkingFileName("ragdoll.fbx"));
	loader.LoadMesh(ndGetWorkingFileName("ragdoll.nd"));

	ndMatrix playerMatrix(PlaceMatrix(scene, 0.0f, 0.0f, 0.0f));
	ndSharedPtr<ndModelNotify> modelNotity(CreateRagdoll(scene, loader, playerMatrix));

	{
#if 1
		// add few more rag dolls
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 0.0f, 0.0f, 0.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 3.0f, 0.0f, 0.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 5.0f, 0.0f, 0.0f));

		CreateRagdoll(scene, loader, PlaceMatrix(scene, 0.0f, 0.0f, 10.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 3.0f, 0.0f, 10.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 5.0f, 0.0f, 10.0f));

		CreateRagdoll(scene, loader, PlaceMatrix(scene, 0.0f, 0.0f, -10.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 3.0f, 0.0f, -10.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 5.0f, 0.0f, -10.0f));
#endif
	}

	ndFloat32 angle = ndFloat32(90.0f * ndDegreeToRad);
	playerMatrix = ndYawMatrix(angle) * playerMatrix;
	playerMatrix.m_posit += playerMatrix.m_front.Scale (-15.0f);
	playerMatrix.m_posit = FindFloor(*scene->GetWorld(), playerMatrix.m_posit, 200.0f);
	scene->SetCameraMatrix(playerMatrix, playerMatrix.m_posit);
}
#endif