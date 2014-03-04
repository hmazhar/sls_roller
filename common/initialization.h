#ifndef CHASSETHELPER_INIT_H
#define CHASSETHELPER_INIT_H

#include "common.h"

void InitObject(ChSharedBodyPtr &body, double mass, ChVector<> pos, ChQuaternion<> rot, ChSharedPtr<ChMaterialSurface> &material, bool do_collide = true, bool is_fixed = false, int collision_family = 2, int do_not_collide_with = 4) {
    body->SetMass(mass);
    body->SetPos(pos);
    body->SetRot(rot);
    body->SetCollide(do_collide);
    body->SetBodyFixed(is_fixed);
    body->SetMaterialSurface(material);
    body->GetCollisionModel()->ClearModel();
    body->GetCollisionModel()->SetFamily(collision_family);
    body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(do_not_collide_with);
    //body->SetLimitSpeed(true);
    //body->SetUseSleeping(true);
}

void InitObject(ChSharedBodyPtr &body, double mass, ChVector<> pos, ChQuaternion<> rot, bool do_collide = true, bool is_fixed = false, int collision_family = 2, int do_not_collide_with = 4) {
    body->SetMass(mass);
    body->SetPos(pos);
    body->SetRot(rot);
    body->SetCollide(do_collide);
    body->SetBodyFixed(is_fixed);
    body->GetCollisionModel()->ClearModel();
    body->GetCollisionModel()->SetFamily(collision_family);
    body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(do_not_collide_with);
    //body->SetLimitSpeed(true);
    //body->SetUseSleeping(true);
}

void AddCollisionGeometry(ChSharedBodyPtr &body, ShapeType type, ChVector<> dimension, ChVector<> local_position = ChVector<>(0,0,0), ChQuaternion<> local_rotation = ChQuaternion<>(1,0,0,0)) {
    ChMatrix33<> rotation = local_rotation;
    ChCollisionModel *model = (ChCollisionModel *) body->GetCollisionModel();
    if (type == SPHERE) {
        model->AddSphere(dimension.x, local_position);
        ChSharedPtr<ChSphereShape> sphere_shape = ChSharedPtr<ChAsset>(new ChSphereShape);
        sphere_shape->GetSphereGeometry().rad = dimension.x;
        sphere_shape->Pos = local_position;
        sphere_shape->Rot = rotation;
        body->GetAssets().push_back(sphere_shape);
    } else if (type == ELLIPSOID) {
        model->AddEllipsoid(dimension.x, dimension.y, dimension.z, local_position, rotation);
        ChSharedPtr<ChEllipsoidShape> ellipsoid_shape = ChSharedPtr<ChAsset>(new ChEllipsoidShape);
        ellipsoid_shape->GetEllipsoidGeometry().rad = dimension;
        ellipsoid_shape->Pos = local_position;
        ellipsoid_shape->Rot = rotation;
        body->GetAssets().push_back(ellipsoid_shape);
    } else if (type == BOX) {
        model->AddBox(dimension.x, dimension.y, dimension.z, local_position, rotation);

        ChSharedPtr<ChBoxShape> box_shape = ChSharedPtr<ChAsset>(new ChBoxShape);
        box_shape->Pos = local_position;
        box_shape->Rot = rotation;
        box_shape->GetBoxGeometry().Size = dimension;
        body->GetAssets().push_back(box_shape);

    } else if (type == CYLINDER) {
        model->AddCylinder(dimension.x, dimension.y, dimension.z, local_position, rotation);
        ChSharedPtr<ChCylinderShape> cylinder_shape = ChSharedPtr<ChAsset>(new ChCylinderShape);
        cylinder_shape->Pos = local_position;
        cylinder_shape->Rot = rotation;
        cylinder_shape->GetCylinderGeometry().rad = dimension.x;
        cylinder_shape->GetCylinderGeometry().p1 = ChVector<>(0, dimension.y / 2.0, 0);
        cylinder_shape->GetCylinderGeometry().p2 = ChVector<>(0, -dimension.y / 2.0, 0);

        body->GetAssets().push_back(cylinder_shape);
    }
    else if (type == CONE) {
        model->AddCone(dimension.x, dimension.y, dimension.z, local_position, rotation);
        ChSharedPtr<ChConeShape> cone_shape = ChSharedPtr<ChAsset>(new ChConeShape);
        cone_shape->Pos = local_position;
        cone_shape->Rot = rotation;
        cone_shape->GetConeGeometry().rad = Vector(dimension.x, dimension.y, dimension.x);
        body->GetAssets().push_back(cone_shape);
    }
}
void AddCollisionGeometryTriangleMesh(ChSharedBodyPtr &body, string file_name, ChVector<> local_position = ChVector<>(0,0,0), ChQuaternion<> local_rotation = ChQuaternion<>(1,0,0,0)) {
    ChMatrix33<> rotation = local_rotation;
    ChCollisionModel *model = (ChCollisionModel *) body->GetCollisionModel();
    ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(file_name, false, false);
    for (int i = 0; i < trimesh.m_vertices.size(); i++) {
        trimesh.m_vertices[i] += local_position;
    }
    model->AddTriangleMesh(trimesh, false, false, local_position, rotation);
    ChSharedPtr<ChTriangleMeshShape> trimesh_shape = ChSharedPtr<ChAsset>(new ChTriangleMeshShape);
    trimesh_shape->SetColor(ChColor(1, 0, 0));
    trimesh_shape->SetMesh(trimesh);
    body->GetAssets().push_back(trimesh_shape);
}

template<class S> void FinalizeObject(ChSharedBodyPtr body, S *chrono_system) {
    body->GetCollisionModel()->BuildModel();
    chrono_system->AddBody(body);
}
#endif
