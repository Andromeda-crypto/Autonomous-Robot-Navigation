#pragma once

#include "physics/RigidBody.h"
#include "physics/Manifold.h"

void resolveCollision(RigidBody& a, RigidBody& b,const Manifold& m);

