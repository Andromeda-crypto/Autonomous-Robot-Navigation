#pragma once

#include "physics/RigidBody.h"
#include "physics/Manifold.h"
#include "physics/Box.h"

Manifold checkCircleCircle(const RigidBody& a, const RigidBody& b);
Manifold checkCircleAABB(const RigidBody& circle, const Box& box);