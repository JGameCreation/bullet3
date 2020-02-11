/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "SimpleBox.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "json.hpp"
#include <fstream>
#include <iostream>

using json = nlohmann::json;

struct SimpleBoxExample : public CommonRigidBodyBase
{
	SimpleBoxExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~SimpleBoxExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 41;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0.46, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void SimpleBoxExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));
	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	btCompoundShape* compoundShape = new btCompoundShape();

	json j;

	try
	{
		std::ifstream stream("complexObjectColliders.json");
		json data = json::parse(stream);
		j = data["array"];

		/*for (json::iterator it = j.begin(); it != j.end(); ++it)
		{
			btCollisionShape* shape;
			if (it->find("halfExtents") != it->end())
			{
				shape = createBoxShape(btVector3((*it)["halfExtents"]["x"], (*it)["halfExtents"]["y"], (*it)["halfExtents"]["z"]));
			}
			else if (it->find("vertices") != it->end())
			{
				btConvexHullShape* convexShape = new btConvexHullShape();
				for (json::iterator point = j["vertices"].begin(); point != j["vertices"].end(); ++point)
					convexShape->addPoint(btVector3((*point)["x"], (*point)["y"], (*point)["z"]), false);
				convexShape->recalcLocalAabb();
				convexShape->optimizeConvexHull();
				shape = convexShape;
			}
			else
			{
				throw std::runtime_error("Invalid collider json file");
			}

			if (it->find("position") != it->end() && it->find("rotation") != it->end())
			{
				auto quat = (*it)["rotation"];
				auto pos = (*it)["position"];
				btTransform t(btQuaternion(quat["x"], quat["y"], quat["z"], quat["w"]), btVector3(pos["x"], pos["y"], pos["z"]));
				compoundShape->addChildShape(t, shape);
			}
			else
			{
				throw std::runtime_error("Invalid collider json file");
			}
		}*/
	}
	catch (json::parse_error& e)
	{
		// output exception information
		std::cout << "message: " << e.what() << '\n'
				  << "exception id: " << e.id << '\n'
				  << "byte position of error: " << e.byte << std::endl;
	}

	m_collisionShapes.push_back(compoundShape);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(1.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		compoundShape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(btVector3(
		btScalar(0),
		btScalar(20),
		btScalar(0)));
	createRigidBody(mass, startTransform, compoundShape);

	/*for (int i=0; i < 2; i++) {
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		btBoxShape* colShapeXZ = createBoxShape(btVector3(2.5, .2, 2.5));
		btBoxShape* colShapeXY = createBoxShape(btVector3(2.5, 2.5, .2));
		btBoxShape* colShapeYZ = createBoxShape(btVector3(.2, 2.5, 2.5));

		btTransform t;
		btCompoundShape* colShapeBox = new btCompoundShape();
		t.setIdentity();
		t.setOrigin(btVector3(0, -2.5 + .2 + 3 + 6 * i, 0));
		colShapeBox->addChildShape(t, colShapeXZ);
		t.setIdentity();
		t.setOrigin(btVector3(0, 2.5 - .2 + 3 + 6 * i, 0));
		colShapeBox->addChildShape(t, colShapeXZ);
		t.setIdentity();
		t.setOrigin(btVector3(-2.5 + .2, 0 + 3 + 6 * i, 0));
		colShapeBox->addChildShape(t, colShapeYZ);
		t.setIdentity();
		t.setOrigin(btVector3(2.5 - .2, 0 + 3 + 6 * i, 0));
		colShapeBox->addChildShape(t, colShapeYZ);
		t.setIdentity();
		t.setOrigin(btVector3(0, 0 + 3 + 6 * i, -2.5 + .2));
		colShapeBox->addChildShape(t, colShapeXY);
		t.setIdentity();
		t.setOrigin(btVector3(0, 0 + 3 + 6 * i, 2.5 - .2));
		colShapeBox->addChildShape(t, colShapeXY);

		m_collisionShapes.push_back(colShapeBox);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShapeBox->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(0),
			btScalar(20),
			btScalar(0)));
		createRigidBody(mass, startTransform, colShapeBox);
	}*/

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void SimpleBoxExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* ET_SimpleBoxCreateFunc(CommonExampleOptions& options)
{
	return new SimpleBoxExample(options.m_guiHelper);
}
