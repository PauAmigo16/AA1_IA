#pragma once
#include "Vector2D.h"
#include "Agent.h"

class SteeringBehaviour
{
private:


public:
	Vector2D Seek(Vector2D targetPos, Agent agent);
	Vector2D Flee(Vector2D targetPos, Agent agent);
	Vector2D Arrive(Vector2D targetPos, Agent agent, float slowingRadius);
	Vector2D Pursue(Vector2D targetPos, Vector2D targetVel, Agent agent);
	Vector2D Evade(Vector2D targetPos, Vector2D targetVel, Agent agent);
	Vector2D Wander(Vector2D circlecenter, float circleRadius, float wanderOffset);
};

