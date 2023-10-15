#pragma once
#include "Vector2D.h"
#include "Agent.h"
#include <vector>

enum class STEERING_TYPE 
{
	SEEK,
	FLEE,
	ARRIVE,
	PURSUE,
	EVADE,
	WANDER
};

class SteeringBehaviour
{
public:
	Vector2D Seek(Vector2D targetPos, Agent* agent);
	Vector2D Flee(Vector2D targetPos, Agent* agent);
	Vector2D Arrive(Vector2D targetPos, Agent* agent, float slowingRadius);
	Vector2D Pursue(Vector2D targetPos, Vector2D targetVel, Agent* agent);
	Vector2D Evade(Vector2D targetPos, Vector2D targetVel, Agent* agent);
	Vector2D Wander(Vector2D circlecenter, float circleRadius, float wanderOffset, Agent* agent);

	//Behaviours, weights and target have to be in order. Ex: targets[0] has behaviour[0] with weight[0]. Radius will be used for arrive and wander, targetVel will be used for pursue and evade and wanderOffset will be used for wander
	Vector2D WeightedBlending(std::vector<STEERING_TYPE> behaviours, std::vector<float> weights, std::vector<Vector2D> targets, Agent* agent, float radius, Vector2D targetVel, float wanderOffset, float dt);
	void Flocking(std::vector<Agent*> flock, float separationWeigth, float cohesionWeigth, float alignmentWeigth, Vector2D flockTarget, float dt);
};