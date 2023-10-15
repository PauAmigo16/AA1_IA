#include "SteeringBehaviour.h"

Vector2D SteeringBehaviour::Seek(Vector2D targetPos, Agent agent)
{
	Vector2D desiredVel = targetPos - agent.getPosition();
	desiredVel.Normalize();
	desiredVel *= agent.getMaxVelocity();
	Vector2D steeringForce = (desiredVel - agent.getVelocity());
	steeringForce /= agent.getMaxVelocity();
	return steeringForce * agent.getMaxForce();
}

Vector2D SteeringBehaviour::Flee(Vector2D targetPos, Agent agent)
{
	Vector2D desiredVel = agent.getPosition() - targetPos;
	desiredVel.Normalize();
	desiredVel *= agent.getMaxVelocity();
	Vector2D steeringForce = (desiredVel - agent.getVelocity());
	steeringForce /= agent.getMaxVelocity();
	return steeringForce * agent.getMaxForce();
}

Vector2D SteeringBehaviour::Arrive(Vector2D targetPos, Agent agent, float slowingRadius)
{
	float distanceToTarget = Vector2D::Distance(targetPos, agent.getPosition());

	if (distanceToTarget > slowingRadius)
		return Seek(targetPos, agent);
	else
	{
		float factor = distanceToTarget / slowingRadius;
		Vector2D desiredVel = agent.getPosition() - targetPos;
		desiredVel.Normalize();
		desiredVel *= agent.getMaxVelocity() * factor;
		Vector2D steeringForce = (desiredVel - agent.getVelocity());
		steeringForce /= agent.getMaxVelocity() * factor;
		return steeringForce * agent.getMaxForce();
	}
}

Vector2D SteeringBehaviour::Pursue(Vector2D targetPos, Vector2D targetVel, Agent agent)
{
	float t = Vector2D::Distance(targetPos, agent.getPosition()) / agent.getVelocity().Length();
	Vector2D predictedTarget = targetPos + targetVel * t;
	return Seek(predictedTarget, agent);
}

Vector2D SteeringBehaviour::Evade(Vector2D targetPos, Vector2D targetVel, Agent agent)
{
	float t = Vector2D::Distance(targetPos, agent.getPosition()) / agent.getVelocity().Length();
	Vector2D predictedTarget = targetPos + targetVel * t;
	return Flee(predictedTarget, agent);
}

Vector2D SteeringBehaviour::Wander(Vector2D circlecenter, float circleRadius, float wanderOffset)
{
	return NULL;
}
