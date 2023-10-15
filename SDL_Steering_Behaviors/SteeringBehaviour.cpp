#include "SteeringBehaviour.h"

Vector2D SteeringBehaviour::Seek(Vector2D targetPos, Agent* agent)
{
	Vector2D desiredVel = targetPos - agent->getPosition();
	desiredVel.Normalize();
	desiredVel *= agent->getMaxVelocity();
	Vector2D steeringForce = (desiredVel - agent->getVelocity());
	steeringForce /= agent->getMaxVelocity();
	return steeringForce * agent->getMaxForce();
}

Vector2D SteeringBehaviour::Flee(Vector2D targetPos, Agent* agent)
{
	Vector2D desiredVel = agent->getPosition() - targetPos;
	desiredVel.Normalize();
	desiredVel *= agent->getMaxVelocity();
	Vector2D steeringForce = (desiredVel - agent->getVelocity());
	steeringForce /= agent->getMaxVelocity();
	return steeringForce * agent->getMaxForce();
}

Vector2D SteeringBehaviour::Arrive(Vector2D targetPos, Agent* agent, float slowingRadius)
{
	float distanceToTarget = Vector2D::Distance(targetPos, agent->getPosition());

	if (distanceToTarget > slowingRadius)
		return Seek(targetPos, agent);
	else
	{
		float factor = distanceToTarget / slowingRadius;
		Vector2D desiredVel = agent->getPosition() - targetPos;
		desiredVel.Normalize();
		desiredVel *= agent->getMaxVelocity() * factor;
		Vector2D steeringForce = (desiredVel - agent->getVelocity());
		steeringForce /= agent->getMaxVelocity() * factor;
		return steeringForce * agent->getMaxForce();
	}
}

Vector2D SteeringBehaviour::Pursue(Vector2D targetPos, Vector2D targetVel, Agent* agent)
{
	float t = Vector2D::Distance(targetPos, agent->getPosition()) / agent->getVelocity().Length();
	Vector2D predictedTarget = targetPos + targetVel * t;
	return Seek(predictedTarget, agent);
}

Vector2D SteeringBehaviour::Evade(Vector2D targetPos, Vector2D targetVel, Agent* agent)
{
	float t = Vector2D::Distance(targetPos, agent->getPosition()) / agent->getVelocity().Length();
	Vector2D predictedTarget = targetPos + targetVel * t;
	return Flee(predictedTarget, agent);
}

Vector2D SteeringBehaviour::Wander(Vector2D circlecenter, float circleRadius, float wanderOffset, Agent* agent)
{
	return NULL;
}

Vector2D SteeringBehaviour::WeightedBlending(std::vector<STEERING_TYPE> behaviours, std::vector<float> weights, std::vector<Vector2D> targets, Agent* agent, float radius, Vector2D targetVel, float wanderOffset, float dt)
{
	Vector2D steeringForce;

	for (int i=0; i<behaviours.size();i++)
	{
		Vector2D currentBehaviour;
		switch (behaviours[i])
		{
		case STEERING_TYPE::SEEK:
			currentBehaviour = Seek(targets[i], agent);
			break;
		case STEERING_TYPE::FLEE:
			currentBehaviour = Flee(targets[i], agent);
			break;
		case STEERING_TYPE::ARRIVE:
			currentBehaviour = Arrive(targets[i],agent, radius);
			break;
		case STEERING_TYPE::PURSUE:
			currentBehaviour = Pursue(targets[i], targetVel,  agent);
			break;
		case STEERING_TYPE::EVADE:
			currentBehaviour = Evade(targets[i],targetVel, agent);
			break;
		case STEERING_TYPE::WANDER:
			currentBehaviour = Wander(targets[i], radius, wanderOffset, agent);
			break;
		default:
			break;
		}
		steeringForce += currentBehaviour * weights[i];
	}

	Vector2D acceleration = steeringForce / agent->getMass();
	
	agent->setVelocity(agent->getVelocity() + acceleration * dt);

	if (agent->getVelocity().Length() > agent->getMaxVelocity()) {
		agent->setVelocity(agent->getVelocity().Normalize() * agent->getMaxVelocity());
	}
}

void SteeringBehaviour::Flocking(std::vector<Agent*> flock, float separationWeigth, float cohesionWeigth, float alignmentWeigth, Vector2D flockTarget, float dt)
{
	for (int i=0;i<flock.size();i++) 
	{
		Vector2D SteeringForce;
		Vector2D separationForce;
		Vector2D cohesionForce;
		for (int j = 0; j < flock.size(); j++) {
			if (i == j)
				continue;
			separationForce += Flee(flock[j]->getPosition(), flock[i]);
			cohesionForce += Seek(flock[j]->getPosition(), flock[i]);
		}
		SteeringForce += Seek(flockTarget, flock[i])*alignmentWeigth + separationForce * separationWeigth + cohesionForce * cohesionWeigth;

		Vector2D acceleration = SteeringForce / flock[i]->getMass();

		flock[i]->setVelocity(flock[i]->getVelocity() + acceleration * dt);

		if (flock[i]->getVelocity().Length() > flock[i]->getMaxVelocity()) {
			flock[i]->setVelocity(flock[i]->getVelocity().Normalize() * flock[i]->getMaxVelocity());
		}
	}
}
