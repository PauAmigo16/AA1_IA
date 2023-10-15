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

	if (distanceToTarget < slowingRadius && distanceToTarget != 0.f) {
		float factor = distanceToTarget / slowingRadius;
		Vector2D desiredVel = agent->getPosition() - targetPos;
		desiredVel.Normalize();
		desiredVel *= agent->getMaxVelocity() * factor;
		Vector2D steeringForce = (desiredVel - agent->getVelocity());
		steeringForce /= agent->getMaxVelocity() * factor;
		return steeringForce * agent->getMaxForce();
	}
	return Seek(targetPos, agent);
}

Vector2D SteeringBehaviour::Pursue(Vector2D targetPos, Vector2D targetVel, Agent* agent)
{
	if (agent->getVelocity().Length() != 0) {
		float t = Vector2D::Distance(targetPos, agent->getPosition()) / agent->getVelocity().Length();
		Vector2D predictedTarget = targetPos + targetVel * t;
		return Seek(predictedTarget, agent);
	}
	return Seek(targetPos, agent);
}

Vector2D SteeringBehaviour::Evade(Vector2D targetPos, Vector2D targetVel, Agent* agent)
{
	if (agent->getVelocity().Length() != 0) {
		float t = Vector2D::Distance(targetPos, agent->getPosition()) / agent->getVelocity().Length();
		Vector2D predictedTarget = targetPos + targetVel * t;
		return Flee(predictedTarget, agent);
	}
	return Flee(targetPos, agent);
}

Vector2D SteeringBehaviour::Wander(Vector2D circlecenter, float circleRadius, float wanderOffset, Agent* agent)
{
	Vector2D target;
	Vector2D dist = Vector2D::Distance(target, agent->getPosition());
	float angle = rand() % 360;

	circlecenter = agent->getPosition() + agent->getVelocity().Normalize() * wanderOffset;

	target.x = circlecenter.x + wanderOffset * sin(angle);
	target.y = circlecenter.y + wanderOffset * cos(angle);

	agent->setTarget(target);
	Vector2D desiredV = target - agent->getPosition();
	desiredV = desiredV.Normalize();
	desiredV *= agent->getMaxVelocity();
	Vector2D steeringForce = desiredV - agent->getVelocity();
	steeringForce /= agent->getMaxVelocity();
	steeringForce *= agent->getMaxForce();;
	return steeringForce;
}

Vector2D SteeringBehaviour::WeightedBlending(std::vector<STEERING_TYPE> behaviours, std::vector<float> weights, Vector2D targets, Agent* agent, float radius, Vector2D targetVel, float wanderOffset, float dt)
{
	Vector2D steeringForce;

	for (int i = 0; i < behaviours.size(); i++)
	{
		Vector2D currentBehaviour;
		switch (behaviours[i])
		{
		case STEERING_TYPE::SEEK:
			currentBehaviour = Seek(targets, agent);
			break;
		case STEERING_TYPE::FLEE:
			currentBehaviour = Flee(targets, agent);
			break;
		case STEERING_TYPE::ARRIVE:
			currentBehaviour = Arrive(targets, agent, radius);
			break;
		case STEERING_TYPE::PURSUE:
			currentBehaviour = Pursue(targets, targetVel, agent);
			break;
		case STEERING_TYPE::EVADE:
			currentBehaviour = Evade(targets, targetVel, agent);
			break;
		case STEERING_TYPE::WANDER:
			currentBehaviour = Wander(targets, radius, wanderOffset, agent);
			break;
		default:
			break;
		}
		steeringForce += currentBehaviour * weights[i];
	}

	Vector2D acceleration = steeringForce / agent->getMass();

	if (agent->getVelocity().Length() > agent->getMaxVelocity()) {
		return agent->getVelocity().Normalize() * agent->getMaxVelocity();
	}

	return agent->getVelocity() + acceleration * dt;
}

Vector2D SteeringBehaviour::PrioritizedWeightedSum(std::vector<STEERING_TYPE> behaviours, std::vector<float> weights, Vector2D targets, Agent* agent, float radius, Vector2D targetVel, float wanderOffset, float dt)
{
	int prioritizedSB = -1;//això ens dira si algun behaviour te prioritat
	for (int i = 0; i < weights.size(); i++) {
		if (weights[i] > 0.7) //considerem que mes de 0.7 de prioritat es una força significativa
			prioritizedSB = i;
	}
	if (prioritizedSB >= 0) {
		switch (behaviours[prioritizedSB])
		{
		case STEERING_TYPE::SEEK:
			return Seek(targets, agent);
			break;
		case STEERING_TYPE::FLEE:
			return Flee(targets, agent);
			break;
		case STEERING_TYPE::ARRIVE:
			return Arrive(targets, agent, radius);
			break;
		case STEERING_TYPE::PURSUE:
			return Pursue(targets, targetVel, agent);
			break;
		case STEERING_TYPE::EVADE:
			return Evade(targets, targetVel, agent);
			break;
		case STEERING_TYPE::WANDER:
			return Wander(targets, radius, wanderOffset, agent);
			break;
		default:
			break;
		}
	}
	return WeightedBlending(behaviours, weights, targets, agent, radius, targetVel, wanderOffset, dt);
}

void SteeringBehaviour::Flocking(std::vector<Agent*> flock, float separationWeigth, float cohesionWeigth, float alignmentWeigth, Vector2D flockTarget, float dt)
{
	for (int i = 0; i < flock.size(); i++)
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
		SteeringForce += Seek(flockTarget, flock[i]) * alignmentWeigth + separationForce * separationWeigth + cohesionForce * cohesionWeigth;

		Vector2D acceleration = SteeringForce / flock[i]->getMass();

		flock[i]->setVelocity(flock[i]->getVelocity() + acceleration * dt);

		if (flock[i]->getVelocity().Length() > flock[i]->getMaxVelocity()) {
			flock[i]->setVelocity(flock[i]->getVelocity().Normalize() * flock[i]->getMaxVelocity());
		}
	}
}
