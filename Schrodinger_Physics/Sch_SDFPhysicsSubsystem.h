// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Sch_SDFPhysicsSubsystem.generated.h"

/**
 * 
 */
UCLASS()
class DEFAULT56_PROJECT_API USch_SDFPhysicsSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()
public:
	virtual void Tick(float DeltaTime) override;

	virtual TStatId GetStatId() const override {RETURN_QUICK_DECLARE_CYCLE_STAT(USchPhysicsSubsystem, STATGROUP_Tickables);}

	//For Registering
	void Register(class USch_SDFComponent* C);
	void Unregister(class USch_SDFComponent* C);

	//For SubSteping
	void Step(float dt);

	void SolvePairs(const TArray<TPair<USch_SDFComponent*, USch_SDFComponent*>>& pairs, float dt);


	//Spatial Accel
	void Broadphase(TArray<TPair<USch_SDFComponent*, USch_SDFComponent*>>& outPairs, float dt);
	FBox ComputeWorldAABB_Swept(USch_SDFComponent* C, double dt, float wiggle, bool bUseSwept);

public:
	//For Registering
	TArray<TWeakObjectPtr<USch_SDFComponent>> GSchSdfRegistry;

	//For SubSteping
	int32 MaxSteps = 1;

	int32  SolverIterations = 1;

	float FixedStepTime = 0.00833; //

	float StackedTime;

	
	//For Collision
	float  BroadphaseWiggle = 2.0f;	//Wiggle Room

	bool   bUseSweptAabb = false;	//AABB Sweep

	float SpeculativeContactSlop = 5.0f; // cm
	

};
