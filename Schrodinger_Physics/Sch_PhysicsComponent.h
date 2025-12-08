// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Schrodinger_Physics/Sch_SDF.h"

#include "Sch_PhysicsComponent.generated.h"


UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class USch_PhysicsComponent : public UActorComponent
{
	GENERATED_BODY()
public:
	USch_PhysicsComponent();


	// --- Simulation params
	UPROPERTY(EditAnywhere, Category = "Physics") float FixedDt = 1.f / 90.f;
	UPROPERTY(EditAnywhere, Category = "Physics") FVector Gravity = FVector(0, 0, -980.f);
	UPROPERTY(EditAnywhere, Category = "Physics") float Restitution = 0.4f; // 0=no bounce, 1=perfect


	// --- Well (interior) as AABox SDF (negative inside)
	UPROPERTY(EditAnywhere, Category = "Well") FVector WellCenter = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, Category = "Well") FVector WellHalfExtents = FVector(300, 300, 200);


	// --- Capsule rigid body (only translation for MVP)
	UPROPERTY(EditAnywhere, Category = "Capsule") float CapsuleHalfLength = 40.f; // along local +Z
	UPROPERTY(EditAnywhere, Category = "Capsule") float CapsuleRadius = 18.f;
	UPROPERTY(EditAnywhere, Category = "Capsule") FVector StartPosition = FVector(0, 0, 0);
	UPROPERTY(EditAnywhere, Category = "Capsule") FQuat StartRotation = FQuat::Identity; // axis = local Z


	UPROPERTY(EditAnywhere, Category = "Debug") bool bDrawDebug = true;
	UPROPERTY(EditAnywhere, Category = "Debug") int32 CapsuleSamples = 9; // along axis for contact search


protected:
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;


private:
	float Accumulator = 0.f;


	// Rigid state (translation-only for now)
	FVector P; // position (capsule midpoint)
	FQuat Q; // orientation (kept constant for MVP)
	FVector V; // linear velocity

	// Cached SDF of well interior
	SchSDF::SDF WellInterior; // negative inside, positive outside

	// MVP physics
	void PhysicsStep(float Dt);
	void BuildWellSDF();
	void DrawDebugNow();
};