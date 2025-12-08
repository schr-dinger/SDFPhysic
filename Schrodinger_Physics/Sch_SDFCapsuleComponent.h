// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Schrodinger_Physics/Sch_SDF.h"
#include "Schrodinger_Physics/Sch_SDFStageComponent.h"

#include "Sch_SDFCapsuleComponent.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class DEFAULT56_PROJECT_API USch_SDFCapsuleComponent : public UActorComponent
{
	GENERATED_BODY()
public:
	USch_SDFCapsuleComponent();

	// Simulation
	UPROPERTY(EditAnywhere, Category = "Physics") float FixedDt = 1.f / 90.f;
	UPROPERTY(EditAnywhere, Category = "Physics") FVector Gravity = FVector(0, 0, -980.f);
	UPROPERTY(EditAnywhere, Category = "Physics") float Restitution = 0.4f;
	UPROPERTY(EditAnywhere, Category = "Physics") float TangentialDamping = 0.02f; // 0~1

	UPROPERTY(EditAnywhere, Category = "Physics") float AngularDamping = 0.05f;
	UPROPERTY(EditAnywhere, Category = "Physics") float LinearDamping = 0.01f;
	UPROPERTY(EditAnywhere, Category = "Physics") float StaticFriction = 0.6f;
	UPROPERTY(EditAnywhere, Category = "Physics") float DynamicFriction = 0.4f;

	// Capsule shape (local Z axis)
	UPROPERTY(EditAnywhere, Category = "Capsule") float HalfLength = 40.f; // cylinder half-length h
	UPROPERTY(EditAnywhere, Category = "Capsule") float Radius = 18.f;

	UPROPERTY(EditAnywhere, Category = "Capsule") float Mass = 1.0f; // 0 => kinematic

	// Initial state (local to this component)
	UPROPERTY(EditAnywhere, Category = "Initial") FVector StartPosition = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, Category = "Initial") FQuat StartRotation = FQuat::Identity;
	UPROPERTY(EditAnywhere, Category = "Initial") FVector InitialVelocity = FVector(220, 160, 280);
	UPROPERTY(EditAnywhere, Category = "Initial") FVector InitialAngularVelocity = FVector(0, 0, 0);
	// Which SDF world to collide against (if null, auto-find first in level)
	UPROPERTY(EditAnywhere, Category = "Collision") TObjectPtr<USch_SDFStageComponent> SDFWorld = nullptr;

	// Debug
	UPROPERTY(EditAnywhere, Category = "Debug") bool bDrawDebug = true;
	UPROPERTY(EditAnywhere, Category = "Debug") bool bDrawContact = true;

protected:
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;


private:
	float Accumulator = 0.f;
	FVector P; FQuat Q; FVector V; FVector W; // world-space state

	float InvMass = 1.f;
	FVector InertiaInvLocal = FVector::ZeroVector;


	void RecomputeInertia();
	void IntegrateOrientation(float Dt);
	FVector WorldInvInertiaMul(const FVector & v) const;
	void ApplyImpulseAtPoint(const FVector & J, const FVector & C);

	void AutoFindSDFWorld();
	void PhysicsStep(float Dt);
	void DrawDebugNow();
};