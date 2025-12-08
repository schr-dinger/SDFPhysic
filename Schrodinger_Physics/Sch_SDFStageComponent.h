// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Schrodinger_Physics/Sch_SDF.h"

#include "Sch_SDFStageComponent.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class DEFAULT56_PROJECT_API USch_SDFStageComponent : public UActorComponent
{
	GENERATED_BODY()
public:
	USch_SDFStageComponent();

	// Interior cavity half-extents (the playable volume inside the well)
	UPROPERTY(EditAnywhere, Category = "Well") FVector InnerHalfExtents = FVector(300, 300, 200);


	// Wall/floor thickness in world units (full thickness)
	UPROPERTY(EditAnywhere, Category = "Well") float WallThickness = 20.f;


	// Add ceiling if needed
	UPROPERTY(EditAnywhere, Category = "Well") bool bHasCeiling = false;


	// Debug draw
	UPROPERTY(EditAnywhere, Category = "Debug") bool bDrawDebug = true;


	// Evaluate union-of-solids SDF at a world-space point (negative inside walls/floor)
	float EvalSolid(const FVector& Xw) const { return SolidSDF ? SolidSDF(Xw) : 1e9f; }


	// World-space normal (outward from solid) via numerical gradient
	FVector SolidNormal(const FVector& Xw) const { return SchSDF::Gradient(SolidSDF, Xw, 1e-3f); }


protected:
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif


private:
	SchSDF::SDF SolidSDF; // union of 5 (or 6) boxes, in world space via captured transform
	void BuildWorldSDF();
	void DrawDebugNow();
};