// Fill out your copyright notice in the Description page of Project Settings.


#include "Schrodinger_Physics/Sch_SDFStageComponent.h"


// Sets default values for this component's properties
USch_SDFStageComponent::USch_SDFStageComponent()
{
	PrimaryComponentTick.bCanEverTick = true; // for debug draw updates
}


void USch_SDFStageComponent::BeginPlay()
{
	Super::BeginPlay();
	BuildWorldSDF();
}


#if WITH_EDITOR
void USch_SDFStageComponent::PostEditChangeProperty(FPropertyChangedEvent& E)
{
	Super::PostEditChangeProperty(E);
	BuildWorldSDF();
}
#endif


void USch_SDFStageComponent::TickComponent(float Dt, ELevelTick, FActorComponentTickFunction*)
{
	if (bDrawDebug) DrawDebugNow();
}


void USch_SDFStageComponent::BuildWorldSDF()
{
	const FTransform Base = GetOwner()->GetActorTransform();
	const FVector I = InnerHalfExtents; // interior half-extents
	const float t = FMath::Max(0.1f, WallThickness * 0.5f); // half-thickness for SDF


	struct FLocalBox { FVector C; FVector H; };
	TArray<FLocalBox> Boxes; Boxes.Reserve(bHasCeiling ? 6 : 5);


	// Floor slab (under interior bottom)
	Boxes.Add({ FVector(0,0, -(I.Z + t)), FVector(I.X + t, I.Y + t, t) });


	// +X wall
	Boxes.Add({ FVector(I.X + t, 0, 0), FVector(t, I.Y + t, I.Z + t) });
	// -X wall
	Boxes.Add({ FVector(-I.X - t, 0, 0), FVector(t, I.Y + t, I.Z + t) });
	// +Y wall
	Boxes.Add({ FVector(0, I.Y + t, 0), FVector(I.X + t, t, I.Z + t) });
	// -Y wall
	Boxes.Add({ FVector(0,-I.Y - t, 0), FVector(I.X + t, t, I.Z + t) });


	if (bHasCeiling)
		Boxes.Add({ FVector(0,0, I.Z + t), FVector(I.X + t, I.Y + t, t) });


	// Build solid SDF in world space by capturing transform & boxes
	SolidSDF = [Base, Boxes](const FVector& Xw) {
		const FVector Xl = Base.InverseTransformPosition(Xw);
		float d = TNumericLimits<float>::Max();
		for (const FLocalBox& B : Boxes) {
			d = FMath::Min(d, SchSDF::BoxAA(Xl - B.C, B.H)); // negative inside slab
		}
		return d; // union of solids
		};
}


void USch_SDFStageComponent::DrawDebugNow()
{
	UWorld* W = GetWorld(); if (!W) return;


	const FTransform Base = GetOwner()->GetActorTransform();
	const FVector I = InnerHalfExtents;
	const float t = FMath::Max(0.1f, WallThickness * 0.5f);


	auto DrawSlab = [&](const FVector& C, const FVector& H) {
		DrawDebugBox(W, Base.TransformPosition(C), H, Base.GetRotation(), FColor::Cyan, false, 0.f, 0, 1.f);
		};


	DrawSlab(FVector(0, 0, -(I.Z + t)), FVector(I.X + t, I.Y + t, t));
	DrawSlab(FVector(I.X + t, 0, 0), FVector(t, I.Y + t, I.Z + t));
	DrawSlab(FVector(-I.X - t, 0, 0), FVector(t, I.Y + t, I.Z + t));
	DrawSlab(FVector(0, I.Y + t, 0), FVector(I.X + t, t, I.Z + t));
	DrawSlab(FVector(0, -I.Y - t, 0), FVector(I.X + t, t, I.Z + t));
	if (bHasCeiling)
		DrawSlab(FVector(0, 0, I.Z + t), FVector(I.X + t, I.Y + t, t));
}