// Fill out your copyright notice in the Description page of Project Settings.


#include "Schrodinger_Physics/Sch_SDFPhysicsSubsystem.h"
#include "Schrodinger_Physics/Sch_SDFComponent.h"



void USch_SDFPhysicsSubsystem::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    StackedTime += DeltaTime;
    int substep = 0;
    while (StackedTime >= FixedStepTime && substep++ < MaxSteps)
    {
        Step(FixedStepTime);
        StackedTime -= FixedStepTime;
    }

    //Step(DeltaTime);

    //UE_LOG(LogTemp, Log, TEXT("[SchPhys] Tick Δ=%.4f, StackedTime=%.4f, Steps=%d, Bodies=%d"),DeltaTime, StackedTime, substep, GSchSdfRegistry.Num());
}

void USch_SDFPhysicsSubsystem::Register(USch_SDFComponent* C)
{
    if (!IsValid(C)) return;
    if (C->GetWorld() != GetWorld()) return;

    //정리
    for (int32 i = GSchSdfRegistry.Num() - 1; i >= 0; --i)
    {
        if (!GSchSdfRegistry[i].IsValid())
        {
            GSchSdfRegistry.RemoveAtSwap(i);
        }
    }

    //중복 방지
    for (int32 i = 0; i < GSchSdfRegistry.Num(); ++i)
    {
        if (GSchSdfRegistry[i].Get() == C)
        {
            return;
        }
    }

    GSchSdfRegistry.Add(C);
}

void USch_SDFPhysicsSubsystem::Unregister(USch_SDFComponent* C)
{
    if (!C) return;

    for (int32 i = GSchSdfRegistry.Num() - 1; i >= 0; --i)
    {
        USch_SDFComponent* Comp = GSchSdfRegistry[i].Get();
        if (!Comp || Comp == C)
        {
            GSchSdfRegistry.RemoveAtSwap(i);
        }
    }
}

void USch_SDFPhysicsSubsystem::Step(float dt)
{
    for (int32 i = 0; i < GSchSdfRegistry.Num(); ++i)
    {
        USch_SDFComponent* C = GSchSdfRegistry[i].Get();
        if (!C) continue;
        C->VelocityIntegrate(dt);
    }

    TArray<TPair<USch_SDFComponent*, USch_SDFComponent*>> Pairs;

    Broadphase(Pairs, dt);

    SolvePairs(Pairs, dt);

    for (int32 i = 0; i < GSchSdfRegistry.Num(); ++i)
    {
        USch_SDFComponent* C = GSchSdfRegistry[i].Get();
        if (!C || !C->CanMove) continue;

        AActor* Owner = C->GetOwner();
        if (!Owner) continue;

        FVector x1 = Owner->GetActorLocation() + C->Velocity * dt;
        Owner->SetActorLocation(x1);

        const float w = C->AngularVelocity.Size();
        if (w > KINDA_SMALL_NUMBER)
        {
            const FVector axis = C->AngularVelocity / w;
            const FQuat dq(axis, w * dt);
            FQuat q1 = (dq * Owner->GetActorQuat()).GetNormalized();
            Owner->SetActorRotation(q1);
        }

        C->SyncLFromAngular();
    }
}

void USch_SDFPhysicsSubsystem::Broadphase(TArray<TPair<USch_SDFComponent*, USch_SDFComponent*>>& outPairs, float dt)
{
    outPairs.Reset();

    const int32 N = GSchSdfRegistry.Num();
    for (int32 i = 0; i < N; ++i)
    {
        USch_SDFComponent* A = GSchSdfRegistry[i].Get();
        if (!A) continue;

        //if (!A->CanMove) continue;

        const FBox BoxA = ComputeWorldAABB_Swept(A, dt, BroadphaseWiggle, bUseSweptAabb);

        for (int32 j = i + 1; j < N; ++j)
        {
            USch_SDFComponent* B = GSchSdfRegistry[j].Get();
            if (!B) continue;

            //if (A->IsClusteredWith(B)) continue;  //같은 클러스터 스킵

            const FBox BoxB = ComputeWorldAABB_Swept(B, dt, BroadphaseWiggle, bUseSweptAabb);

            //안움직이는 애들 끼리는 스킵 벽 이랑 벽 같은
            if (!(A->CanMove || B->CanMove)) continue;

            if (BoxA.Intersect(BoxB))
            {
                outPairs.Add(TPair<USch_SDFComponent*, USch_SDFComponent*>(A, B));
            }
        }
    }
}


void USch_SDFPhysicsSubsystem::SolvePairs(const TArray<TPair<USch_SDFComponent*, USch_SDFComponent*>>& pairs, float dt)
{
    const int32 Iters = SolverIterations;

    UE_LOG(LogTemp, Log, TEXT("[SolvePairs] Processing %d pairs, %d iterations"), pairs.Num(), Iters);

    for (int32 it = 0; it < Iters; ++it)
    {
        const int32 M = pairs.Num();
        for (int32 p = 0; p < M; ++p)
        {
            USch_SDFComponent* A = pairs[p].Key;
            USch_SDFComponent* B = pairs[p].Value;
            if (!A || !B) continue;

            if (!(A->CanMove || B->CanMove)) continue;

            FVector CP(0, 0, 0);
            FVector N(0, 0, 1);
            float   d = 1e9f;

            const bool hit = A->CheckCollision(B, CP, N, d);

            if (!hit) continue;

            UE_LOG(LogTemp, Log, TEXT("  [Pair %d] %s vs %s | d=%.2f"),
                p, *A->GetOwner()->GetName(), *B->GetOwner()->GetName(), d);

            const FVector comA = A->GetComWorld();
            const FVector comB = B->GetComWorld();
            const FVector rA = CP - comA;
            const FVector rB = CP - comB;
            
            auto PointVel = [](USch_SDFComponent* C, const FVector& r) {
                return C->Velocity + FVector::CrossProduct(C->AngularVelocity, r);
            };

            const float vRelN = FVector::DotProduct(PointVel(A, rA) - PointVel(B, rB), N);
            const float closing = FMath::Max(0.f, -vRelN) * dt;
            const float slop = SpeculativeContactSlop + closing;
            
            if (d <= slop)
            {
                if (A->CanMove && d < 0)
                {
                    FVector Location = A->GetOwner()->GetActorLocation() - N * d;
                }

                FVector Jn(0, 0, 0);
                A->ProcessImpact(B, CP, N, d, (float)dt, Jn);
            }
        }
    }
}

FBox USch_SDFPhysicsSubsystem::ComputeWorldAABB_Swept(USch_SDFComponent* C, double dt, float wiggle, bool bUseSwept)
{
    const FTransform Xf = C->StaticMesh->GetComponentTransform();
    FBox WorldAabb = C->Sdf.LocalBounds.TransformBy(Xf);

    if (bUseSwept)
    {
        const FVector v = C->Velocity;
        float expand = v.Size() * (float)dt + wiggle;

        const FVector Extent = WorldAabb.GetExtent();
        const float radius = Extent.Size();
        const float w = C->AngularVelocity.Size();
        expand += radius * w * (float)dt;

        WorldAabb = WorldAabb.ExpandBy(expand);
    }
    else
    {
        WorldAabb = WorldAabb.ExpandBy(wiggle);
    }

    return WorldAabb;
}