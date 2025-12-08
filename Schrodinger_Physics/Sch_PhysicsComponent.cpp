// Fill out your copyright notice in the Description page of Project Settings.


#include "Schrodinger_Physics/Sch_PhysicsComponent.h"
#include "Schrodinger_Physics/Sch_SDF.h"


// Sets default values for this component's properties
//#include "DrawDebugHelpers.h"

USch_PhysicsComponent::USch_PhysicsComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void USch_PhysicsComponent::BeginPlay()
{
	Super::BeginPlay();

	const FTransform Base = GetOwner()->GetTransform();

	// StartPosition/StartRotation을 컴포넌트 로컬로 보고, 월드로 변환해 보관
	P = Base.TransformPosition(StartPosition);
	Q = Base.GetRotation() * StartRotation;

	// init rigid body
	//P = StartPosition; Q = StartRotation;
	V = FVector(220, 160, 280); // some initial kick so it bounces


	BuildWellSDF();
}

void USch_PhysicsComponent::BuildWellSDF()
{
	const FTransform Base = GetOwner()->GetTransform();
	const FVector    C = WellCenter;        // 로컬(컴포넌트) 기준
	const FVector    H = WellHalfExtents;

	// Xw(월드)의 SDF가 호출되면, 컴포넌트 로컬로 바꿔서 AABox SDF 평가
	WellInterior = [Base, C, H](const FVector& Xw)
		{
			const FVector Xl = Base.InverseTransformPosition(Xw);
			return SchSDF::BoxAA(Xl - C, H); // 내부<0
		};
	// Interior AABox SDF = negative inside box centered at WellCenter
}


void USch_PhysicsComponent::TickComponent(float Dt, ELevelTick, FActorComponentTickFunction*)
{
	Accumulator += Dt;
	while (Accumulator >= FixedDt)
	{
		PhysicsStep(FixedDt);
		Accumulator -= FixedDt;
	}
	if (bDrawDebug) DrawDebugNow();
}


void USch_PhysicsComponent::PhysicsStep(float Dt)
{
	// 1) integrate forces
	V += Gravity * Dt;
	P += V * Dt;


	// 2) SDF-based capsule vs interior-box collision
	// Condition: s(Xi) + r > 0 (Xi sample along capsule axis)
	const FVector Axis = Q.GetAxisZ();
	const float h = CapsuleHalfLength;
	const float r = CapsuleRadius;


	float worstPhi = -FLT_MAX; // penetration measure (positive = penetrating)
	FVector contactPoint = P; // point on capsule centerline where phi is worst


	const int32 N = FMath::Max(1, CapsuleSamples);
	for (int32 i = 0; i < N; ++i)
	{
		const float t = (N == 1) ? 0.f : FMath::Lerp(-h, h, float(i) / float(N - 1));
		const FVector Xi = P + Axis * t;
		const float s = WellInterior(Xi); // negative inside interior
		const float phi = s + r; // >0 means the sphere at Xi with radius r crosses boundary
		if (phi > worstPhi) { worstPhi = phi; contactPoint = Xi; }
	}


	if (worstPhi > 0.f)
	{
		// push back inside along outward normal of interior (i.e., push by -n)
		const FVector n = SchSDF::Gradient(WellInterior, contactPoint); // outward from interior


		// positional correction (split-impulse style)
		P -= n * worstPhi;


		// velocity correction (restitution)
		const float vn = FVector::DotProduct(V, n);
		if (vn > 0.f)
		{
			V = V - (1.f + Restitution) * vn * n;
		}


		// simple friction (optional): damp tangent a bit
		const FVector vt = V - FVector::DotProduct(V, n) * n;
		V = vt * 0.98f + (V - vt); // light tangential damping
	}
}


void USch_PhysicsComponent::DrawDebugNow()
{
	UWorld* W = GetWorld(); if (!W) return;


	const FTransform Base = GetOwner()->GetTransform();

	// 우물(OBB로 그리기: 로컬 박스 + 컴포넌트 회전/이동)
	DrawDebugBox(
		W,
		Base.TransformPosition(WellCenter),
		WellHalfExtents,
		Base.GetRotation(),
		FColor::Cyan, false, 0.f, 0, 1.f
	);

	// 캡슐 (축은 Q 기준, 이미 월드 회전 들어간 상태)
	const FVector Axis = Q.GetAxisZ();
	const FVector A = P - Axis * CapsuleHalfLength;
	const FVector B = P + Axis * CapsuleHalfLength;

	DrawDebugLine(W, A, B, FColor::Green, false, 0.f, 0, 1.f);
	//DrawDebugSphere(W, A, CapsuleRadius, 12, FColor::Green, false, 0.f, 0, 1.f);
	//DrawDebugSphere(W, B, CapsuleRadius, 12, FColor::Green, false, 0.f, 0, 1.f);
	DrawDebugPoint(W, P, 6.f, FColor::Yellow, false, 0.f);

	DrawDebugCapsule(
		W,
		P,
		CapsuleHalfLength + CapsuleRadius, // UE의 HalfHeight는 '원통 반길이(h) + r'
		CapsuleRadius,
		Q,
		FColor::Green,
		false, 0.f, 0, 1.f
	);
}