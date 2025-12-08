// Fill out your copyright notice in the Description page of Project Settings.


#include "Schrodinger_Physics/Sch_SDFCapsuleComponent.h"
#include "Schrodinger_Physics/Sch_SDFStageComponent.h"
#include "EngineUtils.h"

// Sets default values for this component's properties
USch_SDFCapsuleComponent::USch_SDFCapsuleComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}


void USch_SDFCapsuleComponent::BeginPlay()
{
	Super::BeginPlay();
	const FTransform Base = GetOwner()->GetActorTransform();
	P = Base.TransformPosition(StartPosition);
	Q = Base.GetRotation() * StartRotation;
	V = InitialVelocity;
	W = InitialAngularVelocity;
	InvMass = (Mass > 0.f) ? 1.f / Mass : 0.f;

	RecomputeInertia();

	if (!SDFWorld) AutoFindSDFWorld();
}


void USch_SDFCapsuleComponent::RecomputeInertia()
{
	const float m = Mass, r = Radius, h = HalfLength;
	const float Ixx = (m > 0) ? (m * (3 * r * r + 4 * h * h) / 12.f) : FLT_MAX; // X,Y
	const float Izz = (m > 0) ? (0.5f * m * r * r) : FLT_MAX;             // Z(축)
	InertiaInvLocal = FVector((Ixx > 0 ? 1.f / Ixx : 0), (Ixx > 0 ? 1.f / Ixx : 0), (Izz > 0 ? 1.f / Izz : 0));
}

void USch_SDFCapsuleComponent::IntegrateOrientation(float Dt)
{
	FQuat Omega(W.X, W.Y, W.Z, 0.f);
	FQuat Qdot = Omega * Q * 0.5f;
	Q = (Q + Qdot * Dt).GetNormalized();
}

FVector USch_SDFCapsuleComponent::WorldInvInertiaMul(const FVector& v) const
{
	const FVector vl = Q.UnrotateVector(v);
	const FVector wl(vl.X * InertiaInvLocal.X, vl.Y * InertiaInvLocal.Y, vl.Z * InertiaInvLocal.Z);
	return Q.RotateVector(wl);
}

void USch_SDFCapsuleComponent::ApplyImpulseAtPoint(const FVector& J, const FVector& C)
{
	if (InvMass == 0.f) return;
	const FVector r = C - P;
	V += InvMass * J;
	W += WorldInvInertiaMul(FVector::CrossProduct(r, J));
}

void USch_SDFCapsuleComponent::AutoFindSDFWorld()
{
	UWorld* World = GetWorld(); if (!World) return;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		USch_SDFStageComponent* Comp = It->FindComponentByClass<USch_SDFStageComponent>();
		if (Comp) { SDFWorld = Comp; break; }
	}
}


void USch_SDFCapsuleComponent::TickComponent(float Dt, ELevelTick, FActorComponentTickFunction*)
{
	Accumulator += Dt;
	while (Accumulator >= FixedDt) {
		PhysicsStep(FixedDt);
		Accumulator -= FixedDt;
	}
	if (bDrawDebug) DrawDebugNow();
}


void USch_SDFCapsuleComponent::PhysicsStep(float Dt)
{
	if (!SDFWorld) return; // no collision world assigned

	 // 1) 자유 적분 (선형/각감쇠 포함)
	if (InvMass > 0.f) V += Gravity * Dt;
	V *= FMath::Max(0.f, 1.f - LinearDamping * Dt);
	W *= FMath::Max(0.f, 1.f - AngularDamping * Dt);
	P += V * Dt;
	IntegrateOrientation(Dt);

	// sample along capsule axis
	const FVector Axis = Q.GetAxisZ();
	const int32 N = 9;
	float minS = TNumericLimits<float>::Max();
	FVector contactXi = P;


	for (int32 i = 0; i < N; ++i) {
		const float t = (N == 1) ? 0.f : FMath::Lerp(-HalfLength, HalfLength, float(i) / float(N - 1));
		const FVector Xi = P + Axis * t;
		const float s = SDFWorld->EvalSolid(Xi); // negative inside any wall
		if (s < minS) { minS = s; contactXi = Xi; }
	}

	const float pen = Radius - minS;
	if (pen > 0.f)
	{
		const FVector n = SDFWorld->SolidNormal(contactXi);
		const FVector C = contactXi - n * Radius; // 캡슐 접점
		const FVector r = C - P;
		   // 위치 보정 (split impulse 느낌)
		P += n * pen;
		
		// 접점 상대속도
		const FVector vC = V + FVector::CrossProduct(W, r);
		const float   vn = FVector::DotProduct(vC, n);
		const float   RestThreshold = 20.f;
		const float   e = (vn < -RestThreshold) ? Restitution : 0.f;
		
		// 노멀 임펄스 jn
		const FVector rn = FVector::CrossProduct(r, n);
		const float   denomN = InvMass + FVector::DotProduct(n, FVector::CrossProduct(WorldInvInertiaMul(rn), r));
		if (denomN > 1e-6f) 
		{
			const float jn = FMath::Max(-(1.f + e) * vn / denomN, 0.f);
			ApplyImpulseAtPoint(jn * n, C);
			
			// 마찰 (쿨롬)
			const FVector vC2 = V + FVector::CrossProduct(W, r);
			const float   vn2 = FVector::DotProduct(vC2, n);
			const FVector vt = vC2 - vn2 * n;
			const float   vtLen = vt.Length();
			if (vtLen > 1e-4f) {
				const FVector tHat = vt / vtLen;
				const FVector rt = FVector::CrossProduct(r, tHat);
				const float denomT = InvMass + FVector::DotProduct(tHat, FVector::CrossProduct(WorldInvInertiaMul(rt), r));
				if (denomT > 1e-6f)
				{
					float jt = -vtLen / denomT;                 // 접선 속도 제거 시도
					const float mu = DynamicFriction;
					jt = FMath::Clamp(jt, -mu * jn, mu * jn);        // |jt| <= μ * jn
					ApplyImpulseAtPoint(jt * tHat, C);
				}
			}
		}
	}
}


void USch_SDFCapsuleComponent::DrawDebugNow()
{
	UWorld* World = GetWorld(); if (!World) return;
	DrawDebugCapsule(
		World,
		P,
		HalfLength + Radius, // UE expects h + r
		Radius,
		Q,
		FColor::Green,
		false, 0.f, 0, 1.f
	);
}
