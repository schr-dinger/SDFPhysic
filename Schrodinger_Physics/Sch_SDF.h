// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

/**
 * 
 */
namespace SchSDF
{
	using SDF = TFunction<float(const FVector&)>; // distance only (normal via numerical gradient)

	// --- Primitives -----------------------------------------------------------
	inline float Sphere(const FVector& P, float Radius)
	{
		return P.Length() - Radius;
	}


	// Axis-aligned box centered at origin with half-extents B (AABox)
	inline float BoxAA(const FVector& P, const FVector& B)
	{
		// iq style AABox SDF
		const FVector Q = P.GetAbs() - B;
		const FVector Qpos(FMath::Max(Q.X, 0.f), FMath::Max(Q.Y, 0.f), FMath::Max(Q.Z, 0.f));
		const float Outside = Qpos.Length();
		const float Inside = FMath::Min(FMath::Max3(Q.X, Q.Y, Q.Z), 0.f);
		return Outside + Inside; // negative when inside
	}


	// Oriented box: transform P to local with R (rotation) and C (center)
	inline float BoxOBB(const FVector& P, const FVector& Center, const FQuat& Rot, const FVector& Half)
	{
		const FVector Pl = Rot.UnrotateVector(P - Center);
		return BoxAA(Pl, Half);
	}


	// Capsule defined by segment [A,B] and radius r
	inline float Capsule(const FVector& P, const FVector& A, const FVector& B, float r)
	{
		const FVector PA = P - A;
		const FVector BA = B - A;
		const float H = FMath::Clamp(FVector::DotProduct(PA, BA) / BA.SizeSquared(), 0.f, 1.f);
		return (PA - BA * H).Length() - r;
	}


	// Infinite plane (n normalized). Signed so that half-space n·X + d < 0 is inside.
	inline float Plane(const FVector& P, const FVector& N, float d)
	{
		return FVector::DotProduct(N, P) + d;
	}


	// --- Combinators ---------------------------------------------------------
	inline float Union(float a, float b) { return FMath::Min(a, b); }
	inline float Intersection(float a, float b) { return FMath::Max(a, b); }
	inline float Difference(float a, float b) { return FMath::Max(a, -b); }


	// Polynomial smooth min (IQ), k > 0 controls blend width in world units
	inline float SmoothUnion(float a, float b, float k)
	{
		const float h = FMath::Clamp(0.5f + 0.5f * (b - a) / k, 0.f, 1.f);
		return FMath::Lerp(b, a, h) - k * h * (1.f - h);
	}


	// Numerical gradient for composed SDFs (central differences)
	inline FVector Gradient(const SDF& f, const FVector& P, float Eps = 1e-3f)
	{
		const FVector ex(Eps, 0, 0), ey(0, Eps, 0), ez(0, 0, Eps);
		const float dx = f(P + ex) - f(P - ex);
		const float dy = f(P + ey) - f(P - ey);
		const float dz = f(P + ez) - f(P - ez);
		FVector g(dx, dy, dz);
		const float inv = 0.5f / Eps;
		g *= inv;
		const float len = g.Length();
		return len > 1e-6f ? (g / len) : FVector(0, 0, 1);
	}


	// Helper to translate/rotate an SDF primitive
	inline SDF Translate(const SDF& f, const FVector& T)
	{
		return [f, T](const FVector& P) { return f(P - T); };
	}


	inline SDF Rotate(const SDF& f, const FQuat& R)
	{
		return [f, R](const FVector& P) { return f(R.UnrotateVector(P)); };
	}


	// Scene union of multiple SDFs (hard min)
	inline SDF UnionAll(const TArray<SDF>& Fields)
	{
		return [Fields](const FVector& P) {
			float d = TNumericLimits<float>::Max();
			for (const auto& f : Fields) d = FMath::Min(d, f(P));
			return d;
		};
	}


}


class DEFAULT56_PROJECT_API Sch_SDF
{
public:
	Sch_SDF();
	~Sch_SDF();
};
