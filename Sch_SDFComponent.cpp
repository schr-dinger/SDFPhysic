// Fill out your copyright notice in the Description page of Project Settings.


#include "Schrodinger_Physics/Sch_SDFComponent.h"
#include "DrawDebugHelpers.h"
//#include "EngineUtils.h"
//#include "Engine/World.h"
//#include "UObject/UObjectIterator.h"
#include "Components/SceneComponent.h"
//#include "Math/MathFwd.h"
// Sets default values for this component's properties

static TArray<TWeakObjectPtr<USch_SDFComponent>> GSchSdfRegistry;

namespace{
FBox GetLocalBox(USch_SDFComponent* Floor)
{
    // return Floor->Sdf.LocalBounds;
    if (UStaticMeshComponent* SM = Floor->StaticMesh.Get())
    {
        FBoxSphereBounds B = SM->CalcBounds(SM->GetComponentTransform());
    }
    return FBox(FVector(-1000, -1000, 0), FVector(1000, 1000, 10));
}


}




USch_SDFComponent::USch_SDFComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void USch_SDFComponent::BeginPlay()
{
	Super::BeginPlay();
    GSchSdfRegistry.AddUnique(this);
    // ...
    if (AActor* Owner = GetOwner())
    {
        TArray<UStaticMeshComponent*> SMs;
        Owner->GetComponents<UStaticMeshComponent>(SMs);
        if (SMs.Num() > 0)
        {
            StaticMesh = SMs[0];
        }
    }

    InvInertiaTensorLocal = FVector(
        InertiaTensorLocal.X > 0 ? 1.0f / InertiaTensorLocal.X : 0.0f,
        InertiaTensorLocal.Y > 0 ? 1.0f / InertiaTensorLocal.Y : 0.0f,
        InertiaTensorLocal.Z > 0 ? 1.0f / InertiaTensorLocal.Z : 0.0f
    );


    //@@@Test
    SyncLFromAngular();

}

void USch_SDFComponent::OnRegister()
{
    Super::OnRegister();

    if (!PrebakedAsset.IsNull())
    {
        if (USchSdfAsset* A = PrebakedAsset.LoadSynchronous())
        {
            LoadFromPrebaked();
            return;
        }
    }
}

bool USch_SDFComponent::GenerateSdfNow()
{
    UStaticMeshComponent* SMC = nullptr;
    ResolveSourceMesh(SMC);
    TArray<FVector> V;
    TArray<FIntVector> Tri;
    if (!ExtractTriangles(SMC, V, Tri))
    {
        return false;
    }

    // 메쉬 로컬 바운즈 → 스케일 적용
    const FBox LocalBounds = SMC->GetStaticMesh()->GetBoundingBox().TransformBy(
        FTransform(SMC->GetStaticMesh()->GetBounds().GetBox().GetCenter() - SMC->GetStaticMesh()->GetBoundingBox().GetCenter())
    );
    FBox MeshLocalBounds = SMC->GetStaticMesh()->GetBoundingBox();
    MeshLocalBounds = MeshLocalBounds.ExpandBy(MeshLocalBounds.GetExtent() * (BoundsScale - 1.0f));

    BuildSdf_FromTriangles(V, Tri, MeshLocalBounds);

    return true;
}

float USch_SDFComponent::SampleSdfWorld(const FVector& WorldPos)
{
    const UStaticMeshComponent* SMC = StaticMesh.IsValid() ? StaticMesh.Get() : nullptr;
    const FTransform MeshToWorld = SMC ? SMC->GetComponentTransform() : GetOwner()->GetActorTransform();
    const FVector LocalPos = MeshToWorld.InverseTransformPosition(WorldPos);
    return TrilinearSample(LocalPos); //테스트용
    //return SampleSdfLocal_Nearest(LocalPos);
}

float USch_SDFComponent::SampleSdfLocal(const FVector& LocalPos)
{
    return TrilinearSample(LocalPos); //테스트용
    //return SampleSdfLocal_Nearest(LocalPos);
}

bool USch_SDFComponent::ResolveSourceMesh(UStaticMeshComponent*& OutSMC)
{
    if (StaticMesh.IsValid())
    {
        OutSMC = StaticMesh.Get();
        return true;
    }

    if (AActor* Owner = GetOwner())
    {
        TArray<UStaticMeshComponent*> SMs;
        Owner->GetComponents<UStaticMeshComponent>(SMs);
        if (SMs.Num() > 0)
        {
            OutSMC = SMs[0];
            StaticMesh = SMs[0];
            return true;
        }
    }
    return false;
}

bool USch_SDFComponent::ExtractTriangles(const UStaticMeshComponent* SMC, TArray<FVector>& OutV, TArray<FIntVector>& OutTri) const
{
    OutV.Reset(); OutTri.Reset();
    const UStaticMesh* SM = SMC->GetStaticMesh();
    //if (!SM || !SM->GetRenderData() || SM->GetRenderData()->LODResources.Num() == 0) return false;

    const int32 NumLODs = SM->GetRenderData()->LODResources.Num();
    const int32 UseLOD = FMath::Max(NumLODs - 1, 0);

    const FStaticMeshLODResources& LOD = SM->GetRenderData()->LODResources[UseLOD];

    const FPositionVertexBuffer& PosBuffer = LOD.VertexBuffers.PositionVertexBuffer;
    const FRawStaticIndexBuffer& IndexBuffer = LOD.IndexBuffer;

    const int32 NumVerts = PosBuffer.GetNumVertices();
    OutV.Reserve(NumVerts);
    for (int32 i = 0; i < NumVerts; ++i)
    {
        OutV.Add((FVector)PosBuffer.VertexPosition(i));
    }

    const int32 NumIndices = IndexBuffer.GetNumIndices();
    if (NumIndices % 3 != 0) return false;
    OutTri.Reserve(NumIndices / 3);
    for (int32 i = 0; i < NumIndices; i += 3)
    {
        const uint32 i0 = IndexBuffer.GetIndex(i + 0);
        const uint32 i1 = IndexBuffer.GetIndex(i + 1);
        const uint32 i2 = IndexBuffer.GetIndex(i + 2);
        OutTri.Add(FIntVector((int32)i0, (int32)i1, (int32)i2));
    }
    return true;
}

void USch_SDFComponent::BuildSdf_FromTriangles(const TArray<FVector>& V, const TArray<FIntVector>& Tri, const FBox& MeshLocalBounds)
{
    Sdf.Init(GridRes, MeshLocalBounds);

    const FVector MinB = MeshLocalBounds.Min;
    const FVector MaxB = MeshLocalBounds.Max;
    const FVector Cell = (MaxB - MinB) / FVector((float)GridRes.X, (float)GridRes.Y, (float)GridRes.Z);

    // 2) 삼각형 AABB
    TArray<FBox> TriBounds;
    TriBounds.SetNumUninitialized(Tri.Num());
    for (int32 i = 0; i < Tri.Num(); ++i)
    {
        const FVector& A = V[Tri[i].X];
        const FVector& B = V[Tri[i].Y];
        const FVector& C = V[Tri[i].Z];
        FBox Bx(A, A);
        Bx += B; Bx += C;
        TriBounds[i] = Bx.ExpandBy(Cell.Size()); //조금 확장
    }

    for (int32 z = 0; z < GridRes.Z; ++z)
    {
        for (int32 y = 0; y < GridRes.Y; ++y)
        {
            for (int32 x = 0; x < GridRes.X; ++x)
            {
                const FVector P = MinB + (FVector(
                    (float)x + 0.5f,
                    (float)y + 0.5f,
                    (float)z + 0.5f) * Cell);

                float best = TNumericLimits<float>::Max();

                // 후보 삼각형만 검사
                for (int32 i = 0; i < Tri.Num(); ++i)
                {
                    if (!TriBounds[i].IsInsideOrOn(P)) continue;
                    const FIntVector& T = Tri[i];
                    const float d = PointTriangleDistance(P, V[T.X], V[T.Y], V[T.Z]);
                    if (d < best) best = d;
                }
                
                if (best == TNumericLimits<float>::Max())
                {
                    for (const FIntVector& T : Tri)
                    {
                        const float d = PointTriangleDistance(P, V[T.X], V[T.Y], V[T.Z]);
                        if (d < best) best = d;
                    }
                }
                
                const bool bInside = RayInsideTest(P, V, Tri);
                const float signedD = bInside ? -best : best;

                Sdf.Values[Sdf.Idx(x, y, z)] = signedD;
            }
        }
    }

    //그래디언트도 미리 계산하기
    const int32 Nx = Sdf.Dim.X, Ny = Sdf.Dim.Y, Nz = Sdf.Dim.Z;
    //const FVector MinB = Sdf.LocalBounds.Min;
    //const FVector MaxB = Sdf.LocalBounds.Max;
    //const FVector Cell = (MaxB - MinB) / FVector((float)Nx, (float)Ny, (float)Nz);

    Sdf.Gradients.SetNumUninitialized(Nx * Ny * Nz);

    for (int32 z = 0; z < Nz; ++z)
    {
        for (int32 y = 0; y < Ny; ++y)
        {
            for (int32 x = 0; x < Nx; ++x)
            {
                const int32 idx = Sdf.Idx(x, y, z);
                FVector localP = MinB + FVector((float)x + 0.5f, (float)y + 0.5f, (float)z + 0.5f) * Cell;

                FVector g = GetGradient(localP);
                if (!g.Normalize()) g = FVector::UpVector; // 방향만
                Sdf.Gradients[idx] = g;
            }
        }
    }
    Sdf.bHasGradients = true;
}

float USch_SDFComponent::PointTriangleDistance(const FVector& P, const FVector& A, const FVector& B, const FVector& C)
{
    FVector Closest;
    FVector Vec = BaryClamp(P, A, B, C, Closest);
    return Vec.Length();
    //return Dist_Tri_NearestVertex(P, A, B, C);
}

float USch_SDFComponent::Dist_Tri_NearestVertex(const FVector& P, const FVector& A, const FVector& B, const FVector& C)
{
    const float da = (P - A).Size();
    const float db = (P - B).Size();
    const float dc = (P - C).Size();
    return FMath::Min3(da, db, dc);
}


FORCEINLINE FVector BaryClamp(const FVector& P, const FVector& A, const FVector& B, const FVector& C, FVector& OutClosest)
{
    // Real-Time Collision Detection (Christer Ericson) 스타일의 점-삼각형 최근접점
    const FVector AB = B - A;
    const FVector AC = C - A;
    const FVector AP = P - A;

    float d1 = FVector::DotProduct(AB, AP);
    float d2 = FVector::DotProduct(AC, AP);
    if (d1 <= 0.f && d2 <= 0.f) { OutClosest = A; return OutClosest - P; }

    const FVector BP = P - B;
    float d3 = FVector::DotProduct(AB, BP);
    float d4 = FVector::DotProduct(AC, BP);
    if (d3 >= 0.f && d4 <= d3) { OutClosest = B; return OutClosest - P; }

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
        float v = d1 / (d1 - d3);
        OutClosest = A + v * AB; return OutClosest - P;
    }

    const FVector CP = P - C;
    float d5 = FVector::DotProduct(AB, CP);
    float d6 = FVector::DotProduct(AC, CP);
    if (d6 >= 0.f && d5 <= d6) { OutClosest = C; return OutClosest - P; }

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
        float w = d2 / (d2 - d6);
        OutClosest = A + w * AC; return OutClosest - P;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
    {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        OutClosest = B + w * (C - B); return OutClosest - P;
    }

    // 내부 영역: 법선 평면 투영
    FVector N = FVector::CrossProduct(AB, AC);
    N.Normalize();
    float dist = FVector::DotProduct(P - A, N);
    OutClosest = P - dist * N;
    return OutClosest - P;
}

bool USch_SDFComponent::RayInsideTest(const FVector& P, const TArray<FVector>& V, const TArray<FIntVector>& Tri)
{
    int32 Count = 0;
    for (const FIntVector& T : Tri)
    {
        const FVector& A = V[T.X];
        const FVector& B = V[T.Y];
        const FVector& C = V[T.Z];
        if (RayIntersectTriangle_PosX(P, A, B, C))
            ++Count;
    }
    // 홀짝(홀수=inside)
    return (Count & 1) != 0;
}

bool USch_SDFComponent::RayIntersectTriangle_PosX(const FVector& P, const FVector& A, const FVector& B, const FVector& C)
{
    const FVector Dir(1, 0, 0);
    const float EPS = 1e-7f;

    FVector E1 = B - A;
    FVector E2 = C - A;
    FVector H = FVector::CrossProduct(Dir, E2);
    float a = FVector::DotProduct(E1, H);
    if (FMath::Abs(a) < EPS) return false;

    float f = 1.f / a;
    FVector S = P - A;
    float u = f * FVector::DotProduct(S, H);
    if (u < 0.f || u > 1.f) return false;

    FVector Q = FVector::CrossProduct(S, E1);
    float v = f * FVector::DotProduct(Dir, Q);
    if (v < 0.f || u + v > 1.f) return false;

    float t = f * FVector::DotProduct(E2, Q);
    return t > EPS;
}

float USch_SDFComponent::TrilinearSample(const FVector& LocalPos)
{
    const FIntVector dim = Sdf.Dim;
    const int32 total = dim.X * dim.Y * dim.Z;
    if (dim.X < 2 || dim.Y < 2 || dim.Z < 2 || Sdf.Values.Num() < total)
    {
        return TNumericLimits<float>::Max(); // 데이터 부족 시 큰 양수
    }

    const FVector minB = Sdf.LocalBounds.Min;
    const FVector size = Sdf.LocalBounds.GetSize();
    const FVector invCell = FVector(
        (float)dim.X / size.X,
        (float)dim.Y / size.Y,
        (float)dim.Z / size.Z
    );

    // 보셀 중심 기준 좌표: (i+0.5, j+0.5, k+0.5)
    const FVector p = (LocalPos - minB) * invCell - FVector(0.5f);

    int32 i0 = FMath::FloorToInt(p.X);
    int32 j0 = FMath::FloorToInt(p.Y);
    int32 k0 = FMath::FloorToInt(p.Z);

    const float fx = FMath::Clamp(p.X - (float)i0, 0.0f, 1.0f);
    const float fy = FMath::Clamp(p.Y - (float)j0, 0.0f, 1.0f);
    const float fz = FMath::Clamp(p.Z - (float)k0, 0.0f, 1.0f);

    // 가장자리 클램프 (i0..i0+1 유효)
    i0 = FMath::Clamp(i0, 0, dim.X - 2);
    j0 = FMath::Clamp(j0, 0, dim.Y - 2);
    k0 = FMath::Clamp(k0, 0, dim.Z - 2);

    // 선형 인덱싱용 stride
    const int32 sx = 1;
    const int32 sy = dim.X;
    const int32 sz = dim.X * dim.Y;

    // (i0,j0,k0)를 기준으로 8코너 인덱스 계산
    const int32 base = i0 + j0 * sy + k0 * sz;

    const float c000 = Sdf.Values[base];                 // (i0, j0, k0)
    const float c100 = Sdf.Values[base + sx];            // (i0+1, j0, k0)
    const float c010 = Sdf.Values[base + sy];            // (i0, j0+1, k0)
    const float c110 = Sdf.Values[base + sy + sx];       // (i0+1, j0+1, k0)
    const float c001 = Sdf.Values[base + sz];            // (i0, j0, k0+1)
    const float c101 = Sdf.Values[base + sz + sx];       // (i0+1, j0, k0+1)
    const float c011 = Sdf.Values[base + sz + sy];       // (i0, j0+1, k0+1)
    const float c111 = Sdf.Values[base + sz + sy + sx];  // (i0+1, j0+1, k0+1)

    // x → y → z 순서 보간
    const float c00 = FMath::Lerp(c000, c100, fx);
    const float c10 = FMath::Lerp(c010, c110, fx);
    const float c01 = FMath::Lerp(c001, c101, fx);
    const float c11 = FMath::Lerp(c011, c111, fx);

    const float c0 = FMath::Lerp(c00, c10, fy);
    const float c1 = FMath::Lerp(c01, c11, fy);

    return FMath::Lerp(c0, c1, fz);
}

float USch_SDFComponent::SampleSdfLocal_Nearest(const FVector& LocalPos) const  //이건 일단 간단용인데 이거부터 해봄
{
    const FIntVector dim = Sdf.Dim;
    const int32 total = dim.X * dim.Y * dim.Z;
    if (dim.X < 1 || dim.Y < 1 || dim.Z < 1 || Sdf.Values.Num() < total)
    {
        return TNumericLimits<float>::Max();
    }

    const FVector minB = Sdf.LocalBounds.Min;
    const FVector size = Sdf.LocalBounds.GetSize();
    if (size.X <= KINDA_SMALL_NUMBER || size.Y <= KINDA_SMALL_NUMBER || size.Z <= KINDA_SMALL_NUMBER)
    {
        return TNumericLimits<float>::Max();
    }

    const FVector invCell(
        (float)dim.X / size.X,
        (float)dim.Y / size.Y,
        (float)dim.Z / size.Z
    );
    const FVector p = (LocalPos - minB) * invCell - FVector(0.5f);

    int32 ix = FMath::RoundToInt(p.X);
    int32 iy = FMath::RoundToInt(p.Y);
    int32 iz = FMath::RoundToInt(p.Z);

    if (ix < 0 || iy < 0 || iz < 0 || ix >= dim.X || iy >= dim.Y || iz >= dim.Z)
    {
        return TNumericLimits<float>::Max();
    }

    const int32 sx = 1;
    const int32 sy = dim.X;
    const int32 sz = dim.X * dim.Y;
    const int32 idx = ix + iy * sy + iz * sz;

    return Sdf.Values[idx];
}

FVector USch_SDFComponent::GetGradient(FVector& LocalP)
{
    const FIntVector dim = Sdf.Dim;
    const int32 total = dim.X * dim.Y * dim.Z;
    if (dim.X < 2 || dim.Y < 2 || dim.Z < 2 || Sdf.Values.Num() < total)
        return FVector::UpVector;

    const FVector minB = Sdf.LocalBounds.Min;
    const FVector size = Sdf.LocalBounds.GetSize();
    const FVector invCell((float)dim.X / size.X,(float)dim.Y / size.Y,(float)dim.Z / size.Z);
    const FVector p = (LocalP - minB) * invCell - FVector(0.5f);

    int32 i = FMath::RoundToInt(p.X);
    int32 j = FMath::RoundToInt(p.Y);
    int32 k = FMath::RoundToInt(p.Z);

    i = FMath::Clamp(i, 0, dim.X - 1);
    j = FMath::Clamp(j, 0, dim.Y - 1);
    k = FMath::Clamp(k, 0, dim.Z - 1);

    const int32 a = FMath::Clamp(i - 1, 0, dim.X - 1);
    const int32 b = FMath::Clamp(i + 1, 0, dim.X - 1);
    const int32 c = FMath::Clamp(j - 1, 0, dim.Y - 1);
    const int32 d = FMath::Clamp(j + 1, 0, dim.Y - 1);
    const int32 e = FMath::Clamp(k - 1, 0, dim.Z - 1);
    const int32 f = FMath::Clamp(k + 1, 0, dim.Z - 1);

    const int32 sx = 1;
    const int32 sy = dim.X;
    const int32 sz = dim.X * dim.Y;

    const float v_imjk = Sdf.Values[a + j * sy + k * sz];
    const float v_ipjk = Sdf.Values[b + j * sy + k * sz];
    const float v_ijmk = Sdf.Values[i + c * sy + k * sz];
    const float v_ijpk = Sdf.Values[i + d * sy + k * sz];
    const float v_ijkm = Sdf.Values[i + j * sy + e * sz];
    const float v_ijkp = Sdf.Values[i + j * sy + f * sz];

    float gx = 0.5f * (v_ipjk - v_imjk);
    float gy = 0.5f * (v_ijpk - v_ijmk);
    float gz = 0.5f * (v_ijkp - v_ijkm);

    FVector g(gx, gy, gz);
    if (g.IsNearlyZero())
        g = FVector::UpVector;
    return g;
}

FVector USch_SDFComponent::GetComWorld()
{
    const FQuat R = GetOwner()->GetActorQuat();
    return GetOwner()->GetActorLocation() + R.RotateVector(CenterOfMassLocal);
}

void USch_SDFComponent::SyncAngularFromL()
{
    // ω_body = I_body^{-1}·L_body  to  ω_world
    const FQuat R = GetOwner()->GetActorQuat();
    const FVector Omega_body(
        InvInertiaTensorLocal.X * L_body.X,
        InvInertiaTensorLocal.Y * L_body.Y,
        InvInertiaTensorLocal.Z * L_body.Z);
    AngularVelocity = R.RotateVector(Omega_body);
}

void USch_SDFComponent::SyncLFromAngular()
{
    // L_body = I_body·(R^T ω_world)  (시작/재계산 시 1회)
    const FQuat R = GetOwner()->GetActorQuat();
    const FVector wL = R.UnrotateVector(AngularVelocity);
    L_body = FVector(
        InertiaTensorLocal.X * wL.X,
        InertiaTensorLocal.Y * wL.Y,
        InertiaTensorLocal.Z * wL.Z);
    SyncAngularFromL(); // 일관성 유지
}

FVector USch_SDFComponent::ApplyGravity()
{
    if (!IsGravityOn) return FVector::ZeroVector;

    FVector accel(0, 0, -980.0f);

    if (bIsOnFloor)
    {
        FVector N = FloorNormal.GetSafeNormal();

        //return FVector::ZeroVector;

        return FVector::VectorPlaneProject(accel, N);
    }

    /*
    for (int32 i = 0; i < GSchSdfRegistry.Num(); i++)
    {
        USch_SDFComponent* Other = GSchSdfRegistry[i].Get();

        if (!Other || Other == this) continue;

        if (Other->IsFloor)
        {
            FVector a = GetOwner()->GetActorLocation();
            FVector b = Other->GetOwner()->GetActorLocation();
            FVector c = b - a;
            float r = c.Size();

            //너무 강한 중력 보정
            r = FMath::Max(r, 500);

            r *= r;
            accel += 10000.0f * (c/c.Size())*((Other->Mass) / r);
        }
    }
    */
    
    return accel;
}

// Called every frame
void USch_SDFComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    //debug Render
    if (DebugSDFRender)
    {
        FVector size = Sdf.LocalBounds.GetSize();
        float band = size.Size() * 0.5f;
        int32 step = 3;
        float psize = 2.5f;

        DebugDrawIsoBand(band, step, psize, DeltaTime);
    }

    if (DebugGradientRender)
    {
        DebugDrawGradients(8, 10, DeltaTime);
    }


    //Basic Physics

    const double now = GetWorld()->GetTimeSeconds();
    const bool wasOnFloor = bIsOnFloor;
    const FVector prevN = FloorNormal;

    if (bIsOnFloor)
    {
        GEngine->AddOnScreenDebugMessage(-1, DeltaTime, FColor::Green, TEXT("IsOnFloor!"));
    }

    /*
    if (bGroundLocked)
    {
        // 락 걸린 프레임은 접지로 간주 + 유지 연장
        bIsOnFloor = true;
        FloorNormal = GroundLockNormal;
        FloorHoldUntil = now + FloorHoldTime;
    }
    else
    {
        // 최근에 접지였으면 잠깐 유지(깜빡임 방지)
        if (bIsOnFloor)
        {
            const FVector N = FloorNormal.GetSafeNormal();
            const float   vnUp = FVector::DotProduct(Velocity, N); // +면 위로 이탈
            const bool breakUp = (vnUp > GroundBreakSpeed);
            if (breakUp)
                bIsOnFloor = false;   // 유지 종료
            // else: 유지
        }
        else
        {
            bIsOnFloor = false;

            FVector accel = ApplyGravity();

            Velocity += accel * DeltaTime;

        }
    }
    */
    if (CanMove)
    {
        FVector Location = GetOwner()->GetActorLocation() + Velocity * DeltaTime;
        GetOwner()->SetActorLocation(Location);
    }

    //GroundLockTest

    if (bGroundLockEnabled)
    {
        //if (!bGroundLocked)
            //TryAcquireGroundLock(GroundSnapDistance);
        //else
            //MaintainGroundLock(DeltaTime);
    }

    //rotation
    if (CanMove)
    {
        /*@@@2
        const float wlen = AngularVelocity.Size();

        const FVector axis = AngularVelocity / wlen;
        const FQuat dq(axis, wlen * DeltaTime); // rad
        FQuat rot = GetOwner()->GetActorQuat();
        rot = dq * rot;
        rot.Normalize();
        GetOwner()->SetActorRotation(rot);
         */
    }

    //Collision
    for (int32 i = 0; i < GSchSdfRegistry.Num(); i++)
    {
        USch_SDFComponent* Other = GSchSdfRegistry[i].Get();

        if (!Other || Other == this) continue;

        //Cluster 충돌 무시
        if (GetClusterRoot() == Other->GetClusterRoot())
        {
            continue;
        }

        FVector CP, N, J; float SD = 0.f;

        const bool bHit = CheckCollision(Other, CP, N, SD, 50);

        if(bHit)
        {
            ProcessImpact(Other, CP, N, SD, DeltaTime, J);
            //ProcessImpactAngular(Other, CP, N, SD, DeltaTime,J);

            //Clustering 시도
            if (bClusterOn && Other->bClusterOn)
            {
                TryCluster(Other, CP);
            }
        }
    }

    //rotation

    if (CanMove)
    {
        
        // 1) 현재 L_body → ω 갱신 (기존 헬퍼)
        SyncLFromAngular();

        const FQuat   R = GetOwner()->GetActorQuat();
        const FVector Tau_body = R.UnrotateVector(AccumTorqueWorld);
        const FVector Omega_body = R.UnrotateVector(AngularVelocity);
        // I·ω (바디 프레임 대각 성분 활용)
        const FVector Iw(
            InertiaTensorLocal.X * Omega_body.X,
            InertiaTensorLocal.Y * Omega_body.Y,
            InertiaTensorLocal.Z * Omega_body.Z
        );
        // Euler's equations: L̇_body = τ_body − ω_body × L_body  (L_body = I·ω)
        const FVector Ldot = Tau_body - FVector::CrossProduct(Omega_body, Iw);

        // 3) 각운동량 업데이트(세미-암시적)
        L_body += Ldot * DeltaTime;

        SyncAngularFromL();  // AngularVelocity(=ω_world) 재동기화

        const float wlen = AngularVelocity.Size();
        if (wlen > KINDA_SMALL_NUMBER)
        {
            const FVector axis = AngularVelocity / wlen;
            const FQuat dq(axis, wlen * DeltaTime);  // 각도 = |ω|·dt (라디안)
            FQuat rot = GetOwner()->GetActorQuat();
            rot = dq * rot;
            rot.Normalize();
            GetOwner()->SetActorRotation(rot);
        }

        AccumTorqueWorld = FVector::ZeroVector;   
    }

    if (bIsOnFloor) SolveContactVelocity(Velocity, FloorNormal, DeltaTime);
}

void USch_SDFComponent::DebugDrawIsoBand(float Band, int32 Step, float PointSize, float LifeTime)
{
    if (!GetWorld()) return;

    if (Step < 1) Step = 1;
    if (Band <= 0.f) Band = 1.f;
    if (PointSize <= 0.f) PointSize = 6.f;

    UStaticMeshComponent* SMC = nullptr;
    if (!ResolveSourceMesh(SMC) || !SMC) return;

    FTransform Xf = SMC->GetComponentTransform();

    FVector minB = Sdf.LocalBounds.Min;
    FVector maxB = Sdf.LocalBounds.Max;
    FVector cell = (maxB - minB) / FVector((float)Sdf.Dim.X, (float)Sdf.Dim.Y, (float)Sdf.Dim.Z);

    for (int32 z = 0; z < Sdf.Dim.Z; z += Step)
    {
        for (int32 y = 0; y < Sdf.Dim.Y; y += Step)
        {
            for (int32 x = 0; x < Sdf.Dim.X; x += Step)
            {
                int32 idx = Sdf.Idx(x, y, z);
                if (!Sdf.Values.IsValidIndex(idx))
                {
                    return;
                }
                float d = Sdf.Values[idx];

                if (FMath::Abs(d) > Band) continue; // 표면 근방만

                // 0에 가까울수록 흰색, 내부면 살짝 초록
                //float t = FMath::Clamp(FMath::Abs(d) / Band, 0.f, 1.f);
                FLinearColor col;
                if (d < 0.f)
                {
                    col = FLinearColor::White; // 내부는 고정 흰색
                }
                else
                {
                    // ❶ 색 대비 스케일: Band와 무관하게 '가까운 거리'를 크게 보이도록 작게 잡음
                    float far = FMath::Max(cell.Size() * 2.0f, Band * 0.20f); // 필요시 1.5~3.0 배 사이로 조절

                    // ❷ 정규화
                    float t = FMath::Clamp(d / far, 0.f, 1.f);

                    // ❸ '빨강 영역'을 넓히는 오프셋(표면 근방을 더 붉게)
                    const float t0 = 0.25f; // 0.10~0.25 사이로 취향 조절
                    t = FMath::Clamp((t - t0) / (1.f - t0), 0.f, 1.f);

                    // ❹ 감마 보정으로 중간톤 띄우기(작을수록 빨강 강조)
                    t = FMath::Pow(t, 0.60f); // 0.45~0.75 범위 시도

                    // ❺ 최종 색: t=0(표면) → 빨강, t=1(멀리) → 파랑
                    col = FLinearColor(1.f - t, 0.f, t);
                }

                FVector localP = minB + FVector((float)x + 0.5f, (float)y + 0.5f, (float)z + 0.5f) * cell;
                FVector worldP = Xf.TransformPosition(localP);
                DrawDebugPoint(GetWorld(), worldP, PointSize, col.ToFColor(true), false, LifeTime);
            }
        }
    }
}

void USch_SDFComponent::DebugDrawGradients(int32 Step, float LengthScale, float LifeTime)
{
    if (!GetWorld()) return;
    const FIntVector D = Sdf.Dim;
    const int32 Total = D.X * D.Y * D.Z;
    if (D.X <= 0 || D.Y <= 0 || D.Z <= 0 || Sdf.Values.Num() < Total) return;

    Step = FMath::Clamp(Step, 1, 1024);

    UStaticMeshComponent* SMC = nullptr;
    if (!ResolveSourceMesh(SMC) || !SMC) return;

    FTransform Xf = SMC->GetComponentTransform();

    const FMatrix IT = Xf.ToMatrixWithScale().InverseFast().GetTransposed();
    const FVector size = Sdf.LocalBounds.GetSize();
    const FVector cell = size / FVector((float)D.X, (float)D.Y, (float)D.Z);
    const FVector minB = Sdf.LocalBounds.Min;

    const float baseLen = cell.GetMin() * LengthScale;
    const float arrowHead = baseLen * 0.25f;

    const bool hasPre = (Sdf.bHasGradients && Sdf.Gradients.Num() == Total);

    for (int32 z = 0; z < D.Z; z += Step)
        for (int32 y = 0; y < D.Y; y += Step)
            for (int32 x = 0; x < D.X; x += Step)
            {
                const int32 idx = Sdf.Idx(x, y, z);

                const FVector pW = Xf.TransformPosition(
                    minB + FVector((x + 0.5f) * cell.X,
                        (y + 0.5f) * cell.Y,
                        (z + 0.5f) * cell.Z));

                const FVector gL = FVector(Sdf.Gradients[idx]);

                const FVector dirW = IT.TransformVector(gL).GetSafeNormal();
                if (dirW.IsNearlyZero()) continue;

                const FVector qW = pW + dirW * baseLen;

                const float dVal = Sdf.Values[idx];
                const FColor col = (dVal < 0.f) ? FColor::Red : FColor::Blue;

                DrawDebugDirectionalArrow(GetWorld(), pW, qW, arrowHead, col,
                    /*bPersistent*/false, LifeTime, 0, /*thickness*/1.0f);
            }
}

void USch_SDFComponent::LoadFromPrebaked()
{
    if (PrebakedAsset.IsNull()) return;
    USchSdfAsset* A = PrebakedAsset.Get();
    if (!A) return;

    Sdf.Dim = A->Grid.Dim;
    Sdf.LocalBounds = A->Grid.LocalBounds;
    Sdf.Values = A->Grid.Values;

    Sdf.Gradients.SetNumUninitialized(A->Grid.Gradients.Num());
    for (int32 i = 0; i < A->Grid.Gradients.Num(); ++i)
        Sdf.Gradients[i] = FVector(A->Grid.Gradients[i]);

    Sdf.bHasGradients = A->Grid.bHasGradients;
}

#if WITH_EDITOR
void USch_SDFComponent::BakeSdfToAsset()
{
    GenerateSdfNow();

    USchSdfAsset* Asset = PrebakedAsset.LoadSynchronous();

    Asset->Grid.Dim = Sdf.Dim;
    Asset->Grid.LocalBounds = Sdf.LocalBounds;
    Asset->Grid.Values = Sdf.Values;

    Asset->Grid.Gradients.SetNumUninitialized(Sdf.Gradients.Num());
    for (int32 i = 0; i < Sdf.Gradients.Num(); ++i)
        Asset->Grid.Gradients[i] = FVector3f(Sdf.Gradients[i]);

    Asset->Grid.bHasGradients = Sdf.bHasGradients;

    Asset->MarkPackageDirty();
}
#endif

bool USch_SDFComponent::SegmentIntervalAgainstLocalBox(
    FVector& P0W,FVector& P1W,
    FTransform& BoxXf,FBox& LocalBounds,
    float& OutTMin, float& OutTMax)
{
    FVector P0 = BoxXf.InverseTransformPosition(P0W);
    FVector P1 = BoxXf.InverseTransformPosition(P1W);
    FVector D = P1 - P0;

    float EPS = 1e-8f;
    float tmin = 0.f, tmax = 1.f;

    // X
    if (FMath::Abs(D.X) < EPS) { if (P0.X < LocalBounds.Min.X || P0.X > LocalBounds.Max.X) return false; }
    else {
        float tx0 = (LocalBounds.Min.X - P0.X) / D.X;
        float tx1 = (LocalBounds.Max.X - P0.X) / D.X;
        if (tx0 > tx1) { float t = tx0; tx0 = tx1; tx1 = t; }
        tmin = FMath::Max(tmin, tx0); tmax = FMath::Min(tmax, tx1); if (tmin > tmax) return false;
    }
    // Y
    if (FMath::Abs(D.Y) < EPS) { if (P0.Y < LocalBounds.Min.Y || P0.Y > LocalBounds.Max.Y) return false; }
    else {
        float ty0 = (LocalBounds.Min.Y - P0.Y) / D.Y;
        float ty1 = (LocalBounds.Max.Y - P0.Y) / D.Y;
        if (ty0 > ty1) { float t = ty0; ty0 = ty1; ty1 = t; }
        tmin = FMath::Max(tmin, ty0); tmax = FMath::Min(tmax, ty1); if (tmin > tmax) return false;
    }
    // Z
    if (FMath::Abs(D.Z) < EPS) { if (P0.Z < LocalBounds.Min.Z || P0.Z > LocalBounds.Max.Z) return false; }
    else {
        float tz0 = (LocalBounds.Min.Z - P0.Z) / D.Z;
        float tz1 = (LocalBounds.Max.Z - P0.Z) / D.Z;
        if (tz0 > tz1) { float t = tz0; tz0 = tz1; tz1 = t; }
        tmin = FMath::Max(tmin, tz0); tmax = FMath::Min(tmax, tz1); if (tmin > tmax) return false;
    }

    OutTMin = tmin; OutTMax = tmax;
    return true;
}

bool USch_SDFComponent::CheckCollision(USch_SDFComponent* Other, FVector& OutContactPoint, FVector& OutNormal, float& OutSignedDistance, int32 MaxIters)
{
    int32 totalSelf = Sdf.Dim.X * Sdf.Dim.Y * Sdf.Dim.Z;
    int32 totalOther = Other->Sdf.Dim.X * Other->Sdf.Dim.Y * Other->Sdf.Dim.Z;
    if (totalSelf <= 0 || totalOther <= 0 ||
        Sdf.Values.Num() < totalSelf || Other->Sdf.Values.Num() < totalOther ||
        Sdf.Gradients.Num() < totalSelf || Other->Sdf.Gradients.Num() < totalOther)
        return false;

    FTransform XfSelf = StaticMesh->GetComponentTransform();
    FTransform XfOther = Other->StaticMesh->GetComponentTransform();
    
    FVector Cself = XfSelf.TransformPosition(Sdf.LocalBounds.GetCenter());
    FVector Cother = XfOther.TransformPosition(Other->Sdf.LocalBounds.GetCenter());
    
    float t0A, t1A, t0B, t1B;
    if (!SegmentIntervalAgainstLocalBox(Cself, Cother, XfSelf, Sdf.LocalBounds, t0A, t1A)) return false;
    if (!SegmentIntervalAgainstLocalBox(Cself, Cother, XfOther, Other->Sdf.LocalBounds, t0B, t1B)) return false;
    float tEnter = FMath::Max(t0A, t0B);
    float tExit = FMath::Min(t1A, t1B);
    if (tEnter > tExit) return false; // 두 OBB가 선분 위에서 겹치지 않음 ⇒ 충돌 불가
        FVector Seed = FMath::Lerp(Cself, Cother, 0.5f * (tEnter + tExit)); // ★ 두 그리드 '동시에 내부' 보장
    float SeedCheckDistance = (Cother - Cself).Size() * 0.25f;

    FBox SelfWB = Sdf.LocalBounds.TransformBy(XfSelf);
    FBox OtherWB = Other->Sdf.LocalBounds.TransformBy(XfOther);

    FVector minV(
        FMath::Max(SelfWB.Min.X, OtherWB.Min.X),
        FMath::Max(SelfWB.Min.Y, OtherWB.Min.Y),
        FMath::Max(SelfWB.Min.Z, OtherWB.Min.Z));
    FVector maxV(
        FMath::Min(SelfWB.Max.X, OtherWB.Max.X),
        FMath::Min(SelfWB.Max.Y, OtherWB.Max.Y),
        FMath::Min(SelfWB.Max.Z, OtherWB.Max.Z));

    if (minV.X < maxV.X && minV.Y < maxV.Y && minV.Z < maxV.Z)
    {
        Seed = (minV + maxV) * 0.5f;   // 교집합 내부(센터) — 두 그리드 모두 안
    }
    else
    {
        return false; // AABB가 안 겹치면 충돌 불가
    }

    int32 idxA;
    int32 idxB;
    FVector x;
    FVector y;

    FVector G = x + y;
    //FMatrix IT_self = GetOwner()->GetTransform().ToMatrixWithScale().InverseFast().GetTransposed();
    //FMatrix IT_other = Other->GetOwner()->GetTransform().ToMatrixWithScale().InverseFast().GetTransposed();

    FMatrix IT_self = StaticMesh->GetComponentTransform().ToMatrixWithScale().InverseFast().GetTransposed();
    FMatrix IT_other = Other->StaticMesh->GetComponentTransform().ToMatrixWithScale().InverseFast().GetTransposed();

    for (int32 i = 0; i < MaxIters; i++)
    {
        idxA = IndexFromWorld(Seed);
        idxB = Other->IndexFromWorld(Seed);

        if (idxA < 0 || idxA >= totalSelf || idxB < 0 || idxB >= totalOther)
        {
            return false;
        }

        x = IT_self.TransformVector(Sdf.Gradients[idxA]);
        y = IT_other.TransformVector(Other->Sdf.Gradients[idxB]);

        float lenA = x.Size();
        float lenB = y.Size();

        x = x / lenA;
        y = y / lenB;

        const float dA = Sdf.Values[idxA] / lenA;
        const float dB = Other->Sdf.Values[idxB] / lenB;

        if ((dA + dB) < 0)
        {
            OutContactPoint = Seed;
            OutNormal = -x;
            OutSignedDistance = dA;

            if (Other->IsFloor)
            {
                bIsOnFloor = true;
                FloorNormal = OutNormal;
                LastFloor = Other;
                FloorHoldUntil = GetWorld()->GetTimeSeconds() + FloorHoldTime;
            }

            if (GEngine)
            {
                GEngine->AddOnScreenDebugMessage(-1, 0.1f, FColor::Green, TEXT("Hit!"));
            }

            if (dA < 0.f)
            {
                const float kSlop = -0.5f;
                const float beta = 1.0f;
                const float corr = FMath::Max(-dA - kSlop, 0.f) * beta;
                if (CanMove)
                {
                    GetOwner()->AddActorWorldOffset(OutNormal * corr);
                }

                float vn_now = FVector::DotProduct(Velocity, OutNormal);
                if (vn_now < 0.f) Velocity -= vn_now * OutNormal;
            }

            return true;
        }
        G = (x + y);
        G.Normalize();
        Seed -= (G * SeedCheckDistance);
        SeedCheckDistance *= 0.5f;

    }

    return false;
}


/*
bool USch_SDFComponent::CheckCollision(USch_SDFComponent* Other, FVector& OutContactPoint, FVector& OutNormal, float& OutSignedDistance, int32 MaxIters)
{
    int32 totalSelf = Sdf.Dim.X * Sdf.Dim.Y * Sdf.Dim.Z;
    int32 totalOther = Other->Sdf.Dim.X * Other->Sdf.Dim.Y * Other->Sdf.Dim.Z;
    if (totalSelf <= 0 || totalOther <= 0 ||
        Sdf.Values.Num() < totalSelf || Other->Sdf.Values.Num() < totalOther ||
        Sdf.Gradients.Num() < totalSelf || Other->Sdf.Gradients.Num() < totalOther)
        return false;

    FVector A = Other->GetOwner()->GetActorLocation();
    FVector B = GetOwner()->GetActorLocation();
    //FVector Seed = (A + B) * 0.5f;
    FVector Seed;

    FTransform XfSelf = StaticMesh->GetComponentTransform();
    FTransform XfOther = Other->StaticMesh->GetComponentTransform();
    FBox SelfWB = Sdf.LocalBounds.TransformBy(XfSelf);
    FBox OtherWB = Other->Sdf.LocalBounds.TransformBy(XfOther);

    FVector minV(
        FMath::Max(SelfWB.Min.X, OtherWB.Min.X),
        FMath::Max(SelfWB.Min.Y, OtherWB.Min.Y),
        FMath::Max(SelfWB.Min.Z, OtherWB.Min.Z));
    FVector maxV(
        FMath::Min(SelfWB.Max.X, OtherWB.Max.X),
        FMath::Min(SelfWB.Max.Y, OtherWB.Max.Y),
        FMath::Min(SelfWB.Max.Z, OtherWB.Max.Z));
    
    if (minV.X < maxV.X && minV.Y < maxV.Y && minV.Z < maxV.Z)
    {
        Seed = (minV + maxV) * 0.5f;   // 교집합 내부(센터) — 두 그리드 모두 안
    }
    else
    {
       return false; // AABB가 안 겹치면 충돌 불가
    }

    //float SeedCheckDistance = (A - B).Size() * 0.25f;
    float SeedCheckDistance = FMath::Min((Seed - A).Size(), (Seed - B).Size()) * 0.25f;

    int32 idxA;
    int32 idxB;
    FVector x;
    FVector y;

    FVector G = x + y;
    //FMatrix IT_self = GetOwner()->GetTransform().ToMatrixWithScale().InverseFast().GetTransposed();
    //FMatrix IT_other = Other->GetOwner()->GetTransform().ToMatrixWithScale().InverseFast().GetTransposed();

    FMatrix IT_self = StaticMesh->GetComponentTransform().ToMatrixWithScale().InverseFast().GetTransposed();
    FMatrix IT_other = Other->StaticMesh->GetComponentTransform().ToMatrixWithScale().InverseFast().GetTransposed();

    for (int32 i = 0; i < MaxIters; i++)
    {
        idxA = IndexFromWorld(Seed);
        idxB = Other->IndexFromWorld(Seed);
        
        if (idxA < 0 || idxA >= totalSelf || idxB < 0 || idxB >= totalOther)
        {
            return false;
        }

        x = IT_self.TransformVector(Sdf.Gradients[idxA]);
        y = IT_other.TransformVector(Other->Sdf.Gradients[idxB]);

        float lenA = x.Size();
        float lenB = y.Size();

        x = x / lenA;
        y = y / lenB;

        //float dA = Sdf.Values[idxA];
        //float dB = Other->Sdf.Values[idxB];
        const float dA = Sdf.Values[idxA] / lenA;
        const float dB = Other->Sdf.Values[idxB] / lenB;
        
        if ((dA + dB) < 0)
        {
            OutContactPoint = Seed;
            OutNormal = -x;
            OutSignedDistance = dA;

            if (Other->IsFloor)
            {
                bIsOnFloor = true;
            }



            if (GEngine)
            {
                GEngine->AddOnScreenDebugMessage(-1,0.1f,FColor::Green,TEXT("Hit!"));
            }

            return true;
        }
        bIsOnFloor = false;

        G = (x + y);
        G.Normalize();
        Seed -= (G * SeedCheckDistance);
        SeedCheckDistance *= 0.5f;
        
    }

    return false;
}
*/

int32 USch_SDFComponent::IndexFromLocal(FVector LocalPos)
{
    FIntVector dim = Sdf.Dim;
    if (dim.X < 1 || dim.Y < 1 || dim.Z < 1) return -1;

    FVector minB = Sdf.LocalBounds.Min;
    FVector size = Sdf.LocalBounds.GetSize();
    if (size.X <= 0.f || size.Y <= 0.f || size.Z <= 0.f) return -1;

    FVector invCell(
        (float)dim.X / size.X,
        (float)dim.Y / size.Y,
        (float)dim.Z / size.Z
    );

    FVector p = (LocalPos - minB) * invCell - FVector(0.5f, 0.5f, 0.5f);

    int32 ix = FMath::RoundToInt(p.X);
    int32 iy = FMath::RoundToInt(p.Y);
    int32 iz = FMath::RoundToInt(p.Z);

    if (ix < 0 || iy < 0 || iz < 0 || ix >= dim.X || iy >= dim.Y || iz >= dim.Z)
        return -1;

    return Sdf.Idx(ix, iy, iz);
}

int32 USch_SDFComponent::IndexFromWorld(FVector WorldPos)
{
    UStaticMeshComponent* SMC = nullptr;
    if (!ResolveSourceMesh(SMC) || !SMC) return -1;

    const FTransform MeshToWorld = SMC->GetComponentTransform();
    const FVector   LocalPos = MeshToWorld.InverseTransformPosition(WorldPos);
    return IndexFromLocal(LocalPos);
}

bool USch_SDFComponent::ProcessImpact(USch_SDFComponent* Other, FVector& ContactPoint, FVector& Normal_WS, float SignedDistance, float DeltaTime, FVector& OutJ)
{
    /*
    if (!Other) return false;

    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(-1, 0.1f, FColor::Green, TEXT("ProcessImpact!"));
    }

    float mA = Mass;
    float mB = Other->Mass;

    FVector VelocityA = Velocity;
    FVector VelocityB = Other->Velocity;

    const FVector vRel = VelocityA - VelocityB;
    const float   vn = FVector::DotProduct(vRel, Normal_WS); // (>0 분리중, <0 접근중)

    if (vn >= 0.f) return false;

    const float e = 0.7f;
    //const float e = (Other->IsFloor || IsFloor) ? 0.0f : 0.7f;
    const float j = -(1.0f + e) * vn / (1/mA + 1/mB);
    const FVector Impulse = j * Normal_WS;

    OutJ = Impulse;

    Velocity = VelocityA + (Impulse / mA);

    Other->Velocity = VelocityB - (Impulse / mB);


    return true;
    */
    //@@@3
    if (!Other) return false;
    USch_SDFComponent* A = GetClusterRoot();
    USch_SDFComponent* B = Other->GetClusterRoot();
    if (!A || !B) return false;

    const FVector n = Normal_WS.GetSafeNormal();

    const FVector comA = A->GetComWorld();
    const FVector comB = B->GetComWorld();
    const FVector rA = ContactPoint - comA;
    const FVector rB = ContactPoint - comB;

    const FVector vA = A->Velocity + FVector::CrossProduct(A->AngularVelocity, rA);
    const FVector vB = B->Velocity + FVector::CrossProduct(B->AngularVelocity, rB);
    const float   vn = FVector::DotProduct(vA - vB, n);
    if (vn >= 0.f) return false;

    auto IinvDot = [](USch_SDFComponent* C, const FVector& w)->float {
        const FQuat R = C->GetOwner()->GetActorQuat();
        const FVector wL = R.UnrotateVector(w);
        const FVector Mw(C->InvInertiaTensorLocal.X * wL.X,
            C->InvInertiaTensorLocal.Y * wL.Y,
            C->InvInertiaTensorLocal.Z * wL.Z);
        const FVector wr = R.RotateVector(Mw);
        return FVector::DotProduct(w, wr);
        };

    const float invMA = 1.f / FMath::Max(A->Mass, KINDA_SMALL_NUMBER);
    const float invMB = 1.f / FMath::Max(B->Mass, KINDA_SMALL_NUMBER);
    const FVector rnA = FVector::CrossProduct(rA, n);
    const FVector rnB = FVector::CrossProduct(rB, n);

    const float denom = invMA + invMB + IinvDot(A, rnA) + IinvDot(B, rnB);
    const float eBase = 0.6f;
    const float vStop = 80.f, vFull = 250.f;
    const float vIn = -vn;
    //float e = (vIn <= vStop) ? 0.f : (vIn >= vFull ? eBase : eBase * FMath::Square((vIn - vStop) / (vFull - vStop)));
    float e;
    if (!Other->IsFloor) 
    {
        e = 0.6f;
    }
    else 
    {
        e = (vIn <= vStop) ? 0.f : (vIn >= vFull ? eBase
            : eBase * FMath::Square((vIn - vStop) / (vFull - vStop)));
    }

    const float j = FMath::Max(0.f, (-(1.f + e) * vn) / FMath::Max(denom, KINDA_SMALL_NUMBER));
    const FVector J = j * n;
    OutJ = J;

    A->ApplyImpulseAtWorldPoint(J, ContactPoint);
    B->ApplyImpulseAtWorldPoint(-J, ContactPoint);
    return true;
}

bool USch_SDFComponent::ProcessImpactAngular(USch_SDFComponent* Other,
    FVector& ContactPoint, FVector& Normal_WS, float SignedDistance, float DeltaTime, FVector j)
{
    if (!Other) return false;

    // 선형에서 구한 J = j * N 를 "각속도만" 적용 (뉴턴 3법칙: ±J)
    const FVector J = j * Normal_WS;

    ApplyImpulseAtWorldPoint(J, ContactPoint);  // 내 객체: +J
    Other->ApplyImpulseAtWorldPoint(-J, ContactPoint); // 상대: -J

    if (GEngine)
    {
        //FString S = FString::Printf(TEXT("[AngularOnly] vn=%.3f  j=%.3f  k=%.6f"), vn, j, k);
        //GEngine->AddOnScreenDebugMessage(-1, 1.5f, FColor::Yellow, S);
    }

    return true;
}


void USch_SDFComponent::ApplyImpulseAtWorldPoint(FVector Impulse, FVector WorldPoint)
{
    /*@@@Test
    //Clustering 부모로 넘기기
    if (!bIsClusterRoot && ClusterParent.IsValid())
    {
        ClusterParent->ApplyImpulseAtWorldPoint(Impulse, WorldPoint);
        return;
    }

    Velocity += Impulse / Mass;

    FVector r = WorldPoint - GetOwner()->GetActorLocation();
    FVector rxJ = FVector::CrossProduct(r, Impulse);

    FQuat   R = GetOwner()->GetActorQuat();
    FVector rxJ_L = R.UnrotateVector(rxJ);  // 월드→바디
    FVector ang_L(InvInertiaTensorLocal.X * rxJ_L.X,InvInertiaTensorLocal.Y * rxJ_L.Y,InvInertiaTensorLocal.Z * rxJ_L.Z);

    FVector ang_W = R.RotateVector(ang_L);  // 바디→월드


    AngularVelocity += ang_W/100.0f;    //언리얼 단위가 cm 젠장

    if (GEngine)
    {
        //FString S = FString::Printf(TEXT("[AngularVelocity] x=%.3f  y=%.3f  z=%.3f"), AngularVelocity[0], AngularVelocity[1], AngularVelocity[2]);
        //GEngine->AddOnScreenDebugMessage(-1, 1.5f, FColor::Yellow, S);
    }
    */
    if (!bIsClusterRoot && ClusterParent.IsValid()) { ClusterParent->ApplyImpulseAtWorldPoint(Impulse, WorldPoint); return; }

    // 선형
    const float m = FMath::Max(Mass, KINDA_SMALL_NUMBER);
    Velocity += Impulse / m;

    //각운동량은 COM 기준으로 갱신 (world → body)
    const FVector comW = GetComWorld();                   // NEW
    const FVector r = WorldPoint - comW;               // ★ COM 기준
    const FVector dL_w = FVector::CrossProduct(r, Impulse);

    const FQuat R = GetOwner()->GetActorQuat();
    L_body += R.UnrotateVector(dL_w);                     // ★ L_body 누적

    // ω/AngularVelocity 동기화
    SyncAngularFromL();
}

USch_SDFComponent* USch_SDFComponent::GetClusterRoot()
{
    USch_SDFComponent* Node = this;
    while (Node && Node->ClusterParent.IsValid())
    {
        Node = Node->ClusterParent.Get();
    }
    return Node ? Node : this;
}

//bool USch_SDFComponent::IsClusteredWith(USch_SDFComponent* Other)
//{
//    return false;
//}

bool USch_SDFComponent::TryCluster(USch_SDFComponent* Other, FVector& ContactPoint)
{
    if (!Other) return false;

    USch_SDFComponent* RootA = GetClusterRoot();
    USch_SDFComponent* RootB = Other->GetClusterRoot();
    if (RootA == RootB) return false;

    USch_SDFComponent* NewRoot = (RootA->Mass >= RootB->Mass) ? RootA : RootB;
    USch_SDFComponent* Child = (NewRoot == RootA) ? RootB : RootA;

    NewRoot->Absorb(Child, ContactPoint);
    return true;
}

void USch_SDFComponent::Absorb(USch_SDFComponent* Child, FVector& ContactPoint)
{
    if (!Child) return;

    float mA = Mass;
    float mB = Child->Mass;

    FVector vA = Velocity;
    FVector vB = Child->Velocity;

    FQuat   qA = GetOwner()->GetActorQuat();
    FQuat   qB = Child->GetOwner()->GetActorQuat();

    FVector xA = GetOwner()->GetActorLocation();
    FVector xB = Child->GetOwner()->GetActorLocation();

    FVector comA_W = xA + qA.RotateVector(CenterOfMassLocal);
    FVector comB_W = xB + qB.RotateVector(Child->CenterOfMassLocal);

    //질량
    float M = mA + mB;
    FVector COM_W = (mA * comA_W + mB * comB_W) / M;

    //Linear
    FVector P = mA * vA + mB * vB;
    FVector vNew = P / M;

    //Angular
    FQuat qRoot = qA;
    FVector rA_L = qRoot.UnrotateVector(comA_W - COM_W);
    FVector rB_L = qRoot.UnrotateVector(comB_W - COM_W);

    // Child의 각속도를 루트 로컬로 변환
    FVector wA_L = qRoot.UnrotateVector(AngularVelocity);
    FVector wB_L = qRoot.UnrotateVector(Child->AngularVelocity);

    // (i) 각운동량(자체 관성) L_ang = I_cm * w
    auto I_diag = [](FVector& d)->FMatrix {
        FMatrix M = FMatrix::Identity;
        M.M[0][0] = d.X; M.M[1][1] = d.Y; M.M[2][2] = d.Z;
        return M;
        };
    // Child의 관성을 루트 축으로 회전(대각만 주는 근사: 회전 후 오프대각 무시)
    FQuat qChildToRoot = qB * qRoot.Inverse();
    FMatrix R = FQuatRotationMatrix(qChildToRoot);
    FMatrix I_B_root_full = R.GetTransposed() * I_diag(Child->InertiaTensorLocal) * R;
    FVector I_B_root_diag(
        I_B_root_full.M[0][0], I_B_root_full.M[1][1], I_B_root_full.M[2][2]);
    FVector I_A_root_diag = InertiaTensorLocal; // 루트는 이미 루트축 로컬

    auto MulDiag = [](FVector& d,FVector& v)->FVector {
        return FVector(d.X * v.X, d.Y * v.Y, d.Z * v.Z);
        };

    FVector L_ang_L =MulDiag(I_A_root_diag, wA_L) + MulDiag(I_B_root_diag, wB_L);

    //선형 운동량의 모멘트 L_lin = r × (m v) (루트 로컬)
    FVector pA_L = qRoot.UnrotateVector(mA * vA);
    FVector pB_L = qRoot.UnrotateVector(mB * vB);
    FVector L_lin_L = FVector::CrossProduct(rA_L, pA_L) + FVector::CrossProduct(rB_L, pB_L);

    FVector L_tot_L = L_ang_L + L_lin_L;

    //총 관성 I_total (루트 로컬, 평행축 정리 - 대각 성분만)
    auto ParallelAxisDiag = [](float m, FVector& r)->FVector {
        float r2 = r.SizeSquared();
        return FVector(m * (r2 - r.X * r.X),
            m * (r2 - r.Y * r.Y),
            m * (r2 - r.Z * r.Z));
        };
    FVector I_total_diag =
        I_A_root_diag + I_B_root_diag +
        ParallelAxisDiag(mA, rA_L) + ParallelAxisDiag(mB, rB_L);

    //w_new ≈ I^{-1}_diag * L  (대각 근사 역행렬)
    FVector wNew_L(
        (I_total_diag.X > KINDA_SMALL_NUMBER) ? L_tot_L.X / I_total_diag.X : 0.f,
        (I_total_diag.Y > KINDA_SMALL_NUMBER) ? L_tot_L.Y / I_total_diag.Y : 0.f,
        (I_total_diag.Z > KINDA_SMALL_NUMBER) ? L_tot_L.Z / I_total_diag.Z : 0.f);

    //상태 적용(루트만 물리 업데이트)
    Mass = M;
    Velocity = vNew;
    AngularVelocity = qRoot.RotateVector(wNew_L);

    // 루트 기준 새 COM 로컬
    CenterOfMassLocal = qRoot.UnrotateVector(COM_W - xA);

    // 루트 로컬 관성(대각 근사치로 갱신)
    InertiaTensorLocal = I_total_diag;
    RecomputeInvInertia();

    //@@@3
    SyncLFromAngular();

    //Child를 루트의 클러스터로 편입
    Child->bIsClusterRoot = false;
    Child->ClusterParent = this;
    ClusterChildren.AddUnique(Child);

    // 자식 이동은 루트에 종속 (물리 Tick 정지)
    Child->CanMove = false;
    Child->AngularVelocity = FVector::ZeroVector;
    Child->Velocity = FVector::ZeroVector;

    // 실제 씬 계층 연결: 월드 변환 유지
    Child->GetOwner()->AttachToActor(GetOwner(), FAttachmentTransformRules::KeepWorldTransform);
}

void USch_SDFComponent::RecomputeInvInertia()
{
    InvInertiaTensorLocal = FVector(
        InertiaTensorLocal.X > 0 ? 1.0f / InertiaTensorLocal.X : 0.0f,
        InertiaTensorLocal.Y > 0 ? 1.0f / InertiaTensorLocal.Y : 0.0f,
        InertiaTensorLocal.Z > 0 ? 1.0f / InertiaTensorLocal.Z : 0.0f
    );
}

void USch_SDFComponent::SolveContactVelocity(FVector& V, const FVector& _FloorNormal, float dt)
{
    const FVector N = _FloorNormal.GetSafeNormal();

    const float vn = FVector::DotProduct(V, N);
    if (vn < 0.f) V -= vn * N;

    FVector vt = FVector::VectorPlaneProject(V, N);
    const float mu = 6.0f;                 // 임의 동마찰 계수
    const float decel = mu * 980.f;        // g 기반 감속
    const float cut = decel * dt;
    const float spd = vt.Size();
    if (spd <= cut) V -= vt;               // 정지
    else            V -= vt * (cut / spd);   // 감속
}

bool USch_SDFComponent::QueryFloorAt(USch_SDFComponent* Floor, const FVector& WorldP, float SearchDown, FVector& OutPoint, FVector& OutNormal, float& OutSigned)
{
    if (!Floor || !Floor->IsFloor) return false;

    const FTransform Xf = Floor->GetOwner()->GetActorTransform(); // 바닥 트랜스폼
    const FTransform Inv = Xf.Inverse();
    const FVector   P_L = Inv.TransformPosition(WorldP);          // 내 위치 → 바닥 로컬

    const FBox LBox = GetLocalBox(Floor);
    const float topZ = LBox.Max.Z;

    // 로컬 XY가 박스 안인가?
    if (P_L.X < LBox.Min.X || P_L.X > LBox.Max.X ||
        P_L.Y < LBox.Min.Y || P_L.Y > LBox.Max.Y) return false;

    // 위에서 얼마만큼 아래에 윗면이 있는지
    const float dz = topZ - P_L.Z;
    if (dz < 0.f || dz > SearchDown) return false; // 너무 멀거나 이미 아래면 실패

    // 히트 지점(로컬) → 월드
    const FVector Hit_L(P_L.X, P_L.Y, topZ);
    OutPoint = Xf.TransformPosition(Hit_L);

    // 윗면 노멀(+Z 로컬 → 월드)
    OutNormal = Xf.TransformVectorNoScale(FVector::UpVector).GetSafeNormal();
    return true;
}

bool USch_SDFComponent::TryAcquireGroundLock(float MaxSearchDown)
{
    if (!bGroundLockEnabled) return false;

    const FVector P = GetOwner()->GetActorLocation();
    USch_SDFComponent* best = nullptr;
    FVector hp, hn; float bestZ = -FLT_MAX;

    extern TArray<TWeakObjectPtr<USch_SDFComponent>> GSchSdfRegistry;
    for (auto& w : GSchSdfRegistry)
    {
        USch_SDFComponent* F = w.Get();
        if (!F || F == this || !F->IsFloor) continue;

        FVector p, n;
        float d;
        if (QueryFloorAt(F, P, MaxSearchDown, p, n, d))
        {
            if (p.Z > bestZ) { bestZ = p.Z; best = F; hp = p; hn = n; }
        }
    }
    if (!best) return false;

    // 스냅 + 노멀 성분 속도 제거
    GetOwner()->SetActorLocation(hp + hn * GroundOffset);
    const float vn = FVector::DotProduct(Velocity, hn);
    if (vn < 0.f) Velocity -= vn * hn;

    GroundLockFloor = best;
    GroundLockNormal = hn;
    bGroundLocked = true;
    return true;
}

void USch_SDFComponent::MaintainGroundLock(float dt)
{
    if (!bGroundLocked || !GroundLockFloor.IsValid()) { ReleaseGroundLock(); return; }

    const FVector P = GetOwner()->GetActorLocation() + GroundLockNormal * 4.f; // 살짝 위에서 다시
    FVector p, n;
    float d;
    if (!QueryFloorAt(GroundLockFloor.Get(), P, GroundSnapDistance + 8.f, p, n, d))
    {
        ReleaseGroundLock(); return;
    }

    // 위로 튀면 해제
    const float vn = FVector::DotProduct(Velocity, n);
    if (vn > GroundBreakSpeed) { ReleaseGroundLock(); return; }

    // 스냅 + 노멀 음수속도 제거 + (선택) 마찰 감속
    GetOwner()->SetActorLocation(p + n * GroundOffset);
    if (vn < 0.f) Velocity -= vn * n;

    FVector vt = FVector::VectorPlaneProject(Velocity, n);
    const float cut = 6.f * 980.f * dt; // 동마찰 근사
    const float spd = vt.Size();
    Velocity -= (spd <= cut) ? vt : vt * (cut / spd);

    GroundLockNormal = n;
}

void USch_SDFComponent::ReleaseGroundLock()
{
    bGroundLocked = false;
    GroundLockFloor.Reset();
}
