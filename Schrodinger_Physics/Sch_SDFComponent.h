// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Schrodinger_Physics/SchSdfAsset.h"

#include "Sch_SDFComponent.generated.h"

USTRUCT(BlueprintType)
struct FSdfGrid
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FIntVector Dim = FIntVector(128, 128, 128);

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FBox LocalBounds = FBox();

    // Flattened ZYX indexing: idx = (z*Dim.Y + y)*Dim.X + x
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    TArray<float> Values;

    TArray<FVector> Gradients;
    bool bHasGradients = false;

    int32 Idx(int32 X, int32 Y, int32 Z) const
    {
        return (Z * Dim.Y + Y) * Dim.X + X;
    }

    bool IsValidIndex(int32 X, int32 Y, int32 Z) const
    {
        return (0 <= X && X < Dim.X) && (0 <= Y && Y < Dim.Y) && (0 <= Z && Z < Dim.Z);
    }

    void Init(const FIntVector& InDim, const FBox& InBounds)
    {
        Dim = InDim;
        LocalBounds = InBounds;
        Values.SetNumUninitialized(Dim.X * Dim.Y * Dim.Z);
    }
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class DEFAULT56_PROJECT_API USch_SDFComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	USch_SDFComponent();

    

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    virtual void OnRegister() override;

    virtual void OnUnregister() override;

    //For SDF Stuff
    bool GenerateSdfNow();

    UFUNCTION(BlueprintCallable)
    float SampleSdfWorld(const FVector& WorldPos);

    UFUNCTION(BlueprintCallable)
    float SampleSdfLocal(const FVector& LocalPos);

    bool ResolveSourceMesh(UStaticMeshComponent*& OutSMC);
    bool ExtractTriangles(const UStaticMeshComponent* SMC, TArray<FVector>& OutV, TArray<FIntVector>& OutTri) const;
    void BuildSdf_FromTriangles(const TArray<FVector>& V, const TArray<FIntVector>& Tri, const FBox& MeshLocalBounds);

    float PointTriangleDistance(const FVector& P, const FVector& A, const FVector& B, const FVector& C);
    float Dist_Tri_NearestVertex(const FVector& P, const FVector& A, const FVector& B, const FVector& C);

    bool RayInsideTest(const FVector& P, const TArray<FVector>& V, const TArray<FIntVector>& Tri);
    bool RayIntersectTriangle_PosX(const FVector& P, const FVector& A, const FVector& B, const FVector& C);

    float TrilinearSample(const FVector& LocalPos);
    float SampleSdfLocal_Nearest(const FVector& LocalPos) const;
    FVector GetGradient(FVector& LocalP);

public:
    FVector GetComWorld();
    void   SyncAngularFromL();
    void   SyncLFromAngular();

protected:

    //Basic Physics

    FVector ApplyGravity();



public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    //For SDF Stuff
    UFUNCTION(CallInEditor, BlueprintCallable, Category = "Sch|SDF|Debug")
    void DebugDrawIsoBand(float Band, int32 Step, float PointSize, float LifeTime);

    void DebugDrawGradients(int32 Step, float LengthScale, float LifeTime);

    void LoadFromPrebaked();

    UFUNCTION(CallInEditor, Category = "SDF|Editor")
    void BakeSdfToAsset();


    bool SegmentIntervalAgainstLocalBox(FVector& P0W, FVector& P1W, FTransform& BoxXf, FBox& LocalBounds, float& OutTMin, float& OutTMax);

    UFUNCTION()
    bool CheckCollision(USch_SDFComponent* Other,
        FVector& OutContactPoint,
        FVector& OutNormal,
        float& OutSignedDistance,
        int32 MaxIters = 24);


    //For Manager Test

    bool CheckCollision_QueryOnly(class USch_SDFComponent* Other,
        FVector& OutContactPoint_WS,
        FVector& OutNormal_WS,
        float& OutSignedDistance);

    int32 IndexFromLocal(FVector LocalPos);

    int32 IndexFromWorld(FVector WorldPos);

    //For Basic Physics

    UFUNCTION()
    bool ProcessImpact(USch_SDFComponent* Other,FVector& ContactPoint,FVector& Normal_WS,float SignedDistance,float DeltaTime,FVector& OutJ);

    bool ProcessImpactAngular(USch_SDFComponent* Other, FVector& ContactPoint, FVector& Normal_WS, float SignedDistance, float DeltaTime,FVector J);

    void ApplyImpulseAtWorldPoint(FVector Impulse,FVector WorldPoint);

    void SolveContactVelocity(FVector& V, const FVector& FloorNormal, float dt);

    //GroundLockTest

    bool QueryFloorAt(USch_SDFComponent* Floor, const FVector& WorldP, float SearchDown, FVector& OutPoint, FVector& OutNormal, float& OutSigned);
    bool TryAcquireGroundLock(float MaxSearchDown);
    void MaintainGroundLock(float dt);
    void ReleaseGroundLock();

    //For Clustering

    UFUNCTION(BlueprintCallable)
    USch_SDFComponent* GetClusterRoot();

    bool TryCluster(USch_SDFComponent* Other,FVector& ContactPoint);
    void Absorb(USch_SDFComponent* Child,FVector& ContactPoint);
    void RecomputeInvInertia();



    //For SubSteping
    //For Manager Test
    void VelocityIntegrate(float dt);

    void TransformIntergrate(float dt);
    //For Manager Test

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TWeakObjectPtr<UStaticMeshComponent> StaticMesh;
		
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FIntVector GridRes = FIntVector(128, 128, 128);
	
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FSdfGrid Sdf;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float BoundsScale = 1.05f;

    UPROPERTY(EditAnywhere)
    bool DebugSDFRender = false;

    UPROPERTY(EditAnywhere)
    bool DebugGradientRender = false;

    UPROPERTY(EditAnywhere, Category = "SDF|Runtime") TSoftObjectPtr<USchSdfAsset> PrebakedAsset;
    UPROPERTY(EditAnywhere, Category = "SDF|Editor")  bool bPrecomputeGradients = true;

    //For Basic Physics
    UPROPERTY(EditAnywhere)
    float Mass = 10.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Velocity;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector AngularVelocity;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector CenterOfMassLocal = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector InertiaTensorLocal = FVector(100, 100, 100);

    FVector InvInertiaTensorLocal;

    UPROPERTY()
    FVector L_body = FVector::ZeroVector;   //각운동량

    UPROPERTY()
    FVector AccumTorqueWorld = FVector::ZeroVector;

    UPROPERTY(EditAnywhere)
    bool bLockRotX = false;
    UPROPERTY(EditAnywhere)
    bool bLockRotY = false;
    UPROPERTY(EditAnywhere)
    bool bLockRotZ = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool IsGravityOn = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool CanMove = true;

    UPROPERTY(EditAnywhere)
    bool IsFloor = false;

    bool bIsOnFloor = false;

    FVector FloorNormal = FVector::UpVector;
    

    //GroundLockTest

    UPROPERTY(EditAnywhere, Category = "GroundLock") bool  bGroundLockEnabled = true;
    UPROPERTY(EditAnywhere, Category = "GroundLock") float GroundSnapDistance = 20.f;   // 위에서 이 만큼 아래로만 검색
    UPROPERTY(EditAnywhere, Category = "GroundLock") float GroundOffset = 0.5f;    // 표면에서 띄울 거리
    UPROPERTY(EditAnywhere, Category = "GroundLock") float MaxGroundSlopeDeg = 55.f;    // 이보다 가파르면 접지 안 함
    UPROPERTY(EditAnywhere, Category = "GroundLock") float GroundBreakSpeed = 50.f;   // 노멀 양(+)속도 초과시 분리

    bool    bGroundLocked = false;
    FVector GroundLockNormal = FVector::UpVector;
    TWeakObjectPtr<USch_SDFComponent> GroundLockFloor;


    UPROPERTY(EditAnywhere, Category = "GroundLock")
    float FloorHoldTime = 0.5f;


    double FloorHoldUntil = 0.0;        // 내부 타이머(초)
    TWeakObjectPtr<USch_SDFComponent> LastFloor; // 마지막 접지 바닥

    //For Clustering
    UPROPERTY(EditAnywhere)
    bool bClusterOn = false; // 둘 다 켜져있으면 충돌 시 붙기

    UPROPERTY(EditAnywhere)
    bool bIsClusterRoot = true;

    UPROPERTY()
    TWeakObjectPtr<USch_SDFComponent> ClusterParent;
    
    UPROPERTY()
    TArray<TWeakObjectPtr<USch_SDFComponent>> ClusterChildren;

    UPROPERTY(EditAnywhere)
    bool UsePhysicManager = true;
};

FVector BaryClamp(const FVector& P, const FVector& A, const FVector& B, const FVector& C, FVector& OutClosest);
