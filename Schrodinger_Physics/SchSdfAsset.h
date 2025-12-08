// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "SchSdfAsset.generated.h"

/**
 * 
 */

USTRUCT(BlueprintType)
struct FSchSdfGridData
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere) FIntVector Dim;
    UPROPERTY(EditAnywhere) FBox       LocalBounds;

    UPROPERTY(EditAnywhere) TArray<float>     Values;

    UPROPERTY(EditAnywhere) TArray<FVector3f> Gradients;

    UPROPERTY(EditAnywhere) bool bHasGradients = false;
};

UCLASS()
class DEFAULT56_PROJECT_API USchSdfAsset : public UDataAsset
{
	GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere, Category = "SDF")
    FSchSdfGridData Grid;

    // (선택) 이게 왜 선택이지
    UPROPERTY(EditAnywhere, Category = "Build Info")
    TSoftObjectPtr<UStaticMesh> SourceMesh;

    UPROPERTY(VisibleAnywhere, Category = "Build Info")
    FString BuildKey; // StaticMesh GUID + GridRes + BoundsScale 등 해시 ???? 이게 멍미
};
