// Copyright Epic Games, Inc. All Rights Reserved.

#include "testButtonStyle.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/SlateStyleRegistry.h"
#include "Slate/SlateGameResources.h"
#include "Interfaces/IPluginManager.h"
#include "Styling/SlateStyleMacros.h"

#define RootToContentDir Style->RootToContentDir

TSharedPtr<FSlateStyleSet> FtestButtonStyle::StyleInstance = nullptr;

void FtestButtonStyle::Initialize()
{
	if (!StyleInstance.IsValid())
	{
		StyleInstance = Create();
		FSlateStyleRegistry::RegisterSlateStyle(*StyleInstance);
	}
}

void FtestButtonStyle::Shutdown()
{
	FSlateStyleRegistry::UnRegisterSlateStyle(*StyleInstance);
	ensure(StyleInstance.IsUnique());
	StyleInstance.Reset();
}

FName FtestButtonStyle::GetStyleSetName()
{
	static FName StyleSetName(TEXT("testButtonStyle"));
	return StyleSetName;
}


TSharedRef< FSlateStyleSet > FtestButtonStyle::Create()
{
	TSharedRef< FSlateStyleSet > Style = MakeShareable(new FSlateStyleSet("testButtonStyle"));
	Style->SetContentRoot(IPluginManager::Get().FindPlugin("newton")->GetBaseDir() / TEXT("Resources"));

	//Style->Set("testButton.PluginAction", new IMAGE_BRUSH_SVG(TEXT("PlaceholderButtonIcon"), Icon40x40));
	//Style->Set("testButton.PluginAction", new IMAGE_BRUSH_SVG(TEXT("newtonIcon"), Icon40x40));

	const FVector2D Icon40x40(40.0f, 40.0f);
	FSlateImageBrush* const newtonIcon = new IMAGE_BRUSH("newtonIcon", Icon40x40);
	Style->Set("newton.PluginAction", newtonIcon);

	return Style;
}

void FtestButtonStyle::ReloadTextures()
{
	if (FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().GetRenderer()->ReloadTextureResources();
	}
}

const ISlateStyle& FtestButtonStyle::Get()
{
	return *StyleInstance;
}
