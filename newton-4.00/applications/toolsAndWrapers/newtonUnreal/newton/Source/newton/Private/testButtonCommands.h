// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Framework/Commands/Commands.h"
#include "testButtonStyle.h"

class FtestButtonCommands : public TCommands<FtestButtonCommands>
{
public:

	FtestButtonCommands()
		: TCommands<FtestButtonCommands>(TEXT("newton"), NSLOCTEXT("Contexts", "newton", "newton Plugin"), NAME_None, FtestButtonStyle::GetStyleSetName())
	{
	}

	// TCommands<> interface
	virtual void RegisterCommands() override;

public:
	TSharedPtr< FUICommandInfo > PluginAction;
};
