# IGVC 2026 Software Implementation

## Overview

This software will host the working code for our IGVC (Intelligent Ground Vehicle Competition) 2026 robot. It will primarily be written in C# using modern language features such as async/await. There will be a few core classes/concepts that everything is based around and are listed as follows.

## Runtime & Platform

This software will run on Ubuntu 24.04 using .NET version 10.0.101.

### Logging

The `Logging.cs` class will contain a simple set of helpers for logging, using Microsoft.Extensions.Logging. By default it should create a ILoggerFactory that outputs to console and a way to output to files rotating. Other files will call this class to create a static logger for its class, such `Logger.From<Class>()`.

### Application

The `Application.cs` class will contain two lines of code, `RobotManager.Init()` and `RobotManager.Dispose()`. They will both be async and will be awaited, we will be using modern top level C# (https://learn.microsoft.com/en-us/dotnet/csharp/tutorials/top-level-statements).

### Constants

The `Constants.cs` class will contain a bunch of constants used throughout the app. For example, there should be a `Constants.PeriodicRate` which is of type TimeSpan used later. Or, for a subsystem this may look like `Constants.VisionSubsystem.BlurRadius`.

### RobotManager

The `RobotManager.cs` class will contain `Init` and `Dispose` methods, called by `Application`. This class will create a `Robot` instance (initialize `Robot.Instance`) and call the initialize method on the `Robot` instance. After being created, it will run the `Periodic` function on `Robot` at a rate equal to `Constants.PeriodicRate`. This class should also handle shutting down the system gracefully, listening for "Ctrl + C" and similar events. On `Shutdown`, it should stop the periodic loop and then call Robot.Shutdown() which will shutdown all subsystems, and then finally dispose of the `Robot`. RobotManager will contain a single CancellationTokenSource that we use to coordinate shutdown throughout the entire program and will be passed to Robot, periodic loops, and subsystems.

The Periodic Loop will used a fixed-delay async loop so that if a periodic invocation overuns, there will be no pileup/backlog. Given that, we will use one large global Task for now and subsystems can create their own.

### BaseRobot

The `BaseRobot.cs` class will handle a few specific lowlevel things like calling `Periodic` on Subsystems. The `Robot` class should call the super method on all extended methods.

### Robot

The `Robot.cs` class will contain all of the methods listed above. It will also contain a list of Subsystems that are initialzied and created within the robot. The list of subsystems that will be created should be created automatically using reflection and attributes. This may be something like `[Subsystem("VisionSubsystem")]`, and should also contain additional metadata like `Disabled`. This attribute will be applied directly to classes implementing `ISubSystem` and should be checked for when fetching available subsystems. The RobotManager should have a `GetEnabledSubsystems()` function that will handle finding all of the Subsystems, `Disabled` subsystems will not be returned from this function and thus should not be initialized.

#### RobotState

The `RobotState.cs` class will contain a few simple fields describing the current state of the robot as follows. It should be noted that because this can be accessed and modified from multiple threads, it needs to be thread safe.
- Mode (Describes the current mode of the entire system/robot, as described by `RobotModeEnum`)
- MotionAllowed (Describes whether the robot is allowed to move when in autonomous mode. This is separate from the `RobotModeEnum` system and is used to test autonomous systems while ensuring the robot can not move)
- Mission (Describes the current mission of the robot. Particiularly, whether it is competiting in Autonav or Selfdrive. This is described by `MissionEnum`.)
- IsSimulation (Describes whether or not the robot is being simulated or is being ran in the real world.)

#### RobotModeEnum

The `RobotModeEnum.cs` enum will decsribe the current mode of the entire robot/system as a `byte` and should be the following:
- Disabled (Describes a system that is performing no operations and is incapable of doing anything)
- Manual (Describes a system that is performing manual control operations)
- Autonomous (Describes a system that is performing autonomous operations)

#### MissionEnum

The `MissionEnum.cs` enum will describe the current mission of the robot as a `byte` and should be following:
- Autonav (Describes a system that is running the Autonav mission)
- Selfdrive (Describes a system that is running the Selfdrive mission)

### ISubsystem

The `ISubsystem.cs` class will be an interface for future sub systems to inherit from. It should contain a `Init`, `Periodoic`, `Shutdown`, and `Restart` method. These should either be ran on their own threads or use the fancy new async/await functionality if that makes sense for this. Subsystems may perform long running tasks such as vision processing, ai segmentation, etc. In some cases, subsystems may implement services like WebsocketServers for frontend communication, etc. `Restart` will be a custom implementation per subsystem such that they must implement their own logic. A Subsystem will handle its own errors and is responsible for setting its own state.

#### SubsystemState

The `SubsystemState.cs` enum will describe the current state of a subsystem as a `byte` and should be the following:
- Initialized (Describes a subsystem that is initialized but is not yet ready to operate. This may occur when a subsystem is waiting for a serial device to connect or something similar.)
- Ready (Describes a subsystem that is ready to operate but has no reason to yet. This may occur when say the PathfindingSubsystem has no reason to operate because the robot is not autonomously navigating)
- Operating (Describes a subsystem that is actively operating and is performing operating. This may occur when say the VisionSubsystem is actively transforming images)
- Errored (Describes a subsystem that has errored and is able to be restarted.)
- Fatal (Describes a subsystem that has fatally errored and is unable to be restarted.)
