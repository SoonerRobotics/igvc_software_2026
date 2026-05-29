using igvc_csharp;
using igvc_csharp.Core.Config;
using igvc_csharp.Utils.Messages;

// Initialize boot parameters
var result = BootParameters.Initialize(args);
if (!result)
{
    return 1;
}

// Register flatbuffer stuff
FlatBufferRegistry.Scan();

// Initialize configuration & presets
ConfigManager.Initialize();
// PresetManager.Initialize();

// Run the robot
await RobotManager.Run();

return 0;