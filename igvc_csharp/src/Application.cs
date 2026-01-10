using igvc_csharp;
using igvc_csharp.Core.Config;
using igvc_csharp.Utilities;

// Register flatbuffer stuff
FlatBufferRegistry.Scan();

// Initialize configuration & presets
ConfigManager.Initialize();
PresetManager.Initialize();

// Run the robot
await RobotManager.Run();