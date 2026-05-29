using igvc_csharp.Core;
using igvc_csharp.Core.Chronos;
using igvc_csharp.Core.Units;
using SocketCANSharp;

namespace igvc_csharp.Subsystems;

[Subsystem("ChronosSubsystem")]
public class ChronosSubsystem : AbstractChronosSubsystem
{
    // Ids
    public static class EntryId
    {
        // General
        public const ushort GenericEvent = 0x1000;
        public const ushort CanFrame = 0x1001;

        // Data
        public const ushort Gps = 0x1100;
        public const ushort Ypr = 0x1101;

        // Current Sensor
        public const ushort CurrentSense = 0x1200;
        public const ushort VoltageSense = 0x1201;

        // Performance
        public const ushort SystemPerformance = 0x1300;
    }

    public static class CameraId
    {
        public const int Left = 0x0001;
        public const int Right = 0x0002;
        public const int Zed2i = 0x0003;
    }

    // Overrides
    protected override void OnRunStarted(string runId, ushort sessionType)
    {
        using var ms = new MemoryStream(sizeof(ushort));
        using var bw = new BinaryWriter(ms);
        bw.Write(sessionType);
        WriteEntry(EntryTypeId.SessionStart, ms.ToArray());

        // Open Cameras
        // OpenCamera(CameraId.Left, 640, 480, 12);
        // OpenCamera(CameraId.Right, 640, 480, 12);
        // OpenCamera(CameraId.Zed2i, 1280, 720, 20, "bgra");
    }

    // Helpers
    public void WriteCan(CanFrame frame)
    {
        // CanId (uint) + DataLength (int) + Data(byte[])
        using var ms = new MemoryStream(sizeof(uint) + sizeof(int) + frame.Data.Length);
        using var bw = new BinaryWriter(ms);
        bw.Write(frame.CanId);
        bw.Write(frame.Data.Length);
        bw.Write(frame.Data);
        WriteEntry(EntryId.CanFrame, ms.ToArray());
    }

    public void WriteGps(LatLng gps, byte fix, byte numSatellites)
    {
        // 2x double, 2x bytes | lat, lng, gps fix, num satellites
        using var ms = new MemoryStream((sizeof(double) * 2) + 2);
        using var writer = new BinaryWriter(ms);
        writer.Write(gps.Latitude);
        writer.Write(gps.Longitude);
        writer.Write(fix);
        writer.Write(numSatellites);
        WriteEntry(EntryId.Gps, ms.ToArray());
    }

    public void WriteYpr(Ypr ypr)
    {
        using var ms = new MemoryStream(sizeof(double) * 3);
        using var writer = new BinaryWriter(ms);
        writer.Write(ypr.Yaw);
        writer.Write(ypr.Pitch);
        writer.Write(ypr.Roll);
        WriteEntry(EntryId.Ypr, ms.ToArray());
    }

    public void WriteCurrentSense(double current)
    {
        using var ms = new MemoryStream(sizeof(double));
        using var writer = new BinaryWriter(ms);
        writer.Write(current);
        WriteEntry(EntryId.CurrentSense, ms.ToArray());
    }

    public void WriteVoltageSense(double voltage)
    {
        using var ms = new MemoryStream(sizeof(double));
        using var writer = new BinaryWriter(ms);
        writer.Write(voltage);
        WriteEntry(EntryId.VoltageSense, ms.ToArray());
    }

    public void WriteSystemPerformance(float ramUsage, float cpuUsage, float gpuUsage)
    {
        using var ms = new MemoryStream(sizeof(float) * 3);
        using var writer = new BinaryWriter(ms);
        writer.Write(ramUsage);
        writer.Write(cpuUsage);
        writer.Write(gpuUsage);
        WriteEntry(EntryId.SystemPerformance, ms.ToArray());
    }
}