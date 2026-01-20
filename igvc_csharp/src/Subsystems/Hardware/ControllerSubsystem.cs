using System.Runtime.InteropServices;
using igvc_csharp.CanSpec;
using igvc_csharp.Core;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("ControllerSubsystem", DependsOn = [typeof(CanbusSubsystem)], Disabled = false)]
public class ControllerSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    private MotorCommandMessage _msg = new(0, 0, 0);

    public override Task Init(CancellationToken token)
    {
        var fs = new FileStream(
            "/dev/input/event13",
            FileMode.Open,
            FileAccess.Read,
            FileShare.ReadWrite,
            bufferSize: Marshal.SizeOf<InputEvent>(),
            useAsync: true
        );

        _ = Task.Run(
            () => ReadLoop(fs, token),
            token
        );

        _ = Task.Run(
            () => WriteControllerLoop(token),
            token
        );

        return Task.CompletedTask;
    }

    async Task WriteControllerLoop(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            canbus?.WriteFrame(_msg.Write());
            await Task.Delay(TimeSpan.FromMilliseconds(20));
        }
    }

    private static float NormalizeAxis(int raw, int min, int max)
    {
        var center = (min + max) * 0.5f;
        var halfRange = (max - min) * 0.5f;

        if (halfRange == 0)
        {
            return 0;
        }

        return (raw - center) / halfRange;
    }

    private static float ApplyDeadZone(float value, float deadZone = 0.1f)
    {
        if (Math.Abs(value) < deadZone)
        {
            return 0f;
        }

        return Math.Sign(value) * (Math.Abs(value) - deadZone) / (1f - deadZone);
    }

    // Reading

    private void ReadLoop(FileStream fs, CancellationToken token)
    {
        var size = Marshal.SizeOf<InputEvent>();
        var buffer = new byte[size];

        const float maxForwardVel = 3.0f;
        const float maxStrafeVel = 2.0f;
        const float maxAngularVel = 2.5f;

        while (!token.IsCancellationRequested)
        {
            var read = fs.Read(buffer, 0, size);
            if (read < size)
            {
                _msg = new MotorCommandMessage(0, 0, 0);
                continue;
            }

            var evt = ByteArrayToStructure<InputEvent>(buffer);

            if (evt is not { type: 3 }) continue;
            
            switch (evt.code)
            {
                case 0:
                    var lX = ApplyDeadZone(NormalizeAxis(evt.value, -32768, 32768));
                    _msg.SidewaysVelocity = (short)(lX * maxStrafeVel / 0.0001f);
                    break;
                case 1:
                    var lY = ApplyDeadZone(NormalizeAxis(evt.value, -32768, 32768));
                    _msg.ForwardVelocity = (short)(-lY * maxForwardVel / 0.0001f);
                    break;
                case 3:
                    var rX = ApplyDeadZone(NormalizeAxis(evt.value, -32768, 32768));
                    _msg.SidewaysVelocity = (short)(rX * maxStrafeVel / 0.001f);
                    break;
            }
        }

        fs.Dispose();
    }


    private static T ByteArrayToStructure<T>(byte[] bytes) where T : struct
    {
        var handle = GCHandle.Alloc(bytes, GCHandleType.Pinned);
        try
        {
            return Marshal.PtrToStructure<T>(handle.AddrOfPinnedObject());
        }
        finally
        {
            handle.Free();
        }
    }

    // EVDev Structs

    [StructLayout(LayoutKind.Sequential)]
    private struct TimeVal
    {
        public long tv_sec; // seconds
        public long tv_usec; // microseconds
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct InputEvent
    {
        public TimeVal time;
        public ushort type;
        public ushort code;
        public int value;
    }
}