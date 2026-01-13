using System.Runtime.InteropServices;
using igvc_csharp.CanSpec;
using igvc_csharp.Core;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("ControllerSubsystem", DependsOn = [typeof(CanbusSubsystem)])]
public class ControllerSubsystem (CanbusSubsystem canbus) : SubsystemBase
{
    private MotorCommandMessage msg = new MotorCommandMessage(0, 0, 0);
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
            canbus.WriteFrame(msg.Write());
            await Task.Delay(TimeSpan.FromMilliseconds(20));
        }
    }
    
    float NormalizeAxis(int raw, int min, int max)
    {
        float center = (min + max) * 0.5f;
        float halfRange = (max - min) * 0.5f;

        if (halfRange == 0)
            return 0;

        return (raw - center) / halfRange;
    }
    
    float ApplyDeadZone(float value, float deadZone = 0.1f)
    {
        if (Math.Abs(value) < deadZone)
            return 0f;

        // Re-scale so full range is still reachable
        return Math.Sign(value) *
            (Math.Abs(value) - deadZone) / (1f - deadZone);
    }
    
    // Reading
    
    void ReadLoop(FileStream fs, CancellationToken token)
    {
        int size = Marshal.SizeOf<InputEvent>();
        byte[] buffer = new byte[size];
        
        const float MAX_FORWARD_VEL = 3.0f;   // meters/sec (or units/sec)
        const float MAX_STRAFE_VEL  = 2.0f;
        const float MAX_ANGULAR_VEL = 2.5f;   // radians/sec

        while (!token.IsCancellationRequested)
        {
            int read = fs.Read(buffer, 0, size);
            if (read < size)
            {
                msg = new MotorCommandMessage(0, 0, 0);
                continue;
            }

            var evt = ByteArrayToStructure<InputEvent>(buffer);

            // Logger.LogInformation("Xbox Event: Type={Type}, Code={Code}, Value={Value}", evt.type, evt.code, evt.value);
            if (evt is { type: 3 })
            {
                float lX = 0;
                float lY = 0;
                float rX = 0;
                float rY = 0;
                switch (evt.code)
                {
                    case 0:
                        lX = ApplyDeadZone(NormalizeAxis(evt.value, -32768, 32768));
                        break;
                    case 1:
                        lY = ApplyDeadZone(NormalizeAxis(evt.value, -32768, 32768));
                        break;
                    case 3:
                        rX = ApplyDeadZone(NormalizeAxis(evt.value, -32768, 32768));
                        break;
                    case 4:
                        rY = ApplyDeadZone(NormalizeAxis(evt.value, -32768, 32768));
                        break;
                }
                
                float forwardVelocity  = -lY * MAX_FORWARD_VEL * 5;
                float sidewaysVelocity =  lX * MAX_STRAFE_VEL * 5;
                float angularVelocity = rX * MAX_ANGULAR_VEL * 5;

                msg = new MotorCommandMessage((short)forwardVelocity, (short)sidewaysVelocity, (short)angularVelocity);
            }
        }
        
        fs.Dispose();
    }

    
    static T ByteArrayToStructure<T>(byte[] bytes) where T : struct
    {
        GCHandle handle = GCHandle.Alloc(bytes, GCHandleType.Pinned);
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
    struct TimeVal
    {
        public long tv_sec;   // seconds
        public long tv_usec;  // microseconds
    }

    [StructLayout(LayoutKind.Sequential)]
    struct InputEvent
    {
        public TimeVal time;
        public ushort type;
        public ushort code;
        public int value;
    }

}