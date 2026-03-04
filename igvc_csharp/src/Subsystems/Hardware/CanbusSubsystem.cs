using System.Collections.Concurrent;
using System.Reflection.Metadata;
using System.Runtime.InteropServices;
using igvc_csharp.Core;
using igvc_csharp.Core.Performance;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware.CanLayers;
using igvc_csharp.Subsystems.Simulator;
using Microsoft.Extensions.Logging;
using SocketCANSharp;
using SocketCANSharp.Network;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("CanbusSubsystem")]
public class CanbusSubsystem(SimulatorSubsystem? simulatorSubsystem) : SubsystemBase
{
    // Variables
    private CanNetworkInterface? _canNetwork;
    private RawCanSocket? _canSocket;
    private BlockingCollection<CanFrame> _frameQueue = new(128);
    private volatile bool _connected;
    private readonly Lock _socketLock = new();
    
    // Layers
    public SafetyLightsLayer SafetyLights = null!;
    public MotorControlLayer MotorControl = null!;

    // Metrics
    // Emits the total bits received and written (so bus utilization) over the last second every 250ms
    [Metric("Total Bits Per Second", "bits", Group = "Hardware", Aggregate = MetricAggregate.Sum, EmitEveryMs = 250, MaxAgeSeconds = 1)]
    private PerformanceMetric<double> _bits;
    
    // Emits the total bytes written over the last second every 50ms
    [Metric("Bytes Written", "bytes", Group = "Hardware", Aggregate = MetricAggregate.Sum, EmitEveryMs = 50, MaxAgeSeconds = 1)]
    private PerformanceMetric<double> _bytesWritten;
    
    // Emits the total bytes read over the last second every 50ms
    [Metric("Bytes Read", "bytes", Group = "Hardware", Aggregate = MetricAggregate.Sum, EmitEveryMs = 50, MaxAgeSeconds = 1)]
    private PerformanceMetric<double> _bytesRead;

    // Emits the frame size every 50ms if applicable (only runs when reading). Useful to see
    // if we are not writing fast enough. This shouldn't happen, but it is good to log and check
    [Metric("Frame Queue Size", "length", Group = "Hardware", EmitEveryMs = 50)]
    private PerformanceMetric<double> _queueSize;
    
    public override Task Init(CancellationToken token)
    {
        SafetyLights = new SafetyLightsLayer(this);
        MotorControl = new MotorControlLayer(this);
        
        // We always want this node to be up, but if we are simulating
        // then it should just not write to the socket and instead
        // write to the simulator
        if (Constants.UseSimulation)
        {
            return Task.CompletedTask;
        }
        
        _ = Task.Factory.StartNew(
            () => WriteTask(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );
        _ = Task.Factory.StartNew(
            () => ReadTask(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );
        
        // List all interfaces
        var interfaces = CanNetworkInterface.GetAllInterfaces(true);
        foreach (var inf in interfaces)
        {
            Logger.LogDebug("Found CAN Interface at {Path}", inf.Name);
        }

        return Task.CompletedTask;
    }

    private static CanNetworkInterface? FindNetwork()
    {
        const string inter = Constants.Hardware.CanbusInterface;
        return CanNetworkInterface
            .GetAllInterfaces(true)
            .First(ifc => ifc.Name.Equals(inter));
    }
    
    private void ConnectSocket()
    {
        lock (_socketLock)
        {
            _canNetwork ??= FindNetwork();
            if (_canNetwork == null)
            {
                SetState(SubsystemState.Ready);
                return;
            }

            _frameQueue = new BlockingCollection<CanFrame>(128);
            _canSocket = new RawCanSocket();
            _canSocket.Bind(_canNetwork);
            _connected = true;
            Logger.LogInformation("Connected to Canbus at {socket}", _canNetwork.Name);
            SetState(SubsystemState.Operating);
        }
    }

    private void CloseSocket()
    {
        lock (_socketLock)
        {
            var wasConnected = _connected;
            var oldName = _canNetwork?.Name;
            _connected = false;
            _canSocket?.Dispose();
            _canSocket = null;
            _canNetwork = null;
            
            // Empty the queue
            _frameQueue.CompleteAdding();
            _frameQueue.Dispose();

            if (!wasConnected)
            {
                return;
            }
            
            if (State != SubsystemState.ShuttingDown)
            {
                SetState(SubsystemState.Ready);
            }
                
            Logger.LogInformation("Disconnected from canbus at {socket}", oldName ?? "Unknown");
        }
    }

    private void WriteTask(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            if (!_connected ||  _canSocket == null)
            {
                continue;
            }
            
            if (!_frameQueue.TryTake(out var frame))
            {
                continue;
            }

            try
            { 
                var bytesWritten = _canSocket.Write(frame);
                if (bytesWritten == 0)
                {
                    // Idk if this is an issue tbh
                    continue;
                }
                
                _bits.AddSample(bytesWritten * 8);
                _bytesWritten.AddSample(bytesWritten);
                Logger.LogDebug("Writing can with frame id: {FrameId}", frame.CanId);
            }
            catch (ObjectDisposedException ex)
            {
                Logger.LogWarning(ex, "Canbus socket has been closed.");
                CloseSocket();
            }
            catch (SocketCanException ex)
            {
                Logger.LogCritical(ex, "A critical error has been thrown on the canbus socket");
                CloseSocket();
            }
        }
    }

    private async Task ReadTask(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            if (!_connected || _canSocket == null)
            {
                ConnectSocket();
                await Task.Delay(Constants.Hardware.CanbusTimeout, token);
                continue;
            }

            try
            {
                var bytesRead = _canSocket.Read(out CanFrame frame);
                if (bytesRead == 0)
                {
                    // Idk if this is an issue tbh
                    continue;
                }
                
                _bits.AddSample(bytesRead * 8);
                _bytesRead.AddSample(bytesRead);
                
                EventBus.Instance.Publish(new CanFrameEvent(frame));
            }
            catch (ObjectDisposedException ex)
            {
                Logger.LogWarning(ex, "Canbus socket has been closed.");
                CloseSocket();
            }
            catch (SocketCanException ex)
            {
                Logger.LogCritical(ex, "A critical error has been thrown on the canbus socket");
                CloseSocket();
            }
        }
    }

    public void SendCanFrame(CanFrame frame)
    {
        // If we are simulator, write to it instead
        if (Constants.UseSimulation)
        {
            simulatorSubsystem.SendCanFrame(frame);
            return;
        }
        
        // We don't want to flood the canbus when we connect
        if (!_connected || _canSocket == null)
        {
            return;
        }

        if (_frameQueue.IsAddingCompleted)
        {
            return;
        }
        
        try
        {
            _frameQueue.TryAdd(frame);
        }
        catch (InvalidOperationException ex)
        {
            // the only way this should get called is if our queue fills up, which is a very bad thing
            Logger.LogCritical(ex, "Canbus frame queue has filled up.");
        }
    }

    public override Task Shutdown()
    {
        if (!_connected || _canSocket == null)
        {
            return Task.CompletedTask;
        }
        
        SetState(SubsystemState.ShuttingDown);
        CloseSocket();
        SetState(SubsystemState.Shutdown);
        return Task.CompletedTask;
    }
    
    // Utils
    
    public static byte[] PacketToBytes<T>(T packet) where T : struct
    {
        var bytes = new byte[Marshal.SizeOf<T>()];
        MemoryMarshal.Write(bytes.AsSpan(), packet);
        return bytes;
    }

    public static T PacketFromBytes<T>(byte[] bytes) where T : struct
    {
        return MemoryMarshal.Read<T>(bytes.AsSpan());
    }
}