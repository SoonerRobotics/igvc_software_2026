using Google.FlatBuffers;
using igvc_csharp.Core;
using igvc_csharp.Core.Hardware;
using igvc_csharp.Core.Units;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using VectornavReport = Messages.VectornavReport;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("VectorNavSubsystem", Disabled = Configuration.UseSimulation)]
public class VectorNavSubsystem() : SubsystemBase
{
    private Task? _readTask;
    private CancellationTokenSource? _cts;
    private ProcessManager? _vnProcessManager;

    // Properties
    private readonly SubsystemProperty<string> _pBaudRate = new("baudrate");
    private readonly SubsystemProperty<uint> _pLastSequence = new("sequence", 0);

    public override async Task Init(CancellationToken token)
    {
        _cts = CancellationTokenSource.CreateLinkedTokenSource(token);
        _readTask = Task.Run(() => ReadLoop(_cts.Token), _cts.Token);
        SetOperatingState(SubsystemState.Idle);

        var vnConfig = new ProcessManagerConfig
        {
            AutoRestart = true,
            RestartDelayMs = 3000,
            CrashThresholdMs = 3000,
            GracefulShutdownTimeoutMs = 3000
        };
        _vnProcessManager = new ProcessManager(
            Path.Combine(FileUtils.GetRepositoryRootDirectory(), "igvc_vectornav", "build", "igvc_vectornav"), 
            vnConfig
        );
        _vnProcessManager.LogReceived += OnLogReceived;
        await _vnProcessManager.StartAsync(token);
    }

    private void OnLogReceived(object? sender, SpdLogStructure log)
    {
        if (log.Message.StartsWith("VECTORNAV_STARTING"))
        {
            SetOperatingState(SubsystemState.Starting);
            return;
        }

        if (log.Message.StartsWith("VECTORNAV_CONNECTION_FAILED"))
        {
            SetOperatingState(SubsystemState.Errored);
            SetError("CONNECTION_FAILED");
            Logger.LogWarning("Vectornav conection failed!");
            return;
        }

        if (log.Message.StartsWith("SENSOR_SETUP_FAILED"))
        {
            SetOperatingState(SubsystemState.Errored);
            SetError("SENSOR_SETUP_FAILED");
            Logger.LogError("Vectornav setup failed!");

            return;
        }

        if (log.Message.StartsWith("BAUD_RATE_"))
        {
            // strip it and extract the baudrate
            var baudRate = log.Message.Replace("BAUD_RATE_", "");
            _pBaudRate.Set(baudRate);
            Logger.LogInformation("VN Connected (baudrate={})", baudRate);
            return;
        }

        if (log.Message.StartsWith("VECTORNAV_DISCONNECTED"))
        {
            SetOperatingState(SubsystemState.Errored);
            _pBaudRate.Set(null);
            ClearError();
            return;
        }
    }

    private async Task ReadLoop(CancellationToken token)
    {
        using var shm = new VectorNavSharedMemoryReader();

        while (!token.IsCancellationRequested)
        {
            shm.Close();

            while (!token.IsCancellationRequested && !shm.IsOpen)
            {
                if (shm.TryOpen())
                {
                    break;
                }

                await Task.Delay(1000, token).ConfigureAwait(false);
            }

            if (!shm.IsOpen) break;

            uint lastSeq = 0;
            var lastNewDataAt = DateTime.UtcNow;
            const int stalenessThresholdMs = 5000;
            try
            {
                while (!token.IsCancellationRequested)
                {
                    var report = shm.TryRead(timeoutMs: 100);

                    if (report is null)
                    {
                        if ((DateTime.UtcNow - lastNewDataAt).TotalMilliseconds > stalenessThresholdMs)
                        {
                            SetError("VN_NO_DATA");
                            break;
                        }

                        await Task.Delay(10, token).ConfigureAwait(false);
                        continue;
                    }

                    if (report.Value.SequenceNum == lastSeq)
                    {
                        if ((DateTime.UtcNow - lastNewDataAt).TotalMilliseconds > stalenessThresholdMs)
                        {
                            SetError("VN_STALE_DATA");
                            break;
                        }

                        await Task.Delay(10, token).ConfigureAwait(false);
                        continue;
                    }

                    lastSeq = report.Value.SequenceNum;
                    lastNewDataAt = DateTime.UtcNow;

                    if (report.Value.Valid == 0)
                    {
                        Logger.LogWarning("[#{Seq}] VectorNav report marked invalid", report.Value.SequenceNum);
                        continue;
                    }

                    SetOperatingState(SubsystemState.Operating);
                    SetError(string.Empty);
                    var r = report.Value;
                    _pLastSequence.Set(r.SequenceNum);

                    var timestamp = DateTimeOffset.FromUnixTimeMilliseconds(r.TimestampUs / 1000);
                    var builder = new FlatBufferBuilder(1024);
                    var reportOffset = VectornavReport.CreateVectornavReport(
                        builder,
                        TimeUtils.Now(),
                        r.SequenceNum,
                        r.Latitude,
                        r.Longitude,
                        r.Pitch,
                        r.Roll,
                        r.Yaw,
                        r.VelNorthMs,
                        r.VelEastMs,
                        r.VelDownMs,
                        (sbyte)r.NumSats,
                        (sbyte)r.GpsFix
                    );
                    builder.Finish(reportOffset.Value);
                    var reportMessage = MessageWrapper.From(MessageType.VectorNav, builder.SizedByteArray());
                    EventBus.Instance.Publish(
                        new MessageWrapperEvent(reportMessage)
                    );

                    // Chronos
                    // chronos.WriteGps(new LatLng(r.Latitude, r.Longitude), r.GpsFix, r.NumSats);
                    // chronos.WriteYpr(new Ypr(r.Yaw, r.Pitch, r.Roll));

                    // Logger.LogTrace(
                    //     "[#{Seq}] {Time} | " +
                    //     "Lat: {Lat:F6}, Lon: {Lon:F6}, Alt: {Alt:F2}m | " +
                    //     "Yaw: {Yaw:F2}, Pitch: {Pitch:F2}, Roll: {Roll:F2} | " +
                    //     "Vel N/E/D: {VN:F2}/{VE:F2}/{VD:F2} m/s | " +
                    //     "Sats: {Sats}, Fix: {Fix}",
                    //     r.SequenceNum, timestamp,
                    //     r.Latitude, r.Longitude, r.Altitude,
                    //     r.Yaw, r.Pitch, r.Roll,
                    //     r.VelNorthMs, r.VelEastMs, r.VelDownMs,
                    //     r.NumSats, r.GpsFix);
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "Unexpected error in VectorNav read loop, reconnecting...");
                await Task.Delay(1000, token).ConfigureAwait(false);
            }
        }

        Logger.LogInformation("VectorNav read loop stopped");
    }

    public override async Task Shutdown()
    {
        // Stop the process
        if (_vnProcessManager?.Status == ProcessStatus.Running)
        {
            await _vnProcessManager.StopAsync();
        }

        // Stop the read task
        if (_readTask is not null)
        {
            await _readTask.ConfigureAwait(false);
        }
    }
}