using Google.FlatBuffers;
using igvc_csharp.Core;
using igvc_csharp.Core.Hardware;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("VectorNavSubsystem")]
public class VectorNavSubsystem : SubsystemBase
{
    private Task? _readTask;
    private CancellationTokenSource? _cts;

    public override Task Init(CancellationToken token)
    {
        _cts = CancellationTokenSource.CreateLinkedTokenSource(token);
        _readTask = Task.Run(() => ReadLoop(_cts.Token), _cts.Token);
        SetOperatingState(SubsystemState.Idle);
        return Task.CompletedTask;
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
                    Logger.LogInformation("VectorNav shared memory opened");
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
                            Logger.LogWarning("VectorNav shared memory appears stale, reconnecting...");
                            break; // back to connect phase
                        }

                        await Task.Delay(10, token).ConfigureAwait(false);
                        continue;
                    }

                    if (report.Value.SequenceNum == lastSeq)
                    {
                        if ((DateTime.UtcNow - lastNewDataAt).TotalMilliseconds > stalenessThresholdMs)
                        {
                            Logger.LogWarning("VectorNav sequence number frozen, reconnecting...");
                            break; // back to connect phase
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

                    var r = report.Value;
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
                    var reportMessage = MessageWrapper.From(MessageType.Gps, builder.SizedByteArray());
                    EventBus.Instance.Publish(
                        new MessageWrapperEvent(reportMessage)
                    );

                    Logger.LogTrace(
                        "[#{Seq}] {Time} | " +
                        "Lat: {Lat:F6}, Lon: {Lon:F6}, Alt: {Alt:F2}m | " +
                        "Yaw: {Yaw:F2}, Pitch: {Pitch:F2}, Roll: {Roll:F2} | " +
                        "Vel N/E/D: {VN:F2}/{VE:F2}/{VD:F2} m/s | " +
                        "Sats: {Sats}, Fix: {Fix}",
                        r.SequenceNum, timestamp,
                        r.Latitude, r.Longitude, r.Altitude,
                        r.Yaw, r.Pitch, r.Roll,
                        r.VelNorthMs, r.VelEastMs, r.VelDownMs,
                        r.NumSats, r.GpsFix);
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
        if (_readTask is not null)
        {
            await _readTask.ConfigureAwait(false);
        }
    }
}