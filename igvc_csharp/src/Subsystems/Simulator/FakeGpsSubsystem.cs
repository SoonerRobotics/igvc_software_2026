using Google.FlatBuffers;
using igvc_csharp.Core;
using igvc_csharp.Core.Units;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using FakeGpsConfig = igvc_csharp.Configuration.FakeGpsSubsystem;

namespace igvc_csharp.src.Subsystems.Simulator;

[Subsystem("FakeGpsSubsystem", Disabled = false)]
public class FakeGpsSubsystem : SubsystemBase
{
    private List<(double, LatLng)> _data = [];

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        ReadGpsLog();

        _ = Task.Factory.StartNew(
            () => GpsPublishingTask(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    // this code is stolen from WaypointSubsystem.ReadWaypointsFile()
    private void ReadGpsLog()
    {
        string? line;

        using (StreamReader gpsFile = new(FileUtils.GetFileRelativeToRoot(FakeGpsConfig.Filename)))
        {
            if (gpsFile == null)
            {
                SetOperatingState(SubsystemState.Errored);

                Logger.LogError("Failed to fake GPS data!");

                return;
            }

            //TODO do we need to use cancellation token here?
            while ((line = gpsFile.ReadLine()) != null)
            {
                var tokens = line.Split(",");

                double timestamp = double.Parse(tokens[0]);
                LatLng point = new(
                    double.Parse(tokens[2]),
                    double.Parse(tokens[3])
                );

                _data.Add((timestamp, point));
            }
        }

        Logger.LogInformation("Lines of GPS data read: {}", _data.Count);

        return;
    }

    private async Task GpsPublishingTask(CancellationToken token)
    {
        int line = 0;
        double lastTimestamp = 0;

        try
        {
            while (!token.IsCancellationRequested && line < _data.Count)
            {
                SetOperatingState(SubsystemState.Operating);

                (var timestamp, var pos) = _data[line];

                if (lastTimestamp == 0)
                {
                    lastTimestamp = timestamp;
                }

                //TODO publish stuff for the other parts of the message?
                var builder = new FlatBufferBuilder(128);
                var reportOffset = VectornavReport.CreateVectornavReport(
                    builder,
                    (ulong)timestamp,
                    (uint)line,
                    pos.Latitude,
                    pos.Longitude,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0
                );
                builder.Finish(reportOffset.Value);

                var reportMessage = MessageWrapper.From(MessageType.VectorNav, builder.SizedByteArray());

                EventBus.Instance.Publish(new MessageWrapperEvent(reportMessage));

                // Logger.LogDebug("publishing line: " + line);
                line++;

                var deltaT = (timestamp - lastTimestamp) * 1000; // convert to milliseconds
                lastTimestamp = timestamp;

                await Task.Delay((int)(deltaT), token);
            }
        }
        catch (OperationCanceledException)
        {
            // Expected on shutdown
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "GPS faker publishing task crashed");
            SetOperatingState(SubsystemState.Errored);
        }

        await Shutdown();
    }

    public override Task Shutdown()
    {
        SetOperatingState(SubsystemState.Shutdown);

        return Task.CompletedTask;
    }
}