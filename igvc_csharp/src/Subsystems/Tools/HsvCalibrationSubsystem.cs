using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.MessageUtils;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Utilities;
using Messages;
using Messages.Arc;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Subsystems.Tools;

[Subsystem("HsvCalibrationSubsystem")]
public class HsvCalibrationSubsystem : SubsystemBase
{
    private bool _isCalibrating;
    private ulong _stopCalibrationTimestamp;
    private bool _doCalibrate;
    private ArcCommand? _currentCommand;
    private Rect? _currentRoi;

    private readonly Channel<ImageFrame> _frameChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    public override Task Init(CancellationToken token)
    {
        SubscribeImage(
            "front_view",
            OnImageReceived,
            token
        );

        _ = Task.Factory.StartNew(
            () => ImageProcessingTask(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        return Task.CompletedTask;
    }

    private async Task ImageProcessingTask(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            switch (_isCalibrating)
            {
                case false:
                    await Task.Delay(100, token);
                    continue;
                case true when TimeUtilities.Now() >= _stopCalibrationTimestamp:
                    _isCalibrating = false;
                    _stopCalibrationTimestamp = 0;
                    Logger.LogInformation("OpenCV calibration timed out");
                    continue;
            }
            
            // Get the Hsv Range
            if (_currentRoi == null)
            {
                return;
            }
            
            // Get the latest image
            var didRead = _frameChannel.Reader.TryRead(out var img);
            if (!didRead || _currentRoi == null)
            {
                return;
            }

            using var imageMat = CvUtils.AsMat(img);
            var range = CvUtils.ExtractHsvRange(imageMat, (Rect)_currentRoi);
            var msg = MessageConstructor.CreateHistogram(range);
            var response = MessageConstructor
                .CreateWrappedResponse(_currentCommand, msg.ByteBuffer.ToFullArray())
                .Event();
            EventBus.Instance.Publish(response);
            
            if (_doCalibrate)
            {
                _doCalibrate = false;
                Calibrate();
            }
        }
    }

    private void Calibrate()
    {
        if (_currentCommand == null)
        {
            Logger.LogError("No command to respond to for Hsv calibration");
            return;
        }

        var response = MessageConstructor.CreateResponse(_currentCommand, [1]);
        EventBus.Instance.Publish(
            MessageWrapper
                .From(MessageType.CommandAck, response.ByteBuffer.ToFullArray())
                .Event()
        );
    }

    private Task OnImageReceived(ImageFrame frame, CancellationToken token)
    {
        _frameChannel.Writer.TryWrite(frame);
        return Task.CompletedTask;
    }

    private Rect? ExtractHsvRect()
    {
        if (_currentCommand == null)
        {
            return null;
        }
        
        // Get the rect from the command
        using var stream = new MemoryStream(_currentCommand.Value.GetDataArray());
        using var reader = new BinaryReader(stream);
        
        // Get x, y, w, and h
        var x = reader.ReadInt32();
        var y = reader.ReadInt32();
        var w = reader.ReadInt32();
        var h = reader.ReadInt32();
        return new Rect(x, y, w, h);
    }

    [ArcCommand(ArcCommandId.ToolsStartHsvCalibration)]
    public void StartCalibration(ArcCommand command)
    {
        _isCalibrating = true;
        _stopCalibrationTimestamp = TimeUtilities.Now() + Constants.CalibrationSubsystem.OpenCvCalibrationTimeoutMs;
        _doCalibrate = false;
        Logger.LogInformation("Started HSV calibration");
    }

    [ArcCommand(ArcCommandId.ToolsStopHsvCalibration)]
    public void StopCalibration()
    {
        _currentCommand = null;
        _isCalibrating = false;
        _stopCalibrationTimestamp = 0;
        _doCalibrate = false;
        Logger.LogInformation("Stopped HSV calibration");
    }

    [ArcCommand(ArcCommandId.ToolsEditHsvCalibration)]
    public void EditCalibration(ArcCommand command)
    {
        if (_doCalibrate)
        {
            return;
        }

        _currentCommand = command;
        _currentRoi = ExtractHsvRect();
    }

    [ArcCommand(ArcCommandId.ToolsSaveHsvCalibration)]
    public void SaveCalibration(ArcCommand command)
    {
        _doCalibrate = true;
        Logger.LogInformation("Saving HSV calibration");
    }
}