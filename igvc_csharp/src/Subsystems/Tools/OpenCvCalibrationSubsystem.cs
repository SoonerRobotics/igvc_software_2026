using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Messages.Arc;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Subsystems.Tools;

[Subsystem("OpenCvCalibrationSubsystem")]
public class OpenCvCalibrationSubsystem : SubsystemBase
{
    private bool _isCalibrating;
    private ulong _stopCalibrationTimestamp;
    private bool _doCalibrate;
    private ArcCommand? _currentCommand;

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
                case true when TimeUtils.Now() >= _stopCalibrationTimestamp:
                    _isCalibrating = false;
                    _stopCalibrationTimestamp = 0;
                    Logger.LogInformation("OpenCV calibration timed out");
                    continue;
            }

            var frame = await _frameChannel.Reader.ReadAsync(token);
            using var mat = CvUtils.AsMat(frame);

            // Find chessboard corners
            var patternSize = new Size(
                Configuration.CalibrationSubsystem.OpenCvCalibrationPatternWidth,
                Configuration.CalibrationSubsystem.OpenCvCalibrationPatternHeight
            );
            var found = Cv2.FindChessboardCornersSB(
                mat,
                patternSize,
                out var corners
            );
            if (!found)
            {
                var frameBytes = CvUtils.FromMat(mat);
                var wrapper = CvUtils.BuildWrapper(frame.Width, frame.Height, "calibration_chessboard", frameBytes);
                EventBus.Instance.Publish(wrapper.Event());
                continue;
            }

            PublishChessboardImage(CvUtils.CloneMat(mat), corners);

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
            Logger.LogError("No command to respond to for OpenCV calibration");
            return;
        }

        var response = MessageConstructor.CreateResponse(_currentCommand, [1]);
        EventBus.Instance.Publish(
            MessageWrapper
                .From(MessageType.CommandAck, response.ByteBuffer.ToFullArray())
                .Event()
        );
    }

    private static void PublishChessboardImage(Mat mat, Point2f[] corners)
    {
        Cv2.DrawChessboardCorners(
            mat,
            new Size(
                Configuration.CalibrationSubsystem.OpenCvCalibrationPatternWidth,
                Configuration.CalibrationSubsystem.OpenCvCalibrationPatternHeight
            ),
            corners,
            true
        );

        var frameBytes = CvUtils.FromMat(mat);
        var wrapper = CvUtils.BuildWrapper((uint)mat.Width, (uint)mat.Height, "calibration_chessboard", frameBytes);
        EventBus.Instance.Publish(wrapper.Event());
    }

    private Task OnImageReceived(ImageFrame frame, CancellationToken token)
    {
        _frameChannel.Writer.TryWrite(frame);
        return Task.CompletedTask;
    }

    [ArcCommand(ArcCommandId.ToolsStartOpenCvCalibration)]
    public void StartCalibration(ArcCommand command)
    {
        _isCalibrating = true;
        _stopCalibrationTimestamp = TimeUtils.Now() + Configuration.CalibrationSubsystem.OpenCvCalibrationTimeoutMs;
        _doCalibrate = false;
        Logger.LogInformation("Started OpenCV calibration");
    }

    [ArcCommand(ArcCommandId.ToolsStopOpenCvCalibration)]
    public void StopCalibration()
    {
        _currentCommand = null;
        _isCalibrating = false;
        _stopCalibrationTimestamp = 0;
        _doCalibrate = false;
        Logger.LogInformation("Stopped OpenCV calibration");
    }

    [ArcCommand(ArcCommandId.ToolsSaveOpenCvCalibration)]
    public void SaveCalibration(ArcCommand command)
    {
        _doCalibrate = true;
        _currentCommand = command;
        Logger.LogInformation("Saving OpenCV calibration");
    }
}