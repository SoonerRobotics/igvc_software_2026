using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using Microsoft.Extensions.Logging;
using sl;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("ZedSubsystem")]
public class ZedSubsystem : SubsystemBase
{
    // Implementation

    private Camera? mCamera;

    public override Task Init(CancellationToken token)
    {        
        _ = StartCamera(token);

        return Task.CompletedTask;
    }

    private async Task StartCamera(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            try
            {
                await ConnectAsync(token);
                await RunAsync(token);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                Logger.LogError("Zed Connection Lost: {reason}", ex.Message);
                mCamera?.Close();
                mCamera = null;
            }

            await Task.Delay(2000, token).ConfigureAwait(false);
        }

        mCamera?.Close();
    }

    private async Task ConnectAsync(CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        
        mCamera = new Camera(0);
        var initParameters = new InitParameters
        {
            resolution = RESOLUTION.HD1080,
            cameraFPS = 12
        };
        var error = mCamera.Open(ref initParameters);
        if (error != ERROR_CODE.SUCCESS)
        {
            throw new InvalidOperationException($"Failed to open ZED camera: {error}");
        }
    }

    private async Task RunAsync(CancellationToken token)
    {
        // Print serial stuff
        Logger.LogInformation("Connected to ZED Camera: {serial} - {model}", mCamera?.GetZEDSerialNumber(), mCamera?.GetCameraModel());

        // while (!token.IsCancellationRequested)
        // {
        //     await Task.Yield();
        // }
    }
}