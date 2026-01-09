using igvc_csharp.Core;
using igvc_csharp.Messages;
using igvc_csharp.Subsystems;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Utilities;
using Messages;
using Microsoft.Extensions.Logging;

[Subsystem("TestSubsystem", DependsOn = [typeof(SimulatorSubsystem)])]
public class TestSubsystem(SimulatorSubsystem arc) : SubsystemBase
{
    private OpenCvImageWindow? _window;
    
    public override Task Init(CancellationToken token)
    {
        _window = new OpenCvImageWindow("Simulator Camera");
        
        SubscribeMessage<ImageFrame>(
            MessageType.ImageFrame,
            OnImageReceived,
            token
        );
        
        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        _window?.Dispose();
        _window = null;
        return Task.CompletedTask;
    }

    private Task OnImageReceived(ImageFrame frame, CancellationToken token)
    {
        _window?.ShowJpeg(frame.GetImageDataArray());
        return Task.CompletedTask;
    }
}