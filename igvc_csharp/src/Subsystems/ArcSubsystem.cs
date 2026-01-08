using System.Collections.Concurrent;
using System.Net;
using System.Net.WebSockets;
using igvc_csharp.Core;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems;

[Subsystem("ArcSubsystem")]
public class ArcSubsystem : ISubsystem
{
    private static readonly ILogger Logger = Logging.From<ArcSubsystem>();

    private readonly ConcurrentDictionary<Guid, WebSocket> _clients = new ();

    private HttpListener? _listener;
    private CancellationTokenSource? _internalCts;
    private Task? _acceptLoopTask;
    
    public Task Init(CancellationToken token)
    {
        _internalCts = CancellationTokenSource.CreateLinkedTokenSource(token);
        
        _listener = new HttpListener();
        _listener.Prefixes.Add($"http://+:{Constants.ArcSubsystem.Port}{Constants.ArcSubsystem.Path}/");
        _listener.Start();

        _acceptLoopTask = Task.Run(() => AcceptLoop(_internalCts.Token), _internalCts.Token);
    }
    
    public Task Periodic(CancellationToken token)
    {
        return Task.CompletedTask;
    }

    public Task Shutdown()
    {
        return Task.CompletedTask;
    }

    public Task Restart()
    {
        return Task.CompletedTask;
    }
    
    // Websocket Stuff

    private async Task AcceptLoop(CancellationToken token)
    {
        
    }
}