using igvc_csharp.Core;
using System.Collections.Concurrent;
using System.Net;
using System.Net.WebSockets;
using System.Net.Sockets;
using igvc_csharp.Events;
using igvc_csharp.MessageUtils;
using Messages;
using Messages.Arc;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Arc;

[Subsystem("ArcSubsystem", Disabled = !Constants.ArcSubsystem.Enabled)]
public class ArcSubsystem : SubsystemBase
{
    private const Endianness Endianness = Constants.ArcSubsystem.Endianness;
    private readonly ConcurrentDictionary<Guid, WebSocket> _clients = new();
    private readonly ConcurrentDictionary<Guid, (uint, uint, uint)> _clientCapabilities = new();
    private CancellationTokenSource? _internalCts;
    private IHost? _host;

    public override async Task Init(CancellationToken token)
    {
        _internalCts = CancellationTokenSource.CreateLinkedTokenSource(token);

        // Wait for port to become available
        await WaitForPortAsync(Constants.ArcSubsystem.Port, token);

        _host = new HostBuilder()
            .ConfigureWebHost(builder =>
            {
                builder.UseKestrel(options =>
                {
                    options.Listen(Constants.ArcSubsystem.Host, Constants.ArcSubsystem.Port);
                    // options.ListenLocalhost(Constants.ArcSubsystem.Port);
                });

                builder.Configure(app =>
                {
                    app.UseWebSockets(new WebSocketOptions()
                    {
                        KeepAliveInterval = TimeSpan.FromSeconds(30)
                    });

                    app.Run(ctx => HandleHttpRequest(ctx, _internalCts.Token));
                });
            })
            .Build();

        // Subscribe to event
        Subscribe<MessageWrapperEvent>(
            OnMessageEvent,
            token
        );

        Logger.LogInformation("ArcSubsystem initialized on port {Port}", Constants.ArcSubsystem.Port);
        await _host.StartAsync(_internalCts.Token);
        SetState(SubsystemState.Operating);
    }

    private async Task OnMessageEvent(MessageWrapperEvent e, CancellationToken token)
    {
        await Broadcast(e.Wrapper, token);
    }

    public async Task Broadcast(MessageWrapper wrapper, CancellationToken token)
    {
        var payload = MessageWriter.Write(
            wrapper.Type,
            wrapper.Data,
            Constants.ArcSubsystem.Endianness
        );
        
        foreach (var (id, socket) in _clients)
        {
            if (socket.State != WebSocketState.Open)
            {
                _clients.TryRemove(id, out _);
                continue;
            }

            // Check if they have the capabilities
            if (!HasWrapperCapability(id, wrapper))
            {
                continue;
            }
            
            await SendToClient(id, payload, token);
        }
    }
    
    private bool HasWrapperCapability(Guid guid, MessageWrapper wrapper)
    {
        return wrapper.Type switch
        {
            MessageType.ImageFrame => HasVisionCapability(guid, wrapper),
            MessageType.Gps => HasCapability(guid, Capabilities.Telemetry.Gps),
            _ => false
        };
    }

    private bool HasVisionCapability(Guid guid, MessageWrapper wrapper)
    {
        var frame = wrapper.As<ImageFrame>();
        return frame.Identifier switch
        {
            "front_view" => HasCapability(guid, Capabilities.Vision.FrontCamera),
            "hsv_view" => HasCapability(guid, Capabilities.Vision.HsvView),
            "yolo_view" => HasCapability(guid, Capabilities.Vision.YoloView),
            _ => false
        };
    }

    private static async Task WaitForPortAsync(int port, CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            TcpListener? listener = null;
            try
            {
                listener = new TcpListener(IPAddress.Loopback, port);
                listener.Start();
                return; // port is available
            }
            catch (SocketException)
            {
                // Port is in use
            }
            finally
            {
                try
                {
                    listener?.Stop();
                }
                catch
                {
                    // ignore
                }
            }

            await Task.Delay(TimeSpan.FromMilliseconds(500), token);
        }
    }

    public override async Task Shutdown()
    {
        SetState(SubsystemState.ShuttingDown);

        if (_internalCts != null)
        {
            await _internalCts.CancelAsync();
        }

        foreach (var (clientId, socket) in _clients)
        {
            try
            {
                await socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "Server Shutdown", CancellationToken.None);
            }
            catch
            {
                // ignore
            }

            socket.Dispose();
        }

        _clients.Clear();

        if (_host != null)
        {
            try
            {
                await _host.StopAsync(TimeSpan.FromSeconds(5));
            }
            catch
            {
                // ignore
            }

            _host.Dispose();
            _host = null;
        }

        _internalCts?.Dispose();
        _internalCts = null;

        SetState(SubsystemState.Shutdown);
    }

    public override async Task Restart()
    {
        await Shutdown();

        EnsureLifetime();
        await Init(LifetimeToken);
    }

    // Websocket Stuff

    public async Task SendToClient(Guid guid, byte[] message, CancellationToken token = default)
    {
        var segment = new ArraySegment<byte>(message);

        var socket = _clients[guid];
        if (socket.State != WebSocketState.Open)
        {
            _clients.TryRemove(guid, out _);
            return;
        }

        try
        {
            await socket.SendAsync(
                segment,
                WebSocketMessageType.Binary,
                endOfMessage: true,
                cancellationToken: token
            );
        }
        catch (Exception ex)
        {
            Logger.LogWarning(ex, "Failed to send message to client {ClientId}", guid);
            _clients.TryRemove(guid, out _);
        }
    }
    
    public async Task BroadcastAsync(byte[] message, CancellationToken token = default)
    {
        foreach (var (id, socket) in _clients)
        {
            if (socket.State != WebSocketState.Open)
            {
                _clients.TryRemove(id, out _);
                continue;
            }

            await SendToClient(id, message, token);
        }
    }

    private async Task HandleHttpRequest(HttpContext ctx, CancellationToken token)
    {
        if (!ctx.WebSockets.IsWebSocketRequest || ctx.Request.Path != Constants.ArcSubsystem.Path)
        {
            Logger.LogTrace("Client tried to connect with no websocket intent, or incorrect path: {Path}",
                ctx.Request.Path);
            ctx.Response.StatusCode = StatusCodes.Status404NotFound;
            return;
        }

        if (_clients.Count >= Constants.ArcSubsystem.MaxConnections)
        {
            Logger.LogWarning("Client tried to connect and was rejected due to max connections being reached");
            ctx.Response.StatusCode = StatusCodes.Status503ServiceUnavailable;
            return;
        }

        var socket = await ctx.WebSockets.AcceptWebSocketAsync();
        var clientId = Guid.NewGuid();
        _clients[clientId] = socket;

        Logger.LogInformation("ARC client connected {ClientId}", clientId);
        await ReceiveLoop(clientId, socket, token);
    }

    private async Task ProcessMessage(Guid guid, MessageWrapper wrapper, CancellationToken token)
    {
        Logger.LogInformation("Processing message {Guid} with type {Type}", guid, wrapper.Type);
        if (wrapper.Type == MessageType.CapabilityReq)
        {
            await HandleCapabilityRequest(guid, wrapper.As<ArcCapability>(), token);
        }

        if (wrapper.Type == MessageType.CommandReq)
        {
            await HandleCommandRequest(guid, wrapper.As<ArcCommand>(), token);
        }
    }

    private bool HasCapability(Guid guid, Capabilities.Vision needs)
    {
        if (!_clientCapabilities.TryGetValue(guid, out var capability)) return false;
        var (visionRaw, _, _) = capability;

        var vision = (Capabilities.Vision)visionRaw;
        return vision.HasFlag(needs);
    }
    
    private bool HasCapability(Guid guid, Capabilities.Telemetry needs)
    {
        if (!_clientCapabilities.TryGetValue(guid, out var capability)) return false;
        var (_, telemetryRaw, _) = capability;

        var telemetry = (Capabilities.Telemetry)telemetryRaw;
        return telemetry.HasFlag(needs);
    }
    
    private bool HasCapability(Guid guid, Capabilities.Misc needs)
    {
        if (!_clientCapabilities.TryGetValue(guid, out var capability)) return false;
        var (_, _, miscRaw) = capability;

        var misc = (Capabilities.Misc)miscRaw;
        return misc.HasFlag(needs);
    }
    
    private async Task HandleCapabilityRequest(Guid guid, ArcCapability request, CancellationToken token)
    {
        _clientCapabilities[guid] = (
            request.VisionCapabilities,
            request.TelemetryCapabilities,
            request.MiscCapabilities
        );

        var response = MessageConstructor.CreateArcCapabilityAck(request);
        var wrappedFrame = MessageWrapper.From(MessageType.CapabilityAck, response.ByteBuffer.ToFullArray());
        var payload = MessageWriter.Write(
            wrappedFrame.Type,
            wrappedFrame.Data,
            Constants.ArcSubsystem.Endianness
        );
        await SendToClient(guid, payload, token);
    }

    private Task HandleCommandRequest(Guid guid, ArcCommand commandRaw, CancellationToken token)
    {
        var command = (Command)commandRaw.CommandId;
        return Task.CompletedTask;
    }

    private async Task ReceiveLoop(Guid clientId, WebSocket socket, CancellationToken token)
    {
        var buffer = new byte[Constants.ArcSubsystem.ReceiveBufferSize];
        var accumulator = new MessageAccumulator(
            Endianness,
            (message) => ProcessMessage(clientId, message, token),
            initialCapacity: Constants.ArcSubsystem.ReceiveBufferSize
        );

        try
        {
            while (!token.IsCancellationRequested && socket.State == WebSocketState.Open)
            {
                var result = await socket.ReceiveAsync(new ArraySegment<byte>(buffer), token);

                if (result.MessageType == WebSocketMessageType.Close)
                {
                    break;
                }

                if (result.Count == 0)
                {
                    continue;
                }

                accumulator.Append(buffer.AsSpan(0, result.Count));
            }
        }
        catch (OperationCanceledException)
        {
            // ignore (expected during shutdown)
        }
        catch (Exception ex)
        {
            Logger.LogWarning(ex, "ARC receive error for client {ClientId}", clientId);
        }
        finally
        {
            _clients.TryRemove(clientId, out _);

            try
            {
                socket.Dispose();
            }
            catch
            {
                // ignore
            }

            Logger.LogInformation("ARC client disconnected {ClientId}", clientId);
        }
    }
}