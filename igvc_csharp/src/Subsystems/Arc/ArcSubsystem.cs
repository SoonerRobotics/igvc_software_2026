using igvc_csharp.Core;
using System.Collections.Concurrent;
using System.Net;
using System.Net.WebSockets;
using System.Net.Sockets;
using System.Reflection;
using System.Threading.Channels;
using Google.FlatBuffers;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Arc.Streaming;
using igvc_csharp.Utils.Messages;
using Messages;
using Messages.Arc;
using Messages.Performance;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Arc;

[Subsystem("ArcSubsystem", Disabled = !Configuration.ArcSubsystem.Enabled)]
public class ArcSubsystem : SubsystemBase
{
    private const Endianness Endianness = Configuration.ArcSubsystem.Endianness;
    private readonly ConcurrentDictionary<Guid, WebSocket> _clients = new();
    private CancellationTokenSource? _internalCts;
    private IHost? _host;
    private JpegServer? _server;

    private readonly Channel<MessageWrapper> _outgoingData = Channel.CreateBounded<MessageWrapper>(
        new BoundedChannelOptions(64)
        {
            SingleReader = true,
            SingleWriter = false,
            FullMode = BoundedChannelFullMode.DropOldest
        });

    private readonly Dictionary<ArcCommandId, List<MethodInfo>> _commandMethods = new();
    

    private Task? _outgoingSendTask;

    public override async Task Init(CancellationToken token)
    {
        _internalCts = CancellationTokenSource.CreateLinkedTokenSource(token);

        // Wait for port to become available
        await WaitForPortAsync(Configuration.ArcSubsystem.Port, token);

        // Load all ArcCommandAttribute users
        var allMethods = AppDomain.CurrentDomain.GetAssemblies()
            .SelectMany(x => x.GetTypes())
            .Where(x => x.IsClass)
            .SelectMany(x => x.GetMethods());
        foreach (var method in allMethods)
        {
            var attrib = method.GetCustomAttribute<ArcCommandAttribute>();
            if (attrib == null)
            {
                continue;
            }
            
            if (!_commandMethods.TryGetValue(attrib.Command, out var value))
            {
                value = [];
                _commandMethods[attrib.Command] = value;
            }

            value.Add(method);
        }
        
        _host = new HostBuilder()
            .ConfigureWebHost(builder =>
            {
                builder.UseKestrel(options =>
                {
                    options.Listen(Configuration.ArcSubsystem.Host, Configuration.ArcSubsystem.Port);
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

        _outgoingSendTask = Task.Run(() => ArcSendLoop(_internalCts.Token), _internalCts.Token);

        _server = new JpegServer($"http://localhost:{Configuration.ArcSubsystem.Port + 1}/");
        _ = _server.StartAsync(token);

        // Subscribe to MessageWrapperEvent
        Subscribe<MessageWrapperEvent>(
            OnMessageEvent,
            token
        );

        Logger.LogInformation("ArcSubsystem initialized on port {Port}", Configuration.ArcSubsystem.Port);
        await _host.StartAsync(_internalCts.Token);
        SetOperatingState(SubsystemState.Operating);
    }

    private async Task ArcSendLoop(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                var wrapper = await _outgoingData.Reader.ReadAsync(token);

                if (wrapper.Type == MessageType.ImageFrame)
                {
                    var frame = wrapper.As<ImageFrame>();
                    var bytes = frame.GetImageDataArray();
                    if (bytes != null)
                        JpegStreamRegistry.Publish(frame.Identifier, bytes);

                    continue;
                }

                Broadcast(wrapper, token);
            }
        }
        catch (OperationCanceledException)
        {
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Arc send loop crashed");
        }
    }

    private Task OnMessageEvent(MessageWrapperEvent e, CancellationToken token)
    {
        _outgoingData.Writer.TryWrite(e.Wrapper);
        return Task.CompletedTask;
    }

    public void Broadcast(MessageWrapper wrapper, CancellationToken token)
    {
        var payload = MessageWriter.Write(
            wrapper.Type,
            wrapper.Data,
            Configuration.ArcSubsystem.Endianness
        );

        foreach (var (id, socket) in _clients)
        {
            if (socket.State != WebSocketState.Open)
            {
                _clients.TryRemove(id, out _);
                continue;
            }

            _ = SendToClient(id, payload, token);
        }
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
        SetOperatingState(SubsystemState.ShuttingDown);

        if (_internalCts != null)
        {
            await _internalCts.CancelAsync();
        }

        foreach (var (_, socket) in _clients)
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

        SetOperatingState(SubsystemState.Shutdown);
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
        if (!_clients.TryGetValue(guid, out var socket))
        {
            return;
        }

        if (socket.State != WebSocketState.Open)
        {
            _clients.TryRemove(guid, out _);
            return;
        }

        try
        {
            await socket.SendAsync(
                message,
                WebSocketMessageType.Binary,
                true,
                token
            );
        }
        catch
        {
            _clients.TryRemove(guid, out _);
            try
            {
                socket.Dispose();
            }
            catch
            {
                // ignored
            }
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
        if (!ctx.WebSockets.IsWebSocketRequest || ctx.Request.Path != Configuration.ArcSubsystem.Path)
        {
            Logger.LogTrace("Client tried to connect with no websocket intent, or incorrect path: {Path}",
                ctx.Request.Path);
            ctx.Response.StatusCode = StatusCodes.Status404NotFound;
            return;
        }

        if (_clients.Count >= 32)
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
        if (wrapper.Type == MessageType.CommandReq)
        {
            await HandleCommandRequest(guid, wrapper.As<ArcCommand>(), token);
        }
    }

    private Task HandleCommandRequest(Guid guid, ArcCommand commandRaw, CancellationToken token)
    {
        var command = commandRaw.CommandId;
        
        if (_commandMethods.TryGetValue(command, out var methods))
        {
            foreach (var method in methods)
            {
                var clazz = method.DeclaringType;
                if (clazz == null)
                {
                    continue;
                }

                var instance = Robot.Instance.GetSubsystem(clazz);
                var parameters = method.GetParameters();
                switch (parameters.Length)
                {
                    case 0:
                        method.Invoke(instance, null);
                        break;
                    case 1:
                        method.Invoke(instance, [commandRaw]);
                        break;
                    default:
                        Logger.LogWarning("Method {Method} has invalid number of parameters for ArcCommand", method.Name);
                        break;
                }
            }
        }
        else
        {
            Logger.LogWarning("No handlers found for command {Command}", command);
        }
        
        return Task.CompletedTask;
    }

    private async Task ReceiveLoop(Guid clientId, WebSocket socket, CancellationToken token)
    {
        var buffer = new byte[Configuration.ArcSubsystem.ReceiveBufferSize];
        var accumulator = new MessageAccumulator(
            Endianness,
            (message) => ProcessMessage(clientId, message, token),
            initialCapacity: Configuration.ArcSubsystem.ReceiveBufferSize
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