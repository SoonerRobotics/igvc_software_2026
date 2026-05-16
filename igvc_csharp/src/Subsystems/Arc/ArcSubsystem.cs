using System;
using System.Buffers;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Net.WebSockets;
using System.Reflection;
using System.Threading;
using System.Threading.Channels;
using System.Threading.Tasks;
using Google.FlatBuffers;
using igvc_csharp.Core;
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
            FullMode     = BoundedChannelFullMode.DropOldest
        });

    private readonly Dictionary<ArcCommandId, List<MethodInfo>> _commandMethods = new();
    private Task? _outgoingSendTask;

    public override async Task Init(CancellationToken token)
    {
        _internalCts = CancellationTokenSource.CreateLinkedTokenSource(token);

        await WaitForPortAsync(Configuration.ArcSubsystem.Port, token);

        var allMethods = AppDomain.CurrentDomain.GetAssemblies()
            .SelectMany(x => x.GetTypes())
            .Where(x => x.IsClass)
            .SelectMany(x => x.GetMethods());

        foreach (var method in allMethods)
        {
            var attrib = method.GetCustomAttribute<ArcCommandAttribute>();
            if (attrib == null) continue;

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
                    options.Listen(Configuration.ArcSubsystem.Host, Configuration.ArcSubsystem.Port));

                builder.Configure(app =>
                {
                    app.UseWebSockets(new WebSocketOptions { KeepAliveInterval = TimeSpan.FromSeconds(30) });
                    app.Run(ctx => HandleHttpRequest(ctx, _internalCts.Token));
                });
            })
            .Build();

        _outgoingSendTask = Task.Run(() => ArcSendLoop(_internalCts.Token), _internalCts.Token);

        _server = new JpegServer("http://localhost:8001/");
        _ = _server.StartAsync(token);

        Subscribe<MessageWrapperEvent>(OnMessageEvent, token);

        Logger.LogInformation("ArcSubsystem initialized on port {Port}", Configuration.ArcSubsystem.Port);
        await _host.StartAsync(_internalCts.Token);
        SetOperatingState(SubsystemState.Operating);
    }

    private async Task ArcSendLoop(CancellationToken token)
    {
        try
        {
            await foreach (var wrapper in _outgoingData.Reader.ReadAllAsync(token))
            {
                using (wrapper) // disposes pooled wrapper when done
                {
                    if (wrapper.Type == MessageType.ImageFrame)
                    {
                        var frame = wrapper.As<ImageFrame>();
                        var bytes = frame.GetImageDataArray();
                        if (bytes != null)
                            JpegStreamRegistry.Publish(frame.Identifier, bytes);
                        continue;
                    }

                    await BroadcastAsync(wrapper, token);
                }
            }
        }
        catch (OperationCanceledException) { }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Arc send loop crashed");
        }
    }

    private Task OnMessageEvent(MessageWrapperEvent e, CancellationToken token)
    {
        // If the wrapper is pooled we need an owned copy for the channel,
        // since the original may be disposed before ArcSendLoop reads it
        var wrapper = e.Wrapper;
        MessageWrapper queued;
        if (wrapper.Data != null)
        {
            var ownedData = new byte[wrapper.Length];
            wrapper.Data.AsSpan(0, wrapper.Length).CopyTo(ownedData);
            queued = MessageWrapper.From(wrapper.Type, ownedData);
        }
        else
        {
            queued = wrapper;
        }

        _outgoingData.Writer.TryWrite(queued);
        return Task.CompletedTask;
    }

    /// <summary>
    /// Encodes the wrapper once into a rented buffer and sends it to all connected clients.
    /// </summary>
    public async Task BroadcastAsync(MessageWrapper wrapper, CancellationToken token)
    {
        // ImageFrame skips CRC — bulk data, TCP/WS is reliable
        var includeCrc = wrapper.Type != MessageType.ImageFrame;
        var (buffer, length) = MessageWriter.WritePooled(
            wrapper.Type,
            wrapper.Data.AsSpan(0, wrapper.Length),
            Endianness,
            includeCrc
        );

        try
        {
            var segment = new ArraySegment<byte>(buffer, 0, length);
            foreach (var (id, socket) in _clients)
            {
                if (socket.State != WebSocketState.Open)
                {
                    _clients.TryRemove(id, out _);
                    continue;
                }

                await SendToClient(id, socket, segment, token);
            }
        }
        finally
        {
            ArrayPool<byte>.Shared.Return(buffer);
        }
    }

    private async Task SendToClient(Guid guid, WebSocket socket, ArraySegment<byte> message, CancellationToken token)
    {
        if (socket.State != WebSocketState.Open)
        {
            _clients.TryRemove(guid, out _);
            return;
        }

        try
        {
            await socket.SendAsync(message, WebSocketMessageType.Binary, true, token);
        }
        catch
        {
            _clients.TryRemove(guid, out _);
            try { socket.Dispose(); } catch { /* ignore */ }
        }
    }

    // Keep old signature for any external callers
    public Task SendToClient(Guid guid, byte[] message, CancellationToken token = default)
    {
        if (!_clients.TryGetValue(guid, out var socket)) return Task.CompletedTask;
        return SendToClient(guid, socket, new ArraySegment<byte>(message), token);
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
                return;
            }
            catch (SocketException) { }
            finally
            {
                try { listener?.Stop(); } catch { /* ignore */ }
            }

            await Task.Delay(TimeSpan.FromMilliseconds(500), token);
        }
    }

    public override async Task Shutdown()
    {
        SetOperatingState(SubsystemState.ShuttingDown);

        if (_internalCts != null)
            await _internalCts.CancelAsync();

        foreach (var (_, socket) in _clients)
        {
            try { await socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "Server Shutdown", CancellationToken.None); }
            catch { /* ignore */ }
            socket.Dispose();
        }
        _clients.Clear();

        if (_host != null)
        {
            try   { await _host.StopAsync(TimeSpan.FromSeconds(5)); }
            catch { /* ignore */ }
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

    // WebSocket receive

    private async Task HandleHttpRequest(HttpContext ctx, CancellationToken token)
    {
        if (!ctx.WebSockets.IsWebSocketRequest || ctx.Request.Path != Configuration.ArcSubsystem.Path)
        {
            Logger.LogTrace("Non-WS or wrong path: {Path}", ctx.Request.Path);
            ctx.Response.StatusCode = StatusCodes.Status404NotFound;
            return;
        }

        if (_clients.Count >= 32)
        {
            Logger.LogWarning("Max connections reached, rejecting client");
            ctx.Response.StatusCode = StatusCodes.Status503ServiceUnavailable;
            return;
        }

        var socket   = await ctx.WebSockets.AcceptWebSocketAsync();
        var clientId = Guid.NewGuid();
        _clients[clientId] = socket;

        Logger.LogInformation("ARC client connected {ClientId}", clientId);
        await ReceiveLoop(clientId, socket, token);
    }

    private async Task ReceiveLoop(Guid clientId, WebSocket socket, CancellationToken token)
    {
        var readBuffer  = ArrayPool<byte>.Shared.Rent(Configuration.ArcSubsystem.ReceiveBufferSize);
        var accumulator = new MessageAccumulator(
            Endianness,
            wrapper =>
            {
                // Fire-and-forget command handling; dispose wrapper when done
                _ = ProcessMessage(clientId, wrapper, token)
                    .ContinueWith(_ => wrapper.Dispose(), TaskContinuationOptions.ExecuteSynchronously);
            },
            initialCapacity: Configuration.ArcSubsystem.ReceiveBufferSize
        );

        try
        {
            while (!token.IsCancellationRequested && socket.State == WebSocketState.Open)
            {
                var result = await socket.ReceiveAsync(new ArraySegment<byte>(readBuffer), token);

                if (result.MessageType == WebSocketMessageType.Close) break;
                if (result.Count == 0) continue;

                accumulator.Append(readBuffer.AsSpan(0, result.Count));
            }
        }
        catch (OperationCanceledException) { }
        catch (Exception ex)
        {
            Logger.LogWarning(ex, "ARC receive error for client {ClientId}", clientId);
        }
        finally
        {
            ArrayPool<byte>.Shared.Return(readBuffer);
            accumulator.Dispose();
            _clients.TryRemove(clientId, out _);
            try { socket.Dispose(); } catch { /* ignore */ }
            Logger.LogInformation("ARC client disconnected {ClientId}", clientId);
        }
    }

    private async Task ProcessMessage(Guid guid, MessageWrapper wrapper, CancellationToken token)
    {
        Logger.LogInformation("Processing message {Guid} with type {Type}", guid, wrapper.Type);
        if (wrapper.Type == MessageType.CommandReq)
            await HandleCommandRequest(guid, wrapper.As<ArcCommand>(), token);
    }

    private Task HandleCommandRequest(Guid guid, ArcCommand commandRaw, CancellationToken token)
    {
        var command = commandRaw.CommandId;

        if (_commandMethods.TryGetValue(command, out var methods))
        {
            foreach (var method in methods)
            {
                var clazz = method.DeclaringType;
                if (clazz == null) continue;

                var instance   = IgvcRobot.Instance.GetSubsystem(clazz);
                var parameters = method.GetParameters();
                switch (parameters.Length)
                {
                    case 0: method.Invoke(instance, null); break;
                    case 1: method.Invoke(instance, [commandRaw]); break;
                    default:
                        Logger.LogWarning("Method {Method} has invalid parameter count for ArcCommand", method.Name);
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
}