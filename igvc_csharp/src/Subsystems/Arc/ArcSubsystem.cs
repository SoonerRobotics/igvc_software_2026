using System.Buffers.Binary;
using igvc_csharp.Core;
using System.Collections.Concurrent;
using System.Net;
using System.Net.WebSockets;
using System.Net.Sockets;
using igvc_csharp.Messages;
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
    private readonly ConcurrentDictionary<Guid, List<Capability>> _clientCapabilities = new();
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
                    // options.Listen(Constants.ArcSubsystem.Host, Constants.ArcSubsystem.Port);
                    options.ListenLocalhost(Constants.ArcSubsystem.Port);
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

        Logger.LogInformation("ArcSubsystem initialized on port {Port}", Constants.ArcSubsystem.Port);
        await _host.StartAsync(_internalCts.Token);
        SetState(SubsystemState.Operating);
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

    public async Task BroadcastAsync(byte[] message, CancellationToken token = default)
    {
        var segment = new ArraySegment<byte>(message);

        foreach (var (id, socket) in _clients)
        {
            if (socket.State != WebSocketState.Open)
            {
                _clients.TryRemove(id, out _);
                continue;
            }

            try
            {
                await socket.SendAsync(
                    segment,
                    WebSocketMessageType.Binary,
                    endOfMessage: true,
                    cancellationToken: token);
            }
            catch (Exception ex)
            {
                Logger.LogWarning(ex, "Failed to send message to client {ClientId}", id);
                _clients.TryRemove(id, out _);
            }
        }
    }

    private async Task HandleHttpRequest(HttpContext ctx, CancellationToken token)
    {
        if (!ctx.WebSockets.IsWebSocketRequest || ctx.Request.Path != Constants.ArcSubsystem.Path)
        {
            ctx.Response.StatusCode = StatusCodes.Status404NotFound;
            return;
        }

        if (_clients.Count >= Constants.ArcSubsystem.MaxConnections)
        {
            ctx.Response.StatusCode = StatusCodes.Status503ServiceUnavailable;
            return;
        }

        var socket = await ctx.WebSockets.AcceptWebSocketAsync();
        var clientId = Guid.NewGuid();
        _clients[clientId] = socket;

        Logger.LogInformation("ARC client connected {ClientId}", clientId);
        await ReceiveLoop(clientId, socket, token);
    }

    private void ProcessMessage(Guid guid, MessageWrapper wrapper)
    {
    }

    private async Task ReceiveLoop(Guid clientId, WebSocket socket, CancellationToken token)
    {
        var buffer = new byte[Constants.ArcSubsystem.ReceiveBufferSize];
        var accumulator = new MessageAccumulator(
            Endianness,
            (message) => ProcessMessage(clientId, message),
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