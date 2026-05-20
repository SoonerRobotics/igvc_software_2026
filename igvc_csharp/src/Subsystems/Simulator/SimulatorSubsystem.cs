using System;
using System.Buffers;
using System.IO;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Channels;
using System.Threading.Tasks;
using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Utils.Messages;
using Messages;
using Messages.Arc;
using Microsoft.Extensions.Logging;
using SocketCANSharp;

namespace igvc_csharp.Subsystems.Simulator;

[Subsystem("SimulatorSubsystem", Disabled = !Configuration.UseSimulation)]
public class SimulatorSubsystem : SubsystemBase
{
    private const Endianness Endianness = Configuration.SimulatorSubsystem.Endianness;

    private TcpClient? _client;
    private Task? _connectTask;
    private CancellationTokenSource? _internalCts;
    private NetworkStream? _stream;

    private readonly Channel<(byte[] buffer, int length)> _sendChannel =
        Channel.CreateBounded<(byte[], int)>(new BoundedChannelOptions(256)
        {
            FullMode = BoundedChannelFullMode.DropOldest,
            SingleReader = true,
            SingleWriter = false
        });

    public override Task Init(CancellationToken token)
    {
        _internalCts = CancellationTokenSource.CreateLinkedTokenSource(token);

        _connectTask = Task.Run(
            () => ConnectionLoop(_internalCts.Token),
            _internalCts.Token
        );

        SetOperatingState(SubsystemState.Initialized);
        return Task.CompletedTask;
    }

    public override async Task Shutdown()
    {
        SetOperatingState(SubsystemState.ShuttingDown);

        if (_internalCts != null)
            await _internalCts.CancelAsync();

        if (_connectTask != null)
        {
            try { await _connectTask; }
            catch { /* ignore */ }
        }

        DrainSendChannel();

        _internalCts?.Dispose();
        _internalCts = null;

        SetOperatingState(SubsystemState.Shutdown);
    }

    public override async Task Restart()
    {
        await Shutdown();
        await Init(LifetimeToken);
    }

    // TCP connection loop

    private async Task ConnectionLoop(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            TcpClient? client = null;
            try
            {
                SetOperatingState(SubsystemState.Idle);
                client = new TcpClient();
                _client = client;

                await client.ConnectAsync(
                    Configuration.SimulatorSubsystem.Host,
                    Configuration.SimulatorSubsystem.Port,
                    token
                );

                SetOperatingState(SubsystemState.Operating);

                _stream = client.GetStream();
                var readTask = ReceiveLoop(client, token);
                var writeTask = WriteLoop(_stream, token);
                await Task.WhenAll(readTask, writeTask);
            }
            catch (OperationCanceledException) { break; }
            catch (IOException ex) when (ex.InnerException is SocketException { SocketErrorCode: SocketError.OperationAborted })
            {
                // Socket cancelled cleanly
            }
            catch (Exception ex)
            {
                // Maybe log but it is really obnoxious to have a million log lines about the connection being lost
            }
            finally
            {
                _stream = null;
                try { client?.Dispose(); } catch { /* ignore */ }
                _client = null;

                DrainSendChannel();
            }

            try { await Task.Delay(Configuration.SimulatorSubsystem.ReconnectDelay, token); }
            catch (OperationCanceledException) { break; }
        }
    }

    private async Task WriteLoop(NetworkStream stream, CancellationToken token)
    {
        try
        {
            await foreach (var (buffer, length) in _sendChannel.Reader.ReadAllAsync(token))
            {
                try
                {
                    await stream.WriteAsync(buffer.AsMemory(0, length), token);
                }
                finally
                {
                    ArrayPool<byte>.Shared.Return(buffer);
                }
            }
        }
        catch (OperationCanceledException) { }
        catch (Exception ex)
        {
            Logger.LogWarning(ex, "Simulator write loop error");
        }
    }

    private async Task ReceiveLoop(TcpClient client, CancellationToken token)
    {
        var readBuffer = ArrayPool<byte>.Shared.Rent(Configuration.SimulatorSubsystem.ReceiveBufferSize);
        var accumulator = new MessageAccumulator(
            Endianness,
            OnMessageReceived,
            initialCapacity: Configuration.SimulatorSubsystem.ReceiveBufferSize
        );

        try
        {
            var stream = client.GetStream();
            while (!token.IsCancellationRequested && client.Connected)
            {
                var bytesRead = await stream.ReadAsync(readBuffer, token);
                if (bytesRead == 0) break;
                accumulator.Append(readBuffer.AsSpan(0, bytesRead));
            }
        }
        catch (OperationCanceledException) { }
        catch (IOException ex) when (ex.InnerException is SocketException { SocketErrorCode: SocketError.OperationAborted })
        {
            throw;
        }
        finally
        {
            ArrayPool<byte>.Shared.Return(readBuffer);
            accumulator.Dispose();
        }
    }

    // Message handling

    private void OnMessageReceived(MessageWrapper wrapper)
    {
        try { ProcessMessage(wrapper); }
        finally { wrapper.Dispose(); }
    }

    private void ProcessMessage(MessageWrapper wrapper)
    {
        if (wrapper.Type == MessageType.CanFrame)
        {
            var arcFrame = wrapper.As<CanMessage>();
            var canId = arcFrame.CanId;
            var canData = arcFrame.GetCanDataArray();
            var canFrame = new CanFrame(canId, canData);
            EventBus.Instance.Publish(new CanFrameEvent(canFrame));
            return;
        }

        var ownedData = new byte[wrapper.Length];
        wrapper.Data!.AsSpan(0, wrapper.Length).CopyTo(ownedData);
        EventBus.Instance.Publish(new MessageWrapperEvent(MessageWrapper.From(wrapper.Type, ownedData)));
    }

    // Sending

    public async Task SendCanFrame(CanFrame frame)
    {
        var arcFrame = MessageConstructor.CreateCanMessage(frame);
        var wrapper = MessageWrapper.From(MessageType.CanFrame, arcFrame.ByteBuffer.ToFullArray());
        await SendWrapper(wrapper);
    }

    private ValueTask SendWrapper(MessageWrapper wrapper)
    {
        if (_client is not { Connected: true })
            return ValueTask.CompletedTask;

        var includeCrc = wrapper.Type != MessageType.ImageFrame;
        var (buffer, length) = MessageWriter.WritePooled(
            wrapper.Type,
            wrapper.Data.AsSpan(0, wrapper.Length),
            Endianness,
            includeCrc
        );

        if (!_sendChannel.Writer.TryWrite((buffer, length)))
        {
            ArrayPool<byte>.Shared.Return(buffer);
        }

        return ValueTask.CompletedTask;
    }

    private void DrainSendChannel()
    {
        while (_sendChannel.Reader.TryRead(out var entry))
            ArrayPool<byte>.Shared.Return(entry.buffer);
    }
}