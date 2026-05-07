using System.Net.Sockets;
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
public class SimulatorSubsystem(ControllerSubsystem controllerSubsystem) : SubsystemBase
{
    private const Endianness Endianness = Configuration.SimulatorSubsystem.Endianness;

    private TcpClient? _client;
    private Task? _connectTask;
    private CancellationTokenSource? _internalCts;

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
        {
            await _internalCts.CancelAsync();
        }

        if (_connectTask != null)
        {
            try
            {
                await _connectTask;
            }
            catch
            {
                // ignore
            }
        }

        // TODO: Cleanup

        _internalCts?.Dispose();
        _internalCts = null;

        SetOperatingState(SubsystemState.Shutdown);
    }

    public override async Task Restart()
    {
        await Shutdown();
        await Init(LifetimeToken);
    }

    // TCP Stuff

    private async Task ConnectionLoop(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            try
            {
                SetOperatingState(SubsystemState.Idle);
                _client = new TcpClient();
                await _client.ConnectAsync(
                    Configuration.SimulatorSubsystem.Host,
                    Configuration.SimulatorSubsystem.Port,
                    token
                );

                SetOperatingState(SubsystemState.Operating);
                await ReceiveLoop(_client, token);
            }
            catch (OperationCanceledException)
            {
                // ignored
            }
            catch (Exception ex)
            {
                Logger.LogWarning(ex, "Simulator connection error, retrying in {Delay}",
                    Configuration.SimulatorSubsystem.ReconnectDelay);
            }
            finally
            {
                // TODO: CLeanup
            }

            try
            {
                await Task.Delay(Configuration.SimulatorSubsystem.ReconnectDelay, token);
            }
            catch (OperationCanceledException)
            {
                break;
            }
        }
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

        EventBus.Instance.Publish(new MessageWrapperEvent(wrapper));
    }

    public async Task SendCanFrame(CanFrame frame)
    {
        var arcFrame = MessageConstructor.CreateCanMessage(frame);
        var wrapper = MessageWrapper.From(MessageType.CanFrame, arcFrame.ByteBuffer.ToFullArray());
        await SendWrapper(wrapper);
    }

    private async Task SendWrapper(MessageWrapper wrapper)
    {
        if (_client is not { Connected: true })
        {
            return;
        }

        // TODO: Store the stream?
        var bytes = MessageWriter.Write(wrapper.Type, wrapper.Data, Configuration.SimulatorSubsystem.Endianness);
        await using var stream = _client.GetStream();
        await stream.WriteAsync(bytes);
    }

    private async Task ReceiveLoop(TcpClient client, CancellationToken token)
    {
        await using var stream = client.GetStream();
        var buffer = new byte[Configuration.SimulatorSubsystem.ReceiveBufferSize];
        var accumulator = new MessageAccumulator(
            Endianness,
            ProcessMessage,
            initialCapacity: Configuration.SimulatorSubsystem.ReceiveBufferSize
        );

        try
        {
            while (!token.IsCancellationRequested && client.Connected)
            {
                var bytesRead = await stream.ReadAsync(buffer, token);
                if (bytesRead == 0)
                {
                    break;
                }

                accumulator.Append(buffer.AsSpan(0, bytesRead));
            }
        }
        catch (OperationCanceledException)
        {
            // ignored
        }
    }
}