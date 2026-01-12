using System.Net.Sockets;
using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.MessageUtils;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems;

[Subsystem("SimulatorSubsystem", Disabled = !Constants.UseSimulation)]
public class SimulatorSubsystem : SubsystemBase
{
    private const Endianness Endianness = Constants.SimulatorSubsystem.Endianness;

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
        
        SetState(SubsystemState.Initialized);
        return Task.CompletedTask;
    }

    public override async Task Shutdown()
    {
        SetState(SubsystemState.ShuttingDown);

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

        SetState(SubsystemState.Shutdown);
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
                SetState(SubsystemState.Ready);
                _client = new TcpClient();
                await _client.ConnectAsync(
                    Constants.SimulatorSubsystem.Host,
                    Constants.SimulatorSubsystem.Port,
                    token
                );

                SetState(SubsystemState.Operating);
                await ReceiveLoop(_client, token);
            }
            catch (OperationCanceledException)
            {
                // ignored
            }
            catch (Exception ex)
            {
                Logger.LogWarning(ex, "Simulator connection error, retrying in {Delay}",
                    Constants.SimulatorSubsystem.ReconnectDelay);
            }
            finally
            {
                // TODO: CLeanup
            }

            try
            {
                await Task.Delay(Constants.SimulatorSubsystem.ReconnectDelay, token);
            }
            catch (OperationCanceledException)
            {
                break;
            }
        }
    }

    private void ProcessMessage(MessageWrapper wrapper)
    {
        EventBus.Instance.Publish(new MessageWrapperEvent(wrapper));
    }

    private async Task ReceiveLoop(TcpClient client, CancellationToken token)
    {
        var stream = client.GetStream();
        var buffer = new byte[Constants.SimulatorSubsystem.ReceiveBufferSize];
        var accumulator = new MessageAccumulator(
            Endianness,
            ProcessMessage,
            initialCapacity: Constants.SimulatorSubsystem.ReceiveBufferSize
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