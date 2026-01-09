using System.Reflection;
using igvc_csharp.Events;
using igvc_csharp.Messages;
using igvc_csharp.MessageUtils;
using Messages;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Core;

public class SubsystemBase : ISubsystem
{
    protected readonly ILogger Logger;

    /// <summary>
    /// The name of this subsystem.
    /// </summary>
    private string Name { get; }

    /// <summary>
    /// The current state of this subsystem.
    /// NOTE: If you are listening to StateChanged, this will reflect the new state.
    /// </summary>
    public SubsystemState State { get; private set; } = SubsystemState.Initialized;

    /// <summary>
    /// Called when this subsystems state is changed.
    /// The first parameter is the old state, the 2nd parameter is the new state.
    /// </summary>
    public event Action<SubsystemState, SubsystemState>? StateChanged;

    /// <summary>
    /// The Lifetime CancellationToken assigned on initialization. Use this for any new tokens.
    /// </summary>
    protected CancellationToken LifetimeToken { get; private set; }

    protected SubsystemBase()
    {
        Name = GetSubsystemName(GetType());
        Logger = Logging.From(GetType());
    }

    protected void Subscribe<TEvent>(
        Func<TEvent, CancellationToken, Task> handler,
        CancellationToken token)
        where TEvent : IRobotEvent
    {
        var reader = EventBus.Instance
            .GetChannel<TEvent>()
            .Reader;

        Task.Run(async () =>
        {
            try
            {
                await foreach (var evt in reader.ReadAllAsync(token))
                {
                    await handler(evt, token);
                }
            }
            catch (OperationCanceledException)
            {
                // expected
            }
            catch (Exception ex)
            {
                Logger.LogError(
                    ex,
                    "Event subscription failed for {Event}",
                    typeof(TEvent).Name);
            }
        }, token);
    }

    protected void Subscribe<TIn, TOut>(
        Func<TIn, bool> filter,
        Func<TIn, TOut> transform,
        Func<TOut, CancellationToken, Task> handler,
        CancellationToken token)
        where TIn : IRobotEvent
    {
        var reader = EventBus.Instance
            .GetChannel<TIn>()
            .Reader;

        Task.Run(async () =>
        {
            try
            {
                await foreach (var evt in reader.ReadAllAsync(token))
                {
                    if (!filter(evt))
                    {
                        continue;
                    }

                    TOut value;
                    try
                    {
                        value = transform(evt);
                    }
                    catch (Exception ex)
                    {
                        Logger.LogWarning(
                            ex,
                            "Event transform failed for {Event}",
                            typeof(TIn).Name);
                        continue;
                    }

                    await handler(value, token);
                }
            }
            catch (OperationCanceledException)
            {
                // expected
            }
            catch (Exception ex)
            {
                Logger.LogError(
                    ex,
                    "Event subscription failed for {Event}",
                    typeof(TIn).Name);
            }
        }, token);
    }

    protected void SubscribeMessage<T>(
        MessageType type,
        Func<T, CancellationToken, Task> handler,
        CancellationToken token)
        where T : struct
    {
        Subscribe<MessageWrapperEvent, T>(
            filter: evt => evt.Wrapper.Type == type,
            transform: evt => evt.Wrapper.As<T>(),
            handler: handler,
            token: token);
    }

    protected void SubscribeImage(
        string identifier,
        Func<ImageFrame, CancellationToken, Task> handler,
        CancellationToken token)
    {
        Subscribe<MessageWrapperEvent, ImageFrame>(
            filter: evt =>
                evt.Wrapper.Type == MessageType.ImageFrame && evt.Wrapper.As<ImageFrame>().Identifier == identifier,
            transform: evt => evt.Wrapper.As<ImageFrame>(),
            handler: handler,
            token: token);
    }

    public virtual Task Init(CancellationToken token)
    {
        SetState(SubsystemState.Initialized);
        LifetimeToken = token;
        return Task.CompletedTask;
    }

    protected void EnsureLifetime()
    {
        if (LifetimeToken.IsCancellationRequested)
        {
            throw new OperationCanceledException("Subsystem ensured lifetime, but lifetime was cancelled");
        }
    }

    public virtual Task Periodic(CancellationToken token)
    {
        return Task.CompletedTask;
    }

    public virtual Task Shutdown()
    {
        return Task.CompletedTask;
    }

    public virtual Task Restart()
    {
        return Task.CompletedTask;
    }

    protected void SetState(SubsystemState newState)
    {
        if (State == newState)
        {
            return;
        }

        var oldState = State;
        State = newState;
        StateChanged?.Invoke(oldState, newState);

        Logger.LogInformation("Subsystem {Subsystem} state changed: {OldState} -> {NewState}", Name, oldState,
            newState);
    }

    private static string GetSubsystemName(Type type)
    {
        var attr = type.GetCustomAttribute<SubsystemAttribute>();
        return !string.IsNullOrWhiteSpace(attr?.Name)
            ? attr!.Name
            : type.Name;
    }
}