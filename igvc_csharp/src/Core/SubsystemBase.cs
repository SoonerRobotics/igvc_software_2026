using System.Reflection;
using igvc_csharp.Core.Performance;
using igvc_csharp.Events;
using igvc_csharp.Messages;
using igvc_csharp.Utils.Messages;
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

    // IGVC Stuff
    
    protected SubsystemBase()
    {
        Name = GetSubsystemName(GetType());
        Logger = Logging.From(GetType());

        // Register any metrics we have
        RegisterMetrics();
    }

    protected void Subscribe<TEvent>(
        Func<TEvent, CancellationToken, Task> handler,
        CancellationToken token)
        where TEvent : IRobotEvent
    {
        var reader = EventBus.Instance.Subscribe<TEvent>();

        Task.Run(async () =>
        {
            try
            {
                await foreach (var evt in reader.ReadAllAsync(token))
                {
                    // Run the callback in the background
                    _ = Task.Run(() => handler(evt, token), token);
                }
            }
            catch (OperationCanceledException)
            {
                // expected
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "Event subscription failed for {Event}", typeof(TEvent).Name);
            }
        }, token);
    }

    protected void RegisterMetrics()
    {
        var type = GetType();
        var subsystem = type.Name;

        foreach (var field in type.GetFields(
                     BindingFlags.Instance |
                     BindingFlags.NonPublic |
                     BindingFlags.Public))
        {
            var attr = field.GetCustomAttribute<MetricAttribute>();
            if (attr == null)
                continue;
            
            var definition = new MetricDefinition(
                subsystem,
                attr.Group,
                attr.Name,
                attr.Unit,
                attr.Aggregate);

            var metric = Activator.CreateInstance(
                field.FieldType,
                definition,
                attr.MaxSamples,
                attr.MaxAgeSeconds > 0
                    ? TimeSpan.FromSeconds(attr.MaxAgeSeconds)
                    : null,
                attr.EmitEveryMs)!;

            field.SetValue(this, metric);
            PerformanceRegistry.Instance.Register((IPerformanceMetric)metric);
        }
    }


    protected void Subscribe<TIn, TOut>(
        Func<TIn, bool> filter,
        Func<TIn, TOut> transform,
        Func<TOut, CancellationToken, Task> handler,
        CancellationToken token)
        where TIn : IRobotEvent
    {
        var reader = EventBus.Instance.Subscribe<TIn>();

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
        Subscribe<MessageWrapperEvent>(
            async (evt, ct) =>
            {
                if (evt.Wrapper.Type != MessageType.ImageFrame)
                    return;

                var frame = evt.Wrapper.As<ImageFrame>();
                if (frame.Identifier != identifier)
                    return;

                await handler(frame, ct);
            },
            token);
    }

    public virtual Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Initialized);
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

    public virtual Task OnRobotModeChanged(RobotModeEnum old, RobotModeEnum current)
    {
        return Task.CompletedTask;
    }

    public virtual Task OnRobotMissionChanged(MissionEnum old, MissionEnum current)
    {
        return Task.CompletedTask;
    }

    public virtual Task OnRobotEstopped()
    {
        return Task.CompletedTask;
    }

    public virtual Task OnMobilityStart()
    {
        return Task.CompletedTask;
    }
    
    public virtual Task OnMobilityStop()
    {
        return Task.CompletedTask;
    }
    
    // Setters

    protected void SetOperatingState(SubsystemState newState)
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

    protected void SetRobotMode(RobotModeEnum mode)
    {
        Robot.Instance.SetMode(mode);
    }

    protected void SetRobotMission(MissionEnum mission)
    {
        Robot.Instance.SetMission(mission);
    }

    protected void SetMobility(bool mobility)
    {
        Robot.Instance.SetMobility(mobility);
    }

    private static string GetSubsystemName(Type type)
    {
        var attr = type.GetCustomAttribute<SubsystemAttribute>();
        return !string.IsNullOrWhiteSpace(attr?.Name)
            ? attr!.Name
            : type.Name;
    }
}