using System.Reflection;
using igvc_csharp.Events;
using igvc_csharp.Messages;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Utils;
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

    // Properties
    private List<AbstractSubsystemProperty> _properties = [];
    private SubsystemProperty<string> _pError = new("error");

    private ArcSubsystem? _arcSubsystem;

    // IGVC Stuff

    protected SubsystemBase()
    {
        Name = GetSubsystemName(GetType());
        Logger = Logging.From(GetType());
        RegisterProperties();
    }

    private void RegisterProperties()
    {
        // Get all instances of AbstractSubsystemProperty in this class (and those who have extended this class) and add them to the list
        var props = GetType()
            .GetFields(BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public)
            .Where(f => typeof(AbstractSubsystemProperty).IsAssignableFrom(f.FieldType))
            .Select(f => f.GetValue(this))
            .OfType<AbstractSubsystemProperty>();
        _properties.AddRange(props);

        // Assign "Parent" of each property to this subsystem, it is protected
        foreach (var prop in _properties)
        {
            var parentField = typeof(AbstractSubsystemProperty).GetField("parent", BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
            if (parentField != null)
            {
                Logger.LogTrace("Registering property {Property} for subsystem {Subsystem}", prop, Name);
                parentField.SetValue(prop, this);
            }
        }
    }

    public void OnPropertyUpdated(string key, object? value)
    {
        _arcSubsystem ??= BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (_arcSubsystem != null)
        {
            _ = _arcSubsystem.BroadcastAsync(
                ArcUtils.CreateArcData_PropertyChanged(Name, key, value?.ToString() ?? "null"),
                CancellationToken.None
            );
        }
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

    public virtual Task OnRobotStateChanged(RobotState old, RobotState updated)
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
        BaseRobot.Instance?.SetMode(mode);
    }

    protected void SetRobotMission(MissionEnum mission)
    {
        BaseRobot.Instance?.SetMission(mission);
    }

    protected void SetMobility(bool mobility)
    {
        BaseRobot.Instance?.SetMobility(mobility);
    }

    protected void SetError(string error)
    {
        // Don't set the error if it is the same as the current error, to avoid spamming updates
        if (_pError.Get() == string.Empty && error == string.Empty)
        {
            return;
        }

        _pError.Set(error);
    }

    protected void ClearError()
    {
        _pError.Set(null);
    }

    private static string GetSubsystemName(Type type)
    {
        var attr = type.GetCustomAttribute<SubsystemAttribute>();
        return !string.IsNullOrWhiteSpace(attr?.Name)
            ? attr!.Name
            : type.Name;
    }
}