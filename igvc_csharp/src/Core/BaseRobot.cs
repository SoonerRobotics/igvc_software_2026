using System.Runtime.CompilerServices;

namespace igvc_csharp.Core;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;
using Messages;

public abstract class BaseRobot : IDisposable
{
    private static readonly ILogger Logger = Logging.From<BaseRobot>();

    // Instance
    public static BaseRobot? Instance { get; protected set; }

    // Subsystem stuff
    private readonly List<SubsystemBase> _subsystems = [];
    private readonly Dictionary<Type, SubsystemBase> _subsystemsByType = new();


    // State
    private bool _initialized;
    private bool _disposed;
    public RobotState State { get; } = new();

    protected BaseRobot()
    {
        if (Instance != null)
        {
            throw new InvalidOperationException("BaseRobot instance already exists");
        }

        Instance = this;
    }

    private static List<Type> DiscoverSubsystems()
    {
        return Assembly.GetExecutingAssembly()
            .GetTypes()
            .Where(t =>
                !t.IsAbstract &&
                typeof(SubsystemBase).IsAssignableFrom(t) &&
                t.GetCustomAttribute<SubsystemAttribute>() is { Disabled: false }
            ).ToList();
    }

    private static List<Type> ResolveDependencies(List<Type> types)
    {
        var remaining = new HashSet<Type>(types);
        var resolved = new List<Type>();

        var depMap = types.ToDictionary(
            t => t,
            t => t.GetCustomAttribute<SubsystemAttribute>()!.DependsOn
        );
        bool progressed;

        do
        {
            progressed = false;

            foreach (var type in remaining.ToArray())
            {
                var deps = depMap[type];
                if (!deps.All(d => resolved.Contains(d)))
                {
                    continue;
                }

                resolved.Add(type);
                remaining.Remove(type);
                progressed = true;
            }
        } while (progressed);

        if (remaining.Count > 0)
        {
            foreach (var type in remaining)
            {
                var attr = type.GetCustomAttribute<SubsystemAttribute>();
                if (attr == null)
                {
                    // This should never happen due to all of the previous checks
                    continue;
                }

                var missing = attr.DependsOn.Where(d => !resolved.Contains(d)).Select(d => d.Name);
                Logger.LogError("Subsystem {Subsystem} not created due to missing dependencies: {Dependencies}",
                    type.Name, string.Join(", ", missing));
            }
        }

        return resolved;
    }

    public T? GetSubsystem<T>() where T : SubsystemBase
    {
        _subsystemsByType.TryGetValue(typeof(T), out var subsystem);
        return subsystem as T;
    }

    public object GetSubsystem(Type type)
    {
        _subsystemsByType.TryGetValue(type, out var subsystem);
        return subsystem!;
    }

    protected void CallSubsystemFunction(string name, params object[] pms)
    {
        foreach (var subsystem in _subsystems)
        {
            var subsystemType = subsystem.GetType();
            var methods = subsystemType.GetMethods(BindingFlags.NonPublic | BindingFlags.Instance);
            foreach (var method in methods)
            {
                if (method.Name != name)
                {
                    continue;
                }

                method.Invoke(subsystem, pms);
            }
        }
    }

    public virtual async Task Init(CancellationToken token)
    {
        if (_initialized)
        {
            throw new InvalidOperationException("Robot already initialized");
        }

        Logger.LogInformation("Discovering subsystems");
        var subsystemTypes = DiscoverSubsystems();
        var orderedTypes = ResolveDependencies(subsystemTypes);
        foreach (var type in orderedTypes)
        {
            try
            {
                var subsystem = CreateSubsystem(type);
                if (subsystem == null)
                {
                    continue;
                }

                _subsystems.Add(subsystem);
                _subsystemsByType.Add(type, subsystem);
                Logger.LogInformation("Created subsystem {Subsystem}", type.Name);
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "Error creating subsystem {Subsystem}", type.Name);
            }
        }

        Logger.LogInformation("Initializing {Count} subsystems", _subsystems.Count);
        foreach (var subsystem in _subsystems)
        {
            try
            {
                await subsystem.Init(token);
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "Subsystem init failed: {Subsystem}", subsystem.GetType().Name);
            }
        }

        _initialized = true;
    }

    private SubsystemBase? CreateSubsystem(Type type)
    {
        var ctor = type.GetConstructors()
            .OrderByDescending(c => c.GetParameters().Length)
            .FirstOrDefault();

        // For those that do not have a constructor, just create them generically and call it a day
        if (ctor == null)
        {
            var instance = Activator.CreateInstance(type);
            return instance as SubsystemBase;
        }

        var args = new List<object?>();
        foreach (var param in ctor.GetParameters())
        {
            if (!typeof(SubsystemBase).IsAssignableFrom(param.ParameterType))
            {
                Logger.LogError("Unsupported constructor parameter {Param}", param.Name);
                return null;
            }

            _subsystemsByType.TryGetValue(param.ParameterType, out var dep);
            // var attribute = param.GetCustomAttribute<SubsystemDependencyAttribute>();
            // if (dep == null && attribute is { Required: true })
            // {
            //     Logger.LogError("Missing required dependency {Dep}", param.ParameterType.Name);
            //     return null;
            // }

            args.Add(dep);
        }

        return ctor.Invoke(args.ToArray()) as SubsystemBase;
    }

    public virtual async Task Periodic(CancellationToken token)
    {
        foreach (var subsystem in _subsystems)
        {
            try
            {
                await subsystem.Periodic(token);
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "Subsystem periodic error: {Subsystem}", subsystem.GetType().Name);
            }
        }
    }

    public virtual async Task Shutdown()
    {
        Logger.LogInformation("Shutting down subsystems");

        for (var i = _subsystems.Count - 1; i >= 0; i--)
        {
            var subsystem = _subsystems[i];
            try
            {
                await subsystem.Shutdown();
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "Subsystem shutdown failed: {Subsystem}", subsystem.GetType().Name);
            }
        }
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _disposed = true;
        _subsystems.Clear();
    }

    // State

    public void SetMobility(bool mobility)
    {
        var oldState = State;
        State.MotionAllowed = mobility;
        CallSubsystemFunction("OnRobotStateChanged", oldState, State);
        Logger.LogDebug("Robot Mobility Changed -> {}", mobility);
    }

    public void SetEstopped(bool estopped)
    {
        var oldState = State;
        State.Estopped = estopped;
        CallSubsystemFunction("OnRobotStateChanged", oldState, State);
        Logger.LogDebug("Robot Estopped Changed -> {}", estopped);
    }

    public void SetMode(RobotModeEnum mode)
    {
        var oldState = State;
        State.Mode = mode;
        CallSubsystemFunction("OnRobotStateChanged", oldState, State);
        Logger.LogDebug("Robot Mode Changed -> {} to {}", oldState.Mode.ToString(), mode.ToString());
    }

    public void SetMission(MissionEnum mission)
    {
        var oldState = State;
        State.Mission = mission;
        CallSubsystemFunction("OnRobotStateChanged", oldState, State);
        Logger.LogDebug("Robot Mission Changed -> {} to {}", oldState.Mission.ToString(), mission.ToString());
    }
}