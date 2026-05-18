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
        var allTypes = new List<Type>();

        foreach (var assembly in AppDomain.CurrentDomain.GetAssemblies())
        {
            try
            {
                allTypes.AddRange(assembly.GetTypes());
            }
            catch (ReflectionTypeLoadException ex)
            {
                allTypes.AddRange(ex.Types.Where(t => t != null)!);
                foreach (var loaderEx in ex.LoaderExceptions.Where(e => e != null))
                {
                    Logger.LogError("Type load error: {Message}", loaderEx!.Message);
                }
            }
        }

        var discovered = new List<Type>();

        foreach (var t in allTypes)
        {
            if (t.IsAbstract || !typeof(SubsystemBase).IsAssignableFrom(t))
            {
                continue;
            }

            var attr = t.GetCustomAttribute<SubsystemAttribute>();
            if (attr == null)
            {
                Logger.LogWarning("Subsystem {Subsystem} has no [Subsystem] attribute, skipping", t.Name);
                continue;
            }

            if (attr.Disabled)
            {
                Logger.LogWarning("Subsystem {Subsystem} is disabled, skipping", t.Name);
                continue;
            }

            discovered.Add(t);
        }

        return discovered;
    }

    private static IReadOnlyList<Type> GetDependencies(Type type)
    {
        var ctor = type.GetConstructors()
            .OrderByDescending(c => c.GetParameters().Length)
            .FirstOrDefault();

        if (ctor == null) return [];

        return ctor.GetParameters()
            .Where(p => typeof(SubsystemBase).IsAssignableFrom(p.ParameterType))
            .Select(p => p.ParameterType)
            .ToList();
    }

    private static List<Type> ResolveDependencies(List<Type> types)
    {
        var resolved = new List<Type>();
        var remaining = new HashSet<Type>(types);
        var depMap = types.ToDictionary(t => t, GetDependencies);

        // Warn about deps that have no known implementation (will be injected as null)
        foreach (var (type, deps) in depMap)
        {
            foreach (var dep in deps)
            {
                var satisfiable = types.Any(t => dep.IsAssignableFrom(t));
                if (!satisfiable)
                {
                    Logger.LogWarning(
                        "Subsystem {Subsystem} declares dependency {Dep} which has no known implementation, " +
                        "it will be injected as null",
                        type.Name, dep.Name);
                }
            }
        }

        bool progressed;
        do
        {
            progressed = false;

            foreach (var type in remaining.ToArray())
            {
                var deps = depMap[type];

                // A non-nullable dep must already be resolved; nullable deps are always satisfied
                var allDepsSatisfied = deps.All(dep =>
                    resolved.Any(r => dep.IsAssignableFrom(r))
                );

                if (!allDepsSatisfied) continue;

                resolved.Add(type);
                remaining.Remove(type);
                progressed = true;
            }
        } while (progressed);

        foreach (var type in remaining)
        {
            var unsatisfied = depMap[type]
                .Where(dep => !resolved.Any(r => dep.IsAssignableFrom(r)))
                .Select(dep => dep.Name);

            Logger.LogError(
                "Subsystem {Subsystem} could not be ordered, unsatisfied or circular dependencies: {Dependencies}",
                type.Name, string.Join(", ", unsatisfied));
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

    private SubsystemBase? CreateSubsystem(Type type)
    {
        var ctor = type.GetConstructors()
            .OrderByDescending(c => c.GetParameters().Length)
            .FirstOrDefault();

        if (ctor == null)
        {
            return Activator.CreateInstance(type) as SubsystemBase;
        }

        var args = new List<object?>();
        foreach (var param in ctor.GetParameters())
        {
            if (!typeof(SubsystemBase).IsAssignableFrom(param.ParameterType))
            {
                Logger.LogError("Unsupported constructor parameter type {Param} ({Type}) on {Subsystem}",
                    param.Name, param.ParameterType.Name, type.Name);
                return null;
            }

            var dep = _subsystemsByType
                .FirstOrDefault(kvp => param.ParameterType.IsAssignableFrom(kvp.Key))
                .Value;

            // Allow injection of null for nullable parameters (e.g. CanbusSubsystem?)
            var isNullable = param.GetCustomAttribute<NullableAttribute>() != null
                             || Nullable.GetUnderlyingType(param.ParameterType) != null;

            if (dep == null && !isNullable)
            {
                Logger.LogError(
                    "Could not resolve constructor parameter {Param} ({Type})",
                    param.Name, param.ParameterType.Name);
                return null;
            }

            args.Add(dep);
        }

        return ctor.Invoke(args.ToArray()) as SubsystemBase;
    }

    protected void CallSubsystemFunction(string name, params object[] pms)
    {
        foreach (var subsystem in _subsystems)
        {
            var subsystemType = subsystem.GetType();
            var methods = subsystemType.GetMethods(BindingFlags.NonPublic | BindingFlags.Instance);
            foreach (var method in methods)
            {
                if (method.Name != name) continue;
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
                if (subsystem == null) continue;

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
        if (_disposed) return;
        _disposed = true;
        _subsystems.Clear();
    }

    public void SetMobility(bool mobility)
    {
        var oldState = State.Clone();
        State.MotionAllowed = mobility;
        CallSubsystemFunction("OnRobotStateChanged", oldState, State);
        Logger.LogDebug("Robot Mobility Changed -> {}", mobility);
    }

    public void SetEstopped(bool estopped)
    {
        var oldState = State.Clone();
        State.Estopped = estopped;
        CallSubsystemFunction("OnRobotStateChanged", oldState, State);
        Logger.LogDebug("Robot Estopped Changed -> {}", estopped);
    }

    public void SetMode(RobotModeEnum mode)
    {
        var oldState = State.Clone();
        State.Mode = mode;
        CallSubsystemFunction("OnRobotStateChanged", oldState, State);
        Logger.LogDebug("Robot Mode Changed -> {old} to {new}", oldState.Mode.ToString(), mode.ToString());
    }

    public void SetMission(MissionEnum mission)
    {
        var oldState = State.Clone();
        State.Mission = mission;
        CallSubsystemFunction("OnRobotStateChanged", oldState, State);
        Logger.LogDebug("Robot Mission Changed -> {old} to {new}", oldState.Mission.ToString(), mission.ToString());
    }
}