namespace igvc_csharp.Core;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;
using Subsystems;
public abstract class BaseRobot : IDisposable
{
    private static readonly ILogger Logger = Logging.From<BaseRobot>();

    private readonly List<ISubsystem> _subsystems = [];
    private bool _initialized;
    private bool _disposed;

    private IEnumerable<Type> GetEnabledSubsystemTypes()
    {
        return Assembly.GetExecutingAssembly()
            .GetTypes()
            .Where(t =>
                !t.IsAbstract &&
                typeof(ISubsystem).IsAssignableFrom(t) &&
                t.GetCustomAttribute<SubsystemAttribute>() is { Disabled: false });
    }

    public virtual async Task Init(CancellationToken token)
    {
        if (_initialized)
        {
            throw new InvalidOperationException("Robot already initialized");
        }

        Logger.LogInformation("Discovering subsystems");
        foreach (var type in GetEnabledSubsystemTypes())
        {
            try
            {
                if (Activator.CreateInstance(type) is not ISubsystem subsystem)
                {
                    Logger.LogWarning("Failed to create subsystem {Subsystem}", type.Name);
                    continue;
                }

                _subsystems.Add(subsystem);
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

        for (int i = _subsystems.Count - 1; i >= 0; i--)
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
}
