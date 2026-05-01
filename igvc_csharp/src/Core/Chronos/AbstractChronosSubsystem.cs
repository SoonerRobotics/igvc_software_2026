namespace igvc_csharp.Core.Chronos;

using System;
using System.IO;
using System.Threading.Tasks;
using OpenCvSharp;

public static class EntryTypeId
{
    public const ushort SessionStart = 0x0001;
    public const ushort SessionEnd = 0x0002;
    public const ushort SessionLog = 0x0003;
    public const ushort CameraFrameSync = 0x0040;
}

public static class SessionType
{
    public const ushort Manual = 0x0001;
    public const ushort Autonomous = 0x0002;
    public const ushort Other = 0x0003;
}

public abstract class AbstractChronosSubsystem : SubsystemBase
{
    private readonly string _outputDirectory;
    private ChronosSession? _session;
    private readonly object _sessionLock = new();
    private bool _disposed;

    // ------------------------------------------------------------------
    // Public state
    // ------------------------------------------------------------------
    public bool IsRunning
    {
        get { lock (_sessionLock) return _session?.IsActive ?? false; }
    }

    /// <summary>RunId of the current session, or null if not running.</summary>
    public string? CurrentRunId
    {
        get { lock (_sessionLock) return _session?.RunId; }
    }

    // ------------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------------
    protected AbstractChronosSubsystem()
    {
        _outputDirectory = Configuration.ChronosOutputDirectory.Replace("~", Environment.GetFolderPath(Environment.SpecialFolder.UserProfile));
        Directory.CreateDirectory(_outputDirectory);
    }

    public void StartRun(ushort sessionType)
    {
        lock (_sessionLock)
        {
            if (_session?.IsActive == true)
                throw new InvalidOperationException(
                    "A run is already in progress. Call StopRunAsync() before starting a new one.");

            _session = new ChronosSession(_outputDirectory, sessionType, LifetimeToken);
        }

        OnRunStarted(_session.RunId, sessionType);
    }

    public async Task StopRunAsync()
    {
        ChronosSession? session;
        lock (_sessionLock)
        {
            session = _session;
            _session = null;
        }

        if (session is null)
            throw new InvalidOperationException("No run is currently in progress.");

        OnRunStopping(session.RunId);
        await session.DisposeAsync();
        OnRunStopped(session.RunId);
    }

    public void OpenCamera(int cameraId, int width, int height, double nominalFps = 30.0)
        => GetActiveSession()?.OpenCamera(cameraId, width, height, nominalFps);

    public void WriteVideoFrame(int cameraId, Mat frame)
        => GetActiveSession()?.WriteVideoFrame(cameraId, frame);

    public void WriteEntry(ushort typeId, byte[] payload)
        => GetActiveSession()?.EnqueueEntry(typeId, payload);

    protected virtual void OnRunStarted(string runId, ushort sessionType) { }
    protected virtual void OnRunStopping(string runId) { }
    protected virtual void OnRunStopped(string runId) { }

    private ChronosSession? GetActiveSession()
    {
        lock (_sessionLock)
        {
            return _session?.IsActive == true ? _session : null;
        }
    }

    public override async Task Shutdown()
    {
        await base.Shutdown();

        if (IsRunning)
        {
            await StopRunAsync();
        }
    }
}
