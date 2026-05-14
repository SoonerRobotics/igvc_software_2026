using Microsoft.Extensions.Logging;

namespace igvc_csharp.Core;

public sealed class LogInterceptorProvider : ILoggerProvider
{
    public delegate void LogHandler(string category, LogLevel level, EventId eventId, string message, Exception? exception);
    
    private readonly LogHandler _handler;

    public LogInterceptorProvider(LogHandler handler)
    {
        _handler = handler;
    }

    public ILogger CreateLogger(string categoryName)
        => new InterceptorLogger(categoryName, _handler);

    public void Dispose() { }

    private sealed class InterceptorLogger(string category, LogHandler handler) : ILogger
    {
        public IDisposable? BeginScope<TState>(TState state) where TState : notnull => null;
        public bool IsEnabled(LogLevel logLevel) => true;

        public void Log<TState>(LogLevel logLevel, EventId eventId, TState state,
            Exception? exception, Func<TState, Exception?, string> formatter)
        {
            handler(category, logLevel, eventId, formatter(state, exception), exception);
        }
    }
}