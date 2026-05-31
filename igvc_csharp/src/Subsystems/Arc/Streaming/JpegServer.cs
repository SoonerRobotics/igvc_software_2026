namespace igvc_csharp.Subsystems.Arc.Streaming;

using System.Net;
using System.Text;

public class JpegServer
{
    private readonly HttpListener _listener = new();

    public JpegServer(string prefix)
    {
        _listener.Prefixes.Add(prefix);
    }

    public async Task StartAsync(CancellationToken ct)
    {
        _listener.Start();
        Console.WriteLine($"MJPEG server listening on {string.Join(", ", _listener.Prefixes)}");

        while (!ct.IsCancellationRequested)
        {
            var context = await _listener.GetContextAsync();
            _ = Task.Run(() => HandleClientAsync(context, ct), ct);
        }
    }

    private static async Task HandleClientAsync(HttpListenerContext context, CancellationToken ct)
    {
        var streamId = context.Request.Url!.AbsolutePath.Trim('/');
        if (string.IsNullOrWhiteSpace(streamId))
        {
            context.Response.StatusCode = 400;
            context.Response.Close();
            return;
        }

        context.Response.StatusCode = 200;
        context.Response.SendChunked = true;
        context.Response.ContentType = "multipart/x-mixed-replace; boundary=frame";
        context.Response.Headers.Add("Cache-Control", "no-cache");
        context.Response.Headers.Add("Connection", "keep-alive");

        var (clientId, channel) = JpegStreamRegistry.Subscribe(streamId);
        var output = context.Response.OutputStream;

        // Per-client CTS so the watchdog and frame loop share a single cancellation signal.
        using var clientCts = CancellationTokenSource.CreateLinkedTokenSource(ct);

        // Watchdog: polls every 500 ms and cancels if the socket is no longer writable.
        // This ensures a dead browser connection is detected promptly even on slow streams
        // rather than waiting until the next WriteAsync throws.
        var watchdog = Task.Run(async () =>
        {
            try
            {
                while (!clientCts.Token.IsCancellationRequested)
                {
                    await Task.Delay(500, clientCts.Token).ConfigureAwait(false);
                    if (!output.CanWrite)
                    {
                        await clientCts.CancelAsync();
                        break;
                    }
                }
            }
            catch (OperationCanceledException) { }
        });

        try
        {
            await foreach (var jpeg in channel.Reader.ReadAllAsync(clientCts.Token))
            {
                var header =
                    "--frame\r\n" +
                    "Content-Type: image/jpeg\r\n" +
                    $"Content-Length: {jpeg.Length}\r\n\r\n";

                try
                {
                    using var writeCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
                    writeCts.CancelAfter(2000); // 2s write deadline per frame

                    await output.WriteAsync(Encoding.ASCII.GetBytes(header), writeCts.Token);
                    await output.WriteAsync(jpeg, writeCts.Token);
                    await output.WriteAsync("\r\n"u8.ToArray(), writeCts.Token);
                    await output.FlushAsync(writeCts.Token);
                }
                catch
                {
                    // Socket write failed — exit immediately so finally runs.
                    break;
                }
            }
        }
        catch (OperationCanceledException) { }
        catch { }
        finally
        {
            await clientCts.CancelAsync();   // stop the watchdog
            await watchdog;                  // wait for it to finish cleanly
            JpegStreamRegistry.Unsubscribe(streamId, clientId);
            output.Close();
        }
    }
}