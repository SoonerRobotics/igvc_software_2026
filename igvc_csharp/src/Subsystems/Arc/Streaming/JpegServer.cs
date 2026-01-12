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

        var channel = JpegStreamRegistry.GetOrCreate(streamId);
        var output = context.Response.OutputStream;

        try
        {
            await foreach (var jpeg in channel.Reader.ReadAllAsync(ct))
            {
                if (!context.Response.OutputStream.CanWrite)
                    break;

                var header =
                    "--frame\r\n" +
                    "Content-Type: image/jpeg\r\n" +
                    $"Content-Length: {jpeg.Length}\r\n\r\n";

                var headerBytes = Encoding.ASCII.GetBytes(header);

                await output.WriteAsync(headerBytes, ct);
                await output.WriteAsync(jpeg, ct);
                await output.WriteAsync("\r\n"u8.ToArray(), ct);
                await output.FlushAsync(ct);
            }
        }
        catch
        {
            // Client disconnected
        }
        finally
        {
            output.Close();
        }
    }
}