using System.Text.Json;
using igvc_csharp.Core.Units;
using igvc_csharp.Utils;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Core.Config;

/// <summary>
/// Central serialization/deserialisation for all config value types.
/// All values are serialized to JSON-safe primitives/objects for Arc and preset files.
/// </summary>
public static class ConfigSerializer
{
    // ── Serialize ─────────────────────────────────────────────────────────────

    public static object Serialize(object value) => value switch
    {
        // Primitives
        bool b => b,
        int i => i,
        double d => d,
        float f => f,
        long l => l,
        ulong ul => ul,
        string s => s,

        // TimeSpan → total milliseconds
        TimeSpan ts => ts.TotalMilliseconds,

        // LogLevel → string name
        Microsoft.Extensions.Logging.LogLevel ll => ll.ToString(),

        // byte[] → base64 string
        byte[] bytes => Convert.ToBase64String(bytes),

        // LinearVelocity → { mph } — user-friendly unit shown in Arc
        LinearVelocity lv => new { mph = lv.ToMilesPerHour() },

        // AngularVelocity → { dps } — degrees per second
        AngularVelocity av => new { dps = av.To(AngularVelocityUnit.DegreesPerSecond) },

        // ColorUtils.Color → { h, s, v } OpenCV HSV (h:0-179, s:0-255, v:0-255)
        ColorUtils.Color c => SerializeColorHsv(c),

        // ColorUtils.ColorRange → { min: {h,s,v}, max: {h,s,v} }
        ColorUtils.ColorRange cr => new
        {
            min = SerializeColorHsv(cr.Lower),
            max = SerializeColorHsv(cr.Upper),
        },

        // Point2f[] → [{x,y}, ...]
        Point2f[] pts => pts.Select(p => new { x = p.X, y = p.Y }).ToArray(),

        // Fallback
        _ => value,
    };

    // ── Deserialize ───────────────────────────────────────────────────────────

    public static object? Deserialize(string json, Type targetType)
    {
        using var doc = JsonDocument.Parse(json);
        return DeserializeElement(doc.RootElement, targetType);
    }

    public static object? DeserializeElement(JsonElement el, Type targetType)
    {
        if (targetType == typeof(bool)) return el.GetBoolean();
        if (targetType == typeof(int)) return el.GetInt32();
        if (targetType == typeof(double)) return el.GetDouble();
        if (targetType == typeof(float)) return (float)el.GetDouble();
        if (targetType == typeof(long)) return el.GetInt64();
        if (targetType == typeof(ulong)) return el.GetUInt64();
        if (targetType == typeof(string)) return el.GetString();

        if (targetType == typeof(TimeSpan))
            return TimeSpan.FromMilliseconds(el.GetDouble());

        if (targetType == typeof(Microsoft.Extensions.Logging.LogLevel))
            return Enum.Parse<Microsoft.Extensions.Logging.LogLevel>(el.GetString()!, ignoreCase: true);

        if (targetType == typeof(byte[]))
            return Convert.FromBase64String(el.GetString()!);

        // LinearVelocity ← { mph }
        if (targetType == typeof(LinearVelocity))
            return LinearVelocityUnit.MilesPerHour.Of(el.GetProperty("mph").GetDouble());

        // AngularVelocity ← { dps }
        if (targetType == typeof(AngularVelocity))
            return AngularVelocityUnit.DegreesPerSecond.Of(el.GetProperty("dps").GetDouble());

        // ColorUtils.Color ← { h, s, v }
        if (targetType == typeof(ColorUtils.Color))
            return DeserializeColorHsv(el);

        // ColorUtils.ColorRange ← { min: {h,s,v}, max: {h,s,v} }
        if (targetType == typeof(ColorUtils.ColorRange))
        {
            return ColorUtils.ColorRange.From(
                DeserializeColorHsv(el.GetProperty("min")),
                DeserializeColorHsv(el.GetProperty("max")));
        }

        // Point2f[] ← [{x,y}, ...]
        if (targetType == typeof(Point2f[]))
        {
            return el.EnumerateArray()
                     .Select(e => new Point2f(
                         e.GetProperty("x").GetSingle(),
                         e.GetProperty("y").GetSingle()))
                     .ToArray();
        }

        return JsonSerializer.Deserialize(el.GetRawText(), targetType);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private static object SerializeColorHsv(ColorUtils.Color c)
    {
        var (h, s, v) = c.ToHsv();
        return new { h, s, v };
    }

    private static ColorUtils.Color DeserializeColorHsv(JsonElement el) =>
        ColorUtils.Color.FromHsv(
            el.GetProperty("h").GetInt32(),
            el.GetProperty("s").GetInt32(),
            el.GetProperty("v").GetInt32());
}