using System.Globalization;

namespace igvc_csharp.Utilities;

public class ColorUtilities
{
    public sealed class Color
    {
        public double R { get; }
        public double G { get; }
        public double B { get; }
        public double A { get; }

        private Color(double r, double g, double b, double a = 1.0)
        {
            R = Clamp01(r);
            G = Clamp01(g);
            B = Clamp01(b);
            A = Clamp01(a);
        }

        public static Color FromRgb(double r, double g, double b, double a = 1.0)
            => new Color(r, g, b, a);

        public static Color FromRgb255(int r, int g, int b, int a = 255)
            => new Color(r / 255.0, g / 255.0, b / 255.0, a / 255.0);

        public static Color FromHex(string hex)
        {
            hex = hex.TrimStart('#');

            if (hex.Length != 6 && hex.Length != 8)
                throw new ArgumentException("Hex must be RRGGBB or RRGGBBAA");

            byte r = byte.Parse(hex.Substring(0, 2), NumberStyles.HexNumber);
            byte g = byte.Parse(hex.Substring(2, 2), NumberStyles.HexNumber);
            byte b = byte.Parse(hex.Substring(4, 2), NumberStyles.HexNumber);
            byte a = hex.Length == 8
                ? byte.Parse(hex.Substring(6, 2), NumberStyles.HexNumber)
                : (byte)255;

            return FromRgb255(r, g, b, a);
        }

        public static Color FromHsv(double h, double s, double v, double a = 1.0)
        {
            h = Mod(h, 360);
            s = Clamp01(s);
            v = Clamp01(v);

            double c = v * s;
            double x = c * (1 - Math.Abs((h / 60.0 % 2) - 1));
            double m = v - c;

            double r;
            double g;
            double b;
            switch (h)
            {
                case < 60:
                    (r, g, b) = (c, x, 0);
                    break;
                case < 120:
                    (r, g, b) = (x, c, 0);
                    break;
                case < 180:
                    (r, g, b) = (0, c, x);
                    break;
                case < 240:
                    (r, g, b) = (0, x, c);
                    break;
                case < 300:
                    (r, g, b) = (x, 0, c);
                    break;
                default:
                    (r, g, b) = (c, 0, x);
                    break;
            }

            return new Color(r + m, g + m, b + m, a);
        }

        public static Color FromHsl(double h, double s, double l, double a = 1.0)
        {
            h = Mod(h, 360);
            s = Clamp01(s);
            l = Clamp01(l);

            double c = (1 - Math.Abs(2 * l - 1)) * s;
            double x = c * (1 - Math.Abs((h / 60.0 % 2) - 1));
            double m = l - c / 2;

            double r;
            double g;
            double b;
            switch (h)
            {
                case < 60:
                    (r, g, b) = (c, x, 0);
                    break;
                case < 120:
                    (r, g, b) = (x, c, 0);
                    break;
                case < 180:
                    (r, g, b) = (0, c, x);
                    break;
                case < 240:
                    (r, g, b) = (0, x, c);
                    break;
                case < 300:
                    (r, g, b) = (x, 0, c);
                    break;
                default:
                    (r, g, b) = (c, 0, x);
                    break;
            }

            return new Color(r + m, g + m, b + m, a);
        }

        public (double H, double S, double V) ToHsv()
        {
            double max = Math.Max(R, Math.Max(G, B));
            double min = Math.Min(R, Math.Min(G, B));
            double delta = max - min;

            double h = delta switch
            {
                0 => 0,
                _ when max == R => 60 * (((G - B) / delta) % 6),
                _ when max == G => 60 * (((B - R) / delta) + 2),
                _ => 60 * (((R - G) / delta) + 4)
            };

            if (h < 0) h += 360;

            double s = max == 0 ? 0 : delta / max;
            return (h, s, max);
        }

        public (double H, double S, double L) ToHsl()
        {
            double max = Math.Max(R, Math.Max(G, B));
            double min = Math.Min(R, Math.Min(G, B));
            double delta = max - min;

            double l = (max + min) / 2;

            double s = delta == 0
                ? 0
                : delta / (1 - Math.Abs(2 * l - 1));

            double h = delta switch
            {
                0 => 0,
                _ when max == R => 60 * (((G - B) / delta) % 6),
                _ when max == G => 60 * (((B - R) / delta) + 2),
                _ => 60 * (((R - G) / delta) + 4)
            };

            if (h < 0) h += 360;

            return (h, s, l);
        }

        public string ToHex(bool includeAlpha = false)
        {
            int r = (int)Math.Round(R * 255);
            int g = (int)Math.Round(G * 255);
            int b = (int)Math.Round(B * 255);
            int a = (int)Math.Round(A * 255);

            return includeAlpha
                ? $"#{r:X2}{g:X2}{b:X2}{a:X2}"
                : $"#{r:X2}{g:X2}{b:X2}";
        }

        private static double Clamp01(double v)
            => v < 0 ? 0 : v > 1 ? 1 : v;

        private static double Mod(double x, double m)
            => (x % m + m) % m;
    }

    public sealed class ColorRange
    {
        public double MinHue { get; }
        public double MaxHue { get; }
        public double MinSaturation { get; }
        public double MaxSaturation { get; }
        public double MinValue { get; }
        public double MaxValue { get; }

        public ColorRange(
            double minHue, double maxHue,
            double minSaturation, double maxSaturation,
            double minValue, double maxValue)
        {
            MinHue = NormalizeHue(minHue);
            MaxHue = NormalizeHue(maxHue);
            MinSaturation = Clamp01(minSaturation);
            MaxSaturation = Clamp01(maxSaturation);
            MinValue = Clamp01(minValue);
            MaxValue = Clamp01(maxValue);
        }

        public bool Contains(Color color)
        {
            var (h, s, v) = color.ToHsv();

            return HueInRange(h)
                   && s >= MinSaturation && s <= MaxSaturation
                   && v >= MinValue && v <= MaxValue;
        }

        private bool HueInRange(double h)
        {
            // Handles wrap-around (e.g. 350°–10°)
            if (MinHue <= MaxHue)
                return h >= MinHue && h <= MaxHue;

            return h >= MinHue || h <= MaxHue;
        }

        private static double NormalizeHue(double h)
            => (h % 360 + 360) % 360;

        private static double Clamp01(double v)
            => v < 0 ? 0 : v > 1 ? 1 : v;

        public static ColorRange From(Color lower, Color upper)
        {
            var hsvA = lower.ToHsv();
            var hsvB = upper.ToHsv();
            return new ColorRange(
                hsvA.H,
                hsvB.H,
                hsvA.S,
                hsvB.S,
                hsvA.V,
                hsvB.V
            );
        }
    }
}