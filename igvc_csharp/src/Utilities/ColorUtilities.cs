using System.Globalization;
using System.Text.Json;
using igvc_csharp.Core;
using igvc_csharp.Core.Config;

namespace igvc_csharp.Utilities;

public static class ColorUtilities
{
    public sealed class Color : IConfigSerializable
    {
        public byte R { private set; get; }
        public byte G { private set; get; }
        public byte B { private set; get; }
        public byte A { private set; get; }

        private Color(byte r, byte g, byte b, byte a = 255)
        {
            R = Assertions.ByteInRange(r, 0, 255);
            G = Assertions.ByteInRange(g, 0, 255);
            B = Assertions.ByteInRange(b, 0, 255);
            A = Assertions.ByteInRange(a, 0, 255);
        }
        
        // Builders
        
        public static Color FromRgb(byte r, byte g, byte b) => new (r, g, b);
        public static Color FromRgba(byte r, byte g, byte b, byte a) => new (r, g, b, a);

        public static Color FromHex(string hex)
        {
            hex = hex.Replace("#", "");
            
            // Validate its at least 6 or 8 characters
            hex = Assertions.StringIsLengths(hex, [6, 8]);
            
            // Add #FF if its not 8
            if (hex.Length != 8)
            {
                hex += "FF";
            }
            
            // Convert
            var r = Convert.ToByte(hex.Substring(0, 2), 16);
            var g = Convert.ToByte(hex.Substring(2, 2), 16);
            var b = Convert.ToByte(hex.Substring(4, 2), 16);
            var a = Convert.ToByte(hex.Substring(6, 2), 16);
            return new Color(r, g, b, a);
        }

        public static Color FromHsv(int h, int s, int v)
        {
            h = Assertions.IntInRange(h, 0, 180);
            s = Assertions.IntInRange(s, 0, 255);
            v = Assertions.IntInRange(v, 0, 255);
            
            var hf = h * 2f;
            var sf = s / 255f;
            var vf = v / 255f;

            var c = vf * sf;
            var x = c * (1 - Math.Abs((hf / 60f) % 2 - 1));
            var m = vf - c;

            float r1 = 0, g1 = 0, b1 = 0;

            switch (hf)
            {
                case < 60:
                    r1 = c; g1 = x; b1 = 0;
                    break;
                case < 120:
                    r1 = x; g1 = c; b1 = 0;
                    break;
                case < 180:
                    r1 = 0; g1 = c; b1 = x;
                    break;
                case < 240:
                    r1 = 0; g1 = x; b1 = c;
                    break;
                case < 300:
                    r1 = x; g1 = 0; b1 = c;
                    break;
                default:
                    r1 = c; g1 = 0; b1 = x;
                    break;
            }

            var r = (byte)Math.Round((r1 + m) * 255);
            var g = (byte)Math.Round((g1 + m) * 255);
            var b = (byte)Math.Round((b1 + m) * 255);

            return new Color(r, g, b);
        }
        
        // Helpers

        public string ToHex(bool includeHashtag = true)
        {
            var hash = includeHashtag ? "#" : "";
            return $"{hash}{R:X2}{G:X2}:{B:X2}{A:X2}";
        }
        
        public(int h, int s, int v) ToHsv()
        {
            var rf = R / 255f;
            var gf = G / 255f;
            var bf = B / 255f;

            var max = Math.Max(rf, Math.Max(gf, bf));
            var min = Math.Min(rf, Math.Min(gf, bf));
            var delta = max - min;

            var hDeg = 0f;

            if (delta > 0)
            {
                if (max == rf)
                {
                    hDeg = 60f * (((gf - bf) / delta) % 6f);
                }
                else if (max == gf)
                {
                    hDeg = 60f * (((bf - rf) / delta) + 2f);
                }
                else
                {
                    hDeg = 60f * (((rf - gf) / delta) + 4f);
                }
            }

            if (hDeg < 0)
            {
                hDeg += 360f;
            }

            var sFloat = max == 0 ? 0 : delta / max;

            var hCv = (int)Math.Round(hDeg / 2f);
            var sCv = (int)Math.Round(sFloat * 255f);
            var vCv = (int)Math.Round(max * 255f);
            return (hCv, sCv, vCv);
        }

        
        // Configuration
        
        public object Serialize() => new { r = R, g = G, b = B, a = A };
        
        public object Deserialize(object value)
        {
            var obj = (JsonElement)value;
            R = obj.GetProperty("r").GetByte();
            G = obj.GetProperty("g").GetByte();
            B = obj.GetProperty("b").GetByte();
            A = obj.GetProperty("a").GetByte();
            return this;
        }

        public static Color StaticDeserialize(object value)
        {
            return (Color)(new Color(0, 0, 0).Deserialize(value));
        }

        // Modifiers

        public Color WithAlpha(byte alpha)
        {
            A = alpha;
            return this;
        }
        
        // Constants
        
        public static Color White => new (255, 255, 255);
    }

    public sealed class ColorRange : IConfigSerializable
    {
        public Color Lower { get; private set; }
        public Color Upper { get; private set; }

        private ColorRange(Color lower, Color upper)
        {
            Lower = lower;
            Upper = upper;
        }
        
        public static ColorRange From(Color lower, Color upper)
        {
            return new ColorRange(lower, upper);
        }

        public object Serialize() => new { min = Lower.Serialize(), max = Upper.Serialize() };
        public object Deserialize(object value)
        {
            var obj = (JsonElement)value;
            var lowerObj = obj.GetProperty("lower");
            var upperObj = obj.GetProperty("upper");
            Lower = Color.StaticDeserialize(lowerObj);
            Upper = Color.StaticDeserialize(upperObj);
            return this;
        }
    }
}