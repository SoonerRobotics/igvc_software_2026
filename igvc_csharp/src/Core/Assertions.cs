namespace igvc_csharp.Core;

public static class Assertions
{
    // Nullables

    public static object IsNotNull(object? val)
    {
        return val ?? throw new AssertionFailedException("Value was null");
    }

    public static object? IsNull(object? val)
    {
        return val != null ? throw new AssertionFailedException("Value was not null") : val;
    }

    // InRange

    public static byte ByteInRange(byte value, byte min, byte max)
    {
        if (value < min || value > max)
        {
            throw new AssertionFailedException("Value {0} out of range [{1}, {2}]", value, min, max);
        }

        return value;
    }

    public static int IntInRange(int value, int min, int max)
    {
        if (value < min || value > max)
        {
            throw new AssertionFailedException("Value {0} out of range [{1}, {2}]", value, min, max);
        }

        return value;
    }
    
    // Strings

    public static string StringIsLength(string str, int length)
    {
        return str.Length != length
            ? throw new AssertionFailedException("String length {0} is incorrect", length)
            : str;
    }

    public static string StringIsLengths(string str, params int[] lengths)
    {
        return lengths.Any(t => str.Length == t)
            ? str
            : throw new AssertionFailedException("String length {0} is incorrect", lengths.Length);
    }

    public class AssertionFailedException(string message, params object?[] p) : Exception(string.Format(message, p));
}