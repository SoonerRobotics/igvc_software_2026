namespace igvc_csharp.External;

using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct FrameHeader
{
    public long Sequence;
    public long LastWriteTicks;

    public int Width;
    public int Height;

    public int RgbStride;
    public int DepthStride;

    public int RgbSizeBytes;
    public int DepthSizeBytes;
}