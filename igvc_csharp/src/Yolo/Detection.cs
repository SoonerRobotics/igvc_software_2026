using OpenCvSharp;

namespace igvc_csharp.Yolo;

public sealed record Detection (string Label, float Confidence, Rect Bounding);