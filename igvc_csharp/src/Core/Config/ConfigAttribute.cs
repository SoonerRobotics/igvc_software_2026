namespace igvc_csharp.Core.Config;

[AttributeUsage(AttributeTargets.Property | AttributeTargets.Field)]
public sealed class ConfigAttribute(string path) : Attribute
{
    public string Path { set; get; } = path;
}