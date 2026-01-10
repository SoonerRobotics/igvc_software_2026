namespace igvc_csharp.Core.Config;

public interface IConfigBinding
{
    string Path { get; }
    Type ValueType { get; }

    object Get();
    bool TrySet(object value);

    object Serialize();
}