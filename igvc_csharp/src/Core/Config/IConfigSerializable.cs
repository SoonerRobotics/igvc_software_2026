namespace igvc_csharp.Core.Config;

public interface IConfigSerializable
{
    object Serialize();
    void Deserialize(object value);
}