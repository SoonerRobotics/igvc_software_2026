namespace igvc_csharp.Core.Config;

public interface IConfigSerializable
{
    object Serialize();

    object Deserialize(object value);
}