using SocketCANSharp;

namespace igvc_csharp.CanSpec;

public interface ICanMessage<T> where T : ICanMessage<T>
{
    static abstract T Read(byte[] data);
    CanFrame Write();
}