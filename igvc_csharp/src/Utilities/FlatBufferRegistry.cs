using System.Linq.Expressions;
using System.Reflection;
using Google.FlatBuffers;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Utilities;

public class FlatBufferRegistry
{
    private static readonly ILogger Logger = Logging.From<FlatBufferRegistry>();
    private static readonly Dictionary<Type, Func<ByteBuffer, object>> Factories = new ();

    public static void Scan()
    {
        var assembly = Assembly.GetExecutingAssembly();
        foreach (var type in assembly.GetTypes())
        {
            if (!type.IsValueType || !type.IsPublic)
            {
                continue;
            }

            var method = type.GetMethod(
                "GetRootAs" + type.Name,
                BindingFlags.Public | BindingFlags.Static,
                binder: null,
                [typeof(ByteBuffer)],
                modifiers: null
            );
            if (method == null)
            {
                continue;
            }

            var del = GetFactory(type, method);
            Factories[type] = del;
            Logger.LogDebug("Registered {Type} for FlatBuffer resolving", type.Name);
        }
    }

    public static T Resolve<T>(ByteBuffer bb) where T : struct
    {
        if (!Factories.TryGetValue(typeof(T), out var factory))
        {
            throw new InvalidOperationException($"No FlatBuffer factory for {typeof(T).Name}");
        }

        return (T)factory(bb);
    }

    private static Func<ByteBuffer, object> GetFactory(Type type, MethodInfo method)
    {
        var bbParam = Expression.Parameter(typeof(ByteBuffer), "bb");
        var call = Expression.Call(method, bbParam);
        var cast = Expression.Convert(call, typeof(object));
        
        return Expression
            .Lambda<Func<ByteBuffer, object>>(cast, bbParam)
            .Compile();
    }
}