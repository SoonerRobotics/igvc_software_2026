using System.Linq.Expressions;
using System.Reflection;
using Google.FlatBuffers;
using Messages;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Utils.Messages;

public class FlatBufferRegistry
{
    private static readonly ILogger Logger = Logging.From<FlatBufferRegistry>();
    private static readonly Dictionary<Type, Func<ByteBuffer, object>> Factories = new();

    public static void Scan()
    {
        var assembly = Assembly.GetExecutingAssembly();
        Type[] types;
        try
        {
            types = assembly.GetTypes();
        }
        catch (ReflectionTypeLoadException ex)
        {
            foreach (var loaderEx in ex.LoaderExceptions.Where(e => e != null))
                Logger.LogWarning("Assembly load warning during FlatBuffer scan: {Message}", loaderEx!.Message);
            types = ex.Types.Where(t => t != null).ToArray()!;
        }
        
        foreach (var type in types)
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
        var bbParam = Expression.Parameter(typeof(ByteBuffer), "_bb");
        var call = Expression.Call(method, bbParam);
        var cast = Expression.Convert(call, typeof(object));

        return Expression
            .Lambda<Func<ByteBuffer, object>>(cast, bbParam)
            .Compile();
    }
}