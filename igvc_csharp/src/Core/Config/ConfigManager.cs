using System.Reflection;
using System.Text.Json;
using igvc_csharp.Events;

namespace igvc_csharp.Core.Config;

public sealed class ConfigManager
{
    private static readonly Dictionary<string, IConfigBinding> _bindings = new();
    public static IReadOnlyDictionary<string, IConfigBinding> Bindings => _bindings;

    public static void Initialize()
    {
        DiscoverConstants();
    }
    
    private static void DiscoverConstants()
    {
        var types = AppDomain.CurrentDomain
            .GetAssemblies()
            .SelectMany(a => a.GetTypes());

        foreach (var type in types)
        {
            foreach (var field in type.GetFields( BindingFlags.Static | BindingFlags.Public))
            {
                var attr = field.GetCustomAttribute<ConfigAttribute>();
                if (attr == null)
                {
                    continue;
                }

                _bindings[attr.Path] = new FieldConfigBinding(field, attr);
            }
        }
    }
    
    public static T Get<T>(string path) => (T)_bindings[path].Get();

    public static bool Set(string path, object value)
    {
        if (_bindings.TryGetValue(path, out var binding))
        {
            return false;
        }

        if (!binding.TrySet(value))
        {
            return false;
        }

        EventBus.Instance.Publish(new ConfigChangedEvent(path, value));
        return true;
    }
}