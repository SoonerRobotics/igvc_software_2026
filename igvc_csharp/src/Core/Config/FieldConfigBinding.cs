using System.Reflection;

namespace igvc_csharp.Core.Config;

internal sealed class FieldConfigBinding : IConfigBinding
{
    private readonly FieldInfo _field;

    public string Path { get; }
    public Type ValueType => _field.FieldType;
    public bool IsReadOnly => _field.IsLiteral || _field.IsInitOnly;

    public FieldConfigBinding(FieldInfo field, ConfigAttribute attr)
    {
        _field = field;
        Path = attr.Path;
    }

    public object Get() => _field.GetValue(null)!;

    public bool TrySet(object value)
    {
        if (IsReadOnly) return false;
        if (!ValueType.IsInstanceOfType(value)) return false;
        _field.SetValue(null, value);
        return true;
    }

    public object Serialize() => ConfigSerializer.Serialize(Get());
}