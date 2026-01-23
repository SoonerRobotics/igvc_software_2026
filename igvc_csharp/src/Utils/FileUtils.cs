namespace igvc_csharp.Utils;

public static class FileUtils
{
    private const string ResourcesFolder = "resources";
    private const string TraversalString = "../../../";

    public static string GetProjectRootDirectory()
    {
        var cwd = Directory.GetCurrentDirectory();

        // Check if `resources` folder exists
        var resourcesPath = Path.Join(cwd, ResourcesFolder);
        if (Directory.Exists(resourcesPath))
        {
            return cwd;
        }

        resourcesPath = Path.Join(Path.Join(cwd, TraversalString), ResourcesFolder);
        return Directory.Exists(resourcesPath) ? Path.Join(cwd, TraversalString) : string.Empty;
    }

    public static string GetRepositoryRootDirectory()
    {
        var projectRoot =  GetProjectRootDirectory();
        return Path.Join(projectRoot, "..");
    }

    public static string GetFileRelativeToRoot(string file, string subfolder = "")
    {
        return Path.Join(
            subfolder == string.Empty ? GetProjectRootDirectory() : Path.Join(GetProjectRootDirectory(), subfolder),
            file);
    }

    public static string ExpandPath(string path)
    {
        if (!path.StartsWith('~'))
        {
            return path;
        }
        
        var home = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
        return Path.Combine(home, path.TrimStart('~', '/', '\\'));
    }
}