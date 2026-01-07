namespace igvc_sharp.Utilities;

public class FileUtilities
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

    public static string GetFileRelativeToRoot(string file, string subfolder = "")
    {
        if (subfolder == string.Empty)
        {
            return Path.Join(GetProjectRootDirectory(), file);
        }
        
        return Path.Join(Path.Join(GetProjectRootDirectory(), subfolder), file);
    }
}