"use client";

export default function ChronosPage()
{
    const handleFolderUpload = (e: React.ChangeEvent<HTMLInputElement>) => {
        const files = e.target.files;
        if (files == null)
        {
            return;
        }

        console.log("Uploaded files:", files);
    };

    return (
        <div className="flex flex-col">
            <h1 className="text-2xl font-bold mb-4">Chronos</h1>
            <input
                type="file"
                webkitdirectory="true"
                mozdirectory="true"
                directory="true"
                onChange={handleFolderUpload}
                className="mb-4"
            />
            <p className="text-gray-600">Upload a folder containing your Chronos data.</p>
        </div>
    )
}