import { useRobotStore } from "@/lib/robot";
import { Loader2 } from "lucide-react";
import { Input } from "./ui/input";

export default function LoadingOverlay() {
    const setPath = useRobotStore((s) => s.setPath);

    return (
        <div className="fixed inset-0 flex items-center justify-center bg-gray-900 bg-opacity-50 z-50">
            <div className="flex flex-col items-center space-y-4">
                <Loader2 className="animate-spin" size={48} />
                <span className="text-white text-lg">Connecting to Robot...</span>
                <Input
                    placeholder="WebSocket URL"
                    defaultValue="ws://localhost:8080"
                    onChange={(e) => setPath(e.target.value)}
                    className="w-64"
                />
            </div>
        </div>
    )
}