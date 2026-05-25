// global.d.ts
import "react";

declare module "react" {
    interface InputHTMLAttributes<T> extends HTMLAttributes<T> {
        webkitdirectory?: string;
        mozdirectory?: string;
        directory?: string;
    }
}