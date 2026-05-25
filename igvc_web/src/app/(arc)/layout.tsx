import "@/app/globals.css";

import type { Metadata } from "next";
import { Roboto } from "next/font/google";
import CoreLayout from '@/components/core-layout';

export const dynamic = 'force-dynamic';
export const ssr = false;

const inter = Roboto({
    subsets: ["latin"],
    weight: ["400", "500", "700"],
});

export const metadata: Metadata = {
    title: "IGVC 2026 | Suspended Disbelief",
};

export default function RootLayout({
    children,
}: Readonly<{
    children: React.ReactNode;
}>) {
    return (
        <html lang="en">
            <body className={`${inter.className} min-h-screen h-screen w-full`}>
                <CoreLayout>
                    {children}
                </CoreLayout>
            </body>
        </html>
    );
}
