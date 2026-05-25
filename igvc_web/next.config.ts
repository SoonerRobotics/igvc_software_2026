import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  /* config options here */
  devIndicators: false,
  allowedDevOrigins: ['192.168.1.83', '192.168.8.235'],
};

export default nextConfig;
