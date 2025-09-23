import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  webpack: (config) => {
    config.experiments = config.experiments ?? {};
    config.experiments.asyncWebAssembly = true;
    config.experiments.layers = true;

    if (!config.resolve.extensions.includes('.wasm')) {
      config.resolve.extensions.push('.wasm');
    }

    return config;
  },
};

export default nextConfig;
