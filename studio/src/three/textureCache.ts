import type { TextureProvider } from "three-usd-robot";
import type { Texture } from "three";

/** Source textures are immutable; identical sampler uses can share one GPU
 * upload. Keep colour/data interpretations and UV transforms distinct. */
export function sharedTextureProvider(provider: TextureProvider): TextureProvider {
  const cache = new Map<string, Texture | null>();
  return (path, options = {}) => {
    const t = options.transform;
    const key = JSON.stringify([path, options.colorSpace ?? "srgb",
      options.wrapS ?? "repeat", options.wrapT ?? "repeat", options.channel ?? 0,
      t?.scale ?? [1, 1], t?.translation ?? [0, 0], t?.rotation ?? 0]);
    if (!cache.has(key)) cache.set(key, provider(path, options));
    return cache.get(key)!;
  };
}
