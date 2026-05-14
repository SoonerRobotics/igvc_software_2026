<script lang="ts">
    import { onMount, onDestroy } from "svelte";
    import maplibregl from "maplibre-gl";
    import type { MapLocation } from "$lib/config/locations";

    const { location } = $props<{ location: MapLocation }>();

    let container: HTMLDivElement;
    let map: maplibregl.Map;

    function buildStyle(loc: MapLocation) {
        return {
            version: 8 as const,
            sources: {
                satellite: {
                    type: "raster" as const,
                    tiles: [loc.tilesPath],
                    tileSize: 256,
                    attribution: "© Esri, Maxar, Earthstar Geographics",
                    bounds: [...loc.bounds[0], ...loc.bounds[1]] as [
                        number,
                        number,
                        number,
                        number,
                    ],
                    minzoom: 14,
                    maxzoom: 19,
                },
            },
            layers: [
                {
                    id: "background",
                    type: "background" as const,
                    paint: { "background-color": "#0e1117" },
                },
                {
                    id: "satellite",
                    type: "raster" as const,
                    source: "satellite",
                },
            ],
        };
    }

    $effect(() => {
        if (map && location) {
            map.setMaxBounds(location.bounds);
            map.jumpTo({
                center: location.center,
                zoom: location.defaultZoom,
            });
            (
                map.getSource("satellite") as maplibregl.RasterTileSource
            )?.setTiles([location.tilesPath]);
        }
    });

    onMount(() => {
        map = new maplibregl.Map({
            container,
            style: buildStyle(location),
            center: location.center,
            zoom: location.defaultZoom,
            maxBounds: location.bounds,
            attributionControl: false,
        });

        map.on("error", (e) => console.error("MapLibre error:", e));

map.on('load', () => {
    setTimeout(() => map.resize(), 1000);
});

        map.addControl(
            new maplibregl.AttributionControl({ compact: true }),
            "bottom-right",
        );
        map.addControl(
            new maplibregl.NavigationControl({ showCompass: true }),
            "top-right",
        );
        map.addControl(
            new maplibregl.ScaleControl({ unit: "imperial" }),
            "bottom-left",
        );
    });

    onDestroy(() => {
        map?.remove();
    });
</script>

<div class="flex w-full h-full flex-1 *:flex-1">
    <div bind:this={container}></div>
</div>

<style>
    @import "maplibre-gl/dist/maplibre-gl.css";
</style>
