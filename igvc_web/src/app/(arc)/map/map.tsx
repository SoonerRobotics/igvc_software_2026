import { useEffect, useRef, useCallback } from "react";
import maplibregl from "maplibre-gl";
import "maplibre-gl/dist/maplibre-gl.css";
import { MapLocation } from "./locations";

// ─── Types ────────────────────────────────────────────────────────────────────

export interface RobotMarker {
    lng: number;
    lat: number;
    /** Heading in degrees, 0 = north, clockwise */
    heading: number;
}

export interface Waypoint {
    id: string | number;
    lng: number;
    lat: number;
    /** Radius in metres */
    radius: number;
    /** Optional display label */
    label?: string;
    /** Whether this is the current target waypoint */
    isTarget?: boolean;
}

export interface DistanceLine {
    from: { lng: number; lat: number };
    to: { lng: number; lat: number };
    distanceMeters: number;
    bearingDegrees: number;
}

export interface SatelliteMapProps {
    location: MapLocation;
    robot?: RobotMarker | null;
    waypoints?: Waypoint[];
    distanceLine?: DistanceLine | null;
}

// ─── Constants ────────────────────────────────────────────────────────────────

const ROBOT_SOURCE = "robot-source";
const ROBOT_LAYER_BODY = "robot-body";
const ROBOT_LAYER_HEADING = "robot-heading";
const WP_SOURCE_PREFIX = "wp-src-";
const WP_FILL_PREFIX = "wp-fill-";
const WP_STROKE_PREFIX = "wp-stroke-";
const WP_LABEL_PT_PREFIX = "wp-label-pt-";
const WP_LABEL_PREFIX = "wp-label-";
const DIST_SOURCE = "dist-source";
const DIST_LINE_LAYER = "dist-line";
const DIST_LABEL_LAYER = "dist-label";

// ─── Colors ───────────────────────────────────────────────────────────────────

const COLOR_DEFAULT = "#00d4ff";
const COLOR_TARGET = "#f97316";
const COLOR_DIST = "#facc15";
const COLOR_ROBOT = "#ff6b35";
const COLOR_BG = "#0e1117";

// ─── Geo helpers ──────────────────────────────────────────────────────────────

function circleGeoJSON(
    lng: number,
    lat: number,
    radiusM: number,
    steps = 64
): GeoJSON.Feature<GeoJSON.Polygon> {
    const R = 6_371_000;
    const dLat = (radiusM / R) * (180 / Math.PI);
    const dLng = dLat / Math.cos((lat * Math.PI) / 180);
    const coords: [number, number][] = Array.from({ length: steps + 1 }, (_, i) => {
        const a = (i / steps) * 2 * Math.PI;
        return [lng + dLng * Math.cos(a), lat + dLat * Math.sin(a)];
    });
    return { type: "Feature", geometry: { type: "Polygon", coordinates: [coords] }, properties: {} };
}

function robotGeoJSON(robot: RobotMarker) {
    const R = 6_371_000;
    const lineLen = 12;
    const headingRad = (robot.heading * Math.PI) / 180;
    const dLat = ((lineLen / R) * Math.cos(headingRad) * 180) / Math.PI;
    const dLng =
        ((lineLen / R / Math.cos((robot.lat * Math.PI) / 180)) * Math.sin(headingRad) * 180) /
        Math.PI;

    const body: GeoJSON.Feature<GeoJSON.Point> = {
        type: "Feature",
        geometry: { type: "Point", coordinates: [robot.lng, robot.lat] },
        properties: { heading: robot.heading },
    };
    const heading: GeoJSON.Feature<GeoJSON.LineString> = {
        type: "Feature",
        geometry: {
            type: "LineString",
            coordinates: [
                [robot.lng, robot.lat],
                [robot.lng + dLng, robot.lat + dLat],
            ],
        },
        properties: {},
    };
    return { body, heading };
}

// ─── Map style ────────────────────────────────────────────────────────────────

function buildStyle(loc: MapLocation): maplibregl.StyleSpecification {
    return {
        version: 8,
        sources: {
            satellite: {
                type: "raster",
                tiles: [loc.tilesPath],
                tileSize: 256,
                attribution: "© Esri, Maxar, Earthstar Geographics",
                bounds: [...loc.bounds[0], ...loc.bounds[1]] as [number, number, number, number],
                minzoom: 14,
                maxzoom: 19,
            },
        },
        layers: [
            { id: "background", type: "background", paint: { "background-color": COLOR_BG } },
            { id: "satellite", type: "raster", source: "satellite" },
        ],
    };
}

// ─── Component ────────────────────────────────────────────────────────────────

export default function SatelliteMap({
    location,
    robot = null,
    waypoints = [],
    distanceLine = null,
}: SatelliteMapProps) {
    const containerRef = useRef<HTMLDivElement>(null);
    const mapRef = useRef<maplibregl.Map | null>(null);
    const prevWaypointIds = useRef<Set<string | number>>(new Set());
    const mapReadyRef = useRef(false);

    // ── Waypoint layers ───────────────────────────────────────────────────────

    const addWaypoint = useCallback((map: maplibregl.Map, wp: Waypoint) => {
        const srcId = `${WP_SOURCE_PREFIX}${wp.id}`;
        const fillId = `${WP_FILL_PREFIX}${wp.id}`;
        const strokeId = `${WP_STROKE_PREFIX}${wp.id}`;
        const labelPtId = `${WP_LABEL_PT_PREFIX}${wp.id}`;
        const labelId = `${WP_LABEL_PREFIX}${wp.id}`;

        if (map.getSource(srcId)) return;

        const color = wp.isTarget ? COLOR_TARGET : COLOR_DEFAULT;

        map.addSource(srcId, {
            type: "geojson",
            data: circleGeoJSON(wp.lng, wp.lat, wp.radius),
        });
        map.addLayer({
            id: fillId,
            type: "fill",
            source: srcId,
            paint: { "fill-color": color, "fill-opacity": wp.isTarget ? 0.18 : 0.08 },
        });
        map.addLayer({
            id: strokeId,
            type: "line",
            source: srcId,
            paint: {
                "line-color": color,
                "line-width": wp.isTarget ? 2.5 : 1.5,
                "line-dasharray": wp.isTarget ? [1, 0] : [4, 3],
                "line-opacity": 0.9,
            },
        });

        map.addSource(labelPtId, {
            type: "geojson",
            data: {
                type: "Feature",
                geometry: { type: "Point", coordinates: [wp.lng, wp.lat] },
                properties: { label: wp.label ?? `WP ${wp.id}` },
            } as GeoJSON.Feature,
        });
        map.addLayer({
            id: labelId,
            type: "symbol",
            source: labelPtId,
            layout: {
                "text-field": ["get", "label"],
                "text-font": ["Open Sans Bold", "Arial Unicode MS Bold"],
                "text-size": 11,
                "text-anchor": "center",
            },
            paint: {
                "text-color": color,
                "text-halo-color": COLOR_BG,
                "text-halo-width": 2,
            },
        });
    }, []);

    const removeWaypoint = useCallback((map: maplibregl.Map, id: string | number) => {
        [`${WP_FILL_PREFIX}${id}`, `${WP_STROKE_PREFIX}${id}`, `${WP_LABEL_PREFIX}${id}`].forEach(
            (l) => { if (map.getLayer(l)) map.removeLayer(l); }
        );
        [`${WP_SOURCE_PREFIX}${id}`, `${WP_LABEL_PT_PREFIX}${id}`].forEach(
            (s) => { if (map.getSource(s)) map.removeSource(s); }
        );
    }, []);

    const updateWaypoint = useCallback((map: maplibregl.Map, wp: Waypoint) => {
        // Update circle geometry
        (map.getSource(`${WP_SOURCE_PREFIX}${wp.id}`) as maplibregl.GeoJSONSource | undefined)
            ?.setData(circleGeoJSON(wp.lng, wp.lat, wp.radius));

        // Update label point
        (map.getSource(`${WP_LABEL_PT_PREFIX}${wp.id}`) as maplibregl.GeoJSONSource | undefined)
            ?.setData({
                type: "Feature",
                geometry: { type: "Point", coordinates: [wp.lng, wp.lat] },
                properties: { label: wp.label ?? `WP ${wp.id}` },
            } as GeoJSON.Feature);

        // Update colors to reflect isTarget change
        const color = wp.isTarget ? COLOR_TARGET : COLOR_DEFAULT;
        const fillId = `${WP_FILL_PREFIX}${wp.id}`;
        const strokeId = `${WP_STROKE_PREFIX}${wp.id}`;
        const labelId = `${WP_LABEL_PREFIX}${wp.id}`;

        if (map.getLayer(fillId)) {
            map.setPaintProperty(fillId, "fill-color", color);
            map.setPaintProperty(fillId, "fill-opacity", wp.isTarget ? 0.18 : 0.08);
        }
        if (map.getLayer(strokeId)) {
            map.setPaintProperty(strokeId, "line-color", color);
            map.setPaintProperty(strokeId, "line-width", wp.isTarget ? 2.5 : 1.5);
            map.setPaintProperty(
                strokeId,
                "line-dasharray",
                wp.isTarget ? [1, 0] : [4, 3]
            );
        }
        if (map.getLayer(labelId)) {
            map.setPaintProperty(labelId, "text-color", color);
        }
    }, []);

    // ── Robot layers ──────────────────────────────────────────────────────────

    const ensureRobotLayers = useCallback((map: maplibregl.Map) => {
        if (map.getSource(ROBOT_SOURCE)) return;
        map.addSource(ROBOT_SOURCE, {
            type: "geojson",
            data: { type: "FeatureCollection", features: [] },
        });
        map.addLayer({
            id: ROBOT_LAYER_HEADING,
            type: "line",
            source: ROBOT_SOURCE,
            filter: ["==", "$type", "LineString"],
            paint: { "line-color": COLOR_ROBOT, "line-width": 2.5, "line-opacity": 0.9 },
        });
        map.addLayer({
            id: ROBOT_LAYER_BODY,
            type: "circle",
            source: ROBOT_SOURCE,
            filter: ["==", "$type", "Point"],
            paint: {
                "circle-radius": 8,
                "circle-color": COLOR_ROBOT,
                "circle-stroke-width": 2,
                "circle-stroke-color": "#ffffff",
            },
        });
    }, []);

    const syncRobot = useCallback(
        (map: maplibregl.Map, robot: RobotMarker | null) => {
            ensureRobotLayers(map);
            const src = map.getSource(ROBOT_SOURCE) as maplibregl.GeoJSONSource | undefined;
            if (!src) return;
            if (!robot) {
                src.setData({ type: "FeatureCollection", features: [] });
                return;
            }
            const { body, heading } = robotGeoJSON(robot);
            src.setData({ type: "FeatureCollection", features: [body, heading] });
        },
        [ensureRobotLayers]
    );

    // ── Distance line layers ──────────────────────────────────────────────────

    const ensureDistanceLayers = useCallback((map: maplibregl.Map) => {
        if (map.getSource(DIST_SOURCE)) return;
        map.addSource(DIST_SOURCE, {
            type: "geojson",
            data: { type: "FeatureCollection", features: [] },
        });
        map.addLayer({
            id: DIST_LINE_LAYER,
            type: "line",
            source: DIST_SOURCE,
            filter: ["==", "$type", "LineString"],
            paint: {
                "line-color": COLOR_DIST,
                "line-width": 2,
                "line-dasharray": [6, 3],
                "line-opacity": 0.9,
            },
        });
        map.addLayer({
            id: DIST_LABEL_LAYER,
            type: "symbol",
            source: DIST_SOURCE,
            filter: ["==", "$type", "Point"],
            layout: {
                "text-field": ["get", "label"],
                "text-font": ["Open Sans Bold", "Arial Unicode MS Bold"],
                "text-size": 12,
                "text-anchor": "center",
                "text-offset": [0, -1.2],
            },
            paint: {
                "text-color": COLOR_DIST,
                "text-halo-color": COLOR_BG,
                "text-halo-width": 2,
            },
        });
    }, []);

    const syncDistanceLine = useCallback(
        (map: maplibregl.Map, line: DistanceLine | null) => {
            ensureDistanceLayers(map);
            const src = map.getSource(DIST_SOURCE) as maplibregl.GeoJSONSource | undefined;
            if (!src) return;

            if (!line) {
                src.setData({ type: "FeatureCollection", features: [] });
                return;
            }

            const midLng = (line.from.lng + line.to.lng) / 2;
            const midLat = (line.from.lat + line.to.lat) / 2;
            const distLabel =
                line.distanceMeters < 1000
                    ? `${line.distanceMeters.toFixed(1)}m`
                    : `${(line.distanceMeters / 1000).toFixed(2)}km`;

            const lineFeature: GeoJSON.Feature<GeoJSON.LineString> = {
                type: "Feature",
                geometry: {
                    type: "LineString",
                    coordinates: [
                        [line.from.lng, line.from.lat],
                        [line.to.lng, line.to.lat],
                    ],
                },
                properties: {},
            };

            const labelFeature: GeoJSON.Feature<GeoJSON.Point> = {
                type: "Feature",
                geometry: { type: "Point", coordinates: [midLng, midLat] },
                properties: {
                    label: `${distLabel} · ${line.bearingDegrees.toFixed(0)}°`,
                },
            };

            src.setData({
                type: "FeatureCollection",
                features: [lineFeature, labelFeature],
            });
        },
        [ensureDistanceLayers]
    );

    // ── Mount ─────────────────────────────────────────────────────────────────

    useEffect(() => {
        if (!containerRef.current) return;

        const map = new maplibregl.Map({
            container: containerRef.current,
            style: buildStyle(location),
            center: location.center,
            zoom: location.defaultZoom,
            maxBounds: location.bounds,
            attributionControl: false,
        });

        mapRef.current = map;

        map.on("error", (e) => console.error("MapLibre error:", e));
        map.on("load", () => {
            mapReadyRef.current = true;
            setTimeout(() => map.resize(), 100);
            syncRobot(map, robot ?? null);
            syncDistanceLine(map, distanceLine ?? null);
            waypoints.forEach((wp) => addWaypoint(map, wp));
            prevWaypointIds.current = new Set(waypoints.map((w) => w.id));
        });

        map.addControl(new maplibregl.AttributionControl({ compact: true }), "bottom-right");
        map.addControl(new maplibregl.NavigationControl({ showCompass: true }), "top-right");
        map.addControl(new maplibregl.ScaleControl({ unit: "imperial" }), "bottom-left");

        return () => {
            map.remove();
            mapRef.current = null;
            mapReadyRef.current = false;
        };
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    // ── Sync location ─────────────────────────────────────────────────────────

    useEffect(() => {
        const map = mapRef.current;
        if (!map || !mapReadyRef.current) return;
        map.setMaxBounds(location.bounds);
        map.jumpTo({ center: location.center, zoom: location.defaultZoom });
        (map.getSource("satellite") as maplibregl.RasterTileSource | undefined)?.setTiles([
            location.tilesPath,
        ]);
    }, [location]);

    // ── Sync robot ────────────────────────────────────────────────────────────

    useEffect(() => {
        const map = mapRef.current;
        if (!map || !mapReadyRef.current) return;
        syncRobot(map, robot ?? null);
    }, [robot, syncRobot]);

    // ── Sync waypoints ────────────────────────────────────────────────────────

    useEffect(() => {
        const map = mapRef.current;
        if (!map || !mapReadyRef.current) return;

        const incoming = new Set(waypoints.map((w) => w.id));
        const prev = prevWaypointIds.current;

        prev.forEach((id) => { if (!incoming.has(id)) removeWaypoint(map, id); });
        waypoints.forEach((wp) => {
            prev.has(wp.id) ? updateWaypoint(map, wp) : addWaypoint(map, wp);
        });

        prevWaypointIds.current = incoming;
    }, [waypoints, addWaypoint, removeWaypoint, updateWaypoint]);

    // ── Sync distance line ────────────────────────────────────────────────────

    useEffect(() => {
        const map = mapRef.current;
        if (!map || !mapReadyRef.current) return;
        syncDistanceLine(map, distanceLine ?? null);
    }, [distanceLine, syncDistanceLine]);

    // ── Render ────────────────────────────────────────────────────────────────

    return (
        <div style={{ display: "flex", width: "100%", height: "100%", flex: 1 }}>
            <div ref={containerRef} style={{ flex: 1 }} />
        </div>
    );
}