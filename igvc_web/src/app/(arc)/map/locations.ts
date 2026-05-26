export interface MapLocation {
    id: string;
    name: string;
    center: [number, number];
    bounds: [[number, number], [number, number]];
    tilesPath: string;
    defaultZoom: number;
}
export const LOCATIONS: MapLocation[] = [
    {
        id: 'igvc',
        name: 'IGVC - Oakland University',
        center: [-83.219444, 42.66925],
        bounds: [[-83.2228, 42.6668], [-83.2118, 42.6697]],
        tilesPath: '/tiles/igvc/{z}/{x}/{y}.png',
        defaultZoom: 16,
    },
    {
        id: 'norman',
        name: 'Norman - University of Oklahoma',
        center: [-97.4419, 35.209999999999994],
        bounds: [[-97.4457, 35.2084], [-97.4381, 35.2116]],
        tilesPath: '/tiles/norman/{z}/{x}/{y}.png',
        defaultZoom: 17,
    }
];
export const DEFAULT_LOCATION = LOCATIONS[0];