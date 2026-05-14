#!/bin/bash
set -e

# Usage: ./generate_tiles.sh <name> <north> <south> <east> <west>
# Example: ./generate_tiles.sh igvc 42.6697 42.6668 -83.2118 -83.2228

usage() {
  echo "Usage: $0 <name> <north> <south> <east> <west>"
  exit 1
}

# Check args
if [ "$#" -ne 5 ]; then
  usage
fi

NAME=$1
NORTH=$2
SOUTH=$3
EAST=$4
WEST=$5

TIFF_FILE="${NAME}_area.tif"
TILES_DIR="static/tiles/${NAME}"
XML_FILE="${NAME}_source.xml"

# Check dependencies
if ! command -v gdal_translate &> /dev/null; then
  echo "Error: gdal_translate not found. Install with:"
  echo "  sudo apt install gdal-bin"
  exit 1
fi

if ! command -v gdal2tiles.py &> /dev/null; then
  echo "Error: gdal2tiles.py not found. Install with:"
  echo "  sudo apt install gdal-bin python3-gdal"
  exit 1
fi

# Write GDAL WMS XML source
cat > "${XML_FILE}" <<EOF
<GDAL_WMS>
  <Service name="TMS">
    <ServerUrl>https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/\${z}/\${y}/\${x}</ServerUrl>
  </Service>
  <DataWindow>
    <UpperLeftX>-20037508.34</UpperLeftX>
    <UpperLeftY>20037508.34</UpperLeftY>
    <LowerRightX>20037508.34</LowerRightX>
    <LowerRightY>-20037508.34</LowerRightY>
    <TileLevel>19</TileLevel>
    <TileCountX>1</TileCountX>
    <TileCountY>1</TileCountY>
    <YOrigin>top</YOrigin>
  </DataWindow>
  <Projection>EPSG:3857</Projection>
  <BlockSizeX>256</BlockSizeX>
  <BlockSizeY>256</BlockSizeY>
  <BandsCount>3</BandsCount>
  <Cache/>
</GDAL_WMS>
EOF

# Fetch imagery as GeoTIFF
=gdal_translate \
  -projwin "${WEST}" "${NORTH}" "${EAST}" "${SOUTH}" \
  -projwin_srs EPSG:4326 \
  -of GTiff \
  -co COMPRESS=JPEG \
  "${XML_FILE}" \
  "${TIFF_FILE}"

# Clear old tiles
if [ -d "${TILES_DIR}" ]; then
  rm -rf "${TILES_DIR}"
fi

# Generate XYZ tiles
gdal2tiles.py \
  --zoom=14-19 \
  --webviewer=none \
  --resampling=average \
  --processes=4 \
  --xyz \
  "${TIFF_FILE}" \
  "${TILES_DIR}"

# Cleanup temp files
rm -f "${XML_FILE}" "${TIFF_FILE}"

# Summary
TILE_COUNT=$(find "${TILES_DIR}" -name "*.png" | wc -l)
TILE_SIZE=$(du -sh "${TILES_DIR}" | cut -f1)

echo ""
echo "=========================================="
echo "  Tiles: ${TILE_COUNT} files"
echo "  Size:  ${TILE_SIZE}"
echo "  Path:  ${TILES_DIR}/"
echo ""
echo "  Add to src/lib/config/locations.ts:"
echo "  {"
echo "    id: '${NAME}',"
echo "    name: '${NAME}',"
echo "    center: [$(python3 -c "print(($EAST + $WEST) / 2)"), $(python3 -c "print(($NORTH + $SOUTH) / 2)")],"
echo "    bounds: [[${WEST}, ${SOUTH}], [${EAST}, ${NORTH}]],"
echo "    tilesPath: '/tiles/${NAME}/{z}/{x}/{y}.png',"
echo "    defaultZoom: 17,"
echo "  }"
echo "=========================================="