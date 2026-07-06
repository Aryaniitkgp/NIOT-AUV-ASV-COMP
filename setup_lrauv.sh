#!/usr/bin/env bash
# Downloads Gazebo's official tutorial AUV ("my_lrauv", based on the MBARI
# Tethys vehicle) with propeller + fin joints, ready to drop into gz-sim 8.
# Source: https://gazebosim.org/api/sim/8/underwater_vehicles.html
set -e

BASE="https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/underwater_vehicles/my_lrauv"
DEST="$(pwd)/models/my_lrauv"

mkdir -p "$DEST/meshes" "$DEST/materials/textures"

echo "Downloading model.sdf ..."
wget -q "$BASE/model.sdf" -O "$DEST/model.sdf"

echo "Downloading meshes ..."
wget -q "$BASE/meshes/tethys.dae"    -O "$DEST/meshes/tethys.dae"
wget -q "$BASE/meshes/propeller.dae" -O "$DEST/meshes/propeller.dae"

echo "Downloading textures ..."
wget -q "$BASE/materials/textures/Tethys_Albedo.png"    -O "$DEST/materials/textures/Tethys_Albedo.png"
wget -q "$BASE/materials/textures/Tethys_Metalness.png" -O "$DEST/materials/textures/Tethys_Metalness.png"
wget -q "$BASE/materials/textures/Tethys_Normal.png"    -O "$DEST/materials/textures/Tethys_Normal.png"
wget -q "$BASE/materials/textures/Tethys_Roughness.png" -O "$DEST/materials/textures/Tethys_Roughness.png"

echo ""
echo "Done. Model installed at: $DEST"
echo ""
echo "Now run:"
echo "  export GZ_SIM_RESOURCE_PATH=\$(pwd)/models:\$GZ_SIM_RESOURCE_PATH"
echo "  gz sim save_arena.sdf"
echo ""
echo "(Make sure save_arena.sdf sits in the same folder you ran this script from,"
echo " or adjust GZ_SIM_RESOURCE_PATH to point at wherever 'models/' ended up.)"
