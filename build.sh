#!/usr/bin/env sh
set -eu

cd "$(dirname "$0")"

CMAKE="/d/msys64/ucrt64/bin/cmake.exe"
if [ ! -x "$CMAKE" ]; then
  CMAKE="/D/msys64/ucrt64/bin/cmake.exe"
fi

echo "Configuring Debug..."
"$CMAKE" --preset ucrt64-debug

echo "Building Debug..."
"$CMAKE" --build --preset ucrt64-debug

echo "Configuring Release..."
"$CMAKE" --preset ucrt64-release

echo "Building Release..."
"$CMAKE" --build --preset ucrt64-release

echo "Build completed."
echo "Debug:   build/ucrt64-debug/heyball.exe"
echo "Release: build/ucrt64-release/heyball.exe"
