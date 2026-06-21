#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
for src in test_*.cpp; do
	name="${src%.cpp}"
	g++ -std=c++17 -O2 -Wall -Wextra -o "/tmp/$name" "$src"
	"/tmp/$name"
done
