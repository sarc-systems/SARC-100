#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -Wextra -o /tmp/test_servo_dsp test_servo_dsp.cpp
/tmp/test_servo_dsp
