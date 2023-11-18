#!/bin/sh

# Set the input and output file names
INPUT_PATTERN="snap%d.0.png"
OUTPUT_FILE="Project6_no_obstacles.gif"

ffmpeg -y -i $INPUT_PATTERN -vf "fps=30,scale=640:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 $OUTPUT_FILE