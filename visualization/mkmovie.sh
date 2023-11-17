#!/bin/sh

ffmpeg -r 3 -f image2 -i snap%d.png -s 1000x1000 -y Project6_simulation.gif

# convert -delay 50 snap*.png Project6_simulation.gif