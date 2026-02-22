ICON SETUP INSTRUCTIONS
=======================

This directory contains the app_icon.svg file which serves as the source for all icon formats.

To generate the required icon files:

1. PNG Files (for Qt application and cross-platform use):
   - app_icon_16.png   (16x16 pixels)
   - app_icon_32.png   (32x32 pixels) 
   - app_icon_48.png   (48x48 pixels)
   - app_icon_64.png   (64x64 pixels)
   - app_icon_128.png  (128x128 pixels)
   - app_icon_256.png  (256x256 pixels)
   - app_icon.png      (256x256 pixels, main icon)

2. ICO File (for Windows executable):
   - app_icon.ico      (multi-resolution icon containing 16, 32, 48, 64, 128, 256 sizes)

HOW TO CONVERT:

magick app_icon.svg -resize 16x16 app_icon_16.png
magick app_icon.svg -resize 32x32 app_icon_32.png
magick app_icon.svg -resize 48x48 app_icon_48.png
magick app_icon.svg -resize 64x64 app_icon_64.png
magick app_icon.svg -resize 128x128 app_icon_128.png
magick app_icon.svg -resize 256x256 app_icon_256.png
magick app_icon.svg -resize 256x256 app_icon.png
magick app_icon_16.png app_icon_32.png app_icon_48.png app_icon_64.png app_icon_128.png app_icon_256.png app_icon.ico