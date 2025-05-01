#!/bin/bash

echo "📦 Downloading ROS bag files into 'bags/' folder..."

# Make sure the bags folder exists
mkdir -p ./bags

# Replace the URL below with your actual download link (Box or Google Drive direct download)
wget -O ./bags/bags.zip "https://uofi.box.com/s/n6ql2a3ekzwmghlxoethjm0rbaomhea2
"

echo "📂 Unzipping..."
unzip -o ./bags/bags.zip -d ./bags

echo "✅ Done: Bags downloaded and unzipped into ./bags/"

