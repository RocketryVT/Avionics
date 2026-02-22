# PowerShell script to generate icon files from SVG using ImageMagick
# Make sure ImageMagick is installed: https://imagemagick.org/script/download.php#windows

$iconDir = "resources\icons"
$svgFile = "$iconDir\app_icon.svg"

# Check if ImageMagick is available
try {
    magick -version | Out-Null
    Write-Host "ImageMagick found, generating icons..." -ForegroundColor Green
} catch {
    Write-Host "ImageMagick not found. Please install from: https://imagemagick.org/script/download.php#windows" -ForegroundColor Red
    Write-Host "Or use online converters as described in resources\icons\README.txt" -ForegroundColor Yellow
    exit 1
}

# Create PNG files at different sizes
$sizes = @(16, 32, 48, 64, 128, 256)

foreach ($size in $sizes) {
    $outputFile = "$iconDir\app_icon_$size.png"
    Write-Host "Creating $outputFile..." -ForegroundColor Cyan
    magick $svgFile -resize "${size}x${size}" $outputFile
}

# Create main icon (256x256)
Write-Host "Creating main icon..." -ForegroundColor Cyan
magick $svgFile -resize "256x256" "$iconDir\app_icon.png"

# Create ICO file for Windows (multi-resolution)
Write-Host "Creating Windows ICO file..." -ForegroundColor Cyan
magick "$iconDir\app_icon_16.png" "$iconDir\app_icon_32.png" "$iconDir\app_icon_48.png" "$iconDir\app_icon_64.png" "$iconDir\app_icon_128.png" "$iconDir\app_icon_256.png" "$iconDir\app_icon.ico"

# For macOS, create ICNS file (if iconutil is available - Mac only)
if (Get-Command "iconutil" -ErrorAction SilentlyContinue) {
    Write-Host "Creating macOS ICNS file..." -ForegroundColor Cyan
    # This would only work on macOS, so skip for now
} else {
    Write-Host "ICNS creation skipped (macOS iconutil not available)" -ForegroundColor Yellow
    Write-Host "For macOS: Convert app_icon.png to ICNS using online tools or Xcode" -ForegroundColor Yellow
}

Write-Host "`nIcon generation complete!" -ForegroundColor Green
Write-Host "Files created in $iconDir:" -ForegroundColor White
Get-ChildItem $iconDir -Filter "*.png" | ForEach-Object { Write-Host "  - $($_.Name)" -ForegroundColor Gray }
Get-ChildItem $iconDir -Filter "*.ico" | ForEach-Object { Write-Host "  - $($_.Name)" -ForegroundColor Gray }
Write-Host "`nNow rebuild your project to embed the icons!" -ForegroundColor Green
