# NOT DONE HERE YET

# names
echo -n "Enter camera name: "
read -r name

# echo -n "Enter input video file name: "
# read -r input

# echo -n "Enter desired ffmpeg fps: "
# read -r fps

# # frame dir
# mkdir frames1

# fill up frame directory with png frames
# ffmpeg

# generate corners.vnl
# mrgingham

# ---- CALIBRATE ---- 
evens='frames/*[02468].png'
odds='frames/*[13579].png'
all='frames/*.png'

mrcal_calibrate=(mrcal-calibrate-cameras                                         \
    --corners-cache corners.vnl                                                   \
    --lensmodel LENSMODEL_SPLINED_STEREOGRAPHIC_order=3_Nx=16_Ny=9_fov_x_deg=70   \
    --focal 1015                                                                  \
    --object-spacing 0.012                                                        \
    --object-width-n 14                                                           \
)

# cross diffs
$mrcal_calibrate $evens
mv camera-0.cameramodel $name-evens.cameramodel

$mrcal_calibrate $odds
mv camera-0.cameramodel $name-odds.cameramodel

# main calibration
$mrcal_calibrate $all
mv camera-0.cameramodel $name.cameramodel

# ---- DONE! ---- 
