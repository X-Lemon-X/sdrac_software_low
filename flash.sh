file=$1

./build-firmware.sh

st-flash --reset write build/firmware.bin 0x08000000