file=$1

st-flash --reset write build/firmware.bin 0x08000000