file=$1

st-flash --reset write build/executable.bin 0x08000000