file=$1

st-flash --reset  write $file 0x08000000