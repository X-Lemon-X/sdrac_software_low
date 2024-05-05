vendor_id="0483"
product_id="5740"
vendor_enc="SDRACproject"


RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
ORANGE='\033[0;33m'
NC='\033[0m' # No Color

# get all fiels in /dev that contain the bus number
devs=$(ls /dev | grep tty)

device_path=""
#iterate over all devices
for dev in $devs; do
  # get the bus number of the device
  device_info=$(udevadm info /dev/$dev)
  venid=$(echo $device_info | grep -o -E "ID_VENDOR_ID=$vendor_id" | grep -o $vendor_id)
  prodid=$(echo $device_info | grep -o -E "ID_MODEL_ID=$product_id"  | grep -o $product_id)
  venenc=$(echo $device_info | grep -o -E "ID_VENDOR_ENC=$vendor_enc"  | grep -o $vendor_enc)

  if [ -z $venid ] || [ -z $prodid ] || [ -z $venenc ]; then
    continue
  fi

  echo -e "${GREEN}Sdrac-board found on: /dev/$dev${NC}"
  # echo -e "${GREEN}ID: $venid:$prodid  ENC: $venenc${NC}"
  device_path="/dev/$dev"
  break
done

if [ -z $device_path ]; then
  echo -e "${RED}No SDRACboard found${NC}"
  exit 1
fi

echo -e "${BLUE}Flashing USB device: $device_path${NC}"


# open viryal com port and serite SB_enterdfu on boud rate 115200
stty -F /dev/ttyACM0 115200 cs8 -cstopb -parenb raw && echo SB_enterdfu > /dev/ttyACM0
sleep 2
dfu-util -a 0 -i 0 -s 0x08000000:leave -D build/executable.bin 2> /dev/null

if [ $? -ne 0 ]; then
  echo -e "${RED}Error flashing the device${NC}"
  exit 1
else
  echo -e "${ORANGE}Device flashed successfully!${NC}"
fi
exit 0








