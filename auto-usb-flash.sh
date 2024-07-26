vendor_id="0483"
product_id="5740"
vendor_enc="SDRACproject"

bin_file="build/firmware.bin"


RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
ORANGE='\033[0;33m'
NC='\033[0m' # No Color

function install_program () {
  if ! [ -x "$(command -v $1)" ]; then
    echo -e "${ORANGE}Installing $1${NC}"
    sudo apt-get install $1 -y
  else
    echo -e "${BLUE}$1 is installed${NC}"
  fi
}

function check_if_program_is_installed () {
  if ! [ -x "$(command -v $1)" ]; then
    echo -e "${RED}Error: $1 is not installed.${NC}"
    #ask user if he wants to install the program
    read -p "Do you want to install $1? (y/n)" -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
      install_program $1
    else
      echo -e "${ORANGE}Exiting...${NC}"
      exit 1
    fi
  else
    echo -e "${BLUE}$1 is installed${NC}"
  fi
}

function check_if_file_exist () {
  if [ ! -f $1 ]; then
    echo -e "${RED}file: $1 not found.${NC}" >&2
    echo 1
  else
    echo -e "${BLUE}file found $1 ${NC}" >&2
    echo 0
  fi
}

# send command to device to first arg is device path second is command
function send_command_to_device () {
  stty -F $1 115200 cs8 -cstopb -parenb raw && echo $2 > $1
}

#check if required programs are installed
check_if_program_is_installed "dfu-util"
check_if_program_is_installed "udevadm"
check_if_program_is_installed "stty"

#check if arguments are provided
if [ $# -eq 1 ]; then
  echo -e "${ORANGE}given custom .bin file path${NC}"
  bin_file=$1
fi
exist=$(check_if_file_exist $bin_file)

if [ $exist -ne 0 ]; then
  exit 1
fi


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


# send_command_to_device $device_path "SB_info"

echo -e "${BLUE}Flashing USB device: $device_path${NC}"


# open viryal com port and write SB_enterdfu on boud rate 115200
# stty -F $device_path 115200 cs8 -cstopb -parenb raw && echo SB_enterdfu > $device_path
send_command_to_device $device_path SB_enterdfu
# send_command_to_device $device_path SB_info 

sleep 2 #hive board some time to enter dfu mode
dfu-util -a 0 -i 0 -s 0x08000000:leave -D $bin_file 2> /dev/null

if [ $? -ne 0 ]; then
  echo -e "${RED}Error flashing the device${NC}"
  exit 1
else
  echo -e "${ORANGE}Device flashed successfully!${NC}"
fi
exit 0








