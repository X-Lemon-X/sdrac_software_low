vendor_id="0483"
product_id="5740"
vendor_enc="SDRACproject"

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
ORANGE='\033[0;33m'
NC='\033[0m' # No Color


if ! [ -x "$(command -v udevadm)" ]; then
  exit 2
fi

# get all fiels in /dev that contain the bus number
devs=$(ls /dev | grep tty)
list_with_foud_devices=()
for dev in $devs; do
  device_info=$(udevadm info /dev/$dev)
  venid=$(echo $device_info | grep -o -E "ID_VENDOR_ID=$vendor_id" | grep -o $vendor_id)
  prodid=$(echo $device_info | grep -o -E "ID_MODEL_ID=$product_id"  | grep -o $product_id)
  venenc=$(echo $device_info | grep -o -E "ID_VENDOR_ENC=$vendor_enc"  | grep -o $vendor_enc)
  if [ -z $venid ] || [ -z $prodid ] || [ -z $venenc ]; then
    continue
  fi
  list_with_foud_devices=("${list_with_foud_devices[@]}" "/dev/$dev")
done

if [ -z $list_with_foud_devices ]; then
  exit 1
fi

for dev in ${list_with_foud_devices[@]}; do
  echo "$dev"
done
exit 0






