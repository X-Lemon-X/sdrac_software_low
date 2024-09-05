

devices=$(./find-sdrac-board-usb.sh)

if [ $? -ne 0 ]; then
  echo -e "${RED}No SDRAC board found${NC}"
  exit 1
fi

echo -e "${GREEN}Found SDRAC board(s):${NC}"

i=1
for dev in $devices; do
  echo -e "${BLUE}$i) $dev${NC}"
  i=$((i+1))
done