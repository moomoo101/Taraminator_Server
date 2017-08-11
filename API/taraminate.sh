#!/bin/bash

RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

printf "${YELLOW}Hello, $USER. This script will install Taraminator on your system!\n${NC}"
echo "Checking release version..."
if [[ `lsb_release -rs` != "14.04" ]] # replace 8.04 by the number of release you want
then
printf "${RED}Error! This script is only compatible with an Ubuntu 14.04 system.\n${NC}"
exit 1
fi

cd Tara_SDK_LINUX_REL_package_2.0.4/Source/
chmod +x configure.sh
printf "${YELLOW}Configuring system...\n${NC}"
./configure.sh
printf "${YELLOW}Compiling source...\n${NC}"
./make
printf "${YELLOW}Installing SDK...\n${NC}"
./make install
printf "${YELLOW}Taraminator API installed!\n${NC}"
