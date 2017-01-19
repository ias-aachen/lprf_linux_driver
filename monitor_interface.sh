#!/bin/bash

LPRF_DIR=$(dirname "$0")

pushd ${LPRF_DIR} > /dev/null

if iwpan dev | grep "wpan0" &> /dev/null
then
	echo "Delete wpan0 interface..."
	iwpan dev wpan0 del
fi

if iwpan dev | grep "monitor0" &> /dev/null
then
	echo "Delete monitor0 interface..."
	iwpan dev monitor0 del
fi

PHY=$(iwpan list | grep wpan_phy | cut -d" " -f2)

echo "Add monitor interface..."
iwpan phy $PHY interface add monitor%d type monitor
echo "activate monitor interface..."
ip link set monitor0 up

popd > /dev/null
