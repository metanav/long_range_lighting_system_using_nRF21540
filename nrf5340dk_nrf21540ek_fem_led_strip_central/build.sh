#!/bin/sh

rm -rf build && west build -b nrf5340dk_nrf5340_cpuapp -- -DSHIELD=nrf21540_ek -Dhci_rpmsg_SHIELD=nrf21540_ek && west flash
