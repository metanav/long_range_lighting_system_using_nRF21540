#!/bin/sh

rm -rf build && west build -b thingy53_nrf5340_cpuapp  && west flash
