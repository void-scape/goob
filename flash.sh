#!/bin/bash
set -e

cargo build --release
avrdude -patmega328p -carduino -P$1 -b115200 -D -Uflash:w:target/avr-none/release/goob.elf:e
stty -F $1 115200 raw cs8 -cstopb -parenb -crtscts -ixon
cat $1
