#!/bin/bash

cargo build --release && avrdude -patmega328p -carduino -P$1 -b115200 -D -Uflash:w:target/avr-none/release/goob.elf:e
