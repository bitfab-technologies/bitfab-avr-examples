#!/usr/bin/env sh

avrdude -p at90can128 -c atmelice_isp -U flash:w:$1

