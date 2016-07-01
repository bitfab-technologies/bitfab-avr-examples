#!/usr/bin/env sh

avrdude -p atxmega32a4u -c atmelice_pdi -U flash:w:$1

