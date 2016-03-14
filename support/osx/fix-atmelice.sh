#!/usr/bin/env sh

echo "RUN WITH ADMIN PRIVILEDGES (SUDO) IN THE OS X SUPPORT DIRECTORY"
echo "REBOOT AFTER RUNNING, AND THEN UNPLUG AND RE-PLUG THE ATMEL ICE"
rm -rf /System/Library/Extensions/AtmelICE-dummy.kext
cp -r AtmelICE-dummy.kext /System/Library/Extensions/
kextcache -system-caches

