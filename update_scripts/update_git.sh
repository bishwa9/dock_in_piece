#!/bin/bash

cp -r ./dji_code/src/navigation_pkg/ ./git_repo/
cp -r ./dji_code/src/dji_sdk/ ./git_repo/
cp -r ./dji_code/src/Guidance_SDK/ ./git_repo/
cp -r ./dji_code/src/april_tag/ ./git_repo/
cp -r ./dji_code/src/usb_cam/ ./git_repo/
cp -r ./dji_code/src/comm_odroid/ ./git_repo/
cp -r ./dji_code/src/state_pkg/ ./git_repo/
cp -r ./dji_code/src/palantir_pkg/ ./git_repo/
cp -r ./*.sh ./git_repo/update_scripts/
#cp -r ./dji_code/*.sh ./git_repo/runMake/ss
cd ./git_repo/
git add -A
git commit
git push origin master
