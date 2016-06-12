#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import cStringIO
import logging
import os
import platform
import shutil
import subprocess
import sys
import urllib2
import zipfile


logging.basicConfig(level=logging.INFO)

# global variables
VERSION="1.39"
if platform.machine() == "x86_64":
    ARCH="amd64"
elif platform.machine() == "i386":
    ARCH="i386"
else:
    logging.fatal("architecture {arch} is not supported.".format(arch=platform.machine()))
    sys.exit(1)
DRIVER_URL="http://www.mvision.co.jp/DL/net3/3iCube_Linux_{version}.zip".format(version=VERSION)
DEB_PATH="3iCube_Linux_{version}/03_Driver/netusbcam_{version}-1_{arch}_libudev.deb".format(version=VERSION,
                                                                                            arch=ARCH)
SHARED_LIB_PATH="usr/lib/libNETUSBCAM.so.0.0.0"
INCLUDE_DIR_PATH="usr/include"
LIBRARY_DESTINATION = sys.argv[1]
INCLUDE_DIR_DESTINATION = sys.argv[2]

logging.info("Downloading driver")
res = urllib2.urlopen(DRIVER_URL)
if res.code != 200:
    logging.fatal("failed to download from {url}: return code: {code}, msg: {msg}".format(url=res.url,
                                                                                          code=res.code,
                                                                                          msg=res.msg))
    sys.exit(1)

logging.info("Unarchiving tarball")
with zipfile.ZipFile(cStringIO.StringIO(res.read())) as z:
    z.extractall()
subprocess.check_call(['dpkg', '--extract', DEB_PATH, '.'])

logging.info("Copying library")
shutil.copyfile(SHARED_LIB_PATH, LIBRARY_DESTINATION)

logging.info("Copying header files")
for f in os.listdir(INCLUDE_DIR_PATH):
    if f.endswith(".h") or f.endswith(".hpp") or f.endswith(".hh"):
        if not os.path.exists(INCLUDE_DIR_DESTINATION):
            os.makedirs(INCLUDE_DIR_DESTINATION)
        shutil.copyfile(os.path.join(INCLUDE_DIR_PATH, f),
                        os.path.join(INCLUDE_DIR_DESTINATION, f))
