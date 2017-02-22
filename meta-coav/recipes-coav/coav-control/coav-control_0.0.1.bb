# coav-control.bb
SUMMARY = "Collision avoidance test application."
HOMEPAGE = "https://github.com/01org/collision-avoidance-library"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://LICENSE;md5=93888867ace35ffec2c845ea90b2e16b"

DEPENDS = "glm librealsense python-future"
RDEPENDS_${PN} = "librealsense"

SRCREV = "${AUTOREV}"
SRC_URI = "gitsm://git@github.com/01org/collision-avoidance-library.git;protocol=https;branch=master"

S = "${WORKDIR}/git"

inherit cmake pythonnative

EXTRA_OECMAKE = "-DWITH_TOOLS=ON -DWITH_REALSENSE=ON"

do_configure_prepend() {
    export PYTHONPATH="${PKG_CONFIG_SYSROOT_DIR}/usr/lib/python2.7/site-packages/"
}

do_install_append () {
    install -d ${D}${sysconfdir}/init.d
    install -m 0755 ${B}/tools/coav-control/coav-control.sh ${D}${sysconfdir}/init.d

    install -d ${D}${sysconfdir}/rc1.d
    ln -sf ../init.d/coav-control.sh ${D}${sysconfdir}/rc1.d/S81coav-control
    install -d ${D}${sysconfdir}/rc2.d
    ln -sf ../init.d/coav-control.sh ${D}${sysconfdir}/rc2.d/S81coav-control
    install -d ${D}${sysconfdir}/rc3.d
    ln -sf ../init.d/coav-control.sh ${D}${sysconfdir}/rc3.d/S81coav-control
    install -d ${D}${sysconfdir}/rc4.d
    ln -sf ../init.d/coav-control.sh ${D}${sysconfdir}/rc4.d/S81coav-control
    install -d ${D}${sysconfdir}/rc5.d
    ln -sf ../init.d/coav-control.sh ${D}${sysconfdir}/rc5.d/S81coav-control
}
