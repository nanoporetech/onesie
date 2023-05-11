#! /bin/bash -e

function make_lintian_override_if_needed() {
    local package_name="$1"

    # The overrides file is needed due to a reported error about virtual
    # packages specified with a version number. This is thought to be a bug in
    # Lintian, later version of Lintian only complain if the version is
    # specified with a greater-than or less-than.
    if dpkg --compare-versions "$(lintian --version | grep -Eo '[0-9]+\.[0-9]+\.[0-9]+.*$')" "gt" "2.6.0"
    then
        return
    fi

    sed -e "s#PACKAGE#${package_name}#g" \
        "${PWD}/debian/lintian-overrides" > "${PWD}/package/debian/${package_name}.lintian-overrides"
}

# expect to be run from the base directory
if [ "$0" != "debian/debian_packaging.sh" ]; then
    echo "expect to be run from the base directory of the onesie project"
    exit 1
fi

# process arguments
while [ -n "$1" ] ;do
    case "$1" in
    "--version")
        VERSION="$2"
        shift 2
        ;;
    "--debian-revision")
        DEBIAN_REVISION="$2"
        shift 2
        ;;
    "--version-suffix")
        VERSION_SUFFIX="$2"
        shift 2
        ;;
    "--kernel")
        KVERS="$2"
        shift 2
        ;;
    "--firmware")
        FIRMWARE_VERSION="$2"
        shift 2
        ;;
    "--architecture")
        DEB_ARCH="$2"
        shift 2
        ;;
    "--package-base-name")
        PACKAGE_BASE_NAME="$2"
        shift 2
        ;;
    "--compiled-module")
        COMPILED_DRIVER_PACKAGE=1
        shift
        ;;
    *)
        echo "Did not recognise option $1"
        exit 1
    esac
done

# assemble all the files under the directory "package/"
rm -rf package
mkdir -p package/debian

# make the .deb control file and change kernel version number
cp debian/control package/debian/control
if [ -n "${COMPILED_DRIVER_PACKAGE}" ]; then
    sed -e "s/_KVERS_/${KVERS}/g;s/_VERSION_/${VERSION}/g;s/_FIRMWARE-VERSION_/${FIRMWARE_VERSION}/g" debian/control.modules.in >> package/debian/control
    sed -e "s/_KVERS_/${KVERS}/g" debian/postinst.modules.in > "package/debian/ont-minion1c-driver-${KVERS}.postinst"
    sed -e "s/_KVERS_/${KVERS}/g" debian/postrm.modules.in > "package/debian/ont-minion1c-driver-${KVERS}.postrm"
fi
sed -i -e "s/_ARCH_/${DEB_ARCH}/g;s/_VERSION_/${VERSION}/g;s/_FIRMWARE-VERSION_/${FIRMWARE_VERSION}/g" package/debian/control
# debhelper version-9
echo 9 > package/debian/compat
echo "VERSION_SUFFIX=${VERSION_SUFFIX}"

# Create a changelog. (Specialise the template.)
cp debian/changelog.template package/debian/changelog
sed -e "s/@NAME@/${PACKAGE_BASE_NAME}/g" -i package/debian/changelog
sed -e "s/@VERSION@/${VERSION}-${DEBIAN_REVISION}${VERSION_SUFFIX}/g" -i package/debian/changelog
sed -e "s/@DATE@/$(date -R)/g" -i package/debian/changelog

sed -e "s/@DATE@/$(date +%Y)/g" debian/utils.copyright.template > "package/debian/${PACKAGE_BASE_NAME}-utils.copyright"
sed -e "s/@DATE@/$(date +%Y)/g" debian/other.copyright.template > "package/debian/copyright"

# Create lintian-overrides for the package
make_lintian_override_if_needed "${PACKAGE_BASE_NAME}-dkms"

# copy the source and packaging information, make the source package
mkdir "${PACKAGE_BASE_NAME}-${VERSION}"
cp -r package/debian "${PACKAGE_BASE_NAME}-${VERSION}"
cp -r driver utils Makefile "${PACKAGE_BASE_NAME}-${VERSION}"
dpkg-source --build "${PACKAGE_BASE_NAME}-${VERSION}"
rm -r "${PACKAGE_BASE_NAME}-${VERSION}"

# cleanup
(cd package && fakeroot dh_prep)

# if this is a binary then make and add driver object file.
if [ -n "${COMPILED_DRIVER_PACKAGE}" ]; then
    # Create lintian-overrides for the package
    make_lintian_override_if_needed "${PACKAGE_BASE_NAME}-${KVERS}"
    make -C driver DESTDIR="${PWD}/package/debian/${PACKAGE_BASE_NAME}-${KVERS}" PREFIX=/usr install-modules
fi
# add driver-includes, add source files for DKMS
make -C driver DESTDIR="${PWD}/package/debian/${PACKAGE_BASE_NAME}-dev" PREFIX=/usr install-dev
make -C driver distdir="${PWD}/package/debian/${PACKAGE_BASE_NAME}-dkms/usr/src/${PACKAGE_BASE_NAME}-${VERSION}" dist
make -C udev DESTDIR="${PWD}/package/debian/${PACKAGE_BASE_NAME}-udev" install
make -C utils VERSION="${VERSION}" DESTDIR="${PWD}/package/debian/${PACKAGE_BASE_NAME}-utils" PREFIX=/usr install
# change the DKMS version to match the driver
sed -e "s/_VERSION_/${VERSION}/g" debian/dkms.conf.in > "package/debian/${PACKAGE_BASE_NAME}-dkms.dkms"

cd package
# generate .deb files to install binary driver
if [ -n "${COMPILED_DRIVER_PACKAGE}" ]; then
    fakeroot dh_installmodules
fi
# generate .deb files for the rest
fakeroot dh_strip --no-automatic-dbgsym
fakeroot dh_lintian
fakeroot dh_installdocs
fakeroot dh_installchangelogs
fakeroot dh_dkms -p "${PACKAGE_BASE_NAME}-dkms"
fakeroot dh_makeshlibs
fakeroot dh_shlibdeps
fakeroot dh_installdeb
fakeroot dh_gencontrol
fakeroot dh_compress
fakeroot dh_md5sums
fakeroot dh_builddeb
cd ..
#rm -r package
