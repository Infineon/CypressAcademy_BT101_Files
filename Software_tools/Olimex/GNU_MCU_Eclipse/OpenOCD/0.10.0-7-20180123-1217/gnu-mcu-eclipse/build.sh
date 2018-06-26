#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Safety settings (see https://gist.github.com/ilg-ul/383869cbb01f61a51c4d).

if [[ ! -z ${DEBUG} ]]
then
  set ${DEBUG} # Activate the expand mode if DEBUG is -x.
else
  DEBUG=""
fi

set -o errexit # Exit if command failed.
set -o pipefail # Exit if pipe failed.
set -o nounset # Exit if variable not set.

# Remove the initial space and instead use '\n'.
IFS=$'\n\t'

# -----------------------------------------------------------------------------

# Script to build the GNU MCU Eclipse OpenOCD distribution packages.
#
# Developed on OS X 10.12 Sierra.
# Also tested on:
#   GNU/Linux Arch (Manjaro 16.08)
#
# The Windows and GNU/Linux packages are build using Docker containers.
# The build is structured in 2 steps, one running on the host machine
# and one running inside the Docker container.
#
# At first run, Docker will download/build 3 relatively large
# images (1-2GB) from Docker Hub.
#
# Prerequisites:
#
#   Docker
#   curl, git, automake, patch, tar, unzip, zip
#
# When running on OS X, a custom Homebrew is required to provide the 
# missing libraries and TeX binaries.
# 
# Without specifying a --xxx platform option, builds are assumed
# to be native on the GNU/Linux or macOS host.
# In this case all prerequisites must be met by the host. For example 
# for Ubuntu the following were needed:
#
# $ apt-get install -y automake cmake libtool libudev-dev patchelf texinfo texlive
#
# To resume a crashed build with the same timestamp, set
# DISTRIBUTION_FILE_DATE='yyyymmdd-HHMM' in the environment.
#
# To build in a custom folder, set WORK_FOLDER_PATH='xyz' 
# in the environment.
#
# Configuration environment variables:
#
# - WORK_FOLDER_PATH
# - DISTRIBUTION_FILE_DATE
# - APP_NAME
# - APP_UC_NAME
# - APP_LC_NAME
# - BUILD_FOLDER_NAME
# - BUILD_FOLDER_PATH
# - DOWNLOAD_FOLDER_NAME
# - DOWNLOAD_FOLDER_PATH
# - DEPLOY_FOLDER_NAME
# - OPENOCD_FOLDER_NAME
# - OPENOCD_GIT_URL
# - OPENOCD_GIT_BRANCH
# - OPENOCD_GIT_COMMIT
#

# -----------------------------------------------------------------------------

# Mandatory definition.
APP_NAME=${APP_NAME:-"OpenOCD"}

# Used as part of file/folder paths.
APP_UC_NAME=${APP_UC_NAME:-"OpenOCD"}
APP_LC_NAME=${APP_LC_NAME:-"openocd"}

jobs="-j2"

# On Parallels virtual machines, prefer host Work folder.
# Second choice are Work folders on secondary disks.
# Final choice is a Work folder in HOME.
if [ -d /media/psf/Home/Work ]
then
  WORK_FOLDER_PATH=${WORK_FOLDER_PATH:-"/media/psf/Home/Work/${APP_LC_NAME}"}
elif [ -d /media/${USER}/Work ]
then
  WORK_FOLDER_PATH=${WORK_FOLDER_PATH:-"/media/${USER}/Work/${APP_LC_NAME}"}
elif [ -d /media/Work ]
then
  WORK_FOLDER_PATH=${WORK_FOLDER_PATH:-"/media/Work/${APP_LC_NAME}"}
else
  # Final choice, a Work folder in HOME.
  WORK_FOLDER_PATH=${WORK_FOLDER_PATH:-"${HOME}/Work/${APP_LC_NAME}"}
fi

# ----- Define build constants. -----

BUILD_FOLDER_NAME=${BUILD_FOLDER_NAME:-"build"}
BUILD_FOLDER_PATH=${BUILD_FOLDER_PATH:-"${WORK_FOLDER_PATH}/${BUILD_FOLDER_NAME}"}

DOWNLOAD_FOLDER_NAME=${DOWNLOAD_FOLDER_NAME:-"download"}
DOWNLOAD_FOLDER_PATH=${DOWNLOAD_FOLDER_PATH:-"${WORK_FOLDER_PATH}/${DOWNLOAD_FOLDER_NAME}"}
DEPLOY_FOLDER_NAME=${DEPLOY_FOLDER_NAME:-"deploy"}

# ----- Define build Git constants. -----

PROJECT_GIT_FOLDER_NAME="openocd-build.git"
PROJECT_GIT_FOLDER_PATH="${WORK_FOLDER_PATH}/${PROJECT_GIT_FOLDER_NAME}"
PROJECT_GIT_DOWNLOADS_FOLDER_PATH="${HOME}/Downloads/${PROJECT_GIT_FOLDER_NAME}"
PROEJCT_GIT_URL="https://github.com/gnu-mcu-eclipse/${PROJECT_GIT_FOLDER_NAME}"

# ----- Docker images. -----

docker_linux64_image="ilegeul/centos:6-xbb-v1"
docker_linux32_image="ilegeul/centos32:6-xbb-v1"

# ----- Create Work folder. -----

echo
echo "Work folder: \"${WORK_FOLDER_PATH}\"."

mkdir -p "${WORK_FOLDER_PATH}"

# ----- Parse actions and command line options. -----

ACTION=""
DO_BUILD_WIN32=""
DO_BUILD_WIN64=""
DO_BUILD_LINUX32=""
DO_BUILD_LINUX64=""
DO_BUILD_OSX=""
helper_script_path=""
do_no_strip=""
do_no_pdf=""
jobs=""
do_develop=""
do_debug=""

while [ $# -gt 0 ]
do
  case "$1" in

    clean|cleanall|preload-images|bootstrap)
      ACTION="$1"
      shift
      ;;

    --win32|--window32)
      DO_BUILD_WIN32="y"
      shift
      ;;
    --win64|--windows64)
      DO_BUILD_WIN64="y"
      shift
      ;;
    --linux32|--deb32|--debian32)
      DO_BUILD_LINUX32="y"
      shift
      ;;
    --linux64|--deb64|--debian64)
      DO_BUILD_LINUX64="y"
      shift
      ;;
    --osx)
      DO_BUILD_OSX="y"
      shift
      ;;

    --all)
      DO_BUILD_WIN32="y"
      DO_BUILD_WIN64="y"
      DO_BUILD_LINUX32="y"
      DO_BUILD_LINUX64="y"
      DO_BUILD_OSX="y"
      shift
      ;;

    --helper-script)
      helper_script_path=$2
      shift 2
      ;;

    --no-strip)
      do_no_strip="y"
      shift
      ;;

    --without-pdf|--no-pdf)
      do_no_pdf="y"
      shift
      ;;

    --jobs)
      jobs="--jobs=$2"
      shift 2
      ;;

    --develop)
      do_develop="y"
      shift
      ;;

    --debug)
      do_debug="y"
      shift
      ;;

    --help)
      echo "Build the GNU MCU Eclipse ${APP_NAME} distributions."
      echo "Usage:"
      echo "    bash $0 helper_script [--win32] [--win64] [--linux32] [--linux64] [--osx] [--all] [clean|cleanall|preload-images|bootstrap] [--no-strip] [--without-pdf] [--develop] [--debug] [--help]"
      echo
      exit 1
      ;;

    *)
      echo "Unknown action/option $1"
      exit 1
      ;;
  esac

done

# ----- Prepare build scripts. -----

build_script_path=$0
if [[ "${build_script_path}" != /* ]]
then
  # Make relative path absolute.
  build_script_path=$(pwd)/$0
fi

# Copy the current script to Work area, to later copy it into the install folder.
mkdir -p "${WORK_FOLDER_PATH}/scripts"
cp "${build_script_path}" "${WORK_FOLDER_PATH}/scripts/build-${APP_LC_NAME}.sh"

# ----- Build helper. -----

if [ -z "${helper_script_path}" ]
then
  script_folder_path="$(dirname ${build_script_path})"
  script_folder_name="$(basename ${script_folder_path})"
  if [ \( "${script_folder_name}" == "scripts" \) \
    -a \( -f "${script_folder_path}/helper/build-helper.sh" \) ]
  then
    helper_script_path="${script_folder_path}/helper/build-helper.sh"
  elif [ \( "${script_folder_name}" == "scripts" \) \
    -a \( -d "${script_folder_path}/helper" \) ]
  then
    (
      cd "$(dirname ${script_folder_path})"
      git submodule update --init --recursive --remote
    )
    helper_script_path="${script_folder_path}/helper/build-helper.sh"
  elif [ -f "${WORK_FOLDER_PATH}/scripts/build-helper.sh" ]
  then
    helper_script_path="${WORK_FOLDER_PATH}/scripts/build-helper.sh"
  fi
else
  if [[ "${helper_script_path}" != /* ]]
  then
    # Make relative path absolute.
    helper_script_path="$(pwd)/${helper_script_path}"
  fi
fi

# Copy the current helper script to Work area, to later copy it into 
# the install folder.
mkdir -p "${WORK_FOLDER_PATH}/scripts"
if [ "${helper_script_path}" != "${WORK_FOLDER_PATH}/scripts/build-helper.sh" ]
then
  cp "${helper_script_path}" "${WORK_FOLDER_PATH}/scripts/build-helper.sh"
fi

echo "Helper script: \"${helper_script_path}\"."
source "${helper_script_path}"


# ----- Libraries sources. -----

# For updates, please check the corresponding pages.

# The custom OpenOCD branch is available from the dedicated Git repository
# which is part of the GNU MCU Eclipse project hosted on GitHub.
# Generally this branch follows the official OpenOCD master branch,
# with updates after every OpenOCD public release.

OPENOCD_FOLDER_NAME="${OPENOCD_FOLDER_NAME:-openocd.git}"
OPENOCD_GIT_URL="${OPENOCD_GIT_URL:-https://github.com/gnu-mcu-eclipse/openocd.git}"
# OPENOCD_GIT_BRANCH="${OPENOCD_GIT_BRANCH:-gnu-mcu-eclipse}"
OPENOCD_GIT_BRANCH="${OPENOCD_GIT_BRANCH:-gnu-mcu-eclipse-dev}"
# OPENOCD_GIT_COMMIT="HEAD"
OPENOCD_GIT_COMMIT="20463c28affea880d167b000192785a48f8974ca"

# Since some of the original URLs are occasionaly unavailable,
# the archives were re-published in a dedicated GitHub project:
# https://github.com/gnu-mcu-eclipse/files.

# https://sourceforge.net/projects/libusb/files/libusb-1.0/
# 1.0.20 from 2015-09-14
LIBUSB1_VERSION="1.0.20"
LIBUSB1_FOLDER="libusb-${LIBUSB1_VERSION}"
LIBUSB1="${LIBUSB1_FOLDER}"
LIBUSB1_ARCHIVE="${LIBUSB1}.tar.bz2"
# LIBUSB1_URL="http://sourceforge.net/projects/libusb/files/libusb-1.0/${LIBUSB1_FOLDER}/${LIBUSB1_ARCHIVE}"
LIBUSB1_URL="https://github.com/gnu-mcu-eclipse/files/raw/master/libs/${LIBUSB1_ARCHIVE}"

# https://sourceforge.net/projects/libusb/files/libusb-compat-0.1/
# 0.1.5 from 2013-05-21
LIBUSB0_VERSION="0.1.5"
LIBUSB0_FOLDER="libusb-compat-${LIBUSB0_VERSION}"
LIBUSB0="${LIBUSB0_FOLDER}"
LIBUSB0_ARCHIVE="${LIBUSB0_FOLDER}.tar.bz2"
# LIBUSB0_URL="http://sourceforge.net/projects/libusb/files/libusb-compat-0.1/${LIBUSB0_FOLDER}/${LIBUSB0_ARCHIVE}"
LIBUSB0_URL="https://github.com/gnu-mcu-eclipse/files/raw/master/libs/${LIBUSB0_ARCHIVE}"

# https://sourceforge.net/projects/libusb-win32/files/libusb-win32-releases/
# 1.2.6.0 from 2012-01-17
LIBUSB_W32_PREFIX="libusb-win32"
LIBUSB_W32_VERSION="1.2.6.0"
LIBUSB_W32="${LIBUSB_W32_PREFIX}-${LIBUSB_W32_VERSION}"
LIBUSB_W32_FOLDER="${LIBUSB_W32_PREFIX}-src-${LIBUSB_W32_VERSION}"
LIBUSB_W32_ARCHIVE="${LIBUSB_W32_FOLDER}.zip"
# LIBUSB_W32_URL="http://sourceforge.net/projects/libusb-win32/files/libusb-win32-releases/${LIBUSB_W32_VERSION}/${LIBUSB_W32_ARCHIVE}"
LIBUSB_W32_URL="https://github.com/gnu-mcu-eclipse/files/raw/master/libs/${LIBUSB_W32_ARCHIVE}"

# http://www.intra2net.com/en/developer/libftdi/download.php
# 1.2 (no date)
LIBFTDI_VERSION="1.2"
LIBFTDI_FOLDER="libftdi1-${LIBFTDI_VERSION}"
LIBFTDI_ARCHIVE="${LIBFTDI_FOLDER}.tar.bz2"
LIBFTDI="${LIBFTDI_FOLDER}"
# LIBFTDI_URL="http://www.intra2net.com/en/developer/libftdi/download/${LIBFTDI_ARCHIVE}"
LIBFTDI_URL="https://github.com/gnu-mcu-eclipse/files/raw/master/libs/${LIBFTDI_ARCHIVE}"

# https://www.gnu.org/software/libiconv/
# https://ftp.gnu.org/pub/gnu/libiconv/
# 2017-02-02
LIBICONV_VERSION="1.15"
LIBICONV_FOLDER="libiconv-${LIBICONV_VERSION}"
LIBICONV="libiconv-${LIBICONV_VERSION}"
LIBICONV_ARCHIVE="${LIBICONV}.tar.gz"
LIBICONV_URL="https://ftp.gnu.org/pub/gnu/libiconv/${LIBICONV_ARCHIVE}"

# https://github.com/signal11/hidapi/downloads
# Oct 26, 2011
# HIDAPI_VERSION="0.7.0"

# https://github.com/signal11/hidapi/archive/hidapi-0.8.0-rc1.zip
# Oct 7, 2013

HIDAPI_VERSION="0.8.0-rc1"
HIDAPI_FOLDER="hidapi-hidapi-${HIDAPI_VERSION}"
HIDAPI="hidapi-${HIDAPI_VERSION}"
HIDAPI_ARCHIVE="${HIDAPI}.zip"
# HIDAPI_URL="https://github.com/signal11/hidapi/archive/${HIDAPI_ARCHIVE}"
HIDAPI_URL="https://github.com/gnu-mcu-eclipse/files/raw/master/libs/${HIDAPI_ARCHIVE}"


# ----- Process actions. -----

if [ \( "${ACTION}" == "clean" \) -o \( "${ACTION}" == "cleanall" \) ]
then
  # Remove most build and temporary folders.
  echo
  if [ "${ACTION}" == "cleanall" ]
  then
    echo "Remove all the build folders..."
  else
    echo "Remove most of the build folders (except output)..."
  fi

  rm -rf "${BUILD_FOLDER_PATH}"
  rm -rf "${WORK_FOLDER_PATH}/install"

  rm -rf "${WORK_FOLDER_PATH}/${LIBUSB1_FOLDER}"
  rm -rf "${WORK_FOLDER_PATH}/${LIBUSB0_FOLDER}"
  rm -rf "${WORK_FOLDER_PATH}/${LIBUSB_W32_FOLDER}"
  rm -rf "${WORK_FOLDER_PATH}/${LIBFTDI_FOLDER}"
  rm -rf "${WORK_FOLDER_PATH}/${HIDAPI_FOLDER}"
  rm -rf "${WORK_FOLDER_PATH}/${LIBICONV_FOLDER}"

  rm -rf "${WORK_FOLDER_PATH}/scripts"

  if [ "${ACTION}" == "cleanall" ]
  then
    rm -rf "${PROJECT_GIT_FOLDER_PATH}"
    rm -rf "${WORK_FOLDER_PATH}/${OPENOCD_FOLDER_NAME}"
    rm -rf "${WORK_FOLDER_PATH}/${DEPLOY_FOLDER_NAME}"
  fi

  echo
  echo "Clean completed. Proceed with a regular build."

  exit 0
fi

# ----- Start build. -----

do_host_start_timer

do_host_detect

# ----- Prepare prerequisites. -----

do_host_prepare_prerequisites

# ----- Process "preload-images" action. -----

if [ "${ACTION}" == "preload-images" ]
then
  do_host_prepare_docker

  echo
  echo "Check/Preload Docker images..."

  echo
  docker run --interactive --tty ${docker_linux64_image} \
    lsb_release --description --short

  echo
  docker run --interactive --tty ${docker_linux32_image} \
    lsb_release --description --short

  echo
  docker images

  do_host_stop_timer

  exit 0
fi

do_host_bootstrap() {

  # Prepare autotools.
  echo
  echo "bootstrap..."

  cd "${WORK_FOLDER_PATH}/${OPENOCD_FOLDER_NAME}"
  rm -f aclocal.m4
  ./bootstrap

}

if [ \( "${ACTION}" == "bootstrap" \) ]
then

  do_host_bootstrap

  do_host_stop_timer

  exit 0

fi

# ----- Prepare Docker, if needed. -----

if [ -n "${DO_BUILD_WIN32}${DO_BUILD_WIN64}${DO_BUILD_LINUX32}${DO_BUILD_LINUX64}" ]
then
  do_host_prepare_docker
fi

# ----- Check some more prerequisites. -----

# Used by bootstrap.
echo
echo "Checking host automake..."
automake --version 2>/dev/null | grep automake

echo
echo "Checking host patch..."
patch --version | grep patch

echo
echo "Checking host tar..."
tar --version

echo
echo "Checking host unzip..."
unzip | grep UnZip

if [ "${HOST_UNAME}" == "Darwin" ]
then

  echo
  echo "Checking host makeinfo..."
  makeinfo --version | grep 'GNU texinfo'
  makeinfo_ver=$(makeinfo --version | grep 'GNU texinfo' | sed -e 's/.*) //' -e 's/\..*//')
  if [ "${makeinfo_ver}" -lt "6" ]
  then
    echo "makeinfo too old, abort."
    exit 1
  fi

  if which libtoolize > /dev/null; then
      libtoolize="libtoolize"
  elif which glibtoolize >/dev/null; then
      libtoolize="glibtoolize"
  else
      echo "$0: Error: libtool is required" >&2
      exit 1
  fi

fi

# ----- Get the project git repository. -----

if [ -d "${PROJECT_GIT_DOWNLOADS_FOLDER_PATH}" ]
then

  # If the folder is already present in Downloads, copy it.
  echo "Copying ${PROJECT_GIT_FOLDER_NAME} from Downloads..."
  rm -rf "${PROJECT_GIT_FOLDER_PATH}"
  mkdir -p "${PROJECT_GIT_FOLDER_PATH}"
  git clone "${PROJECT_GIT_DOWNLOADS_FOLDER_PATH}" "${PROJECT_GIT_FOLDER_PATH}"

else

  if [ ! -d "${PROJECT_GIT_FOLDER_PATH}" ]
  then

    cd "${WORK_FOLDER_PATH}"

    echo "If asked, enter ${USER} GitHub password for git clone"
    git clone "${PROJECT_GIT_URL}" "${PROJECT_GIT_FOLDER_PATH}"

  fi

fi

# ----- Get current date. -----

# Use the UTC date as version in the name of the distribution file.
do_host_get_current_date

# ----- Get OPENOCD. -----

if [ ! -d "${WORK_FOLDER_PATH}/${OPENOCD_FOLDER_NAME}" ]
then

  echo
  echo "Cloning '${OPENOCD_GIT_URL}'..."

  cd "${WORK_FOLDER_PATH}"
  git clone --branch "${OPENOCD_GIT_BRANCH}" "${OPENOCD_GIT_URL}" \
    "${OPENOCD_FOLDER_NAME}"
  
  cd "${OPENOCD_FOLDER_NAME}"
  git checkout -qf "${OPENOCD_GIT_COMMIT}"

  git submodule update --init --recursive --remote

  git branch

fi

# ----- Bootstrap. -----

# If 'configure' is not yet there, make sure it is generated.
if [ ! -f "${WORK_FOLDER_PATH}/${OPENOCD_FOLDER_NAME}/configure" ]
then
  echo
  echo "Creating the automake files..."

  do_host_bootstrap
fi

# ----- Get the USB libraries. -----

# Both USB libraries are available from a single project LIBUSB
# 	http://www.libusb.info
# with source files ready to download from SourceForge
# 	https://sourceforge.net/projects/libusb/files

# Download the new USB library.
if [ ! -f "${DOWNLOAD_FOLDER_PATH}/${LIBUSB1_ARCHIVE}" ]
then
  mkdir -p "${DOWNLOAD_FOLDER_PATH}"

  echo
  echo "Downloading \"${LIBUSB1_ARCHIVE}\"..."

  cd "${DOWNLOAD_FOLDER_PATH}"
  curl -L "${LIBUSB1_URL}" --output "${LIBUSB1_ARCHIVE}"
fi

# Unpack the new USB library.
if [ ! -d "${WORK_FOLDER_PATH}/${LIBUSB1_FOLDER}" ]
then
  cd "${WORK_FOLDER_PATH}"
  tar -xjvf "${DOWNLOAD_FOLDER_PATH}/${LIBUSB1_ARCHIVE}"
fi

# http://www.libusb.org

# Download the old USB library.
if [ ! -f "${DOWNLOAD_FOLDER_PATH}/${LIBUSB0_ARCHIVE}" ]
then
  mkdir -p "${DOWNLOAD_FOLDER_PATH}"

  echo
  echo "Downloading \"${LIBUSB0_ARCHIVE}\"..."

  cd "${DOWNLOAD_FOLDER_PATH}"
  curl -L "${LIBUSB0_URL}" --output "${LIBUSB0_ARCHIVE}"
fi

# Unpack the old USB library.
if [ ! -d "${WORK_FOLDER_PATH}/${LIBUSB0_FOLDER}" ]
then
  cd "${WORK_FOLDER_PATH}"
  tar -xjvf "${DOWNLOAD_FOLDER_PATH}/${LIBUSB0_ARCHIVE}"
fi

# https://sourceforge.net/projects/libusb-win32

# Download the old Win32 USB library.
if [ ! -f "${DOWNLOAD_FOLDER_PATH}/${LIBUSB_W32_ARCHIVE}" ]
then
  mkdir -p "${DOWNLOAD_FOLDER_PATH}"

  echo
  echo "Downloading \"${LIBUSB_W32_ARCHIVE}\"..."

  cd "${DOWNLOAD_FOLDER_PATH}"
  curl -L "${LIBUSB_W32_URL}" --output "${LIBUSB_W32_ARCHIVE}"
fi

# Unpack the old Win32 USB library.
if [ ! -d "${WORK_FOLDER_PATH}/${LIBUSB_W32_FOLDER}" ]
then
  cd "${WORK_FOLDER_PATH}"
  unzip "${DOWNLOAD_FOLDER_PATH}/${LIBUSB_W32_ARCHIVE}"
fi


# ----- Get the FTDI library. -----

# There are two versions of the FDDI library; we recommend using the
# open source one, available from intra2net.
#	http://www.intra2net.com/en/developer/libftdi/

# Download the FTDI library.
if [ ! -f "${DOWNLOAD_FOLDER_PATH}/${LIBFTDI_ARCHIVE}" ]
then
  mkdir -p "${DOWNLOAD_FOLDER_PATH}"

  echo
  echo "Downloading \"${LIBFTDI_ARCHIVE}\"..."

  cd "${DOWNLOAD_FOLDER_PATH}"
  curl -L "${LIBFTDI_URL}" --output "${LIBFTDI_ARCHIVE}"
fi

# Unpack the FTDI library.
if [ ! -d "${WORK_FOLDER_PATH}/${LIBFTDI_FOLDER}" ]
then
  echo
  echo "Unpacking \"${LIBFTDI_ARCHIVE}\"..."

  cd "${WORK_FOLDER_PATH}"
  tar -xjvf "${DOWNLOAD_FOLDER_PATH}/${LIBFTDI_ARCHIVE}"

  echo
  echo "Patching \"${LIBFTDI_FOLDER}\"..."

  cd "${WORK_FOLDER_PATH}/${LIBFTDI_FOLDER}"
  # Patch to prevent the use of system libraries and force the use of local ones.
  patch -p0 < "${PROJECT_GIT_FOLDER_PATH}/gnu-mcu-eclipse/patches/${LIBFTDI}-cmake-FindUSB1.patch"
fi

# ----- Get the HDI library. -----

# This is just a simple wrapper over libusb.
# http://www.signal11.us/oss/hidapi/

# Download the HDI library.
if [ ! -f "${DOWNLOAD_FOLDER_PATH}/${HIDAPI_ARCHIVE}" ]
then
  mkdir -p "${DOWNLOAD_FOLDER_PATH}"

  cd "${DOWNLOAD_FOLDER_PATH}"
  echo
  echo "Downloading \"${HIDAPI_ARCHIVE}\"..."

  # https://github.com/downloads/signal11/hidapi
  # https://github.com/signal11/hidapi/archive/
  curl -L "${HIDAPI_URL}" --output "${HIDAPI_ARCHIVE}"
fi

# Unpack the HDI library.
if [ ! -d "${WORK_FOLDER_PATH}/${HIDAPI_FOLDER}" ]
then
  cd "${WORK_FOLDER_PATH}"
  unzip "${DOWNLOAD_FOLDER_PATH}/${HIDAPI_ARCHIVE}"
fi

# Download the LIBICONV library.
if [ ! -f "${DOWNLOAD_FOLDER_PATH}/${LIBICONV_ARCHIVE}" ]
then
  mkdir -p "${DOWNLOAD_FOLDER_PATH}"

  cd "${DOWNLOAD_FOLDER_PATH}"
  echo
  echo "Downloading \"${LIBICONV_ARCHIVE}\"..."

  curl --fail -L "${LIBICONV_URL}" --output "${LIBICONV_ARCHIVE}"
fi

# Unpack the LIBICONV library.
if [ ! -d "${WORK_FOLDER_PATH}/${LIBICONV_FOLDER}" ]
then
  cd "${WORK_FOLDER_PATH}"
  tar -xvf "${DOWNLOAD_FOLDER_PATH}/${LIBICONV_ARCHIVE}"
fi


# v===========================================================================v
# Create the build script (needs to be separate for Docker).

script_name="inner-build.sh"
script_file_path="${WORK_FOLDER_PATH}/scripts/${script_name}"

rm -f "${script_file_path}"
mkdir -p "$(dirname ${script_file_path})"
touch "${script_file_path}"

# Note: __EOF__ is quoted to prevent substitutions here.
cat <<'__EOF__' >> "${script_file_path}"
#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Safety settings (see https://gist.github.com/ilg-ul/383869cbb01f61a51c4d).

if [[ ! -z ${DEBUG} ]]
then
  set -x # Activate the expand mode if DEBUG is anything but empty.
else
  DEBUG=""
fi

set -o errexit # Exit if command failed.
set -o pipefail # Exit if pipe failed.
set -o nounset # Exit if variable not set.

# Remove the initial space and instead use '\n'.
IFS=$'\n\t'

# -----------------------------------------------------------------------------

__EOF__
# The above marker must start in the first column.

# Note: __EOF__ is not quoted to allow local substitutions.
cat <<__EOF__ >> "${script_file_path}"

APP_NAME="${APP_NAME}"
APP_LC_NAME="${APP_LC_NAME}"
APP_UC_NAME="${APP_UC_NAME}"
DISTRIBUTION_FILE_DATE="${DISTRIBUTION_FILE_DATE}"
PROJECT_GIT_FOLDER_NAME="${PROJECT_GIT_FOLDER_NAME}"
OPENOCD_FOLDER_NAME="${OPENOCD_FOLDER_NAME}"

DEPLOY_FOLDER_NAME="${DEPLOY_FOLDER_NAME}"

LIBUSB1_FOLDER="${LIBUSB1_FOLDER}"
LIBUSB0_FOLDER="${LIBUSB0_FOLDER}"
LIBUSB_W32="${LIBUSB_W32}"
LIBUSB_W32_FOLDER="${LIBUSB_W32_FOLDER}"
LIBFTDI_FOLDER="${LIBFTDI_FOLDER}"
HIDAPI_FOLDER="${HIDAPI_FOLDER}"
HIDAPI="${HIDAPI}"
LIBICONV_FOLDER="${LIBICONV_FOLDER}"
LIBICONV="${LIBICONV}"

do_no_strip="${do_no_strip}"
do_no_pdf="${do_no_pdf}"
do_debug="${do_debug}"
jobs="${jobs}"

__EOF__
# The above marker must start in the first column.

# Propagate DEBUG to guest.
set +u
if [[ ! -z ${DEBUG} ]]
then
  echo "DEBUG=${DEBUG}" "${script_file_path}"
  echo
fi
set -u

# Note: __EOF__ is quoted to prevent substitutions here.
cat <<'__EOF__' >> "${script_file_path}"

PKG_CONFIG_LIBDIR=${PKG_CONFIG_LIBDIR:-""}

# For just in case.
export LC_ALL="C"
export CONFIG_SHELL="/bin/bash"

script_name="$(basename "$0")"
args="$@"
docker_container_name=""

while [ $# -gt 0 ]
do
  case "$1" in
    --container-build-folder)
      container_build_folder_path="$2"
      shift 2
      ;;

    --container-install-folder)
      container_install_folder_path="$2"
      shift 2
      ;;

    --container-output-folder)
      container_output_folder_path="$2"
      shift 2
      ;;

    --shared-install-folder)
      shared_install_folder_path="$2"
      shift 2
      ;;

    --docker-container-name)
      docker_container_name="$2"
      shift 2
      ;;

    --target-os)
      target_os="$2"
      shift 2
      ;;

    --target-bits)
      target_bits="$2"
      shift 2
      ;;

    --work-folder)
      work_folder_path="$2"
      shift 2
      ;;

    --distribution-folder)
      distribution_folder="$2"
      shift 2
      ;;

    --download-folder)
      download_folder="$2"
      shift 2
      ;;

    --helper-script)
      helper_script_path="$2"
      shift 2
      ;;

    --group-id)
      group_id="$2"
      shift 2
      ;;

    --user-id)
      user_id="$2"
      shift 2
      ;;

    --host-uname)
      host_uname="$2"
      shift 2
      ;;

    *)
      echo "Unknown option $1, exit."
      exit 1
  esac
done

# -----------------------------------------------------------------------------

# XBB not available when running from macOS.
if [ -f "/opt/xbb/xbb.sh" ]
then
  source "/opt/xbb/xbb.sh"
fi

# -----------------------------------------------------------------------------

# Run the helper script in this shell, to get the support functions.
source "${helper_script_path}"

# Requires XBB_FOLDER.
do_container_detect

if [ -f "/opt/xbb/xbb.sh" ]
then

  # Required by jimtcl, building their bootstrap fails on 32-bits.
  # Must be installed befoare activating XBB, otherwise yum fails,
  # with python failing some crypto.
  yum install -y tcl

  xbb_activate

  # Don't forget to add `-static-libstdc++` to app LDFLAGS,
  # otherwise the final executable may have a reference to 
  # a wrong `libstdc++.so.6`.

  export PATH="/opt/texlive/bin/${CONTAINER_MACHINE}-linux":${PATH}

fi

# -----------------------------------------------------------------------------

git_folder_path="${work_folder_path}/${PROJECT_GIT_FOLDER_NAME}"

EXTRA_CFLAGS="-ffunction-sections -fdata-sections -m${target_bits} -pipe"
EXTRA_CXXFLAGS="-ffunction-sections -fdata-sections -m${target_bits} -pipe"
if [ "${do_debug}" == "y" ]
then
  EXTRA_CFLAGS="${EXTRA_CFLAGS} -g"
  EXTRA_CXXFLAGS="${EXTRA_CXXFLAGS} -g"
fi

EXTRA_CPPFLAGS="-I${install_folder}/include"
EXTRA_LDFLAGS="-L${install_folder}/lib64 -L${install_folder}/lib -static-libstdc++ -Wl,--gc-sections"

# export PKG_CONFIG_PREFIX="${install_folder}"
# export PKG_CONFIG="${git_folder_path}/gnu-mcu-eclipse/scripts/cross-pkg-config"
export PKG_CONFIG=pkg-config-verbose
export PKG_CONFIG_LIBDIR="${install_folder}/lib64/pkgconfig":"${install_folder}/lib/pkgconfig"


# -----------------------------------------------------------------------------

mkdir -p "${build_folder_path}"
cd "${build_folder_path}"

# ----- Test if various tools are present -----

echo
echo "Checking automake..."
automake --version 2>/dev/null | grep automake

echo "Checking cmake..."
cmake --version | grep cmake

echo "Checking pkg-config..."
pkg-config --version

if [ "${target_os}" != "osx" ]
then
  echo "Checking readelf..."
  readelf --version | grep readelf
fi

if [ "${target_os}" == "win" ]
then
  echo "Checking ${cross_compile_prefix}-gcc..."
  ${cross_compile_prefix}-gcc --version 2>/dev/null | egrep -e 'gcc|clang'

  echo "Checking unix2dos..."
  unix2dos --version 2>&1 | grep unix2dos

  echo "Checking makensis..."
  echo "makensis $(makensis -VERSION)"

  # Now in Docker image
  # apt-get --yes install zip

  echo "Checking zip..."
  zip -v | grep "This is Zip"
else
  echo "Checking gcc..."
  gcc --version 2>/dev/null | egrep -e 'gcc|clang'
fi

if [ "${target_os}" == "linux" ]
then
  echo "Checking patchelf..."
  patchelf --version
fi

echo "Checking shasum..."
shasum --version

if [ "${target_os}" != "win" ]
then
  export CC=gcc
  export CXX=g++
fi

# ----- Recreate the output folder. -----

# rm -rf "${output_folder_path}"
mkdir -p "${output_folder_path}"

# ----- Build and install the new USB library. -----

libusb1_stamp_file="${build_folder_path}/${LIBUSB1_FOLDER}/stamp-install-completed"

if [ ! -f "${libusb1_stamp_file}" ]
then

  rm -rfv "${build_folder_path}/${LIBUSB1_FOLDER}"
  mkdir -p "${build_folder_path}/${LIBUSB1_FOLDER}"

  mkdir -p "${install_folder}"

  echo
  echo "Running libusb1 configure..."

  (
    cd "${build_folder_path}/${LIBUSB1_FOLDER}"

    "${work_folder_path}/${LIBUSB1_FOLDER}/configure" --help

    # --enable-shared required by libftdi.
    export CFLAGS="${EXTRA_CFLAGS} -Wno-non-literal-null-conversion -Wno-deprecated-declarations  -Wno-format"
    bash "${work_folder_path}/${LIBUSB1_FOLDER}/configure" \
        --prefix="${install_folder}" \
        --build=${BUILD} \
        --host=${HOST} \
        --target=${TARGET} \
        --enable-shared \
        --enable-static

    echo
    echo "Running libusb1 make..."

    # Build.
    make ${jobs}
    make install
  )

  touch "${libusb1_stamp_file}"
fi

# ----- Build and install the old USB library. -----

libusb0_stamp_file="${build_folder_path}/${LIBUSB0_FOLDER}/stamp-install-completed"

if [ \( "${target_os}" != "win" \) -a \
    ! \( -f "${libusb0_stamp_file}" \) ]
then

  rm -rf "${build_folder_path}/${LIBUSB0_FOLDER}"
  mkdir -p "${build_folder_path}/${LIBUSB0_FOLDER}"

  mkdir -p "${install_folder}"

  echo
  echo "Running libusb0 configure..."

  (
    cd "${build_folder_path}/${LIBUSB0_FOLDER}"

    "${work_folder_path}/${LIBUSB0_FOLDER}/configure" --help

    export CFLAGS="${EXTRA_CFLAGS}"

    bash "${work_folder_path}/${LIBUSB0_FOLDER}/configure" \
      --prefix="${install_folder}" \
      --disable-shared \
      --enable-static

    echo
    echo "Running libusb0 make..."

    # Build.
    # make clean 
    make ${jobs}
    make install
  )

  touch "${libusb0_stamp_file}"
fi

# ----- Build and install the old Win32 USB library. -----

libusb_w32_stamp_file="${build_folder_path}/${LIBUSB_W32}/stamp-install-completed"

if [ \( "${target_os}" == "win" \) -a \
     ! \( -f "${libusb_w32_stamp_file}" \)  ]
then

  mkdir -p "${build_folder_path}/${LIBUSB_W32}"

  cd "${build_folder_path}/${LIBUSB_W32}"
  cp -r "${work_folder_path}/${LIBUSB_W32_FOLDER}/"* \
    "${build_folder_path}/${LIBUSB_W32}"

  echo
  echo "Running libusb-win32 make..."

  (
  cd "${build_folder_path}/${LIBUSB_W32}"

  # Patch from:
  # https://gitorious.org/jtag-tools/openocd-mingw-build-scripts

  # The conversions are needed to avoid errors like:
  # 'Hunk #1 FAILED at 31 (different line endings).'
  dos2unix src/install.c
  dos2unix src/install_filter_win.c
  dos2unix src/registry.c
  patch -p1 < "${git_folder_path}/gnu-mcu-eclipse/patches/${LIBUSB_W32}-mingw-w64.patch"

  # Build.
  (
      export CFLAGS="${EXTRA_CFLAGS} -Wno-unknown-pragmas -Wno-unused-variable -Wno-pointer-sign -Wno-unused-but-set-variable"
      make \
        host_prefix=${cross_compile_prefix} \
        host_prefix_x86=i686-w64-mingw32 \
        dll
    )

    mkdir -p "${install_folder}/bin"
    # Skipping it does not remove the reference from openocd, so for the
    # moment it is preserved.
    cp -v "${build_folder_path}/${LIBUSB_W32}/libusb0.dll" \
      "${install_folder}/bin"

    mkdir -p "${install_folder}/lib"
    cp -v "${build_folder_path}/${LIBUSB_W32}/libusb.a" \
      "${install_folder}/lib"

    mkdir -p "${install_folder}/lib/pkgconfig"
    sed -e "s|XXX|${install_folder}|" \
      "${git_folder_path}/gnu-mcu-eclipse/pkgconfig/${LIBUSB_W32}.pc" \
      > "${install_folder}/lib/pkgconfig/libusb.pc"

    mkdir -p "${install_folder}/include/libusb"
    cp -v "${build_folder_path}/${LIBUSB_W32}/src/lusb0_usb.h" \
      "${install_folder}/include/libusb/usb.h"
  )

  touch "${libusb_w32_stamp_file}"
fi

# ----- Build and install the FTDI library. -----

libftdi_stamp_file="${build_folder_path}/${LIBFTDI_FOLDER}/stamp-install-completed"

if [ ! -f "${libftdi_stamp_file}" ]
then

  rm -rfv "${build_folder_path}/${LIBFTDI_FOLDER}"
  mkdir -p "${build_folder_path}/${LIBFTDI_FOLDER}"

  mkdir -p "${install_folder}"

  echo
  echo "Running libftdi cmake..."

  cd "${build_folder_path}/${LIBFTDI_FOLDER}"
  
  (
    export CFLAGS="${EXTRA_CFLAGS}"

    # Note: I could not make it generate only static libs, so it also
    # requires the shared libusb.

    if [ "${target_os}" == "win" ]
    then

      # Configure.
      cmake \
      -DPKG_CONFIG_EXECUTABLE="${PKG_CONFIG}" \
      -DCMAKE_TOOLCHAIN_FILE="${work_folder_path}/${LIBFTDI_FOLDER}/cmake/Toolchain-${cross_compile_prefix}.cmake" \
      -DCMAKE_INSTALL_PREFIX="${install_folder}" \
      -DLIBUSB_INCLUDE_DIR="${install_folder}/include/libusb-1.0" \
      -DLIBUSB_LIBRARIES="${install_folder}/lib/libusb-1.0.a" \
      -DBUILD_TESTS:BOOL=off \
      -DFTDIPP:BOOL=off \
      -DPYTHON_BINDINGS:BOOL=off \
      -DEXAMPLES:BOOL=off \
      -DDOCUMENTATION:BOOL=off \
      -DFTDI_EEPROM:BOOL=off \
      "${work_folder_path}/${LIBFTDI_FOLDER}"

    else

      cmake \
      -DCMAKE_INSTALL_PREFIX="${install_folder}" \
      -DBUILD_TESTS:BOOL=off \
      -DFTDIPP:BOOL=off \
      -DPYTHON_BINDINGS:BOOL=off \
      -DEXAMPLES:BOOL=off \
      -DDOCUMENTATION:BOOL=off \
      -DFTDI_EEPROM:BOOL=off \
      "${work_folder_path}/${LIBFTDI_FOLDER}"

    fi

    echo
    echo "Running libftdi make..."

    # Build.
    make ${jobs} 
    make install

    echo
    echo "Initial shared libraries..."
    if [ "${target_os}" == "win" ]
    then
      ls -lR "${install_folder}"/bin/*.dll
    fi
    ls -lR "${install_folder}"/lib*/

    echo
    echo "Removing shared libraries..."

    # FTDI insists on building the shared libraries, but we do not want them.
    # Remove them dependencies.

    rm -f "${install_folder}"/bin/libftdi1-config
    rm -f "${install_folder}"/bin/libusb-config
    rm -f "${install_folder}"/lib*/pkgconfig/libftdipp1.pc

    if [ "${target_os}" == "win" ]
    then

      # Remove DLLs to force static link for final executable.
      rm -f "${install_folder}"/bin/libftdi*.dll*
      rm -f "${install_folder}"/bin/libusb-1*.dll*

      rm -f "${install_folder}"/lib/libftdi*.dll*

      rm -f "${install_folder}"/lib/libusb*.dll*
      rm -f "${install_folder}"/lib/libusb*.la

    elif [ "${target_os}" == "linux" ]
    then

      # Remove shared to force static link for final executable.
      rm -f "${install_folder}"/lib*/libftdi*.so*

      rm -f "${install_folder}"/lib*/libusb*.so*
      rm -f "${install_folder}"/lib/libusb*.la

    elif [ "${target_os}" == "osx" ]
    then

      # Remove dynamic to force static link for final executable.
      rm -f "${install_folder}"/lib/libftdi*.dylib

      rm -f "${install_folder}"/lib/libusb*.dylib
      rm -f "${install_folder}"/lib/libusb*.la

    fi

    echo
    echo "Final shared libraries..."
    if [ "${target_os}" == "win" ]
    then
      ls -lR "${install_folder}"/bin/*.dll
    fi
    ls -lR "${install_folder}"/lib*/
  )

  touch "${libftdi_stamp_file}"
fi

# ----- Build and install the LIBICONV library. -----

libiconv_stamp_file="${build_folder_path}/${LIBICONV}/stamp-install-completed"

if [ ! -f "${libiconv_stamp_file}" ]
then

  rm -rfv "${build_folder_path}/${LIBICONV_FOLDER}"
  mkdir -p "${build_folder_path}/${LIBICONV_FOLDER}"

  mkdir -p "${install_folder}"

  echo
  echo "Running libiconv configure..."

  (
    cd "${build_folder_path}/${LIBICONV_FOLDER}"

    "${work_folder_path}/${LIBICONV_FOLDER}/configure" --help

    export CFLAGS="${EXTRA_CFLAGS} -Wno-non-literal-null-conversion -Wno-tautological-compare -Wno-parentheses-equality -Wno-static-in-inline -Wno-unused-command-line-argument -Wno-pointer-to-int-cast"

    bash "${work_folder_path}/${LIBICONV_FOLDER}/configure" \
      --prefix="${install_folder}" \
      --build=${BUILD} \
      --host=${HOST} \
      --target=${TARGET} \
      --disable-shared \
      --enable-static \
      --disable-rpath

    echo
    echo "Running libiconv make..."

    # Build.
    make ${jobs}
    make install

    rm -f "${install_folder}"/lib/preloadable_libiconv.so
  )

  touch "${libiconv_stamp_file}"
fi

# ----- Build the new HDI library. -----

libhdi_stamp_file="${build_folder_path}/${HIDAPI_FOLDER}/stamp-install-completed"

if [ "${target_os}" == "win" ]
then
  HIDAPI_TARGET="windows"
  HIDAPI_OBJECT="hid.o"
  HIDAPI_A="libhid.a"
elif [ "${target_os}" == "osx" ]
then
  HIDAPI_TARGET="mac"
  HIDAPI_A="libhidapi.a"
elif [ "${target_os}" == "linux" ]
then
  HIDAPI_TARGET="linux"
  HIDAPI_A="libhidapi-hidraw.a"
fi

if [ ! -f "${libhdi_stamp_file}" ]
then

  rm -rfv "${build_folder_path}/${HIDAPI_FOLDER}"
  mkdir -p "${build_folder_path}/${HIDAPI_FOLDER}"

  cp -r "${work_folder_path}/${HIDAPI_FOLDER}/"* \
    "${build_folder_path}/${HIDAPI_FOLDER}"

  echo
  echo "Building libhid..."

  (
    cd "${build_folder_path}/${HIDAPI_FOLDER}"

    if [ "${target_os}" == "win" ]
    then

      cd "${build_folder_path}/${HIDAPI_FOLDER}/${HIDAPI_TARGET}"

      export CFLAGS="${EXTRA_CFLAGS}" \

      make -f Makefile.mingw \
        CC=${cross_compile_prefix}-gcc \
        "${HIDAPI_OBJECT}"

      # Make just compiles the file. Create the archive and convert it to library.
      # No dynamic/shared libs involved.
      ar -r  libhid.a "${HIDAPI_OBJECT}"
      ${cross_compile_prefix}-ranlib libhid.a

      mkdir -p "${install_folder}/lib"
      cp -v libhid.a \
        "${install_folder}/lib"

      mkdir -p "${install_folder}/lib/pkgconfig"
      sed -e "s|XXX|${install_folder}|" \
        "${git_folder_path}/gnu-mcu-eclipse/pkgconfig/${HIDAPI}-${HIDAPI_TARGET}.pc" \
        > "${install_folder}/lib/pkgconfig/hidapi.pc"

      mkdir -p "${install_folder}/include/hidapi"
      cp -v "${work_folder_path}/${HIDAPI_FOLDER}/hidapi/hidapi.h" \
        "${install_folder}/include/hidapi"

    elif [ "${target_os}" == "linux" ]
    then

      if [ "${target_bits}" == "64" ]
      then
        cp "/usr/include/libudev.h" "${install_folder}/include"
        if [ -f "/usr/lib/x86_64-linux-gnu/libudev.so" ]
        then
          cp "/usr/lib/x86_64-linux-gnu/libudev.so" "${install_folder}/lib"
          cp "/usr/lib/x86_64-linux-gnu/pkgconfig/libudev.pc" "${install_folder}/lib/pkgconfig"
        elif [ -f "/lib/x86_64-linux-gnu/libudev.so" ]
        then
          # In Debian 9 the location changed to /lib
          cp "/lib/x86_64-linux-gnu/libudev.so" "${install_folder}/lib"
          cp "/usr/lib/x86_64-linux-gnu/pkgconfig/libudev.pc" "${install_folder}/lib/pkgconfig"
        elif [ -f "/usr/lib/libudev.so" ]
        then
          # In ARCH the location is /usr/lib
          cp "/usr/lib/libudev.so" "${install_folder}/lib"
          cp "/usr/lib/pkgconfig/libudev.pc" "${install_folder}/lib/pkgconfig"
        elif [ -f "/usr/lib64/libudev.so" ]
        then
          # In CentOS the location is /usr/lib64
          cp "/usr/lib64/libudev.so" "${install_folder}/lib"
          cp "/usr/lib64/pkgconfig/libudev.pc" "${install_folder}/lib/pkgconfig"
        else
          echo "No libudev.so; abort."
          exit 1
        fi
      elif [ "${target_bits}" == "32" ] 
      then
        cp "/usr/include/libudev.h" "${install_folder}/include"
        if [ -f "/usr/lib/i386-linux-gnu/libudev.so" ]
        then
          cp "/usr/lib/i386-linux-gnu/libudev.so" "${install_folder}/lib"
          cp /usr/lib/i386-linux-gnu/pkgconfig/libudev.pc "${install_folder}/lib/pkgconfig"
        elif [ -f "/lib/i386-linux-gnu/libudev.so" ]
        then
          # In Debian 9 the location changed to /lib
          cp "/lib/i386-linux-gnu/libudev.so" "${install_folder}/lib"
          cp /usr/lib/i386-linux-gnu/pkgconfig/libudev.pc "${install_folder}/lib/pkgconfig"
        elif [ -f "/lib/libudev.so.0" ]
        then
          # In CentOS the location is /lib 
          cp "/lib/libudev.so.0" "${install_folder}/lib"
          cp "/usr/lib/pkgconfig/libudev.pc" "${install_folder}/lib/pkgconfig"
        else
          echo "No libudev.so; abort."
          exit 1
        fi
      fi

      ./bootstrap

      ./configure --help

      export CFLAGS="${EXTRA_CFLAGS}"
      export LIBS="-liconv -lpthread -ludev"
      
      ./configure \
        --prefix="${install_folder}" \
        --build=${BUILD} \
        --host=${HOST} \
        --target=${TARGET} \
        --disable-shared \
        --enable-static

      make ${jobs} 
      make install

    elif [ "${target_os}" == "osx" ]
    then

      ./bootstrap

      export CFLAGS="${EXTRA_CFLAGS}"
      export LIBS="-liconv"
      
      ./configure \
        --prefix="${install_folder}" \
        --build=${BUILD} \
        --host=${HOST} \
        --target=${TARGET} \
        --disable-shared \
        --enable-static

      make ${jobs}
      make install

    fi

    rm -f "${install_folder}"/lib*/libhidapi-hidraw.la
  )

  touch "${libhdi_stamp_file}"
fi


# Create the build folder.
mkdir -p "${build_folder_path}/openocd"

# ----- Configure OpenOCD. Use the same options as Freddie Chopin. -----

if [ ! -f "${build_folder_path}/${APP_LC_NAME}/config.h" ]
then

  echo
  echo "Running OpenOCD configure..."

  # May be required for repetitive builds, because this is an executable built 
  # in place and using one for a different architecture may not be a good idea.
  rm -rfv "${work_folder_path}/${OPENOCD_FOLDER_NAME}/jimtcl/autosetup/jimsh0"

  (
    cd "${build_folder_path}/openocd"

    # Deprecated:
    # --enable-ioutil
    # --enable-oocd_trace
    # --enable-zy1000
    # --enable-legacy-ft2232_libftdi

    bash "${work_folder_path}/${OPENOCD_FOLDER_NAME}/configure" --help

    if [ "${target_os}" == "win" ]
    then

      # --enable-minidriver-dummy -> configure error
      # --enable-buspirate -> not supported on mingw
      # --enable-zy1000 -> netinet/tcp.h: No such file or directory
      # --enable-sysfsgpio -> available only on Linux

      # --enable-openjtag_ftdi -> --enable-openjtag
      # --enable-presto_libftdi -> --enable-presto
      # --enable-usb_blaster_libftdi -> --enable-usb_blaster

      export OUTPUT_DIR="${build_folder_path}"
      
      export CFLAGS="${EXTRA_CXXFLAGS} -Wno-pointer-to-int-cast" 
      export CXXFLAGS="${EXTRA_CXXFLAGS}" 
      export LDFLAGS="${EXTRA_LDFLAGS} -static"
      
      bash "${work_folder_path}/${OPENOCD_FOLDER_NAME}/configure" \
      --build="$(uname -m)-linux-gnu" \
      --host="${cross_compile_prefix}" \
      --prefix="${install_folder}/openocd"  \
      --datarootdir="${install_folder}" \
      --infodir="${install_folder}/${APP_LC_NAME}/info"  \
      --localedir="${install_folder}/${APP_LC_NAME}/locale"  \
      --mandir="${install_folder}/${APP_LC_NAME}/man"  \
      --docdir="${install_folder}/${APP_LC_NAME}/doc"  \
      --disable-wextra \
      --disable-werror \
      --enable-dependency-tracking \
      \
      --enable-branding="GNU MCU Eclipse" \
      \
      --enable-aice \
      --enable-amtjtagaccel \
      --enable-armjtagew \
      --enable-at91rm9200 \
      --enable-bcm2835gpio \
      --disable-buspirate \
      --enable-cmsis-dap \
      --enable-dummy \
      --enable-ep93xx \
      --enable-ftdi \
      --enable-gw16012 \
      --disable-ioutil \
      --enable-jlink \
      --enable-jtag_vpi \
      --disable-minidriver-dummy \
      --disable-oocd_trace \
      --enable-opendous \
      --enable-openjtag \
      --enable-osbdm \
      --enable-parport \
      --disable-parport-ppdev \
      --enable-parport-giveio \
      --enable-presto \
      --enable-remote-bitbang \
      --enable-riscv \
      --enable-rlink \
      --enable-stlink \
      --disable-sysfsgpio \
      --enable-ti-icdi \
      --enable-ulink \
      --enable-usb_blaster \
      --enable-usb-blaster-2 \
      --enable-usbprog \
      --enable-vsllink \
      --disable-zy1000-master \
      --disable-zy1000 \
      | tee "${output_folder_path}/configure-output.txt"
      # Note: don't forget to update the INFO.txt file after changing these.

    elif [ "${target_os}" == "linux" ]
    then

      # --enable-minidriver-dummy -> configure error

      # --enable-openjtag_ftdi -> --enable-openjtag
      # --enable-presto_libftdi -> --enable-presto
      # --enable-usb_blaster_libftdi -> --enable-usb_blaster

      export CFLAGS="${EXTRA_CFLAGS} -Wno-format-truncation -Wno-format-overflow"
      export CXXFLAGS="${EXTRA_CXXFLAGS}"
      export LIBS="-lpthread -lrt -ludev"
      export LDFLAGS="${EXTRA_LDFLAGS}" 
       
      bash "${work_folder_path}/${OPENOCD_FOLDER_NAME}/configure" \
      --prefix="${install_folder}/openocd"  \
      --datarootdir="${install_folder}" \
      --infodir="${install_folder}/${APP_LC_NAME}/info"  \
      --localedir="${install_folder}/${APP_LC_NAME}/locale"  \
      --mandir="${install_folder}/${APP_LC_NAME}/man"  \
      --docdir="${install_folder}/${APP_LC_NAME}/doc"  \
      --disable-wextra \
      --disable-werror \
      --enable-dependency-tracking \
      \
      --enable-branding="GNU MCU Eclipse" \
      \
      --enable-aice \
      --enable-amtjtagaccel \
      --enable-armjtagew \
      --enable-at91rm9200 \
      --enable-bcm2835gpio \
      --enable-buspirate \
      --enable-cmsis-dap \
      --enable-dummy \
      --enable-ep93xx \
      --enable-ftdi \
      --enable-gw16012 \
      --disable-ioutil \
      --enable-jlink \
      --enable-jtag_vpi \
      --disable-minidriver-dummy \
      --disable-oocd_trace \
      --enable-opendous \
      --enable-openjtag \
      --enable-osbdm \
      --enable-parport \
      --disable-parport-ppdev \
      --enable-parport-giveio \
      --enable-presto \
      --enable-remote-bitbang \
      --enable-riscv \
      --enable-rlink \
      --enable-stlink \
      --enable-sysfsgpio \
      --enable-ti-icdi \
      --enable-ulink \
      --enable-usb_blaster \
      --enable-usb-blaster-2 \
      --enable-usbprog \
      --enable-vsllink \
      --disable-zy1000-master \
      --disable-zy1000 \
      | tee "${output_folder_path}/configure-output.txt"
      # Note: don't forget to update the INFO.txt file after changing these.

    elif [ "${target_os}" == "osx" ]
    then

      # --enable-minidriver-dummy -> configure error
      # --enable-sysfsgpio -> available only on Linux
      # --enable-amtjtagaccel -> 'sys/io.h' file not found
      # --enable-gw16012 -> 'sys/io.h' file not found

      # --enable-openjtag_ftdi -> --enable-openjtag
      # --enable-presto_libftdi -> --enable-presto
      # --enable-usb_blaster_libftdi -> --enable-usb_blaster

      export CFLAGS="${EXTRA_CFLAGS}"
      export CXXFLAGS="${EXTRA_CXXFLAGS}"
      if [ "${target_os}" != "osx" ]
      then
        export LDFLAGS="${EXTRA_LDFLAGS} -static"
      fi
      export JAYLINK_CFLAGS='${EXTRA_CFLAGS} -Wall -Wextra -fvisibility=hidden'
      
      bash "${work_folder_path}/${OPENOCD_FOLDER_NAME}/configure" \
      --prefix="${install_folder}/openocd"  \
      --datarootdir="${install_folder}" \
      --infodir="${install_folder}/${APP_LC_NAME}/info"  \
      --localedir="${install_folder}/${APP_LC_NAME}/locale"  \
      --mandir="${install_folder}/${APP_LC_NAME}/man"  \
      --docdir="${install_folder}/${APP_LC_NAME}/doc"  \
      --disable-wextra \
      --disable-werror \
      --enable-dependency-tracking \
      \
      --enable-branding="GNU MCU Eclipse" \
      \
      --enable-aice \
      --disable-amtjtagaccel \
      --enable-armjtagew \
      --enable-at91rm9200 \
      --enable-bcm2835gpio \
      --enable-buspirate \
      --enable-cmsis-dap \
      --enable-dummy \
      --enable-ep93xx \
      --enable-ftdi \
      --disable-gw16012 \
      --disable-ioutil \
      --enable-jlink \
      --enable-jtag_vpi \
      --disable-minidriver-dummy \
      --disable-oocd_trace \
      --enable-opendous \
      --enable-openjtag \
      --enable-osbdm \
      --disable-parport \
      --disable-parport-ppdev \
      --disable-parport-giveio \
      --enable-presto \
      --enable-remote-bitbang \
      --enable-riscv \
      --enable-rlink \
      --enable-stlink \
      --disable-sysfsgpio \
      --enable-ti-icdi \
      --enable-ulink \
      --enable-usb-blaster \
      --enable-usb_blaster_2 \
      --enable-usbprog \
      --enable-vsllink \
      --disable-zy1000-master \
      --disable-zy1000 \
      | tee "${output_folder_path}/configure-output.txt"
      # Note: don't forget to update the INFO.txt file after changing these.

    else

      echo "Unsupported target os ${target_os}."
      exit 1

    fi
  )

  cd "${build_folder_path}/${APP_LC_NAME}"
  cp config.* "${output_folder_path}"

fi

# ----- Full build, with documentation. -----

openocd_stamp_file="${build_folder_path}/${APP_LC_NAME}/stamp-install-completed"

if [ ! -f "${openocd_stamp_file}" ]
then

  # The bindir and pkgdatadir are required to configure bin and scripts folders
  # at the same level in the hierarchy.

  echo
  echo "Running OpenOCD make..."

  (
    cd "${build_folder_path}/${APP_LC_NAME}"
    make ${jobs} bindir="bin" pkgdatadir=""
    if [ -z "${do_no_pdf}" ]
    then
      make bindir="bin" pkgdatadir="" pdf html 
    fi
  ) | tee "${output_folder_path}/make-all-output.txt"

  echo
  echo "Running OpenOCD make install..."

  (
    cd "${build_folder_path}/${APP_LC_NAME}"

    if [ -z "${do_no_strip}" ]
    then
      make install-strip
    else
      make install  
    fi

    if [ -z "${do_no_pdf}" ]
    then
      make install-pdf install-html install-man
    fi
  )  | tee "${output_folder_path}/make-install-output.txt"

  touch "${openocd_stamp_file}"
fi

if [ "${do_debug}" == "y" ]
then
  rm "${openocd_stamp_file}"

  echo
  echo "Proceed with the debug session."
  echo "${install_folder}"/openocd/bin
  ls -l "${install_folder}"/openocd/bin
  echo
  exit 0
fi

# ----- Copy dynamic libraries to the install bin folder. -----

checking_stamp_file="${build_folder_path}/stamp_check_completed"

if [ ! -f "${checking_stamp_file}" ]
then

  if [ "${target_os}" == "win" ]
  then

    echo
    echo "Copying DLLs..."

    # Identify the current cross gcc version, to locate the specific dll folder.
    CROSS_GCC_VERSION=$(${cross_compile_prefix}-gcc --version | grep 'gcc' | sed -e 's/.*\s\([0-9]*\)[.]\([0-9]*\)[.]\([0-9]*\).*/\1.\2.\3/')
    CROSS_GCC_VERSION_SHORT=$(echo $CROSS_GCC_VERSION | sed -e 's/\([0-9]*\)[.]\([0-9]*\)[.]\([0-9]*\).*/\1.\2/')
    SUBLOCATION="-win32"

    echo "${CROSS_GCC_VERSION}" "${CROSS_GCC_VERSION_SHORT}" "${SUBLOCATION}"

    if [ "${target_bits}" == "32" ]
    then
      do_container_win_copy_gcc_dll "libgcc_s_sjlj-1.dll"
    elif [ "${target_bits}" == "64" ]
    then
      do_container_win_copy_gcc_dll "libgcc_s_seh-1.dll"
    fi

    # For unknown reasons, openocd still has a reference to libusb0.dll,
    # although everything should have been compiled as static.
    cp -v "${install_folder}"/bin/libusb0.dll "${install_folder}/${APP_LC_NAME}"/bin

  elif [ "${target_os}" == "linux" ]
  then

    # This is a very important detail: 'patchelf' sets "runpath"
    # in the ELF file to $ORIGIN, telling the loader to search
    # for the libraries first in LD_LIBRARY_PATH (if set) and, if not found there,
    # to look in the same folder where the executable is located -- where
    # this build script installs the required libraries. 
    # Note: LD_LIBRARY_PATH can be set by a developer when testing alternate 
    # versions of the openocd libraries without removing or overwriting 
    # the installed library files -- not done by the typical user. 
    # Note: patchelf changes the original "rpath" in the executable (a path 
    # in the docker container) to "runpath" with the value "$ORIGIN". rpath 
    # instead or runpath could be set to $ORIGIN but rpath is searched before
    # LD_LIBRARY_PATH which requires an installed library be deleted or
    # overwritten to test or use an alternate version. In addition, the usage of
    # rpath is deprecated. See man ld.so for more info.  
    # Also, runpath is added to the installed library files using patchelf, with 
    # value $ORIGIN, in the same way. See patchelf usage in build-helper.sh.
    #
    patchelf --set-rpath '$ORIGIN' "${install_folder}/${APP_LC_NAME}/bin/openocd"

    echo
    echo "Copying shared libs..."

    if [ "${target_bits}" == "64" ]
    then
      distro_machine="x86_64"
    elif [ "${target_bits}" == "32" ]
    then
      distro_machine="i386"
    fi

    # do_container_linux_copy_user_so libusb-1.0
    # do_container_linux_copy_user_so libusb-0.1
    # do_container_linux_copy_user_so libftdi1
    # do_container_linux_copy_user_so libhidapi-hidraw

    do_container_linux_copy_system_so libudev
    # do_container_linux_copy_librt_so

  fi

  touch "${checking_stamp_file}"
fi

# ----- Copy the license files. -----

license_stamp_file="${build_folder_path}/stamp_license_completed"

if [ ! -f "${license_stamp_file}" ]
then

  echo
  echo "Copying license files..."

  do_container_copy_license "${git_folder_path}" "openocd"
  do_container_copy_license "${work_folder_path}/${HIDAPI_FOLDER}" "${HIDAPI_FOLDER}"
  do_container_copy_license "${work_folder_path}/${LIBFTDI_FOLDER}" "${LIBFTDI_FOLDER}"
  do_container_copy_license "${work_folder_path}/${LIBUSB1_FOLDER}" "${LIBUSB1_FOLDER}"
  do_container_copy_license "${work_folder_path}/${LIBICONV_FOLDER}" "${LIBICONV_FOLDER}"

  if [ "${target_os}" == "win" ]
  then
    do_container_copy_license "${work_folder_path}/${LIBUSB_W32_FOLDER}" "${LIBUSB_W32}"
  else
    do_container_copy_license "${work_folder_path}/${LIBUSB0_FOLDER}" "${LIBUSB0_FOLDER}"
  fi

  if [ "${target_os}" == "win" ]
  then
    # Copy the LICENSE to be used by nsis.
    /usr/bin/install -v -c -m 644 "${git_folder_path}/LICENSE" "${install_folder}/${APP_LC_NAME}/licenses"

    # For Windows, process cr lf
    find "${install_folder}/${APP_LC_NAME}/licenses" -type f \
      -exec unix2dos {} \;
  fi

  touch "${license_stamp_file}"
fi


# ----- Copy the GNU MCU Eclipse info files. -----

info_stamp_file="${build_folder_path}/stamp_info_completed"

if [ ! -f "${info_stamp_file}" ]
then

  do_container_copy_info

  touch "${info_stamp_file}"

fi

# ----- Create the distribution package. -----

mkdir -p "${output_folder_path}"

distribution_file_version=$(cat "${git_folder_path}/gnu-mcu-eclipse/VERSION")-${DISTRIBUTION_FILE_DATE}

do_container_create_distribution

do_check_application "openocd" --version

do_container_copy_install

# Requires ${distribution_file} and ${result}
do_container_completed

exit 0

__EOF__
# The above marker must start in the first column.
# ^===========================================================================^

# ----- Build the native distribution. -----

if [ -z "${DO_BUILD_OSX}${DO_BUILD_LINUX64}${DO_BUILD_WIN64}${DO_BUILD_LINUX32}${DO_BUILD_WIN32}" ]
then

  do_host_build_target "Creating the native distribution..." 

else

  if [ "${DO_BUILD_OSX}" == "y" ]
  then
    if [ "${HOST_UNAME}" == "Darwin" ]
    then
      do_host_build_target "Creating the OS X distribution..." \
        --target-os osx
    else
      echo "Building the macOS image is not possible on this platform."
      exit 1
    fi
  fi

  # ----- Build the GNU/Linux 64-bits distribution. -----

  if [ "${DO_BUILD_LINUX64}" == "y" ]
  then
    do_host_build_target "Creating the GNU/Linux 64-bits distribution..." \
      --target-os linux \
      --target-bits 64 \
      --docker-image "${docker_linux64_image}"
  fi

  # ----- Build the Windows 64-bits distribution. -----

  if [ "${DO_BUILD_WIN64}" == "y" ]
  then
    do_host_build_target "Creating the Windows 64-bits distribution..." \
      --target-os win \
      --target-bits 64 \
      --docker-image "${docker_linux64_image}" 
  fi

  # ----- Build the GNU/Linux 32-bits distribution. -----

  if [ "${DO_BUILD_LINUX32}" == "y" ]
  then
    do_host_build_target "Creating the GNU/Linux 32-bits distribution..." \
      --target-os linux \
      --target-bits 32 \
      --docker-image "${docker_linux32_image}"
  fi

  # ----- Build the Windows 32-bits distribution. -----

  # Use the debian64 container.
  if [ "${DO_BUILD_WIN32}" == "y" ]
  then
    do_host_build_target "Creating the Windows 32-bits distribution..." \
      --target-os win \
      --target-bits 32 \
      --docker-image "${docker_linux32_image}"
  fi

fi

do_host_show_sha

do_host_stop_timer

# ----- Done. -----
exit 0
