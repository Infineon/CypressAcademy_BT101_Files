#!/bin/bash
#set -euo pipefail
#IFS=$'\n\t'

# Multi-platform helper for OpenOCD & QEMU builds, using Docker.

# v===========================================================================v
do_host_start_timer() {

  BEGIN_SEC=$(date +%s)
  echo
  echo "Script \"$0\" started at $(date)."
}

# v===========================================================================v
do_host_stop_timer() {

  END_SEC=$(date +%s)
  echo
  echo "Script \"$0\" completed at $(date)."
  DELTA_SEC=$((END_SEC-BEGIN_SEC))
  if [ ${DELTA_SEC} -lt 100 ]
  then
    echo "Duration: ${DELTA_SEC} seconds."
  else
    DELTA_MIN=$(((DELTA_SEC+30)/60))
    echo "Duration: ${DELTA_MIN} minutes."
  fi

  if [ "${HOST_UNAME}" == "Darwin" ]
  then
    say "Wake up, the build completed successfully"
  fi
}

# v===========================================================================v
do_host_detect() {

  echo
  uname -a

  HOST_DISTRO_NAME=""
  HOST_UNAME="$(uname)"
  if [ "${HOST_UNAME}" == "Darwin" ]
  then
        HOST_BITS="64"
        HOST_MACHINE="x86_64"

        HOST_DISTRO_NAME=Darwin
        HOST_DISTRO_LC_NAME=darwin

  elif [ "${HOST_UNAME}" == "Linux" ]
  then
        # ----- Determine distribution name and word size -----

        set +e
        HOST_DISTRO_NAME=$(lsb_release -si)
        set -e

        if [ -z "${HOST_DISTRO_NAME}" ]
        then
          echo "Please install the lsb core package and rerun."
          HOST_DISTRO_NAME="Linux"
        fi

        if [ "$(uname -m)" == "x86_64" ]
        then
          HOST_BITS="64"
          HOST_MACHINE="x86_64"
        elif [ "$(uname -m)" == "i686" ]
        then
          HOST_BITS="32"
          HOST_MACHINE="i386"
        else
          echo "Unknown uname -m $(uname -m)"
          exit 1
        fi

        HOST_DISTRO_LC_NAME=$(echo ${HOST_DISTRO_NAME} | tr "[:upper:]" "[:lower:]")

  else
        echo "Unknown uname ${HOST_UNAME}"
        exit 1
  fi

  echo
  echo "Running on ${HOST_DISTRO_NAME} ${HOST_BITS}-bits."


  # When running on Docker, the host Work folder is used, if available.
  HOST_WORK_FOLDER="${WORK_FOLDER_PATH}/../../Host/Work/${APP_LC_NAME}"

  DOCKER_HOST_WORK="/Host/Work/${APP_LC_NAME}"
  DOCKER_GIT_FOLDER="${DOCKER_HOST_WORK}/${APP_LC_NAME}.git"
  DOCKER_BUILD="/root/build"

  GROUP_ID=$(id -g)
  USER_ID=$(id -u)
}

# v===========================================================================v
do_host_prepare_prerequisites() {

  if [ "${HOST_UNAME}" == "Darwin" ]
  then
        local hb_folder="${HOME}/opt/homebrew/gme"
        
        local must_install=""
        # Check local Homebrew.
        if [ ! -d "${hb_folder}" ]
        then
          must_install="y"
	      else
          PATH="${hb_folder}/bin":${PATH}
          export PATH

          echo
          echo "Checking Homebrew in '${hb_folder}'..."
          set +e
          brew --version | grep 'Homebrew '
          if [ $? -ne 0 ]
          then
            must_install="y"
          fi
          set -e
        fi

        if [ -n "${must_install}" ]
        then
          echo
          echo "Installing a custom Homebrew instance..."
          
          if [ ! -d "${HOME}/opt/install-scripts.git" ]
          then
            mkdir -p "${HOME}/opt"
            git clone https://github.com/ilg-ul/opt-install-scripts "${HOME}/opt/install-scripts.git"
          fi

          bash "${HOME}/opt/install-scripts.git/install-homebrew-gme.sh"

          PATH="${hb_folder}/bin":${PATH}
          export PATH

        fi

	    if [ -z "${do_no_pdf}" ]
	    then

	      local tl_folder="$HOME/opt/texlive"

        must_install=""
        # Check local TeX Live.
        if [ ! -d "${tl_folder}" ]
        then
          must_install="y"
        else

          PATH="${tl_folder}/bin/x86_64-darwin":$PATH
          export PATH

          echo
          echo "Checking TeX Live in '${tl_folder}'..."
          set +e
          tex --version | grep 'TeX 3'
          if [ $? != 0 ]
          then
            must_install="y"
          fi
          set -e

        fi

        if [ -n "${must_install}" ]
        then

            echo
            echo "Please install TeX Live and rerun."
            echo "Alternatively restart the build script using '--without-pdf'."
            echo 
            echo "mkdir -p \${HOME}/opt"
            echo "git clone https://github.com/ilg-ul/opt-install-scripts \${HOME}/opt/install-scripts.git"
            echo "bash \${HOME}/opt/install-scripts.git/install-texlive.sh"
            exit 1
        fi

	    fi
    fi

    echo
    echo "Checking host curl..."
    curl --version | grep curl

    echo
    echo "Checking host git..."
    git --version
}

# v===========================================================================v
do_host_prepare_prerequisites_riscv() {

      if [ "${HOST_UNAME}" == "Darwin" ]
      then

        local hb_folder="$HOME/opt/homebrew-rv"
        local tl_folder="$HOME/opt/texlive"

        # Check local Homebrew.
        if [ -d "${hb_folder}" ]
        then

          PATH="${hb_folder}/bin":$PATH
          export PATH

          echo
          echo "Checking Homebrew in '${hb_folder}'..."
          set +e
          brew --version | grep 'Homebrew '
          if [ $? != 0 ]
          then
            echo "Installing Homebrew..."
             
            mkdir -p "${HOME}/opt"
            git clone https://github.com/ilg-ul/opt-install-scripts "${HOME}/opt/install-scripts.git"
            bash "${HOME}/opt/install-scripts.git/install-homebrew-rv.sh"
          fi
          set -e
        fi

        # Check local TeX Live.
        if [ -d "${tl_folder}" ]
        then

          PATH="${tl_folder}/bin/x86_64-darwin":$PATH
          export PATH

          echo
          echo "Checking TeX Live in '${tl_folder}'..."
          set +e
          tex --version | grep 'TeX 3'
          if [ $? != 0 ]
          then
            echo "Please install TeX Live and rerun."
            echo 
            echo "mkdir -p \${HOME}/opt"
            echo "git clone https://github.com/ilg-ul/opt-install-scripts \${HOME}/opt/install-scripts.git"
            echo "bash \${HOME}/opt/install-scripts.git/install-texlive.sh"
            exit 1
          fi
          set -e

        fi
      fi

      echo
      echo "Checking host curl..."
      curl --version | grep curl

      echo
      echo "Checking host git..."
      git --version
}

# v===========================================================================v
do_host_prepare_docker() {

      echo
      echo "Checking Docker..."
      set +e
      docker --version
      if [ $? != 0 ]
      then
        echo "Please install Docker (https://docs.docker.com/installation/) and rerun."
        exit 1
      fi
      set -e

      if [ "${HOST_UNAME}" == "Darwin" ]
      then
        echo "Preparing Docker environment..."
        eval "$(docker-machine env default)"
      fi
}

# v===========================================================================v
do_host_get_git_head() {

      # Get the current Git branch name, to know if we are building the stable or
      # the development release.
      cd "${PROJECT_GIT_FOLDER_PATH}"
      set +e
      GIT_HEAD=$(git symbolic-ref -q --short HEAD)
      if [ $? -eq 1 ]
      then
        GIT_HEAD=""
      fi
      set -e
}

# v===========================================================================v
do_host_get_current_date() {

      # Use the UTC date as version in the name of the distribution file.
      DISTRIBUTION_FILE_DATE=${DISTRIBUTION_FILE_DATE:-$(date -u +%Y%m%d-%H%M)}
}

# v===========================================================================v
do_host_build_target() {

  message="$1"
  shift

  echo
  echo "================================================================================"
  echo "${message}"

  target_os=""
  target_bits="-"
  docker_image=""
  build_binaries_path=""

  while [ $# -gt 0 ]
  do
    case "$1" in
      --target-os)
        target_os="$2"
        shift 2
        ;;
      --target-bits)
        target_bits="$2"
        shift 2
        ;;
      --docker-image)
        docker_image="$2"
        shift 2
        ;;
      --build-binaries-path)
        build_binaries_path="$2"
        shift 2
        ;;
      *)
        echo "Unknown option $1, exit."
        exit 1
    esac
  done

  if [ -z "${target_os}" -a -n "${HOST_UNAME}" ]
  then
    # Build native
    if [ "${HOST_UNAME}" == "Darwin" ]
    then
      target_os="osx"
      target_bits="-"
    elif [ "${HOST_UNAME}" == "Linux" ]
    then
      target_os="linux"
      target_bits="-"
    else
      echo "Unsupported host ${HOST_UNAME}, exit."
      exit 1
    fi
  fi

  if [ -n "${docker_image}" ]
  then

    if [ "${do_develop}" == "y" ]
    then
      # Host build folder.
      docker_build_folder_path="${DOCKER_HOST_WORK}/build"
      docker_install_folder_path="${DOCKER_HOST_WORK}/install"
    else
      # Non persistent docker folder.
      docker_build_folder_path="/tmp/build"
      docker_install_folder_path="/tmp/install"
    fi

    if [ -n "${build_binaries_path}" ]
    then

      run_docker_script \
        --script "${DOCKER_HOST_WORK}/scripts/${script_name}" \
        --docker-image "${docker_image}" \
        --docker-container-name "${APP_LC_NAME}-${target_os}${target_bits}-build" \
        --host-uname "${HOST_UNAME}" \
        -- \
        --container-build-folder "${docker_build_folder_path}" \
        --container-install-folder "${docker_install_folder_path}" \
        --container-output-folder "${DOCKER_HOST_WORK}/${DEPLOY_FOLDER_NAME}" \
        --shared-install-folder "${DOCKER_HOST_WORK}/install" \
        --target-os "${target_os}" \
        --target-bits "${target_bits}" \
        --distribution-folder "${DOCKER_HOST_WORK}/${DEPLOY_FOLDER_NAME}" \
        --download-folder "${DOCKER_HOST_WORK}/download" \
        --helper-script "${DOCKER_HOST_WORK}/scripts/build-helper.sh" \
        --work-folder "${DOCKER_HOST_WORK}" \
        --group-id "${GROUP_ID}" \
        --user-id "${USER_ID}" \
        --host-uname "${HOST_UNAME}" \
        --extra-path "${DOCKER_HOST_WORK}/${build_binaries_path}"
    
    else

      run_docker_script \
        --script "${DOCKER_HOST_WORK}/scripts/${script_name}" \
        --docker-image "${docker_image}" \
        --docker-container-name "${APP_LC_NAME}-${target_os}${target_bits}-build" \
        --host-uname "${HOST_UNAME}" \
        -- \
        --container-build-folder "${docker_build_folder_path}" \
        --container-install-folder "${docker_install_folder_path}" \
        --container-output-folder "${DOCKER_HOST_WORK}/${DEPLOY_FOLDER_NAME}" \
        --shared-install-folder "${DOCKER_HOST_WORK}/install" \
        --target-os "${target_os}" \
        --target-bits "${target_bits}" \
        --distribution-folder "${DOCKER_HOST_WORK}/${DEPLOY_FOLDER_NAME}" \
        --download-folder "${DOCKER_HOST_WORK}/download" \
        --helper-script "${DOCKER_HOST_WORK}/scripts/build-helper.sh" \
        --work-folder "${DOCKER_HOST_WORK}" \
        --group-id "${GROUP_ID}" \
        --user-id "${USER_ID}" \
        --host-uname "${HOST_UNAME}"

    fi

  else

    run_local_script \
      --script "${script_file_path}" \
      --host-uname "${HOST_UNAME}" \
      -- \
      --container-build-folder "${WORK_FOLDER_PATH}/build" \
      --container-install-folder "${WORK_FOLDER_PATH}/install" \
      --container-output-folder "${WORK_FOLDER_PATH}/${DEPLOY_FOLDER_NAME}" \
      --shared-install-folder "${WORK_FOLDER_PATH}/install" \
      --target-os "${target_os}" \
      --target-bits "${target_bits}" \
      --distribution-folder "${WORK_FOLDER_PATH}/${DEPLOY_FOLDER_NAME}" \
      --download-folder "${WORK_FOLDER_PATH}/download" \
      --helper-script "${WORK_FOLDER_PATH}/scripts/build-helper.sh" \
      --work-folder "${WORK_FOLDER_PATH}" \
      --host-uname "${HOST_UNAME}"

  fi

  echo "do_host_build_target $@ completed"
}

# v===========================================================================v
do_host_show_sha() {

  # ---- Prevent script break because of not found SHA file without arguments ----
  mkdir -p ${WORK_FOLDER_PATH}/${DEPLOY_FOLDER_NAME}
  echo "" > ${WORK_FOLDER_PATH}/${DEPLOY_FOLDER_NAME}/empty.sha
  # ----

  cat "${WORK_FOLDER_PATH}/${DEPLOY_FOLDER_NAME}/"*.sha

}

# v===========================================================================v
run_docker_script() {

  while [ $# -gt 0 ]
  do
    case "$1" in
      --script)
        docker_script="$2"
        shift 2
        ;;
      --docker-image)
        docker_image="$2"
        shift 2
        ;;
      --docker-container-name)
        docker_container_name="$2"
        shift 2
        ;;
      --host-uname)
        host_uname="$2"
        shift 2
        ;;
      --)
        shift
        break;
        ;;
    esac
  done

  set +e
  # Remove a possible previously crashed container.
  docker rm --force "${docker_container_name}" > /dev/null 2> /dev/null
  set -e

  echo
  echo "Running \"$(basename "${docker_script}")\" script inside \"${docker_container_name}\" container, image \"${docker_image}\"..."

  # Run the second pass script in a fresh Docker container.
  docker run \
    --name="${docker_container_name}" \
    --tty \
    --hostname "docker" \
    --workdir="/root" \
    --volume="${WORK_FOLDER_PATH}/:${DOCKER_HOST_WORK}" \
    ${docker_image} \
    /bin/bash ${DEBUG} "${docker_script}" \
      --docker-container-name "${docker_container_name}" \
      $@

  # Remove the container.
  docker rm --force "${docker_container_name}"

  # echo "2|$@|"
}

# v===========================================================================v
run_local_script() {

  while [ $# -gt 0 ]
  do
    case "$1" in
      --script)
        local_script="$2"
        shift 2
        ;;
      --host-uname)
        host_uname="$2"
        shift 2
        ;;
      --)
        shift
        break;
        ;;
    esac
  done

  echo
  echo "Running \"$(basename "${local_script}")\" script locally..."

  # Run the second pass script in a local sub-shell.
  /bin/bash ${DEBUG} "${local_script}" $@

  # echo "1|$@|"
}
# ^===========================================================================^


# ----- Functions used in the Docker container build script. -----

do_container_detect() {

  echo
  uname -a

  CONTAINER_DISTRO_NAME=""
  CONTAINER_UNAME="$(uname)"
  if [ "${CONTAINER_UNAME}" == "Darwin" ]
  then
        CONTAINER_BITS="64"
        CONTAINER_MACHINE="x86_64"

        CONTAINER_DISTRO_NAME=Darwin
        CONTAINER_DISTRO_LC_NAME=darwin

  elif [ "${CONTAINER_UNAME}" == "Linux" ]
  then
        # ----- Determine distribution name and word size -----

        set +e
        CONTAINER_DISTRO_NAME=$(lsb_release -si)
        set -e

        if [ -z "${CONTAINER_DISTRO_NAME}" ]
        then
          echo "Please install the lsb core package and rerun."
          CONTAINER_DISTRO_NAME="Linux"
        fi

        if [ "$(uname -m)" == "x86_64" ]
        then
          CONTAINER_BITS="64"
          CONTAINER_MACHINE="x86_64"
        elif [ "$(uname -m)" == "i686" ]
        then
          CONTAINER_BITS="32"
          CONTAINER_MACHINE="i386"
        else
          echo "Unknown uname -m $(uname -m)"
          exit 1
        fi

        CONTAINER_DISTRO_LC_NAME=$(echo ${CONTAINER_DISTRO_NAME} | tr "[:upper:]" "[:lower:]")

  else
        echo "Unknown uname ${CONTAINER_UNAME}"
        exit 1
  fi

  cross_compile_prefix=""
  if [ "${target_os}" == "win" ]
  then
    target_folder="${target_os}${target_bits}"

    # For Windows targets, decide which cross toolchain to use.
    if [ ${target_bits} == "32" ]
    then
      cross_compile_prefix="i686-w64-mingw32"
    elif [ ${target_bits} == "64" ]
    then
      cross_compile_prefix="x86_64-w64-mingw32"
    fi

    BUILD="$(${XBB_FOLDER}/share/libtool/build-aux/config.guess)"
    HOST="${cross_compile_prefix}"
    TARGET=${HOST}

  elif [ "${target_os}" == "osx" ]
  then

    target_bits="64" # Only 64-bts macOS binaries
    target_folder="${target_os}"

    BUILD="$(${HOME}/opt/homebrew/gme/share/libtool/build-aux/config.guess)"
    HOST=${BUILD}
    TARGET=${HOST}

  elif [ "${target_os}" == "linux" ]
  then

    if [ "${target_bits}" == "-" ]
    then
      target_bits="${CONTAINER_BITS}"
    else
      if [ "${target_bits}" != "${CONTAINER_BITS}" ]
      then
        echo "Cannot build ${target_bits} target on the ${CONTAINER_BITS} container."
        exit 1
      fi
    fi

    target_folder=${CONTAINER_DISTRO_LC_NAME}${target_bits:-""}

    BUILD="$(${XBB_FOLDER}/share/libtool/build-aux/config.guess)"
    HOST=${BUILD}
    TARGET=${HOST}

  else
    echo "Unsupported target os ${target_os}"
    exit 1
  fi

  build_folder_path="${container_build_folder_path}/${target_folder}"
  install_folder="${container_install_folder_path}/${target_folder}"
  output_folder_path="${container_output_folder_path}/${target_folder}"

  echo
  echo "Container running on ${CONTAINER_DISTRO_NAME} ${CONTAINER_BITS}-bits."

}

# v===========================================================================v
do_container_copy_info() {

      echo
      echo "Copying info files..."

      if [ -f "${git_folder_path}/gnu-mcu-eclipse/info/INFO.md" ]
      then
        /usr/bin/install -cv -m 644 "${git_folder_path}/gnu-mcu-eclipse/info/INFO.md" \
          "${install_folder}/${APP_LC_NAME}/README.md"
        do_unix2dos "${install_folder}/${APP_LC_NAME}/README.md"
      elif [ -f "${git_folder_path}/gnu-mcu-eclipse/info/INFO.txt" ]
      then
        /usr/bin/install -cv -m 644 "${git_folder_path}/gnu-mcu-eclipse/info/INFO.txt" \
          "${install_folder}/${APP_LC_NAME}/INFO.txt"
        do_unix2dos "${install_folder}/${APP_LC_NAME}/INFO.txt"
      else
        /usr/bin/install -cv -m 644 "${git_folder_path}/gnu-mcu-eclipse/info/INFO-${target_os}.txt" \
          "${install_folder}/${APP_LC_NAME}/INFO.txt"
        do_unix2dos "${install_folder}/${APP_LC_NAME}/INFO.txt"
      fi

      mkdir -p "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse"

      if [ -f "${git_folder_path}/gnu-mcu-eclipse/info/BUILD-${target_os}.md" ]
      then
        /usr/bin/install -cv -m 644 "${git_folder_path}/gnu-mcu-eclipse/info/BUILD-${target_os}.md" \
          "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse/BUILD.md"
        do_unix2dos "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse/BUILD.md"
      else
        /usr/bin/install -cv -m 644 "${git_folder_path}/gnu-mcu-eclipse/info/BUILD-${target_os}.txt" \
          "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse/BUILD.txt"
        do_unix2dos "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse/BUILD.txt"
      fi

      /usr/bin/install -cv -m 644 "${git_folder_path}/gnu-mcu-eclipse/info/CHANGES.txt" \
        "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse/CHANGES.txt"
      do_unix2dos "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse/CHANGES.txt"

      # Copy the current build script
      /usr/bin/install -cv -m 644 "${work_folder_path}/scripts/build-${APP_LC_NAME}.sh" \
        "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse/build.sh"
      do_unix2dos "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse/build.sh"

      # Copy the current build helper script
      /usr/bin/install -cv -m 644 "${work_folder_path}/scripts/build-helper.sh" \
        "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse/build-helper.sh"
      do_unix2dos "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse/build-helper.sh"

      if [ -f "${output_folder_path}/config.log" ]
      then
        /usr/bin/install -cv -m 644 "${output_folder_path}/config.log" \
        "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse/config.log"
        do_unix2dos "${install_folder}/${APP_LC_NAME}/gnu-mcu-eclipse/config.log"
      fi
}

# v===========================================================================v
do_container_create_distribution() {

  mkdir -p "$(dirname "${output_folder_path}")"

      if [ "${target_os}" == "win" ]
      then

        echo
        echo "Creating archive..."
        echo

        distribution_file="${distribution_folder}/gnu-mcu-eclipse-${APP_LC_NAME}-${distribution_file_version}-${target_folder}.zip"

        rm -rfv "${install_folder}/archive/"
        # The archive will use the 'GNU MCU Eclipse/app/version' hierarchy.
        mkdir -p "${install_folder}/archive/GNU MCU Eclipse/${APP_UC_NAME}/${distribution_file_version}"
        cp -r "${install_folder}/${APP_LC_NAME}"/* "${install_folder}/archive/GNU MCU Eclipse/${APP_UC_NAME}/${distribution_file_version}"

        pushd "${install_folder}/archive"
        zip -r -q "${distribution_file}" "GNU MCU Eclipse"
        popd

        pushd "$(dirname ${distribution_file})"
        do_compute_sha shasum -a 256 -p "$(basename ${distribution_file})"
        popd

        if false
        then

        echo
        echo "Creating setup..."
        echo

        distribution_setup="${distribution_folder}/gnu-mcu-eclipse-${APP_LC_NAME}-${distribution_file_version}-${target_folder}-setup.exe"

        if [ ! -f "${install_folder}/${APP_LC_NAME}/licenses/LICENSE" ]
        then
          echo "Missing LICENSE"
          exit 1
        fi
        
        nsis_folder="${git_folder_path}/gnu-mcu-eclipse/nsis"
        nsis_file="${nsis_folder}/nsis.nsi"

        ls -lR "${nsis_folder}"

        cd "${build_folder_path}"
        makensis -V4 -NOCD \
          -DINSTALL_FOLDER="${install_folder}/${APP_LC_NAME}" \
          -DNSIS_FOLDER="${nsis_folder}" \
          -DOUTFILE="${distribution_setup}" \
          -DW${target_bits} \
          -DBITS=${target_bits} \
          -DVERSION=${distribution_file_version} \
          "${nsis_file}"
        result="$?"

        pushd "$(dirname ${distribution_setup})"
        do_compute_sha shasum -a 256 -p "$(basename ${distribution_setup})"
        popd

        fi

      elif [ "${target_os}" == "linux" ]
      then

        echo
        echo "Creating tgz archive..."
        echo

        distribution_file="${distribution_folder}/gnu-mcu-eclipse-${APP_LC_NAME}-${distribution_file_version}-${target_folder}.tgz"

        rm -rf "${install_folder}/archive/"
        mkdir -p "${install_folder}/archive/gnu-mcu-eclipse/${APP_LC_NAME}/${distribution_file_version}"
        cp -r "${install_folder}/${APP_LC_NAME}"/* "${install_folder}/archive/gnu-mcu-eclipse/${APP_LC_NAME}/${distribution_file_version}"
        cd "${install_folder}/archive"
        tar -c -z -f "${distribution_file}" --owner root --group root gnu-mcu-eclipse

        pushd "$(dirname ${distribution_file})"
        do_compute_sha shasum -a 256 -p "$(basename ${distribution_file})"
        popd

      elif [ "${target_os}" == "osx" ]
      then

        echo
        echo "Creating archive..."
        echo

        distribution_file="${distribution_folder}/gnu-mcu-eclipse-${APP_LC_NAME}-${distribution_file_version}-${target_folder}.tgz"

        rm -rf "${install_folder}/archive/"
        # The archive will use the 'gnu-mcu-eclipse/app/version' hierarchy.
        mkdir -p "${install_folder}/archive/gnu-mcu-eclipse/${APP_LC_NAME}/${distribution_file_version}"
        cp -r "${install_folder}/${APP_LC_NAME}"/* "${install_folder}/archive/gnu-mcu-eclipse/${APP_LC_NAME}/${distribution_file_version}"

        pushd "${install_folder}/archive"
        tar -c -z -f "${distribution_file}" gnu-mcu-eclipse
        popd

        pushd "$(dirname ${distribution_file})"
        do_compute_sha shasum -a 256 -p "$(basename ${distribution_file})"
        popd

        if false
        then
        
        echo
        echo "Creating installer package..."
        echo

        distribution_pkg="${distribution_folder}/gnu-mcu-eclipse-${APP_LC_NAME}-${distribution_file_version}-${target_folder}.pkg"

        distribution_install_folder=${distribution_install_folder:-"/Applications/GNU MCU Eclipse/${APP_NAME}"}

        # Create the installer package, with content from the
        # ${distribution_install_folder}/${APP_LC_NAME} folder.
        # The ID includes the version, which is a kludge to prevent the
        # installer to remove preious versions.
        # The "${distribution_install_folder:1}" is a substring that skips first char.
        cd "${work_folder_path}"
        pkgbuild \
          --root "${install_folder}/${APP_LC_NAME}" \
          --identifier "ilg.gnu-mcu-eclipse.${APP_LC_NAME}.${DISTRIBUTION_FILE_DATE}" \
          --install-location "${distribution_install_folder:1}/${distribution_file_version}" \
          "${distribution_pkg}"

        pushd "$(dirname ${distribution_pkg})"
        do_compute_sha shasum -a 256 -p "$(basename ${distribution_pkg})"
        popd
        
        fi

        echo
        ls -l "${install_folder}/${APP_LC_NAME}/bin"

      fi

      if [ "${docker_container_name}" != "" -a "${host_uname}" == "Linux" ]
      then
        # Set the owner of the folder and files created by the docker debian 
        # container to match the user running the build script on the host. 
        # When run on linux host, these folders and their content remain owned 
        # by root if this is not done. However, when host is osx (mac os), the 
        # owner produced by boot2docker is the same as the osx user, so an 
        # ownership change is not done. 
        echo
        echo "Changing owner to non-root Linux user..."
        if [ -d "${build_folder_path}" ]
        then
          chown -R ${user_id}:${group_id} "${build_folder_path}"
        fi
        if [ -d "${install_folder}" ]
        then
          chown -R ${user_id}:${group_id} "${install_folder}"
        fi
        chown -R ${user_id}:${group_id} "${work_folder_path}/${DEPLOY_FOLDER_NAME}"
      fi
}

# v===========================================================================v
do_container_copy_install() {
  if [ "${container_install_folder_path}" != "${shared_install_folder_path}" ]
  then
    echo
    echo "Copying install to shared folder..."
    mkdir -p "$(dirname ${shared_install_folder_path})"
    cp -R "${container_install_folder_path}" \
      "$(dirname ${shared_install_folder_path})"
  fi
}

do_check_application() {
  local executable_name=$1
  shift

  echo
  echo "Checking ${executable_name}..."

  if [ "${target_os}" == "win" ]
  then

    # Display some information about the created application.
    echo
    echo "DLLs:"
    set +e
    ${cross_compile_prefix}-objdump -x "${install_folder}/${APP_LC_NAME}/bin/${executable_name}.exe" | grep -i 'DLL Name'
    set -e
    result=0

  elif [ "${target_os}" == "linux" ]
  then

    # Display some information about the created application.
    pushd "${install_folder}/${APP_LC_NAME}/bin"
    rm -rf /tmp/mylibs
    echo
    echo "Libraries:"
    set +e
    echo "${executable_name}"
    readelf -d "${executable_name}" | egrep -i 'library|dynamic'
    readelf -d "${executable_name}" | egrep -i 'library|dynamic' | grep NEEDED >>/tmp/mylibs
    for f in `find . -type f -name '*.so*' -print`
    do
      echo "${f}"
      readelf -d "${f}" | egrep -i 'library|dynamic'
      readelf -d "${f}" | egrep -i 'library|dynamic' | grep NEEDED >>/tmp/mylibs
    done
    echo
    echo "Needed:"
    sort -u /tmp/mylibs

    readelf -d "${executable_name}" | egrep -i 'runpath'
    rpl=$(readelf -d "${executable_name}" | egrep -i 'runpath' | sed -e 's/.*\[\(.*\)\]/\1/')
    if [ "$rpl" != '$ORIGIN' -a "$rpl" != '' ]
    then
      echo "Wrong runpath $rpl"
      exit 1
    fi
    popd
    set -e
    echo
    # ls -l "${install_folder}/${APP_LC_NAME}/bin"

    # Check if the application starts (if all dynamic libraries are available).
    echo
    "${install_folder}/${APP_LC_NAME}/bin/${executable_name}" $@
    result="$?"

  elif [ "${target_os}" == "osx" ]
  then

    otool -L "${install_folder}/${APP_LC_NAME}/bin/${executable_name}"
    echo
    # Check if the application starts (if all dynamic libraries are available).
    "${install_folder}/${APP_LC_NAME}/bin/${executable_name}" $@
    result="$?"
  
  fi
}
# v===========================================================================v
do_container_completed() {

      echo
      if [ "${result}" == "0" ]
      then
        echo "Build completed."
        echo
        echo "Distribution file ${distribution_file} created."
        ls -l "${distribution_file}"

        if false
        then

        if [ "${target_os}" == "osx" ]
        then
          echo "Distribution file ${distribution_pkg} created."
          ls -l "${distribution_pkg}"
        elif [ "${target_os}" == "win" ]
        then
          echo "Distribution file ${distribution_setup} created."
          ls -l "${distribution_setup}"
        fi

        fi
      else
        echo "Build failed."
      fi

      echo
      echo "Script \"$(basename $0)\" completed."
}

# v===========================================================================v
do_download_() {

  while [ $# -gt 0 ]
  do
    case "$1" in
      --url)
        url="$2"
        shift 2
        ;;
      --download-folder)
        download_folder="$2"
        shift 2
        ;;
      --archive-name)
        archive_name="$2"
        shift 2
        ;;
      *)
        echo "Unknown option $1, exit."
        exit 1
    esac
  done

  if [ ! -f "${download_folder}/${archive_name}" ]
  then
    mkdir -p "${download_folder}"

    cd "${download_folder}"

    echo
    echo "Downloading ${url}..."
    curl -L "${url}" --output "${archive_name}"
  fi

}

# v===========================================================================v
do_container_linux_copy_user_so() {
  # $1 = dll name

  ILIB=$(find ${install_folder}/lib* -type f -name $1'.so.*.*' -print)
  if [ ! -z "${ILIB}" ]
  then
    echo "Found user ${ILIB}"

    ihead=$(echo "${ILIB}" | head -n 1)
    # Add "runpath" in library with value $ORIGIN.
    patchelf --set-rpath '$ORIGIN' "${ihead}"

    ILIB_BASE="$(basename ${ihead})"
    /usr/bin/install -v -c -m 644 "${ihead}" "${install_folder}/${APP_LC_NAME}/bin"
    ILIB_SHORT="$(echo $ILIB_BASE | sed -e 's/\([[:alnum:]]*\)[.]\([[:alnum:]]*\)[.]\([[:digit:]]*\)[.].*/\1.\2.\3/')"
    (
      cd "${install_folder}/${APP_LC_NAME}/bin"
      rm --force "${ILIB_SHORT}"
      ln -sv "${ILIB_BASE}" "${ILIB_SHORT}"
    )
    ILIB_SHORT="$(echo $ILIB_BASE | sed -e 's/\([[:alnum:]]*\)[.]\([[:alnum:]]*\)[.]\([[:digit:]]*\)[.].*/\1.\2/')"
    (
      cd "${install_folder}/${APP_LC_NAME}/bin"
      rm --force "${ILIB_SHORT}"
      ln -sv "${ILIB_BASE}" "${ILIB_SHORT}"
    )
  else
    ILIB=$(find ${install_folder}/lib* -type f -name $1'.so.*' -print)
    if [ ! -z "${ILIB}" ]
    then
      echo "Found user 2 ${ILIB}"

      ihead=$(echo "${ILIB}" | head -n 1)
      # Add "runpath" in library with value $ORIGIN.
      patchelf --set-rpath '$ORIGIN' "${ihead}"

      ILIB_BASE="$(basename ${ihead})"
      /usr/bin/install -v -c -m 644 "${ihead}" "${install_folder}/${APP_LC_NAME}/bin"
      ILIB_SHORT="$(echo $ILIB_BASE | sed -e 's/\([[:alnum:]]*\)[.]\([[:alnum:]]*\)[.]\([[:digit:]]*\).*/\1.\2/')"
      echo "${ILIB_SHORT}"
      (
        cd "${install_folder}/${APP_LC_NAME}/bin"
        rm --force "${ILIB_SHORT}"
        ln -sv "${ILIB_BASE}" "${ILIB_SHORT}"
      )
    else
      ILIB=$(find ${install_folder}/lib* -type f -name $1'.so' -print)
      if [ ! -z "${ILIB}" ]
      then
        echo "Found user 3 ${ILIB}"

        ihead=$(echo "${ILIB}" | head -n 1)
        # Add "runpath" in library with value $ORIGIN.
        patchelf --set-rpath '$ORIGIN' "${ihead}"

        ILIB_BASE="$(basename ${ihead})"
        /usr/bin/install -v -c -m 644 "${ihead}" "${install_folder}/${APP_LC_NAME}/bin"
      else
        echo $1 not found
        exit 1
      fi
    fi
  fi
}

# v===========================================================================v
do_container_linux_copy_system_so() {
  # $1 = dll name

  ILIB=$(find /lib* -type f -name $1'.so.*.*' -print)
  if [ ! -z "${ILIB}" ]
  then
    echo "Found system ${ILIB}"
    ihead=$(echo "${ILIB}" | head -n 1)
    ILIB_BASE="$(basename ${ihead})"
    /usr/bin/install -v -c -m 644 "${ihead}" "${install_folder}/${APP_LC_NAME}/bin"
    ILIB_SHORT="$(echo $ILIB_BASE | sed -e 's/\([[:alnum:]]*\)[.]\([[:alnum:]]*\)[.]\([[:digit:]]*\)[.].*/\1.\2.\3/')"
    (
      cd "${install_folder}/${APP_LC_NAME}/bin"
      rm --force "${ILIB_SHORT}"
      ln -sv "${ILIB_BASE}" "${ILIB_SHORT}"
    )
    ILIB_SHORT="$(echo $ILIB_BASE | sed -e 's/\([[:alnum:]]*\)[.]\([[:alnum:]]*\)[.]\([[:digit:]]*\)[.].*/\1.\2/')"
    (
      cd "${install_folder}/${APP_LC_NAME}/bin"
      rm --force "${ILIB_SHORT}"
      ln -sv "${ILIB_BASE}" "${ILIB_SHORT}"
    )
  else
    ILIB=$(find /lib* -type f -name $1'.so.*' -print)
    if [ ! -z "${ILIB}" ]
    then
      echo "Found system 2 ${ILIB}"
      ihead=$(echo "${ILIB}" | head -n 1)
      ILIB_BASE="$(basename ${ihead})"
      /usr/bin/install -v -c -m 644 "${ihead}" "${install_folder}/${APP_LC_NAME}/bin"
      ILIB_SHORT="$(echo $ILIB_BASE | sed -e 's/\([[:alnum:]]*\)[.]\([[:alnum:]]*\)[.]\([[:digit:]]*\).*/\1.\2/')"
      echo "${ILIB_SHORT}"
      (
        cd "${install_folder}/${APP_LC_NAME}/bin"
        rm --force "${ILIB_SHORT}"
        ln -sv "${ILIB_BASE}" "${ILIB_SHORT}"
      )
    else
      ILIB=$(find /lib* -type f -name $1'.so' -print)
      if [ ! -z "${ILIB}" ]
      then
        echo "Found system 3 ${ILIB}"
        ihead=$(echo "${ILIB}" | head -n 1)
        ILIB_BASE="$(basename ${ihead})"
        /usr/bin/install -v -c -m 644 "${ihead}" "${install_folder}/${APP_LC_NAME}/bin"
      else
        echo $1 not found
        exit 1
      fi
    fi
  fi
}

# v===========================================================================v
do_container_linux_copy_librt_so() {
  ILIB=$(find /lib* -type f -name 'librt-*.so' -print | grep -v i686)
  if [ ! -z "${ILIB}" ]
  then
    echo "Found system ${ILIB}"
    ihead=$(echo "${ILIB}" | head -n 1)
    ILIB_BASE="$(basename ${ihead})"
    /usr/bin/install -v -c -m 644 "${ihead}" \
    "${install_folder}/${APP_LC_NAME}/bin"
    (
      cd "${install_folder}/${APP_LC_NAME}/bin"
      rm --force "librt.so.1"
      ln -sv "${ILIB_BASE}" "librt.so.1"
    )
    (
      cd "${install_folder}/${APP_LC_NAME}/bin"
      rm --force "librt.so"
      ln -sv "${ILIB_BASE}" "librt.so"
    )
  else
    echo
    echo "WARNING: librt.so not copied locally!"
    exit 1
  fi
}

# v===========================================================================v
do_container_linux_check_libs() {

  local fn
  if [ $# -gt 0 ]
  then
    fn=$1
  else
    fn=${ILIB}
  fi

  echo "${fn}"
  readelf -d "${fn}" | egrep -i 'library|dynamic'
  set +e
  local unxp=$(readelf -d "${fn}" | egrep -i 'library|dynamic' | grep -e "NEEDED" | grep -e "macports" -e "homebrew" -e "opt" -e "install")
  set -e
  #echo "|${unxp}|"
  if [ ! -z "$unxp" ]
  then
     exit 1
  fi
}

# v===========================================================================v
# $1 = dll name
do_container_win_copy_gcc_dll() {

  # First try Ubuntu specific locations,
  # then do a long full search.

  if [ -f "${XBB_FOLDER}/${cross_compile_prefix}/lib/$1" ]
  then
    cp -v "${XBB_FOLDER}/${cross_compile_prefix}/lib/$1" \
      "${install_folder}/${APP_LC_NAME}/bin"
  elif [ -f "/usr/lib/gcc/${cross_compile_prefix}/${CROSS_GCC_VERSION}/$1" ]
  then
    cp -v "/usr/lib/gcc/${cross_compile_prefix}/${CROSS_GCC_VERSION}/$1" \
      "${install_folder}/${APP_LC_NAME}/bin"
  elif [ -f "/usr/lib/gcc/${cross_compile_prefix}/${CROSS_GCC_VERSION_SHORT}/$1" ]
  then
    cp -v "/usr/lib/gcc/${cross_compile_prefix}/${CROSS_GCC_VERSION_SHORT}/$1" \
      "${install_folder}/${APP_LC_NAME}/bin"
  elif [ -f "/usr/lib/gcc/${cross_compile_prefix}/${CROSS_GCC_VERSION_SHORT}${SUBLOCATION}/$1" ]
  then
    cp -v "/usr/lib/gcc/${cross_compile_prefix}/${CROSS_GCC_VERSION_SHORT}${SUBLOCATION}/$1" \
      "${install_folder}/${APP_LC_NAME}/bin"
  else
    echo "Searching /usr for $1..."
    SJLJ_PATH=$(find /usr \! -readable -prune -o -name $1 -print | grep ${cross_compile_prefix})
    cp -v ${SJLJ_PATH} "${install_folder}/${APP_LC_NAME}/bin"
  fi
}

# v===========================================================================v
do_container_win_copy_gcc_dlls() {

  if [ -d "/usr/lib/gcc/${cross_compile_prefix}/${CROSS_GCC_VERSION}/" ]
  then
    cp -v "/usr/lib/gcc/${cross_compile_prefix}/${CROSS_GCC_VERSION}/"*.dll \
      "${install_folder}/${APP_LC_NAME}/bin"
  elif [ -d "/usr/lib/gcc/${cross_compile_prefix}/${CROSS_GCC_VERSION_SHORT}/" ]
  then
    cp -v "/usr/lib/gcc/${cross_compile_prefix}/${CROSS_GCC_VERSION_SHORT}/"*.dll \
      "${install_folder}/${APP_LC_NAME}/bin"
  elif [ -d "/usr/lib/gcc/${cross_compile_prefix}/${CROSS_GCC_VERSION_SHORT}${SUBLOCATION}/" ]
  then
    cp -v "/usr/lib/gcc/${cross_compile_prefix}/${CROSS_GCC_VERSION_SHORT}${SUBLOCATION}/"*.dll \
      "${install_folder}/${APP_LC_NAME}/bin"
  else
    echo "No DLLs"
    exit 1
  fi
}

# v===========================================================================v
do_container_win_copy_libwinpthread_dll() {

  if [ -f "${XBB_FOLDER}/${cross_compile_prefix}"/bin/libwinpthread-1.dll ]
  then
    cp "${XBB_FOLDER}/${cross_compile_prefix}"/bin/libwinpthread-1.dll \
      "${install_folder}/${APP_LC_NAME}/bin"
  else
    echo "No libwinpthread-1.dll"
    exit 1
  fi
}

# v===========================================================================v
do_container_win_check_libs() {

  local fn
  if [ $# -gt 0 ]
  then
    fn=$1
  else
    fn=${ILIB}
  fi

  echo "${fn}"
  ${cross_compile_prefix}-objdump -x "${install_folder}/${APP_LC_NAME}/bin/${fn}" | grep -i 'DLL Name'

  set +e
  local unxp=$(${cross_compile_prefix}-objdump -x "${install_folder}/${APP_LC_NAME}/bin/${fn}" | grep -i 'DLL Name' | grep -e "macports" -e "homebrew" -e "opt" -e "install")
  set -e
  #echo "|${unxp}|"
  if [ ! -z "$unxp" ]
  then
    exit 1
  fi
}

# v===========================================================================v
# $1 - absolute path to input folder
# $2 - name of output folder below INSTALL_FOLDER
do_container_copy_license() {

  # Iterate all files in a folder and install some of them in the
  # destination folder
  echo "$2"
  for f in "$1/"*
  do
    if [ -f "$f" ]
    then
      if [[ "$f" =~ AUTHORS.*|NEWS.*|COPYING.*|README.*|LICENSE.*|FAQ.*|DEPENDENCIES.*|THANKS.* ]]
      then
        /usr/bin/install -d -m 0755 "${install_folder}/${APP_LC_NAME}/licenses/$2"
        /usr/bin/install -v -c -m 644 "$f" "${install_folder}/${APP_LC_NAME}/licenses/$2"
      fi
    fi
  done
}

# v===========================================================================v
# $1 = dylib name (like libSDL-1.2.0.dylib)
do_container_mac_copy_built_lib() {

  echo
  ILIB=$1
  cp -v "${install_folder}/lib/${ILIB}" "${install_folder}/${APP_LC_NAME}/bin/${ILIB}"
  # otool -L "${install_folder}/${APP_LC_NAME}/bin/${ILIB}"
  install_name_tool -id "${ILIB}" "${install_folder}/${APP_LC_NAME}/bin/${ILIB}"
}

# v===========================================================================v
# $1 = dylib name (like libSDL-1.2.0.dylib)
do_container_mac_change_built_lib() {
  install_name_tool -change "${install_folder}/lib/$1" \
    "@executable_path/$1" \
    "${install_folder}/${APP_LC_NAME}/bin/${ILIB}"
}

# v===========================================================================v
# $1 = dylib name (like libXau.6.dylib)
# $2 = folder (like /opt/X11/lib)
do_container_mac_change_lib() {
  install_name_tool -change "$2/$1" \
    "@executable_path/$1" \
    "${install_folder}/${APP_LC_NAME}/bin/${ILIB}"
}

# v===========================================================================v
# $1 = dylib name (like libXau.6.dylib)
# $2 = folder (like /opt/X11/lib)
do_container_mac_copy_lib() {

  echo
  ILIB=$1
  cp -v "${2}/${ILIB}" "${install_folder}/${APP_LC_NAME}/bin/${ILIB}"
  # otool -L "${install_folder}/${APP_LC_NAME}/bin/${ILIB}"
  install_name_tool -id "${ILIB}" "${install_folder}/${APP_LC_NAME}/bin/${ILIB}"
}

# v===========================================================================v
do_container_mac_check_libs() {

  local fn
  if [ $# -gt 0 ]
  then
    fn=$1
  else
    fn=${ILIB}
  fi

  otool -L "${install_folder}/${APP_LC_NAME}/bin/${fn}"
  set +e
  local unxp=$(otool -L "${install_folder}/${APP_LC_NAME}/bin/${fn}" | sed '1d' | grep -e "macports" -e "homebrew" -e "opt" -e "install")
  set -e
  # echo "|${unxp}|"
  if [ ! -z "$unxp" ]
  then
    exit 1
  fi
}

# ^===========================================================================^


# ----- General usage functions. -----
# v===========================================================================v
do_unix2dos() {

  if [ "${target_os}" == "win" ]
  then
    while (($#))
    do
      unix2dos "$1"
      shift
    done
  fi
}

# v===========================================================================v
do_strip_() {

  strip_app="$1"
  shift

  echo

  for f in "$@"
  do
    base="$(basename $f)"
    tmp_file=$(mktemp /tmp/${base}.XXXXXX)

    cp "$f" "${tmp_file}"
    echo "${strip_app}" "$f"
    "${strip_app}" "${tmp_file}"
    cp "${tmp_file}" "$f"

    rm --force "${tmp_file}"
  done
}

# v===========================================================================v
do_compute_sha() {
  # $1 shasum program
  # $2.. options
  # ${!#} file

  file=${!#}
  sha_file="${file}.sha"
  "$@" >"${sha_file}"
  echo "SHA: $(cat ${sha_file})"
}

# v===========================================================================v
# Copy one folder to another
do_copy_dir() {
    set +u
    mkdir -p "$2"

    (cd "$1" && tar cf - .) | (cd "$2" && tar xf -)
    set -u
}


# v===========================================================================v

do_copy_libs() {
    local src_folder="$1"
    local dst_folder="$2"

    cp -v -f "${src_folder}/libc.a" "${dst_folder}/libc_nano.a"
    cp -v -f "${src_folder}/libg.a" "${dst_folder}/libg_nano.a"
    if [ -f "${src_folder}/libstdc++.a" ]
    then
      cp -v -f "${src_folder}/libstdc++.a" "${dst_folder}/libstdc++_nano.a"
    fi
    if [ -f "${src_folder}/libsupc++.a" ]
    then
      cp -v -f "${src_folder}/libsupc++.a" "${dst_folder}/libsupc++_nano.a"
    fi
    # cp -f "${src_folder}/librdimon.a" "${dst_folder}/librdimon_nano.a"

    # cp -v -f "${src_folder}/nano.specs" "${dst_folder}/"
    # On RISC-V, the nano.specs file is not generated by the GCC build,
    # try to create it manually.
    
    # Quoted, do not substitute.
    cat <<'__EOF__' >"${dst_folder}/nano.specs"
%rename link                 nano_link
%rename link_gcc_c_sequence  nano_link_gcc_c_sequence
%rename cpp_unique_options   nano_cpp_unique_options

*cpp_unique_options:
-isystem =/include/newlib-nano %(nano_cpp_unique_options)

*nano_libc:
-lc_nano

*nano_libgloss:
%{specs=nosys.specs:-lnosys}

*link_gcc_c_sequence:
%(nano_link_gcc_c_sequence) --start-group %G %(nano_libc) %(nano_libgloss) --end-group

*link:
%(nano_link) %:replace-outfile(-lc -lc_nano) %:replace-outfile(-lg -lg_nano) %:replace-outfile(-lrdimon -lrdimon_nano) %:replace-outfile(-lstdc++ -lstdc++_nano) %:replace-outfile(-lsupc++ -lsupc++_nano)

*lib:
%{!shared:%{g*:-lg_nano} %{!p:%{!pg:-lc_nano}}%{p:-lc_p}%{pg:-lc_p}}

__EOF__

    # cp -f "${src_folder}/rdimon.specs" "${dst_folder}/"
    cp -v -f "${src_folder}/nosys.specs" "${dst_folder}/"
    cp -v -f "${src_folder}/"*crt0.o "${dst_folder}/"
}

# Copy target libraries from each multilib directories.
# $1=source
# $2=destination
# $3=target gcc
do_copy_multi_libs() {
    local -a multilibs
    local multilib
    local multi_folder
    local src_folder="$1"
    local dst_folder="$2"
    local gcc_target="$3"

    echo ${gcc_target}
    multilibs=( $("${gcc_target}" -print-multi-lib 2>/dev/null) )
    if [ ${#multilibs[@]} -gt 0 ]
    then
      for multilib in "${multilibs[@]}"
      do
        multi_folder="${multilib%%;*}"
        do_copy_libs "${src_folder}/${multi_folder}" \
          "${dst_folder}/${multi_folder}"
      done
    else
      do_copy_libs "${src_folder}" "${dst_folder}"
    fi

}

# ^===========================================================================^

# Continue in calling script.
