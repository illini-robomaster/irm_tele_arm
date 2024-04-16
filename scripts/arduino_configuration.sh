#!/bin/bash
HELP=$(cat << EOF
Usage: $(basename "$0") [options] {[s]etup,[c]lean,[r]esetup,[u]pdate}

  -h, --help            Show this message
  -m, --metrics         Enable metrics
  -u, --notify-update   Enable update notifications

Sets up boards and compilation for Arduino OpenRB-150.
Requires \`arduino-cli\`.
EOF
)
script_name=$(basename "$0")
script_file=$(realpath "$0")
script_dir=$(realpath "${script_file}" | xargs dirname)
arduino_dir=$(dirname ${script_dir})/arduino
arduino_conf="${arduino_dir}"/arduino-cli.yaml

arduino_data="${arduino_dir}"/.arduino15
arduino_downloads="${arduino_data}"/staging

acli="arduino-cli --config-file ${arduino_conf}"


function printhelp() {
    echo "$HELP"
    exit ${1:-0}
}


function setup() {
    metrics=${1/-}
    notify_update=${2/-}
    if [ -f "${arduino_conf}" ]; then
        echo "${arduino_conf}" already exists.
        echo Run \`${script_name} clean\` before rerunning.
        exit 1
    fi

    echo +++ Creating new configuration file at "${arduino_conf}"
    arduino-cli config init --dest-file "${arduino_conf}"

    echo
    echo +++ +++ CONFIGURATION
    if [ "${metrics}" ]; then
        echo +++ Disable metrics "(default)"
        ${acli} config set metrics.enabled false
    fi
    if [ "${notify_update}" ]; then
        echo +++ Disable update notifications "(default)"
        ${acli} config set updater.enable_notification false
    fi
    echo +++ Setting user directory to "${arduino_dir}"
    ${acli} config set directories.user "${arduino_dir}"
    echo +++ Setting data directory to "${arduino_data}"
    ${acli} config set directories.data "${arduino_data}"
    echo +++ Setting downloads directory to "${arduino_downloads}"
    ${acli} config set directories.downloads "${arduino_downloads}"
    echo +++ Add board manager for OpenRB-150 '(at https://raw.githubusercontent.com/ROBOTIS-GIT/OpenRB-150/master/package_openrb_index.json)'
    ${acli} config set board_manager.additional_urls "https://raw.githubusercontent.com/ROBOTIS-GIT/OpenRB-150/master/package_openrb_index.json"
    echo +++ Current config:
    ${acli} config dump

    echo
    echo +++ +++ INSTALL BOARDS
    echo +++ Install arduino:samd
    ${acli} core install arduino:samd
    echo +++ Install OpenRB-150:samd
    ${acli} core install OpenRB-150:samd
    echo +++ Current boards:
    ${acli} core list
}

function clean() {
    echo Cleaning.
    rm -fv "${arduino_conf}"
    rm -rfv "${arduino_data}"
    echo Done.
}

function update() {
    if [ ! -f "${arduino_conf}" ]; then
        echo "${arduino_conf}" does not exist.
        echo Run \`${script_name} setup\` first.
        exit 1
    fi
    ${acli} update
    ${acli} upgrade
}

metrics=-
notify_update=-
function run() {
    type arduino-cli >& /dev/null || {
        echo 'Exiting: `arduino-cli` is not installed.'
        exit 1
    }

    while [ $# -gt 0 ]; do
        case $1 in
            -m|--metrics)
                metrics=1;
                shift;;
            -u|--notify-update)
                notify_update=1;
                shift;;
            s|setup)
                setup ${metrics} ${notify_update};
                shift;;
            c|clean)
                clean;
                shift;;
            r|resetup)
                clean;
                setup ${metrics} ${notify_update};
                shift;;
            u|update)
                update;
                shift;;
            *)
                printhelp;
                shift;;
            esac
    done
}

run "${@:-help}"
