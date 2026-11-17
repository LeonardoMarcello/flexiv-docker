#!/bin/bash

# ------------------------------------------------------------
#   INSTALL: scarica flexiv_ros2 (humble-v1.7, RDK 1.7) da Flexiv, applica le nostre modifiche
#   e costruisce l'immagine Docker
# ------------------------------------------------------------
#
# Uso: ./install.sh [--no-cache] [--skip-build] [--reset]
#   --no-cache    ricostruisce l'immagine da zero (docker compose build --no-cache)
#   --skip-build  prepara solo i sorgenti, senza costruire l'immagine
#   --reset       riscarica flexiv_ros2 da zero (la cartella esistente viene cancellata)
#
# Contenuto di questo repository:
#   src/      i nostri pacchetti ROS 2 (controllori, simulazione Gazebo)
#   overlay/  file che sostituiscono quelli di flexiv_ros2 (stessi percorsi relativi)
# flexiv_ros2/ non è versionato qui: viene scaricato da Flexiv da questo script.

set -euo pipefail

REPO_URL="https://github.com/flexivrobotics/flexiv_ros2.git"
TAG="humble-v1.7"
RDK_VERSION="1.7.0"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${SCRIPT_DIR}/flexiv_ros2"
OVERLAY_DIR="${SCRIPT_DIR}/overlay"
SRC_DIR="${SCRIPT_DIR}/src"

NO_CACHE=""
SKIP_BUILD=false
RESET=false
for arg in "$@"; do
    case "$arg" in
        --no-cache) NO_CACHE="--no-cache" ;;
        --skip-build) SKIP_BUILD=true ;;
        --reset) RESET=true ;;
        -h|--help) sed -n '3,16p' "$0"; exit 0 ;;
        *) echo "Argomento sconosciuto: $arg"; exit 1 ;;
    esac
done

info() { echo -e "\e[1;34m[install]\e[0m $*"; }
warn() { echo -e "\e[1;33m[install] ATTENZIONE:\e[0m $*"; }
fail() { echo -e "\e[1;31m[install] ERRORE:\e[0m $*" >&2; exit 1; }

# Prerequisiti
command -v git >/dev/null || fail "git non installato"
command -v docker >/dev/null || fail "docker non installato"
docker compose version >/dev/null 2>&1 || fail "docker compose (plugin v2) non disponibile"

# ------------------------------------------------------------
#   1. flexiv_ros2 humble-v1.7 + RDK 1.7 da Flexiv
# ------------------------------------------------------------
if [ "${RESET}" = true ] && [ -e "${REPO_DIR}" ]; then
    info "--reset: cancello ${REPO_DIR}"
    rm -rf "${REPO_DIR}"
fi

if [ ! -d "${REPO_DIR}/.git" ]; then
    [ -e "${REPO_DIR}" ] && fail "${REPO_DIR} esiste ma non è un repository git: spostalo o usa --reset"
    info "Scarico flexiv_ros2 (${TAG}) da ${REPO_URL}"
    git -c advice.detachedHead=false clone --branch "${TAG}" --depth 1 "${REPO_URL}" "${REPO_DIR}"
else
    info "flexiv_ros2 già presente: non lo riscarico (usa --reset per ricominciare da zero)"
    CURRENT=$(git -C "${REPO_DIR}" -c core.warnAmbiguousRefs=false describe --tags --always 2>/dev/null || echo "?")
    [ "${CURRENT}" = "${TAG}" ] || warn "flexiv_ros2 è su '${CURRENT}', atteso '${TAG}' (usa --reset per riallinearlo)"
fi

info "Scarico il submodule RDK"
git -C "${REPO_DIR}" submodule update --init --recursive

RDK_DIR="${REPO_DIR}/flexiv_hardware/rdk"
[ -f "${RDK_DIR}/thirdparty/build_and_install_dependencies.sh" ] || fail "submodule RDK incompleto in ${RDK_DIR}"
FOUND_RDK=$(sed -n 's/.*project(flexiv_rdk VERSION \([0-9.]*\)).*/\1/p' "${RDK_DIR}/CMakeLists.txt")
[ "${FOUND_RDK}" = "${RDK_VERSION}" ] || fail "attesa RDK ${RDK_VERSION}, trovata ${FOUND_RDK:-?}"
info "flexiv_ros2 ${TAG} | RDK ${FOUND_RDK}"

# ------------------------------------------------------------
#   2. Le nostre modifiche ai file di flexiv_ros2 (overlay/)
# ------------------------------------------------------------
if [ -d "${OVERLAY_DIR}" ]; then
    info "Applico overlay/ su flexiv_ros2:"
    (cd "${OVERLAY_DIR}" && find . -type f | sed 's#^\./#    #')
    cp -r "${OVERLAY_DIR}/." "${REPO_DIR}/"
fi

# ------------------------------------------------------------
#   3. I nostri pacchetti (src/)
# ------------------------------------------------------------
for pkg in custom_controllers thunder_controllers flexiv_gazebo; do
    [ -f "${SRC_DIR}/${pkg}/package.xml" ] || warn "manca il pacchetto src/${pkg}"
done
THUNDER_GEN="${SRC_DIR}/thunder_controllers/thunder"
ls "${THUNDER_GEN}"/*_generatedFiles/*_gen.cpp >/dev/null 2>&1 \
    || warn "nessun modello Thunder in ${THUNDER_GEN}: thunder_controllers verrà compilato senza controllori"

# ------------------------------------------------------------
#   4. Immagine Docker
# ------------------------------------------------------------
mkdir -p "${SCRIPT_DIR}/docker/history"

if [ "${SKIP_BUILD}" = true ]; then
    info "Sorgenti pronti (--skip-build): per costruire l'immagine esegui 'docker compose build'"
    exit 0
fi

info "Costruisco l'immagine Docker ${NO_CACHE}"
cd "${SCRIPT_DIR}"
docker compose build ${NO_CACHE}

info "Fatto. Avvia il container con ./run.sh"
