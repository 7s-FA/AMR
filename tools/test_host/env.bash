# source this file in the host terminal; no .bashrc changes required.
AMR_TEST_HOST_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
M1() { bash "$AMR_TEST_HOST_DIR/run.sh" M1 "$@"; }
M2() { bash "$AMR_TEST_HOST_DIR/run.sh" M2 "$@"; }
