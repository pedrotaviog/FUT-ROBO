#!/usr/bin/env sh

set -u

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=$(dirname -- "$SCRIPT_DIR")

find_python() {
    for candidate in python3 python; do
        if command -v "$candidate" >/dev/null 2>&1; then
            if "$candidate" -c "import tkinter" >/dev/null 2>&1; then
                printf '%s\n' "$candidate"
                return 0
            fi
        fi
    done
    return 1
}

PYTHON_BIN=$(find_python) || {
    printf '\n[ERRO] Python 3 com Tkinter não foi encontrado.\n' >&2
    printf 'No Ubuntu/Debian, instale com: sudo apt install python3 python3-tk\n\n' >&2
    exit 1
}

if [ ! -f "$PROJECT_DIR/vision/config.yaml" ] || [ ! -d "$PROJECT_DIR/strategy" ]; then
    printf '\n[ERRO] A pasta ihm deve ficar na raiz do projeto, ao lado de vision e strategy.\n' >&2
    printf 'Local esperado: <projeto>/ihm/iniciar_ihm.sh\n\n' >&2
    exit 1
fi

if [ "${1:-}" = "--check" ]; then
    printf '[OK] Python/Tkinter: %s\n' "$PYTHON_BIN"
    printf '[OK] Projeto: %s\n' "$PROJECT_DIR"
    printf '[OK] IHM pronta para iniciar no Linux.\n'
    exit 0
fi

cd "$SCRIPT_DIR"
exec "$PYTHON_BIN" ./app.py
