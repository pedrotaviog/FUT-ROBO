"""Leitura e escrita conservadora do config.yaml usado pelo OpenCV."""

from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
import os
import re
import tempfile


@dataclass
class VisionConfig:
    hue_tol: int = 5
    sat_tol: int = 75
    rectMinArea: float = 200.0
    circleMinArea: float = 5.0
    ballMinArea: float = 15.0
    roiWidth: int = 60
    roiHeight: int = 60
    id: int = 0
    width: int = 1280
    height: int = 720
    fps: int = 30
    name: str = "window"
    delay: int = 34


_NUMERIC_FIELDS = {
    "hue_tol": int,
    "sat_tol": int,
    "rectMinArea": float,
    "circleMinArea": float,
    "ballMinArea": float,
    "roiWidth": int,
    "roiHeight": int,
    "id": int,
    "width": int,
    "height": int,
    "fps": int,
    "delay": int,
}


def load_config(path: Path) -> VisionConfig:
    text = path.read_text(encoding="utf-8")
    config = VisionConfig()
    for field in fields(config):
        match = re.search(
            rf"(?m)^\s{{4}}{re.escape(field.name)}:\s*(.*?)\s*$", text
        )
        if not match:
            continue
        raw = match.group(1).strip().strip('"\'')
        converter = _NUMERIC_FIELDS.get(field.name, str)
        setattr(config, field.name, converter(raw))
    return config


def load_field_size(path: Path) -> tuple[float, float]:
    """Tamanho real do campo em cm (secao `field` do config.yaml).

    Cai no campo atual (237 x 165) se o arquivo ou as chaves faltarem, para a
    IHM continuar desenhando o campo.
    """
    size = [237.0, 165.0]
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return size[0], size[1]
    for index, key in enumerate(("sizeX_cm", "sizeY_cm")):
        match = re.search(rf"(?m)^\s{{4}}{key}:\s*([-+0-9.eE]+)\s*$", text)
        if match and float(match.group(1)) > 0:
            size[index] = float(match.group(1))
    return size[0], size[1]


def save_config(path: Path, config: VisionConfig) -> None:
    """Atualiza somente valores conhecidos e troca o arquivo atomicamente."""
    text = path.read_text(encoding="utf-8")
    for field in fields(config):
        value = getattr(config, field.name)
        replacement = f"    {field.name}: {value}"
        text, count = re.subn(
            rf"(?m)^\s{{4}}{re.escape(field.name)}:\s*.*$",
            replacement,
            text,
            count=1,
        )
        if count != 1:
            raise ValueError(f"chave ausente ou duplicada no config.yaml: {field.name}")

    handle, temp_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent, text=True
    )
    try:
        with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as temp:
            temp.write(text)
        os.replace(temp_name, path)
    except Exception:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass
        raise
