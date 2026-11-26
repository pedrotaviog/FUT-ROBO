"""Codificacao e decodificacao do protocolo binario do FUT-ROBO."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct


FIELD_POSES = struct.Struct("<hh" + "Bhhf" * 4)
TWIST_CMD = struct.Struct("<ff")
MOTOR_PWM_CMD = struct.Struct("<hh")


@dataclass(frozen=True)
class Vec2:
    x: int
    y: int


@dataclass(frozen=True)
class RobotPose:
    robot_id: int
    position: Vec2
    angle: float


@dataclass(frozen=True)
class FieldPoses:
    ball: Vec2
    robots: tuple[RobotPose, RobotPose, RobotPose, RobotPose]


def decode_field_poses(payload: bytes) -> FieldPoses:
    """Decodifica exatamente os 40 bytes emitidos por common/protocol.h."""
    if len(payload) != FIELD_POSES.size:
        raise ValueError(
            f"pacote FieldPoses com {len(payload)} bytes; esperado {FIELD_POSES.size}"
        )

    values = FIELD_POSES.unpack(payload)
    ball = Vec2(values[0], values[1])
    robots = []
    offset = 2
    for _ in range(4):
        robot_id, x, y, angle = values[offset : offset + 4]
        robots.append(RobotPose(robot_id, Vec2(x, y), angle))
        offset += 4

    return FieldPoses(ball, tuple(robots))  # type: ignore[arg-type]


def encode_twist(v: float, w: float) -> bytes:
    """Monta o comando de velocidade aceito pelo firmware."""
    if not (math.isfinite(v) and math.isfinite(w)):
        raise ValueError("v e w devem ser numeros finitos")
    return TWIST_CMD.pack(v, w)


def encode_motor_pwm(left: int, right: int) -> bytes:
    """Monta o comando de PWM direto por roda (calibracao dos motores)."""
    if not (-255 <= left <= 255 and -255 <= right <= 255):
        raise ValueError("PWM deve estar entre -255 e 255")
    return MOTOR_PWM_CMD.pack(left, right)
