"""IHM desktop para operacao local do FUT-ROBO."""

from __future__ import annotations

from dataclasses import fields
from collections import deque
from datetime import datetime
from pathlib import Path
import math
import os
import queue
import shutil
import socket
import struct
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from config_store import VisionConfig, load_config, load_field_size, save_config
from protocol import FIELD_POSES, FieldPoses, decode_field_poses, encode_twist


ROOT = Path(__file__).resolve().parents[1]
VISION_DIR = ROOT / "vision"
CONFIG_PATH = VISION_DIR / "config.yaml"
# Metade do campo em cm, lida da secao `field` do config.yaml da visao.
FIELD_HALF_X_CM, FIELD_HALF_Y_CM = (size / 2 for size in load_field_size(CONFIG_PATH))
MULTICAST_GROUP = "224.1.1.1"
MULTICAST_PORT = 5007

COLORS = {
    "bg": "#060b14",
    "sidebar": "#0a1220",
    "panel": "#0f1928",
    "panel_alt": "#162338",
    "border": "#24344b",
    "text": "#f8fafc",
    "muted": "#94a3b8",
    "accent": "#22c55e",
    "accent_hover": "#16a34a",
    "ball": "#f97316",
    "ok": "#22c55e",
    "warning": "#f59e0b",
    "danger": "#dc2626",
    "danger_hover": "#b91c1c",
    "field": "#0c241d",
}


class ManagedProcess:
    def __init__(self, name: str, events: queue.Queue):
        self.name = name
        self.events = events
        self.process: subprocess.Popen[str] | None = None

    @property
    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self, command: list[str], cwd: Path) -> None:
        if self.running:
            raise RuntimeError(f"{self.name} ja esta em execucao")
        flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
        self.events.put(("log", self.name, f"$ {' '.join(command)}"))
        self.process = subprocess.Popen(
            command,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            creationflags=flags,
        )
        threading.Thread(target=self._read_output, daemon=True).start()

    def _read_output(self) -> None:
        assert self.process is not None
        assert self.process.stdout is not None
        for line in self.process.stdout:
            self.events.put(("log", self.name, line.rstrip()))
        code = self.process.wait()
        self.events.put(("process_exit", self.name, code))

    def stop(self) -> None:
        if not self.running:
            return
        assert self.process is not None
        self.process.terminate()
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.kill()


class TelemetryListener:
    def __init__(self, events: queue.Queue):
        self.events = events
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        sock: socket.socket | None = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("", MULTICAST_PORT))
            membership = struct.pack(
                "=4s4s",
                socket.inet_aton(MULTICAST_GROUP),
                socket.inet_aton("0.0.0.0"),
            )
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
            sock.settimeout(0.5)
            self.events.put(
                ("log", "telemetria", f"Escutando {MULTICAST_GROUP}:{MULTICAST_PORT}")
            )
            while not self._stop.is_set():
                try:
                    payload, sender = sock.recvfrom(2048)
                except socket.timeout:
                    continue
                if len(payload) != FIELD_POSES.size:
                    self.events.put(
                        (
                            "log",
                            "telemetria",
                            f"Pacote ignorado: {len(payload)} bytes de {sender[0]}",
                        )
                    )
                    continue
                self.events.put(
                    (
                        "telemetry",
                        decode_field_poses(payload),
                        sender[0],
                        time.monotonic(),
                    )
                )
        except OSError as exc:
            self.events.put(
                ("log", "telemetria", f"Falha ao abrir multicast: {exc}")
            )
        finally:
            if sock:
                sock.close()


class FutRoboIHM(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("FUT-ROBO | Console de Operacao")
        self.geometry("1440x900")
        self.minsize(1180, 720)
        self.configure(bg=COLORS["bg"])

        self.events: queue.Queue = queue.Queue()
        self.processes = {
            "visao": ManagedProcess("visao", self.events),
            "estrategia": ManagedProcess("estrategia", self.events),
            "build-visao": ManagedProcess("build-visao", self.events),
            "build-estrategia": ManagedProcess("build-estrategia", self.events),
        }
        self.telemetry = TelemetryListener(self.events)
        self.latest_field: FieldPoses | None = None
        self.last_telemetry = 0.0
        self.started_at = time.monotonic()
        self.packet_count = 0
        self.log_count = 0
        self.packet_times: deque[float] = deque(maxlen=120)
        self.last_sender = "—"
        self.build_after_config: dict[str, tuple[str, str]] = {}
        self.settings_vars: dict[str, tk.StringVar] = {}
        self.robot_host = tk.StringVar(value="robot.local")
        self.robot_port = tk.StringVar(value="3333")

        self._configure_styles()
        self._build_ui()
        self._load_settings()
        self.telemetry.start()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(100, self._drain_events)
        self.after(500, self._refresh_status)

    def _configure_styles(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background=COLORS["bg"])
        style.configure("Panel.TFrame", background=COLORS["panel"])
        style.configure("Sidebar.TFrame", background=COLORS["sidebar"])
        style.configure(
            "TLabel",
            background=COLORS["bg"],
            foreground=COLORS["text"],
            font=("Segoe UI", 10),
        )
        style.configure(
            "Panel.TLabel", background=COLORS["panel"], foreground=COLORS["text"]
        )
        style.configure(
            "Muted.Panel.TLabel",
            background=COLORS["panel"],
            foreground=COLORS["muted"],
        )
        style.configure(
            "Title.TLabel",
            background=COLORS["bg"],
            foreground=COLORS["text"],
            font=("Segoe UI Semibold", 20),
        )
        style.configure(
            "Section.Panel.TLabel",
            background=COLORS["panel"],
            foreground=COLORS["text"],
            font=("Segoe UI Semibold", 12),
        )
        style.configure(
            "TButton",
            background=COLORS["panel_alt"],
            foreground=COLORS["text"],
            borderwidth=1,
            padding=(14, 9),
            font=("Segoe UI Semibold", 9),
        )
        style.map(
            "TButton",
            background=[("active", COLORS["border"]), ("disabled", COLORS["panel"])],
            foreground=[("disabled", COLORS["muted"])],
        )
        style.configure(
            "Accent.TButton",
            background=COLORS["accent"],
            foreground="#ffffff",
            padding=(16, 10),
        )
        style.map(
            "Accent.TButton", background=[("active", COLORS["accent_hover"])]
        )
        style.configure(
            "Compact.TButton",
            background=COLORS["panel_alt"],
            foreground=COLORS["text"],
            borderwidth=1,
            padding=(11, 6),
            font=("Segoe UI Semibold", 9),
        )
        style.map("Compact.TButton", background=[("active", COLORS["border"])])
        style.configure(
            "CompactAccent.TButton",
            background=COLORS["accent"],
            foreground="#ffffff",
            borderwidth=1,
            padding=(11, 6),
            font=("Segoe UI Semibold", 9),
        )
        style.map(
            "CompactAccent.TButton",
            background=[("active", COLORS["accent_hover"])],
        )
        style.configure(
            "Side.TButton",
            background=COLORS["sidebar"],
            foreground=COLORS["muted"],
            borderwidth=0,
            padding=(18, 14),
            anchor="w",
            font=("Segoe UI Semibold", 10),
        )
        style.map(
            "Side.TButton",
            background=[("active", COLORS["panel_alt"])],
            foreground=[("active", COLORS["text"])],
        )
        style.configure(
            "SideActive.TButton",
            background=COLORS["panel_alt"],
            foreground=COLORS["text"],
            borderwidth=0,
            padding=(18, 14),
            anchor="w",
            font=("Segoe UI Semibold", 10),
        )
        style.configure(
            "Danger.TButton",
            background=COLORS["danger"],
            foreground="#ffffff",
            padding=(20, 15),
            font=("Segoe UI Bold", 11),
        )
        style.map(
            "Danger.TButton", background=[("active", COLORS["danger_hover"])]
        )
        style.configure(
            "TEntry",
            fieldbackground=COLORS["panel_alt"],
            foreground=COLORS["text"],
            insertcolor=COLORS["text"],
            bordercolor=COLORS["border"],
            padding=7,
        )
        style.configure("TNotebook", background=COLORS["bg"], borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            background=COLORS["panel"],
            foreground=COLORS["muted"],
            padding=(18, 10),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", COLORS["panel_alt"])],
            foreground=[("selected", COLORS["text"])],
        )

    def _build_ui(self) -> None:
        shell = ttk.Frame(self)
        shell.pack(fill="both", expand=True)

        sidebar = ttk.Frame(shell, style="Sidebar.TFrame", width=220, padding=(14, 20))
        sidebar.pack(side="left", fill="y")
        sidebar.pack_propagate(False)
        tk.Label(
            sidebar,
            text="FUT‑ROBO",
            bg=COLORS["sidebar"],
            fg=COLORS["text"],
            font=("Segoe UI Semibold", 20),
            anchor="w",
        ).pack(fill="x", padx=8)
        tk.Label(
            sidebar,
            text="COMMAND CENTER  /  V2",
            bg=COLORS["sidebar"],
            fg=COLORS["accent"],
            font=("Consolas", 8),
            anchor="w",
        ).pack(fill="x", padx=8, pady=(2, 30))

        self.nav_buttons: dict[str, ttk.Button] = {}
        for page, label in (
            ("operation", "01   Operação"),
            ("settings", "02   Calibração"),
            ("logs", "03   Eventos"),
        ):
            button = ttk.Button(
                sidebar,
                text=label,
                style="Side.TButton",
                command=lambda key=page: self._show_page(key),
            )
            button.pack(fill="x", pady=3)
            self.nav_buttons[page] = button

        sidebar_footer = tk.Frame(sidebar, bg=COLORS["sidebar"])
        sidebar_footer.pack(side="bottom", fill="x", padx=8)
        tk.Label(
            sidebar_footer,
            text="MULTICAST",
            bg=COLORS["sidebar"],
            fg=COLORS["muted"],
            font=("Segoe UI Semibold", 8),
            anchor="w",
        ).pack(fill="x")
        tk.Label(
            sidebar_footer,
            text=f"{MULTICAST_GROUP}:{MULTICAST_PORT}",
            bg=COLORS["sidebar"],
            fg=COLORS["text"],
            font=("Consolas", 9),
            anchor="w",
        ).pack(fill="x", pady=(3, 0))

        workspace = ttk.Frame(shell)
        workspace.pack(side="left", fill="both", expand=True)
        header = ttk.Frame(workspace, padding=(24, 17, 24, 12))
        header.pack(fill="x")
        title_box = ttk.Frame(header)
        title_box.pack(side="left")
        self.page_title = ttk.Label(title_box, text="Visão geral", style="Title.TLabel")
        self.page_title.pack(anchor="w")
        self.page_subtitle = ttk.Label(
            title_box,
            text="Acompanhamento dos módulos e da telemetria em tempo real",
            foreground=COLORS["muted"],
        )
        self.page_subtitle.pack(anchor="w", pady=(2, 0))
        self.system_status = tk.Label(
            header,
            text="● AGUARDANDO TELEMETRIA",
            bg=COLORS["bg"],
            fg=COLORS["warning"],
            font=("Segoe UI Semibold", 10),
        )
        self.system_status.pack(side="right", pady=(10, 0))

        content = ttk.Frame(workspace)
        content.pack(fill="both", expand=True, padx=18, pady=(0, 18))
        self.pages = {
            "operation": ttk.Frame(content),
            "settings": ttk.Frame(content),
            "logs": ttk.Frame(content),
        }
        for page in self.pages.values():
            page.place(relx=0, rely=0, relwidth=1, relheight=1)
        self._build_dashboard(self.pages["operation"])
        self._build_settings(self.pages["settings"])
        self._build_logs(self.pages["logs"])
        self._show_page("operation")
        self.bind("<Control-Key-1>", lambda _event: self._show_page("operation"))
        self.bind("<Control-Key-2>", lambda _event: self._show_page("settings"))
        self.bind("<Control-Key-3>", lambda _event: self._show_page("logs"))

    def _show_page(self, page: str) -> None:
        titles = {
            "operation": (
                "Visão geral",
                "Acompanhamento dos módulos e da telemetria em tempo real",
            ),
            "settings": (
                "Calibração e parâmetros",
                "Prepare a câmera, o campo e as faixas de cor para a partida",
            ),
            "logs": (
                "Central de eventos",
                "Saída consolidada da visão, estratégia, build e rede",
            ),
        }
        self.pages[page].lift()
        title, subtitle = titles[page]
        self.page_title.configure(text=title)
        self.page_subtitle.configure(text=subtitle)
        for key, button in self.nav_buttons.items():
            button.configure(style="SideActive.TButton" if key == page else "Side.TButton")

    def _panel(self, parent: tk.Widget, **grid: int) -> ttk.Frame:
        frame = ttk.Frame(parent, style="Panel.TFrame", padding=13)
        frame.grid(sticky="nsew", padx=6, pady=6, **grid)
        return frame

    def _build_dashboard(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=3)
        parent.columnconfigure(1, weight=2)
        parent.rowconfigure(1, weight=1)

        metrics = ttk.Frame(parent)
        metrics.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        for column in range(4):
            metrics.columnconfigure(column, weight=1)
        self.packet_rate_var = tk.StringVar(value="0.0 Hz")
        self.packet_total_var = tk.StringVar(value="0")
        self.last_packet_var = tk.StringVar(value="Sem dados")
        self.session_var = tk.StringVar(value="00:00:00")
        self._metric_card(metrics, 0, "FREQUÊNCIA", self.packet_rate_var, "pacotes por segundo")
        self._metric_card(metrics, 1, "PACOTES", self.packet_total_var, "recebidos nesta sessão")
        self._metric_card(metrics, 2, "ÚLTIMO FRAME", self.last_packet_var, "idade da telemetria")
        self._metric_card(metrics, 3, "SESSÃO", self.session_var, "tempo de operação")

        field_panel = self._panel(parent, row=1, column=0)
        field_panel.rowconfigure(1, weight=1)
        field_panel.columnconfigure(0, weight=1)
        field_head = ttk.Frame(field_panel, style="Panel.TFrame")
        field_head.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        ttk.Label(field_head, text="CAMPO EM TEMPO REAL", style="Section.Panel.TLabel").pack(side="left")
        ttk.Label(
            field_head,
            text="BOLA  ●     TIME A  ●     TIME B  ●",
            style="Muted.Panel.TLabel",
            font=("Consolas", 8),
        ).pack(side="right")
        self.field_canvas = tk.Canvas(
            field_panel,
            bg=COLORS["field"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
        )
        self.field_canvas.grid(row=1, column=0, sticky="nsew")
        self.field_canvas.bind("<Configure>", lambda _event: self._draw_field())

        right = ttk.Frame(parent)
        right.grid(row=1, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)

        modules = self._panel(right, row=0, column=0)
        ttk.Label(modules, text="MODULOS", style="Section.Panel.TLabel").grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(0, 7)
        )
        self.vision_state = self._status_row(modules, 1, "Visao")
        self.vision_start_button = ttk.Button(
            modules,
            text="Iniciar",
            style="CompactAccent.TButton",
            command=lambda: self._start_module("visao"),
        )
        self.vision_start_button.grid(row=1, column=2, padx=4)
        self.vision_stop_button = ttk.Button(
            modules, text="Parar", style="Compact.TButton", command=lambda: self._stop_module("visao")
        )
        self.vision_stop_button.grid(row=1, column=3, padx=4)
        self.strategy_state = self._status_row(modules, 2, "Estrategia")
        self.strategy_start_button = ttk.Button(
            modules,
            text="Iniciar",
            style="CompactAccent.TButton",
            command=lambda: self._start_module("estrategia"),
        )
        self.strategy_start_button.grid(row=2, column=2, padx=4, pady=6)
        self.strategy_stop_button = ttk.Button(
            modules, text="Parar", style="Compact.TButton", command=lambda: self._stop_module("estrategia")
        )
        self.strategy_stop_button.grid(row=2, column=3, padx=4, pady=6)
        self.build_vision_button = ttk.Button(
            modules,
            text="Compilar visao",
            style="Compact.TButton",
            command=lambda: self._build_module("visao"),
        )
        self.build_vision_button.grid(row=3, column=0, columnspan=2, sticky="ew", padx=(0, 4), pady=(10, 0))
        self.build_strategy_button = ttk.Button(
            modules,
            text="Compilar estrategia",
            style="Compact.TButton",
            command=lambda: self._build_module("estrategia"),
        )
        self.build_strategy_button.grid(row=3, column=2, columnspan=2, sticky="ew", padx=(4, 0), pady=(10, 0))

        robots = self._panel(right, row=1, column=0)
        ttk.Label(robots, text="TELEMETRIA", style="Section.Panel.TLabel").pack(
            anchor="w", pady=(0, 5)
        )
        self.ball_label = ttk.Label(
            robots, text="Bola: aguardando", style="Muted.Panel.TLabel"
        )
        self.ball_label.pack(anchor="w", pady=1)
        self.robot_labels = []
        for index in range(4):
            label = ttk.Label(
                robots,
                text=f"Robo {index}: aguardando",
                style="Muted.Panel.TLabel",
            )
            label.pack(anchor="w", pady=1)
            self.robot_labels.append(label)

        emergency = self._panel(right, row=2, column=0)
        ttk.Label(emergency, text="SEGURANCA", style="Section.Panel.TLabel").pack(
            anchor="w"
        )
        ttk.Label(
            emergency,
            text="Envia v=0 e w=0 diretamente ao firmware configurado.",
            style="Muted.Panel.TLabel",
            wraplength=390,
        ).pack(anchor="w", pady=(3, 7))
        ttk.Button(
            emergency,
            text="PARADA DE EMERGENCIA",
            style="Danger.TButton",
            command=self._emergency_stop,
        ).pack(fill="x", pady=(0, 1))

    def _metric_card(
        self,
        parent: ttk.Frame,
        column: int,
        title: str,
        variable: tk.StringVar,
        caption: str,
    ) -> None:
        card = ttk.Frame(parent, style="Panel.TFrame", padding=(16, 12))
        card.grid(row=0, column=column, sticky="ew", padx=6, pady=4)
        tk.Frame(card, bg=COLORS["accent"], height=2).pack(fill="x", pady=(0, 9))
        ttk.Label(card, text=title, style="Muted.Panel.TLabel", font=("Segoe UI Semibold", 8)).pack(anchor="w")
        ttk.Label(card, textvariable=variable, style="Panel.TLabel", font=("Consolas", 18, "bold")).pack(anchor="w", pady=(2, 0))
        ttk.Label(card, text=caption, style="Muted.Panel.TLabel", font=("Segoe UI", 8)).pack(anchor="w", pady=(1, 0))

    def _status_row(self, parent: ttk.Frame, row: int, name: str) -> tk.Label:
        ttk.Label(parent, text=name, style="Panel.TLabel").grid(
            row=row, column=0, sticky="w", padx=(0, 12)
        )
        label = tk.Label(
            parent,
            text="● PARADO",
            bg=COLORS["panel"],
            fg=COLORS["muted"],
            font=("Segoe UI Semibold", 9),
            width=14,
            anchor="w",
        )
        label.grid(row=row, column=1, sticky="w")
        return label

    def _build_settings(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=3)
        parent.columnconfigure(1, weight=2)
        parent.rowconfigure(0, weight=1)
        form = self._panel(parent, row=0, column=0)
        ttk.Label(
            form, text="PARAMETROS DA VISAO", style="Section.Panel.TLabel"
        ).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 12))
        labels = {
            "hue_tol": "Tolerancia de matiz",
            "sat_tol": "Tolerancia de saturacao",
            "rectMinArea": "Area minima do robo",
            "circleMinArea": "Area minima da marca",
            "ballMinArea": "Area minima da bola",
            "roiWidth": "Largura da ROI",
            "roiHeight": "Altura da ROI",
            "id": "ID da camera",
            "width": "Largura da camera",
            "height": "Altura da camera",
            "fps": "FPS",
            "name": "Nome da janela",
            "delay": "Atraso da janela (ms)",
        }
        for index, field in enumerate(fields(VisionConfig), start=1):
            column_group = 0 if index <= 7 else 2
            row = index if index <= 7 else index - 7
            ttk.Label(
                form, text=labels[field.name], style="Muted.Panel.TLabel"
            ).grid(row=row, column=column_group, sticky="w", padx=(0, 8), pady=5)
            variable = tk.StringVar()
            self.settings_vars[field.name] = variable
            ttk.Entry(form, textvariable=variable, width=18).grid(
                row=row,
                column=column_group + 1,
                sticky="ew",
                padx=(0, 18),
                pady=5,
            )
        ttk.Button(form, text="Recarregar", command=self._load_settings).grid(
            row=9, column=0, sticky="ew", pady=(18, 0), padx=(0, 4)
        )
        ttk.Button(
            form,
            text="Salvar configuracao",
            style="Accent.TButton",
            command=self._save_settings,
        ).grid(
            row=9,
            column=1,
            columnspan=3,
            sticky="ew",
            pady=(18, 0),
            padx=(4, 0),
        )

        side = ttk.Frame(parent)
        side.grid(row=0, column=1, sticky="nsew")
        side.columnconfigure(0, weight=1)
        calibration = self._panel(side, row=0, column=0)
        ttk.Label(
            calibration, text="CALIBRACAO", style="Section.Panel.TLabel"
        ).pack(anchor="w")
        ttk.Label(
            calibration,
            text="A IHM preserva o YAML atual como backup e abre a janela da visao para receber os cliques.",
            style="Muted.Panel.TLabel",
            wraplength=410,
        ).pack(anchor="w", pady=(5, 12))
        workflow = tk.Frame(calibration, bg=COLORS["panel_alt"], padx=12, pady=10)
        workflow.pack(fill="x", pady=(0, 12))
        for number, text in (
            ("01", "Posicione e fixe a câmera"),
            ("02", "Marque os quatro cantos"),
            ("03", "Amostre as cinco cores"),
            ("04", "Confira máscaras e telemetria"),
        ):
            row = tk.Frame(workflow, bg=COLORS["panel_alt"])
            row.pack(fill="x", pady=2)
            tk.Label(
                row,
                text=number,
                bg=COLORS["panel_alt"],
                fg=COLORS["accent"],
                font=("Consolas", 9, "bold"),
                width=4,
                anchor="w",
            ).pack(side="left")
            tk.Label(
                row,
                text=text,
                bg=COLORS["panel_alt"],
                fg=COLORS["text"],
                font=("Segoe UI", 9),
                anchor="w",
            ).pack(side="left")
        self.calibration_status_label = ttk.Label(
            calibration,
            text="Verificando arquivos de calibração…",
            style="Muted.Panel.TLabel",
        )
        self.calibration_status_label.pack(anchor="w", pady=(0, 8))
        ttk.Button(
            calibration,
            text="Calibrar campo (4 pontos)",
            style="Accent.TButton",
            command=self._calibrate_field,
        ).pack(fill="x", pady=4)
        color_grid = ttk.Frame(calibration, style="Panel.TFrame")
        color_grid.pack(fill="x", pady=(8, 0))
        for index, color in enumerate(
            ("laranja", "verde", "azul", "rosa", "amarelo")
        ):
            ttk.Button(
                color_grid,
                text=color.capitalize(),
                command=lambda c=color: self._calibrate_color(c),
            ).grid(row=index // 2, column=index % 2, sticky="ew", padx=3, pady=3)
        color_grid.columnconfigure(0, weight=1)
        color_grid.columnconfigure(1, weight=1)
        self._refresh_calibration_status()

        network = self._panel(side, row=1, column=0)
        ttk.Label(
            network, text="REDE DO ROBO", style="Section.Panel.TLabel"
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))
        ttk.Label(network, text="Host", style="Muted.Panel.TLabel").grid(
            row=1, column=0, sticky="w", pady=5
        )
        ttk.Entry(network, textvariable=self.robot_host).grid(
            row=1, column=1, sticky="ew", pady=5
        )
        ttk.Label(network, text="Porta UDP", style="Muted.Panel.TLabel").grid(
            row=2, column=0, sticky="w", pady=5
        )
        ttk.Entry(network, textvariable=self.robot_port).grid(
            row=2, column=1, sticky="ew", pady=5
        )
        network.columnconfigure(1, weight=1)

    def _build_logs(self, parent: ttk.Frame) -> None:
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill="x", pady=(4, 8))
        ttk.Label(
            toolbar, text="SAIDA DOS PROCESSOS", style="Section.Panel.TLabel"
        ).pack(side="left")
        self.log_count_var = tk.StringVar(value="0 eventos")
        ttk.Label(
            toolbar,
            textvariable=self.log_count_var,
            foreground=COLORS["muted"],
        ).pack(side="left", padx=12)
        self.log_text = tk.Text(
            parent,
            bg="#080d13",
            fg="#cfdae7",
            insertbackground="#ffffff",
            relief="flat",
            font=("Consolas", 10),
            padx=14,
            pady=12,
            wrap="word",
        )
        ttk.Button(
            toolbar,
            text="Limpar",
            command=lambda: self._clear_logs(),
        ).pack(side="right")
        ttk.Button(
            toolbar,
            text="Copiar tudo",
            command=self._copy_logs,
        ).pack(side="right", padx=6)
        self.log_text.pack(fill="both", expand=True)
        self.log_text.configure(state="disabled")

    def _clear_logs(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self.log_count = 0
        self.log_count_var.set("0 eventos")

    def _copy_logs(self) -> None:
        content = self.log_text.get("1.0", "end-1c")
        self.clipboard_clear()
        self.clipboard_append(content)
        self._log("ihm", "Conteúdo dos eventos copiado")

    def _log(self, source: str, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] [{source}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")
        self.log_count += 1
        if hasattr(self, "log_count_var"):
            self.log_count_var.set(f"{self.log_count} eventos")

    def _candidate_executable(self, module: str) -> Path | None:
        module_dir = ROOT / module
        executable = f"{module}.exe" if os.name == "nt" else module
        candidates = [
            module_dir / "build" / "Debug" / executable,
            module_dir / "build" / "Release" / executable,
            module_dir / "build" / executable,
        ]
        return next((path for path in candidates if path.exists()), None)

    def _start_module(self, display_name: str) -> None:
        module = "vision" if display_name == "visao" else "strategy"
        executable = self._candidate_executable(module)
        if not executable:
            messagebox.showwarning(
                "Executavel ausente",
                f"O modulo {module} ainda nao foi compilado. Use o botao Compilar {display_name}.",
            )
            return
        try:
            self.processes[display_name].start([str(executable)], ROOT / module)
            self._log(display_name, "Modulo iniciado")
        except Exception as exc:
            messagebox.showerror("Falha ao iniciar", str(exc))

    def _stop_module(self, name: str) -> None:
        self.processes[name].stop()
        self._log(name, "Solicitacao de parada enviada")

    def _build_module(self, display_name: str) -> None:
        if not shutil.which("cmake"):
            messagebox.showerror(
                "CMake nao encontrado",
                "Instale o CMake e deixe o comando cmake disponivel no PATH.",
            )
            return
        module = "vision" if display_name == "visao" else "strategy"
        build_dir = ROOT / module / "build"
        runner = self.processes[f"build-{display_name}"]
        try:
            if not (build_dir / "CMakeCache.txt").exists():
                self.build_after_config[runner.name] = (module, display_name)
                runner.start(
                    [
                        "cmake",
                        "-S",
                        ".",
                        "-B",
                        "build",
                        "-DCMAKE_BUILD_TYPE=Debug",
                    ],
                    ROOT / module,
                )
            else:
                runner.start(
                    ["cmake", "--build", "build", "--config", "Debug"],
                    ROOT / module,
                )
        except Exception as exc:
            messagebox.showerror("Falha na compilacao", str(exc))

    def _load_settings(self) -> None:
        try:
            config = load_config(CONFIG_PATH)
            for field in fields(config):
                self.settings_vars[field.name].set(str(getattr(config, field.name)))
            if hasattr(self, "log_text"):
                self._log("config", "config.yaml carregado")
        except Exception as exc:
            messagebox.showerror("Falha ao carregar configuracao", str(exc))

    def _save_settings(self) -> None:
        try:
            values = {}
            float_fields = {"rectMinArea", "circleMinArea", "ballMinArea"}
            for field in fields(VisionConfig):
                raw = self.settings_vars[field.name].get().strip()
                if field.name in float_fields:
                    values[field.name] = float(raw)
                elif field.name == "name":
                    values[field.name] = raw
                else:
                    values[field.name] = int(raw)
            config = VisionConfig(**values)
            if config.width <= 0 or config.height <= 0 or config.fps <= 0:
                raise ValueError("resolucao e FPS devem ser positivos")
            if config.roiWidth <= 0 or config.roiHeight <= 0:
                raise ValueError("as dimensoes da ROI devem ser positivas")
            save_config(CONFIG_PATH, config)
            self._log("config", "Configuracao salva com sucesso")
            messagebox.showinfo(
                "Configuracao", "Parametros salvos em vision/config.yaml."
            )
        except Exception as exc:
            messagebox.showerror("Configuracao invalida", str(exc))

    def _backup_calibration(self, filename: str) -> Path | None:
        path = VISION_DIR / "data" / filename
        if not path.exists():
            return None
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        backup = path.with_name(f"{path.stem}.backup-{stamp}{path.suffix}")
        path.rename(backup)
        self._log("calibracao", f"Backup criado: {backup.name}")
        self._refresh_calibration_status()
        return backup

    def _refresh_calibration_status(self) -> None:
        expected = (
            "calib.yaml",
            "laranja.yaml",
            "verde.yaml",
            "azul.yaml",
            "rosa.yaml",
            "amarelo.yaml",
        )
        ready = sum((VISION_DIR / "data" / name).exists() for name in expected)
        complete = ready == len(expected)
        if hasattr(self, "calibration_status_label"):
            self.calibration_status_label.configure(
                text=(
                    "●  Calibração completa  /  6 de 6 arquivos"
                    if complete
                    else f"●  Calibração pendente  /  {ready} de 6 arquivos"
                ),
                foreground=COLORS["ok"] if complete else COLORS["warning"],
            )

    def _ensure_vision_for_calibration(self) -> bool:
        if self.processes["visao"].running:
            messagebox.showwarning(
                "Visao em execucao",
                "Pare a visao antes de iniciar uma nova calibracao.",
            )
            return False
        if not self._candidate_executable("vision"):
            messagebox.showwarning(
                "Visao nao compilada", "Compile a visao antes de calibrar."
            )
            return False
        return True

    def _calibrate_field(self) -> None:
        if not self._ensure_vision_for_calibration():
            return
        confirmed = messagebox.askokcancel(
            "Calibrar campo",
            "A janela da visao sera aberta. Clique: superior esquerdo, superior direito, inferior direito e inferior esquerdo. O arquivo atual sera preservado como backup.",
        )
        if not confirmed:
            return
        self._backup_calibration("calib.yaml")
        self._start_module("visao")

    def _calibrate_color(self, color: str) -> None:
        if not self._ensure_vision_for_calibration():
            return
        confirmed = messagebox.askokcancel(
            "Calibrar cor",
            f"A janela da visao sera aberta. Clique em uma regiao uniforme da cor {color}. O arquivo atual sera preservado como backup.",
        )
        if not confirmed:
            return
        self._backup_calibration(f"{color}.yaml")
        self._start_module("visao")

    def _send_stop(self, show_error: bool) -> bool:
        host = self.robot_host.get().strip()
        try:
            port = int(self.robot_port.get())
            if not 1 <= port <= 65535:
                raise ValueError("porta fora do intervalo")
            payload = encode_twist(0.0, 0.0)
            addresses = socket.getaddrinfo(
                host, port, socket.AF_INET, socket.SOCK_DGRAM
            )
            targets = {item[4] for item in addresses}
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                for target in targets:
                    for _ in range(3):
                        sock.sendto(payload, target)
            self._log("seguranca", f"PARADA enviada 3x para {host}:{port}")
            return True
        except Exception as exc:
            self._log("seguranca", f"Falha na parada: {exc}")
            if show_error:
                messagebox.showerror("Parada nao enviada", str(exc))
            return False

    def _emergency_stop(self) -> None:
        self._send_stop(show_error=True)

    def _drain_events(self) -> None:
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == "log":
                    _, source, message = event
                    self._log(source, message)
                elif event[0] == "process_exit":
                    _, source, code = event
                    self._log(source, f"Processo encerrado com codigo {code}")
                    pending = self.build_after_config.pop(source, None)
                    if pending and code == 0:
                        module, display_name = pending
                        self._log(source, "Configuracao concluida; iniciando compilacao")
                        self.processes[source].start(
                            ["cmake", "--build", "build", "--config", "Debug"],
                            ROOT / module,
                        )
                    elif pending:
                        self._log(
                            source,
                            f"Configuracao falhou; compilacao de {display_name} cancelada",
                        )
                elif event[0] == "telemetry":
                    _, field, sender, timestamp = event
                    self.latest_field = field
                    self.last_telemetry = timestamp
                    self.packet_count += 1
                    self.packet_times.append(timestamp)
                    self.last_sender = sender
                    self._update_telemetry_labels(sender)
                    self._draw_field()
        except queue.Empty:
            pass
        self.after(100, self._drain_events)

    def _update_telemetry_labels(self, sender: str) -> None:
        if not self.latest_field:
            return
        self.ball_label.configure(
            text=(
                f"Bola: x={self.latest_field.ball.x:>4}  "
                f"y={self.latest_field.ball.y:>4}  | origem {sender}"
            ),
            foreground=COLORS["text"],
        )
        for index, robot in enumerate(self.latest_field.robots):
            degrees = math.degrees(robot.angle)
            self.robot_labels[index].configure(
                text=(
                    f"Robo {robot.robot_id}: x={robot.position.x:>4}  "
                    f"y={robot.position.y:>4}  angulo={degrees:>6.1f}°"
                ),
                foreground=COLORS["text"],
            )

    def _draw_field(self) -> None:
        canvas = self.field_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 100)
        height = max(canvas.winfo_height(), 100)
        margin = 38
        # Grade de coordenadas: ajuda a perceber deslocamento e escala sem
        # transformar o campo em um grafico visualmente pesado.
        for step in range(1, 5):
            gx = margin + step * (width - 2 * margin) / 5
            canvas.create_line(gx, margin, gx, height - margin, fill="#173b32")
        for step in range(1, 6):
            gy = margin + step * (height - 2 * margin) / 6
            canvas.create_line(margin, gy, width - margin, gy, fill="#173b32")
        canvas.create_rectangle(
            margin,
            margin,
            width - margin,
            height - margin,
            outline="#dbeafe",
            width=2,
        )
        canvas.create_line(
            width / 2,
            margin,
            width / 2,
            height - margin,
            fill="#78909c",
            width=1,
        )
        radius = min(width, height) * 0.12
        canvas.create_oval(
            width / 2 - radius,
            height / 2 - radius,
            width / 2 + radius,
            height / 2 + radius,
            outline="#78909c",
        )
        goal_depth = max(10, (width - 2 * margin) * 0.035)
        goal_height = (height - 2 * margin) * 0.28
        canvas.create_rectangle(
            margin - goal_depth,
            height / 2 - goal_height / 2,
            margin,
            height / 2 + goal_height / 2,
            outline="#78909c",
        )
        canvas.create_rectangle(
            width - margin,
            height / 2 - goal_height / 2,
            width - margin + goal_depth,
            height / 2 + goal_height / 2,
            outline="#78909c",
        )
        canvas.create_text(margin, height - 18, text=f"−{FIELD_HALF_X_CM:g}", fill=COLORS["muted"], font=("Consolas", 8))
        canvas.create_text(width / 2, height - 18, text="0", fill=COLORS["muted"], font=("Consolas", 8))
        canvas.create_text(width - margin, height - 18, text=f"+{FIELD_HALF_X_CM:g}", fill=COLORS["muted"], font=("Consolas", 8))
        if not self.latest_field:
            canvas.create_text(
                width / 2,
                height / 2,
                text="Aguardando pacotes da visao em 224.1.1.1:5007",
                fill=COLORS["muted"],
                font=("Segoe UI", 11),
            )
            return

        def point(x: float, y: float) -> tuple[float, float]:
            px = margin + ((x + FIELD_HALF_X_CM) / (2 * FIELD_HALF_X_CM)) * (width - 2 * margin)
            py = margin + ((FIELD_HALF_Y_CM - y) / (2 * FIELD_HALF_Y_CM)) * (height - 2 * margin)
            return px, py

        bx, by = point(self.latest_field.ball.x, self.latest_field.ball.y)
        canvas.create_oval(
            bx - 7,
            by - 7,
            bx + 7,
            by + 7,
            fill=COLORS["ball"],
            outline="#ffffff",
            width=2,
        )
        robot_colors = ("#22c55e", "#3b82f6", "#22c55e", "#3b82f6")
        for index, robot in enumerate(self.latest_field.robots):
            x, y = point(robot.position.x, robot.position.y)
            canvas.create_oval(
                x - 12,
                y - 12,
                x + 12,
                y + 12,
                fill=robot_colors[index],
                outline="#ffffff",
                width=2,
            )
            direction_x = x + 22 * math.cos(robot.angle)
            direction_y = y - 22 * math.sin(robot.angle)
            canvas.create_line(
                x,
                y,
                direction_x,
                direction_y,
                fill="#ffffff",
                width=3,
                arrow="last",
            )
            canvas.create_text(
                x,
                y - 22,
                text=f"R{robot.robot_id}",
                fill="#ffffff",
                font=("Segoe UI Semibold", 9),
            )
        if time.monotonic() - self.last_telemetry >= 1.5:
            canvas.create_rectangle(
                margin + 10,
                margin + 10,
                margin + 155,
                margin + 38,
                fill="#3a1f16",
                outline=COLORS["warning"],
            )
            canvas.create_text(
                margin + 82,
                margin + 24,
                text="DADOS CONGELADOS",
                fill="#fbbf24",
                font=("Segoe UI Semibold", 9),
            )

    def _refresh_status(self) -> None:
        mapping = (
            ("visao", self.vision_state),
            ("estrategia", self.strategy_state),
        )
        for name, label in mapping:
            running = self.processes[name].running
            label.configure(
                text="● EXECUTANDO" if running else "● PARADO",
                fg=COLORS["ok"] if running else COLORS["muted"],
            )
        self.vision_start_button.configure(
            state="disabled" if self.processes["visao"].running else "normal"
        )
        self.vision_stop_button.configure(
            state="normal" if self.processes["visao"].running else "disabled"
        )
        self.strategy_start_button.configure(
            state="disabled" if self.processes["estrategia"].running else "normal"
        )
        self.strategy_stop_button.configure(
            state="normal" if self.processes["estrategia"].running else "disabled"
        )
        self.build_vision_button.configure(
            state="disabled" if self.processes["build-visao"].running else "normal"
        )
        self.build_strategy_button.configure(
            state="disabled"
            if self.processes["build-estrategia"].running
            else "normal"
        )
        online = bool(
            self.last_telemetry and time.monotonic() - self.last_telemetry < 1.5
        )
        now = time.monotonic()
        while self.packet_times and now - self.packet_times[0] > 2.0:
            self.packet_times.popleft()
        rate = 0.0
        if len(self.packet_times) >= 2:
            span = self.packet_times[-1] - self.packet_times[0]
            if span > 0:
                rate = (len(self.packet_times) - 1) / span
        age = now - self.last_telemetry if self.last_telemetry else None
        elapsed = int(now - self.started_at)
        hours, remainder = divmod(elapsed, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.packet_rate_var.set(f"{rate:0.1f} Hz")
        self.packet_total_var.set(f"{self.packet_count:,}".replace(",", "."))
        if age is None:
            age_text = "Sem dados"
        elif age < 10.0:
            age_text = f"{age * 1000:0.0f} ms"
        elif age < 60.0:
            age_text = f"{age:0.1f} s"
        else:
            age_text = "Sem sinal"
        self.last_packet_var.set(age_text)
        self.session_var.set(f"{hours:02d}:{minutes:02d}:{seconds:02d}")
        self._refresh_calibration_status()
        self.system_status.configure(
            text=(
                f"● TELEMETRIA ONLINE  /  {self.last_sender}"
                if online
                else "● AGUARDANDO TELEMETRIA"
            ),
            fg=COLORS["ok"] if online else COLORS["warning"],
        )
        if self.latest_field:
            self._draw_field()
        self.after(500, self._refresh_status)

    def _on_close(self) -> None:
        if not messagebox.askokcancel(
            "Encerrar IHM", "Encerrar a IHM e os processos iniciados por ela?"
        ):
            return
        self._send_stop(show_error=False)
        self.telemetry.stop()
        for process in self.processes.values():
            process.stop()
        self.destroy()


if __name__ == "__main__":
    FutRoboIHM().mainloop()
