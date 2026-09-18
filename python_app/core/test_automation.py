"""
Test Automation and Command Sequencer Engine for Blue Mechanic V1.
Enables script-based multi-step command sequences, continuous looping,
delays, single-step execution, and broadcast execution via Teensy 4.1.
"""

import os
import re
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from PyQt6.QtCore import QObject, QThread, pyqtSignal, pyqtSlot

from .comm_manager import CommManager



@dataclass
class AutomationStep:
    line_number: int
    raw_line: str
    step_type: str  # "COMMAND", "DELAY", "COMMENT"
    command: str = ""
    delay_ms: int = 0
    description: str = ""


PRESET_SWEEP_CAZ = """# -----------------------------------------------------------------
# 1. Ciclo de Varredura Sincronizada C/A/Z (Hardware RMT)
# -----------------------------------------------------------------
# Habilita os motores de passo
E 0 1
WAIT 300

# Define nível de velocidade 4 (Rápida - 150 us/passo)
S 0 4
WAIT 200

# Deslocamento Sincronizado: C=+45.0°, A=-30.0°, Z=+10.00mm
MS 0 45.0 -30.0 10.00
WAIT 1800

# Deslocamento Inverso: C=-90.0°, A=+60.0°, Z=-20.00mm
MS 0 -90.0 60.0 -20.00
WAIT 2200

# Retorno ao Centro: C=+45.0°, A=-30.0°, Z=+10.00mm
MS 0 45.0 -30.0 10.00
WAIT 1800
"""

PRESET_LASER_STRESS = """# -----------------------------------------------------------------
# 2. Teste de Estresse e Rampa dos Lasers 1 e 2
# -----------------------------------------------------------------
# Laser 1: 25% (1024)
L 0 1 1024
WAIT 400

# Laser 1: 50% (2048)
L 0 1 2048
WAIT 400

# Laser 1: 100% (4095)
L 0 1 4095
WAIT 800

# Desliga Laser 1 e aciona Laser 2 em 100%
L 0 1 0
L 0 2 4095
WAIT 800

# Laser 2: 50% (2048)
L 0 2 2048
WAIT 400

# Desliga ambos os lasers
L 0 1 0
L 0 2 0
WAIT 500
"""

PRESET_HOMING_CYCLE = """# -----------------------------------------------------------------
# 3. Ciclo de Homing Completo (Z, C, A)
# -----------------------------------------------------------------
# Garante drivers habilitados
E 0 1
WAIT 200

# Busca fim de curso do atuador linear Z
H 0 Z
WAIT 3000

# Ajusta zero angular do Eixo C (Base Rotativa)
H 0 C
WAIT 1500

# Ajusta zero angular do Eixo A (Pivot dos Lasers)
H 0 A
WAIT 1500
"""

PRESET_BROADCAST_ALL_NODES = """# -----------------------------------------------------------------
# 4. Teste Broadcast em Todos os Nós (CAN ID 0x200 / Node 0)
# -----------------------------------------------------------------
# Habilita drivers em todos os nós
E 0 1
WAIT 400

# Configura velocidade média (nível 3) em todos
S 0 3
WAIT 200

# Movimenta Base C: +45° (400 passos)
M 0 C 400
WAIT 1200

# Retorna Base C: -45° (-400 passos)
M 0 C -400
WAIT 1200

# Movimenta Pivot A: +45° (400 passos)
M 0 A 400
WAIT 1200

# Retorna Pivot A: -45° (-400 passos)
M 0 A -400
WAIT 1200

# Pulso sincronizado nos Lasers 1 e 2
L 0 1 4095
L 0 2 4095
WAIT 600
L 0 1 0
L 0 2 0
WAIT 500
"""

PRESET_SPEED_STAIRS = """# -----------------------------------------------------------------
# 5. Teste de Escalonamento de Velocidades (Níveis 1 a 5)
# -----------------------------------------------------------------
E 0 1
WAIT 200

# Nível 1: Muito Lenta (2000 us/passo)
S 0 1
M 0 C 200
WAIT 1200

# Nível 2: Lenta (800 us/passo)
S 0 2
M 0 C -200
WAIT 800

# Nível 3: Média (400 us/passo)
S 0 3
M 0 C 400
WAIT 800

# Nível 4: Rápida (150 us/passo)
S 0 4
M 0 C -400
WAIT 600

# Nível 5: Turbo (50 us/passo)
S 0 5
M 0 C 800
WAIT 600
M 0 C -800
WAIT 600
"""

PRESET_LASER_NODES_1_4 = """# -----------------------------------------------------------------
# 6. Show DMX Festival Rave: Nós 1 & 4 (Feixes Cruzados em X)
# -----------------------------------------------------------------
# Preparação: Blackout Limpo
L 1 1 0
L 1 2 0
L 4 1 0
L 4 2 0
WAIT 150

# Fase 1: Respiração Analógica Ultra-Suave (Breath Wave)
L 1 1 20
L 1 2 20
L 4 1 20
L 4 2 20
WAIT 25
L 1 1 55
L 1 2 55
L 4 1 55
L 4 2 55
WAIT 25
L 1 1 110
L 1 2 110
L 4 1 110
L 4 2 110
WAIT 25
L 1 1 180
L 1 2 180
L 4 1 180
L 4 2 180
WAIT 25
L 1 1 250
L 1 2 250
L 4 1 250
L 4 2 250
WAIT 25
L 1 1 300
L 1 2 300
L 4 1 300
L 4 2 300
WAIT 80

# Fade Out Suave
L 1 1 240
L 1 2 240
L 4 1 240
L 4 2 240
WAIT 25
L 1 1 170
L 1 2 170
L 4 1 170
L 4 2 170
WAIT 25
L 1 1 100
L 1 2 100
L 4 1 100
L 4 2 100
WAIT 25
L 1 1 45
L 1 2 45
L 4 1 45
L 4 2 45
WAIT 25
L 1 1 15
L 1 2 15
L 4 1 15
L 4 2 15
WAIT 25
L 1 1 0
L 1 2 0
L 4 1 0
L 4 2 0
WAIT 80

# Fase 2: Crossfade Dinâmico Diagonal (A vs B)
L 1 1 40
L 4 2 40
WAIT 20
L 1 1 120
L 4 2 120
WAIT 20
L 1 1 220
L 4 2 220
WAIT 20
L 1 1 300
L 4 2 300
WAIT 60

L 1 1 180
L 4 2 180
L 1 2 70
L 4 1 70
WAIT 20
L 1 1 80
L 4 2 80
L 1 2 170
L 4 1 170
WAIT 20
L 1 1 0
L 4 2 0
L 1 2 300
L 4 1 300
WAIT 60

L 1 2 200
L 4 1 200
WAIT 20
L 1 2 100
L 4 1 100
WAIT 20
L 1 2 30
L 4 1 30
WAIT 20
L 1 2 0
L 4 1 0
WAIT 60

# Fase 3: Strobo Cruzado Rave (14 Hz Diagonal X-Strobe)
L 1 1 300
L 4 2 300
WAIT 35
L 1 1 0
L 4 2 0
WAIT 25

L 1 2 300
L 4 1 300
WAIT 35
L 1 2 0
L 4 1 0
WAIT 25

L 1 1 300
L 4 2 300
WAIT 35
L 1 1 0
L 4 2 0
WAIT 25

L 1 2 300
L 4 1 300
WAIT 35
L 1 2 0
L 4 1 0
WAIT 25

L 1 1 300
L 4 2 300
WAIT 35
L 1 1 0
L 4 2 0
WAIT 25

L 1 2 300
L 4 1 300
WAIT 35
L 1 2 0
L 4 1 0
WAIT 50

# Fase 4: Chaser Rotativo 3D (Giro no X)
L 1 1 280
WAIT 30
L 1 1 0
L 4 1 280
WAIT 30
L 4 1 0
L 4 2 280
WAIT 30
L 4 2 0
L 1 2 280
WAIT 30
L 1 2 0

L 1 1 300
WAIT 25
L 1 1 0
L 4 1 300
WAIT 25
L 4 1 0
L 4 2 300
WAIT 25
L 4 2 0
L 1 2 300
WAIT 25
L 1 2 0

L 1 1 300
WAIT 20
L 1 1 0
L 4 1 300
WAIT 20
L 4 1 0
L 4 2 300
WAIT 20
L 4 2 0
L 1 2 300
WAIT 20
L 1 2 0

L 1 2 300
WAIT 20
L 1 2 0
L 4 2 300
WAIT 20
L 4 2 0
L 4 1 300
WAIT 20
L 4 1 0
L 1 1 300
WAIT 20
L 1 1 0
WAIT 60

# Fase 5: Tesoura Sincronizada (Esquerda vs Direita)
L 1 1 300
L 1 2 300
WAIT 35
L 1 1 0
L 1 2 0
WAIT 30
L 1 1 300
L 1 2 300
WAIT 45
L 1 1 0
L 1 2 0
WAIT 50

L 4 1 300
L 4 2 300
WAIT 35
L 4 1 0
L 4 2 0
WAIT 30
L 4 1 300
L 4 2 300
WAIT 45
L 4 1 0
L 4 2 0
WAIT 60

# Fase 6: The Build-Up & The Drop
L 1 1 260
L 4 2 260
WAIT 40
L 1 1 0
L 4 2 0
WAIT 100

L 1 2 260
L 4 1 260
WAIT 40
L 1 2 0
L 4 1 0
WAIT 100

L 1 1 280
L 4 2 280
WAIT 35
L 1 1 0
L 4 2 0
WAIT 60

L 1 2 280
L 4 1 280
WAIT 35
L 1 2 0
L 4 1 0
WAIT 60

L 1 1 300
L 4 2 300
WAIT 25
L 1 1 0
L 4 2 0
WAIT 20

L 1 2 300
L 4 1 300
WAIT 25
L 1 2 0
L 4 1 0
WAIT 20

L 1 1 300
L 4 2 300
WAIT 25
L 1 1 0
L 4 2 0
WAIT 20

L 1 2 300
L 4 1 300
WAIT 25
L 1 2 0
L 4 1 0
WAIT 20

# Silêncio Suspense (Blackout)
L 1 1 0
L 1 2 0
L 4 1 0
L 4 2 0
WAIT 140

# THE DROP: Full Power 300 no X
L 1 1 300
L 1 2 300
L 4 1 300
L 4 2 300
WAIT 180

L 1 1 0
L 1 2 0
L 4 1 0
L 4 2 0
WAIT 30

L 1 1 300
L 1 2 300
L 4 1 300
L 4 2 300
WAIT 50

L 1 1 0
L 1 2 0
L 4 1 0
L 4 2 0
WAIT 30

L 1 1 300
L 1 2 300
L 4 1 300
L 4 2 300
WAIT 50

L 1 1 0
L 1 2 0
L 4 1 0
L 4 2 0
WAIT 30

L 1 1 300
L 1 2 300
L 4 1 300
L 4 2 300
WAIT 80

# Fase 7: Dissolução Final
L 1 1 230
L 1 2 230
L 4 1 230
L 4 2 230
WAIT 25

L 1 1 160
L 1 2 160
L 4 1 160
L 4 2 160
WAIT 25

L 1 1 95
L 1 2 95
L 4 1 95
L 4 2 95
WAIT 25

L 1 1 40
L 1 2 40
L 4 1 40
L 4 2 40
WAIT 25

L 1 1 10
L 1 2 10
L 4 1 10
L 4 2 10
WAIT 25

L 1 1 0
L 1 2 0
L 4 1 0
L 4 2 0
WAIT 200
"""

PRESET_MACROS_SHOW = """# -----------------------------------------------------------------
# 8. Demonstração de Macros, Funções & Loops Procedurais
# Feixes em X dos Nós 1 & 4 - Escrito em pouquíssimas linhas!
# -----------------------------------------------------------------
# Preparação: Blackout Limpo
BLACKOUT 0
WAIT 150

# 1. Fade-in Sincronizado Suave nos Dois Lasers (0 a 300)
SYNC_FADE 1 2 4 2 0 300 10 2
WAIT 100

# 2. Fade-out Suave apenas no Laser 1 (300 a 0)
FADE 1 2 300 0 10 3
WAIT 80

# 3. Fade-out Suave no Laser 4 (300 a 0)
FADE 4 2 300 0 10 3
WAIT 120

# 4. Crossfade Suave Dinâmico com Loop FOR e Expressão Matemática {300 - P}
FOR P = 0 TO 300 STEP 15
  L 1 2 {P}
  L 4 2 {300 - P}
  WAIT 15
END

# 5. Estrobo Cruzado Alternado no X (100 ciclos @ 20ms)
STROBE_CROSS 1 2 4 2 300 20 100

# 6. Finalização: Blackout Limpo
BLACKOUT 1
BLACKOUT 4
"""


def _load_fade_strobe_preset() -> str:
    path = os.path.join(os.path.dirname(__file__), "..", "..", "animacao_laser_fade_strobe.txt")
    if os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                return f.read()
        except Exception:
            pass
    return ""


AUTOMATION_PRESETS = {
    "1. Varredura Sincronizada C/A/Z": PRESET_SWEEP_CAZ,
    "2. Teste de Lasers (Fade & Pulso)": PRESET_LASER_STRESS,
    "3. Ciclo de Homing Completo": PRESET_HOMING_CYCLE,
    "4. Broadcast Todos os Nós (Node 0)": PRESET_BROADCAST_ALL_NODES,
    "5. Escalonamento de Velocidades": PRESET_SPEED_STAIRS,
    "6. Show de Lasers (Nós 1 e 4)": PRESET_LASER_NODES_1_4,
    "7. Fade Suave (1 a 300) & 200 Piscadas (Nós 1 e 4)": _load_fade_strobe_preset(),
    "8. Show com Macros & Loops (FOR, FADE, STROBE)": PRESET_MACROS_SHOW,
}


class ScriptExpansionError(Exception):
    """Raised when a macro, loop, or conditional syntax error occurs during expansion."""
    def __init__(self, message: str, line_num: int = 0):
        super().__init__(f"Linha {line_num}: {message}" if line_num else message)
        self.line_num = line_num
        self.raw_message = message


def _fmt_script_num(val: float) -> str:
    """Format float cleanly: integers as '100', floats as '12.5' without trailing zeros."""
    if abs(val - round(val)) < 1e-9:
        return str(int(round(val)))
    return f"{val:.4f}".rstrip("0").rstrip(".")


def _substitute_script_vars(text: str, vars_dict: Dict[str, float]) -> str:
    """Substitute variables in text: {VAR}, $VAR, whole word VAR, and evaluate math in {expr}."""
    for v_name, v_val in vars_dict.items():
        formatted = _fmt_script_num(v_val)
        text = re.sub(rf"\{{{re.escape(v_name)}\}}", formatted, text, flags=re.IGNORECASE)
        text = re.sub(rf"\${re.escape(v_name)}\b", formatted, text, flags=re.IGNORECASE)
        text = re.sub(rf"\b{re.escape(v_name)}\b", formatted, text, flags=re.IGNORECASE)

    # Evaluate inline math expressions inside curly braces: e.g. {300 - P} or {P + 10}
    def _eval_brace_match(m: re.Match) -> str:
        expr = m.group(1).strip()
        if re.match(r"^[0-9\.\s\+\-\*\/\%\(\)]+$", expr):
            try:
                res = float(eval(expr, {"__builtins__": {}}, {}))
                return _fmt_script_num(res)
            except Exception:
                pass
        return m.group(0)

    text = re.sub(r"\{([^{}]+)\}", _eval_brace_match, text)
    return text


def _eval_script_condition(cond_str: str, vars_dict: Dict[str, float], line_num: int) -> bool:
    """Safely evaluate simple conditional expressions (==, !=, <, <=, >, >=, %, and/or)."""
    s = _substitute_script_vars(cond_str, vars_dict).strip()
    if not re.match(r"^[0-9\.\s\+\-\*\/\%\(\)\<\>\=\!\&\|]+$", s):
        raise ScriptExpansionError(f"Expressão condicional inválida: '{cond_str}'", line_num)
    try:
        return bool(eval(s, {"__builtins__": {}}, {}))
    except Exception as e:
        raise ScriptExpansionError(f"Erro ao avaliar condição '{cond_str}': {e}", line_num)


def _expand_single_macro(line: str, line_num: int) -> List[str]:
    """Expands built-in laser macros (FADE, FADE_IN, FADE_OUT, SYNC_FADE, STROBE, STROBE_CROSS, BLACKOUT)."""
    tokens = line.split()
    if not tokens:
        return [line]

    op = tokens[0].upper()

    if op == "BLACKOUT":
        node = tokens[1] if len(tokens) > 1 else "0"
        return [f"L {node} 1 0", f"L {node} 2 0"]

    elif op == "FADE":
        # FADE <node> <chan> <start> <end> <step_ms> [step_inc]
        if len(tokens) < 6:
            raise ScriptExpansionError(
                "Uso incorreto de FADE. Sintaxe: FADE <node> <chan> <start> <end> <step_ms> [step_inc]",
                line_num,
            )
        node, chan = tokens[1], tokens[2]
        start, end = float(tokens[3]), float(tokens[4])
        step_ms = int(float(tokens[5]))
        step_inc = abs(float(tokens[6])) if len(tokens) > 6 else 5.0
        if step_inc <= 0:
            step_inc = 5.0

        out = []
        if start <= end:
            val = start
            while val < end:
                out.append(f"L {node} {chan} {_fmt_script_num(val)}")
                out.append(f"WAIT {step_ms}")
                val += step_inc
            out.append(f"L {node} {chan} {_fmt_script_num(end)}")
            out.append(f"WAIT {step_ms}")
        else:
            val = start
            while val > end:
                out.append(f"L {node} {chan} {_fmt_script_num(val)}")
                out.append(f"WAIT {step_ms}")
                val -= step_inc
            out.append(f"L {node} {chan} {_fmt_script_num(end)}")
            out.append(f"WAIT {step_ms}")
        return out

    elif op == "FADE_IN":
        # FADE_IN <node> <chan> <target> <step_ms> [step_inc]
        if len(tokens) < 5:
            raise ScriptExpansionError(
                "Uso incorreto de FADE_IN. Sintaxe: FADE_IN <node> <chan> <target> <step_ms> [step_inc]",
                line_num,
            )
        node, chan = tokens[1], tokens[2]
        target = tokens[3]
        step_ms = tokens[4]
        step_inc = tokens[5] if len(tokens) > 5 else "5"
        return _expand_single_macro(f"FADE {node} {chan} 0 {target} {step_ms} {step_inc}", line_num)

    elif op == "FADE_OUT":
        # FADE_OUT <node> <chan> [start=300] <step_ms> [step_inc]
        if len(tokens) == 4:
            node, chan, step_ms = tokens[1], tokens[2], tokens[3]
            return _expand_single_macro(f"FADE {node} {chan} 300 0 {step_ms} 5", line_num)
        elif len(tokens) >= 5:
            node, chan = tokens[1], tokens[2]
            start, step_ms = tokens[3], tokens[4]
            step_inc = tokens[5] if len(tokens) > 5 else "5"
            return _expand_single_macro(f"FADE {node} {chan} {start} 0 {step_ms} {step_inc}", line_num)
        else:
            raise ScriptExpansionError(
                "Uso incorreto de FADE_OUT. Sintaxe: FADE_OUT <node> <chan> [start=300] <step_ms> [step_inc]",
                line_num,
            )

    elif op == "SYNC_FADE":
        # SYNC_FADE <node1> <chan1> <node2> <chan2> <start> <end> <step_ms> [step_inc]
        if len(tokens) < 8:
            raise ScriptExpansionError(
                "Uso incorreto de SYNC_FADE. Sintaxe: SYNC_FADE <node1> <chan1> <node2> <chan2> <start> <end> <step_ms> [step_inc]",
                line_num,
            )
        n1, c1, n2, c2 = tokens[1], tokens[2], tokens[3], tokens[4]
        start, end = float(tokens[5]), float(tokens[6])
        step_ms = int(float(tokens[7]))
        step_inc = abs(float(tokens[8])) if len(tokens) > 8 else 5.0
        if step_inc <= 0:
            step_inc = 5.0

        out = []
        if start <= end:
            val = start
            while val < end:
                out.append(f"L {n1} {c1} {_fmt_script_num(val)}")
                out.append(f"L {n2} {c2} {_fmt_script_num(val)}")
                out.append(f"WAIT {step_ms}")
                val += step_inc
            out.append(f"L {n1} {c1} {_fmt_script_num(end)}")
            out.append(f"L {n2} {c2} {_fmt_script_num(end)}")
            out.append(f"WAIT {step_ms}")
        else:
            val = start
            while val > end:
                out.append(f"L {n1} {c1} {_fmt_script_num(val)}")
                out.append(f"L {n2} {c2} {_fmt_script_num(val)}")
                out.append(f"WAIT {step_ms}")
                val -= step_inc
            out.append(f"L {n1} {c1} {_fmt_script_num(end)}")
            out.append(f"L {n2} {c2} {_fmt_script_num(end)}")
            out.append(f"WAIT {step_ms}")
        return out

    elif op == "STROBE":
        # STROBE <node> <chan> <power> <on_ms> <off_ms> <count>
        if len(tokens) < 7:
            raise ScriptExpansionError(
                "Uso incorreto de STROBE. Sintaxe: STROBE <node> <chan> <power> <on_ms> <off_ms> <count>",
                line_num,
            )
        node, chan = tokens[1], tokens[2]
        power = tokens[3]
        on_ms = int(float(tokens[4]))
        off_ms = int(float(tokens[5]))
        count = int(float(tokens[6]))
        out = []
        for _ in range(count):
            out.append(f"L {node} {chan} {power}")
            out.append(f"WAIT {on_ms}")
            out.append(f"L {node} {chan} 0")
            out.append(f"WAIT {off_ms}")
        return out

    elif op == "STROBE_CROSS":
        # STROBE_CROSS <node1> <chan1> <node2> <chan2> <power> <interval_ms> <count>
        if len(tokens) < 7:
            raise ScriptExpansionError(
                "Uso incorreto de STROBE_CROSS. Sintaxe: STROBE_CROSS <node1> <chan1> <node2> <chan2> <power> <interval_ms> <count>",
                line_num,
            )
        n1, c1 = tokens[1], tokens[2]
        n2, c2 = tokens[3], tokens[4]
        power = tokens[5]
        interval_ms = int(float(tokens[6]))
        count = int(float(tokens[7]))
        out = []
        for _ in range(count):
            out.append(f"L {n1} {c1} {power}")
            out.append(f"L {n2} {c2} 0")
            out.append(f"WAIT {interval_ms}")
            out.append(f"L {n1} {c1} 0")
            out.append(f"L {n2} {c2} {power}")
            out.append(f"WAIT {interval_ms}")
        return out

    return [line]


def expand_automation_script(script_text: str, max_steps: int = 100000) -> str:
    """
    Expands high-level automation constructs (FOR, REPEAT, WHILE, IF, DEF, FADE, STROBE, etc.)
    into standard sequential commands (L, M, WAIT, etc.).
    Supports nested blocks, variable substitutions, and arithmetic.
    """
    raw_lines = script_text.splitlines()
    indexed_lines = [(i + 1, line) for i, line in enumerate(raw_lines)]
    user_funcs: Dict[str, Tuple[List[str], List[Tuple[int, str]]]] = {}

    def parse_block(
        idx: int,
        stop_tokens: Tuple[str, ...],
        variables: Dict[str, float],
        step_counter: List[int],
    ) -> Tuple[List[str], int, Optional[str]]:
        result: List[str] = []
        n = len(indexed_lines)

        while idx < n:
            line_num, raw_line = indexed_lines[idx]
            clean = raw_line.strip()

            if not clean:
                idx += 1
                continue

            # Comments
            if clean.startswith("#") or clean.startswith("//"):
                result.append(clean)
                idx += 1
                continue

            first_word = clean.split()[0].upper()
            if first_word in stop_tokens:
                return result, idx + 1, first_word

            # DEF / FUNCTION <name> <params...> ... END / ENDDEF / ENDFUNCTION
            if first_word in ("DEF", "FUNCTION"):
                tokens = clean.split()
                if len(tokens) < 2:
                    raise ScriptExpansionError("Sintaxe de DEF inválida. Ex: DEF NOME_FUNC p1 p2", line_num)
                func_name = tokens[1].upper()
                param_names = tokens[2:]

                cur = idx + 1
                func_body = []
                depth = 1
                while cur < n:
                    c_num, c_line = indexed_lines[cur]
                    c_clean = c_line.strip()
                    c_word = c_clean.split()[0].upper() if c_clean else ""
                    if c_word in ("DEF", "FUNCTION"):
                        depth += 1
                    elif c_word in ("END", "ENDDEF", "ENDFUNCTION"):
                        depth -= 1
                        if depth == 0:
                            break
                    func_body.append((c_num, c_line))
                    cur += 1

                if depth != 0:
                    raise ScriptExpansionError(
                        f"Função '{func_name}' aberta na linha {line_num} não foi fechada com ENDDEF",
                        line_num,
                    )

                user_funcs[func_name] = (param_names, func_body)
                idx = cur + 1
                continue

            # CALL <func_name> <args...> or <func_name> <args...>
            if first_word == "CALL" or first_word in user_funcs:
                tokens = clean.split()
                if first_word == "CALL":
                    if len(tokens) < 2:
                        raise ScriptExpansionError("Uso de CALL inválido. Sintaxe: CALL NOME_FUNC arg1 arg2...", line_num)
                    f_name = tokens[1].upper()
                    f_args = tokens[2:]
                else:
                    f_name = first_word
                    f_args = tokens[1:]

                if f_name not in user_funcs:
                    raise ScriptExpansionError(f"Função '{f_name}' não definida", line_num)

                p_names, f_body = user_funcs[f_name]
                if len(f_args) != len(p_names):
                    raise ScriptExpansionError(
                        f"Função '{f_name}' espera {len(p_names)} argumentos ({', '.join(p_names)}), recebeu {len(f_args)}",
                        line_num,
                    )

                f_vars = dict(variables)
                for p_name, arg_val in zip(p_names, f_args):
                    subbed_arg = _substitute_script_vars(arg_val, variables)
                    try:
                        f_vars[p_name] = float(eval(subbed_arg, {"__builtins__": {}}, {}))
                    except Exception:
                        pass

                b_sub, _, _ = parse_block_subset(f_body, f_vars, step_counter, max_steps)
                result.extend(b_sub)
                idx += 1
                continue

            # REPEAT <count> / WHILE <count>
            repeat_match = re.match(r"^(?:REPEAT|WHILE)\s+(.+)$", clean, re.IGNORECASE)
            if repeat_match:
                expr = _substitute_script_vars(repeat_match.group(1).strip(), variables)
                try:
                    count = int(float(eval(expr, {"__builtins__": {}}, {})))
                except Exception as e:
                    raise ScriptExpansionError(f"Contagem inválida em REPEAT: '{expr}'", line_num)

                body_lines = []
                depth = 1
                cur = idx + 1
                while cur < n:
                    c_num, c_line = indexed_lines[cur]
                    c_clean = c_line.strip()
                    c_word = c_clean.split()[0].upper() if c_clean else ""
                    if c_word in ("REPEAT", "WHILE", "FOR", "IF", "DEF", "FUNCTION"):
                        depth += 1
                    elif c_word in ("END", "ENDREPEAT", "ENDWHILE", "ENDFOR", "ENDIF", "ENDDEF", "ENDFUNCTION"):
                        depth -= 1
                        if depth == 0:
                            break
                    body_lines.append((c_num, c_line))
                    cur += 1

                if depth != 0:
                    raise ScriptExpansionError(f"Bloco REPEAT/WHILE aberto na linha {line_num} não foi fechado com END", line_num)

                idx = cur + 1
                for _ in range(count):
                    b_sub, _, _ = parse_block_subset(body_lines, variables, step_counter, max_steps)
                    result.extend(b_sub)
                continue

            # FOR <var> = <start> TO <end> [STEP <step>]
            for_match = re.match(
                r"^FOR\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.+?)\s+TO\s+(.+?)(?:\s+STEP\s+(.+?))?$",
                clean,
                re.IGNORECASE,
            )
            if for_match:
                var_name = for_match.group(1)
                s_expr = _substitute_script_vars(for_match.group(2).strip(), variables)
                e_expr = _substitute_script_vars(for_match.group(3).strip(), variables)
                step_expr = _substitute_script_vars(for_match.group(4).strip(), variables) if for_match.group(4) else None

                try:
                    start_val = float(eval(s_expr, {"__builtins__": {}}, {}))
                    end_val = float(eval(e_expr, {"__builtins__": {}}, {}))
                    if step_expr:
                        step_val = float(eval(step_expr, {"__builtins__": {}}, {}))
                    else:
                        step_val = 1.0 if start_val <= end_val else -1.0
                except Exception as e:
                    raise ScriptExpansionError(f"Parâmetros de FOR inválidos: {clean} ({e})", line_num)

                if step_val == 0:
                    raise ScriptExpansionError("STEP não pode ser zero no loop FOR", line_num)

                body_lines = []
                depth = 1
                cur = idx + 1
                while cur < n:
                    c_num, c_line = indexed_lines[cur]
                    c_clean = c_line.strip()
                    c_word = c_clean.split()[0].upper() if c_clean else ""
                    if c_word in ("REPEAT", "WHILE", "FOR", "IF", "DEF", "FUNCTION"):
                        depth += 1
                    elif c_word in ("END", "ENDREPEAT", "ENDWHILE", "ENDFOR", "ENDIF", "ENDDEF", "ENDFUNCTION"):
                        depth -= 1
                        if depth == 0:
                            break
                    body_lines.append((c_num, c_line))
                    cur += 1

                if depth != 0:
                    raise ScriptExpansionError(f"Bloco FOR aberto na linha {line_num} não foi fechado com END", line_num)

                idx = cur + 1

                curr = start_val
                iter_count = 0
                max_loop_iters = 50000
                while (curr <= end_val + 1e-9 if step_val > 0 else curr >= end_val - 1e-9):
                    iter_count += 1
                    if iter_count > max_loop_iters:
                        raise ScriptExpansionError(f"Loop FOR excedeu limite de {max_loop_iters} iterações", line_num)

                    sub_vars = dict(variables)
                    sub_vars[var_name] = curr
                    b_sub, _, _ = parse_block_subset(body_lines, sub_vars, step_counter, max_steps)
                    result.extend(b_sub)
                    curr += step_val
                continue

            # IF <cond> ... [ELSE] ... END / ENDIF
            if_match = re.match(r"^IF\s+(.+)$", clean, re.IGNORECASE)
            if if_match:
                cond_str = if_match.group(1).strip()
                cond_res = _eval_script_condition(cond_str, variables, line_num)

                then_lines = []
                else_lines = []
                depth = 1
                cur = idx + 1
                in_else = False

                while cur < n:
                    c_num, c_line = indexed_lines[cur]
                    c_clean = c_line.strip()
                    c_word = c_clean.split()[0].upper() if c_clean else ""
                    if c_word in ("IF", "FOR", "REPEAT", "WHILE", "DEF", "FUNCTION"):
                        depth += 1
                    elif c_word == "ELSE" and depth == 1:
                        in_else = True
                        cur += 1
                        continue
                    elif c_word in ("END", "ENDIF", "ENDFOR", "ENDREPEAT", "ENDWHILE", "ENDDEF", "ENDFUNCTION"):
                        depth -= 1
                        if depth == 0:
                            break

                    if in_else:
                        else_lines.append((c_num, c_line))
                    else:
                        then_lines.append((c_num, c_line))
                    cur += 1

                if depth != 0:
                    raise ScriptExpansionError(f"Bloco IF aberto na linha {line_num} não foi fechado com END/ENDIF", line_num)

                idx = cur + 1

                chosen_lines = then_lines if cond_res else else_lines
                b_sub, _, _ = parse_block_subset(chosen_lines, variables, step_counter, max_steps)
                result.extend(b_sub)
                continue

            # Regular command or single-line macro: substitute variables first
            sub_line = _substitute_script_vars(clean, variables)
            expanded_macro_lines = _expand_single_macro(sub_line, line_num)
            for mline in expanded_macro_lines:
                step_counter[0] += 1
                if step_counter[0] > max_steps:
                    raise ScriptExpansionError(f"Script excedeu o limite máximo de {max_steps} passos gerados", line_num)
                result.append(mline)

            idx += 1

        return result, idx, None

    def parse_block_subset(
        subset_lines: List[Tuple[int, str]],
        variables: Dict[str, float],
        step_counter: List[int],
        max_steps: int,
    ) -> Tuple[List[str], int, Optional[str]]:
        nonlocal indexed_lines
        old_lines = indexed_lines
        indexed_lines = subset_lines
        try:
            res, next_idx, stop_tok = parse_block(0, (), variables, step_counter)
            return res, next_idx, stop_tok
        finally:
            indexed_lines = old_lines

    counter = [0]
    expanded_lines, _, _ = parse_block(0, (), {}, counter)
    return "\n".join(expanded_lines)


def parse_script(
    script_text: str,
    default_delay_ms: int = 500,
    force_broadcast: bool = False,
    default_node: int = 1,
) -> List[AutomationStep]:
    """
    Parse a text script into executable steps.
    Pre-processes high-level macros (FOR, REPEAT, IF, FADE, STROBE, etc.)
    and handles commands, delays (WAIT/DELAY/SLEEP), comments, and node replacements.
    """
    # Pre-process procedural macros, loops, and conditions
    expanded_text = expand_automation_script(script_text)

    steps: List[AutomationStep] = []
    lines = expanded_text.splitlines()

    for line_idx, line in enumerate(lines, start=1):
        clean = line.strip()
        if not clean:
            continue


        # Comments
        if clean.startswith("#") or clean.startswith("//"):
            steps.append(
                AutomationStep(
                    line_number=line_idx,
                    raw_line=clean,
                    step_type="COMMENT",
                    description=clean.lstrip("#/ ").strip(),
                )
            )
            continue

        # Delays
        delay_match = re.match(
            r"^(?:WAIT|DELAY|SLEEP|PAUSE)\s+([\d.]+)\s*(ms|s|seg)?$",
            clean,
            re.IGNORECASE,
        )
        if delay_match:
            value = float(delay_match.group(1))
            unit = (delay_match.group(2) or "ms").lower()
            if unit in ("s", "seg"):
                delay_ms = int(value * 1000)
            else:
                delay_ms = int(value)
            steps.append(
                AutomationStep(
                    line_number=line_idx,
                    raw_line=clean,
                    step_type="DELAY",
                    delay_ms=max(1, delay_ms),
                    description=f"Aguardar {delay_ms} ms",
                )
            )
            continue

        # Command preparation
        cmd = clean

        # Substitute {node} placeholder
        target_node = 0 if force_broadcast else default_node
        cmd = cmd.replace("{node}", str(target_node))

        # If force_broadcast is true, replace explicit node parameter in standard Teensy actuation commands
        if force_broadcast:
            # Pattern: (COMMAND) <node_number> <args...>
            # Commands: M, MF, MS, MSF, H, E, S, L, F
            teensy_actuation_pattern = re.match(
                r"^(M|MF|MS|MSF|H|E|S|L|F)\s+(\d+)(\s+.*)?$",
                cmd,
                re.IGNORECASE,
            )
            if teensy_actuation_pattern:
                op = teensy_actuation_pattern.group(1).upper()
                rest = teensy_actuation_pattern.group(3) or ""
                cmd = f"{op} 0{rest}"
        else:
            # When force_broadcast is False: preserve explicit node IDs (like 'L 1 2 300')!
            # Only if the user omitted the node argument entirely, insert default_node
            tokens = cmd.split()
            if tokens:
                op = tokens[0].upper()
                args = tokens[1:]
                expected_params = {
                    "M": 3,
                    "MF": 3,
                    "MS": 4,
                    "MSF": 4,
                    "H": 2,
                    "E": 2,
                    "S": 2,
                    "L": 3,
                    "F": 2,
                    "R": 1,
                    "P": 1,
                }
                if op in expected_params and len(args) == expected_params[op] - 1:
                    tokens.insert(1, str(default_node))
                    cmd = " ".join(tokens)

        steps.append(
            AutomationStep(
                line_number=line_idx,
                raw_line=clean,
                step_type="COMMAND",
                command=cmd,
                delay_ms=default_delay_ms,
                description=f"Executar: {cmd}",
            )
        )

    return steps


class AutomationWorker(QThread):
    """
    Background worker thread that executes the automation script.
    Guarantees GUI responsiveness while looping or waiting.
    """

    state_changed = pyqtSignal(str)  # "IDLE", "RUNNING", "PAUSED", "STOPPED", "ERROR"
    step_started = pyqtSignal(int, int, str, int)  # step_idx, total_steps, command, loop_num
    step_completed = pyqtSignal(int, bool, str)  # step_idx, success, message
    loop_completed = pyqtSignal(int)  # loop_num
    finished = pyqtSignal(int, int, int)  # total_loops, total_commands, errors
    log_message = pyqtSignal(str, str, str, str)  # timestamp, direction, message, tag
    progress_updated = pyqtSignal(int, int)  # current_step, total_steps

    def __init__(self, comm: CommManager, parent: Optional[QObject] = None):
        super().__init__(parent)
        self.comm = comm

        self.steps: List[AutomationStep] = []
        self.loop_mode = "INFINITE"  # "INFINITE", "COUNT", "ONCE"
        self.target_loops = 1
        self.default_delay_ms = 500
        self.stop_on_error = False

        self._stop_requested = False
        self._pause_requested = False
        self._step_once_requested = False

        self.current_loop = 0
        self.current_step_idx = 0
        self.total_commands_sent = 0
        self.error_count = 0

    def configure(
        self,
        steps: List[AutomationStep],
        loop_mode: str = "INFINITE",
        target_loops: int = 1,
        default_delay_ms: int = 500,
        stop_on_error: bool = False,
    ) -> None:
        self.steps = steps
        self.loop_mode = loop_mode
        self.target_loops = max(1, target_loops)
        self.default_delay_ms = max(10, default_delay_ms)
        self.stop_on_error = stop_on_error

    def request_stop(self) -> None:
        self._stop_requested = True
        self._pause_requested = False

    def request_pause(self) -> None:
        self._pause_requested = True

    def request_resume(self) -> None:
        self._pause_requested = False

    def request_single_step(self) -> None:
        self._step_once_requested = True

    def _sleep_cancellable(self, duration_ms: int) -> bool:
        """Sleep in small 20ms slices to react quickly to stop/pause requests."""
        elapsed = 0
        slice_ms = 20
        while elapsed < duration_ms:
            if self._stop_requested:
                return False
            time.sleep(slice_ms / 1000.0)
            elapsed += slice_ms
        return True

    def run(self) -> None:
        self._stop_requested = False
        self._pause_requested = False
        self._step_once_requested = False
        self.current_loop = 0
        self.total_commands_sent = 0
        self.error_count = 0

        # Filter executable steps (COMMAND and DELAY)
        executable_steps = [s for s in self.steps if s.step_type in ("COMMAND", "DELAY")]
        total_exec_steps = len(executable_steps)
        has_explicit_delays = any(s.step_type == "DELAY" for s in executable_steps)

        if total_exec_steps == 0:
            self._log("SYS", "Nenhum comando executável encontrado no script.", "warn")
            self.state_changed.emit("STOPPED")
            self.finished.emit(0, 0, 0)
            return

        self.state_changed.emit("RUNNING")
        self._log("SYS", f"Iniciando automação: {total_exec_steps} passos por ciclo.", "info")

        while not self._stop_requested:
            self.current_loop += 1
            self._log(
                "LOOP",
                f"--- Início do Loop #{self.current_loop}"
                + (f" / {self.target_loops}" if self.loop_mode == "COUNT" else "")
                + " ---",
                "highlight",
            )

            for step_i, step in enumerate(executable_steps, start=1):
                if self._stop_requested:
                    break

                # Handle pause
                while self._pause_requested and not self._stop_requested:
                    self.state_changed.emit("PAUSED")
                    time.sleep(0.05)
                if self._stop_requested:
                    break
                self.state_changed.emit("RUNNING")

                self.current_step_idx = step_i
                self.progress_updated.emit(step_i, total_exec_steps)

                if step.step_type == "COMMAND":
                    self.step_started.emit(
                        step_i, total_exec_steps, step.command, self.current_loop
                    )
                    self._log("TX", step.command, "tx")

                    success = self.comm.send_raw(step.command)
                    self.total_commands_sent += 1

                    if not success:
                        self.error_count += 1
                        self.step_completed.emit(
                            step_i, False, f"Falha ao enviar: {step.command}"
                        )
                        self._log("ERR", f"Falha na transmissão: {step.command}", "error")
                        if self.stop_on_error:
                            self._stop_requested = True
                            break
                    else:
                        self.step_completed.emit(step_i, True, "Enviado com sucesso")

                    # Se o próximo passo já for um DELAY explícito (WAIT/DELAY),
                    # não adicionamos espera artificial (apenas 2ms) para que o WAIT execute no tempo exato.
                    # Se o script tiver DELAYs explícitos, comandos consecutivos (ex: acender 2 lasers juntos)
                    # têm pacing ultra-rápido de apenas 10ms.
                    # Apenas se o script NÃO tiver nenhum WAIT/DELAY em todo o arquivo, usamos default_delay_ms.
                    next_is_delay = (
                        step_i < total_exec_steps
                        and executable_steps[step_i].step_type == "DELAY"
                    )
                    if next_is_delay:
                        delay = 2
                    elif has_explicit_delays:
                        delay = 10
                    else:
                        delay = step.delay_ms or self.default_delay_ms

                    if delay > 0 and not self._sleep_cancellable(delay):
                        break

                elif step.step_type == "DELAY":
                    self.step_started.emit(
                        step_i,
                        total_exec_steps,
                        f"Aguardando {step.delay_ms} ms",
                        self.current_loop,
                    )
                    self._log("WAIT", f"Aguardando {step.delay_ms} ms...", "wait")
                    if not self._sleep_cancellable(step.delay_ms):
                        break
                    self.step_completed.emit(step_i, True, f"Pausa de {step.delay_ms} ms concluída")

            if self._stop_requested:
                break

            self.loop_completed.emit(self.current_loop)
            self._log(
                "LOOP",
                f"Ciclo #{self.current_loop} finalizado com sucesso.",
                "success",
            )

            # Check termination for ONCE or COUNT
            if self.loop_mode == "ONCE":
                break
            if self.loop_mode == "COUNT" and self.current_loop >= self.target_loops:
                break

        final_state = "STOPPED"
        if self.error_count > 0 and self.stop_on_error:
            final_state = "ERROR"

        self.state_changed.emit(final_state)
        self._log(
            "SYS",
            f"Automação encerrada. Loops: {self.current_loop}, "
            f"Comandos enviados: {self.total_commands_sent}, Erros: {self.error_count}.",
            "info",
        )
        self.finished.emit(
            self.current_loop, self.total_commands_sent, self.error_count
        )

    def _log(self, direction: str, message: str, tag: str = "") -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_message.emit(timestamp, direction, message, tag)
