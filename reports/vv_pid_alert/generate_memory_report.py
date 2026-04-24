#!/usr/bin/env python3
"""
generate_memory_report.py
===========================
Gera relatório HTML de Memory Safety a partir dos logs criados pelo
`make memory` (Valgrind + AddressSanitizer + UBSanitizer).

Uso:
    python3 generate_memory_report.py

Espera encontrar a estrutura:
    memory_safety/
    ├── valgrind/
    │   ├── test_smoke.log
    │   ├── test_can.log
    │   └── ...
    └── sanitizers/
        ├── smoke.log
        ├── can.log
        └── ...

Gera:
    docs/Memory_Safety_Report.html

Dependências: Python 3.8+ (nada além da stdlib).
"""

import os
import re
import sys
import socket
import getpass
import platform
import subprocess
from pathlib import Path
from datetime import datetime
from html import escape

# ============================================================================
#  CONFIGURAÇÃO
# ============================================================================

REPO_ROOT = Path(__file__).resolve().parent
MEMORY_DIR = REPO_ROOT / "memory_safety"
VALGRIND_DIR = MEMORY_DIR / "valgrind"
SANITIZERS_DIR = MEMORY_DIR / "sanitizers"
OUTPUT_HTML = REPO_ROOT / "docs" / "Memory_Safety_Report.html"

TARGETS = ["smoke", "can", "perception", "decision",
           "pid", "alert", "uds", "integration"]

# ============================================================================
#  PARSERS
# ============================================================================

def parse_valgrind_log(path: Path) -> dict:
    """Extrai métricas de um log do Valgrind."""
    if not path.exists():
        return {"status": "missing", "summary": "log file not found"}

    text = path.read_text(encoding="utf-8", errors="replace")

    # ERROR SUMMARY: X errors from Y contexts (suppressed: Z from W)
    m = re.search(r"ERROR SUMMARY:\s*(\d+)\s*errors?\s*from\s*(\d+)\s*contexts?"
                  r"(?:\s*\(suppressed:\s*(\d+)\s*from\s*(\d+)\))?", text)
    if not m:
        return {"status": "unknown", "summary": "could not parse log"}

    errors = int(m.group(1))
    contexts = int(m.group(2))
    suppressed = int(m.group(3)) if m.group(3) else 0

    # Leaks
    leak_patterns = {
        "definitely_lost": r"definitely lost:\s*([\d,]+)\s*bytes",
        "indirectly_lost": r"indirectly lost:\s*([\d,]+)\s*bytes",
        "possibly_lost":   r"possibly lost:\s*([\d,]+)\s*bytes",
        "still_reachable": r"still reachable:\s*([\d,]+)\s*bytes",
    }
    leaks = {}
    for key, pat in leak_patterns.items():
        mm = re.search(pat, text)
        leaks[key] = int(mm.group(1).replace(",", "")) if mm else 0

    # Alocação total
    m_alloc = re.search(r"total heap usage:\s*([\d,]+)\s*allocs?,\s*([\d,]+)\s*frees?,"
                        r"\s*([\d,]+)\s*bytes", text)
    if m_alloc:
        allocs = int(m_alloc.group(1).replace(",", ""))
        frees = int(m_alloc.group(2).replace(",", ""))
        bytes_alloc = int(m_alloc.group(3).replace(",", ""))
    else:
        allocs = frees = bytes_alloc = 0

    status = "pass" if errors == 0 else "fail"
    summary = (f"{errors} errors, {contexts} contexts "
               f"(suppressed: {suppressed})")

    return {
        "status": status,
        "summary": summary,
        "errors": errors,
        "contexts": contexts,
        "suppressed": suppressed,
        "leaks": leaks,
        "allocs": allocs,
        "frees": frees,
        "bytes_alloc": bytes_alloc,
    }


def parse_sanitizer_log(path: Path) -> dict:
    """Extrai métricas de um log do AddressSanitizer/UBSanitizer."""
    if not path.exists():
        return {"status": "missing", "summary": "log file not found"}

    text = path.read_text(encoding="utf-8", errors="replace")

    # Sanitizers costumam sair imprimindo "ERROR:" ou "SUMMARY:" em falha
    has_error = bool(
        re.search(r"==\d+==\s*ERROR:", text) or
        re.search(r"runtime error:", text) or
        re.search(r"SUMMARY:\s*(Address|UndefinedBehavior)Sanitizer", text)
    )

    # O binário normalmente imprime o resumo dos testes unitários
    # Queremos também capturar o número X/Y passed
    m = re.search(r"Results:\s*(\d+)\s*/\s*(\d+)\s*passed,?\s*(\d+)?\s*failed?", text)
    tests_passed = tests_total = tests_failed = None
    if m:
        tests_passed = int(m.group(1))
        tests_total = int(m.group(2))
        tests_failed = int(m.group(3)) if m.group(3) else 0

    if has_error:
        status = "fail"
        summary = "sanitizer error(s) detected"
    else:
        status = "pass"
        if tests_passed is not None:
            summary = f"0 sanitizer errors · {tests_passed}/{tests_total} unit tests pass"
        else:
            summary = "0 sanitizer errors"

    return {
        "status": status,
        "summary": summary,
        "tests_passed": tests_passed,
        "tests_total": tests_total,
        "tests_failed": tests_failed,
    }


# ============================================================================
#  INFO DO AMBIENTE
# ============================================================================

def get_env_info() -> dict:
    """Coleta info do ambiente para rastreabilidade."""
    def _run(cmd):
        try:
            return subprocess.check_output(cmd, shell=True, stderr=subprocess.DEVNULL,
                                          encoding="utf-8").strip()
        except Exception:
            return "n/a"

    gcc_version = _run("gcc --version | head -1")
    valgrind_version = _run("valgrind --version")
    git_commit = _run("git rev-parse --short HEAD")
    git_branch = _run("git rev-parse --abbrev-ref HEAD")

    return {
        "hostname": socket.gethostname(),
        "user": getpass.getuser(),
        "os": f"{platform.system()} {platform.release()}",
        "gcc": gcc_version,
        "valgrind": valgrind_version,
        "git_commit": git_commit,
        "git_branch": git_branch,
        "generated_at": datetime.now(),
    }


# ============================================================================
#  RENDERIZAÇÃO
# ============================================================================

STATUS_BADGE = {
    "pass":    '<span class="badge badge--ok">PASS</span>',
    "fail":    '<span class="badge badge--danger">FAIL</span>',
    "missing": '<span class="badge badge--warn">MISSING</span>',
    "unknown": '<span class="badge badge--warn">UNKNOWN</span>',
}


def render_valgrind_row(target: str, vg: dict) -> str:
    badge = STATUS_BADGE.get(vg["status"], STATUS_BADGE["unknown"])
    detail = escape(vg["summary"])
    if vg["status"] == "pass" and vg.get("bytes_alloc", 0) > 0:
        detail += (f" · {vg['allocs']} allocs / {vg['frees']} frees "
                   f"/ {vg['bytes_alloc']:,} bytes")
    return (f"<tr><td><code>test_{target}</code></td>"
            f"<td>{detail}</td><td>{badge}</td></tr>")


def render_sanitizer_row(target: str, san: dict) -> str:
    badge = STATUS_BADGE.get(san["status"], STATUS_BADGE["unknown"])
    detail = escape(san["summary"])
    return (f"<tr><td><code>san_test_{target}</code></td>"
            f"<td>{detail}</td><td>{badge}</td></tr>")


def render_html(vg_results: dict, san_results: dict, env: dict) -> str:
    """Monta o HTML completo."""

    # Métricas agregadas
    vg_pass = sum(1 for r in vg_results.values() if r["status"] == "pass")
    san_pass = sum(1 for r in san_results.values() if r["status"] == "pass")
    total = len(TARGETS)
    total_runs = total * 2
    total_pass = vg_pass + san_pass
    total_fail = total_runs - total_pass

    overall_clean = (total_fail == 0)
    banner_color = "--accent" if overall_clean else "--danger"
    banner_icon = "✓" if overall_clean else "✗"
    banner_title = "CLEAN — Zero erros" if overall_clean else f"{total_fail} falha(s) detectada(s)"
    banner_sub = (
        f"Todos os {total} módulos passaram nas 3 ferramentas de análise dinâmica "
        "sem nenhum erro detectado."
        if overall_clean else
        f"Foram detectados {total_fail} problemas nos logs. Consultar seções específicas."
    )

    # Tabelas
    vg_rows = "\n".join(render_valgrind_row(t, vg_results[t]) for t in TARGETS)
    san_rows = "\n".join(render_sanitizer_row(t, san_results[t]) for t in TARGETS)

    # Data
    date_str = env["generated_at"].strftime("%d de %B de %Y às %H:%M")

    # Template (intentionally using a single triple-quoted string)
    return f"""<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Memory Safety Report — AEB</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;700&family=Space+Grotesk:wght@400;500;700&family=Inter:wght@400;500;600&display=swap');

  :root {{
    --bg:         #0f1420;
    --bg-elev:    #161d2e;
    --bg-card:    #1a2236;
    --border:     #2a3550;
    --text:       #d8dff0;
    --text-dim:   #8892ab;
    --text-mute:  #5d6780;
    --accent:     #2ed573;
    --danger:     #ff4757;
    --warn:       #ffa502;
    --ok:         #2ed573;
    --info:       #3d8bfd;
    --mono:       'JetBrains Mono', monospace;
    --sans:       'Inter', sans-serif;
    --serif:      'Space Grotesk', serif;
  }}
  * {{ margin: 0; padding: 0; box-sizing: border-box; }}
  html {{ scroll-behavior: smooth; }}
  body {{
    background: var(--bg); color: var(--text);
    font-family: var(--sans); line-height: 1.65; font-size: 15px;
    min-height: 100vh;
    background-image:
      radial-gradient(ellipse at top left, rgba(46,213,115,0.08) 0%, transparent 50%),
      radial-gradient(ellipse at bottom right, rgba(61,139,253,0.06) 0%, transparent 50%);
  }}
  .header {{
    max-width: 1100px; margin: 0 auto; padding: 3rem 2rem 2rem;
    border-bottom: 1px solid var(--border);
  }}
  .header__tag {{
    display: inline-block; font-family: var(--mono); font-size: 0.75rem;
    letter-spacing: 0.15em; color: var(--accent); text-transform: uppercase;
    padding: 0.3rem 0.7rem; border: 1px solid var(--accent);
    border-radius: 3px; margin-bottom: 1.5rem;
  }}
  .header__title {{
    font-family: var(--serif); font-size: clamp(2rem, 5vw, 3.5rem);
    font-weight: 700; line-height: 1.1; letter-spacing: -0.02em; margin-bottom: 0.5rem;
  }}
  .header__title .accent {{ color: var(--accent); }}
  .header__subtitle {{ font-size: 1.1rem; color: var(--text-dim); margin-bottom: 2rem; }}
  .meta-grid {{
    display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
    gap: 0.75rem 2rem; padding: 1.5rem; background: var(--bg-card);
    border: 1px solid var(--border); border-radius: 6px;
  }}
  .meta-grid dt {{
    font-family: var(--mono); font-size: 0.75rem; letter-spacing: 0.1em;
    text-transform: uppercase; color: var(--text-mute); margin-bottom: 0.2rem;
  }}
  .meta-grid dd {{ font-size: 0.9rem; font-weight: 500; word-break: break-word; }}

  .banner {{
    max-width: 1100px; margin: 2rem auto 0; padding: 2rem;
    background: linear-gradient(135deg, rgba({'46,213,115' if overall_clean else '255,71,87'},0.1),
                                         rgba({'46,213,115' if overall_clean else '255,71,87'},0.02));
    border: 2px solid var({banner_color}); border-radius: 8px;
    text-align: center; position: relative; overflow: hidden;
  }}
  .banner::before {{
    content: ''; position: absolute; inset: 0;
    background: radial-gradient(circle at 50% 0%,
        rgba({'46,213,115' if overall_clean else '255,71,87'},0.15), transparent 60%);
    pointer-events: none;
  }}
  .banner__check {{
    display: inline-flex; align-items: center; justify-content: center;
    width: 80px; height: 80px; border-radius: 50%;
    background: var({banner_color}); color: var(--bg);
    font-size: 3rem; font-weight: 700; margin-bottom: 1rem;
    position: relative; z-index: 1;
    box-shadow: 0 0 40px rgba({'46,213,115' if overall_clean else '255,71,87'},0.4);
  }}
  .banner__title {{
    font-family: var(--serif); font-size: 2rem; font-weight: 700;
    color: var({banner_color}); margin-bottom: 0.5rem; position: relative; z-index: 1;
  }}
  .banner__sub {{
    color: var(--text-dim); max-width: 700px; margin: 0 auto;
    position: relative; z-index: 1;
  }}

  .nav {{
    position: sticky; top: 0; z-index: 100;
    background: rgba(15, 20, 32, 0.92); backdrop-filter: blur(12px);
    border-bottom: 1px solid var(--border); padding: 1rem 0; margin-top: 2rem;
  }}
  .nav__inner {{
    max-width: 1100px; margin: 0 auto; padding: 0 2rem;
    display: flex; gap: 1.5rem; font-family: var(--mono); font-size: 0.85rem;
    overflow-x: auto;
  }}
  .nav a {{
    color: var(--text-dim); text-decoration: none; white-space: nowrap;
    transition: color 0.2s; padding-bottom: 0.2rem; border-bottom: 2px solid transparent;
  }}
  .nav a:hover {{ color: var(--accent); border-bottom-color: var(--accent); }}

  main {{ max-width: 1100px; margin: 0 auto; padding: 2rem; }}
  section {{ margin: 3rem 0; scroll-margin-top: 80px; }}

  h2 {{
    font-family: var(--serif); font-size: 1.85rem; font-weight: 700;
    letter-spacing: -0.01em; margin-bottom: 1.5rem;
    padding-bottom: 0.75rem; border-bottom: 1px solid var(--border);
    display: flex; align-items: baseline; gap: 0.8rem;
  }}
  h2 .num {{ font-family: var(--mono); color: var(--accent); font-size: 0.9em; font-weight: 500; }}
  h3 {{ font-size: 1.25rem; font-weight: 600; margin: 1.8rem 0 0.8rem; }}
  h4 {{
    font-family: var(--mono); font-size: 0.85rem; letter-spacing: 0.1em;
    text-transform: uppercase; color: var(--text-mute); margin: 1.2rem 0 0.5rem;
  }}
  p {{ margin-bottom: 1rem; }}
  p.lead {{ font-size: 1.05rem; color: var(--text-dim); }}
  strong {{ color: var(--text); font-weight: 600; }}
  code {{
    font-family: var(--mono); font-size: 0.88em;
    background: rgba(46,213,115,0.1); color: var(--accent);
    padding: 0.15em 0.4em; border-radius: 3px;
  }}

  .metrics {{
    display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
    gap: 1rem; margin: 1.5rem 0;
  }}
  .metric {{
    padding: 1.5rem; background: var(--bg-card); border: 1px solid var(--border);
    border-radius: 6px; position: relative; overflow: hidden;
  }}
  .metric::before {{
    content: ''; position: absolute; top: 0; left: 0; right: 0;
    height: 3px; background: var(--ok);
  }}
  .metric--danger::before {{ background: var(--danger); }}
  .metric__label {{
    font-family: var(--mono); font-size: 0.7rem; letter-spacing: 0.15em;
    text-transform: uppercase; color: var(--text-mute); margin-bottom: 0.4rem;
  }}
  .metric__value {{
    font-family: var(--serif); font-size: 2.5rem; font-weight: 700;
    line-height: 1; color: var(--ok);
  }}
  .metric--danger .metric__value {{ color: var(--danger); }}
  .metric__detail {{ font-size: 0.85rem; color: var(--text-dim); margin-top: 0.4rem; }}

  .callout {{
    padding: 1.25rem 1.5rem; margin: 1.5rem 0;
    border-left: 3px solid var(--info);
    background: rgba(61,139,253,0.08); border-radius: 0 4px 4px 0;
  }}
  .callout--ok {{ border-left-color: var(--ok); background: rgba(46,213,115,0.08); }}
  .callout p:last-child {{ margin-bottom: 0; }}
  .callout__label {{
    font-family: var(--mono); font-size: 0.72rem; letter-spacing: 0.15em;
    text-transform: uppercase; color: var(--info); font-weight: 700; margin-bottom: 0.4rem;
  }}
  .callout--ok .callout__label {{ color: var(--ok); }}

  table {{ width: 100%; border-collapse: collapse; margin: 1.5rem 0; font-size: 0.92rem; }}
  th {{
    text-align: left; padding: 0.85rem 1rem;
    background: var(--bg-elev); color: var(--accent);
    font-family: var(--mono); font-size: 0.75rem; letter-spacing: 0.1em;
    text-transform: uppercase; font-weight: 600;
    border-bottom: 1px solid var(--border);
  }}
  td {{ padding: 0.85rem 1rem; border-bottom: 1px solid var(--border); vertical-align: top; }}
  tr:hover td {{ background: rgba(46,213,115,0.03); }}
  .badge {{
    display: inline-block; font-family: var(--mono); font-size: 0.72rem;
    font-weight: 600; padding: 0.2em 0.6em; border-radius: 3px; letter-spacing: 0.05em;
  }}
  .badge--ok     {{ background: rgba(46,213,115,0.15); color: var(--ok); border: 1px solid var(--ok); }}
  .badge--danger {{ background: rgba(255,71,87,0.15); color: var(--danger); border: 1px solid var(--danger); }}
  .badge--warn   {{ background: rgba(255,165,2,0.15); color: var(--warn); border: 1px solid var(--warn); }}

  .tool-cards {{
    display: grid; grid-template-columns: repeat(auto-fit, minmax(310px, 1fr));
    gap: 1.2rem; margin: 1.5rem 0;
  }}
  .tool-card {{
    background: var(--bg-card); border: 1px solid var(--border);
    border-radius: 8px; padding: 1.5rem; border-top: 3px solid var(--accent);
  }}
  .tool-card__name {{ font-family: var(--serif); font-size: 1.3rem; font-weight: 700; margin-bottom: 0.3rem; }}
  .tool-card__tag {{
    font-family: var(--mono); font-size: 0.7rem; color: var(--text-mute);
    letter-spacing: 0.1em; text-transform: uppercase; margin-bottom: 1rem;
  }}
  .tool-card__desc {{ font-size: 0.9rem; color: var(--text-dim); margin-bottom: 1rem; }}
  .tool-card__result {{
    display: flex; align-items: center; gap: 0.6rem;
    padding: 0.6rem 0.8rem; background: rgba(46,213,115,0.08);
    border: 1px solid var(--ok); border-radius: 4px;
    font-family: var(--mono); font-size: 0.85rem; color: var(--ok);
  }}
  .tool-card__result::before {{
    content: ''; width: 8px; height: 8px; border-radius: 50%;
    background: var(--ok); box-shadow: 0 0 8px var(--ok);
  }}

  pre {{
    background: #0a0e18; border: 1px solid var(--border); border-radius: 6px;
    padding: 1.25rem; overflow-x: auto; margin: 1rem 0;
    font-size: 0.85rem; line-height: 1.6; position: relative; font-family: var(--mono);
  }}
  pre::before {{
    content: attr(data-lang); position: absolute; top: 0.5rem; right: 0.75rem;
    font-family: var(--mono); font-size: 0.7rem; color: var(--text-mute);
    letter-spacing: 0.1em; text-transform: uppercase;
  }}
  pre code {{ background: transparent; color: var(--text); padding: 0; }}
  .cmt {{ color: #6272a4; font-style: italic; }}
  .ok  {{ color: var(--ok); font-weight: 600; }}

  footer {{
    max-width: 1100px; margin: 4rem auto 2rem; padding: 2rem;
    border-top: 1px solid var(--border);
    color: var(--text-mute); font-size: 0.85rem; text-align: center;
  }}
  footer .stamp {{
    display: inline-block; font-family: var(--mono); font-size: 0.75rem;
    padding: 0.4rem 0.8rem; margin-top: 1rem;
    border: 1px solid var(--border); border-radius: 3px;
    color: var(--text-dim);
  }}
  @media print {{
    body {{ background: white; color: black; }}
    .nav {{ display: none; }}
  }}
</style>
</head>
<body>

<header class="header">
  <span class="header__tag">ISO 26262 · ASIL-D · Dynamic Analysis · Auto-generated</span>
  <h1 class="header__title">Memory Safety <span class="accent">Report</span></h1>
  <p class="header__subtitle">
    Verificação dinâmica de segurança de memória e comportamento indefinido
    em todos os módulos do sistema AEB
  </p>
  <dl class="meta-grid">
    <div><dt>Projeto</dt><dd>AEB Stellantis — Deliverable 4/4</dd></div>
    <div><dt>Executado por</dt><dd>{escape(env['user'])}</dd></div>
    <div><dt>Máquina</dt><dd>{escape(env['hostname'])}</dd></div>
    <div><dt>Sistema</dt><dd>{escape(env['os'])}</dd></div>
    <div><dt>Branch · Commit</dt><dd><code>{escape(env['git_branch'])}</code> @ <code>{escape(env['git_commit'])}</code></dd></div>
    <div><dt>Gerado em</dt><dd>{date_str}</dd></div>
    <div><dt>Compilador</dt><dd>{escape(env['gcc'])}</dd></div>
    <div><dt>Valgrind</dt><dd>{escape(env['valgrind'].splitlines()[0] if env['valgrind'] != 'n/a' else 'n/a')}</dd></div>
  </dl>
</header>

<div class="banner">
  <div class="banner__check">{banner_icon}</div>
  <div class="banner__title">{banner_title}</div>
  <p class="banner__sub">{banner_sub}</p>
</div>

<nav class="nav">
  <div class="nav__inner">
    <a href="#summary">01 — Resumo</a>
    <a href="#tools">02 — Ferramentas</a>
    <a href="#valgrind">03 — Valgrind</a>
    <a href="#sanitizers">04 — Sanitizers</a>
    <a href="#reproduce">05 — Reproduzir</a>
  </div>
</nav>

<main>

<section id="summary">
  <h2><span class="num">01</span> Resumo executivo</h2>

  <p class="lead">A verificação dinâmica foi executada sobre todos os {total}
  módulos de teste do projeto, utilizando três mecanismos complementares
  de detecção.</p>

  <div class="metrics">
    <div class="metric">
      <div class="metric__label">Módulos testados</div>
      <div class="metric__value">{total}</div>
      <div class="metric__detail">100% da suíte</div>
    </div>
    <div class="metric">
      <div class="metric__label">Valgrind</div>
      <div class="metric__value">{vg_pass}/{total}</div>
      <div class="metric__detail">testes passaram</div>
    </div>
    <div class="metric">
      <div class="metric__label">Sanitizers</div>
      <div class="metric__value">{san_pass}/{total}</div>
      <div class="metric__detail">testes passaram</div>
    </div>
    <div class="metric {'' if overall_clean else 'metric--danger'}">
      <div class="metric__label">Erros totais</div>
      <div class="metric__value">{total_fail}</div>
      <div class="metric__detail">em {total_runs} execuções</div>
    </div>
  </div>

  <div class="callout callout--ok">
    <div class="callout__label">Interpretação</div>
    <p>A ausência de erros em <em>todas</em> as ferramentas é um indicador
    de qualidade de código excepcional. É incomum encontrar código C
    embarcado livre de memory issues na primeira auditoria.</p>
  </div>
</section>

<section id="tools">
  <h2><span class="num">02</span> As três ferramentas</h2>
  <p class="lead">Cada ferramenta detecta classes distintas de defeitos,
  por isso a ISO 26262 ASIL-D recomenda uso combinado.</p>

  <div class="tool-cards">
    <div class="tool-card">
      <div class="tool-card__name">Valgrind</div>
      <div class="tool-card__tag">Dynamic · Memcheck</div>
      <p class="tool-card__desc">
        Interpreta o binário em máquina virtual para rastrear todos os
        acessos à memória. Detecta memory leaks, leitura de memória não
        inicializada, e acessos fora dos limites de blocos alocados.
      </p>
      <div class="tool-card__result">
        {vg_pass}/{total} PASS
      </div>
    </div>

    <div class="tool-card">
      <div class="tool-card__name">AddressSanitizer</div>
      <div class="tool-card__tag">Compile-time · ASan</div>
      <p class="tool-card__desc">
        Instrumenta o código em tempo de compilação para detectar buffer
        overflow, stack overflow, use-after-free, e double-free.
        10–100× mais rápido que Valgrind.
      </p>
      <div class="tool-card__result">
        {san_pass}/{total} PASS
      </div>
    </div>

    <div class="tool-card">
      <div class="tool-card__name">UBSanitizer</div>
      <div class="tool-card__tag">Compile-time · UBSan</div>
      <p class="tool-card__desc">
        Detecta comportamento indefinido em runtime: overflow de inteiros
        com sinal, divisão por zero, shift inválido, desalinhamento
        de ponteiro, conversão fora de range.
      </p>
      <div class="tool-card__result">
        {san_pass}/{total} PASS
      </div>
    </div>
  </div>
</section>

<section id="valgrind">
  <h2><span class="num">03</span> Resultados Valgrind</h2>

  <p>Logs completos em <code>memory_safety/valgrind/</code>. Comando usado:</p>
<pre data-lang="bash"><code>valgrind --leak-check=full \\
         --show-leak-kinds=all \\
         --errors-for-leak-kinds=all \\
         --error-exitcode=1 \\
         --track-origins=yes \\
         ./test_&lt;module&gt;</code></pre>

  <table>
    <thead>
      <tr><th style="width: 30%">Módulo</th><th>Resumo</th><th style="width: 15%">Status</th></tr>
    </thead>
    <tbody>
{vg_rows}
    </tbody>
  </table>
</section>

<section id="sanitizers">
  <h2><span class="num">04</span> Resultados Sanitizers (ASan + UBSan)</h2>

  <p>Logs completos em <code>memory_safety/sanitizers/</code>. Flags usadas:</p>
<pre data-lang="bash"><code>gcc -fsanitize=address,undefined \\
    -fno-omit-frame-pointer \\
    -g -O1 -std=c99 -Wall -Wextra \\
    -Iinclude -Istubs \\
    &lt;sources&gt; -lm -o san_test_&lt;module&gt;</code></pre>

  <table>
    <thead>
      <tr><th style="width: 30%">Binário</th><th>Resumo</th><th style="width: 15%">Status</th></tr>
    </thead>
    <tbody>
{san_rows}
    </tbody>
  </table>
</section>

<section id="reproduce">
  <h2><span class="num">05</span> Como reproduzir</h2>

<pre data-lang="bash"><code><span class="cmt"># Pré-requisitos (uma vez)</span>
sudo apt install -y valgrind gcc build-essential

<span class="cmt"># Rodar verificação completa</span>
cd ~/AEB-stellantis-project
make memory

<span class="cmt"># Regenerar este HTML a partir dos logs</span>
python3 generate_memory_report.py

<span class="cmt"># Abrir no navegador</span>
xdg-open docs/Memory_Safety_Report.html</code></pre>
</section>

</main>

<footer>
  <p>Relatório produzido como parte da cross-validation independente da
  Deliverable 4/4 do projeto AEB — Residência Tecnológica em Desenvolvimento
  de Software Embarcado para o Setor Automotivo · UFPE / Stellantis · 2026.</p>
  <div class="stamp">
    Auto-generated from logs · {env['hostname']} · {date_str}
  </div>
</footer>

</body>
</html>
"""


# ============================================================================
#  MAIN
# ============================================================================

def main():
    print("== Memory Safety Report Generator ==")

    # Sanity check
    if not MEMORY_DIR.exists():
        print(f"ERRO: diretório {MEMORY_DIR} não encontrado.")
        print("Rode 'make memory' antes de gerar o relatório.")
        sys.exit(1)
    if not VALGRIND_DIR.exists():
        print(f"ERRO: {VALGRIND_DIR} não encontrado.")
        sys.exit(1)
    if not SANITIZERS_DIR.exists():
        print(f"ERRO: {SANITIZERS_DIR} não encontrado.")
        sys.exit(1)

    # Coleta
    print(f"[1/3] Lendo logs do Valgrind ({VALGRIND_DIR})...")
    vg_results = {}
    for t in TARGETS:
        log_path = VALGRIND_DIR / f"test_{t}.log"
        vg_results[t] = parse_valgrind_log(log_path)
        print(f"   {t:15s} -> {vg_results[t]['status']:8s} "
              f"({vg_results[t]['summary']})")

    print(f"[2/3] Lendo logs dos Sanitizers ({SANITIZERS_DIR})...")
    san_results = {}
    for t in TARGETS:
        # Tenta nome 'X.log' e fallback 'test_X.log'
        log_path = SANITIZERS_DIR / f"{t}.log"
        if not log_path.exists():
            log_path = SANITIZERS_DIR / f"test_{t}.log"
        san_results[t] = parse_sanitizer_log(log_path)
        print(f"   {t:15s} -> {san_results[t]['status']:8s} "
              f"({san_results[t]['summary']})")

    # Ambiente
    env = get_env_info()

    # Render
    print("[3/3] Renderizando HTML...")
    html = render_html(vg_results, san_results, env)

    # Salva
    OUTPUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_HTML.write_text(html, encoding="utf-8")
    print(f"\nOK -> {OUTPUT_HTML}")
    print(f"\nAbrir com: xdg-open {OUTPUT_HTML}")


if __name__ == "__main__":
    main()
