"""Create a reproducible, read-only fingerprint of the current PL baseline."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess


PARAMETER_PATTERN = re.compile(r"\b(?:localparam|parameter)\b[^;]*;", re.MULTILINE)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_object:
        for block in iter(lambda: file_object.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source_files(repo: Path):
    candidates = []
    candidates.extend((repo / "sources_1" / "new").glob("*.sv"))
    candidates.extend((repo / "sources_1" / "ip").glob("**/*S00_AXI.v"))
    candidates.extend((repo / "FPGA_project" / "FPGA_project.gen").glob(
        "sources_1/bd/design_1/ipshared/*/sources_1/new/*.sv"
    ))
    candidates.extend((repo / "FPGA_project" / "FPGA_project.gen").glob(
        "sources_1/bd/design_1/ipshared/*/sources_1/ip/**/*.v"
    ))
    for fixed in (
        repo / "component.xml",
        repo / "FPGA_project" / "FPGA_project.xpr",
        repo / "FPGA_project" / "design_1_wrapper.xsa",
        repo / "FPGA_project" / "FPGA_project.runs" / "impl_1" / "design_1_wrapper.bit",
    ):
        if fixed.exists():
            candidates.append(fixed)
    unique = {path.resolve() for path in candidates if path.is_file()}
    return sorted(unique, key=lambda path: str(path).lower())


def git_info(repo: Path):
    def run(*args):
        completed = subprocess.run(
            ["git", "-c", f"safe.directory={repo.as_posix()}", "-C", str(repo), *args],
            check=False, text=True, capture_output=True
        )
        return completed.stdout.strip() if completed.returncode == 0 else completed.stderr.strip()

    return {"commit": run("rev-parse", "HEAD"), "status_porcelain": run("status", "--porcelain")}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    repo = args.repo.resolve()
    files = source_files(repo)
    entries = []
    for path in files:
        entry = {
            "path": str(path.relative_to(repo)),
            "bytes": path.stat().st_size,
            "sha256": sha256(path),
        }
        if path.suffix.lower() in {".sv", ".v"}:
            text = path.read_text(encoding="utf-8", errors="replace")
            entry["parameter_declarations"] = [
                " ".join(match.group(0).split()) for match in PARAMETER_PATTERN.finditer(text)
            ]
        entries.append(entry)

    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "repo": str(repo),
        "policy": "PL source and parameters frozen for CARLA-based calibration",
        "git": git_info(repo),
        "files": entries,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(args.output.resolve())
    print(f"fingerprinted_files={len(entries)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
