"""Rebuild CH584 with a selected WCH compiler; never flashes hardware.

Uses project-relative source/output paths because some WCH GCC releases cannot
write object files to Chinese absolute paths on Windows. No compiler-version gate.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def image(path):
    data, upper, ended = {}, 0, False
    for line in path.read_text(encoding="ascii").splitlines():
        assert line.startswith(":") and not ended, "HEX framing/EOF"
        record = bytes.fromhex(line[1:])
        assert len(record) == record[0] + 5 and sum(record) % 256 == 0, "HEX record"
        count, addr, kind = record[0], int.from_bytes(record[1:3], "big"), record[3]
        payload = record[4:4 + count]
        if kind == 0:
            for offset, byte in enumerate(payload):
                at = upper + addr + offset
                assert 0 <= at < 448 * 1024 and at not in data, "HEX address/overlap"
                data[at] = byte
        elif kind == 1:
            assert count == 0 and addr == 0
            ended = True
        elif kind in (2, 4):
            assert count == 2 and addr == 0
            upper = int.from_bytes(payload, "big") << (4 if kind == 2 else 16)
        else:
            assert kind in (3, 5) and count == 4
    assert ended and 0 in data
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler-bin", required=True, type=Path)
    parser.add_argument("--project", default="CH584M_TFT_HB.wvproj")
    parser.add_argument("--out", required=True, help="Fresh output directory inside this project")
    args = parser.parse_args()
    out = (ROOT / args.out).resolve()
    out.relative_to(ROOT)
    # A fresh directory prevents an old object/image from surviving a failed run.
    out.mkdir(parents=True, exist_ok=False)
    result = {"verified": False, "flashed": False, "hardware_verified": False}
    report = out / "result.json"
    report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    compiler_bin = args.compiler_bin.resolve()
    candidates = sorted(compiler_bin.glob("*gcc.exe"))
    if len(candidates) != 1:
        raise ValueError("Select a WCH compiler bin directory containing one *gcc.exe")
    gcc = candidates[0]
    prefix = gcc.name[:-len("gcc.exe")]
    tool = lambda name: str(compiler_bin / (prefix + name + ".exe"))
    project = (ROOT / args.project).resolve()
    project.relative_to(ROOT)
    config = json.loads(project.read_text(encoding="utf-8"))["buildConfig"]["configurations"][0]
    excluded = [x.replace("${project}/", "").rstrip("/") for x in config["excludeResources"]]
    # Discover all current inputs, including new source files; do not replay an old list.
    def included(path):
        relative = path.relative_to(ROOT).as_posix()
        return not any(relative == x or relative.startswith(x + "/") for x in excluded)
    sources = sorted(p for p in ROOT.rglob("*")
                     if p.is_file() and p.suffix in (".c", ".S", ".s") and included(p))
    inputs = sorted(p for p in ROOT.rglob("*") if p.is_file() and included(p)
                    and p.suffix.lower() in (".c", ".s", ".h", ".ld", ".a", ".wvproj"))
    manifest = {p.relative_to(ROOT).as_posix(): sha(p) for p in inputs}
    manifest[project.relative_to(ROOT).as_posix()] = sha(project)
    manifest["tools/build_wch.py"] = sha(Path(__file__))
    assert config["riscvTargetProcessor"]["integer_ABI"] == "ilp32"
    assert config["optimization"]["level"] == "size"
    # Current CH584 target processor settings, checked against WCH's CH585 template.
    common = ["-march=rv32imc_zba_zbb_zbc_zbs_xw", "-mabi=ilp32", "-mcmodel=medany",
              "-msmall-data-limit=8", "-mno-save-restore", "-fmax-errors=20", "-Os",
              "-fmessage-length=0", "-fsigned-char", "-ffunction-sections", "-fdata-sections",
              "-fno-common", "--param=highcode-gen-section-name=1", "-g"]
    defines = ["-D" + x for x in config["ccompiler"]["preprocessor"]["defined_symbols"]]
    includes = ["-I" + x.replace("${project}", ".")
                for x in config["ccompiler"]["includes"]["include_paths"]]
    relative = lambda p: p.relative_to(ROOT).as_posix()
    commands = []
    log = out / "build.log"
    def run(command):
        commands.append(command)
        with log.open("ab") as stream:
            stream.write((subprocess.list2cmdline(command) + "\n").encode("utf-8"))
            process = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        if process.returncode:
            raise RuntimeError(f"Command failed ({process.returncode}); see {log}")
    version = subprocess.check_output([str(gcc), "--version"], cwd=ROOT).decode(errors="replace")
    (out / "compiler.txt").write_text(version, encoding="utf-8")
    objects = []
    for source in sources:
        obj = out / "obj" / source.relative_to(ROOT).with_suffix(".o")
        obj.parent.mkdir(parents=True, exist_ok=True)
        objects.append(relative(obj))
        flags = defines + includes + ["-std=" + config["ccompiler"]["optimization"]["language_standard"]] if source.suffix == ".c" else ["-x", "assembler-with-cpp"]
        run([str(gcc), *common, *flags, "-MMD", "-MP", "-c", relative(source), "-o", relative(obj)])
    base = out / "CH584M_TFT_HB"
    linker = config["clinker"]
    scripts = [x.replace("${project}", ".") for x in linker["general"]["scriptFiles"]]
    assert len(scripts) == 1 and linker["general"]["do_not_use_standard_start_files"]
    run([str(gcc), *common, "-T", scripts[0], "-nostartfiles", "-Wl,--gc-sections",
         "-Wl,--print-memory-usage", "-Wl,-Map," + relative(base.with_suffix(".map")),
         "--specs=nano.specs", "--specs=nosys.specs",
         *["-L" + x.replace("${project}", ".") for x in linker["libraries"]["library_search_path"]],
         "-o", relative(base.with_suffix(".elf")), *objects,
         *["-l" + x for x in linker["libraries"]["libraries"]]])
    run([tool("objcopy"), "-O", "ihex", relative(base.with_suffix(".elf")), relative(base.with_suffix(".hex"))])
    run([tool("size"), relative(base.with_suffix(".elf"))])
    for name, command in {
        "elf.txt": [tool("readelf"), "-h", "-A", relative(base.with_suffix(".elf"))],
        "startup.txt": [tool("objdump"), "-d", "--section=.init", relative(base.with_suffix(".elf"))],
        "symbols.txt": [tool("nm"), "-n", relative(base.with_suffix(".elf"))],
    }.items():
        (out / name).write_bytes(subprocess.check_output(command, cwd=ROOT))
    elf = (out / "elf.txt").read_text()
    assert re.search(r"Entry point address:\s+0x0", elf) and "soft-float ABI" in elf
    assert "handle_reset" in (out / "startup.txt").read_text()
    symbols = (out / "symbols.txt").read_text()
    for name in ("main", "_vector_base", "__highcode_init", "UART3_IRQHandler", "CH58x_BLEInit"):
        assert re.search(r"\b" + name + r"$", symbols, re.M), name
    data = image(base.with_suffix(".hex"))
    # Confirm every inspected input stayed unchanged for the whole build.
    assert all(sha(ROOT / name) == digest for name, digest in manifest.items())
    text = log.read_text(encoding="utf-8", errors="replace")
    result.update(verified=True, compiler=version.splitlines()[0], source_count=len(sources),
                  warnings=len(re.findall(r"warning:", text)), input_sha256=manifest,
                  commands=commands, hex_min=min(data), hex_max=max(data), hex_bytes=len(data),
                  hex_sha256=sha(base.with_suffix(".hex")))
    report.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"PASS: {len(sources)} sources; {result['warnings']} warnings; {base.with_suffix('.hex')}")


if __name__ == "__main__":
    main()
