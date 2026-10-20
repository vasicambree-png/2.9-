"""Rebuild the project and produce an isolated UI timing diagnostic image."""
from pathlib import Path
import argparse
import hashlib
import os
import shlex
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "obj/ui_latency_check"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler-bin", type=Path, required=True)
    parser.add_argument("--make-bin", type=Path, required=True)
    args = parser.parse_args()
    gcc = args.compiler_bin / "riscv-wch-elf-gcc.exe"
    make = args.make_bin / "make.exe"
    objcopy = args.compiler_bin / "riscv-wch-elf-objcopy.exe"
    for tool in (gcc, make, objcopy):
        if not tool.is_file():
            raise FileNotFoundError(tool)
    env = os.environ.copy()
    env["PATH"] = os.pathsep.join((str(args.compiler_bin), str(args.make_bin), env["PATH"]))
    OUT.mkdir(parents=True, exist_ok=True)

    def run(command, log):
        with (OUT / log).open("wb") as output:
            subprocess.run(command, cwd=ROOT / "obj", env=env,
                           stdout=output, stderr=subprocess.STDOUT, check=True)

    # Force recompilation of every input: no old object masquerades as evidence.
    run([str(make), "-B", "-j4", "all"], "firmware_build.log")
    shutil.copy2(ROOT / "obj/CH584M_TFT_HB.hex", OUT / "optimized.hex")
    dry = subprocess.check_output([str(make), "-n", "-B", "main-build"],
                                  cwd=ROOT / "obj", env=env).decode("utf-8")
    lines = [shlex.split(line.strip()) for line in dry.splitlines()
             if line.strip().startswith("riscv-wch-elf-gcc ")]
    compile_command = next(cmd for cmd in lines if "../APP/Usart3_task.c" in cmd)
    tft_compile_command = next(cmd for cmd in lines if "../APP/yuying_TFT.c" in cmd)
    link_command = next(cmd for cmd in lines if "-c" not in cmd and "-o" in cmd)

    def replace_arg(command, option, value):
        for i, arg in enumerate(command):
            if arg == option:
                command[i + 1] = str(value)
                return
            if arg.startswith(option) and option in ("-MF", "-MT"):
                command[i] = option + str(value)
                return
        raise ValueError("Missing compiler option: " + option)

    def variant(name, source, diagnostic, tft_source=None):
        object_path = OUT / (name + ".o")
        command = compile_command.copy()
        command[0] = str(gcc)
        replace_arg(command, "-o", object_path)
        replace_arg(command, "-MF", OUT / (name + ".d"))
        replace_arg(command, "-MT", object_path)
        command[-1] = str(source)
        if diagnostic:
            command.insert(1, "-DUI_LATENCY_DIAG=1")
        run(command, name + "_compile.log")
        tft_object = None
        if diagnostic or tft_source is not None:
            tft_object = OUT / (name + "_tft.o")
            command = tft_compile_command.copy()
            command[0] = str(gcc)
            if diagnostic:
                command.insert(1, "-DUI_LATENCY_DIAG=1")
            if tft_source is not None:
                command[-1] = str(tft_source)
            replace_arg(command, "-o", tft_object)
            replace_arg(command, "-MF", OUT / (name + "_tft.d"))
            replace_arg(command, "-MT", tft_object)
            run(command, name + "_tft_compile.log")
        command = link_command.copy()
        command[0] = str(gcc)
        matches = [i for i, value in enumerate(command)
                   if value.replace("\\", "/").lstrip("./") == "APP/Usart3_task.o"]
        assert len(matches) == 1, "Expected exactly one UART task object in link inputs"
        command[matches[0]] = str(object_path)
        if tft_object is not None:
            matches = [i for i, value in enumerate(command)
                       if value.replace("\\", "/").lstrip("./") == "APP/yuying_TFT.o"]
            assert len(matches) == 1, "Expected exactly one TFT object in link inputs"
            command[matches[0]] = str(tft_object)
        replace_arg(command, "-o", OUT / (name + ".elf"))
        for i, value in enumerate(command):
            if value.startswith("-Wl,-Map,"):
                command[i] = "-Wl,-Map," + str(OUT / (name + ".map"))
        run(command, name + "_link.log")
        run([str(objcopy), "-O", "ihex", str(OUT / (name + ".elf")),
             str(OUT / (name + ".hex"))], name + "_objcopy.log")

    variant("diagnostic", ROOT / "APP/Usart3_task.c", True)
    # Optional local baseline snapshot is rebuilt against the same fresh inputs.
    # This is a comparison image, not proof of the firmware running on the board.
    baseline = OUT / "Usart3_task.before.c"
    if baseline.is_file():
        stock_tft = OUT / "yuying_TFT.before_fast.c"
        variant("baseline", baseline, False, stock_tft if stock_tft.is_file() else None)
    names = ("optimized", "diagnostic", "baseline") if baseline.is_file() else ("optimized", "diagnostic")
    for name in names:
        image = OUT / (name + ".hex")
        print(name, hashlib.sha256(image.read_bytes()).hexdigest(), image)


if __name__ == "__main__":
    main()
