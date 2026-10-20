"""Compare actual pre/post-optimization drawing, using project u8g2 on MSVC."""
from pathlib import Path
import argparse
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "obj/ui_latency_check"
SOURCES = ("u8g2_setup.c u8g2_buffer.c u8g2_hvline.c u8g2_ll_hvline.c "
           "u8g2_box.c u8g2_bitmap.c u8g2_intersection.c u8g2_circle.c "
           "u8g2_font.c u8g2_line.c u8g2_kerning.c u8x8_setup.c u8x8_cad.c "
           "u8x8_display.c u8x8_string.c u8x8_byte.c u8x8_gpio.c u8x8_8x8.c "
           "u8x8_u16toa.c u8x8_capture.c").split()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vcvars", type=Path, required=True)
    args = parser.parse_args()
    assert args.vcvars.is_file()
    quote = lambda p: '"' + str(p) + '"'
    for variant, path in (("stock", OUT / "yuying_TFT.before_fast.c"),
                          ("fast", ROOT / "APP/yuying_TFT.c")):
        folder = OUT / (variant + "_pages")
        folder.mkdir(exist_ok=True)
        source = path.read_text(encoding="utf-8")
        source = source[:source.index("void u8g2Init(u8g2_t *u8g2)")]
        # Unused GNU empty arrays cannot compile on MSVC; host copy only.
        source = source.replace("const uint8_t bmp[]={\n};", "const uint8_t bmp[]={0};")
        (folder / "ui_host_module.c").write_text(source, encoding="utf-8")
        (folder / "CONFIG.h").write_text(
            "#pragma once\n#include <stdint.h>\n#include <stdio.h>\n"
            "#include <string.h>\n#define __HIGH_CODE\n", encoding="utf-8")
        rsp = ["/nologo", "/utf-8", "/W3", "/O2", "/Gy",
               "/I" + quote(folder), "/I" + quote(ROOT / "APP/include"),
               "/I" + quote(ROOT / "u8g2/include"), quote(ROOT / "tools/test_ui_pages.c")]
        rsp += [quote(ROOT / "u8g2" / name) for name in SOURCES]
        rsp += ["/Fe:pages.exe", "/link", "/OPT:REF"]
        (folder / "build.rsp").write_text(" ".join(rsp), encoding="utf-16")
        command = ("@echo off\ncall " + quote(args.vcvars) + " >nul\n"
                   "if errorlevel 1 exit /b 1\ncl @build.rsp\n"
                   "if errorlevel 1 exit /b 1\npages.exe pages.raw\n")
        (folder / "build.cmd").write_text(command, encoding="utf-8")
        result = subprocess.run(["cmd.exe", "/d", "/c", str(folder / "build.cmd")],
                                cwd=folder, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        (folder / "build.log").write_bytes(result.stdout)
        if result.returncode:
            print(result.stdout.decode("utf-8", errors="replace")[-7000:])
            raise SystemExit(result.returncode)
        print(variant, result.stdout.decode("utf-8", errors="replace").splitlines()[-1])
    before = (OUT / "stock_pages/pages.raw").read_bytes()
    after = (OUT / "fast_pages/pages.raw").read_bytes()
    assert len(before) == len(after) and len(before) % 8067 == 0
    assert before == after, "Production pages changed at pixel/state level"
    print("PASS:", len(after) // 8067, "actual UI cases match byte-for-byte, including drawing state")


if __name__ == "__main__":
    main()
