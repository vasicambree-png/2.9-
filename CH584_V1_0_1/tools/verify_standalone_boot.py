"""Host replay using production UI_Control, u8g2, ST7305 and task branches.

Only hardware, sensor-request backend, scheduler and time are stubs. Production
CMD03 frame guards/checksum, successful UART dispatch and display branches
execute verbatim. No hardware is used.
Cold first-page, warm soft-off/resume and navigation wake sequences exercise
production parser branches and verify intro-before-page ordering without rank 5.
"""
from pathlib import Path
import argparse
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "obj/standalone_boot_check"
SOURCES = ("u8g2_setup.c u8g2_buffer.c u8g2_hvline.c u8g2_ll_hvline.c "
           "u8g2_box.c u8g2_bitmap.c u8g2_intersection.c u8g2_circle.c "
           "u8g2_font.c u8g2_line.c u8g2_kerning.c u8x8_setup.c u8x8_cad.c "
           "u8x8_display.c u8x8_string.c u8x8_byte.c u8x8_gpio.c u8x8_8x8.c "
           "u8x8_u16toa.c u8x8_capture.c u8g2_d_memory.c u8x8_d_st7305.c").split()


def block(source, marker):
    match = re.search(r"(?m)^[ \t]*" + re.escape(marker) + r"[^;]*?\{", source)
    assert match, marker
    start = match.start()
    cursor = source.index("{", start)
    depth, end = 1, cursor + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vcvars", type=Path, required=True)
    args = parser.parse_args()
    assert args.vcvars.is_file()
    OUT.mkdir(exist_ok=True)
    uart = (ROOT / "APP/Usart3_task.c").read_text(encoding="utf-8")
    tft = (ROOT / "APP/yuying_TFT.c").read_text(encoding="utf-8")
    setup = (ROOT / "u8g2/u8g2_d_setup.c").read_text(encoding="utf-8", errors="replace")
    prefix = tft[:tft.index("void u8g2Init(u8g2_t *u8g2)")]
    prefix = prefix.replace("const uint8_t bmp[]={\n};", "const uint8_t bmp[]={0};")
    (OUT / "ui_host_module.c").write_text(prefix, encoding="utf-8")
    (OUT / "CONFIG.h").write_text(
        "#pragma once\n#include <stdint.h>\n#include <stdio.h>\n"
        "#include <string.h>\n#define __HIGH_CODE\n", encoding="utf-8")
    marker = "if (menu_rank >= 1u && menu_rank <= 4u)"
    field_guard = block(uart[uart.index(marker):], marker)
    assert "pData->menu_rank = menu_rank;" in field_guard
    assert "pData->Type = sensor_type;" in field_guard
    assert "if (param_cnt < 10u) return 5;" in uart
    page_tail = block(uart[uart.rindex(marker):], marker)
    rank5 = uart[uart.index("\n              case 5:"):uart.index("\n              case 6:")]
    rank5 = rank5.split(":", 1)[1].rsplit("break;", 1)[0]
    assert "pData->" not in rank5 and "g_ui_reinit_req = 1" in rank5
    assert "screen_startup_request_artwork(&screen_startup)" in rank5
    assert "test_dirty = 1u" in rank5
    assert "ui_show_msg(" not in rank5
    message = uart[uart.index("\n              case 6:"):uart.index("\n              default:", uart.index("\n              case 6:"))]
    message = message.split(":", 1)[1].rsplit("break;", 1)[0]
    # Save-result handling may read the cached settings draft to update BLE,
    # but messages must still never overwrite the cached page fields.
    assert not re.search(r"pData->[\w.]+\s*(?:=(?!=)|\+=|-=|\+\+|--)", message)
    cmd03 = uart[uart.index("        case 0x03:"):uart.index("        case 0x04:")]
    assert not re.search(r"screen_(?:power|startup)|g_ui_msg|u8g2|UI_Control", cmd03)
    frame_head = uart[uart.index("uint8_t parse_received_frame("):uart.index("    switch(cmd)")]
    frame_head = frame_head.replace("parse_received_frame(", "parse_cmd03_frame(", 1)
    cmd03_parser = frame_head + "switch(cmd) {\n" + cmd03 + "default: break;\n}\nreturn 0;\n}"
    page_key = re.search(r"typedef struct\s*\{[^}]+\}\s*ui_page_key_t;", uart)
    assert page_key
    uart_success = block(uart, "if (result == 0u)")
    cmd03_dispatch = (
        "static uint8_t dispatch_cmd03_frame(uint8_t *packet, uint16_t pack_len, data_LIST *pData) {\n"
        "ui_page_key_t before=ui_current_page_key();\n"
        "uint32_t received_at=TMOS_GetSystemClock(), old_dis_flag=dis_flag_cnt;\n"
        "unsigned old_events=event_count; ui_redraw_policy_t old_redraw=ui_redraw;\n"
        "uint8_t result=parse_cmd03_frame(packet,pack_len,pData);\n"
        "for(unsigned iteration=0;iteration<1u;iteration++) {\n" + uart_success + "\n}\n"
        "assert(dis_flag_cnt==old_dis_flag && event_count==old_events);\n"
        "assert(!memcmp(&ui_redraw,&old_redraw,sizeof(ui_redraw)));\nreturn result;\n}")
    display_event = block(uart, "if(events & START_DATA_EVT)")
    assert "screen_startup_artwork_sent(&screen_startup)" in display_event
    real_setup = block(setup, "void u8g2_Setup_st7305_yuying_168x384_f(")
    real_setup = real_setup.replace("u8x8_cad_011, byte_cb", "record_cad, byte_cb")
    production = "\n".join((real_setup,
        block(tft, "void u8g2Init(u8g2_t *u8g2)"),
        block(tft, "void UI_TestArtwork(u8g2_t *u8g2)"),
        block(uart, "void screen_reset_display_init("),
        block(uart, "static uint8_t screen_display_is_off("),
        block(uart, "static uint8_t screen_power_process("),
        block(uart, "void usart_task("),
        block(uart, "uint8_t calculate_checksum("),
        page_key.group(),
        block(uart, "static ui_page_key_t ui_current_page_key("),
        block(uart, "static uint8_t ui_page_changed("),
        cmd03_parser,
        cmd03_dispatch,
        "static uint16_t step_display(void) { uint16_t events=START_DATA_EVT;\n" +
        display_event + "\nreturn 0; }",
        "static void step_reinit(void) {\n" + block(uart, "if (g_ui_reinit_req)") + "\n}",
        "static void accept_page(unsigned menu_rank) { data_LIST *pData=&Data_list1; "
        "uint8_t rank2=pData->rank2_addr, rank3=pData->rank3_addr;\n" + page_tail + "\n}",
        "static void accept_rank5(void) {\n" + rank5 + "\n}",
        "static unsigned accept_msg(unsigned code) { data_LIST *pData=&Data_list1; "
        "unsigned param_cnt=1; uint16_t param_data[1]; "
        "param_data[0]=(uint16_t)code;\n" + message + "\n return 0; }"))
    (OUT / "startup_paths.c").write_text(production, encoding="utf-8")
    quote = lambda p: '"' + str(p) + '"'
    rsp = ["/nologo", "/utf-8", "/W3", "/O2", "/Gy", "/I" + quote(OUT),
           "/I" + quote(ROOT / "u8g2/include"), "/I" + quote(ROOT / "APP/include"),
           quote(ROOT / "tools/test_startup_integration.c")]
    rsp += [quote(ROOT / "u8g2" / name) for name in SOURCES]
    rsp += ["/Fe:startup_test.exe", "/link", "/OPT:REF"]
    (OUT / "startup.rsp").write_text(" ".join(rsp), encoding="utf-16")
    (OUT / "startup_build.cmd").write_text(
        "@echo off\ncall " + quote(args.vcvars) + " >nul\nif errorlevel 1 exit /b 1\n"
        "cl @startup.rsp\nif errorlevel 1 exit /b 1\nstartup_test.exe\n"
        "if errorlevel 1 exit /b 1\nstartup_test.exe\n", encoding="utf-8")
    result = subprocess.run(["cmd.exe", "/d", "/c", str(OUT / "startup_build.cmd")],
                            cwd=OUT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (OUT / "startup_test.log").write_bytes(result.stdout)
    output = result.stdout.decode("utf-8", errors="replace")
    print(output[-8000:] if result.returncode else "\n".join(output.splitlines()[-6:]))
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
