"""Verify silent ST7305 initialization and explicit repeatable S2 artwork."""
from pathlib import Path
import argparse
import re
import subprocess
import sys
from PIL import Image, ImageChops

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('compiler', nargs='?', help='Host GCC-compatible compiler')
parser.add_argument('--vcvars', type=Path, help='MSVC vcvars64.bat')
options = parser.parse_args()
if not options.compiler and not options.vcvars:
    parser.error('provide a host compiler or --vcvars')
out = root / 'verification'
out.mkdir(exist_ok=True)
cad = (root / 'u8g2/u8x8_cad.c').read_text(encoding='utf-8')

def extract(name):
    match = re.search(r'^(?:uint8_t|void) ' + name + r'\([^;\n]*\)\n\{', cad, re.M)
    assert match, name
    start = match.start()
    cursor = cad.index('{', start)
    depth = 1
    end = cursor + 1
    while depth:
        depth += (cad[end] == '{') - (cad[end] == '}')
        end += 1
    return cad[start:end]

wrappers = '\n'.join(extract(n) for n in (
    'u8x8_cad_SendCmd', 'u8x8_cad_SendArg', 'u8x8_cad_SendData',
    'u8x8_cad_StartTransfer', 'u8x8_cad_EndTransfer', 'u8x8_cad_SendSequence'))
harness = r'''
#include "u8x8.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t ram[8064];
static unsigned written, on_count, delay_total, transfer;
static unsigned expected_bytes = 8064;
static uint8_t command, args[2];
static unsigned arg_count;
static uint8_t record(u8x8_t *x, uint8_t msg, uint8_t arg, void *ptr) {
    (void)x;
    if (msg == U8X8_MSG_CAD_START_TRANSFER) { assert(!transfer); transfer=1; }
    else if (msg == U8X8_MSG_CAD_END_TRANSFER) { assert(transfer); transfer=0; }
    else if (msg == U8X8_MSG_CAD_SEND_CMD) {
        assert(transfer);
        command=arg; arg_count=0;
        if (arg == 0x2c) written=0;
        if (arg == 0x29) { assert(written==expected_bytes); ++on_count; }
    } else if (msg == U8X8_MSG_CAD_SEND_ARG) {
        if (arg_count<2) args[arg_count]=arg;
        ++arg_count;
        if (arg_count==2 && command==0x2a) assert(args[0]==0x17 && args[1]==0x24);
        if (arg_count==2 && command==0x2b) assert(args[0]==0 && args[1]==0xbf);
    } else if (msg == U8X8_MSG_CAD_SEND_DATA) {
        assert(transfer && command==0x2c);
        assert(arg==168 && written+arg<=sizeof(ram));
        memcpy(ram+written, ptr, arg); written+=arg;
    }
    return 1;
}
void u8x8_d_helper_display_init(u8x8_t *x) { (void)x; }
void u8x8_d_helper_display_setup_memory(u8x8_t *x, const u8x8_display_info_t *i) {
    x->display_info=i;
}
void u8x8_gpio_call(u8x8_t *x, uint8_t msg, uint8_t arg) {
    (void)x;
    if (msg==U8X8_MSG_DELAY_MILLI) delay_total+=arg;
}
''' + wrappers + r'''
int main(void) {
    u8x8_t x;
    memset(&x, 0, sizeof(x));
    x.cad_cb=record;
    memset(ram, 0x55, sizeof(ram));
    assert(u8x8_d_st7305_168x384_yuying(&x,U8X8_MSG_DISPLAY_INIT,0,NULL));
    assert(written==0 && on_count==0 && delay_total==255 && !transfer);
    u8x8_st7305_yuying_send_boot(&x);
    assert(written==8064 && on_count==0 && !transfer);
    assert(u8x8_d_st7305_168x384_yuying(&x,U8X8_MSG_DISPLAY_SET_POWER_SAVE,0,NULL));
    assert(on_count==1);
    FILE *f=NULL;
    assert(fopen_s(&f,"verification/boot_native.bin","wb")==0 && f);
    assert(fwrite(ram,1,sizeof(ram),f)==sizeof(ram));
    assert(fclose(f)==0);
    /* Emulate a new home/menu image in controller RAM, then replay deferred
       reinitialization. No startup bytes may overwrite that image. */
    expected_bytes=0;
    for (unsigned iteration=1; iteration<4; ++iteration) {
        written=on_count=delay_total=transfer=0;
        memset(ram, 0x33*iteration, sizeof(ram));
        assert(u8x8_d_st7305_168x384_yuying(&x,U8X8_MSG_DISPLAY_INIT,0,NULL));
        assert(written==0 && on_count==0 && delay_total==255 && !transfer);
        for (unsigned i=0; i<sizeof(ram); ++i) assert(ram[i]==0x33*iteration);
        /* A later S2 request must still send the complete image. */
        u8x8_st7305_yuying_send_boot(&x);
        assert(written==8064 && on_count==0 && !transfer);
    }
    return 0;
}
'''
(out / 'boot_preview.c').write_text(harness, encoding='ascii')
if options.vcvars:
    quote = lambda p: '"' + str(p) + '"'
    rsp = ['/nologo', '/utf-8', '/O2', '/I'+quote(root/'u8g2/include'),
           '/I'+quote(root/'APP/include'), quote(out/'boot_preview.c'),
           quote(root/'u8g2/u8x8_d_st7305.c'), '/Fe:boot_preview.exe']
    (out/'preview.rsp').write_text(' '.join(rsp), encoding='utf-16')
    (out/'preview_build.cmd').write_text(
        '@echo off\ncall '+quote(options.vcvars)+' >nul\n'
        'if errorlevel 1 exit /b 1\ncl @preview.rsp\n', encoding='utf-8')
    subprocess.run(['cmd.exe','/d','/c',str(out/'preview_build.cmd')],cwd=out,check=True)
else:
    subprocess.run([options.compiler, '-O2', '-I'+str(root/'u8g2/include'),
                    '-I'+str(root/'APP/include'), str(out/'boot_preview.c'),
                    str(root/'u8g2/u8x8_d_st7305.c'), '-o', str(out/'boot_preview.exe')],
                   cwd=root, check=True)
subprocess.run([str(out/'boot_preview.exe')], cwd=root, check=True)
# A fresh executable process starts BSS at zero, like a complete MCU reset.
subprocess.run([str(out/'boot_preview.exe')], cwd=root, check=True)
wire = (out/'boot_native.bin').read_bytes()
actual = Image.new('L', (384,168))
for physical_y in range(384):
    for physical_x in range(168):
        packed=wire[(physical_y//2)*42 + physical_x//4]
        shift=2*(3-physical_x%4) + (physical_y%2==0)
        black=(packed>>shift)&1
        actual.putpixel((383-physical_y, physical_x),0 if black else 255)
source=Image.open(root/'resources/boot_reference.png').convert('L')
scale=min(384/source.width,168/source.height)
size=(round(source.width*scale),round(source.height*scale))
art=source.resize(size,Image.Resampling.LANCZOS).point(lambda p:255 if p>=128 else 0)
expected=Image.new('L',(384,168),0)
expected.paste(art,((384-size[0])//2,(168-size[1])//2))
assert ImageChops.difference(actual,expected).getbbox() is None
actual.save(out/'boot_preview.png')
actual.resize((768,336),Image.Resampling.NEAREST).save(out/'boot_preview_2x.png')
print('PASS: initialization never writes artwork or turns the display on.')
print('PASS: explicit S2 artwork writes 8064 bytes before display-on.')
print('PASS: repeated initialization preserves RAM; explicit artwork remains repeatable.')
print('PASS: existing 255 ms controller delay unchanged; no new delay.')
print('PASS: decoded controller RAM equals expected 384x168 artwork pixel-for-pixel.')
