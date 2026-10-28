"""Check saved AC5/AC6 builds; no probe, serial port, or firmware download."""
from pathlib import Path
import collections
import hashlib
import json
import re
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent


def hex_image(path):
    image, base, ended = {}, 0, False
    for line in path.read_text(encoding='ascii').splitlines():
        assert line.startswith(':') and not ended
        record = bytes.fromhex(line[1:])
        assert len(record) == record[0] + 5 and sum(record) % 256 == 0
        size, address, kind = record[0], int.from_bytes(record[1:3], 'big'), record[3]
        data = record[4:-1]
        if kind == 0:
            for offset, byte in enumerate(data):
                at = base + address + offset
                assert 0x08000000 <= at < 0x08010000 and at not in image
                image[at] = byte
        elif kind == 1:
            assert size == 0 and address == 0
            ended = True
        elif kind in (2, 4):
            assert size == 2 and address == 0
            base = int.from_bytes(data, 'big') << (4 if kind == 2 else 16)
        else:
            assert kind in (3, 5) and size == 4
    assert ended and image
    return image


def main():
    fromelf = Path(sys.argv[1])
    report = HERE / 'results.json'
    report.write_text(json.dumps({'verified': False, 'flashed': False}))
    project = ET.parse(ROOT / 'MDK-ARM/MX_02.uvprojx')
    targets = project.findall('./Targets/Target')
    assert [t.findtext('TargetName') for t in targets] == ['MX_02', 'MX_02_AC6']
    assert ET.tostring(targets[0].find('Groups')) == ET.tostring(targets[1].find('Groups'))
    results = {}
    for compiler, directory in [('ac5', 'MX_02'), ('ac6', 'MX_02_AC6')]:
        output = ROOT / 'MDK-ARM' / directory
        log = (HERE / f'{compiler}-build.log').read_text(errors='replace')
        summary = re.search(r'0 Error\(s\), (\d+) Warning\(s\)', log)
        assert summary, compiler
        axf = output / 'MX_02.axf'
        exported = HERE / f'{compiler}-from-axf.hex'
        subprocess.run([str(fromelf), '--i32combined', '--output', str(exported), str(axf)], check=True)
        image = hex_image(output / 'MX_02.hex')
        assert image == hex_image(exported)
        def words(address, size):
            return struct.unpack('<' + 'I' * (size // 4), bytes(image[address+i] for i in range(size)))
        sp, reset = words(0x08000000, 8)
        assert 0x20000000 < sp <= 0x20001fc0 and sp % 8 == 0
        assert reset & 1 and (reset & ~1) in image
        mapping = (output / 'MX_02.map').read_text(errors='replace')
        assert re.search(r'Execution Region UART_NOINIT .*?Size: 0x00000040,.*?UNINIT', mapping)
        assert re.search(r's_uart1_retained\s+0x20001fc0\s+Data\s+64\s+usart.o', mapping)
        assert re.search(r'fputc\s+0x[0-9a-f]+\s+Thumb Code\s+\d+\s+usart.o', mapping)
        assert not re.search(r'^\s*__io_putchar\s+0x', mapping, re.M)
        def symbol(name):
            return int(re.search(r'^\s*' + re.escape(name) + r'\s+(0x[0-9a-f]+)', mapping, re.M)[1], 16)
        start, end = symbol('Region$$Table$$Base'), symbol('Region$$Table$$Limit')
        assert end > start and (end-start) % 16 == 0
        entries = [words(address, 16) for address in range(start, end, 16)]
        for load, destination, size, handler in entries:
            assert destination + size <= 0x20001fc0 or destination >= 0x20002000
        disassembly = HERE / f'{compiler}-disassembly.txt'
        subprocess.run([str(fromelf), '--text', '-c', '--output', str(disassembly), str(axf)], check=True)
        assert not re.search(r'\bBKPT\b', disassembly.read_text(errors='replace'), re.I)
        results[compiler] = dict(warnings=int(summary[1]), stack_pointer=hex(sp), reset_vector=hex(reset),
            flash_bytes=len(image), retained_address='0x20001fc0', retained_bytes=64,
            scatter_table=entries, hex_axf_equal=True, no_bkpt=True,
            hex_sha256=hashlib.sha256((output/'MX_02.hex').read_bytes()).hexdigest(),
            warning_classes=dict(collections.Counter(re.findall(r'\[(-W[^\]]+)\]', log))))
    results.update(verified=True, flashed=False, hardware_verified=False)
    report.write_text(json.dumps(results, indent=2), encoding='utf-8')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
