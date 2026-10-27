"""Render repository previews/PDFs from the ordered Gerbers and native schematic SVG.

Read-only with respect to the EasyEDA project and manufacturing inputs. Generated
drawings are documentation, not a replacement fabrication package.
"""
from pathlib import Path
import json
import re
import zipfile

import pymupdf as fitz
from gerbonara.rs274x import GerberFile
from gerbonara.utils import Tag, MM

ROOT = Path(__file__).resolve().parents[1]
PCB = ROOT / 'hardware/pcb/6dof2_r14'
EXPORT = PCB / 'exports/2026-10-26'
DRAWINGS = PCB / 'drawings'
IMAGES = PCB / 'images'


def svg_document(objects, background='#164d3c'):
    return str(Tag('svg', [
        Tag('rect', x='-3', y='-3', width='306', height='107.6', fill=background),
        Tag('g', objects, transform='translate(0 101.6) scale(1 -1)'),
    ], xmlns='http://www.w3.org/2000/svg', width='306mm', height='107.6mm',
        viewBox='-3 -3 306 107.6'))


def drill_objects(data):
    """Read this export's explicit metric circles and inline G85 slots.

    Gerbonara's Excellon reader rejects EasyEDA's XYG85XY form. Interpret that
    form directly for the drawing; never alter the manufacturing drill input.
    """
    assert 'METRIC,LZ,000.000000' in data
    tools, selected, objects = {}, None, []
    number = r'(-?\d+(?:\.\d+)?)'
    for line in data.splitlines():
        line = line.strip()
        if not line or line.startswith(';') or line in ['M48', 'METRIC,LZ,000.000000', '%', 'G05', 'G90', 'M30']:
            continue
        if match := re.fullmatch(r'T(\d+)C'+number, line):
            tools[match[1]] = float(match[2])
        elif match := re.fullmatch(r'T(\d+)', line):
            selected = tools[match[1]]
        elif match := re.fullmatch('X'+number+'Y'+number+'G85X'+number+'Y'+number, line):
            assert selected is not None
            x1, y1, x2, y2 = map(float, match.groups())
            objects.append(Tag('line', x1=x1, y1=y1, x2=x2, y2=y2, stroke='#08291f',
                               stroke_width=selected, stroke_linecap='round'))
        elif match := re.fullmatch('X'+number+'Y'+number, line):
            assert selected is not None
            objects.append(Tag('circle', cx=float(match[1]), cy=float(match[2]), r=selected/2, fill='#08291f'))
        else:
            raise ValueError('Unsupported drill drawing statement: '+line)
    return objects


def main():
    DRAWINGS.mkdir(exist_ok=True)
    IMAGES.mkdir(exist_ok=True)
    pdf = fitz.open()
    with zipfile.ZipFile(EXPORT / 'R14-Gerber-FS36.zip') as archive:
        layers = {}
        for extension in ['.GTL', '.G1', '.G2', '.GBL', '.GTO', '.GKO']:
            name, = [n for n in archive.namelist() if n.upper().endswith(extension)]
            layers[extension] = GerberFile.from_string(archive.read(name).decode('ascii'), filename=name)
        drills = []
        for name in archive.namelist():
            # The separate VIA drill file duplicates PTH operations.
            if name.upper().endswith('.DRL') and 'VIA' not in name.upper():
                drills.extend(drill_objects(archive.read(name).decode('ascii')))
        assert len(drills) == 1086
        for extension, title, filename in [
                ('.GTL', 'Top copper and silkscreen', 'R14-PCB-top.svg'),
                ('.G1', 'Inner 1 copper', None),
                ('.G2', 'Inner 2 copper', None),
                ('.GBL', 'Bottom copper (viewed through top)', None)]:
            objects = list(layers[extension].svg_objects(svg_unit=MM, fg='#d5b778', bg='#164d3c'))
            if extension == '.GTL':
                objects.extend(layers['.GTO'].svg_objects(svg_unit=MM, fg='#f9faf8', bg='#164d3c'))
            objects.extend(drills)
            objects.extend(layers['.GKO'].svg_objects(svg_unit=MM, fg='#93c5b3', bg='#164d3c'))
            svg = svg_document(objects)
            if filename:
                (DRAWINGS / filename).write_text(svg, encoding='utf8')
            source = fitz.open(stream=svg.encode(), filetype='svg')
            vector = fitz.open('pdf', source.convert_to_pdf())
            page = pdf.new_page(width=1000, height=410)
            page.insert_text((28, 27), 'R14 ordered 2026-10-26 | ' + title, fontsize=14)
            page.show_pdf_page(fitz.Rect(15, 42, 985, 383), vector, 0)
            page.insert_text((28, 400), 'Derived from frozen production Gerbers. Board 300 x 101.6 mm. Gerber ZIP remains fabrication authority.', fontsize=9)
            if filename:
                source[0].get_pixmap(matrix=fitz.Matrix(3200/source[0].rect.width, 3200/source[0].rect.width), alpha=False).save(IMAGES / 'r14-pcb-top.png')
    pdf.save(DRAWINGS / 'R14-PCB.pdf', garbage=4, deflate=True)

    native = fitz.open(stream=(EXPORT / 'R14-schematic.svg').read_bytes(), filetype='svg')
    full = fitz.open('pdf', native.convert_to_pdf())
    full[0].get_pixmap(matrix=fitz.Matrix(2800/full[0].rect.width, 2800/full[0].rect.width), alpha=False).save(IMAGES / 'r14-schematic.png')
    # Add readable source-coordinate detail pages, retaining the complete sheet.
    viewbox = re.search(rb'viewBox="([^"]+)"', (EXPORT / 'R14-schematic.svg').read_bytes()).group(1).decode().split()
    min_x, min_y = map(float, viewbox[:2])
    detail_pdf = fitz.open()
    for title, bounds in [
            ('Field-voltage monitor: U18 / R80-R83', (5930, -4770, 6570, -4380)),
            ('Pulse permission and direct hardware clear: U11 / U41', (2000, -4420, 2590, -3990)),
            ('Input ideal-diode bypass: U42 / Q11', (14130, -1320, 14585, -920))]:
        x0, y0, x1, y1 = bounds
        clip = fitz.Rect(x0-min_x, y0-min_y, x1-min_x, y1-min_y) & full[0].rect
        page = detail_pdf.new_page(width=1000, height=740)
        page.insert_text((28, 30), title, fontsize=17)
        page.show_pdf_page(fitz.Rect(25, 48, 975, 705), full, 0, clip=clip)
        page.insert_text((28, 727), 'Unmodified native schematic region; complete sheet follows. Named nets continue elsewhere.', fontsize=10)
    montage = fitz.open()
    page = montage.new_page(width=1400, height=1050)
    page.show_pdf_page(fitz.Rect(0, 0, 1400, 525), detail_pdf, 0)
    page.show_pdf_page(fitz.Rect(0, 525, 700, 1050), detail_pdf, 1)
    page.show_pdf_page(fitz.Rect(700, 525, 1400, 1050), detail_pdf, 2)
    page.get_pixmap(matrix=fitz.Matrix(1.7, 1.7), alpha=False).save(IMAGES / 'r14-schematic-details.png')
    detail_pdf.insert_pdf(full)
    detail_pdf.save(DRAWINGS / 'R14-schematic.pdf', garbage=4, deflate=True)
    print(json.dumps({'pcbPdfPages': len(pdf), 'schematicPdfPages': len(detail_pdf),
                      'inputs': ['R14-Gerber-FS36.zip', 'R14-schematic.svg'],
                      'method': 'Gerbonara vector Gerber rendering; MuPDF native-SVG rendering. No CAD mutation.'}, indent=2))


if __name__ == '__main__':
    main()
