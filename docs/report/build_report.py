"""Build the team report PDF from REPORT.md (ReportLab, runs in the dev container).

    docker/hub.sh report        ->  docs/report/Smart_Mobility_Hub_Report.pdf

Supports the Markdown subset used in REPORT.md: '#' title, '##' sections,
paragraphs, '- ' bullets, pipe tables, **bold** and `code`, plus three
directives: '!meta text' (title-page line), '!note text' (boxed note) and
'!img path | caption | width_cm' (figure, path relative to this file).
"""
import os
import re
import sys

import matplotlib
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (Image, KeepTogether, ListFlowable, ListItem, PageBreak,
                                Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle)

HERE = os.path.dirname(os.path.abspath(__file__))
FONT_DIR = os.path.join(matplotlib.get_data_path(), 'fonts', 'ttf')
for name, file in (('DV', 'DejaVuSans.ttf'), ('DV-B', 'DejaVuSans-Bold.ttf'),
                   ('DV-I', 'DejaVuSans-Oblique.ttf'), ('DV-BI', 'DejaVuSans-BoldOblique.ttf'),
                   ('DV-M', 'DejaVuSansMono.ttf')):
    pdfmetrics.registerFont(TTFont(name, os.path.join(FONT_DIR, file)))
pdfmetrics.registerFontFamily('DV', normal='DV', bold='DV-B', italic='DV-I', boldItalic='DV-BI')

BASE = getSampleStyleSheet()
BODY = ParagraphStyle('body', parent=BASE['BodyText'], fontName='DV', fontSize=9.5,
                      leading=13, spaceAfter=5)
H1 = ParagraphStyle('h1', parent=BODY, fontName='DV-B', fontSize=20, leading=25,
                    alignment=TA_CENTER, spaceAfter=14)
H2 = ParagraphStyle('h2', parent=BODY, fontName='DV-B', fontSize=13, leading=17,
                    spaceBefore=10, spaceAfter=6, textColor=colors.HexColor('#1f3b5a'),
                    keepWithNext=1)
META = ParagraphStyle('meta', parent=BODY, alignment=TA_CENTER, fontSize=10.5, leading=15)
CELL = ParagraphStyle('cell', parent=BODY, fontSize=8, leading=10, spaceAfter=0)
CAPTION = ParagraphStyle('cap', parent=BODY, fontSize=8, leading=10, alignment=TA_CENTER,
                         textColor=colors.HexColor('#444444'))
NOTE = ParagraphStyle('note', parent=BODY, fontSize=9, leading=12, backColor='#fff4d6',
                      borderColor='#e0b84a', borderWidth=0.8, borderPadding=6,
                      spaceBefore=6, spaceAfter=12)


def inline(text):
    """Escape XML, then turn **bold** and `code` into ReportLab markup."""
    text = text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
    text = re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', text)
    text = re.sub(r'(?<![\w*])\*(?!\s)(.+?)(?<!\s)\*(?![\w*])', r'<i>\1</i>', text)
    text = re.sub(r'`(.+?)`', r'<font name="DV-M" size="8.5">\1</font>', text)
    return text


def table(rows, width):
    cells = [[Paragraph(('<b>%s</b>' if r == 0 else '%s') % inline(c), CELL) for c in row]
             for r, row in enumerate(rows)]
    ncol = len(rows[0])
    # Each column gets at least the width of its longest word (bold, as in the
    # header) so no word is split; the rest is shared by text length.
    floor = [max(pdfmetrics.stringWidth(w, 'DV-B', 8) for row in rows
                 for w in (row[i].split() or [''])) + 8 for i in range(ncol)]
    lengths = [min(max(len(row[i]) for row in rows), 60) for i in range(ncol)]
    spare = width - sum(floor)
    if spare <= 0:
        widths = [width * f / sum(floor) for f in floor]
    else:
        widths = [f + spare * n / sum(lengths) for f, n in zip(floor, lengths)]
    t = Table(cells, colWidths=widths, repeatRows=1)
    t.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#dfe8f2')),
        ('GRID', (0, 0), (-1, -1), 0.4, colors.HexColor('#9aa8b8')),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('LEFTPADDING', (0, 0), (-1, -1), 3), ('RIGHTPADDING', (0, 0), (-1, -1), 3)]))
    return t


def figure(spec):
    path, caption, width_cm = [s.strip() for s in spec.split('|')]
    full = os.path.normpath(os.path.join(HERE, path))
    img = Image(full)
    w = float(width_cm) * cm
    img.drawHeight = img.imageHeight * w / img.imageWidth
    img.drawWidth = w
    return KeepTogether([Spacer(1, 4), img, Paragraph(inline(caption), CAPTION),
                         Spacer(1, 6)])


def build(md_path, pdf_path):
    lines = open(md_path, encoding='utf-8').read().splitlines()
    doc = SimpleDocTemplate(pdf_path, pagesize=A4, leftMargin=2 * cm, rightMargin=2 * cm,
                            topMargin=1.8 * cm, bottomMargin=1.8 * cm,
                            title='Smart Mobility Hub — Engineering Report')
    width = A4[0] - 4 * cm
    story, para, bullets, rows = [], [], [], []
    first_section = True

    def flush():
        if para:
            story.append(Paragraph(inline(' '.join(para)), BODY))
            para.clear()
        if bullets:
            story.append(ListFlowable([ListItem(Paragraph(inline(b), BODY), leftIndent=12)
                                       for b in bullets], bulletType='bullet', start='•'))
            bullets.clear()
        if rows:
            story.append(table([r for r in rows if not set(''.join(r)) <= set('-: ')],
                               width))
            story.append(Spacer(1, 6))
            rows.clear()

    for line in lines:
        s = line.strip()
        if s.startswith('|'):
            if para or bullets:
                flush()
            rows.append([c.strip() for c in s.strip('|').split('|')])
            continue
        if rows:
            flush()
        if not s:
            flush()
        elif s.startswith('# '):
            story += [Spacer(1, 4 * cm), Paragraph(inline(s[2:]), H1)]
        elif s.startswith('!meta '):
            story.append(Paragraph(inline(s[6:]), META))
        elif s.startswith('!note '):
            flush()
            story += [Spacer(1, 1 * cm), Paragraph(inline(s[6:]), NOTE)]
        elif s.startswith('## '):
            flush()
            if first_section:
                story.append(PageBreak())
                first_section = False
            story.append(Paragraph(inline(s[3:]), H2))
        elif s.startswith('!img '):
            flush()
            story.append(figure(s[5:]))
        elif s.startswith('- '):
            if para:
                flush()
            bullets.append(s[2:])
        else:
            para.append(s)
    flush()

    def footer(canvas, doc_):
        canvas.saveState()
        canvas.setFont('DV', 7.5)
        canvas.setFillColor(colors.grey)
        canvas.drawString(2 * cm, 1 * cm, 'Smart Mobility Hub — engineering report (DRAFT)')
        canvas.drawRightString(A4[0] - 2 * cm, 1 * cm, 'page %d' % doc_.page)
        canvas.restoreState()
    doc.build(story, onFirstPage=footer, onLaterPages=footer)


if __name__ == '__main__':
    md = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'REPORT.md')
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        HERE, 'Smart_Mobility_Hub_Report.pdf')
    build(md, out)
    print('wrote', out)
