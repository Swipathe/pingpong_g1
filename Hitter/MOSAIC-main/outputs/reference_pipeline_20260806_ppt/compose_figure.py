"""Compose the real frame/render panels into a PNG and self-contained SVG."""

import base64
import html
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


OUT = Path(__file__).resolve().parent
RENDER = OUT / "renders"
WIDTH, HEIGHT = 4200, 1260
REGULAR = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def main():
    canvas = Image.new("RGB", (WIDTH, HEIGHT), "white")
    draw = ImageDraw.Draw(canvas)
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}">',
           '<title>Reference Motion Processing</title>',
           '<desc>Matched forehand F7 and backhand B7 video frames, actual GVHMR SMPL-X reconstruction overlays, and G1 robot poses from the corresponding NPZ frames.</desc>',
           f'<rect width="{WIDTH}" height="{HEIGHT}" fill="white"/>']

    def text(x, y, value, size, fill="#162c49", bold=False, anchor="mm"):
        font = ImageFont.truetype(BOLD if bold else REGULAR, size)
        draw.text((x, y), value, font=font, fill=fill, anchor=anchor)
        svg_anchor = "middle" if anchor.startswith("m") else "start"
        svg.append(f'<text x="{x}" y="{y}" fill="{fill}" font-family="DejaVu Sans, sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" text-anchor="{svg_anchor}" dominant-baseline="central">{html.escape(value)}</text>')

    def rect(box, fill, outline=None, radius=0, line_width=1):
        draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=line_width)
        x, y, right, bottom = box
        svg.append(f'<rect x="{x}" y="{y}" width="{right-x}" height="{bottom-y}" rx="{radius}" fill="{fill}" stroke="{outline or "none"}" stroke-width="{line_width}"/>')

    def picture(path, x, y, width, height):
        im = Image.open(path).convert("RGB")
        assert im.size == (1080, 1600), (path, im.size)
        canvas.paste(im.resize((width, height), Image.Resampling.LANCZOS), (x, y))
        data = base64.b64encode(path.read_bytes()).decode()
        svg.append(f'<image x="{x}" y="{y}" width="{width}" height="{height}" preserveAspectRatio="none" xlink:href="data:image/png;base64,{data}"/>')

    def arrow(left, right, y, label, tool):
        center = (left + right) / 2
        text(center, y - 85, label, 35, bold=True)
        draw.line((left, y, right - 29, y), fill="#3569ae", width=8)
        draw.polygon([(right, y), (right-33, y-21), (right-33, y+21)], fill="#3569ae")
        svg.append(f'<path d="M {left} {y} H {right-28}" fill="none" stroke="#3569ae" stroke-width="8"/>')
        svg.append(f'<path d="M {right} {y} L {right-33} {y-21} L {right-33} {y+21} Z" fill="#3569ae"/>')
        text(center, y + 64, tool, 31, fill="#5a7192")

    text(90, 88, "Reference Motion Processing", 64, bold=True, anchor="lm")
    text(90, 151, "Human demonstrations to robot reference motion", 32, fill="#61728a", anchor="lm")
    stages = [(90, "Human Video", "video"), (1550, "Reconstructed Human", "reconstruction"), (3010, "Retargeted Robot", "robot")]
    for x, title, suffix in stages:
        rect((x, 220, x+1100, 1116), "#f6f9fd", "#cfdded", radius=24, line_width=3)
        text(x+550, 263, title, 39, bold=True)
        for index, (code, label) in enumerate([("F7", "Forehand"), ("B7", "Backhand")]):
            left = x + 30 + index * 540
            picture(RENDER / f"{code}_{suffix}.png", left, 310, 510, 756)
            text(left+255, 1090, label, 29, fill="#476381")
    arrow(1220, 1520, 665, "Reconstruction", "GVHMR")
    arrow(2680, 2980, 665, "Retargeting", "GMR")
    # Explicitly separate the pose-mapping stage from training-data packaging.
    text(3560, 1174, "Post-processing & alignment → Reference NPZ", 30, fill="#476381")
    svg.append("</svg>")
    canvas.save(OUT / "reference_pipeline.png", dpi=(300, 300))
    canvas.save(OUT / "reference_pipeline.pdf", "PDF", resolution=300)
    (OUT / "reference_pipeline.svg").write_text("\n".join(svg), encoding="utf-8")
    canvas.resize((2100, 630), Image.Resampling.LANCZOS).save(OUT / "reference_pipeline_preview.png")
    print("Saved PNG (4200 x 1260), PDF, SVG, and preview.")


if __name__ == "__main__":
    main()
