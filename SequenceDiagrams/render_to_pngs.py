from cairosvg import svg2png
from pathlib import Path

for subpath in Path(__file__).parent.iterdir():
    if ".svg" in subpath.parts[-1]:
        with open(subpath, "r") as f:
            svg_code = f.read()
            svg2png(bytestring=svg_code,write_to=str(subpath)+".png")