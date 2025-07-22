import re
from pathlib import Path


ROOT = Path(__file__).parent.parent


if __name__ == "__main__":
    # Get the coverage percentage from the coverage file
    with open(f"{ROOT}/coverage.xml", "r") as f:
        coverage = f.read().split("line-rate=")[1].split('"')[1]
    coverage = float(coverage) * 100

    # Update coverage.svg with the new coverage percentage
    with open(f"{ROOT}/coverage.svg", "r") as f:
        svg = f.read()

    # Replace coverage percentage in aria-label with two decimals
    svg = re.sub(r'aria-label=".*%"', f'aria-label="{coverage:.2f}%"', svg)

    # Replace coverage in title
    svg = re.sub(r"<title>.*</title>", f"<title>{coverage:.2f}%</title>", svg)

    # Replace coverage in text
    svg = re.sub(r">.*%</text>", f">{coverage:.2f}%</text>", svg)

    # Calculate red and blue rgb values based on coverage (till 50% red)
    red = int(255 * (1 - max(coverage - 50, 0) / 50))
    green = int(255 * max(coverage - 50, 0) / 50)

    # Get rgb string
    rgb_string = "rgb({},{},{})".format(red, green, 0)

    # Replace rgb in fill
    svg = re.sub(r'fill="rgb\(.*\)"', f'fill="{rgb_string}"', svg)

    # Save the new svg
    with open(f"{ROOT}/coverage.svg", "w") as f:
        f.write(svg)
