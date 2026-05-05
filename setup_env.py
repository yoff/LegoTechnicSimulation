#!/usr/bin/env python3
"""Setup helper: download an LDraw parts library and/or a Blender build.

Usage examples
--------------
Download only the LDraw library::

    python setup_env.py --ldraw-dir /opt/ldraw

Download only Blender (extracted into a directory)::

    python setup_env.py --blender-dir ~/apps/blender

Download both::

    python setup_env.py --ldraw-dir /opt/ldraw --blender-dir ~/apps/blender

Specify a different Blender version (default: 4.1.0)::

    python setup_env.py --blender-dir ~/apps/blender --blender-version 4.2.0

All operations use only the Python standard library.
"""

from __future__ import annotations

import argparse
import platform
import shutil
import sys
import tarfile
import tempfile
import urllib.request
import zipfile
from pathlib import Path

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

LDRAW_LIBRARY_URL = (
    "https://library.ldraw.org/library/updates/complete.zip"
)

# Blender download base URL template.
# {major_minor} e.g. "4.1", {version} e.g. "4.1.0", {filename} as detected.
BLENDER_RELEASE_BASE = "https://download.blender.org/release/Blender{major_minor}/"

DEFAULT_BLENDER_VERSION = "4.1.0"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _download(url: str, dest: Path, label: str) -> None:
    """Download *url* to *dest*, printing a simple progress indicator."""
    print(f"Downloading {label} …")
    print(f"  URL: {url}")

    def _reporthook(block_num: int, block_size: int, total_size: int) -> None:
        if total_size > 0:
            downloaded = min(block_num * block_size, total_size)
            pct = downloaded * 100 // total_size
            print(f"\r  {pct:3d}%", end="", flush=True)

    urllib.request.urlretrieve(url, dest, reporthook=_reporthook)
    print()  # newline after progress


def _blender_filename(version: str) -> str:
    """Return the expected Blender archive filename for the current platform."""
    system = platform.system().lower()
    machine = platform.machine().lower()

    if system == "linux":
        arch = "x64" if machine in ("x86_64", "amd64") else machine
        return f"blender-{version}-linux-{arch}.tar.xz"
    elif system == "darwin":
        arch = "arm64" if machine == "arm64" else "x64"
        return f"blender-{version}-macos-{arch}.dmg"
    elif system == "windows":
        arch = "x64" if machine in ("amd64", "x86_64") else machine
        return f"blender-{version}-windows-{arch}.zip"
    else:
        raise RuntimeError(
            f"Unsupported platform: {system!r}.  "
            "Download Blender manually from https://www.blender.org/download/"
        )


def _blender_url(version: str) -> tuple[str, str]:
    """Return (url, filename) for the Blender archive matching this platform."""
    filename = _blender_filename(version)
    major_minor = ".".join(version.split(".")[:2])
    url = BLENDER_RELEASE_BASE.format(major_minor=major_minor) + filename
    return url, filename


# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------


def download_ldraw(dest_dir: Path) -> None:
    """Download and extract the complete LDraw parts library into *dest_dir*."""
    dest_dir = dest_dir.expanduser().resolve()
    dest_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "complete.zip"
        _download(LDRAW_LIBRARY_URL, archive, "LDraw complete library")

        print("Extracting LDraw library …")
        with zipfile.ZipFile(archive) as zf:
            zf.extractall(dest_dir)

    # The zip typically extracts into an "ldraw/" subdirectory.
    ldraw_sub = dest_dir / "ldraw"
    if ldraw_sub.is_dir():
        print(f"LDraw library available at: {ldraw_sub}")
    else:
        print(f"LDraw library extracted to:  {dest_dir}")


def download_blender(dest_dir: Path, version: str = DEFAULT_BLENDER_VERSION) -> None:
    """Download and extract Blender *version* into *dest_dir*.

    The archive is extracted directly into *dest_dir*, giving you a
    platform-specific sub-directory like ``blender-4.1.0-linux-x64/``.

    On macOS a ``.dmg`` is downloaded; extraction from DMG requires
    ``hdiutil`` (available on macOS) or manual mounting.
    """
    dest_dir = dest_dir.expanduser().resolve()
    dest_dir.mkdir(parents=True, exist_ok=True)

    url, filename = _blender_url(version)

    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / filename
        _download(url, archive, f"Blender {version}")

        suffix = "".join(Path(filename).suffixes)
        print("Extracting Blender …")

        if suffix in (".tar.xz", ".tar.gz", ".tar.bz2"):
            with tarfile.open(archive) as tf:
                members = tf.getmembers()
                top_dir = members[0].name.split("/")[0] if members else ""
                tf.extractall(dest_dir)
            blender_path = dest_dir / top_dir if top_dir else dest_dir
            print(f"Blender extracted to: {blender_path}")
        elif suffix == ".zip":
            with zipfile.ZipFile(archive) as zf:
                zf.extractall(dest_dir)
            print(f"Blender extracted to: {dest_dir}")
        elif filename.endswith(".dmg"):
            # Copy the DMG and instruct the user — we cannot mount without hdiutil.
            shutil.copy(archive, dest_dir / filename)
            print(
                f"\nBlender DMG saved to: {dest_dir / filename}\n"
                "To install on macOS, open the DMG and drag Blender to /Applications:\n"
                f"  open {dest_dir / filename}"
            )
            return
        else:
            raise RuntimeError(f"Unrecognised archive format: {filename}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="setup_env.py",
        description=(
            "Download an LDraw parts library and/or Blender for "
            "use with lego-technic-sim.  At least one of --ldraw-dir or "
            "--blender-dir must be provided."
        ),
    )
    p.add_argument(
        "--ldraw-dir",
        type=Path,
        metavar="PATH",
        help="Directory into which the LDraw complete library will be downloaded and extracted.",
    )
    p.add_argument(
        "--blender-dir",
        type=Path,
        metavar="PATH",
        help="Directory into which the Blender archive will be downloaded and extracted.",
    )
    p.add_argument(
        "--blender-version",
        default=DEFAULT_BLENDER_VERSION,
        metavar="VERSION",
        help=f"Blender version to download (default: {DEFAULT_BLENDER_VERSION}).",
    )
    return p


def main(argv: list[str] | None = None) -> None:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if not args.ldraw_dir and not args.blender_dir:
        parser.error("Provide at least one of --ldraw-dir or --blender-dir.")

    if args.ldraw_dir:
        download_ldraw(args.ldraw_dir)

    if args.blender_dir:
        download_blender(args.blender_dir, version=args.blender_version)


if __name__ == "__main__":
    main()
