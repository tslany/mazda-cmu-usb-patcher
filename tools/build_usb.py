#!/usr/bin/env python3
"""Build the USB payload and the native update-scanner entry point."""

import gzip
import hashlib
import io
from pathlib import Path
import re
import sys
import tarfile
import zipfile


ROOT = Path(__file__).resolve().parents[1]
IDENTITY = b"mazda-cmu-usb-patcher-v1\n"

# FAT filenames cannot contain '/'. The update service runs with HOME=/root.
# Count matching drives so a second installer USB cannot select an arbitrary one.
LAUNCHER = (
    "$(s=${HOME%root};n=0;for d in sda1 sdb1 sdc1 sdd1;do "
    "p=${s}tmp${s}mnt${s}$d;[ -f $p${s}CMU-PATCHER.ID ]&&"
    "{ n=$((n+1));f=$p${s}install.sh;};done;[ $n = 1 ]&&sh $f).up"
)

# These are update-container metadata, not firmware update instructions.
# No executable update step or firmware image is included.
UPDATE_METADATA = {
    "main_instructions.ini": b"""[Settings]
PackageID = 1791936000
CompressionType = NONE
TotalStepsCount = 0

[Instructions]
Count = 0

[DataStorage]
Count = 5
UPType = "Reinstall"
SubUPType = "Mass"
ReTransmit = "1"
NewPackage = "1"
CMUBoardType = "3"
""",
    "versions.ini": b"""[Settings]
CheckCompatibility = 1,"Compat_Section"
Labels = 2,"JCI_SW_VER","DisplayVersion"

[Compat_Section]
Count = 1
1 = CheckLabel,"JCI_SW_VER",1,"[>='MAZ_CMU-150_74.00.324']"

[JCI_SW_VER]
LabelName = "JCI_SW_VER"
LoadLocation = Custom

[DisplayVersion]
LabelName = "DisplayVersion"
LoadLocation = UP
LabelValue = "74.00.324"
""",
}


def zip_bytes(files):
    output = io.BytesIO()
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in sorted(files.items()):
            entry = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            entry.compress_type = zipfile.ZIP_DEFLATED
            entry.create_system = 3
            entry.external_attr = 0o100644 << 16
            archive.writestr(entry, data)
    return output.getvalue()


def collect_files():
    files = {
        "install.sh": (ROOT / "installer/install.sh").read_bytes(),
        "apply-patch.awk": (ROOT / "installer/apply-patch.awk").read_bytes(),
        "CMU-PATCHER.ID": IDENTITY,
    }
    for folder in ("patches", "packages"):
        for path in sorted((ROOT / folder).rglob("*")):
            if path.is_symlink():
                raise ValueError(f"symlink in payload: {path}")
            if path.is_file():
                name = path.relative_to(ROOT).as_posix()
                if not re.fullmatch(r"[A-Za-z0-9_./+-]+", name):
                    raise ValueError(f"unsupported payload filename: {name}")
                files[name] = path.read_bytes()
    libraries = [name for name in files if name.endswith(".so")]
    expected = {f"libpatch-{role}.so" for role in (
        "aap_service", "blmjciaapa", "blmjcicarplay", "svcjcinavi")}
    if {Path(name).name for name in libraries} != expected or len(libraries) != 4:
        raise ValueError("expected exactly four projection libraries")
    for name in libraries:
        data = files[name]
        if data[:5] != b"\x7fELF\x01" or data[5] != 1 or data[18:20] != b"\x28\x00":
            raise ValueError(f"not a little-endian ARM ELF32 library: {name}")
    files["SHA256SUMS"] = "".join(
        f"{hashlib.sha256(data).hexdigest()}  {name}\n"
        for name, data in sorted(files.items())
    ).encode("ascii")
    for name in ("README.md", "LICENSE"):
        files[name] = (ROOT / name).read_bytes()
    if len(LAUNCHER.encode("ascii")) > 255 or any(c in LAUNCHER for c in '<>:"/\\|?*'):
        raise ValueError("update entry filename is not FAT-compatible")
    files["jci-autoupdate"] = b""
    files[LAUNCHER] = zip_bytes(UPDATE_METADATA)
    return files


def main():
    if len(sys.argv) != 2 or not re.fullmatch(r"[A-Za-z0-9._-]+", sys.argv[1]):
        sys.exit("usage: build_usb.py VERSION (letters, digits, dot, underscore, dash)")
    files = collect_files()
    destination = ROOT / "dist"
    destination.mkdir(exist_ok=True)
    archive = destination / f"usb_patcher_{sys.argv[1]}.tar.gz"
    temporary = archive.with_name(archive.name + ".tmp")
    output = io.BytesIO()
    with tarfile.open(fileobj=output, mode="w", format=tarfile.PAX_FORMAT) as bundle:
        for name, data in sorted(files.items()):
            entry = tarfile.TarInfo(name)
            entry.size = len(data)
            entry.mode = 0o644
            bundle.addfile(entry, io.BytesIO(data))
    temporary.write_bytes(gzip.compress(output.getvalue(), mtime=0))
    with tarfile.open(fileobj=io.BytesIO(gzip.decompress(temporary.read_bytes()))) as check:
        if check.getnames() != sorted(files):
            raise ValueError("USB archive file list verification failed")
        for name, data in files.items():
            member = check.extractfile(name)
            if member is None or member.read() != data:
                raise ValueError(f"USB archive verification failed: {name}")
    temporary.replace(archive)
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    archive.with_name(archive.name + ".sha256").write_text(f"{digest}  {archive.name}\n")
    print(f"created {archive}")


if __name__ == "__main__":
    main()
