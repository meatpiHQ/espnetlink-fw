# WiCAN Firmware Device Types & Updates

## Overview
WiCAN devices come in three variants, each requiring specific firmware files from GitHub releases. The integration supports both automatic updates to the latest version and installing specific versions.

## Device Types & Firmware Files

### 1. WiCAN-OBD (Standard)
- **Hardware Version**: `WiCAN-OBD` or similar (without "PRO" or "USB")
- **Firmware Pattern**: `wican-fw_obd_vXXX.bin`
- **Example**: `wican-fw_obd_v413.bin`
- **GitHub Release**: Standard releases (e.g., v4.13)
- **Detection**: release tag has no suffix letter; asset name contains "obd" but NOT "usb" and NOT "pro"

### 2. WiCAN-USB
- **Hardware Version**: `WiCAN-USB`
- **Firmware Pattern**: `wican-fw_usb_vXXXu.bin` (note: 'u' version suffix)
- **Example**: `wican-fw_usb_v413u.bin`
- **GitHub Release**: Standard releases (e.g., v4.13)
- **Detection**: release tag suffix `u` or a standard release carrying a USB asset; asset name contains "usb" but NOT "pro"

### 3. WiCAN-PRO
- **Hardware Version**: `WiCAN-PRO`
- **Firmware Pattern**: `wican-fw_obd_pro_vXXXp.bin` (note: 'p' version suffix)
- **Example**: `wican-fw_obd_pro_v445p.bin`
- **GitHub Release**: PRO-specific releases (e.g., v4.45p)
- **Detection**: release tag suffix `p` (or PRO in the name); asset name contains "pro"

## Release Selection (stream layer)

Before any asset is looked at, `github_releases.py` decides which
*releases* a device may be offered:

1. **Stream classification** — the version-tag suffix letter is
   authoritative: `v4.45p` → PRO, `v4.13u` → USB, `v4.46` → OBD (release
   name is the fallback; matching is case-insensitive, so `v4.45P` works).
   A bare substring match is deliberately avoided so a tag like
   `v4.50-patch1` is not misread as PRO.
2. **Asset eligibility** — PRO devices only match PRO releases. OBD and
   USB devices share the standard releases, and are only offered releases
   that actually carry their `.bin` asset — a combined release like
   `v4.13` (OBD + USB assets) serves both, while an OBD-only release is
   never offered to a USB device.

The GitHub release list is fetched once per hour and shared across all
configured devices. A manually added device that reveals its `hw_version`
on its first push is retargeted to the right stream without a reload.

## Asset Selection Logic

Within the chosen release, the firmware update implementation in [update.py](../../custom_components/wican/update.py) picks the binary:

```python
# Determine device type from hardware version
hw_version = self.config_entry.data.get("hw_version", "").lower()
is_pro = "pro" in hw_version
is_usb = "usb" in hw_version

# Search GitHub release assets for matching .bin file
for asset in assets:
    name = asset.get("name", "").lower()
    if not name.endswith(".bin"):
        continue
    
    has_pro = "pro" in name
    has_usb = "usb" in name and "pro" not in name
    has_obd = "obd" in name and "usb" not in name and "pro" not in name
    
    # Match asset to device type
    if is_pro and has_pro:
        return asset  # PRO device gets PRO firmware
    elif is_usb and has_usb:
        return asset  # USB device gets USB firmware
    elif not is_pro and not is_usb and has_obd:
        return asset  # Standard OBD device gets OBD firmware
```

## GitHub Release Structure

### Standard Release (v4.13)
Supports both WiCAN-OBD and WiCAN-USB:
```
Assets:
- wican-fw_obd_v413.bin      (1.57 MB) - for WiCAN-OBD
- wican-fw_obd_v413.zip      (934 KB)
- wican-fw_usb_v413u.bin     (1.57 MB) - for WiCAN-USB
- wican-fw_usb_v413u.zip     (947 KB)
```

### PRO Release (v4.45p)
Supports only WiCAN-PRO:
```
Assets:
- wican-fw_obd_pro_v445p.bin (3.19 MB) - for WiCAN-PRO
- wican-fw_obd_pro_v445p.zip (1.71 MB)
```

## Installing Firmware Updates

### Update to Latest Version (Default)

Use the Home Assistant UI or call the service without specifying a version:

```yaml
service: update.install
target:
  entity_id: update.wican_device_firmware
```

### Install Specific Version

You can install any available version by specifying it in the service call:

```yaml
service: update.install
target:
  entity_id: update.wican_device_firmware
data:
  version: "4.44"  # Install this specific version
```

**Important Notes:**
- Any natural form of the version is accepted — `"4.44"`, `"v4.44"`,
  `"4.44p"`, `"V4.44P"` all normalize to the same lookup
- The requested version must exist in the device's release stream
- The integration automatically finds the correct firmware file for your device type
- Only stable (non-prerelease) versions are available
- The integration will fetch the specified version from GitHub releases

### Version Comparison

The integration normalizes version strings for proper comparison:
- Device firmware reports: `4.46` (no suffix)
- GitHub releases include: `4.45p` (PRO), `4.13u` (USB), `4.13` (standard)
- Both are normalized to compare correctly: `4.46` vs `4.45`

This ensures accurate update detection regardless of version format differences.

## Testing

Unit tests verify:
- ✅ OBD devices select only OBD firmware ([test_firmware_asset_selection.py](tests/test_firmware_asset_selection.py))
- ✅ USB devices select only USB firmware
- ✅ PRO devices select only PRO firmware
- ✅ Version normalization removes suffixes ([test_version_normalization.py](tests/test_version_normalization.py))
- ✅ Specific version installation fetches correct release
- ✅ Latest version uses cached coordinator data
- ✅ No incorrect cross-matching between device types

All tests passing confirms the asset selection and version management logic correctly handles all three variants.
