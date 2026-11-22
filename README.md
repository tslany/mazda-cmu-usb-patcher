# Mazda CMU USB Patcher

USB installer for stock **74.00.324A** firmware; checked against EU and NA.

## Packages

- Android Auto and CarPlay cluster guidance, with native navigation speed limits.
- Built-in navigation unlock.

## Patches

| Patch | Description |
| --- | --- |
| `main-menu-loop` | Wraps the main menu at both ends. |
| `audio-source-order` | Reorders the audio-source menu. |
| `improved-list-loop` | Wraps lists at both ends. |
| `list-hold-delay` | Shortens the delay when holding the commander. |
| `list-performance` | Creates list-row styles only when needed. |
| `list-cleanup` | Cleans up unused list listeners. |
| `tabs-overflow-cache` | Caches tab-overflow layout calculations. |
| `settings-redraw` | Avoids duplicate Settings redraws. |
| `clock-redraw` | Avoids unnecessary clock redraws. |
| `gui-log-warn` | Reduces GUI logging to warnings and errors. |
| `debug-message-work` | Skips message formatting when the debug panel is off. |
| `native-log-warn` | Reduces native service logging. |
| `console-log-off` | Disables console logging. |
| `debug-callback-work` | Skips unused debug callbacks. |
| `websocket-order` | Preserves screen-transition message order and delivery. |
| `touchscreen-ui` | Allows touchscreen use while moving. |
| `message-footer` | Removes the Mazda quick-message footer. |
| `eco-flow-cleanup` | Stops unused fuel-flow animations and timers. |
| `eco-status-cleanup` | Stops unused fuel-status animations and timers. |
| `eco-effect-cleanup` | Stops unused fuel-economy graph timers. |
| `hud-menu` | Shows region-hidden color-HUD options. |
| `projection-navigation-capability` | Advertises navigation support to CarPlay. |
| `aa-touch` | Enables Android Auto touch input. |
| `aa-media-pause` | Enables Android Auto pause-on-mute. |
| `aa-navigation` | Enables Android Auto navigation data for the HUD. |
| `aa-video-density` | Corrects Android Auto scaling. |
| `projection-services` | Loads the CarPlay and Android Auto patches at startup. |
| `projection-cpu` | Assigns CarPlay and network services to CPU1. |
| `disable-streaming` | Disables Aha, Pandora and Stitcher startup. |
| `disable-sirius` | Disables Sirius startup outside North America. |
| `audio-log` | Reduces audio-service logging. |

## Build

Requires Python 3 and make. Projection libraries are included as package payloads.
To rebuild them: `make -C src M3TOOLCHAIN=/path/to/m3-toolchain`.

```sh
make
```

## Use

1. Remove older mods first, or reinstall stock firmware and clean up any leftover mod directories in `/data_persist`. Extract the archive from `dist/` onto an empty FAT32 USB.
2. Insert the USB and choose **Install**, then **Reboot**. Unplug it when the screen goes dark.
3. To undo, use the same USB and choose **Remove**, then **Reboot**.

Originals: `/data_persist/mazda-cmu-usb-patcher`. Logs: `logs/` on the USB.
Do not delete originals while installed. Remove before changing the bundle.
The message-footer change needs a factory reset; run Install again afterwards
to restore touch/navigation activation.

## Credits

- Android Auto code adapted from [oem-aa-mod](https://github.com/VitaliyKurokhtin/oem-aa-mod) and [PingusPepan's fork](https://github.com/PingusPepan/oem-aa-mod).
- CarPlay code adapted from [mazda-carplay-hud](https://github.com/KidMixer/mazda-carplay-hud).

Licensed under [AGPL-3.0-or-later](LICENSE).
