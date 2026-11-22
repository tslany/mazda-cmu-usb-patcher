#!/bin/sh
# Reversible, receipt-based installer for Mazda Connect 74.00.324A.
set -u
LC_ALL=C
export LC_ALL

NAME="Mazda CMU USB Patcher"
BUNDLE=$(dirname "$(readlink -f "$0")")
MOD=/data_persist/mazda-cmu-usb-patcher
STATE=$MOD/state-v1
LOCK=/tmp/mazda-cmu-usb-patcher.lock
DIALOG=/jci/tools/jci-dialog
WATCHDOG='/sys/class/gpio/Watchdog Disable/value'
MENU_SCRIPT=/jci/scripts/set_speed_restriction_config.sh
TOUCH_SCRIPT=/jci/scripts/set_lvds_speed_restriction_config.sh
NVRAM=/sys/class/nvram/nv-config/keys
NAV_LINK=/data/debug_options.ini
NAV_STOCK=/jci/nng/NNG_debugging_OFF.ini
NAV_INSTALLED=$MOD/nng-unrestricted.ini
MESSAGE=/data/mazda-cmu-usb-patcher-message-reset-needed
PACKAGE_FILES="packages/navigation-unlock/root/data_persist/mazda-cmu-usb-patcher/nng-unrestricted.ini
packages/projection-guidance/root/data_persist/mazda-cmu-usb-patcher/libpatch-aap_service.so
packages/projection-guidance/root/data_persist/mazda-cmu-usb-patcher/libpatch-blmjciaapa.so
packages/projection-guidance/root/data_persist/mazda-cmu-usb-patcher/libpatch-blmjcicarplay.so
packages/projection-guidance/root/data_persist/mazda-cmu-usb-patcher/libpatch-svcjcinavi.so"
STAGE= LOCKED=0 ROOT_RW=0 REBOOT=0 WATCHDOG_SAVED= PROGRESS= ACTIVE_TEMP= ERROR=
LOG_FILE=$BUNDLE/logs/install.log

log() { printf '%s\n' "$*"; }
die() { ERROR=$*; log "ERROR: $*" >&2; exit 1; }
file_digest() {
    digest_output=$(openssl dgst -sha256 "$1") || return 1
    printf '%s\n' "${digest_output##* }"
}
verify_sums() {
    while read -r expected_digest digest_file; do
        actual_digest=$(file_digest "$digest_file") || return 1
        [ "$actual_digest" = "$expected_digest" ] || {
            log "Checksum mismatch: $digest_file" >&2
            return 1
        }
    done < "$1"
}
regular() { [ -f "$1" ] && [ ! -L "$1" ] || die "Missing or unsafe file: $1"; }
exists() { [ -e "$1" ] || [ -L "$1" ]; }
has_line() { awk -v wanted="$1" '$0 == wanted { found=1 } END { exit !found }' "$2"; }
safe_path() {
    case "$1" in ''|/*|*..*|*[!A-Za-z0-9._/+-]*|*//*|*/|./*|*/./*) return 1;; esac
}
no_links() {
    nl_path=$1
    while [ "$nl_path" != / ] && [ "$nl_path" != . ]; do
        if [ -L "$nl_path" ]; then
            case "$nl_path:$(readlink "$nl_path")" in
                /data_persist:/mnt/data_persist) nl_path=/mnt/data_persist; continue;;
                /data:/mnt/data) nl_path=/mnt/data; continue;;
                /mnt:/tmp/mnt) nl_path=/tmp/mnt; continue;;
                *) die "Symlink in managed path: $nl_path";;
            esac
        fi
        nl_path=$(dirname "$nl_path")
    done
}
stop_progress() {
    if [ -n "$PROGRESS" ]; then
        kill "$PROGRESS" 2>/dev/null || true
        wait "$PROGRESS" 2>/dev/null || true
        PROGRESS=
    fi
}
root_mode() {
    # Linux can list the underlying rootfs as rw even when the real root is ro.
    awk '$2 == "/" && ($3 != "rootfs" || !seen) { options=$4; seen=1 }
         END {
             if (options ~ /(^|,)rw(,|$)/) print "rw";
             else if (options ~ /(^|,)ro(,|$)/) print "ro";
             else exit 1;
         }' /proc/mounts
}
close_root() {
    [ "$ROOT_RW" -eq 1 ] || return 0
    sync
    # Live CMU processes can keep the filesystem busy. That does not undo
    # verified file writes; finish the transaction and require a restart.
    if ! mount -o ro,remount / || [ "$(root_mode)" != ro ]; then
        log "WARNING: Root could not be made read-only; restart the CMU before using it."
    fi
    ROOT_RW=0
}
restore_watchdog() {
    [ -n "$WATCHDOG_SAVED" ] || return 0
    printf '%s\n' "$WATCHDOG_SAVED" > "$WATCHDOG" || return 1
    [ "$(cat "$WATCHDOG")" = "$WATCHDOG_SAVED" ] || return 1
    WATCHDOG_SAVED=
}
cleanup() {
    cleanup_status=$1
    trap - 0 1 2 15
    stop_progress
    [ -z "$ACTIVE_TEMP" ] || rm -f "$ACTIVE_TEMP"
    close_root
    restore_watchdog || { log "ERROR: watchdog state could not be restored"; cleanup_status=1; }
    if [ "$cleanup_status" -ne 0 ] && [ -x "$DIALOG" ]; then
        sync
        "$DIALOG" --question --title="$NAME" --text="Operation stopped.\n$ERROR\n\nLog: $LOG_FILE\nIf a transaction started, choose Remove to restore it." --ok-label="Close" --cancel-label="Close" || true
    fi
    case "$STAGE" in /tmp/cmu-install.*) rm -rf "$STAGE";; esac
    if [ "$LOCKED" -eq 1 ]; then rm -f "$LOCK/pid"; rmdir "$LOCK"; fi
    if [ "$cleanup_status" -eq 0 ] && [ "$REBOOT" -eq 1 ]; then
        log "Rebooting CMU."
        sync
        if ! reboot; then
            log "ERROR: Could not reboot the CMU."
            "$DIALOG" --question --title="$NAME" --text="Could not reboot. Restart the CMU manually." --ok-label="Close" --cancel-label="Close" || true
            cleanup_status=1
        fi
    fi
    exit "$cleanup_status"
}
take_lock() {
    if ! mkdir "$LOCK" 2>/dev/null; then
        # A crashed process can leave a lock; never steal one from a live PID.
        if [ -d "$LOCK" ] && [ ! -L "$LOCK" ] && [ -f "$LOCK/pid" ] && [ ! -L "$LOCK/pid" ]; then
            lock_pid=$(cat "$LOCK/pid")
            case "$lock_pid" in ''|*[!0-9]*) return 1;; esac
            kill -0 "$lock_pid" 2>/dev/null && return 1
            rm -f "$LOCK/pid" && rmdir "$LOCK" && mkdir "$LOCK" || return 1
        else return 1; fi
    fi
    LOCKED=1
    printf '%s\n' "$$" > "$LOCK/pid" || die "Could not record the installer lock"
}
start_log() {
    case "$BUNDLE" in
        /tmp/mnt/sda1|/tmp/mnt/sdb1|/tmp/mnt/sdc1|/tmp/mnt/sdd1)
            log_usb_options=$(awk -v usb="$BUNDLE" '$2 == usb && $3 == "vfat" { print $4 }' /proc/mounts)
            case ",$log_usb_options," in
                *,ro,*) mount -o remount,rw "$BUNDLE" || log "WARNING: USB could not be made writable";;
            esac;;
    esac
    log_dir=$BUNDLE/logs
    if [ -L "$log_dir" ] || [ -L "$log_dir/install.log" ] || ! mkdir -p "$log_dir" 2>/dev/null || ! (umask 077; : >> "$log_dir/install.log") 2>/dev/null; then
        log_dir=/tmp/mazda-cmu-usb-patcher-logs
        no_links "$log_dir"
        mkdir -p "$log_dir" || die "Could not create a log directory"
    fi
    LOG_FILE=$log_dir/install.log
    no_links "$LOG_FILE"
    exec >> "$LOG_FILE" 2>&1
    log "--- $NAME: $(date) ---"
}
firmware_guard() {
    regular /jci/version.ini
    has_line 'JCI_SW_VER="MAZ_CMU-150_74.00.324"' /jci/version.ini || die "Only firmware 74.00.324A is supported"
    has_line 'JCI_SW_VER_PATCH="A"' /jci/version.ini || die "Only firmware patch A is supported"
    FLAVOR=$(awk '/^JCI_SW_FLAVOR=/ { ++count; if ($0 !~ /^JCI_SW_FLAVOR="cmu150_[A-Za-z0-9]+"$/) bad=1; value=$0 } END { if (count != 1 || bad) exit 1; sub(/^JCI_SW_FLAVOR="/, "", value); sub(/"$/, "", value); print value }' /jci/version.ini) || die "Unrecognized or ambiguous firmware variant"
    log "Firmware: 74.00.324A $FLAVOR"
}
manifest_has() { awk -v wanted="$1" '$2 == wanted { found=1 } END { exit !found }' "$BUNDLE/SHA256SUMS" || die "File is missing from SHA256SUMS: $1"; }
verify_bundle() {
    regular "$BUNDLE/SHA256SUMS"
    awk 'NF != 2 || length($1) != 64 || $1 ~ /[^0-9a-f]/ || seen[$2]++ { exit 1 } END { if (!NR) exit 1 }' "$BUNDLE/SHA256SUMS" || die "Invalid checksum manifest"
    while read -r digest relative; do
        safe_path "$relative" || die "Unsafe manifest path"
        no_links "$BUNDLE/$relative"
        regular "$BUNDLE/$relative"
    done < "$BUNDLE/SHA256SUMS"
    (cd "$BUNDLE" && verify_sums SHA256SUMS) || die "USB checksum verification failed"
    manifest_has install.sh
    manifest_has apply-patch.awk
}
choose_action() {
    "$DIALOG" --3-button-dialog --title="$NAME" --text="Install this bundle or remove it using its saved originals?" --ok-label="Install" --cancel-label="Remove" --button3-label="Cancel"
    case "$?" in
        0) ACTION=install;;
        1) ACTION=remove;;
        2) ACTION=cancel;;
        *) die "Action dialog failed";;
    esac
}
atomic_copy() {
    ac_source=$1 ac_destination=$2
    regular "$ac_source"
    no_links "$ac_destination"
    mkdir -p "$(dirname "$ac_destination")" || die "Could not create destination directory"
    # Persistent payloads share the receipt's filesystem. Keep their temporary
    # files there so Remove also retires copies interrupted by power loss.
    case "$ac_destination" in
        "$MOD"/*) ac_template=$STATE/.copy.XXXXXX;;
        *) ac_template=$ac_destination.cmu.XXXXXX;;
    esac
    ACTIVE_TEMP=$(mktemp "$ac_template") || die "Could not stage $ac_destination"
    cp -p "$ac_source" "$ACTIVE_TEMP" || die "Could not copy $ac_destination"
    /bin/fsync "$ACTIVE_TEMP" || die "Could not flush $ac_destination"
    cmp -s "$ac_source" "$ACTIVE_TEMP" || die "Copy verification failed: $ac_destination"
    mv -f "$ACTIVE_TEMP" "$ac_destination" || die "Could not activate $ac_destination"
    ACTIVE_TEMP=
    sync
}
write_value() {
    printf '%s\n' "$2" > "$STAGE/value" || die "Could not stage state value"
    chmod 0600 "$STAGE/value" || die "Could not protect state value"
    atomic_copy "$STAGE/value" "$1"
}
phase() { write_value "$STATE/phase" "$1"; }
root_writable() {
    # Do not force an already-writable root into a different mount mode.
    case "$(root_mode)" in
        rw) return 0;;
        ro) ;;
        *) die "Could not determine root mount mode";;
    esac
    # Set the cleanup flag first, including when remount reports a partial failure.
    ROOT_RW=1
    mount -o rw,remount / || die "Could not remount root writable"
    [ "$(root_mode)" = rw ] || die "Root remount did not become writable"
}
begin_mutation() {
    WATCHDOG_SAVED=$(cat "$WATCHDOG") || die "Cannot read watchdog state"
    case "$WATCHDOG_SAVED" in 0|1) ;; *) WATCHDOG_SAVED=; die "Unexpected watchdog state";; esac
    printf '1\n' > "$WATCHDOG" || die "Cannot suspend watchdog for file installation"
    [ "$(cat "$WATCHDOG")" = 1 ] || die "Watchdog write did not take effect"
    "$DIALOG" --info --title="$NAME" --text="Applying changes. Keep USB connected and power on." --no-cancel &
    PROGRESS=$!
}
root_target() {
    safe_path "${1#/}" || return 1
    case "$1" in /jci/*|/etc/devmgr_config_master.xml|/etc/aap_system_attributes.xml|/etc/aap_system_attributes_UCP.xml|/etc/syslog-ng.conf|/config/common/log/settings.txt) return 0;; esac
    return 1
}
disabled_plugin_target() {
    case "$1" in
        /jci/vui/plugins/vui_plugin_aharadio.so|/jci/vui/plugins/vui_plugin_pandora.so|/jci/vui/plugins/vui_plugin_stitcher.so) return 0;;
        /jci/vui/plugins/vui_plugin_satradio.so) [ "$FLAVOR" != cmu150_NA ]; return;;
    esac
    return 1
}
add_target() {
    at_kind=$1 at_target=$2
    has_line "$at_target" "$STAGE/seen" && return 0
    no_links "$at_target"
    at_original=absent
    if exists "$at_target"; then
        regular "$at_target"
        at_original=file
        mkdir -p "$STAGE/receipt/original$(dirname "$at_target")" || die "Could not stage originals"
        cp -p "$at_target" "$STAGE/receipt/original$at_target" || die "Could not snapshot $at_target"
    fi
    printf '%s|%s|%s\n' "$at_kind" "$at_original" "$at_target" >> "$STAGE/receipt/targets"
    printf '%s\n' "$at_target" >> "$STAGE/seen"
}
stage_patch_target() {
    pt_kind=$1 pt_target=$2
    regular "$pt_target"
    add_target "$pt_kind" "$pt_target"
    pt_staged=$STAGE/receipt/installed$pt_target
    if [ ! -f "$pt_staged" ]; then
        mkdir -p "$(dirname "$pt_staged")" || die "Could not stage patch target"
        cp -p "$pt_target" "$pt_staged" || die "Could not stage $pt_target"
    fi
    # An already-patched runtime copy needs no change, but still gets a receipt.
    if [ "$pt_target" = /data/log/syslog-ng.conf ] && cmp -s "$pt_staged" "$STAGE/receipt/installed/etc/syslog-ng.conf"; then
        return
    fi
    pt_final_newline=$(tail -c 1 "$pt_staged" | wc -l)
    cp -p "$pt_staged" "$STAGE/patched" || die "Could not stage patched output"
    awk -v final_newline="$pt_final_newline" -f "$BUNDLE/apply-patch.awk" "$patch_file" "$pt_staged" > "$STAGE/patched" || die "Exact patch check failed: $patch_name ($pt_target)"
    mv -f "$STAGE/patched" "$pt_staged" || die "Could not retain patched output"
}
stage_patches() {
    streaming_standard=0 streaming_wcp=0 sirius_standard=0 sirius_wcp=0
    for patch_group in normal boot-hooks; do
        [ -d "$BUNDLE/patches/$patch_group" ] || die "Missing patch directory"
        no_links "$BUNDLE/patches/$patch_group"
        for patch_file in "$BUNDLE/patches/$patch_group"/*.patch; do
            exists "$patch_file" || continue
            patch_name=$(basename "$patch_file")
            case "$patch_name" in [0-9][0-9][0-9]-*.patch) ;; *) die "Invalid patch filename";; esac
            safe_path "$patch_name" || die "Unsafe patch filename"
            regular "$patch_file"
            manifest_has "patches/$patch_group/$patch_name"
            case "$FLAVOR:$patch_name" in cmu150_NA:*-disable-sirius-*.patch) log "Keeping Sirius on NA: $patch_name"; continue;; esac
            patch_relative=$(sed -n '1s|^--- a/||p' "$patch_file")
            patch_target=/$patch_relative
            root_target "$patch_target" || die "Unsupported patch target: $patch_target"
            case "$patch_group:$patch_target" in
                normal:/jci/sm/*) die "Service patches belong in boot-hooks";;
                boot-hooks:/jci/sm/sm.conf|boot-hooks:/jci/sm/sm_WCP.conf) ;;
                boot-hooks:*) die "Invalid boot-hook target";;
            esac
            stage_patch_target "$patch_group" "$patch_target"
            case "$patch_group:$patch_name:$patch_target" in
                boot-hooks:240-disable-streaming-standard.patch:/jci/sm/sm.conf) streaming_standard=1;;
                boot-hooks:250-disable-streaming-wcp.patch:/jci/sm/sm_WCP.conf) streaming_wcp=1;;
                boot-hooks:260-disable-sirius-standard.patch:/jci/sm/sm.conf) sirius_standard=1;;
                boot-hooks:270-disable-sirius-wcp.patch:/jci/sm/sm_WCP.conf) sirius_wcp=1;;
                normal:063-console-log-off.patch:/etc/syslog-ng.conf)
                    if exists /data/log/syslog-ng.conf; then
                        stage_patch_target normal /data/log/syslog-ng.conf
                    else
                        # Own the runtime copy too, including its original absence.
                        add_target normal /data/log/syslog-ng.conf
                        mkdir -p "$STAGE/receipt/installed/data/log" || die "Could not stage runtime log template"
                        cp -p "$STAGE/receipt/installed/etc/syslog-ng.conf" "$STAGE/receipt/installed/data/log/syslog-ng.conf" || die "Could not save runtime log template"
                    fi;;
            esac
            log "Staged $patch_name"
        done
    done
    [ "$streaming_standard" = "$streaming_wcp" ] || die "Streaming disable patches must be included or omitted together"
    [ "$sirius_standard" = "$sirius_wcp" ] || die "Sirius disable patches must be included or omitted together"
}
stage_disabled_plugins() {
    for plugin in aharadio pandora stitcher satradio; do
        case "$plugin" in
            satradio) [ "$sirius_standard" -eq 1 ] || continue;;
            *) [ "$streaming_standard" -eq 1 ] || continue;;
        esac
        plugin_target=/jci/vui/plugins/vui_plugin_$plugin.so
        disabled_plugin_target "$plugin_target" || die "Unsupported disabled plugin"
        regular "$plugin_target"
        # Keep the original in the receipt, outside the VUI loader's directory.
        add_target disabled-plugin "$plugin_target"
    done
}
stage_packages() {
    for package_file in $PACKAGE_FILES; do
        manifest_has "$package_file"
        regular "$BUNDLE/$package_file"
        [ -s "$BUNDLE/$package_file" ] || die "Empty package payload"
        package_target=/${package_file#*/root/}
        add_target package "$package_target"
        mkdir -p "$STAGE/receipt/installed$(dirname "$package_target")" || die "Could not stage package"
        cp "$BUNDLE/$package_file" "$STAGE/receipt/installed$package_target" || die "Could not stage package payload"
        chmod 0644 "$STAGE/receipt/installed$package_target" || die "Could not set payload mode"
    done
    add_target preference "$MOD/disable-mute-pause"
    if exists "$BUNDLE/disable-mute-pause"; then
        regular "$BUNDLE/disable-mute-pause"
        : > "$STAGE/receipt/installed$MOD/disable-mute-pause" || die "Could not stage mute preference"
        chmod 0644 "$STAGE/receipt/installed$MOD/disable-mute-pause" || die "Could not protect mute preference"
    fi
}
read_nvram() {
    nvram_value=$(cat "$NVRAM/$1") || die "Cannot read NVRAM key $1"
    case "$nvram_value" in enable|disable) printf '%s\n' "$nvram_value";; *) die "Unexpected NVRAM value for $1";; esac
}
navigation_state() {
    if [ -L "$NAV_LINK" ]; then readlink "$NAV_LINK"
    elif ! exists "$NAV_LINK"; then printf '%s\n' absent
    else return 1; fi
}
stage_actions() {
    no_links "$(dirname "$NAV_LINK")"
    nav_before=$(navigation_state) || die "Navigation configuration belongs to another install"
    case "$nav_before" in absent|"$NAV_STOCK") ;; *) die "Remove the existing navigation tweak first";; esac
    printf '%s\n' "$nav_before" > "$STAGE/receipt/nav-before" || die "Could not save navigation state"
    if has_line /jci/gui/common/js/Common.js "$STAGE/seen"; then
        read_nvram bus_bcm_speed_restriction > "$STAGE/receipt/menu-before" || die "Could not capture menu setting"
        read_nvram lvds_speed_restriction > "$STAGE/receipt/touch-before" || die "Could not capture touch setting"
    fi
    if has_line /jci/settings/configurations/blm_msg-system.xml "$STAGE/seen"; then
        no_links "$MESSAGE"
        : > "$STAGE/receipt/message-change" || die "Could not save message action"
    fi
    file_digest "$BUNDLE/SHA256SUMS" > "$STAGE/receipt/bundle-digest" || die "Could not identify the bundle"
    printf '%s\n' "$FLAVOR" > "$STAGE/receipt/flavor" || die "Could not save firmware variant"
}
seal_receipt() {
    (cd "$STAGE/receipt" && find . -type f | sort | while IFS= read -r sealed_file; do
        sealed_digest=$(file_digest "$sealed_file") || exit 1
        printf '%s  %s\n' "$sealed_digest" "$sealed_file"
    done) > "$STAGE/receipt-sums" || die "Could not checksum the recovery receipt"
    mv "$STAGE/receipt-sums" "$STAGE/receipt/SHA256SUMS" || die "Could not finish recovery receipt"
    mkdir -p "$MOD" || die "Could not create persistent directory"
    if exists "$STATE.prepare"; then
        [ -d "$STATE.prepare" ] && [ ! -L "$STATE.prepare" ] || die "Unsafe pending receipt"
        rm -rf "$STATE.prepare" || die "Could not clear an uncommitted receipt"
    fi
    cp -a "$STAGE/receipt" "$STATE.prepare" || die "Could not save recovery receipt"
    chmod 0700 "$STATE.prepare" || die "Could not protect recovery receipt"
    sync
    mv "$STATE.prepare" "$STATE" || die "Could not activate recovery receipt"
    sync
    phase prepared
}
check_legacy() {
    for legacy_path in "$MOD/state" "$MOD/state.restored" /data_persist/oem-aa-mod /data_persist/cp-hud-mod /data_persist/mazda-cmu-projection-fixes /data_persist/cmu-tweaker; do
        exists "$legacy_path" && die "Remove the older installation using its original USB first: $legacy_path"
    done
    no_links "$MOD"
}
prepare_install() {
    exists "$STATE" && die "Already installed or interrupted. Remove before installing or changing this bundle."
    for existing in "$MOD"/* "$MOD"/.[!.]* "$MOD"/..?*; do
        exists "$existing" || continue
        case "$existing" in "$MOD/disable-mute-pause"|"$STATE.prepare"|"$STATE.removed") ;; *) die "Unowned existing file: $existing";; esac
    done
    mkdir -p "$STAGE/receipt/original" "$STAGE/receipt/installed" || die "Could not stage receipt"
    : > "$STAGE/receipt/targets"
    : > "$STAGE/seen"
    stage_patches
    stage_disabled_plugins
    stage_packages
    stage_actions
    seal_receipt
}
receipt_target() {
    case "$1:$2" in
        normal:/data/log/syslog-ng.conf) return 0;;
        normal:*) root_target "$2";;
        boot-hooks:/jci/sm/sm.conf|boot-hooks:/jci/sm/sm_WCP.conf) return 0;;
        disabled-plugin:*) disabled_plugin_target "$2";;
        preference:"$MOD/disable-mute-pause") return 0;;
        package:*)
            for allowed_package in $PACKAGE_FILES; do
                [ "$2" = "/${allowed_package#*/root/}" ] && return 0
            done
            return 1;;
        *) return 1;;
    esac
}
load_receipt() {
    [ -d "$STATE" ] && [ ! -L "$STATE" ] || die "No recovery receipt found"
    find "$STATE" -type l | grep -q . && die "Symlink in recovery receipt"
    regular "$STATE/SHA256SUMS"
    awk 'NF != 2 || length($1) != 64 || $1 ~ /[^0-9a-f]/ || $2 !~ /^\.\// || $2 ~ /\.\./ || seen[$2]++ { exit 1 } END { if (!NR) exit 1 }' "$STATE/SHA256SUMS" || die "Invalid recovery checksums"
    (cd "$STATE" && verify_sums SHA256SUMS) || die "Recovery receipt is damaged"
    regular "$STATE/targets"
    for receipt_required in targets nav-before bundle-digest flavor; do
        awk -v wanted="./$receipt_required" '$2 == wanted { found=1 } END { exit !found }' "$STATE/SHA256SUMS" || die "Incomplete recovery checksum manifest"
    done
    for receipt_optional in menu-before touch-before message-change; do
        if exists "$STATE/$receipt_optional"; then
            regular "$STATE/$receipt_optional"
            awk -v wanted="./$receipt_optional" '$2 == wanted { found=1 } END { exit !found }' "$STATE/SHA256SUMS" || die "Unsealed activation receipt"
        fi
    done
    if [ -f "$STATE/touch-before" ]; then
        regular "$STATE/menu-before"
        for receipt_key in menu touch; do
            case "$(cat "$STATE/$receipt_key-before")" in enable|disable) ;; *) die "Invalid original restriction value";; esac
        done
    fi
    [ "$(cat "$STATE/flavor")" = "$FLAVOR" ] || die "Recovery receipt is for a different firmware variant"
    : > "$STAGE/seen"
    while IFS='|' read -r receipt_kind receipt_original receipt_path; do
        receipt_target "$receipt_kind" "$receipt_path" || die "Invalid recovery target"
        case "$receipt_original" in file) regular "$STATE/original$receipt_path";; absent) ! exists "$STATE/original$receipt_path" || die "Unexpected original image";; *) die "Invalid original type";; esac
        has_line "$receipt_path" "$STAGE/seen" && die "Duplicate recovery target"
        printf '%s\n' "$receipt_path" >> "$STAGE/seen"
        no_links "$receipt_path"
        case "$receipt_kind" in
            preference) ;;
            disabled-plugin)
                [ "$receipt_original" = file ] && ! exists "$STATE/installed$receipt_path" || die "Invalid disabled plugin image";;
            *) regular "$STATE/installed$receipt_path";;
        esac
        for receipt_image in original installed; do
            if [ -f "$STATE/$receipt_image$receipt_path" ]; then
                awk -v wanted="./$receipt_image$receipt_path" '$2 == wanted { found=1 } END { exit !found }' "$STATE/SHA256SUMS" || die "Unsealed recovery image"
            fi
        done
    done < "$STATE/targets"
    [ -s "$STAGE/seen" ] || die "Empty recovery receipt"
    regular "$STATE/nav-before"
    case "$(cat "$STATE/nav-before")" in absent|"$NAV_STOCK") ;; *) die "Invalid original navigation link";; esac
}
image_matches() {
    if [ -f "$1" ]; then [ -f "$2" ] && [ ! -L "$2" ] && cmp -s "$1" "$2"
    else ! exists "$2"; fi
}
verify_owned_live() {
    while IFS='|' read -r owned_kind owned_original owned_path; do
        image_matches "$STATE/original$owned_path" "$owned_path" || image_matches "$STATE/installed$owned_path" "$owned_path" || die "File changed outside this installer: $owned_path"
    done < "$STATE/targets"
    no_links "$(dirname "$NAV_LINK")"
    current_nav=$(navigation_state) || die "Navigation link changed outside this installer"
    case "$current_nav" in absent|"$NAV_STOCK"|"$NAV_INSTALLED") ;; *) die "Navigation link changed outside this installer";; esac
    if [ -f "$STATE/touch-before" ]; then
        for nv_key in menu touch; do
            case "$nv_key" in menu) nv_name=bus_bcm_speed_restriction nv_installed=enable;; touch) nv_name=lvds_speed_restriction nv_installed=disable;; esac
            nv_live=$(read_nvram "$nv_name") || die "Cannot read restriction state"
            [ "$nv_live" = "$nv_installed" ] || [ "$nv_live" = "$(cat "$STATE/$nv_key-before")" ] || [ "$nv_live" = enable ] || die "Restriction state changed outside this installer"
        done
    fi
}
write_group() {
    wg_group=$1 wg_image=$2
    while IFS='|' read -r wg_kind wg_original wg_target; do
        [ "$wg_kind" = "$wg_group" ] || continue
        # Recheck immediately before each write, including resumed removal.
        no_links "$wg_target"
        image_matches "$STATE/original$wg_target" "$wg_target" || image_matches "$STATE/installed$wg_target" "$wg_target" || die "File changed during transaction: $wg_target"
        if [ -f "$STATE/$wg_image$wg_target" ]; then atomic_copy "$STATE/$wg_image$wg_target" "$wg_target"
        else rm -f "$wg_target" || die "Could not remove owned file: $wg_target"; sync; fi
        image_matches "$STATE/$wg_image$wg_target" "$wg_target" || die "Write verification failed: $wg_target"
    done < "$STATE/targets"
}
apply_actions() {
    if [ "$1" = installed ]; then action_nav=$NAV_INSTALLED action_menu=enable action_touch=disable
    else
        action_nav=$(cat "$STATE/nav-before")
        action_menu= action_touch=
        if [ -f "$STATE/touch-before" ]; then action_menu=$(cat "$STATE/menu-before"); action_touch=$(cat "$STATE/touch-before"); fi
    fi
    if [ -f "$STATE/touch-before" ]; then
        "$MENU_SCRIPT" "$action_menu" || die "Menu restriction command failed"
        "$TOUCH_SCRIPT" "$action_touch" || die "Touch restriction command failed"
        [ "$(read_nvram bus_bcm_speed_restriction)" = "$action_menu" ] && [ "$(read_nvram lvds_speed_restriction)" = "$action_touch" ] || die "Restriction verification failed"
    fi
    if [ "$action_nav" = absent ]; then rm -f "$NAV_LINK" || die "Could not restore absent navigation link"
    else
        ACTIVE_TEMP=$NAV_LINK.cmu-$$
        ! exists "$ACTIVE_TEMP" || die "Unexpected navigation temporary"
        ln -s "$action_nav" "$ACTIVE_TEMP" && mv -f "$ACTIVE_TEMP" "$NAV_LINK" || die "Could not update navigation link"
        ACTIVE_TEMP=
    fi
    [ "$(navigation_state)" = "$action_nav" ] || die "Navigation activation verification failed"
    if [ -f "$STATE/message-change" ]; then
        no_links "$MESSAGE"
        if [ "$1" = installed ]; then write_value "$MESSAGE" installed
        else write_value "$MESSAGE" stock; fi
    fi
    sync
}
install_bundle() {
    if exists "$STATE"; then
        load_receipt
        [ "$(cat "$STATE/phase")" = installed ] || die "Interrupted transaction: choose Remove before installing again"
        [ "$(cat "$STATE/bundle-digest")" = "$(file_digest "$BUNDLE/SHA256SUMS")" ] || die "Bundle changed: remove the installed bundle first"
        if exists "$BUNDLE/disable-mute-pause"; then
            regular "$BUNDLE/disable-mute-pause"
            [ -f "$STATE/installed$MOD/disable-mute-pause" ] || die "Mute preference changed: remove first"
        fi
        verify_owned_live
        while IFS='|' read -r repeat_kind repeat_original repeat_path; do
            image_matches "$STATE/installed$repeat_path" "$repeat_path" || die "Installed file differs: remove before reinstalling"
        done < "$STATE/targets"
        begin_mutation
        phase activating
        apply_actions installed
        phase installed
        return
    fi
    prepare_install
    load_receipt
    verify_owned_live
    begin_mutation
    root_writable
    phase installing
    write_group package installed
    write_group preference installed
    write_group normal installed
    apply_actions installed
    write_group disabled-plugin installed
    write_group boot-hooks installed
    close_root
    phase installed
}
remove_bundle() {
    load_receipt
    verify_owned_live
    begin_mutation
    phase removing
    root_writable
    write_group disabled-plugin original
    write_group boot-hooks original
    write_group normal original
    close_root
    apply_actions original
    write_group preference original
    write_group package original
    phase removed
    if exists "$STATE.removed"; then
        [ -d "$STATE.removed" ] && [ ! -L "$STATE.removed" ] || die "Unsafe completed receipt"
        rm -rf "$STATE.removed" || die "Could not clear an earlier completed receipt"
    fi
    mv "$STATE" "$STATE.removed" || die "Files restored, but receipt retirement failed"
    sync
    rm -rf "$STATE.removed" || die "Files restored, but receipt cleanup failed"
}

# Main entry point. Function definitions above can be tested without running it.
trap 'cleanup $?' 0
trap 'ERROR="Interrupted; use Remove to recover any saved transaction"; exit 130' 1 2 15
take_lock || exit 0
start_log
for tool in awk openssl cmp readlink find sort tail wc mktemp; do command -v "$tool" >/dev/null 2>&1 || die "Missing tool: $tool"; done
[ -x "$DIALOG" ] && [ -x /bin/fsync ] || die "Required CMU tools are unavailable"
firmware_guard
verify_bundle
check_legacy
ACTION=${1:-}
[ -n "$ACTION" ] || choose_action
case "$ACTION" in install|remove|uninstall) ;; cancel) log "Cancelled without changes"; exit 0;; *) die "Unknown action: $ACTION";; esac
STAGE=$(mktemp -d /tmp/cmu-install.XXXXXX) || die "Could not create staging directory"
chmod 0700 "$STAGE" || die "Could not protect staging directory"
[ -x "$MENU_SCRIPT" ] && [ -x "$TOUCH_SCRIPT" ] || die "Restriction scripts are unavailable"
regular "$NAV_STOCK"
case "$ACTION" in install) install_bundle;; *) remove_bundle;; esac
restore_watchdog || die "Could not restore watchdog state"
stop_progress
completion="Completed. Reboot to apply changes.\nUnplug the USB when the screen goes dark."
log "$completion"
if "$DIALOG" --question --title="$NAME" --text="$completion" --ok-label="Reboot" --cancel-label="Close"; then
    REBOOT=1
fi
exit 0
