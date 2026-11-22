# Apply one unified diff without dropping context or guessing between matches.
# Only ordinary, single-file text patches are supported.
BEGIN { if (final_newline == "") final_newline = 1 }
function emit(line) {
    if (have_output) print pending_output
    pending_output = line
    have_output = 1
}
function fail(message) {
    print "Patch rejected: " message > "/dev/stderr"
    failed = 1
    exit 1
}
function finish_hunk() {
    if (h && (old_count[h] != expected_old || new_count[h] != expected_new))
        fail("hunk length differs from its header")
}
FILENAME == ARGV[1] {
    if (FNR == 1) {
        if ($0 !~ /^--- a\//) fail("missing original header")
        target = substr($0, 7)
        next
    }
    if (FNR == 2) {
        if ($0 != "+++ b/" target) fail("mismatched target header")
        next
    }
    if ($0 ~ /^@@ /) {
        finish_hunk()
        if ($0 !~ /^@@ -[0-9]+(,[0-9]+)? \+[0-9]+(,[0-9]+)? @@( .*)?$/)
            fail("unsupported hunk header")
        split($0, fields, " ")
        n = split(substr(fields[2], 2), range, ",")
        old_start = range[1] + 0
        expected_old = n == 2 ? range[2] + 0 : 1
        n = split(substr(fields[3], 2), range, ",")
        expected_new = n == 2 ? range[2] + 0 : 1
        if (expected_old < 1 || old_start < 1 || old_start < previous_end)
            fail("empty or overlapping original hunk")
        previous_end = old_start + expected_old
        ++h
        next
    }
    if (!h) fail("content outside a hunk")
    prefix = substr($0, 1, 1)
    if (prefix != " " && prefix != "-" && prefix != "+")
        fail("unsupported patch record")
    content = substr($0, 2)
    if (prefix != "+") old[h, ++old_count[h]] = content
    if (prefix != "-") replacement[h, ++new_count[h]] = content
    next
}
{ source[++lines] = $0 }
END {
    if (failed) exit 1
    finish_hunk()
    if (!h || !lines) fail("empty patch or target")
    cursor = 1
    for (part = 1; part <= h; ++part) {
        matches = 0
        for (start = cursor; start <= lines - old_count[part] + 1; ++start) {
            for (i = 1; i <= old_count[part]; ++i)
                if (source[start + i - 1] != old[part, i]) break
            if (i > old_count[part]) {
                found = start
                ++matches
            }
        }
        if (matches != 1) fail("expected one exact match for hunk " part ", found " matches)
        while (cursor < found) emit(source[cursor++])
        for (i = 1; i <= new_count[part]; ++i) emit(replacement[part, i])
        cursor = found + old_count[part]
    }
    while (cursor <= lines) emit(source[cursor++])
    if (have_output) {
        printf "%s", pending_output
        if (final_newline) printf "\n"
    }
}
