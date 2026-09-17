#!/usr/bin/env bash
# git_backup.sh — Stop hook cua Claude Code: backup workspace len GitHub sau moi luot tra loi.
#
#   - Co thay doi chua commit -> commit het (tru file > 50 MB: GitHub khong nhan file > 100 MB,
#     lot vao lich su thi moi lan push sau deu hong).
#   - Da co remote "origin" va con commit chua day -> push.
#   - Khong bao gio chan Claude (luon exit 0). Loi thi hien canh bao cho nguoi dung.
#
# Khai bao trong .claude/settings.local.json (hooks.Stop). Xem / tat: go /hooks trong Claude Code.

cd "${CLAUDE_PROJECT_DIR:-$(dirname "$0")/../..}" 2>/dev/null || exit 0
git rev-parse --is-inside-work-tree >/dev/null 2>&1 || exit 0

export GIT_TERMINAL_PROMPT=0            # khong bao gio treo cho nhap mat khau
MAX_BYTES=$((50 * 1024 * 1024))
msgs=()

if [ -n "$(git status --porcelain)" ]; then
    git add -A
    skipped=()
    while IFS= read -r -d '' f; do
        if [ -f "$f" ] && [ "$(stat -c %s "$f")" -gt "$MAX_BYTES" ]; then
            git reset -q -- "$f"
            skipped+=("$f")
        fi
    done < <(git diff --cached --name-only -z --diff-filter=AM)
    [ ${#skipped[@]} -gt 0 ] && msgs+=("BO QUA file > 50 MB: ${skipped[*]}")

    n=$(git diff --cached --name-only | wc -l)
    if [ "$n" -gt 0 ]; then
        files=$(git diff --cached --name-only | head -20)
        if git commit -q -m "Backup tu dong: $n file thay doi" -m "$files" >/dev/null 2>&1; then
            msgs+=("da commit $n file")
        else
            msgs+=("commit THAT BAI")
        fi
    fi
fi

if git remote get-url origin >/dev/null 2>&1; then
    if git rev-parse --abbrev-ref '@{u}' >/dev/null 2>&1; then
        ahead=$(git rev-list --count '@{u}..HEAD' 2>/dev/null || echo 0)
    else
        ahead=1                         # chua co nhanh tren GitHub -> day lan dau
    fi
    if [ "$ahead" -gt 0 ]; then
        if err=$(timeout 90 git push -q -u origin HEAD 2>&1); then
            msgs+=("da push len GitHub")
        else
            msgs+=("push THAT BAI: $(printf '%s' "$err" | tail -n 1)")
        fi
    fi
fi

if [ ${#msgs[@]} -gt 0 ]; then
    text="Backup:"
    for m in "${msgs[@]}"; do text="$text $m;"; done
    python3 -c 'import json, sys; print(json.dumps({"systemMessage": sys.argv[1]}))' "${text%;}"
fi
exit 0
