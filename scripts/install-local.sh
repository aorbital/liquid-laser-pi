#!/usr/bin/env bash
set -euo pipefail
PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$HOME/.local/bin"
ln -sf "$PROJECT/scripts/laser" "$HOME/.local/bin/laser"
ln -sf "$PROJECT/scripts/laser-update" "$HOME/.local/bin/laser-update"
chmod +x "$PROJECT/scripts/laser" "$PROJECT/scripts/laser-update"
if ! grep -q 'HOME/.local/bin' "$HOME/.bashrc" 2>/dev/null; then echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"; fi
echo "Installed laser + laser-update. Run: source ~/.bashrc"
