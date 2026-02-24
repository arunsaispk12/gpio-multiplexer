#!/data/data/com.termux/files/usr/bin/bash
set -e

REPO_DIR="/storage/emulated/0/CLI_PATH/gpio-multiplexer"
cd "$REPO_DIR"

echo "==> Removing corrupted .git directory..."
rm -rf .git

echo "==> Initializing fresh git repository..."
git init
git config --global --add safe.directory "$REPO_DIR"

echo "==> Staging files..."
git add README.md docs/

echo "==> Creating initial commit..."
git commit -m "Initial commit: GPIO multiplexer documentation suite

Covers analog muxes (74HC4051/4052/4053, CD74HC4067),
I2C expanders (PCF8574, MCP23017, MCP23008),
SPI expanders (MCP23S17), shift registers (74HC595/165),
and I2C bus mux (TCA9548A). Includes ESP-IDF C examples,
wiring diagrams, comparison tables, and troubleshooting guide."

echo "==> Checking GitHub repo status..."
if gh repo view gpio-multiplexer &>/dev/null 2>&1; then
  echo "==> GitHub repo already exists, adding remote..."
  REMOTE_URL=$(gh repo view gpio-multiplexer --json sshUrl -q .sshUrl)
  git remote add origin "$REMOTE_URL"
else
  echo "==> Creating new private GitHub repo..."
  gh repo create gpio-multiplexer \
    --private \
    --description "GPIO multiplexer & I/O expander reference — types, wiring, ESP-IDF examples" \
    --source . \
    --remote origin
fi

echo "==> Pushing to GitHub..."
git push -u origin HEAD

echo ""
echo "Done! Repository:"
gh repo view gpio-multiplexer --json url -q .url
