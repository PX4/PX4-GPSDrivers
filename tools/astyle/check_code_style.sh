#!/usr/bin/env bash
# Checks every tracked C/C++ file against tools/astyle/astylerc; --fix rewrites them in place.
set -eu

DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$DIR/../.."

ASTYLE_VER=$(astyle --version 2>/dev/null || true)
case "$ASTYLE_VER" in
	"Artistic Style Version 2.06" | "Artistic Style Version 3.0" | "Artistic Style Version 3.0.1" | "Artistic Style Version 3.1") ;;
	*)
		echo "astyle 2.06, 3.0, 3.0.1 or 3.1 is required, found: ${ASTYLE_VER:-none}"
		exit 1
		;;
esac

FILES=$(git ls-files '*.c' '*.cpp' '*.h' '*.hpp')

if [ "${1:-}" = "--fix" ]; then
	astyle --options="$DIR/astylerc" --suffix=none --quiet $FILES
	exit 0
fi

status=0

for file in $FILES; do
	if ! astyle --options="$DIR/astylerc" --dry-run "$file" | grep -q '^Unchanged'; then
		echo "Formatting error in $file, run tools/astyle/check_code_style.sh --fix"
		status=1
	fi
done

exit $status
