# ABI/API Compatibility Checking

This library is consumed by PX4 firmware and QGroundControl as a submodule. To maintain compatibility, pull requests that modify header files (`src/*.h`) are automatically checked for ABI/API breaking changes.

## CI Workflow

The `.github/workflows/abi-check.yml` workflow runs on PRs that modify header files and:
- Compares the PR branch against the base branch using [libabigail](https://sourceware.org/libabigail/)
- Reports ABI compatibility status
- Posts a comment on PRs with breaking changes
- Uploads detailed reports as artifacts

## Local Testing

To check ABI compatibility locally before submitting a PR:

```bash
# Install dependencies (Ubuntu/Debian)
sudo apt-get install abigail-tools cmake

# Run the check (compares current changes against origin/main)
./tools/check-abi.sh

# Or specify custom references
./tools/check-abi.sh origin/main HEAD
./tools/check-abi.sh v1.0.0 my-branch
```

## Breaking Changes

The following changes are considered breaking:

| Change Type | Impact |
|-------------|--------|
| Removed public function | Binary + Source |
| Changed function signature | Binary + Source |
| Changed struct/class layout | Binary |
| Removed struct field | Binary + Source |
| Changed enum values | Binary |
| Changed virtual table layout | Binary |

If breaking changes are intentional, document them in the PR description and update consuming projects accordingly.
