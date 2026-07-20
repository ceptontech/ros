# Contributing (ROS2)

These packages follow the official ROS 2 code style:

- **C++**: [ROS 2 Code Style](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html),
  enforced with the official `ament_clang_format` configuration
  ([ament/ament_lint](https://github.com/ament/ament_lint/blob/rolling/ament_clang_format/ament_clang_format/configuration/.clang-format)),
  checked into this repo as [`ros2/.clang-format`](./.clang-format).
- **C++ includes**: every header that provides a symbol a `.cpp` file uses must be included
  directly ([Include What You Use](https://include-what-you-use.org/)), checked with
  `clang-tidy`'s `misc-include-cleaner`, configured in [`ros2/.clang-tidy`](./.clang-tidy).
  See [Include What You Use](#include-what-you-use-c) below.
- **Python**: enforced with the official `ament_flake8` configuration
  ([ament/ament_lint](https://github.com/ament/ament_lint/blob/rolling/ament_flake8/ament_flake8/configuration/ament_flake8.ini)),
  checked into this repo as [`ros2/.flake8`](./.flake8) — 99-column lines, single quotes,
  Google import ordering, plus the [flake8 plugins](https://index.ros.org/p/ament_flake8/)
  ament_flake8 runs with (blind-except, builtins, class-newline, comprehensions, deprecated,
  docstrings, import-order, quotes).

GitHub Actions runs these checks on every push and fails the build if the code does not
conform (see the `lint` job in [`.github/workflows/main.yml`](../.github/workflows/main.yml)
for clang-format/flake8, and the `build-driver-ros2` job for the IWYU check).

## Setting up the tools

Install the pinned formatter/linter versions (the same versions CI uses):

```bash
pip install -r ../ci/lint-requirements.txt
```

## Formatting your changes

```bash
# From the repository root

# C++: reformat in place
git ls-files 'ros2/*.cpp' 'ros2/*.h' 'ros2/*.hpp' | xargs clang-format -i

# Python: reformat in place (fixes whitespace/line-length issues;
# quote style and import order still need to follow ros2/.flake8 by hand)
autopep8 --in-place ros2/scripts/*.py ros2/cepton_subscriber/launch/*.py
```

## Checking your changes (what CI runs)

```bash
# From the repository root

# C++
git ls-files 'ros2/*.cpp' 'ros2/*.h' 'ros2/*.hpp' | xargs clang-format --dry-run -Werror

# Python
git ls-files 'ros2/*.py' | xargs flake8 --config=ros2/.flake8
```

`colcon test` with `ament_lint_auto`/`ament_lint_common` is declared as a test dependency in
each package's `package.xml`, but is **not** the source of truth here: `ament_lint_common`
pulls in `ament_uncrustify`/`ament_cpplint`, which enforce a different style than
`ament_clang_format`. The `lint` GitHub Actions job (running the exact commands above) is
authoritative.

## Include What You Use (C++)

Every `.cpp` file must directly `#include` the header providing each symbol it uses, and must
not include headers it doesn't need — the
[Include What You Use](https://include-what-you-use.org/) (IWYU) principle. This is checked
with `clang-tidy`'s
[`misc-include-cleaner`](https://clang.llvm.org/extra/clang-tidy/checks/misc/include-cleaner.html),
configured in [`ros2/.clang-tidy`](./.clang-tidy).

- `rclcpp/rclcpp.hpp` (the umbrella header) and generated message headers (`<pkg>/msg/*.hpp`)
  are exempted via `IgnoreHeaders` in `ros2/.clang-tidy`. This matches ROS 2 convention
  (including the `rclcpp` umbrella rather than its individual headers), and jazzy's `rclcpp`
  does not ship IWYU pragmas that would let the checker see through it on its own.
- The check only analyzes the `.cpp` file passed to `clang-tidy`, not the headers it
  `#include`s. **Header files (`.h`) are not verified by CI** — apply the same discipline by
  hand when a header directly uses a symbol (e.g. `std::mutex`, `std::vector`).
- Covers the `cepton_messages`, `cepton_subscriber`, and `cepton_publisher` packages.

GitHub Actions runs this as a step in the `build-driver-ros2` job (a compilation database from
`colcon build` is required first).

### Checking your changes locally

```bash
# From the repository root, using the same image CI uses
docker build . --tag ros2_dev -f ci/Dockerfile.ros2
docker run --rm -v "$PWD:/app:rw" ros2_dev bash -c '
cd ros2
colcon build --packages-select cepton_messages cepton_subscriber cepton_publisher \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
jq -s add build/*/compile_commands.json > build/compile_commands.json
git ls-files "*.cpp" | grep -E "cepton_(publisher|subscriber)/" | xargs clang-tidy -p build
'
```

### Editor warnings (optional)

`cpptools` has no IWYU diagnostics, so this repo's default editor setup (see below) won't flag
include problems inline. To see the same warnings while editing, install the
[`clangd`](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd)
extension and point it at a `compile_commands.json` generated as above (clangd looks for it at
the workspace root or a parent of the source file). clangd uses the same
`clang-include-cleaner` library as this check and will honor `ros2/.clang-tidy`'s
`IgnoreHeaders`, so its diagnostics match what CI reports — including the header-file blind
spot noted above. This is **not** the default for this repo; `cpptools`/format-on-save remains
the recommended baseline (see [VSCode setup](#vscode-setup)).

## VSCode setup

1. Open this repository (either the repo root or the `ros2/` folder) in VSCode.
2. Install the recommended extensions when prompted, or manually install:
   - [`ms-vscode.cpptools`](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) (C/C++)
   - [`ms-python.python`](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
   - [`ms-python.flake8`](https://marketplace.visualstudio.com/items?itemName=ms-python.flake8)
   - [`ms-python.autopep8`](https://marketplace.visualstudio.com/items?itemName=ms-python.autopep8)
3. Format-on-save is preconfigured in `.vscode/settings.json`: C++ files are formatted with
   `ros2/.clang-format` and Python files with `autopep8`/`flake8` using `ros2/.flake8`.

**Note on clang-format versions**: `cpptools` bundles its own clang-format, which can differ
from the version pinned in CI (`clang-format==18.1.3`). The settings point
`C_Cpp.clang_format_path` at the system `clang-format` binary (`/usr/bin/clang-format` on
Ubuntu) to keep editor output aligned with CI. If your system clang-format differs from
18.1.3, install the pinned version instead: `pip install -r ../ci/lint-requirements.txt` and
update `C_Cpp.clang_format_path` to point at that installation.

**Note on flake8 plugins in the editor**: the `ms-python.flake8` extension uses its own
bundled flake8 without the ament plugins, so it will show a subset of what CI reports (no
quote-style or import-order warnings). For full parity, install
`pip install -r ../ci/lint-requirements.txt` into the interpreter VSCode uses and set
`"flake8.importStrategy": "fromEnvironment"` / `"autopep8.importStrategy": "fromEnvironment"`.
