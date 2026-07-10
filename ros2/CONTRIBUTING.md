# Contributing (ROS2)

These packages follow the official ROS 2 code style:

- **C++**: [ROS 2 Code Style](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html),
  enforced with the official `ament_clang_format` configuration
  ([ament/ament_lint](https://github.com/ament/ament_lint/blob/rolling/ament_clang_format/ament_clang_format/configuration/.clang-format)),
  checked into this repo as [`ros2/.clang-format`](./.clang-format).
- **Python**: enforced with the official `ament_flake8` configuration
  ([ament/ament_lint](https://github.com/ament/ament_lint/blob/rolling/ament_flake8/ament_flake8/configuration/ament_flake8.ini)),
  checked into this repo as [`ros2/.flake8`](./.flake8) — 99-column lines, single quotes,
  Google import ordering, plus the [flake8 plugins](https://index.ros.org/p/ament_flake8/)
  ament_flake8 runs with (blind-except, builtins, class-newline, comprehensions, deprecated,
  docstrings, import-order, quotes).

GitHub Actions runs both checks on every push and fails the build if the code does not
conform (see the `lint` job in [`.github/workflows/main.yml`](../.github/workflows/main.yml)).

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
