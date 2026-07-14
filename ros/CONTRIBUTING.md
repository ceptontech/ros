# Contributing (ROS1)

This package follows the official ROS coding style guides:

- **C++**: [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide), enforced with the
  community `.clang-format` config linked from that page
  ([davetcoleman/roscpp_code_format](https://github.com/davetcoleman/roscpp_code_format)),
  checked into this repo as [`ros/.clang-format`](./.clang-format).
- **Python**: [ROS Python Style Guide](http://wiki.ros.org/PyStyleGuide), which is PEP 8,
  enforced with [flake8](https://flake8.pycqa.org/) using
  [`ros/.flake8`](./.flake8) (79-column line length).

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
git ls-files 'ros/*.cpp' 'ros/*.hpp' | xargs clang-format -i

# Python: reformat in place
autopep8 --in-place ros/scripts/*.py
```

## Checking your changes (what CI runs)

```bash
# From the repository root

# C++
git ls-files 'ros/*.cpp' 'ros/*.hpp' | xargs clang-format --dry-run -Werror

# Python
git ls-files 'ros/*.py' | xargs flake8 --config=ros/.flake8
```

## VSCode setup

1. Open this repository (either the repo root or the `ros/` folder) in VSCode.
2. Install the recommended extensions when prompted, or manually install:
   - [`ms-vscode.cpptools`](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) (C/C++)
   - [`ms-python.python`](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
   - [`ms-python.flake8`](https://marketplace.visualstudio.com/items?itemName=ms-python.flake8)
   - [`ms-python.autopep8`](https://marketplace.visualstudio.com/items?itemName=ms-python.autopep8)
3. Format-on-save is preconfigured in `.vscode/settings.json`: C++ files are formatted with
   `ros/.clang-format` and Python files with `autopep8`/`flake8` using `ros/.flake8`.

**Note on clang-format versions**: `cpptools` bundles its own clang-format, which can differ
from the version pinned in CI (`clang-format==18.1.3`). The settings point
`C_Cpp.clang_format_path` at the system `clang-format` binary (`/usr/bin/clang-format` on
Ubuntu) to keep editor output aligned with CI. If your system clang-format differs from
18.1.3, install the pinned version instead: `pip install -r ../ci/lint-requirements.txt` and
update `C_Cpp.clang_format_path` to point at that installation.
