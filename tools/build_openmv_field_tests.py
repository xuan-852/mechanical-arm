"""Export self-contained scripts to open in OpenMV IDE or save as main.py."""

from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = TOOLS_DIR / "field_tests"
PROFILES = {
    "01_loop.py": ("LOOP", 0, "Place on the black line, then press Key1."),
    "02_home_exit.py": ("HOME_EXIT", 450, "Start inside HOME, facing the line, then press Key1."),
    "03_cross_view.py": ("CROSS_VIEW", 0, "Leave Key1 disarmed and move the camera over crosses by hand."),
}


def main():
    source = (TOOLS_DIR / "openmv_comp.py").read_text(encoding="utf-8")
    mode_default = 'FIELD_TEST          = None'
    home_default = 'HOME_EXIT_MS        = 0'
    for default in (mode_default, home_default):
        if sum(line.startswith(default) for line in source.splitlines()) != 1:
            raise SystemExit("Cannot find exactly one configuration line for " + default)

    scripts = {}
    for filename, (mode, home_exit_ms, setup) in PROFILES.items():
        script = source.replace(mode_default, 'FIELD_TEST          = "%s"' % mode, 1)
        script = script.replace(home_default, "HOME_EXIT_MS        = %d" % home_exit_ms, 1)
        script = ("# %s\n" % setup +
                  "# Generated from tools/openmv_comp.py by tools/build_openmv_field_tests.py.\n" + script)
        compile(script, filename, "exec")
        scripts[filename] = script

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for filename, script in scripts.items():
        path = OUTPUT_DIR / filename
        path.write_text(script, encoding="utf-8")
        print(path)


if __name__ == "__main__":
    main()
