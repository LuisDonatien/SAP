#!/usr/bin/env python3
"""
Script to generate eros_pkg.sv from a HJSON config and a SystemVerilog template,
by replacing placeholders `${section.subsection}` with uppercase hex values
(assuming the template already includes the `32'h` prefix).

Usage:
    $ python generate_eros_pkg.py

Assumes:
  - config file at: configs/addr.hjson
  - template file at: rtl/include/eros_pkg.sv.tpl
  - output file will be written to: rtl/include/eros_pkg.sv

Dependencies:
  pip install hjson
"""
import hjson
import re
from pathlib import Path
import sys


def get_config_value(cfg, path):
    """
    Traverse nested dict `cfg` by keys in `path` list.
    """
    val = cfg
    for key in path:
        if isinstance(val, dict) and key in val:
            val = val[key]
        else:
            raise KeyError(f"Config key '{'.'.join(path)}' not found")
    return val


def to_hex_string(val):
    """
    Convert a value to uppercase hex string without '0x' prefix.
    """
    if isinstance(val, int):
        return format(val, 'X')
    s = str(val).strip().strip('"').strip("'")
    if s.lower().startswith('0x'):
        s = s[2:]
    return s.upper()


def main():
    # Paths relative to project root
    config_path = Path('configs/addr.hjson')
    template_path = Path('rtl/include/eros_pkg.sv.tpl')
    output_path = template_path.with_suffix('')  # remove .tpl to .sv

    # Validate files
    if not config_path.is_file():
        print(f"Error: config file not found at {config_path}", file=sys.stderr)
        sys.exit(1)
    if not template_path.is_file():
        print(f"Error: template file not found at {template_path}", file=sys.stderr)
        sys.exit(1)

    # Load config text and strip HJSON comments (lines or inline #)
    raw = config_path.read_text()
    # Remove any '#' comments (everything after # on a line)
    clean = re.sub(r"#.*", "", raw)
    try:
        cfg = hjson.loads(clean)
    except Exception as e:
        print(f"Error parsing HJSON config: {e}", file=sys.stderr)
        sys.exit(1)

    # Read template
    content = template_path.read_text()

    # Regex to match ${section.subsection}
    pattern = re.compile(r"\$\{([A-Za-z0-9_]+(?:\.[A-Za-z0-9_]+)*)\}")

    # Replacement function
    def repl(match):
        keypath = match.group(1).split('.')
        try:
            val = get_config_value(cfg, keypath)
        except KeyError as e:
            print(f"Warning: {e}", file=sys.stderr)
            return match.group(0)
        hexstr = to_hex_string(val)
        return hexstr

    # Perform substitution
    new_content, count = pattern.subn(repl, content)
    print(f"Total replacements: {count}")

    # Write output
    output_path.write_text(new_content)
    print(f"Generated {output_path}")


if __name__ == '__main__':
    main()
