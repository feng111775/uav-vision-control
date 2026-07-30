"""Load the flat, numeric D-task field configuration."""


def load_field_parameters(path):
    values = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if line:
            key, value = line.split(":", 1)
            values[key.strip()] = float(value.strip())
    return values
