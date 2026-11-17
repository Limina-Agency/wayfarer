"""Copy a dump directory with the named JSON keys removed, for G3 comparisons
across a step that deletes or renames response fields on purpose.

usage: normalize.py <dump_dir> <out_dir> <keys_file> [renames_file]

keys_file lists one key per line. renames_file has `old new` per line: keys named
`old` become `new`, and `.old.` in file names becomes `.new.`. Every .jsonl line
and .json file is parsed, keys are dropped or renamed wherever they occur, and the
value is written back in one canonical form, so both sides of a comparison must
be normalized.
"""
import json
import os
import sys


def strip(value, keys, renames):
    if isinstance(value, dict):
        return {renames.get(k, k): strip(v, keys, renames) for k, v in value.items() if k not in keys}
    if isinstance(value, list):
        return [strip(v, keys, renames) for v in value]
    return value


def main():
    source, target, keys_file, *renames_file = sys.argv[1:]
    keys = {line.strip() for line in open(keys_file) if line.strip()}
    renames = dict(line.split() for f in renames_file for line in open(f) if line.strip())
    for directory, _, files in os.walk(source):
        out_directory = os.path.join(target, os.path.relpath(directory, source))
        os.makedirs(out_directory, exist_ok=True)
        for name in files:
            if not name.endswith(('.json', '.jsonl')):
                continue
            out_name = name
            for old, new in renames.items():
                out_name = out_name.replace(f'.{old}.', f'.{new}.')
            with open(os.path.join(directory, name)) as reader, \
                    open(os.path.join(out_directory, out_name), 'w') as writer:
                if name.endswith('.jsonl'):
                    for line in reader:
                        writer.write(json.dumps(strip(json.loads(line), keys, renames), sort_keys=True) + '\n')
                else:
                    writer.write(json.dumps(strip(json.load(reader), keys, renames), sort_keys=True) + '\n')


if __name__ == '__main__':
    main()
