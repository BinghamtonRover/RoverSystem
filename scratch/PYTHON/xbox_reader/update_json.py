from .rewrite_json import rewrite


class JSONUpdateError(Exception):
    """An exception for JSON update failures."""


def update(json_file, instruction):
    try:
        pass # do_update
    except JSONUpdateError:
        rewrite(json_file, instruction)