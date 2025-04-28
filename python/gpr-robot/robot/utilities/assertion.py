class AssertionError(Exception):
    pass

def error(message = 'my bad!'):
    raise AssertionError(f'Ops, {message}')

def ensure(condition, message="assertion failed", error_type=AssertionError):
    if not condition:
        raise error_type(message)