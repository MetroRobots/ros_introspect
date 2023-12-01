def identifier_split(s, delimiters=' _-'):
    tokens = []
    current = ''

    def starts_new_token(c):
        if c in delimiters:
            return True
        return current and current[-1].islower() and c.isupper()

    for c in s:
        if starts_new_token(c):
            if current:
                tokens.append(current)
            current = ''

        if c not in delimiters:
            current += c
    if current:
        tokens.append(current)
    return tokens


def convert_to_underscore_notation(name):
    return '_'.join(a.lower() for a in identifier_split(name))


def convert_to_caps_notation(name):
    return ''.join([x.title() for x in identifier_split(name)])
