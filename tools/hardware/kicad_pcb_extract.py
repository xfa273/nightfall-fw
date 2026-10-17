#!/usr/bin/env python3
"""Extract footprint reference/value/layer/attrs from a kicad_pcb file."""
import sys, json

def parse_sexp(s):
    i = 0; stack = [[]]; cur = stack[0]; n = len(s)
    while i < n:
        c = s[i]
        if c == '(':
            new = []; cur.append(new); stack.append(new); cur = new; i += 1
        elif c == ')':
            if len(stack) > 1: stack.pop()
            cur = stack[-1]; i += 1
        elif c in ' \t\r\n': i += 1
        elif c == '"':
            j = i+1; buf = ''
            while j < n:
                if s[j] == '\\': buf += s[j+1]; j += 2
                elif s[j] == '"': j += 1; break
                else: buf += s[j]; j += 1
            cur.append(buf); i = j
        else:
            j = i
            while j < n and s[j] not in ' \t\r\n()': j += 1
            cur.append(s[i:j]); i = j
    return stack[0][0]

def get_prop(node, name):
    for x in node:
        if isinstance(x, list) and len(x) >= 2 and x[0] == 'property' and x[1] == name:
            return x[2]
    return None

def find(node, tag, out=None):
    if out is None: out = []
    if isinstance(node, list):
        if node and node[0] == tag: out.append(node)
        for x in node:
            if isinstance(x, list): find(x, tag, out)
    return out

if __name__ == '__main__':
    tree = parse_sexp(open(sys.argv[1], encoding='utf-8').read())
    rows = []
    for fp in [x for x in tree if isinstance(x, list) and x and x[0] == 'footprint']:
        ref = get_prop(fp, 'Reference'); val = get_prop(fp, 'Value')
        layer = None; at = None; rot = None; attrs = []
        for x in fp:
            if isinstance(x, list):
                if x[0] == 'layer' and isinstance(x[1], str): layer = x[1]
                if x[0] == 'at' and len(x) >= 2:
                    at = [float(x[1]), float(x[2]) if len(x) > 2 else None]
                    rot = float(x[3]) if len(x) > 3 else 0.0
                if x[0] == 'attr': attrs = [str(v) for v in x[1:]]
        fields = {}
        for x in find(fp, 'property'):
            if len(x) >= 3: fields[x[1]] = x[2]
        rows.append({'ref': ref, 'value': val, 'footprint': fp[1], 'layer': layer,
                     'at': at, 'rot': rot, 'attr': attrs, 'fields': fields})
    json.dump(rows, sys.stdout, ensure_ascii=False, indent=1)
