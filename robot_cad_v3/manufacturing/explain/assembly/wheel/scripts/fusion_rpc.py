"""Run a Fusion script file through Fusion's local MCP server with a long timeout.

    python fusion_rpc.py script.py [timeout_s]
"""
import json
import sys
import urllib.request

URL = 'http://127.0.0.1:27182/mcp'


def post(body, sid=None, timeout=60):
    h = {'Content-Type': 'application/json', 'Accept': 'application/json, text/event-stream'}
    if sid:
        h['Mcp-Session-Id'] = sid
    req = urllib.request.Request(URL, json.dumps(body).encode(), h)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        sid = r.headers.get('Mcp-Session-Id', sid)
        raw = r.read().decode('utf-8', 'replace')
    msgs = []
    if raw.lstrip().startswith('{'):
        msgs.append(json.loads(raw))
    else:
        for line in raw.splitlines():
            if line.startswith('data:'):
                msgs.append(json.loads(line[5:].strip()))
    return sid, msgs


def main():
    script = open(sys.argv[1], encoding='utf-8').read()
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 1800
    sid, _ = post({'jsonrpc': '2.0', 'id': 1, 'method': 'initialize',
                   'params': {'protocolVersion': '2025-03-26', 'capabilities': {},
                              'clientInfo': {'name': 'rpc', 'version': '1'}}})
    try:
        post({'jsonrpc': '2.0', 'method': 'notifications/initialized'}, sid)
    except Exception:
        pass
    _, msgs = post({'jsonrpc': '2.0', 'id': 2, 'method': 'tools/call',
                    'params': {'name': 'fusion_mcp_execute',
                               'arguments': {'featureType': 'script', 'object': {'script': script}}}},
                   sid, timeout)
    for m in msgs:
        res = m.get('result') or m.get('error')
        for c in (res or {}).get('content', []):
            t = c.get('text', '')
            try:
                d = json.loads(t)
                print(d.get('message', t))
                if not d.get('success', True):
                    print('FAILED:', d)
            except ValueError:
                print(t)
        if 'error' in m:
            print('ERROR', m['error'])


if __name__ == '__main__':
    main()
