"""Small phone UI server; only stationary capture/cancel actions are exposed."""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from urllib.parse import parse_qs, urlsplit


def make_server(host, port, bridge, token):
    page = (Path(__file__).parent/'web/index.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # Do not log the session token in URLs.

        def reply(self, code, body, content_type='application/json'):
            if not isinstance(body, bytes):
                body = json.dumps(body, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def authorized(self):
            query = parse_qs(urlsplit(self.path).query)
            supplied = self.headers.get('X-Session-Token', query.get('token', [''])[0])
            return hmac.compare_digest(supplied, token)

        def do_GET(self):
            if not self.authorized():
                self.reply(403, {'message': 'Bitte den vollständigen Sitzungslink öffnen.'})
                return
            path = urlsplit(self.path).path
            if path == '/':
                self.reply(200, page, 'text/html; charset=utf-8')
            elif path == '/api/status':
                self.reply(200, bridge.snapshot())
            elif path in ('/image/left.jpg', '/image/right.jpg'):
                frame = bridge.jpeg(path.split('/')[-1].split('.')[0])
                if frame is None:
                    self.reply(503, {'message': 'Kein frisches Kamerabild verfügbar.'})
                else:
                    self.reply(200, frame, 'image/jpeg')
            else:
                self.reply(404, {'message': 'Unbekannter Pfad.'})

        def do_POST(self):
            if not self.authorized():
                self.reply(403, {'message': 'Ungültiger Sitzungslink.'})
                return
            path = urlsplit(self.path).path
            if path == '/api/heartbeat':
                bridge.heartbeat()
                self.reply(200, {'success': True})
            elif path in ('/api/capture', '/api/cancel'):
                action = 'capture' if path.endswith('capture') else 'stop'
                try:
                    result = bridge.request(action).result(timeout=5.)
                    self.reply(200 if result['success'] else 409, result)
                except TimeoutError:
                    self.reply(504, {'success': False, 'message': 'Sitzung antwortet nicht; Status prüfen.'})
                except Exception:
                    self.reply(503, {'success': False, 'message': 'Sitzung nicht erreichbar.'})
            else:
                self.reply(404, {'message': 'Diese Aktion ist im manuellen Modus nicht verfügbar.'})

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
