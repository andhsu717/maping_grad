import http.server
import socketserver
import webbrowser
import os

# 切換到當前檔案所在目錄
os.chdir(os.path.dirname(os.path.abspath(__file__)))

PORT = 8080

class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

Handler = NoCacheHandler

print(f"正在啟動本機伺服器：http://localhost:{PORT}")
# 自動打開預設瀏覽器訪問該 HTML
webbrowser.open(f"http://localhost:{PORT}/index.html")

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n伺服器已停止")