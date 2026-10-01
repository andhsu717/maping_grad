import http.server
import socketserver
import webbrowser
import os

# 切換到當前檔案所在目錄
os.chdir(os.path.dirname(os.path.abspath(__file__)))

PORT = 8080
Handler = http.server.SimpleHTTPRequestHandler

print(f"正在啟動本機伺服器：http://localhost:{PORT}")
# 自動打開預設瀏覽器訪問該 HTML
webbrowser.open(f"http://localhost:{PORT}/index.html")

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n伺服器已停止")