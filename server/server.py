from http.server import BaseHTTPRequestHandler, HTTPServer
from cgi import parse_header, parse_multipart
import multiprocessing
import server.bagfilter
import server.baglas
import server.bagimg
import urllib
from urllib.parse import urlparse, parse_qs
import json
import os
import time
from threading import Thread


class handler(BaseHTTPRequestHandler):
    def do_GET(self):
        root = os.getcwd()
        self.path = self.path.replace("/%PUBLIC_URL%", "")
        print(f"GET ", self.path)
        filename = root + "/client/build" + self.path
        if self.path.endswith("/"):
            filename += "/index.html"

        self.send_response(200)
        if filename[-4:] == ".css":
            self.send_header("Content-type", "text/css")
        elif filename[-5:] == ".json":
            self.send_header("Content-type", "application/javascript")
        elif filename[-3:] == ".js":
            self.send_header("Content-type", "application/javascript")
        elif filename[-4:] == ".ico":
            self.send_header("Content-type", "image/x-icon")
        else:
            self.send_header("Content-type", "text/html")
        self.end_headers()
        try:
            with open(filename, "rb") as fh:
                html = fh.read()
                self.wfile.write(html)
        except:
            with open(root + "/client/build/index.html", "rb") as fh:
                html = fh.read()
                self.wfile.write(html)


def startServer(requestOpenBrowser, port):
    def openBrowser():
        try:
            import webbrowser

            time.sleep(0.25)
            if requestOpenBrowser:
                print("Opening app at 127.0.0.1:" + str(port))
                print(webbrowser.open("http://127.0.0.1:" + str(port)))
            else:
                print("\033[33mPlease open app at 127.0.0.1:" + str(port) + "\033[0m")
        except:
            print("Cannot open browser automatically. Please go to 127.0.0.1:" + str(port))

    def createServer():
        print("Starting server on 0.0.0.0:" + str(port))
        with HTTPServer(("0.0.0.0", port), handler) as server:
            server.serve_forever()

    Thread(target=createServer).start()
    Thread(target=openBrowser).start()
