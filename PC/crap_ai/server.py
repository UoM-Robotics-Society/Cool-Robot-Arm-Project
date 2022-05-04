from email import message
from http.server import BaseHTTPRequestHandler, HTTPServer
import time

# HTTPRequestHandler class
class testHTTPServer_RequestHandler(BaseHTTPRequestHandler):

  # GET
  def do_GET(self):
    # Send response status code
    self.send_response(200)

    # Send message back to client
    message = bytes(
    #   str(self.headers) +
    #   "\n" +
    #   self.requestline +
    #   "\n"
 """BR BN BB BQ BK BB BR BR
    BP BP BP BP .. BP BP BP
    .. .. .. .. .. .. .. ..
    .. .. .. WP BP .. .. ..
    .. .. .. .. .. .. .. ..
    .. .. .. .. .. .. .. ..
    WP WP WP .. WP WP WP WP
    WR WN WB .. WK WB WN WR"""
      , 'utf8')

    # Send headers
    self.send_header('Content-type','text/plain; charset=utf-8')
    self.send_header('Content-length', str(len(message)))
    self.end_headers()

    # Write content as utf-8 data
    self.wfile.write(message)
    return

def run():
  print('starting server...')

  # Server settings
  # Choose port 8080, for port 80, which is normally used for a http server, you need root access
  server_address = ('127.0.0.1', 5000)
  httpd = HTTPServer(server_address, testHTTPServer_RequestHandler)
  print('running server...')
  httpd.serve_forever()


run()