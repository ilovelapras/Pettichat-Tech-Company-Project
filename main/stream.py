import websocket, json

ws = websocket.WebSocket()
ws.connect("ws://192.168.0.40/live")

while True:
    data = json.loads(ws.recv())
    print(data)
