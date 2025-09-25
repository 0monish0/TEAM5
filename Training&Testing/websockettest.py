import websocket
import time

ESP32_IP = "192.168.137.10"
ESP32_PORT = 81

def on_open(ws):
    print("Connected to ESP32")
    for i in range(7):
        msg = f"{i}"
        ws.send(msg)
        print(f"Sent: {msg}")
        time.sleep(7)
    ws.close()

def on_message(ws, message):
    print(f"Received from ESP32: {message}")

def on_error(ws, error):
    print("Error:", error)

def on_close(ws, close_status_code, close_msg):
    print("Closed connection")

ws = websocket.WebSocketApp(f"ws://{ESP32_IP}:{ESP32_PORT}/",
                            on_open=on_open,
                            on_message=on_message,
                            on_error=on_error,
                            on_close=on_close)

ws.run_forever()
