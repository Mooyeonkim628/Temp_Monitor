import socket

HOST = "0.0.0.0"  
PORT = 9000      

def main():
    print(f"APP: Starting TCP server on {HOST}:{PORT}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((HOST, PORT))
        server_sock.listen(1)
        print("APP: Waiting for ESP32 to connect...")

        conn, addr = server_sock.accept()
        with conn:
            print(f"PC APP: Connected by {addr}")
            buffer = b""
            while True:
                data = conn.recv(1024)
                if not data:
                    print("APP: Connection shut down by ESP32.")
                    break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    line_str = line.decode("utf-8", errors="ignore").strip()
                    if line_str:
                        print(f"APP: Received -> {line_str}")

if __name__ == "__main__":
    main()