import socket
import serial
import threading

TCP_IP = '0.0.0.0'  # รับทุก IP ที่เชื่อมมา
TCP_PORT = 8888
BUFFER_SIZE = 1024

# กำหนด Serial Port ตามที่เชื่อมต่อ ESP32 (เช่น /dev/ttyUSB0)
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

def handle_client(client_socket, ser):
    print("เชื่อมต่อ TCP Client เรียบร้อย")
    try:
        while True:
            data = client_socket.recv(BUFFER_SIZE)
            if not data:
                break
            # ส่งข้อมูลผ่าน Serial
            ser.write(data)
            print(f"ส่ง UART: {data}")
    except Exception as e:
        print(f"เกิดข้อผิดพลาด: {e}")
    finally:
        client_socket.close()
        print("ปิดการเชื่อมต่อ TCP Client")

def tcp_server():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((TCP_IP, TCP_PORT))
    server.listen(1)
    print(f"พร้อมรับการเชื่อมต่อ TCP ที่ {TCP_IP}:{TCP_PORT}")

    try:
        while True:
            client_sock, addr = server.accept()
            print(f"มีการเชื่อมต่อจาก {addr}")
            client_thread = threading.Thread(target=handle_client, args=(client_sock, ser))
            client_thread.start()
    except KeyboardInterrupt:
        print("หยุด Server")
    finally:
        ser.close()
        server.close()

if __name__ == "__main__":
    tcp_server()
