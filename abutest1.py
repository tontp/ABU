import socket
import keyboard
import time
from datetime import datetime

class SimpleKeyboardClient:
    def __init__(self, host="192.168.0.200", port=8888):
        self.host = host
        self.port = port
        self.sock = None
        self.connected = False

        self.last_cmd = None
        # ตัวแปรเก็บสถานะการกดปุ่ม
        self.space_was_pressed = False
        self.enter_was_pressed = False
        self.f_was_pressed = False
        self.up_arrow_pressed = False
        self.down_arrow_pressed = False
        self.j_was_pressed = False
        self.l_was_pressed = False

        self.stats = {
            'start_time': datetime.now(),
            'commands_sent': 0,
            'connection_count': 0
        }

    def connect_to_server(self):
        try:
            print(f"🔄 กำลังเชื่อมต่อไปยัง {self.host}:{self.port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            self.connected = True
            self.stats['connection_count'] += 1
            print("✅ เชื่อมต่อสำเร็จ!")
            self.show_controls()
            return True
        except Exception as e:
            print(f"❌ ไม่สามารถเชื่อมต่อได้: {e}")
            return False

    def show_controls(self):
        print("\n" + "="*60)
        print("🎮 คำสั่งการควบคุม")
        print("="*60)
        print("🚀 การเคลื่อนไหวพื้นฐาน:")
        print("   W = เดินหน้า    S = ถอยหลัง")
        print("   A = เลี้ยวซ้าย   D = เลี้ยวขวา")
        print("   Q = หมุนซ้าย    E = หมุนขวา")
        print("\n🔄 การเคลื่อนไหวแนวทแยง:")
        print("   W+A = เดินหน้าซ้าย    W+D = เดินหน้าขวา")
        print("   S+A = ถอยหลังซ้าย    S+D = ถอยหลังขวา")
        print("\n⚡ ฟังก์ชันพิเศษ:")
        print("   Space = ยิงกระบอกสูบ")
        print("   Enter = เปิด/ปิดชุดยิง")
        print("   ↑ = Linear UP")
        print("   ↓ = Linear DOWN")
        print("   F = ฟังก์ชันพิเศษ")
        print("\n🛡 j: ยกที่รับบอลขึ้น")
        print("🛡 l: เอาที่รับบอลลง")
        print("\n💡 เพิ่มเติม: กด 0=ปิดมอเตอร์, 1=แรงต่ำ, 2=แรงกลาง, 3=แรงสูง")
        print("\n❌ หยุดโปรแกรม: Ctrl+C")
        print("="*60 + "\n")

    def get_key(self):
        # ปุ่มปรับความแรงมอเตอร์ 0-3
        for key in ['0', '1', '2', '3']:
            if keyboard.is_pressed(key):
                return key

        # ปุ่มอื่น ๆ (เหมือนเดิม)
        if keyboard.is_pressed('f') and not self.f_was_pressed:
            self.f_was_pressed = True
            return 'f'
        elif not keyboard.is_pressed('f'):
            self.f_was_pressed = False

        if keyboard.is_pressed('w') and keyboard.is_pressed('a'):
            return '3'
        elif keyboard.is_pressed('w') and keyboard.is_pressed('d'):
            return '4'
        elif keyboard.is_pressed('s') and keyboard.is_pressed('a'):
            return '1'
        elif keyboard.is_pressed('s') and keyboard.is_pressed('d'):
            return '2'
        elif keyboard.is_pressed('w'):
            return 'w'
        elif keyboard.is_pressed('s'):
            return 's'
        elif keyboard.is_pressed('a'):
            return 'a'
        elif keyboard.is_pressed('d'):
            return 'd'
        elif keyboard.is_pressed('q'):
            return 'q'
        elif keyboard.is_pressed('e'):
            return 'e'
        return 'x'

    def get_command_description(self, char):
        commands = {
            'w': 'เดินหน้า',
            's': 'ถอยหลัง',
            'a': 'เลี้ยวซ้าย',
            'd': 'เลี้ยวขวา',
            'x': 'หยุด',
            'q': 'หมุนซ้าย',
            'e': 'หมุนขวา',
            '1': 'ถอยซ้าย',
            '2': 'ถอยขวา',
            '3': 'เดินหน้าซ้าย',
            '4': 'เดินหน้าขวา',
            ' ': 'ยิงกระบอกสูบ',
            'm': 'เปิด/ปิดชุดยิง',
            'U': 'Linear UP ON',
            'u': 'Linear UP OFF',
            'O': 'Linear DOWN ON',
            'o': 'Linear DOWN OFF',
            'f': 'ฟังก์ชันพิเศษ',
            'j': 'ยกที่รับบอลขึ้น',
            'l': 'เอาที่รับบอลลง',
            '0': 'มอเตอร์ปิด',
            '1': 'มอเตอร์ระดับต่ำ',
            '2': 'มอเตอร์ระดับกลาง',
            '3': 'มอเตอร์แรงสุด',
        }
        return commands.get(char, 'ไม่รู้จัก')

    def send_command(self, command):
        if not self.connected or not self.sock:
            return False
        try:
            self.sock.send(command.encode())
            self.stats['commands_sent'] += 1
            desc = self.get_command_description(command)
            print(f"📤 ส่ง: '{command}' - {desc}")
            return True
        except Exception as e:
            print(f"❌ ไม่สามารถส่งคำสั่งได้: {e}")
            self.connected = False
            return False

    def handle_special_keys(self):
        if keyboard.is_pressed('space') and not self.space_was_pressed:
            self.send_command(' ')
            self.space_was_pressed = True
        elif not keyboard.is_pressed('space'):
            self.space_was_pressed = False

        if keyboard.is_pressed('enter') and not self.enter_was_pressed:
            self.send_command('m')
            self.enter_was_pressed = True
        elif not keyboard.is_pressed('enter'):
            self.enter_was_pressed = False

        if keyboard.is_pressed('up') and not self.up_arrow_pressed:
            self.send_command('U')
            self.up_arrow_pressed = True
        elif self.up_arrow_pressed and not keyboard.is_pressed('up'):
            self.send_command('u')
            self.up_arrow_pressed = False

        if keyboard.is_pressed('down') and not self.down_arrow_pressed:
            self.send_command('O')
            self.down_arrow_pressed = True
        elif self.down_arrow_pressed and not keyboard.is_pressed('down'):
            self.send_command('o')
            self.down_arrow_pressed = False

        if keyboard.is_pressed("j") and not self.j_was_pressed:
            self.send_command("J")
            self.j_was_pressed = True
        elif self.j_was_pressed and not keyboard.is_pressed("j"):
            self.send_command("j")
            self.j_was_pressed = False

    def show_stats(self):
        uptime = datetime.now() - self.stats['start_time']
        uptime_str = str(uptime).split('.')[0]
        print("\n" + "="*50)
        print("📊 สถิติการทำงาน")
        print("="*50)
        print(f"⏱️  เวลาทำงาน: {uptime_str}")
        print(f"📤 คำสั่งที่ส่ง: {self.stats['commands_sent']}")
        print(f"🔗 การเชื่อมต่อ: {self.stats['connection_count']}")
        print(f"🌐 สถานะ: {'✅ เชื่อมต่อ' if self.connected else '❌ ไม่เชื่อมต่อ'}")
        print("="*50 + "\n")

    def control_loop(self):
        try:
            print("🚀 เริ่มระบบควบคุม - กด Ctrl+C เพื่อออก")
            while self.connected:
                cmd = self.get_key()
                if cmd != self.last_cmd:
                    if not self.send_command(cmd):
                        break
                    self.last_cmd = cmd
                self.handle_special_keys()
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n⚠️  ได้รับสัญญาณหยุด (Ctrl+C)")
        except Exception as e:
            print(f"❌ เกิดข้อผิดพลาด: {e}")

    def disconnect(self):
        if self.sock:
            try:
                self.sock.send(b'x')  # ส่งหยุด
                self.sock.close()
            except:
                pass
        self.connected = False
        print("🔌 ตัดการเชื่อมต่อแล้ว")

def main():
    client = SimpleKeyboardClient()
    if client.connect_to_server():
        client.control_loop()
        client.show_stats()
        client.disconnect()
    else:
        print("โปรดตรวจสอบการเชื่อมต่อเครือข่ายและ IP:PORT")

if __name__ == "__main__":
    main()
