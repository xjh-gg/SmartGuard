# -*- coding: utf-8 -*-
"""
SmartGuard UDP 接收端 (v0.7)
接收 STM32 通过 ESP8266 发来的告警/数据 JSON 消息

用法: python udp_receiver.py
监听: 0.0.0.0:12345  (所有网卡的 12345 端口)
"""
import socket
import json
from datetime import datetime

UDP_PORT = 12345

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))

    print("=" * 50)
    print("  SmartGuard UDP Receiver v0.7")
    print(f"  监听端口: {UDP_PORT}")
    print(f"  本地 IP:  192.168.137.1 (热点网关)")
    print("  等待 STM32 告警消息...")
    print("=" * 50)

    while True:
        try:
            data, addr = sock.recvfrom(256)
            msg = data.decode('utf-8', errors='replace').strip()
            ts = datetime.now().strftime("%H:%M:%S")

            try:
                obj = json.loads(msg)
                mtype = obj.get('type', '?')
                if mtype == 'alert':
                    print(f"[{ts}] ⚠️  ALERT from {addr[0]}: {obj.get('msg', msg)}")
                elif mtype == 'data':
                    print(f"[{ts}] 📊  DATA from {addr[0]}: {obj.get('payload', msg)}")
                else:
                    print(f"[{ts}] ???  {addr[0]}: {msg}")
            except json.JSONDecodeError:
                print(f"[{ts}] 📨  RAW from {addr[0]}: {msg}")

        except KeyboardInterrupt:
            print("\n已退出")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == '__main__':
    main()
