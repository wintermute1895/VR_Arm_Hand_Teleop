#!/usr/bin/env python3
import socket
import json
import sys

def main():
    """
    一个极简的UDP监听器，用于调试。
    它假设每个UDP包都是一个独立的、完整的JSON。
    """
    udp_ip = "0.0.0.0"
    udp_port = 9999

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((udp_ip, udp_port))
        print(f"Simple Inspector started. Listening on UDP port {udp_port}...")
        print("Press Ctrl+C to stop.")
    except Exception as e:
        print(f"Error: Could not bind to port {udp_port}. Details: {e}", file=sys.stderr)
        return

    packet_count = 0
    try:
        while True:
            # 直接接收一个数据包
            data, addr = sock.recvfrom(8192)
            packet_count += 1
            
            # 立即尝试解码和解析
            try:
                json_string = data.decode('utf-8').strip() # .strip() 移除开头结尾的空白符
                
                # 如果解码后是空字符串，就跳过
                if not json_string:
                    continue

                data_packet = json.loads(json_string)
                
                # --- 如果成功解析，就打印出来 ---
                print("-" * 20 + f" Packet #{packet_count} " + "-" * 20)
                # 完整地打印整个解析后的字典
                print(json.dumps(data_packet, indent=2))

            except (UnicodeDecodeError, json.JSONDecodeError) as e:
                # 如果解析失败，打印错误和原始数据，让我们看看是什么
                print(f"\n--- ERROR parsing Packet #{packet_count} ---")
                print(f"Error: {e}")
                print(f"Raw data received (first 100 bytes): {data[:100]}")
                print("-" * 50)
            
    except KeyboardInterrupt:
        print("\n\nInspector stopped by user.")
    finally:
        sock.close()
        print("Socket closed.")

if __name__ == '__main__':
    main()