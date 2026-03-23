#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import struct
import numpy as np
import cv2
from time import sleep


class TCPImageClient(Node):
    def __init__(self, server_ip, server_port):
        super().__init__('tcp_image_display')
        self.server_ip = server_ip
        self.server_port = server_port
        self.sock = None
        self.connect()
        cv2.namedWindow('Received Image', cv2.WINDOW_NORMAL)

    def connect(self):
        if self.sock:
            self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.server_ip, self.server_port))
            self.get_logger().info(f"Connected to {self.server_ip}:{self.server_port}")
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except socket.error as e:
            self.get_logger().error(f"Connection failed: {e}")
            self.sock = None

    def parse_device_message(self, data: bytes) -> bytes:
        """Parse the device's custom binary format (mirrors C++ deserializeROSMessage).

        Layout (all little-endian):
          [4B seq] [4B sec] [4B nsec]
          [4B frame_id_len] [frame_id_len B frame_id]
          [4B format_len]   [format_len B format]
          [4B jpeg_len]     [jpeg_len B jpeg_data]
        """
        offset = 0
        offset += 4  # skip seq
        offset += 8  # skip sec + nsec

        frame_id_len = struct.unpack_from('<I', data, offset)[0]
        offset += 4 + frame_id_len

        format_len = struct.unpack_from('<I', data, offset)[0]
        offset += 4 + format_len

        jpeg_len = struct.unpack_from('<I', data, offset)[0]
        offset += 4

        return data[offset: offset + jpeg_len]

    def recv_exact(self, n: int) -> bytes | None:
        """Read exactly n bytes from the socket, return None on disconnect."""
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def receive_images(self):
        while rclpy.ok():
            if not self.sock:
                # Keep the OpenCV event loop alive while waiting to reconnect
                cv2.waitKey(100)
                sleep(0.9)
                self.connect()
                continue

            try:
                header = self.recv_exact(4)
                if header is None:
                    self.get_logger().warn("Connection closed by server")
                    self.sock = None
                    continue

                data_size = struct.unpack('!I', header)[0]

                data = self.recv_exact(data_size)
                if data is None:
                    self.get_logger().warn("Connection closed during data transfer")
                    self.sock = None
                    continue

                jpeg_bytes = self.parse_device_message(data)

                np_arr = np.frombuffer(jpeg_bytes, np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if cv_img is not None:
                    cv2.imshow("Received Image", cv_img)
                    cv2.waitKey(1)
                else:
                    self.get_logger().error("Failed to decode image")

            except socket.error as e:
                self.get_logger().error(f"Socket error: {e}")
                self.sock = None
            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")

        cv2.destroyAllWindows()
        if self.sock:
            self.sock.close()


def main():
    rclpy.init()
    client = TCPImageClient("192.168.1.2", 8888)
    try:
        client.receive_images()
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
