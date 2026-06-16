#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ReSpeaker Mic Array v3.0 (XVF3000) - ROS2 Full Node
퍼블리시 토픽:
  /respeaker/doa    - 소리 방향 각도 (Int32, 0~359도)
  /respeaker/vad    - 음성 감지 여부 (Bool)
  /respeaker/audio  - 마이크 raw 오디오 (UInt8MultiArray, PCM 16bit)
  /respeaker/energy - 음성 에너지 레벨 (Float32, RMS)
"""

import numpy as np
import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32, UInt8MultiArray

# 기존 tuning.py 그대로 import (이미 동작 확인된 코드)
from respeaker.firmware.tuning import find as tuning_find


class RespeakerFullNode(Node):
    RATE   = 16000
    CHUNK  = 1024
    FORMAT = pyaudio.paInt16

    def __init__(self):
        super().__init__('respeaker_full_node')

        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('device_name', 'ReSpeaker')

        update_rate = self.get_parameter('update_rate').value
        device_name = self.get_parameter('device_name').value

        # 1) USB 초기화 — 기존 tuning.find() 그대로 사용
        self.get_logger().info('ReSpeaker USB 연결 중...')
        self.tuning = tuning_find()
        if self.tuning is None:
            raise RuntimeError('ReSpeaker를 찾을 수 없습니다. USB 연결을 확인하세요.')
        self.get_logger().info('ReSpeaker USB 연결 성공')

        # 2) 오디오 장치 탐색
        self.pa = pyaudio.PyAudio()
        device_index, channels = self._find_device(device_name)
        self._channels = channels

        # 3) 오디오 스트림 오픈
        self.stream = self.pa.open(
            rate=self.RATE,
            channels=channels,
            format=self.FORMAT,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=self.CHUNK,
        )

        # 퍼블리셔
        self.pub_doa    = self.create_publisher(Int32,           '/respeaker/doa',    10)
        self.pub_vad    = self.create_publisher(Bool,            '/respeaker/vad',    10)
        self.pub_audio  = self.create_publisher(UInt8MultiArray, '/respeaker/audio',  10)
        self.pub_energy = self.create_publisher(Float32,         '/respeaker/energy', 10)

        # 타이머
        self.timer       = self.create_timer(1.0 / update_rate,     self._usb_callback)
        self.audio_timer = self.create_timer(self.CHUNK / self.RATE, self._audio_callback)

        self.get_logger().info(
            'RespeakerFullNode 시작\n'
            '  /respeaker/doa    (Int32)           소리 방향 0~359도\n'
            '  /respeaker/vad    (Bool)             음성 감지 여부\n'
            '  /respeaker/audio  (UInt8MultiArray)  PCM 16bit raw\n'
            '  /respeaker/energy (Float32)          RMS 에너지'
        )

    def _find_device(self, device_name):
        for i in range(self.pa.get_device_count()):
            info = self.pa.get_device_info_by_index(i)
            if device_name.lower() in info['name'].lower() and info['maxInputChannels'] > 0:
                ch = min(int(info['maxInputChannels']), 6)
                self.get_logger().info(f'오디오 장치 발견: [{i}] {info["name"]} (ch={ch})')
                return i, ch

        self.get_logger().warn(f'"{device_name}" 장치를 찾지 못했습니다. 사용 가능한 입력 장치:')
        for i in range(self.pa.get_device_count()):
            info = self.pa.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                self.get_logger().warn(f'  [{i}] {info["name"]} (ch={info["maxInputChannels"]})')
        return None, 1

    def _usb_callback(self):
        try:
            doa_msg      = Int32()
            doa_msg.data = int(self.tuning.direction)
            self.pub_doa.publish(doa_msg)

            vad_msg      = Bool()
            vad_msg.data = bool(self.tuning.is_voice())
            self.pub_vad.publish(vad_msg)

        except Exception as e:
            self.get_logger().warn(f'USB 읽기 실패: {e}')

    def _audio_callback(self):
        try:
            if self.stream.get_read_available() < self.CHUNK:
                return

            raw = self.stream.read(self.CHUNK, exception_on_overflow=False)

            # Raw 오디오 퍼블리시
            audio_msg      = UInt8MultiArray()
            audio_msg.data = list(raw)
            self.pub_audio.publish(audio_msg)

            # RMS 에너지 계산 (ch0만)
            samples = np.frombuffer(raw, dtype=np.int16)
            if self._channels > 1:
                samples = samples[::self._channels]
            rms             = float(np.sqrt(np.mean(samples.astype(np.float32) ** 2)))
            energy_msg      = Float32()
            energy_msg.data = rms
            self.pub_energy.publish(energy_msg)

        except Exception as e:
            self.get_logger().warn(f'오디오 읽기 실패: {e}')

    def destroy_node(self):
        if hasattr(self, 'tuning') and self.tuning:
            try:
                self.tuning.close()
            except Exception:
                pass
        if hasattr(self, 'stream'):
            try:
                self.stream.stop_stream()
                self.stream.close()
            except Exception:
                pass
        if hasattr(self, 'pa'):
            self.pa.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RespeakerFullNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[ERROR] {e}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
