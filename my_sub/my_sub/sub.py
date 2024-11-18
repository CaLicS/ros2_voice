import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import vosk
import sys
import sounddevice as sd
import queue
import json
import threading

# Vosk 음성 인식을 위한 설정
model = vosk.Model("/home/downy/vosk-model-small-ko-0.22")  # Vosk 모델 경로
q = queue.Queue()

def callback(indata, frames, time, status):
    q.put(bytes(indata))

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        self.subscription = self.create_subscription(
            String,
            'help',  # ROS2 토픽 이름
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'gui_text', 10)  # 인식된 텍스트 발행을 위한 퍼블리셔
        self.is_recognizing = False  # 음성 인식 활성화 상태 플래그
        self.recognizer_thread = None  # 음성 인식을 위한 스레드

    def listener_callback(self, msg):
        if msg.data == "start" and not self.is_recognizing:
            self.get_logger().info('Starting speech recognition...')
            self.is_recognizing = True
            self.recognizer_thread = threading.Thread(target=self.start_recognition)
            self.recognizer_thread.start()
        elif msg.data == "stop" and self.is_recognizing:
            self.get_logger().info('Stopping speech recognition...')
            self.is_recognizing = False
            if self.recognizer_thread is not None:
                self.recognizer_thread.join()  # 스레드 종료 대기

    def start_recognition(self):
        mic_device_index = 0

        with sd.RawInputStream(samplerate=48000, blocksize=8000, dtype='int16',
                               channels=1, callback=callback, device=mic_device_index):
            rec = vosk.KaldiRecognizer(model, 48000)
            self.get_logger().info('Listening for speech...')
            while self.is_recognizing:
                data = q.get()
                if rec.AcceptWaveform(data):
                    result = json.loads(rec.Result())
                    recognized_text = result["text"]
                    self.get_logger().info(f'Recognized text: {recognized_text}')
                    
                    # 인식된 텍스트를 토픽 메시지로 발행
                    msg = String()
                    msg.data = recognized_text
                    self.publisher_.publish(msg)
                else:
                    self.get_logger().info('Listening...')
        self.get_logger().info('Speech recognition stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

