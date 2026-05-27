import os
import threading
from flask import Flask, Response
import numpy as np
import imageio.v3 as iio
import cv2
from skimage.transform import ProjectiveTransform, warp
from picamera2 import Picamera2
from PIL import Image, ImageDraw, ImageFont
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8

# ---------------- Configuración ----------------
W, H = 820, 616              # resolución de captura
OUT_W, OUT_H = 820, 616      # resolución de procesamiento
JPEG_QUALITY = 50
BOUNDARY = b"frame"

app = Flask(__name__)

# ---------------- Cámara ----------------
picam2 = Picamera2()
picam2.configure(
    picam2.create_video_configuration(
        main={"format": "BGR888", "size": (W, H)}
    )
)
picam2.start()
camera_lock = threading.Lock()

class ROS2ConfigCam(Node): 
    def __init__(self):
        super().__init__("cam_control") 
        self.publisher_ = self.create_publisher(Float32, "curvature", 10)
        self.subscription_ = self.create_subscription(Int8, "robot/mode_cmd", self.callback_change_mode, 10)
        self.get_logger().info("Listener ROS2 has been started")
        self.current_mode = 0
    
    def callback_change_mode(self, msg: Int8):
        self.get_logger().info(f"Modo recibido: {msg.data}")
        self.current_mode = int(msg.data)

    def capture_jpeg(self):
        with camera_lock:
            frame_bgr = picam2.capture_array()
        jpg = iio.imwrite("<bytes>", frame_bgr, extension=".jpg", quality=JPEG_QUALITY)
        return jpg

    def frames_feedforward(self):
        while True:
            if self.current_mode:
                with camera_lock:
                    frame = picam2.capture_array()  
                
                threshold = 130   
                blue = frame[:, :, 0]
                green = frame[:, :, 1]
                red = frame[:, :, 2]
                
                mask = (blue < threshold) & (green < threshold) & (red < threshold)
                resultado = np.zeros_like(green)
                resultado[mask] = 255

                resultado = np.repeat(resultado[:, :, None], 3, axis=2)
                
                puntos = [[325, 242],
                    [262, 467],
                    [569, 467],
                    [523, 240]]
                pts1 = np.float32(puntos)
                pts2 = np.float32([[0,0],[0,OUT_W],[OUT_H,OUT_W],[OUT_H,0]])
                
                tform = ProjectiveTransform()
                tform.estimate(pts1, pts2)

                new_img = warp(
                    resultado,
                    inverse_map=tform.inverse,
                    output_shape=(820, 626),
                    preserve_range=True
                ).astype(np.uint8)

                final_img = Image.fromarray(new_img)
                draw2 = ImageDraw.Draw(final_img)
                
                try:
                    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size=20)
                except IOError:
                    font = ImageFont.load_default()
                
                shape = new_img.shape
                n_partes_image = 17
                tam_parte = shape // n_partes_image
                fin_parte = tam_parte
                origen_image = 0
                r = 4  
                puntos_linea = []
                
                for i in range(n_partes_image):
                    cx, cy = self.blobDetector(new_img[origen_image:fin_parte, :, 0])
                    cx_i = int(cx)
                    cy_i = int(cy + origen_image)
                    if cx_i == -1:
                        puntos_linea.append((-1, -1))
                    else:
                        puntos_linea.append((cx_i, cy_i))
                        draw2.ellipse(
                            [(cx_i - r, cy_i - r), (cx_i + r, cy_i + r)],
                            fill=(255, 0, 0)
                        )
                    origen_image = fin_parte
                    fin_parte += tam_parte
                
                total_deviation = 0
                total_weight = 0

                for i in range(1, len(puntos_linea)):
                    x_last, y_last = puntos_linea[i-1]
                    x_current, y_current = puntos_linea[i]
                    
                    weight = (len(puntos_linea) - i) / len(puntos_linea)
                    total_weight += weight

                    if x_last == -1 or x_current == -1 or x_current <= 1 or x_current >= OUT_W - 1:
                        total_deviation += (np.pi / 2) * weight
                    else:
                        dx = x_current - x_last
                        dy = y_current - y_last
                        theta = np.arctan2(dy, dx)
                        deviation = abs(theta - np.pi / 2)
                        total_deviation += deviation * weight

                avg_deviation = total_deviation / total_weight if total_weight > 0 else (np.pi / 2)
                speed_multiplier = 2.0 * (1.0 - (avg_deviation / (np.pi / 2)))
                curvatura_feedforward = max(0.0, min(2.0, speed_multiplier))

                # Publicación en ROS2
                msg = Float32()
                msg.data = float(curvatura_feedforward)
                self.publisher_.publish(msg)

                text = f"speed_mult= {curvatura_feedforward:.6f}"
                draw2.text((20, 50), text, fill=(255, 0, 0), font=font)

                jpg = iio.imwrite("<bytes>", final_img, extension=".jpg", quality=JPEG_QUALITY)
            else:
                # Modo 0: Stream limpio sin procesar
                jpg = self.capture_jpeg()

            yield (
                b"--" + BOUNDARY + b"\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n" +
                jpg + b"\r\n"
            )

    def blobDetector(self, mask):
        mask = mask.astype(np.uint8)
        m00 = mask.sum()
        if m00 == 0:
            return -1, -1
        ys, xs = np.indices(mask.shape)
        m10 = (xs * mask).sum()
        m01 = (ys * mask).sum()
        return m10 / m00, m01 / m00

ros2_node = None

@app.get("/")
def index():
    return "<img src='/stream' style='max-width:100%;height:auto'/>"

@app.get("/stream")
def stream():
    return Response(ros2_node.frames_feedforward(), mimetype="multipart/x-mixed-replace; boundary=frame")

def run_ros2_loop():
    global ros2_node
    rclpy.init()
    ros2_node = ROS2ConfigCam()
    rclpy.spin(ros2_node)
    ros2_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    ros_thread = threading.Thread(target=run_ros2_loop, daemon=True)
    ros_thread.start()

    try:
        app.run(host="0.0.0.0", port=8080, threaded=True)
    finally:
        picam2.stop()