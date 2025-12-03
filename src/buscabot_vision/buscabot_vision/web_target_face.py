#!/usr/bin/env python3
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, File, UploadFile, Form
from fastapi.responses import HTMLResponse
import uvicorn
from PIL import Image
import numpy as np
import insightface
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

app = FastAPI()

# Estado global de la última detección recibida desde ROS2
last_detection_message = "Sin detecciones todavía."
last_detection_timestamp = None

# Cargar modelo InsightFace una vez al inicio
face_app = insightface.app.FaceAnalysis(
    name="buffalo_l",
    providers=["CPUExecutionProvider"],
)
face_app.prepare(ctx_id=0, det_size=(640, 640))

BASE_DIR = Path.home() / ".buscabot_face_gallery"
BASE_DIR.mkdir(parents=True, exist_ok=True)
TARGET_JSON = BASE_DIR / "target_current.json"
TARGET_IMG_DIR = Path.home() / "target_faces"
TARGET_IMG_DIR.mkdir(parents=True, exist_ok=True)

class FaceStatusNode(Node):
    """
    Nodo ROS2 que se suscribe a /face_recognition y actualiza
    variables globales que luego consulta la interfaz web.
    """
    def __init__(self):
        super().__init__('web_face_status')
        self.subscription = self.create_subscription(
            String,
            '/face_recognition',
            self.status_callback,
            10
        )

    def status_callback(self, msg: String):
        global last_detection_message, last_detection_timestamp
        last_detection_message = msg.data
        last_detection_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def ros_spin_thread():
    """
    Hilo en segundo plano para correr rclpy.spin sin bloquear FastAPI.
    """
    rclpy.init()
    node = FaceStatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

@app.on_event("startup")
def start_ros_background():
    """
    Al iniciar la aplicación web, se lanza un hilo que escucha /face_recognition.
    """
    thread = threading.Thread(target=ros_spin_thread, daemon=True)
    thread.start()

@app.get("/", response_class=HTMLResponse)
async def index():
    """
    Panel web: carga de persona objetivo + estado del reconocimiento.
    """
    return """
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <title>BuscaBot – Panel de visión</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #0b1120;
      color: #e5e7eb;
      margin: 0;
      padding: 0;
    }
    .container {
      max-width: 900px;
      margin: 0 auto;
      padding: 1.5rem;
    }
    .card {
      background: #020617;
      border-radius: 1rem;
      padding: 1.5rem;
      box-shadow: 0 10px 25px rgba(0,0,0,.35);
      margin-bottom: 1.5rem;
    }
    h1 {
      font-size: 1.5rem;
      margin-bottom: 0.5rem;
    }
    h2 {
      font-size: 1.1rem;
      margin-bottom: 0.75rem;
    }
    label {
      font-weight: 500;
      font-size: 0.9rem;
    }
    input[type="text"],
    input[type="file"] {
      width: 100%;
      margin: 0.35rem 0 1rem;
      padding: 0.5rem 0.75rem;
      border-radius: 0.5rem;
      border: 1px solid #334155;
      background: #020617;
      color: #e5e7eb;
    }
    input[type="submit"] {
      background: #22c55e;
      color: #020617;
      border: none;
      border-radius: 0.75rem;
      padding: 0.6rem 1.2rem;
      font-weight: 600;
      cursor: pointer;
    }
    input[type="submit"]:active {
      transform: scale(0.97);
    }
    .status-pill {
      display: inline-flex;
      align-items: center;
      gap: 0.4rem;
      padding: 0.25rem 0.75rem;
      border-radius: 999px;
      font-size: 0.85rem;
      font-weight: 500;
    }
    .status-ok {
      background: rgba(34,197,94,.15);
      color: #4ade80;
    }
    .status-wait {
      background: rgba(234,179,8,.15);
      color: #facc15;
    }
    .status-off {
      background: rgba(148,163,184,.15);
      color: #cbd5f5;
    }
    .status-time {
      font-size: 0.8rem;
      color: #9ca3af;
      margin-top: 0.25rem;
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="card">
      <h1>Panel de visión BuscaBot</h1>
      <p style="font-size:0.9rem;color:#9ca3af;">
        Estado actual del reconocimiento facial en la Jetson Orin Nano.
      </p>

      <div id="status-pill" class="status-pill status-off">
        <span id="status-text">Sin detecciones todavía…</span>
      </div>
      <div id="status-time" class="status-time">
        Última actualización: –
      </div>
    </div>

    <div class="card">
      <h2>Configurar persona objetivo</h2>
      <form action="/set_target" method="post" enctype="multipart/form-data">
        <label for="person_name">Nombre de la persona</label>
        <input id="person_name" type="text" name="person_name" value="persona_desaparecida">

        <label for="file">Foto (jpg/png)</label>
        <input id="file" type="file" name="file" accept="image/jpeg,image/png">

        <input type="submit" value="Cargar y generar embedding">
      </form>
    </div>
  </div>

  <script>
    async function updateStatus() {
      try {
        const res = await fetch('/status');
        if (!res.ok) return;
        const data = await res.json();

        const text = data.message || 'Sin datos';
        const time = data.timestamp || '—';

        const pill = document.getElementById('status-pill');
        const textEl = document.getElementById('status-text');
        const timeEl = document.getElementById('status-time');

        textEl.textContent = text;
        timeEl.textContent = 'Última actualización: ' + time;

        pill.classList.remove('status-ok','status-wait','status-off');

        if (text.includes('detectada')) {
          pill.classList.add('status-ok');
        } else if (text.includes('Sin detecciones')) {
          pill.classList.add('status-wait');
        } else {
          pill.classList.add('status-off');
        }
      } catch (err) {
        const pill = document.getElementById('status-pill');
        const textEl = document.getElementById('status-text');
        pill.classList.remove('status-ok','status-wait','status-off');
        pill.classList.add('status-off');
        textEl.textContent = 'Error al consultar /status';
      }
    }

    // Primera consulta y refresco periódico
    updateStatus();
    setInterval(updateStatus, 1500);
  </script>
</body>
</html>
"""

@app.post("/set_target", response_class=HTMLResponse)
async def set_target(
    person_name: str = Form(...),
    file: UploadFile = File(...),
):
    # Guardar la imagen subida
    img_bytes = await file.read()
    img_path = TARGET_IMG_DIR / f"{person_name}.jpg"
    with open(img_path, "wb") as f:
        f.write(img_bytes)

    # Abrir la imagen con PIL y convertir a formato adecuado
    img = Image.open(img_path).convert("RGB")
    img_np = np.array(img)

    # Detectar cara
    faces = face_app.get(img_np)
    if not faces:
        return HTMLResponse(
            f"<h2>No se detectó ninguna cara en la imagen subida.</h2>", status_code=400
        )

    # Tomar la cara más grande
    face = max(
        faces,
        key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]),
    )
    emb = face.embedding  # vector de 512

    # Guardar embedding en JSON sencillo
    import json

    data = {
        "person_name": person_name,
        "embedding": emb.tolist(),
    }
    with open(TARGET_JSON, "w") as f:
        json.dump(data, f)

    return f"""
    <html>
      <body>
        <h2>Persona objetivo configurada:</h2>
        <p>Nombre: <b>{person_name}</b></p>
        <p>Imagen guardada en: {img_path}</p>
        <p>Embedding guardado en: {TARGET_JSON}</p>
        <a href="/">Volver</a>
      </body>
    </html>
    """

from fastapi.responses import JSONResponse
# (si JSONResponse ya está importado, no lo repitas)

@app.get("/status")
async def get_status():
    """
    Devuelve el último mensaje recibido en /face_recognition para
    que la interfaz web lo consulte periódicamente.
    """
    return JSONResponse({
        "message": last_detection_message,
        "timestamp": last_detection_timestamp,
    })

def main():
    # Servidor en 0.0.0.0 para que puedas entrar desde el celular
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()

