import io
import json
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, request, render_template_string, jsonify
import insightface

# ---------------- CONFIGURACIÓN ----------------

GALLERY_DIR = Path.home() / ".buscabot_face_gallery"
GALLERY_DIR.mkdir(parents=True, exist_ok=True)

THRESHOLD = 0.35  
STATUS_PATH = GALLERY_DIR / "face_status.txt"  # Estado generado por ROS2

app = Flask(__name__)


# ------------- CARGA DE MODELO GLOBAL -------------

print("[INFO] Cargando modelo InsightFace (buffalo_l)...")
app_face = insightface.app.FaceAnalysis(
    name="buffalo_l",
    providers=["CPUExecutionProvider"]
)
app_face.prepare(ctx_id=0, det_size=(640, 480))
print("[INFO] Modelo cargado.")


def obtener_embedding(imagen_cv2):
    """Detecta una cara y devuelve su embedding normalizado"""
    faces = app_face.get(imagen_cv2)
    if not faces:
        return None

    emb = faces[0].normed_embedding
    norm = np.linalg.norm(emb)
    if norm == 0:
        return None

    return emb / norm


# ------------------ HTML DE LA INTERFAZ -------------------

HTML = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8" />
    <title>BuscaBot · Reconocimiento Facial</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 30px;
            background: #f4f4f7;
        }
        h1 { color: #222; }
        .card {
            background: white;
            padding: 20px;
            margin-bottom: 25px;
            border-radius: 12px;
            box-shadow: 0 2px 6px rgba(0,0,0,0.15);
        }
        .ok     { color: green; font-weight: bold; }
        .fail   { color: red; font-weight: bold; }
        #estado {
            font-size: 20px;
            padding: 10px;
        }
    </style>
</head>
<body>

<h1>🔎 BuscaBot – Herramienta de Reconocimiento Facial</h1>

<div class="card">
    <h2>1️⃣ Subir fotografía de la persona objetivo</h2>
    <form action="/subir_objetivo" method="POST" enctype="multipart/form-data">
        <input type="text" name="nombre" placeholder="Nombre de la persona" required />
        <br><br>
        <input type="file" name="foto" required />
        <button type="submit">Cargar persona objetivo</button>
    </form>
    <p>{{ mensaje_objetivo }}</p>
</div>

<div class="card">
    <h2>2️⃣ Estado de la cámara en vivo</h2>

    <div id="estado">Cargando...</div>
</div>

<script>
    // Refrescar cada 1 segundo
    function actualizarEstado() {
        fetch("/estado")
            .then(r => r.json())
            .then(data => {
                let e = document.getElementById("estado");
                if (data.ok) {
                    e.innerHTML = "🟢 " + data.texto;
                    e.className = "ok";
                } else {
                    e.innerHTML = "🔴 " + data.texto;
                    e.className = "fail";
                }
            })
    }

    setInterval(actualizarEstado, 1000);
    actualizarEstado();
</script>

</body>
</html>
"""


# ------------------ RUTAS FLASK -------------------

@app.route("/")
def index():
    return render_template_string(HTML, mensaje_objetivo="")


@app.route("/subir_objetivo", methods=["POST"])
def subir_objetivo():
    # Nombre escrito en el formulario
    nombre = request.form.get("nombre", "Persona objetivo")

    archivo = request.files.get("foto")
    if not archivo:
        return render_template_string(HTML, mensaje_objetivo="❌ No seleccionaste imagen")

    img_bytes = archivo.read()
    img_np = np.frombuffer(img_bytes, np.uint8)
    img_cv = cv2.imdecode(img_np, cv2.IMREAD_COLOR)

    emb = obtener_embedding(img_cv)
    if emb is None:
        return render_template_string(
            HTML,
            mensaje_objetivo="❌ No se detectó un rostro claro en la foto"
        )

    data = {
        "person_name": nombre,        # 🔹 clave que usa el nodo ROS2
        "embedding": emb.tolist()
    }

    json_path = GALLERY_DIR / "target_current.json"
    json_path.write_text(json.dumps(data, indent=2))

    return render_template_string(
        HTML,
        mensaje_objetivo=f"✅ Persona objetivo '{nombre}' guardada correctamente"
    )


@app.route("/estado")
def estado():
    """Devuelve el estado actual del nodo ROS2"""
    if not STATUS_PATH.exists():
        return jsonify({"ok": False, "texto": "Sin datos aún"})

    texto = STATUS_PATH.read_text().strip()

    ok = "detectada" in texto or "✔" in texto

    return jsonify({
        "ok": ok,
        "texto": texto
    })


# ------------------- MAIN --------------------

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000, debug=False)
