#!/usr/bin/env python3
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, File, UploadFile, Form
from fastapi.responses import HTMLResponse
import uvicorn
from PIL import Image
import numpy as np
import insightface

app = FastAPI()

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


@app.get("/", response_class=HTMLResponse)
async def index():
    # Página simple para subir foto
    return """
    <html>
      <head>
        <title>BuscaBot - Cargar persona objetivo</title>
      </head>
      <body>
        <h1>Persona objetivo para búsqueda</h1>
        <form action="/set_target" method="post" enctype="multipart/form-data">
          <label>Nombre de la persona:</label>
          <input type="text" name="person_name" value="persona_desaparecida"><br><br>
          <label>Foto (jpg/png):</label>
          <input type="file" name="file"><br><br>
          <input type="submit" value="Cargar y generar embedding">
        </form>
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


def main():
    # Servidor en 0.0.0.0 para que puedas entrar desde el celular
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()

