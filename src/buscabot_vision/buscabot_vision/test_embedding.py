import cv2
from insightface.app import FaceAnalysis

def gstreamer_pipeline(
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=30,
        flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, "
        f"format=NV12, framerate={framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={display_width}, height={display_height}, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! appsink"
    )

# Inicializar InsightFace en CPU usando el modelo 'buffalo_l'
app = FaceAnalysis(name='buffalo_l', providers=['CPUExecutionProvider'])
app.prepare(ctx_id=-1, det_size=(640, 640))

print("Abriendo cámara CSI con GStreamer...")
cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("❌ No se pudo abrir la cámara con GStreamer.")
    exit(1)

print("✅ Cámara abierta. Presiona 'q' para salir. "
      "Cada vez que detecte una cara, imprimirá un embedding de 512 valores.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠ No se pudo leer un frame de la cámara.")
        break

    faces = app.get(frame)

    for face in faces:
        x1, y1, x2, y2 = face.bbox.astype(int)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        emb = face.embedding
        print("\nEmbedding detectado (primeros 10 valores):")
        print(emb[:10])

    cv2.imshow("Prueba de Embeddings (CSI + InsightFace)", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
