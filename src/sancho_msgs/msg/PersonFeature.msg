# Un único bloque de características para una persona detectada
uint32 id               # identificador de la persona (track_id)
float32[] embedding     # vector de características (Re‑ID) de longitud dinámica
geometry_msgs/PointStamped position   # posición de la persona en el frame 'map'
