import spacy

# Cargar el modelo de spaCy en español
nlp = spacy.load("es_core_news_sm")

def extraer_nombre(texto):
    doc = nlp(texto)
    for entidad in doc.ents:
        if entidad.label_ == "PER":  # Etiqueta para personas
            return entidad.text
    return None

texto_reconocido = "Eulogio"
if texto_reconocido:
    nombre = extraer_nombre(texto_reconocido)
    if nombre:
        print(f"Nombre extraído: {nombre}")
    else:
        print("No se encontró un nombre válido.")
