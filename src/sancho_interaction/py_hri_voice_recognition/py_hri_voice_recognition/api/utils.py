import requests

def secondsToCoolFormat(seconds):
    '''Converts seconds to a cool format.
    
    Args:
        seconds (int): Seconds.
    
    Returns:
        result (str): Seconds in "X minuto(s) y X segundo(s)" format.
    '''

    minutes = seconds // 60
    remainingSeconds = seconds % 60

    pluralMinutes = "s" if minutes != 1 else ""
    pluralSeconds = "s" if remainingSeconds != 1 else ""

    result = ""
    if minutes:
        result += f"{minutes} minuto{pluralMinutes}"

        if remainingSeconds:
            result += " y "

    if remainingSeconds:
        result += f"{remainingSeconds} segundo{pluralSeconds}"

    if not result:
        result = "0 segundos"

    return result

def translate(texto, idioma_origen="es", idioma_destino="en"):
    url = "https://translate.googleapis.com/translate_a/single?client=gtx&sl={}&tl={}&dt=t&q={}".format(idioma_origen, idioma_destino, texto)
    response = requests.get(url)
    traduccion = response.json()[0][0][0]
    
    return traduccion