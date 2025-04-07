import json

from ..api.utils import normalized_cosine_similarity_distance
from ..database.database_manager import EmbeddedDatabase

# make_complex_classifier an Object that extends from a classifier interface or abstract object
# Nota: el size es necesario porque lelva la cuenta de cuantos refinamientos tiene cada vector especifico

# Hacer un sistema de mensajes con json para enviar parametros y para devolver informacion de la ejecucion de los comandos tanto para el panel de control
# como para python

# para enviar cmd y args en json o algo asi

class ComplexClassifier:

    def __init__(self, use_database = False):
        '''Inits classifier

        Args:
            use_database (str): If not True, uses and stores new data into database.
        '''

        self.use_database = use_database
        if self.use_database:
            self.database = EmbeddedDatabase("database.db")
            self.load()
    
        self.people = {}
        self.size = {}

    def save(self):
        '''Saves the learned data to a file.'''

        if self.use_database:
            self.database.add_users_from_dictionary(self.people, self.size)

    def load(self):
        '''Loads the learned data from a file.'''
        
        if self.use_database:
            self.people, self.size = self.database.get_all_users_as_dictionary()
        
        self.print_people()

    def get_people(self):
        '''Get all people names.
        
        Returns:
            str: JSON Array with all people names.
        '''

        return json.dumps(list(self.people.keys()))

    def print_people(self):
        '''Prints the names of the existing people.'''
        
        print("Personas que conozco:")
        for key in self.people:
            print(key)

    def classify_face(self, new_features):
        '''Given a feature vector, gives the closest class.
        
        Args:
            new_features (Array: float): Feature vector.
        
        Returns:
            closest_class (str): The name of the closest class.
            closes_distance (float): The normalized cosine distance to the closest_class.
            position (int): The position of the vector in the array of vectors of the class that
                was the closest to the given vector.
        '''

        closest_class = None
        closest_distance = 0
        position = 0

        if not self.people:
            return closest_class, closest_distance, position

        for class_name, feature_list in self.people.items():
            for i in range(0, len(feature_list)):
                distance = normalized_cosine_similarity_distance(new_features, feature_list[i])

                if distance > closest_distance:
                    closest_distance = distance
                    closest_class = class_name
                    position = i

        return closest_class, closest_distance, position

    def refine_class(self, class_name, features, position):
        '''Makes a certain feature vector more precise by averaging with a new feature vector.

        Args:
            class_name (str): The class.
            features (Array: float): The new feature vector.
            position (int): The position of the known feature vector we want to make more precise.
        '''

        new_size = self.size[class_name][position] + 1
        self.size[class_name][position] = new_size

        self.people[class_name][position] = [(x * (new_size - 1)+ y) / new_size for x, y in 
                                             zip(self.people[class_name][position], features)]

        result = 1
        message = "La clase " + class_name + " ha sido refinada"

        return result, message

    def add_features(self, class_name, features):
        '''Adds a new feature vector to the array of vectors that describes a class.
        
        Args:
            class_name (str): The class.
            features (Array: float): The new feature vector.
        '''

        self.people[class_name].append(features)
        self.size[class_name].append(1)

        result = 1
        message = ("La clase " + class_name + " ahora tiene " + 
            str(len(self.people[class_name])) + " vectores de características independientes.")

        return result, message

    def add_class(self, class_name, features):
        '''Adds a new class with a unique feature vector.

        Args:
            class_name (str): The class.
            features (Array: float): The feature vector.
        '''

        already_known = class_name in self.people

        if already_known:
            _, message = self.add_features(class_name, features)
        else:
            self.people[class_name] = [features]
            self.size[class_name] = [1]
            message = "Nueva clase aprendida: " + class_name

        result = int(already_known)

        return result, message

    def rename_class(self, old_name, new_name):
        '''Renames a class.

        Args:
            old_name (str): The actual class.
            new_name (str): The new class name.
        '''

        if old_name not in self.people or old_name not in self.size:
            result = 0 
            message = "La clase " + old_name + " no existe."
        elif new_name in self.people or new_name in self.size:
            result = 0 
            message = "La clase " + new_name + " ya existe."
        else:
            self.people[new_name] = self.people.pop(old_name)
            self.size[new_name] = self.size.pop(old_name)
        
            result = 1
            message = "La clase " + old_name + " ha sido renombrada a " + new_name

        return result, message

    def delete_class(self, class_name):
        '''Removes a class.
        
        Args:
            class_name (str): The class.
        '''
        
        if class_name in self.people:
            self.people.pop(class_name)
            self.size.pop(class_name)

            result = 1
            message = "La clase " + class_name + " ha sido eliminada correctamente"
        
        else:
            result = -1
            message = "La clase " + class_name + " no existe"

        return result, message