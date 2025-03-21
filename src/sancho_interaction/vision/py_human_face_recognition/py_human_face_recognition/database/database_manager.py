import json
import sqlite3
import datetime


class EmbeddedDatabase:
    def __init__(self, db_name):
        self.connection = sqlite3.connect(db_name)
        self.cursor = self.connection.cursor()
        self.create_tables()

    def create_tables(self):
        self.cursor.execute(
            """CREATE TABLE IF NOT EXISTS users (
                                id INTEGER PRIMARY KEY,
                                name TEXT,
                                characteristics TEXT,
                                size TEXT,
                                learning_date DATE,
                                rank_id INTEGER,
                                FOREIGN KEY (rank_id) REFERENCES ranks(id))"""
        )

        self.cursor.execute(
            """CREATE TABLE IF NOT EXISTS ranks (
                                id INTEGER PRIMARY KEY,
                                description TEXT)"""
        )
        self.cursor.execute(
            "INSERT OR IGNORE INTO ranks (description) VALUES ('Persistent')"
        )
        self.cursor.execute(
            "INSERT OR IGNORE INTO ranks (description) VALUES ('Ephemeral')"
        )
        self.connection.commit()

    def add_user(self, name, characteristics, size):
        learning_date = datetime.date.today().isoformat()
        self.cursor.execute(
            """SELECT id FROM ranks WHERE UPPER(description) = UPPER(?)""",
            ("Ephemeral",),
        )
        rank_id = self.cursor.fetchone()[
            0
        ]  # Obtener el primer user de la tupla resultante
        self.cursor.execute(
            """INSERT INTO users (name, characteristics, size, learning_date, rank_id) 
                            VALUES (?, ?, ?, ?, ?)""",
            (name, characteristics, size, learning_date, rank_id),
        )
        self.connection.commit()

    def update_user(self, name, new_characteristics, new_size):
        self.cursor.execute(
            """UPDATE users 
            SET characteristics = ?, size = ?
            WHERE name = ?""",
            (new_characteristics,new_size, name),
        )
        self.connection.commit()

    def remove_user(self, name):
        self.cursor.execute(
            """DELETE FROM users 
            WHERE name = ?""",
            (name,),
        )
        self.connection.commit()

    def get_name_by_characteristics(self, characteristics):
        self.cursor.execute(
            """SELECT name FROM users WHERE characteristics = ?""",
            (characteristics,),
        )
        result = self.cursor.fetchone()
        return result[0] if result else None

    def get_characteristics_by_name(self, name):
        self.cursor.execute(
            """SELECT characteristics FROM users WHERE name = ?""", (name,)
        )
        result = self.cursor.fetchone()
        return result[0] if result else None

    def get_user_by_name(self, name):
        self.cursor.execute("""SELECT * FROM users WHERE name = ?""", (name,))
        return self.cursor.fetchone()

    def get_user_by_characteristics(self, characteristics):
        self.cursor.execute(
            """SELECT * FROM users WHERE characteristics = ?""", (characteristics,)
        )
        return self.cursor.fetchone()

    def get_all_users(self):
        self.cursor.execute("""SELECT * FROM users""")
        return self.cursor.fetchall()

    def add_users_from_dictionary(self, users_dict_characteristics, users_dict_size):
        for name, characteristics in users_dict_characteristics.items():
            size = users_dict_size.get(name)
            characteristics_json = json.dumps(characteristics)
            size_json = json.dumps(size)
            # Verificar si el nombre ya existe en la base de datos
            existing_user = self.get_user_by_name(name)
            if existing_user:
                # Si el nombre existe, actualizar las características
                self.update_user(name, characteristics_json, size_json)
            else:
                # Si el nombre no existe, agregar el usero
                self.add_user(name, characteristics_json,size_json)

    def get_all_users_as_dictionary(self):
        users_dict_characteristics = {}
        users_dict_size = {}
        users = self.get_all_users()
        for user in users:
            name, characteristics_json, size_json = user[1], user[2], user[3]
            characteristics = json.loads(characteristics_json)
            size = json.loads(size_json)
            users_dict_characteristics[name] = characteristics
            users_dict_size[name] = size
            
        return users_dict_characteristics,users_dict_size

    def make_user_persistent(self, user_id):
        # Obtener el id del rango "Persistent"
        self.cursor.execute(
            """SELECT id FROM ranks WHERE UPPER(description) = UPPER(?)""",
            ("Persistent",),
        )
        persistent_rank_id = self.cursor.fetchone()[0]

        # Actualizar el usuario para que tenga el rango "Persistent"
        self.cursor.execute(
            """UPDATE users 
                SET rank_id = ?
                WHERE id = ?""",
            (persistent_rank_id, user_id),
        )
        self.connection.commit()

    def make_user_ephemeral(self, user_id):
        # Obtener el id del rango "Ephemeral"
        self.cursor.execute(
            """SELECT id FROM ranks WHERE UPPER(description) = UPPER(?)""",
            ("Ephemeral",),
        )
        ephemeral_rank_id = self.cursor.fetchone()[0]

        # Actualizar el usuario para que tenga el rango "Ephemeral"
        self.cursor.execute(
            """UPDATE users 
                SET rank_id = ?
                WHERE id = ?""",
            (ephemeral_rank_id, user_id),
        )
        self.connection.commit()

    def close_connection(self):
        self.connection.close()


if __name__ == "__main__":
    # Create an instance of the database
    db_name = "database.db"
    db = EmbeddedDatabase(db_name)

    # Add users
    db.add_user("user1", "Characteristics of user 1","1321")
    db.add_user("user2", "Characteristics of user 2","123123")
    db.add_user("user3", "Characteristics of user 3","1323123")

    # Update an user
    db.update_user("user1", "New characteristics of user 1","9999999")

    # Remove an user
    db.remove_user("user2")

    # Get all users
    users = db.get_all_users_as_dictionary()
    print("All users:", users)

    # Close the connection
    db.close_connection()
