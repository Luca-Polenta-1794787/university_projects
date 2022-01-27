import sqlite3
import base64
from Item import *




conn = sqlite3.connect('items.db')
print("db opened")

items = conn.execute("SELECT * FROM ITEMS;")

#if you don't stop the reads, you will read all the base64 trash
for i in items:
    print(i[0] + " " + str(i[1]) + " " + str(i[2]) + " " + i[3][:50])

conn.close()