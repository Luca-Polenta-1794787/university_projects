import sqlite3
import base64
from Item import *





def getImg(name):
    with open('/home/Heinzeen/mysite/' + name + '.jpg', mode='rb') as file:
        img = file.read()
    return str(base64.b64encode(img))[2:-1]

items = []
I1 = Item("bundle", 19.99, 34, getImg("bundle"))
items.append(I1)
I2 = Item("original", 12.99, 20, getImg("original"))
items.append(I2)
I3 = Item("pop_trooper", 29.90, 10, getImg("pop_trooper"))
items.append(I3)
I4 = Item("pop_vader", 29.90, 10, getImg("pop_vader"))
items.append(I4)
I5 = Item("pop_yoda", 29.90, 10, getImg("pop_yoda"))
items.append(I5)
I6 = Item("prequel", 13.99, 19, getImg("prequel"))
items.append(I6)
I7 = Item("shirt_trooper", 15, 30, getImg("shirt_trooper"))
items.append(I7)
I8 = Item("shirt_vader", 15, 30, getImg("shirt_vader"))
items.append(I8)
I9 = Item("shirt_yoda", 15, 30, getImg("shirt_yoda"))
items.append(I9)
I10 = Item("small_Yoda", 4.99, 55, getImg("small_Yoda"))
items.append(I10)


conn = sqlite3.connect('items.db')
print("db opened")

conn.execute("DROP TABLE ITEMS;")
conn.execute("CREATE TABLE ITEMS (name text primary key not null, cost real not null, quantity int not null, img text not null);")

for i in items:
    conn.execute("INSERT INTO ITEMS (name, cost, quantity, img) VALUES ('"+ i.name + "', " + str(i.cost) + ", " + str(i.quantity) + ", '" + i.img + "')");

conn.commit()
conn.close()