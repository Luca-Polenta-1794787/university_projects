from flask import Flask, jsonify, json
from flask_restful import Resource, Api, reqparse
import random
from Item import *
import base64
import sqlite3
import threading


sem = threading.Semaphore()




def getImg(name):
    with open('/home/Heinzeen/mysite/' + name + '.jpg', mode='rb') as file:
        img = file.read()
    return str(base64.b64encode(img))[2:-1]

def query_all():
    items = []
    conn = sqlite3.connect('/home/Heinzeen/mysite/items.db')

    rows = conn.execute("select * from items;")

    for i in rows:
        I1 = Item(i[0], i[1], i[2], i[3])
        items.append(I1)

    conn.close()
    return items

def check_db(name, quantity):
    items = query_all()
    for i in items:
        if i.name == name:
            if quantity <= i.quantity:
                return 0
            else:
                return 1

def exec_query(name, quantity):
    items = []
    conn = sqlite3.connect('/home/Heinzeen/mysite/items.db')

    #we only have 1 item in vals, but for simplicity we still loop
    vals = conn.execute("select quantity from items where name = '" + name + "';")
    for q in vals:
        conn.execute("update items set quantity = " + str(q[0] - quantity) + " where name = '" + name + "';")

    conn.commit()

    conn.close()
    return items


def execute_order(order):
    orderlist = order.split("+")[1:]
    sem.acquire()
    for i in orderlist:
        name = i.split("-")[0]
        quantity = int(i.split("-")[1])
        if (check_db(name, quantity) != 0):
            sem.release()
            return 1

    for i in orderlist:
        name = i.split("-")[0]
        quantity = int(i.split("-")[1])
        exec_query(name, quantity)

    sem.release()
    return 0


class MyJSONEncoder(json.JSONEncoder):

    def default(self, obj):
        if isinstance(obj, Item):
            res = {
                "name" : obj.name,
                "cost" : obj.cost,
                "quantity" : obj.quantity,
                "img" : obj.img
                }
            return res
        return super(MyJSONEncoder, self).default(obj)


app = Flask(__name__)
app.json_encoder = MyJSONEncoder
api = Api(app)


parser = reqparse.RequestParser()



class myHUB(Resource):

    def get(self):
        print("Received GET!")
        #parser.add_argument('ID',type=int,required=True)
        #args = parser.parse_args()
        #id = args['ID']
        #print("request with id: " + str(id))
        #if id == 1:
        #    reply_msg['lat']=str(lat2)
        #    reply_msg['lon']=str(lon2)
        items = query_all()
        return jsonify(items)

    def post(self):
        print("Received POST!")
        parser.add_argument('order',type=str,required=True)
        args = parser.parse_args()
        id = args['order']
        print("received order =" + str(id))
        if execute_order(id) != 0:
            return jsonify("TOO many objects")
        else:
            return jsonify("ORDER ok")

api.add_resource(myHUB,'/')

if __name__ == '__main__':

    print('starting myHUB api...waiting')
    #app.run(host='0.0.0.0')
