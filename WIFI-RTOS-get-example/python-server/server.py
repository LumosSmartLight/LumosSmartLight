from flask import Flask, request
from flask_restful import Resource, Api
from firebase import firebase
import json

app = Flask(__name__)
api = Api(app)
firebase = firebase.FirebaseApplication("https://lumossmartlight.firebaseio.com/", None)

class LumosAPI(Resource):
    @app.route('/api/getconfig', methods=['GET'])
    def getConfig():
        result = firebase.get('/config', None)
        print(result)
        return json.dumps(result)

    @app.route('/api/sendDht', methods=['GET'])
    #URL Example: http://localhost:5000/api/sendDht?temp=25.3&hum=53.7
    def sendDHT():
        temperature = request.args.get('temp', default = 1.0, type = float)
        humidity = request.args.get('hum', default = 1.0, type = float)
        print(temperature, humidity)
        firebase.post('/temperature', temperature)
        firebase.post('/humidity', humidity)
        return "new temperature and humidity data sent"
        
api.add_resource(LumosAPI, '/')

if __name__ == '__main__':
    app.run(host='0.0.0.0',debug=True) #Change host to internal IP

