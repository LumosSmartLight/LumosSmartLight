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
        intensity = result['intensity']
        return json.dumps(result)

    @app.route('/api/setconfig', methods=['PUT'])
    def setConfig():
        new_intensity = request['intensity']
        new_ison = request['ison']
        new_mode = request['mode']
        

        new_config = {
            'intensity': str(new_intensity),
            'ison': str(new_ison),
            'new_mode': str(new_mode)
        }

        response = firebase.put('/config', new_config, None, None)
        return response
        


#class TodoSimple(Resource):
#    def get(self, todo_id):
#        return {todo_id: todos[todo_id]}
#
#    def put(self, todo_id):
#        todos[todo_id] = request.form['data']
#        return {todo_id: todos[todo_id]}

#api.add_resource(TodoSimple, '/<string:todo_id>')
api.add_resource(LumosAPI, '/')

if __name__ == '__main__':
    app.run(host='0.0.0.0',debug=True) #Change host to internal IP
    #app.run(debug=True)

