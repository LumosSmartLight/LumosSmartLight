from flask import Flask, request
from flask_restful import Resource, Api
from firebase import firebase
import json

app = Flask(__name__)
api = Api(app)
firebase = firebase.FirebaseApplication("https://lumossmartlight.firebaseio.com/", None)

class LumosAPI(Resource):
    @app.route('/api/config', methods=['GET'])
    def getConfig():
        result = firebase.get('/config', None)
        print(result)
        return json.dumps(result)

    # @app.route('/api/config', method=['POST'])
    # def setConfig():


#class TodoSimple(Resource):
#    def get(self, todo_id):
#        return {todo_id: todos[todo_id]}
#
#    def put(self, todo_id):
#        todos[todo_id] = request.form['data']
#        return {todo_id: todos[todo_id]}

#api.add_resource(TodoSimple, '/<string:todo_id>')
api.add_resource(LumosAPI, '/api/config')

if __name__ == '__main__':
    app.run(host='10.92.72.181',debug=True) #Change host do user IP
    #app.run(debug=True)

