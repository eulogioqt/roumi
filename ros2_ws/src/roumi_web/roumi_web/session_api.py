import json

from .client_node import ClientNode
from .interfaces import SessionAPIInterface, JSONResponse


class SessionAPI(SessionAPIInterface):

    def __init__(self, node: ClientNode):
        self.node = node

    def get_all_sessions(self, faceprint_id=None):
        if faceprint_id is not None:
            sessions_json = self.node.get_sessions_request(json.dumps({ "faceprint_id": faceprint_id }))
        else:
            sessions_json = self.node.get_sessions_request()

        sessions = json.loads(sessions_json)

        return JSONResponse(content=sessions)

    def get_session_by_id(self, id):
        session_json = self.node.get_sessions_request(json.dumps({ "id": id }))
        session = json.loads(session_json)

        return JSONResponse(content=session)
