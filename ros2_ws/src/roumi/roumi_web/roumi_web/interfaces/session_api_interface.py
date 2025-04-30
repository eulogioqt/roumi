from abc import ABC, abstractmethod

from .api_responses import APIResponse


class SessionAPIInterface(ABC):

    @abstractmethod
    def get_all_sessions(self, faceprint_id: str = None) -> APIResponse:
        pass

    @abstractmethod
    def get_session_by_id(self, id: str) -> APIResponse:
        pass