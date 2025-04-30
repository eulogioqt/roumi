from abc import ABC, abstractmethod

from .api_responses import APIResponse


class FaceprintAPIInterface(ABC):

    @abstractmethod
    def get_all_faceprints(self) -> APIResponse:
        pass

    @abstractmethod
    def get_faceprint(self, id: str) -> APIResponse:
        pass

    @abstractmethod
    def create_faceprint(self, name: str, image_base64: str) -> APIResponse:
        pass

    @abstractmethod
    def update_faceprint(self, id: str, faceprint: str) -> APIResponse:
        pass

    @abstractmethod
    def delete_faceprint(self, id: str) -> APIResponse:
        pass