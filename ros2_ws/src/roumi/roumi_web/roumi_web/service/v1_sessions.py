from typing import Optional, List
from fastapi import APIRouter, HTTPException, Query, Request, Path

from .session_model import Session
from .api_utils import APIUtils
from ..interfaces import SessionAPIInterface

from dotenv import load_dotenv

load_dotenv()

router = APIRouter()

endpoint_name = "sessions"
version = "v1"

session_interface: SessionAPIInterface = None
def set_session_api(interface):
    global session_interface
    session_interface = interface

@router.get("", tags=["Sessions CRUD endpoints"], response_model=List[Session])
async def get_sessions(
    request: Request,
    faceprint_id: Optional[str] = Query(None, description="ID de un faceprint"),
    fields: Optional[str] = Query(None, description="Campos específicos a devolver"),
    sort: Optional[str] = Query(None, description="Campos por los que ordenar, separados por comas"),
    offset: int = Query(default=0, description="Índice de inicio para los resultados de la paginación"),
    limit: int = Query(default=10, description="Cantidad de marcadores a devolver, por defecto 10"),
    hateoas: Optional[bool] = Query(default=False, description="Incluir enlace HATEOAS")
):
    APIUtils.check_accept_json(request)

    try:
        response = session_interface.get_all_sessions(**({ "faceprint_id": faceprint_id} if faceprint_id else {}))
        
        return response.to_fastapi()
    
    except HTTPException as e:
        raise e 
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al buscar las sesiones: {str(e)}")

@router.get("/{id}", tags=["Sessions CRUD endpoints"], response_model=Session)
async def get_sessions_by_faceprint_id(
    request: Request,
    id: str = Path(description="Id de la sesión"),
    fields: Optional[str] = Query(None, description="Campos específicos a devolver"),
    sort: Optional[str] = Query(None, description="Campos por los que ordenar, separados por comas"),
    offset: int = Query(default=0, description="Índice de inicio para los resultados de la paginación"),
    limit: int = Query(default=10, description="Cantidad de marcadores a devolver, por defecto 10"),
    hateoas: Optional[bool] = Query(default=False, description="Incluir enlace HATEOAS")
):
    APIUtils.check_accept_json(request)
    
    try:
        response = session_interface.get_session_by_id(id)

        return response.to_fastapi()
    
    except HTTPException as e:
        raise e 
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al obtener la sesión: {str(e)}")
