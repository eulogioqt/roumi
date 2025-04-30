from fastapi import FastAPI
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

from .v1_faceprints import router as v1_faceprints
from .v1_sessions import router as v1_sessions

load_dotenv()

app = FastAPI(
    docs_url="/api/hri/docs",  # Documentación Swagger
    redoc_url="/api/hri/redoc",  # Documentación Redoc
    openapi_url="/api/hri/openapi.json"  # OpenAPI Schema
)
app.title = "HRI Service"
app.version = "1.0.0"
app.add_middleware(GZipMiddleware, minimum_size=1000)
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:5173"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(v1_faceprints, prefix="/api/v1/faceprints")
app.include_router(v1_sessions, prefix="/api/v1/sessions")