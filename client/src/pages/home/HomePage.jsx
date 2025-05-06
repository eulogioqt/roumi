import React from "react";

const HomePage = () => {
    return (
        <div className="px-5 py-5 mt-5 w-100">
            <div className="text-center mb-5">
                <h1 className="fw-bold display-4">RUMI</h1>
                <p className="text-muted fs-5">
                    Interfaz web para la gestión remota de usuarios en sistemas HRI basados en ROS2 y reconocimiento facial.
                </p>
            </div>

            <div className="mx-auto mb-5 p-4 rounded shadow-sm border bg-white" style={{ maxWidth: "900px" }}>
                <p className="mb-3 text-muted">
                    RUMI es una herramienta diseñada específicamente para facilitar la supervisión y gestión remota de sistemas de interacción humano-robot (HRI) basados en reconocimiento facial. La interfaz permite visualizar y editar la base de datos interna del sistema, incluyendo imágenes de rostros, vectores de embedding, número de interacciones y otros metadatos asociados.
                </p>
                <p className="mb-3 text-muted">
                    También es posible añadir nuevas identidades mediante la carga remota de imágenes etiquetadas, permitiendo preparar el sistema con antelación para el reconocimiento de visitantes o participantes. Esta capacidad mejora la fluidez y naturalidad de las interacciones en escenarios reales.
                </p>
                <p className="mb-0 text-muted">
                    Integrada dentro de arquitecturas basadas en ROS 2, RUMI proporciona una solución robusta, extensible y fácilmente desplegable para la gestión de identidades en sistemas HRI. Su diseño modular y su enfoque práctico la convierten en una herramienta eficaz para investigadores y desarrolladores que trabajan con robots sociales o entornos interactivos.
                </p>
            </div>

            <div className="mx-auto text-center" style={{ maxWidth: "800px" }}>
                <h5 className="mb-3 text-secondary">Características principales</h5>
                <ul className="list-unstyled text-start mx-auto" style={{ maxWidth: "600px" }}>
                    <li className="mb-2">• Visualización y edición de imágenes de rostros y vectores de embedding</li>
                    <li className="mb-2">• Gestión de identidades con carga remota de imágenes etiquetadas</li>
                    <li className="mb-2">• Consulta de metadatos como número de interacciones y sesiones activas</li>
                    <li className="mb-2">• Integración nativa con sistemas ROS 2</li>
                    <li className="mb-2">• Interfaz sencilla, accesible y preparada para entornos reales</li>
                </ul>
            </div>
        </div>
    );
};

export default HomePage;
