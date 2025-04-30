import { createRoot } from "react-dom/client";
import App from "./App.jsx";

import { ToastProvider } from "./contexts/ToastContext.jsx";
import { EventBusProvider } from "./contexts/EventBusContext.jsx";
import { APIProvider } from "./contexts/APIContext.jsx";
import { WebSocketProvider } from "./contexts/WebSocketContext.jsx";
import { FaceprintsProvider } from "./contexts/FaceprintsContext.jsx";

import "bootstrap/dist/css/bootstrap.min.css";
import "bootstrap/dist/js/bootstrap.bundle.min.js";
import "bootstrap-icons/font/bootstrap-icons.css";
import "./css/main.css";

createRoot(document.getElementById("root")).render(
    <ToastProvider>
        <EventBusProvider>
            <APIProvider>
                <WebSocketProvider>
                    <FaceprintsProvider>
                        <App />
                    </FaceprintsProvider>
                </WebSocketProvider>
            </APIProvider>
        </EventBusProvider>
    </ToastProvider>
);
