import { useContext, createContext, useState, useEffect } from "react";
import axios from "axios";

const APIContext = createContext();

const BASE_URL = import.meta.env.VITE_BACKEND_URL || "http://localhost:7654";
console.log("BASE_URL:", BASE_URL);

export const APIProvider = ({ children }) => {
    const [loadCount, setLoadCount] = useState(0);
    const [authToken, setAuthToken] = useState(undefined);

    const setLoading = (val) =>
        setLoadCount((oldCount) => {
            const newCount = oldCount + (val ? 1 : -1);
            return newCount < 0 ? 0 : newCount;
        });

    useEffect(() => {
        // document.getElementById("loading-screen").style.display = loadCount === 0 ? "none" : "block";
    }, [loadCount]);

    const getAuthorizationHeader = () => (authToken ? { Authorization: `Bearer ${authToken}` } : {});

    const requestHandler = async (method, url, data = undefined, headers = {}) => {
        console.log(`${method.toUpperCase()}: ${url}`);
        if (data) console.log("Body", data);

        setLoading(true);

        try {
            const response = await axios({ method, url, data, headers });
            setLoading(false);
            return response;
        } catch (error) {
            console.log("Error en la peticion (error, data):", error.response, data);
            setLoading(false);
            return error.response;
        }
    };

    const apiMethods = {
        get: (url, headers = undefined) => requestHandler("get", url, undefined, headers),
        post: (url, data, headers = undefined) => requestHandler("post", url, data, headers),
        put: (url, data, headers = undefined) => requestHandler("put", url, data, headers),
        delete: (url, headers = undefined) => requestHandler("delete", url, undefined, headers),
    };

    const isResponseOk = (response) => response.status >= 200 && response.status < 300;

    const createEndpointMethods = (entity, extraEndpoints) => ({
        getAll: (params = "", version = "v1") =>
            apiMethods.get(`${BASE_URL}/api/${version}/${entity}${params}`, getAuthorizationHeader()),
        getById: (id, params = "", version = "v1") =>
            apiMethods.get(`${BASE_URL}/api/${version}/${entity}/${id}${params}`, getAuthorizationHeader()),
        create: (body, version = "v1") =>
            apiMethods.post(`${BASE_URL}/api/${version}/${entity}`, body, getAuthorizationHeader()),
        update: (id, body, version = "v1") =>
            apiMethods.put(`${BASE_URL}/api/${version}/${entity}/${id}`, body, getAuthorizationHeader()),
        delete: (id, version = "v1") =>
            apiMethods.delete(`${BASE_URL}/api/${version}/${entity}/${id}`, getAuthorizationHeader()),
        ...extraEndpoints,
    });

    const faceprintsAPI = createEndpointMethods("faceprints");
    const sessionsAPI = createEndpointMethods("sessions", {
        create: () => alert("Not implemented"),
        update: () => alert("Not implemented"),
        delete: () => alert("Not implemented"),
    })

    return (
        <APIContext.Provider
            value={{
                faceprints: faceprintsAPI,
                sessions: sessionsAPI,

                isResponseOk: isResponseOk,
                setAuthToken: setAuthToken,
                axios: apiMethods,
            }}
        >
            {children}
        </APIContext.Provider>
    );
};

export const useAPI = () => useContext(APIContext);
