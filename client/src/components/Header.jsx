import React from "react";
import { useNavigate } from "react-router-dom";

const Header = () => {
    const navigate = useNavigate();

    return (
        <div className="d-flex justify-content-start align-items-center position-absolute w-100 bg-primary top-0">
            <span className="text-white my-1 fw-bold fs-3 ms-5">RUMI</span>
            <span className="text-white my-1 fw-bold fs-5 ms-3" onClick={() => navigate("/")}>
                Home
            </span>
            <span className="text-white my-1 fw-bold fs-5 ms-3" onClick={() => navigate("/faceprints")}>
                Faceprints
            </span>
        </div>
    );
};

export default Header;
