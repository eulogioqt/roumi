import React, { useState } from "react";

import { useFaceprints } from "../../contexts/FaceprintsContext";

import NewFaceprintModal from "./components/NewFaceprintModal";
import FaceprintList from "./components/FaceprintList";

const FaceprintListPage = () => {
    const { doAddFaceprint, fetchFaceprintsData, loadingFaceprints, faceprintsData } = useFaceprints();

    const [isOpenFaceModal, setIsOpenFaceModal] = useState(false);

    return (
        <>
            <NewFaceprintModal
                isOpen={isOpenFaceModal}
                handleClose={() => setIsOpenFaceModal(false)}
                doAddFaceprint={doAddFaceprint}
            />

            <div className="container mt-5">
                <div className="d-flex justify-content-between align-items-center mb-4 pt-4">
                    <h2>Base de Datos de Rostros Reconocidos</h2>
                    <div>
                        <button className="btn btn-primary me-2" onClick={() => setIsOpenFaceModal(true)}>
                            <i className="bi bi-database-add me-2" /> Añadir cara
                        </button>
                        <button className="btn btn-outline-secondary" onClick={fetchFaceprintsData}>
                            <i className="bi bi-arrow-clockwise me-2" /> Recargar
                        </button>
                    </div>
                </div>

                {loadingFaceprints ? (
                    <div className="text-center py-5">
                        <div className="spinner-border text-primary" role="status" />
                    </div>
                ) : (
                    <FaceprintList faceprints={faceprintsData} />
                )}
            </div>
        </>
    );
};

export default FaceprintListPage;
