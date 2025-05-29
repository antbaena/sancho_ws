import React from "react";
import MapView from "../components/MapView";
import NavStatus from "../components/NavStatus";
import NavGoalForm from "../components/NavGoalForm";

export default function MapPage() {
  return (
    <div className="container my-5">
      <div className="card shadow-sm border-0">
        <div className="card-body">
          <h2 className="card-title mb-4">Mapa y Navegación</h2>
          <div className="row">
            <div className="col-md-8 mb-4">
              <div className="border rounded shadow-sm" style={{ height: "400px" }}>
                <MapView />
              </div>
            </div>
            <div className="col-md-4">
              <NavStatus />
              <NavGoalForm />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
