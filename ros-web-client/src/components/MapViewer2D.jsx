// src/components/MapViewer2D.js
import React, { useEffect, useRef } from 'react';
import ROSLIB from 'roslib';


const MapViewer2D = () => {
  const viewerRef = useRef(null);

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: 'ws://192.168.88.252:9090'  // <-- Cambia tu IP aquí
    });

    ros.on('connection', () => {
      console.log('Conectado a rosbridge.');
    });

    ros.on('error', (error) => {
      console.error('Error conectando a rosbridge: ', error);
    });

    ros.on('close', () => {
      console.log('Conexión cerrada.');
    });

    // Crear el viewer
    const viewer = new ROS2D.Viewer({
      divID: viewerRef.current.id,
      width: 600,
      height: 600
    });

    // Mostrar el mapa usando OccupancyGridClient
    const gridClient = new ROS2D.OccupancyGridClient({
      ros: ros,
      rootObject: viewer.scene,
      continuous: true, // true para seguir recibiendo updates
      topic: '/map'
    });

    // Cuando recibimos el mapa, hacer fit automático
    gridClient.on('change', () => {
      viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
      viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });

  }, []);

  return (
    <div className="p-4">
      <h2 className="text-xl font-bold mb-4">🗺️ Mapa usando ros2djs</h2>
      <div id="map-canvas" ref={viewerRef} style={{ width: '600px', height: '600px', border: '1px solid black' }} />
    </div>
  );
};

export default MapViewer2D;
