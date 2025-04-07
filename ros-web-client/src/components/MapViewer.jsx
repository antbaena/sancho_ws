// src/components/MapViewer.js
import React, { useRef, useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const MapViewer = () => {
  const canvasRef = useRef(null);
  const [ros, setRos] = useState(null);

  useEffect(() => {
    // Conectar a ROS
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://192.168.88.252:9090'  // <-- Cambia a tu IP ROSBRIDGE si es otra
    });

    rosInstance.on('connection', () => {
      console.log('Conectado a rosbridge websocket server.');
    });

    rosInstance.on('error', (error) => {
      console.error('Error de conexión: ', error);
    });

    rosInstance.on('close', () => {
      console.log('Conexión cerrada.');
    });

    setRos(rosInstance);

    return () => {
      if (rosInstance) rosInstance.close();
    };
  }, []);

  useEffect(() => {
    if (ros) {
      const mapListener = new ROSLIB.Topic({
        ros: ros,
        name: '/map',
        messageType: 'nav_msgs/OccupancyGrid'
      });

      mapListener.subscribe((message) => {
        drawMap(message);
      });

      return () => {
        mapListener.unsubscribe();
      };
    }
  }, [ros]);

  const drawMap = (message) => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    const width = message.info.width;
    const height = message.info.height;
    const data = message.data;

    // Ajustar tamaño del canvas
    canvas.width = width;
    canvas.height = height;

    const imgData = ctx.createImageData(width, height);

    for (let i = 0; i < data.length; i++) {
      const value = data[i];
      const pixelIndex = i * 4;

      let color = 255; // Default: unknown (grey)

      if (value === 100) { 
        color = 0;   // Wall: black
      } else if (value === 0) { 
        color = 255; // Free: white
      }

      imgData.data[pixelIndex] = color;     // R
      imgData.data[pixelIndex + 1] = color; // G
      imgData.data[pixelIndex + 2] = color; // B
      imgData.data[pixelIndex + 3] = 255;   // A
    }

    ctx.putImageData(imgData, 0, 0);
  };

  return (
    <div className="p-4">
      <h2 className="text-xl font-bold mb-4">🗺️ Mapa del robot</h2>
      <canvas ref={canvasRef} style={{ border: '1px solid black' }} />
    </div>
  );
};

export default MapViewer;
