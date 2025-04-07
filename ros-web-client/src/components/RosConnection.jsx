// src/components/RosConnection.js
import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const RosConnection = () => {
  const [ros, setRos] = useState(null);
  const [message, setMessage] = useState('No message received yet.');

  useEffect(() => {
    // Crear la conexión a ROS
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    rosInstance.on('connection', () => {
      console.log('Connected to websocket server.');
    });

    rosInstance.on('error', (error) => {
      console.error('Error connecting to websocket server: ', error);
    });

    rosInstance.on('close', () => {
      console.log('Connection to websocket server closed.');
    });

    setRos(rosInstance);

    return () => {
      // Cleanup en el desmontaje del componente
      if (rosInstance) {
        rosInstance.close();
      }
    };
  }, []);

  useEffect(() => {
    if (ros) {
      const listener = new ROSLIB.Topic({
        ros: ros,
        name: '/your_topic', // <-- cámbialo a tu tópico, por ejemplo /chatter
        messageType: 'std_msgs/String'
      });

      listener.subscribe((msg) => {
        console.log('Received message: ', msg);
        setMessage(msg.data);
      });

      // Limpieza del listener
      return () => listener.unsubscribe();
    }
  }, [ros]);

  return (
    <div className="p-4">
      <h1 className="text-2xl font-bold mb-4">ROS WebSocket Connection</h1>
      <p>Último mensaje: {message}</p>
    </div>
  );
};

export default RosConnection;
