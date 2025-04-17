// src/contexts/ROSContext.js
import React, { createContext, useState, useEffect, useContext, useRef } from 'react';
import ROSLIB from 'roslib';

const ROSContext = createContext();

export const useROS = () => {
  return useContext(ROSContext);
};

export const ROSProvider = ({ children }) => {
  const [connected, setConnected] = useState(false);
  const rosRef = useRef(null);

  useEffect(() => {
    const connectToROS = () => {
      const ros = new ROSLIB.Ros({
        url: 'ws://192.168.0.191:9090'
      });

      ros.on('connection', () => {
        console.log('Connected to websocket server.');
        setConnected(true);
        rosRef.current = ros;
      });

      ros.on('error', (error) => {
        console.log('Error connecting to websocket server: ', error);
        setConnected(false);
        setTimeout(connectToROS, 5000); // Attempt to reconnect every 5 seconds
      });

      ros.on('close', () => {
        console.log('Connection to websocket server closed.');
        setConnected(false);
        setTimeout(connectToROS, 5000); // Attempt to reconnect every 5 seconds
      });

      return ros;
    };

    const ros = connectToROS();

    return () => {
      if (ros) {
        ros.close();
      }
    };
  }, []);

  return (
    <ROSContext.Provider value={{ ros: rosRef.current, connected }}>
      {children}
    </ROSContext.Provider>
  );
};
