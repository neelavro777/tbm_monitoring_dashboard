import React, { useState, useEffect } from 'react';
import { useROS } from '../contexts/ROSContext';
import { Alert } from 'react-bootstrap';
import ROSLIB from 'roslib';

const TemperatureDisplay = () => {
  const [temperatures, setTemperatures] = useState({});
  const { ros } = useROS();
  const staleThreshold = 5000; // 5 seconds threshold for considering data as stale

  useEffect(() => {
    if (!ros) return;

    const topics = ['/temperature_1', '/temperature_2', '/temperature_3', '/temperature_4', '/temperature_5'];
    topics.forEach(topic => {
      const subscriber = new ROSLIB.Topic({
        ros: ros,
        name: topic,
        messageType: 'std_msgs/Float32'
      });

      subscriber.subscribe((message) => {
        setTemperatures(prevTemps => ({
          ...prevTemps,
          [topic]: {
            value: message.data,
            lastUpdated: Date.now()
          }
        }));
      });
    });

    const checkStaleData = setInterval(() => {
      const currentTime = Date.now();
      setTemperatures(prevTemps => {
        const newTemps = { ...prevTemps };
        Object.keys(newTemps).forEach(topic => {
          if (currentTime - newTemps[topic].lastUpdated > staleThreshold) {
            newTemps[topic] = { ...newTemps[topic], value: 'None' };
          }
        });
        return newTemps;
      });
    }, 1000);

    return () => {
      clearInterval(checkStaleData);
      topics.forEach(topic => {
        const subscriber = new ROSLIB.Topic({ ros, name: topic, messageType: 'std_msgs/Float32' });
        subscriber.unsubscribe();
      });
    };
  }, [ros]);

  return (
    <div className="col-6 col-md-3 mt-4 mx-4">
      <h2 className="text-center mb-4">Temperature Sensors</h2>
      <ul className="list-unstyled">
        {Object.entries(temperatures).map(([topic, { value }]) => {
          let alertVariant = "light";
          if (value !== 'None') {
            if (value > 80) {
              alertVariant = "danger"; 
            } else if (value < 20) {
              alertVariant = "info"; 
            }
          }

          return (
            <li key={topic}>
              <Alert variant={alertVariant} className="d-flex justify-content-between align-items-center">
                <strong>{topic}:</strong> {value === 'None' ? value : `${value.toFixed(2)} °C`}
              </Alert>
            </li>
          );
        })}
      </ul>
    </div>
  );
};

export default TemperatureDisplay;
