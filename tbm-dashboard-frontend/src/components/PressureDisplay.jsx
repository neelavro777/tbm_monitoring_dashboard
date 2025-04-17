import React, { useState, useEffect } from 'react';
import { useROS } from '../contexts/ROSContext';
import { Alert } from 'react-bootstrap';
import ROSLIB from 'roslib';

const PressureDisplay = () => {
  const [pressures, setPressures] = useState({});
  const { ros } = useROS();
  const staleThreshold = 5000; // 5 seconds threshold for considering data as stale

  useEffect(() => {
    if (!ros) return;

    const topics = ['/pressure_1', '/pressure_2', '/pressure_3', '/pressure_4', '/pressure_5'];
    topics.forEach(topic => {
      const subscriber = new ROSLIB.Topic({
        ros: ros,
        name: topic,
        messageType: 'std_msgs/Float32'
      });

      subscriber.subscribe((message) => {
        setPressures(prevPressures => ({
          ...prevPressures,
          [topic]: {
            value: message.data,
            lastUpdated: Date.now()
          }
        }));
      });
    });

    const checkStaleData = setInterval(() => {
      const currentTime = Date.now();
      setPressures(prevPressures => {
        const newPressures = { ...prevPressures };
        Object.keys(newPressures).forEach(topic => {
          if (currentTime - newPressures[topic].lastUpdated > staleThreshold) {
            newPressures[topic] = { ...newPressures[topic], value: 'None' };
          }
        });
        return newPressures;
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
      <h2 className="text-center mb-4">Pressure Sensors</h2>
      <ul className="list-unstyled">
        {Object.entries(pressures).map(([topic, { value }]) => {
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
                <strong>{topic}:</strong> {value === 'None' ? value : `${value.toFixed(2)} PSI`}
              </Alert>
            </li>
          );
        })}
      </ul>
    </div>
  );
};

export default PressureDisplay;
