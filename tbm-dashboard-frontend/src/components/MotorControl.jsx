import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import { useROS } from '../contexts/ROSContext';

const MotorControl = () => {
  const { ros, connected } = useROS();
  const [alertMessage, setAlertMessage] = useState('');
  const [shutdownReason, setShutdownReason] = useState('');

  useEffect(() => {
    if (!ros) return;

    const reasonTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/shutdown_reason',
      messageType: 'std_msgs/String'
    });

    reasonTopic.subscribe((message) => {
      setShutdownReason(message.data);
    });

    return () => {
      reasonTopic.unsubscribe();
    };
  }, [ros]);

  const sendMotorCommand = (turnOn) => {
    if (!connected) {
      setAlertMessage('Cannot send command. Disconnected from ROS.');
      return;
    }

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/web_motor_control',
      messageType: 'std_msgs/Bool'
    });

    const message = new ROSLIB.Message({
      data: turnOn
    });

    topic.publish(message);
    setAlertMessage(`Motor command to turn ${turnOn ? 'ON' : 'OFF'} sent.`);
  };

  return (
    <div className="col-6 col-md-3 mt-4 mx-4">
      {alertMessage && <div className="alert alert-warning">{alertMessage}</div>}
      {shutdownReason && <div className="alert alert-info">{shutdownReason}</div>}
      <button onClick={() => sendMotorCommand(true)} className="btn btn-primary me-2">Turn On Motor</button>
      <button onClick={() => sendMotorCommand(false)} className="btn btn-secondary">Turn Off Motor</button>
    </div>
  );
};

export default MotorControl;
