import React from 'react';
import TemperatureDisplay from '../components/TemperatureDisplay';
import PressureDisplay from '../components/PressureDisplay';
import MotorControl from '../components/MotorControl';
import CameraFeed from '../components/CameraFeed';
import { useROS } from '../contexts/ROSContext';

const HomePage = () => {
  const { connected } = useROS();

  return (
    <div className="App">
      <div className='mt-3'>
        <h1 className='text-center'>TBM Monitoring Dashboard</h1>
        <div className="status-indicator">
          <h2 className={`text-center ${connected ? 'text-success' : 'text-danger'}`}>
            {connected ? 'Connected to ROSBridge' : 'Disconnected from ROSBridge'}
          </h2>
        </div>
        <div className="row">
          <TemperatureDisplay />
          <PressureDisplay />
          <MotorControl />
        </div>
        <CameraFeed />
      </div>
    </div>
  );
};

export default HomePage;
