import React from 'react';

const CameraFeed = () => {
  const videoStreamUrl = 'http://192.168.0.191:8080/stream?topic=/usb_cam/image_raw';

  return (
    <div className="position-fixed bottom-0 end-0 p-3">
      <div className="card" style={{ width: '320px' }}>
        <div className="card-body">
          <h5 className="card-title text-center">Live Camera Feed</h5>
          <img src={videoStreamUrl} alt="Live Video Feed" className="card-img-top" />
        </div>
      </div>
    </div>
  );
};

export default CameraFeed;
