import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import './App.css';

const App = () => {
  const canvasRef = useRef(null);
  const [connected, setConnected] = useState(false);
  const [map, setMap] = useState(null);
  const ros = useRef(null);

  useEffect(() => {
    if (!ros.current) {
      ros.current = new ROSLIB.Ros();

      ros.current.on('connection', () => {
        console.log('Connected to websocket server.');
        setConnected(true);
      });

      ros.current.on('error', (error) => {
        console.log('Error connecting to websocket server: ', error);
      });

      ros.current.on('close', () => {
        console.log('Connection to websocket server closed.');
        setConnected(false);
      });

      ros.current.connect('ws://localhost:9090');
    }

    const mapListener = new ROSLIB.Topic({
      ros: ros.current,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid'
    });

    const updateMap = (message) => {
      console.log('Received map data'); // Debug statement
      setMap(message);
    };

    mapListener.subscribe(updateMap);

    const interval = setInterval(() => {
      mapListener.subscribe(updateMap);
    }, 3000); // Update every 3 seconds

    return () => {
      clearInterval(interval);
      mapListener.unsubscribe();
      if (ros.current && ros.current.isConnected) {
        ros.current.close();
      }
    };
  }, []);

  useEffect(() => {
    if (map && canvasRef.current) {
      const canvas = canvasRef.current;
      const ctx = canvas.getContext('2d');

      // Clear the canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      
      const mapWidth = map.info.width;
      const mapHeight = map.info.height;
      const resolution = map.info.resolution;


      const canvasWidth = 800;
      const canvasHeight = 800;
      const scaleX = canvasWidth / mapWidth;
      const scaleY = canvasHeight / mapHeight;

      // Draw map
      const imageData = ctx.createImageData(mapWidth, mapHeight);
      for (let y = 0; y < mapHeight; y++) {
        for (let x = 0; x < mapWidth; x++) {
          const i = y * mapWidth + x;
          const value = map.data[i];
          const color = value === -1 ? 127 : value > 50 ? 0 : 255;
          const index = (y * mapWidth + x) * 4;
          imageData.data[index] = color;
          imageData.data[index + 1] = color;
          imageData.data[index + 2] = color;
          imageData.data[index + 3] = 255;
        }
      }

      // Scale the image data to fit the canvas
      const tempCanvas = document.createElement('canvas');
      tempCanvas.width = mapWidth;
      tempCanvas.height = mapHeight;
      const tempCtx = tempCanvas.getContext('2d');
      tempCtx.putImageData(imageData, 0, 0);

      ctx.save();
      ctx.scale(scaleX, scaleY);
      ctx.drawImage(tempCanvas, 0, 0);
      ctx.restore();

      // Draw axes
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(canvas.width, 0);
      ctx.moveTo(0, 0);
      ctx.lineTo(0, canvas.height);
      ctx.strokeStyle = 'black';
      ctx.stroke();

      // Label axes
      ctx.fillStyle = 'black';
      ctx.fillText('X', canvas.width - 20, 20);
      ctx.fillText('Y', 20, canvas.height - 20);
    }
  }, [map]);


  return (
    <div className="App">
      <header className="App-header">
        <h1>Robot Map Visualization</h1>
        {connected ? <p>Connected to ROS</p> : <p>Connecting to ROS...</p>}
        <canvas ref={canvasRef} width="800"  id="my-map" height="800"></canvas>
      </header>
    </div>
  );
};

export default App;
