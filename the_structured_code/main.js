import './style.css'

import ROSLIB from 'roslib'

const inputLinearX = document.getElementById("input-linear-x")
const inputLinearY = document.getElementById("input-linear-y")
const inputLinearZ = document.getElementById("input-linear-z")
const inputAngularX = document.getElementById("input-angular-x")
const inputAngularY = document.getElementById("input-angular-y")
const inputAngularZ = document.getElementById("input-angular-z")
const velocityButton = document.getElementById("velocity-button")
const buttonAuto = document.getElementById("btn-auto")
const buttonManual = document.getElementById("btn-manual")
const buttonClearLogs = document.getElementById("clear-logs-btn")
const statusText = document.getElementById("status")
const logsEl = document.getElementById("logs")

/** @type {HTMLCanvasElement} */
const mapEl = document.getElementById("map")

const joystick = {
  forward: document.getElementById("joystick-btn-forward"),
  left: document.getElementById("joystick-btn-left"),
  right: document.getElementById("joystick-btn-right"),
  backward: document.getElementById("joystick-btn-backward"),
}

const joystickDataDirections = {
  forward: {
    linear: { x: 0.1, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 },
  },
  backward: {
    linear: { x: -0.1, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 },
  },
  left: {
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0.1 },
  },
  right: {
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: -0.1 },
  },
  stop: {
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 },
  },
}

let connected = false
const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
})
ros.on('error', function (error) {
  console.log(error);
  statusText.innerText = 'Error'
  statusText.classList.add('text-error')
});
ros.on('connection', function () {
  console.log('Connection established!');
  connected = true
  statusText.innerText = 'Connected'
  statusText.classList.add('text-ok')
});

const velocityTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/man_cmd_vel',
  messageType: 'geometry_msgs/Twist'
})

const changeModeService = new ROSLIB.Service({
  ros: ros,
  name: '/change_mode',
  serviceType: 'overlord100_msgs/ChangeMode'
});

const logsTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/logs',
  messageType: 'overlord100_msgs/LogMessage'
})

var mapTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/map",
  messageType: "nav_msgs/msg/OccupancyGrid",
});

velocityButton.addEventListener("click", () => {
  const speed = {
    linear: {
      x: Number(inputLinearX.value),
      y: Number(inputLinearY.value),
      z: Number(inputLinearZ.value),
    },
    angular: {
      x: Number(inputAngularX.value),
      y: Number(inputAngularY.value),
      z: Number(inputAngularZ.value),
    },
  }
  setSpeed(speed)
})

buttonAuto.addEventListener("click", () => {
  const req = new ROSLIB.ServiceRequest({ mode: 1 });
  console.log("Setting auto mode...")
  changeModeService.callService(req, (res) => {
    console.log("Set auto mode response:", res)
  });
})

buttonManual.addEventListener("click", () => {
  const req = new ROSLIB.ServiceRequest({ mode: 0 });
  console.log("Setting manual mode...")
  changeModeService.callService(req, (res) => {
    console.log("Set manual mode response:", res)
  });
})

/**
 * @param {({ level: number, node_name: string, message: string })} log 
 */
function receiveLog(log) {
  const entry = document.createElement("div")
  entry.classList.add("log-entry")

  const nodeName = document.createElement("span")
  nodeName.classList.add("log-entry_node")
  nodeName.innerText = log.node_name 

  const level = document.createElement("span")
  level.classList.add("log-entry_level")
  level.innerText = log.level + "\n"

  const content = document.createElement("span")
  content.innerText = log.message

  entry.appendChild(nodeName)
  entry.appendChild(level)
  entry.appendChild(content)

  logsEl.appendChild(entry)
}

function clearLogs() {
  logsEl.innerHTML = ""
}

logsTopic.subscribe((msg) => {
  receiveLog(msg)
})

buttonClearLogs.addEventListener("click", () => {
  clearLogs()
})

mapTopic.subscribe((message) => {
  console.log("[map] > ", message)
  drawMap(message)
})

function setSpeed(newSpeed) {
  if (!connected) {
    alert('not connected')
    return
  }

  velocityTopic.publish(new ROSLIB.Message(newSpeed))
}

for (const [dir, btnEl] of Object.entries(joystick)) {
  btnEl.addEventListener("mousedown", () => {
    setSpeed(joystickDataDirections[dir])
  })
  btnEl.addEventListener("mouseup", () => {
    setSpeed(joystickDataDirections['stop'])
  })
}

function drawMap(map) {
  const canvas = mapEl; // Ensure we are getting the correct canvas element
  const ctx = canvas.getContext('2d'); // Get the 2D context

  const width = map.info.width;
  const height = map.info.height;
  const data = map.data;

  // Create imageData object to store map data
  const imageData = ctx.createImageData(width, height);

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const index = x + y * width;
      const value = data[index];
      
      let gray;
      // Determine the color based on the value
      if (value === -1) {
        gray = 127; // Gray for unknown cells
      } else if (value > 50) {
        gray = 0; // Black for occupied cells
      } else {
        gray = 255; // White for free cells
      }

      // Set the pixel color in the image data object
      const pixelIndex = (x + (height - y - 1) * width) * 4;
      imageData.data[pixelIndex] = gray;
      imageData.data[pixelIndex + 1] = gray;
      imageData.data[pixelIndex + 2] = gray;
      imageData.data[pixelIndex + 3] = 255; // Fully opaque
    }
  }

  // Clear the entire canvas
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  
  // Create a temporary canvas to draw the imageData and scale it
  const tempCanvas = document.createElement('canvas');
  const tempCtx = tempCanvas.getContext('2d');
  tempCanvas.width = width;
  tempCanvas.height = height;
  tempCtx.putImageData(imageData, 0, 0);

  // Calculate the scaling factor to fit the map into the canvas
  const scaleX = canvas.width / width;
  const scaleY = canvas.height / height;

  // Draw the scaled image onto the main canvas
  ctx.drawImage(tempCanvas, 0, 0, width, height, 0, 0, canvas.width, canvas.height);
}
