import holoocean
import numpy as np
import math
import matplotlib.pyplot as plt


cfg = {
    "name": "AnimalTagSearch",
    "world": "OpenWater",
    "package_name": "Ocean",
    "main_agent": "Surface",
    "ticks_per_sec": 60,
    "agents": [
        {
            "agent_name": "Surface",
            "agent_type": "SurfaceVessel",
            "sensors": [
                {
                    "sensor_type": "LocationSensor"
                },
                {
                    "sensor_type": "RotationSensor"
                },
                {
                    "sensor_type": "AcousticBeaconSensor",
                    "configuration": {
                            "id": 1,
                            "CheckVisible": False,
                             "rotation": [0,0,0]
                            }
                }
            ],
            "control_scheme": 1,
            "location": [50, 0, 0],
            "rotation": [0,0,0]
        },
        {
            "agent_name": "Buoy",
            "agent_type": "SphereAgent",
            "sensors": [
                {
                    "sensor_type": "LocationSensor"
                },
                {
                    "sensor_type": "AcousticBeaconSensor",
                    "configuration": {
                            "id": 2,
                            "CheckVisible": False,
                            "location": [0,0,0],
                            "rotation": [0,0,0]
                            }
                }
            ],
            "control_scheme": 1,
            "location": [30,40,0]
        },
        {
            "agent_name": "Goniometer",
            "agent_type": "SphereAgent",
            "sensors": [
                {
                    "sensor_type": "LocationSensor"
                },
                {
                    "sensor_type": "AcousticBeaconSensor",
                    "configuration": {
                            "id": 4,
                            "CheckVisible": False,
                            "location": [0,0,0],
                            "rotation": [0,0,0]
			}
                }
            ],
            "control_scheme": 1,
            "location": [0,0,0],
            "rotation": [0,0,0]
        }
	]
}

env = holoocean.make(scenario_cfg=cfg)
env.reset()



theta = None  # Angle from Goniometer to Animal Tag
phi = None    # Angle from Drone to Animal Tag
d = None      # Distance from Goniometer to Drone
goal_position_drone = None

plt.ion()
fig, ax = plt.subplots()
sc1 = ax.scatter([], [], color='blue', label='Estimated Tag Position')
sc2 = ax.scatter([], [], color='red', label='Buoy Location')
sc3 = ax.scatter([], [], color='green', label='Goniometer Location')
sc4 = ax.scatter([], [], color='yellow', label='Surface Location')

# Set plot limits
ax.set_xlim(-100, 100)
ax.set_ylim(-100, 100)
ax.legend()

def update_plot():
	sc1.set_offsets(np.append(sc1.get_offsets(), [tag_est_pos[:2]], axis=0))
	sc2.set_offsets(np.append(sc2.get_offsets(), [states["Buoy"]["LocationSensor"][:2]], axis=0))
	sc3.set_offsets(np.append(sc3.get_offsets(), [states["Goniometer"]["LocationSensor"][:2]], axis=0))
	sc4.set_offsets(np.append(sc4.get_offsets(), [states["Surface"]["LocationSensor"][:2]], axis=0))
	plt.draw()
	plt.pause(0.01)
	
counter = 0
while True:
	states = env.tick()

	if (counter % 600 == 0):
		env.send_acoustic_message(2, -1, "OWAYU", "test1")
		
		
	counter = counter + 1

	
	
	# Assigning Phi
	if "AcousticBeaconSensor" in states["Surface"]:
		azimuth = states["Surface"]["AcousticBeaconSensor"][-2]
		elevation = states["Surface"]["AcousticBeaconSensor"][-1]
		
		if abs(elevation) < 0.2:
			phi = azimuth
		else:
			phi = azimuth
			
		print("Drone to Animal Tag (Theta)", phi*(180/math.pi))
	
	# Assigning Theta
	if "AcousticBeaconSensor" in states["Goniometer"]:
		azimuth = states["Goniometer"]["AcousticBeaconSensor"][-2]
		elevation = states["Goniometer"]["AcousticBeaconSensor"][-1]
		
		if abs(elevation) < 0.2:
			theta = azimuth
		else:
			theta = azimuth
			
		print("Goniometer to Animal Tag (Phi)", theta*(180/math.pi))
	
	if phi is not None and theta is not None:
		d = np.linalg.norm(states["Surface"]["LocationSensor"] - states["Goniometer"]["LocationSensor"])
		print("Distance", d)
		# angle between Drone and Goniometer
		e = math.atan2(states["Surface"]["LocationSensor"][1] - states["Goniometer"]["LocationSensor"][1], states["Surface"]["LocationSensor"][0] - states["Goniometer"]["LocationSensor"][0])
		print("Angle from G to D", e)
		# Determine A_1 # FOOLPROOF
		A_1 = theta - e #When theta is less than 90 degrees
		
		yaw_drone = (states["Surface"]["RotationSensor"][2]) * math.pi /180
		
		print("Drone Orientation", yaw_drone)
		# Determine A_2 # FOOLPROOF
		if phi > (math.pi/2):
			A_2 = (math.pi - phi) #+ yaw_drone
		else:
			A_2 = math.pi - yaw_drone - phi + e
			  
		print("A_1", A_1)
		print("A_2", A_2)
		
		gamma = math.pi - (A_2+A_1)
		
		tag_est_dis = d * math.sin(A_2)/math.sin(gamma)
		tag_est_pos = tag_est_dis*np.array([math.cos(theta), math.sin(theta),0])
		
		print("Estimated Tag Position", tag_est_pos)
		print("Ground Truth Tag Position", states["Buoy"]["LocationSensor"])
		print("Tag Distance Estimate Error", tag_est_dis - np.linalg.norm(states["Goniometer"]["LocationSensor"] - states["Buoy"]["LocationSensor"]))
		print("Position Estimate Error", np.linalg.norm(tag_est_pos - states["Buoy"]["LocationSensor"]))
		print("Surface to Buoy Distance", np.linalg.norm(states["Surface"]["LocationSensor"] - states["Buoy"]["LocationSensor"]))
		print('__________________')
		phi = None
		theta = None
		
		env.act('Surface', tag_est_pos[:2])
		
		### Matplotlib section 
		update_plot()
		if np.linalg.norm(states["Surface"]["LocationSensor"] - states["Buoy"]["LocationSensor"]) <= 3:
			break

plt.ioff()
plt.show()
	
	
			
	
	
	
	
	

