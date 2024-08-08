import holoocean
import numpy as np
import math
import matplotlib.pyplot as plt
from holoocean.agents import AgentDefinition, SensorDefinition
import time

########## Signal Strength Stuff ####################
__DIST_SENSOR_SENSITIVITY__ = 0.2

def signal_strength(ground_truth_distance, sensor_range=50000, sensitivity=__DIST_SENSOR_SENSITIVITY__): 
	# Range on the Acoustic sensor set to 50km default
	# Increase sensitivity to get more accurate reading closer to the tag
	normalized_gt = (((ground_truth_distance+sensor_range)*2)/(sensor_range)) - 1
	s = 1/(1+np.exp(-normalized_gt/sensitivity))
	return s
	
def distance_from_signal_strength(s, x_ref=np.array([-50000,0]), k=__DIST_SENSOR_SENSITIVITY__):
	x_norm = -k * np.log((1/s)-1)
	x_min=x_ref.min()
	x_max=x_ref.max()
	x = (x_norm+1)/2 * (x_max-x_min) + x_min
	return -x

'''
# Singnal Strength graph
x_un = np.linspace(-50000,0,1000)
plt.plot(x_un,signal_strength(x_un))
plt.show()

raise SystemExit()
'''
#########################################

########## Holoocean Definitions ##################
cfg = {
    "name": "AnimalTagSearch",
    "world": "OpenWater",
    "package_name": "Ocean",
    "main_agent": "Surface",
    "ticks_per_sec": 60,
    "agents": [
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
                            "CheckVisible": False
                            #"location": [0,0,0],
                            #"rotation": [0,0,0]
                            }
                }
            ],
            "control_scheme": 1,
            "location": [300,300,0]
        },
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
        },
        {
            "agent_name": "Drone",
            "agent_type": "UavAgent",
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
                            "id": 9,
                            "CheckVisible": False
			}
                }
            ],
            "control_scheme": 1,
            "location": [-5,-5,20],
            "rotation": [0,0,0]
        }
	]
}

'''
sensors_drone = [Sensordefinition]
sensor_names = {"DroneLocSensor": "LocationSensor", "DroneAcousticBeacon": "LocationSensor"}
for keys in sensor_names:


	sensors_drone.append(
                    SensorDefinition(
                        "Drone",
                        "UavDrone",
                        keys,
                        sensor_names[keys]
                    )
                )

sensor_drone = [SensorDefinition("Drone", "UavDrone", "DroneLocSensor", "LocationSensor", config={}), SensorDefinition("Drone", "UavDrone", "DroneAcousSensor", "AcousticBeaconSensor", config={"id":9})]
'''


env = holoocean.make(scenario_cfg=cfg)
env.reset()

###########################################

# Angles Initialization
theta = None  # Angle from Goniometer to Animal Tag
phi = None    # Angle from Drone to Animal Tag
d = None      # Distance from Goniometer to Drone
goal_position_drone = None
signal_str = None
surface_vessel_goal = None

################### Plot Initialization ##########################
plt.ion()
fig, ax = plt.subplots()
sc1 = ax.scatter([], [], color='blue', label='Estimated Tag Position')
sc2 = ax.scatter([], [], color='red', label='Buoy Location')
sc3 = ax.scatter([], [], color='green', label='Drone Location')
sc4 = ax.scatter([], [], color='yellow', label='Surface Location')

# Set plot limits
ax.set_xlim(-50, 500)
ax.set_ylim(-50, 500)
ax.legend()


def update_plot():
	# Use tag_est_pos with the close thing
	sc1.set_offsets(np.append(sc1.get_offsets(), [surface_vessel_goal], axis=0))
	sc2.set_offsets(np.append(sc2.get_offsets(), [states["Buoy"]["LocationSensor"][:2]], axis=0))
	if states.get("Drone") is not None:
		sc3.set_offsets(np.append(sc3.get_offsets(), [states.get("Drone").get("LocationSensor")[:2]], axis=0))
	sc4.set_offsets(np.append(sc4.get_offsets(), [states["Surface"]["LocationSensor"][:2]], axis=0))
	plt.draw()
	plt.pause(0.01)
	
####################################################################

# Starting Loop
spawned = False
counter = 0
states = env.tick()
'''
####### Spawninh Code #############
while True:
	if not spawned:
		# env.act("Surface", np.array([0,0]))
		test = holoocean.agents.AgentDefinition("Drone", "UavAgent", sensors=None, starting_loc=np.array([10,10,10]), starting_rot=(0, 0, 0), existing=False, is_main_agent=False)
		env.add_agent(test, is_main_agent=False)
		print("new agent spawned")
		spawned = True
	# states["Surface"]["LocationSensor"]+
	env.tick()	
'''

while True:
	# Make sure the robot is not spinning when Initialized
	if surface_vessel_goal is None:
		env.act("Surface", np.array([0,0]))
	# Send acoustic signal from the bouy
	if (counter % 600 == 0):
		print("signal sent")
		env.send_acoustic_message(2, -1, "OWAYU", "test1")

	counter = counter + 1
	
	
	if signal_str is not None:
		if (signal_str>0.9928) and not spawned:
			env.act("Surface", states["Surface"]["LocationSensor"][:2])
			env.agents["Drone"].set_physics_state(states["Surface"]["LocationSensor"]+np.array([0,0,30]), np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0]))
			env.act("Drone", np.array([0,0,0,30]))
			print("new agent spawned")
			spawned = True



	
	# Assigning Phi
if "AcousticBeaconSensor" in states["Surface"] and (signal_str is None or signal_str<=0.9928):
		phi = states["Surface"]["AcousticBeaconSensor"][-2]
		
		print("Surface to Animal Tag Angle in Deg", phi*(180/math.pi))
		
		yaw_surface = states["Surface"]["RotationSensor"][2]*(math.pi/180)
		
		signal_str = signal_strength(-np.linalg.norm(states["Surface"]["LocationSensor"]-states["Buoy"]["LocationSensor"]))
		print("Signal Strength", signal_str)
		print("Estimate Distance", distance_from_signal_strength(signal_str))
		print("Actual Distance", np.linalg.norm(states["Surface"]["LocationSensor"]-states["Buoy"]["LocationSensor"]))
		if phi>=0:
			surface_vessel_goal = distance_from_signal_strength(signal_str)*np.array([math.cos(phi + yaw_surface), math.sin(phi + yaw_surface)])+states["Surface"]["LocationSensor"][:2]
		else:
			surface_vessel_goal = distance_from_signal_strength(signal_str)*np.array([math.cos(2*math.pi + phi + yaw_surface), math.sin(2*math.pi + phi + yaw_surface)])+states["Surface"]["LocationSensor"][:2]
		
		print("Surface Vessel Goal", surface_vessel_goal)
		env.act("Surface", surface_vessel_goal)
		update_plot()
		print("____________________________________")
		
		states = env.tick()
	
	
	
		
		
		

'''	# Assigning Theta
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
'''
