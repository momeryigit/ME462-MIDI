# PROJECT SPECIFICATION DOCUMENT OF MIDI ROBOT PROJECT


<p align="center">
<img src="https://github.com/momeryigit/ME462-MIDI/blob/7dae15a87a1ac55c6e7e3ea2433b3f86bfee68b8/Project_Specification_Docs/cloudy.png" width=35% height=35%>
</p>

## PROBLEM DEFINITION

Cloudy Mini - AGV is a fully open-source and autonomous robotics learning platform based on ROS 2 framework, allowing users to fully customize and modify the platform for their own educational purposes.   

Aim of this project, which is based on Cloudy Mini MKIII, is to improve the precursor such that:
1. It will be more durable,
2. It will be easier and faster to manufacture,
3. It will be more intelligent,
4. It will be more open to hardware changes

### Example Use Cases
Some different use cases for the robot could be:
1. If load carrying capacity can be high enough, like around 100 kg, this robot can be used in factories with inventory management purposes. This is a simpler use case in my opinion, because the factory would be a controlled environment and the ground will be near perfectly flat, so there won't be much problem due to rough terrain. The main requirement for this use case would be high load carrying capacity and fully autonomous driving and being trackable from a main server. __(ERDEM)__
2. If integrated with a camera as well as a lidar scanner, the robot could be used as a surveillance robot where the robot continously moves through its operating area scanning and mapping the enviorement. An algorithm could be implemented to detect changes in the previous 3d mapping such as open doors or traces left behind by a possible intruder. The camera could also be used to detect humans. If a change or human is detected the robot could inform the responsible person through the cloud. __(SARP)__


## PROJECT REQUIREMENTS

1. The robot should use Makita Li-ion rechargable batteries as its battery (BL1830B, 18V-3.0Ah-54Wh) and Makita charging dock for its charging purposes.
2. The robot should be able to change batteries seamlessly such that it will not shut down between battery change process (Hot-swap batteries) or it will inform the user about the low battery status and shut itself down properly and then let the user will change the battery.
3. The robot should be easily manufacturable and the manufacturing processes can include: 3D printing (only FDM  type) (PLA and TPU materials), laser cutting (only plexyglass or wood, no metal), metal-cutting. Also off-the-shelf materials like sigma profiles (only 20x20mm or 20x40mm sized), machine elements like bearings springs etc. can be used.
4. The robot should be able to carry a payload of 20 kg at least.
5. The robot should be able to easily adapt to new different sensors for different purposes.
6. The robot should have led or some other indicator on its outer side with the aim of informing the user about its current status.
7. The robot should be compatible with ROS2 environment.
8. The main processor of the robot will be a Raspberry Pi 4 or Raspberry Pi 5. But use of additional microcontrollers is not restricted.


## DESIGN CRITERIA

1. Rechargability: With the use of Makita BL1830B Li-ion batteries, the robot should be rechargable.
2. Uninterrupted operation of the robot: The robot should have a uninterrupted operation by seamless battery change.
3. Ease of manufacturing and assembly: The manufacturing of the robot should not take more than 12 hours.
4. Payload capacity: The robot should be able carry a minimum of 20 kg payload.
5. Adaptive design: The design of the robot should give the user enough flexibility to use it with different sensors and also the size of the robot should be changeable.
6. Status indicator: The robot should give information about its status via leds or some other indicator.
7. ROS2 compatibility: The robot should be compatible with ROS2 environment.
8. Main-processor type: The main processor of the robot should be Raspberry Pi 4 or 5.
