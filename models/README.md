 <h1>MODELS DOCUMENTATION</h1><br>
<p>
   This folder contains details about the model, design development and functionality of the robot developed by Team Artemis for the World Robot Olympiad(WRO) Future Engineers Challenge. The purpose of this documentation is to provide an overview of the robots mechanical structure, key components and design considerations . it highlights the engineering decisions made during the development process and explains the various design  alterations made to ensure effective performance.
</p><br>
<h2>🛠️Mechanical Design And Sizing</h2><br>
<p> The design of Team Artemis's robot was guided by two main objectives: compactness and component protection. The robot of size 14cm(L) 8.8cm(W) and 5.63cm(H) was built with a small footprint to improve maneuverability and reduce the likelihood of collisions during operation.</p>
<p>In addition, the majority of the electronic components were positioned within the robot's structure rather than being exposed externally. This approach helps protect sensitive components, reduces the risk of accidental damage or tampering, and contributes to a cleaner and more organized design. The compact arrangement of components also improves weight distribution and overall stability during movement.</p><br>
Figures 1&2 show the completed robot from different perspectives.<br>
<h4> ISOMETRIC VIEW </h4>
<p align="center">
<img width="1191" height="778" alt="Isometric" src="https://github.com/user-attachments/assets/be3c25d9-5065-48c6-8190-f8b5faf5acaf" />
</p>
<h4> ORTHOGRAPHIC VIEWS </h4>
<P><img width="714" height="417" alt="FOUR SIDE" src="https://github.com/user-attachments/assets/0fa2cfac-3aa2-4138-8db0-d97fbfe9e14b" />
</P><br>
<h2>🔧STEERING MECHANISM </h2><br>
<p>
The robot uses a steering system based on Ackermann steering geometry, allowing the inner wheel to turn at a greater angle than the outer wheel during cornering. This reduces wheel slip and improves turning accuracy. Steering is actuated by a servo motor, with the servo horn directly connected to the steering linkage to control the front wheels.<br>
The simulations and steering analysis graph used to validate the design are shown below.
</p><br>
<img width="737" height="536" alt="TEST" src="https://github.com/user-attachments/assets/3ec526db-7bef-41e5-b09d-fc89b952c792" />
<br>
https://github.com/user-attachments/assets/2cb1c8b4-25f4-4e00-9db0-9a84759839ec
<h2> ⚙️REAR DRIVE / MOTION </h2><br>
<P>
Artemis employs a single-motor differential rear drive system, designed in accordance with competition rules that restrict the use of multiple drive motors for the rear drivetrain. This constraint influenced the decision to optimize performance around a single high-speed motor while maintaining mechanical simplicity and efficiency.<br>
A custom gearbox was developed to support the drivetrain layout and improve mechanical control of power transmission to the rear wheels. The gearbox also ensured proper integration between the motor and wheel assembly within the limited available space, addressing key sizing constraints in the robot design.<br>
A 1:1 gear ratio was selected to preserve the motor’s rated speed of 500 RPM, prioritizing velocity and responsiveness over torque multiplication. While alternative gear ratios were considered to increase torque, spatial limitations and design constraints prevented further reduction or multiplication within the gearbox.<br>
Encoders are incorporated into the drivetrain to enable measurement of wheel rotation, supporting precise motion tracking for autonomous navigation and improving control accuracy of the system.<br>
 See below simulations and graph analysis for the rear motion.<br></P>
 <img width="353" height="200" alt="Differential drive " src="https://github.com/user-attachments/assets/9aeccc85-b13d-44da-b332-d97253333e70" />
 <h2>🪚CHASIS MODIFICATIONS </h2>
<p>The chassis was iteratively redesigned to improve performance, manufacturability, and integration of components. Material was removed in selected regions to increase steering clearance, enabling a greater steering angle and improved maneuverability.<br>
To optimize production efficiency, the base structure thickness was reduced to lower filament usage and printing time while still maintaining sufficient rigidity. To compensate for the reduced material, reinforcement extrusions were added at key load-bearing points to improve structural strength and prevent deformation under load.<br>
A slip-fit mounting system was implemented between the chassis and the upper body to allow quick and tool-free assembly and disassembly, improving accessibility for maintenance and internal adjustments. The upper body was designed as a hollow enclosure to house electronic components securely, while ventilation cutouts were incorporated to support airflow and reduce heat accumulation during operation.<p><br>
See the figures below to view iterations made.<br>
<div align ="centre">
 <img width="353" height="200" alt="first" src="https://github.com/user-attachments/assets/10d61965-bf15-4e79-865b-48f4ca92aa18"
  />➡️➡️
 <img width="353" height="200" alt="final" src="https://github.com/user-attachments/assets/8fa4f160-aa62-4e27-ad81-3dd9c0513495" />
</div>












