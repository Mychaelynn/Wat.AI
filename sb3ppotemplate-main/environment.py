"""
Self-Driving Car Environment for SB3 PPO Training

This is a Gymnasium-compatible environment that uses PyBullet for physics simulation.
The environment integrates with the Track and TankDriveController classes from the setup folder.

TODO: Fill in the blanks marked with # TODO comments and answer the Socratic questions!
"""
import yaml
import os
import numpy as np
import pybullet as p
import gymnasium as gym
from gymnasium import spaces
from typing import Dict, Tuple, Optional, Any
import pybullet as p
import pybullet_data


# Import the track and controller from setup
from setup.track import Track
from setup.controls import TankDriveController


class SelfDrivingCarEnv(gym.Env):
    """
    A self-driving car environment using PyBullet physics.
    
    The agent controls a car that must navigate around a procedural racetrack.
    Think about: What makes a good observation space for a self-driving car?
    What actions should the agent be able to take?
    """
    
    metadata = {"render_modes": [" ", "rgb_array"], "render_fps": 30}
    
    def __init__(
        self,
        config_path: str = "setup/track_config.yaml",
        render_mode: Optional[str] = None,
        max_episode_steps: int = 1000,
    ):
        """
        Initialize the environment.
        
        Questions to consider:
        - What information does the agent need to see? (observation space)
        - What actions can the agent take? (action space)
        - How do we define success or failure? (reward function)
        - When should an episode end? (termination conditions)
        
        Args:
            config_path: Path to the YAML configuration file
            render_mode: "human" for GUI, "rgb_array" for images, None for headless
            max_episode_steps: Maximum steps before truncation
        """
        super().__init__()
        self.stagnation_counter = 0
        self.config_path = config_path
        self.render_mode = render_mode
        self.max_episode_steps = max_episode_steps
        self.current_step = 0
        
        # Load configuration
        # TODO: Load the YAML config file here. What information do you need from it?
        # Hint: You'll need physics settings, spawn position, etc.
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Initialize PyBullet
        # TODO: How do you connect to PyBullet? What's the difference between GUI and DIRECT mode?
        # Think about: When would you use each mode?
        if render_mode == "human":
        # If a DIRECT connection exists, this might still fail. 
        # Try connecting to the GUI.
            try:
                self.physics_client = p.connect(p.GUI)
            except p.error:
        # Fallback to DIRECT if GUI is occupied
                self.physics_client = p.connect(p.DIRECT)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        physicsClient= self.physics_client
        
        # TODO: Set up physics parameters from config
        
        self.physics_client = physicsClient
        
        # TODO: Set up physics parameters from config
        # What physics settings affect the simulation? (gravity, time step, etc.)
        p.setGravity(0, 0, -9.8, physicsClientId=self.physics_client)
        p.setTimeStep(1/240, physicsClientId=self.physics_client)
        
        # Initialize track
        # TODO: Create a Track instance. What parameters does it need?
        self.track = Track(self.config_path, self.physics_client) # where to load file from, where to spawn in 
        self.track.spawn_in_pybullet(self.physics_client)
        # Add ground plane for car to drive on
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.physics_client)
        
        # Load car URDF
        # TODO: Load the car model. Where is the URDF file located?
        # Think about: What happens if the file doesn't exist?
        car_urdf_path = "setup/car.urdf"
        # Check if file exists to avoid crashes
        if not os.path.exists(car_urdf_path):
            raise FileNotFoundError(f"Car URDF file not found at {car_urdf_path}")
        
        # TODO: Set initial car position and orientation from config
        # What's the difference between position and orientation?
        spawn_pos = self.config['spawn']['position'] 
        spawn_orn = self.config['spawn']['orientation']
      
        
        self.car_id = p.loadURDF(car_urdf_path, spawn_pos, spawn_orn, physicsClientId=self.physics_client)

        p.resetBasePositionAndOrientation(self.car_id,
                                           spawn_pos, 
                                           spawn_orn, 
                                           physicsClientId=self.physics_client)
##################################
        # Initialize controller
        # TODO: Create the TankDriveController. What does it need?
        # The TankDriveController needs to know which car to move, 
        # the physics ID, and the specific wheel joint indices.
        self.controller = TankDriveController(car_id = self.car_id, 
                                              physics_client = self.physics_client,
                                              config_path = self.config_path)
        
        # Define action space
        # TODO: What actions should the agent control?
        # Think about: The TankDriveController uses forward_input (-1 to +1) and turn_input (-1 to +1)
        # What type of action space is appropriate? (Discrete, Box, etc.)
        # Should actions be continuous or discrete? Why?
        self.action_space = spaces.Box(low = -1.0,
                                       high = 1.0,
                                       shape=(2,),
                                       dtype = np.float32)
        
        # Define observation space
        # TODO: What should the agent observe?
        # Consider: Car position, velocity, orientation, distance to track boundaries, etc.
        # What shape should observations have? What are reasonable bounds?
        self.observation_space = spaces.Box(
             low=np.array([-100.0, -100.0, -50.0, -50.0, -np.pi, -10.0]),
             high=np.array([100.0, 100.0, 50.0, 50.0, np.pi, 10.0]),
             dtype=np.float32
             #Position (±100): Track is small (~3 radius), but giving extra room for safety
            # Velocity (±50): Max is 2.0 m/s, but 50 gives huge safety margin
            # Yaw (±π): Angles naturally bounded to ±π radians (±180°)
            # Angular vel (±10): Max is 2.0 rad/s, 10 is generous buffer

        )
        #observation is values env gives to AI
        #AI reads these numbers to understand current state

        #actoin space is what AI gives to enviornment, tells AI what to do next 
        
        # TODO: Initialize any tracking variables you need
        # What information do you need to compute rewards and check termination?
        self.last_position = np.array(spawn_pos[:2])
        self.progress = 0.0
        self.episode_reward = 0.0
        
    def _get_observation(self) -> np.ndarray:
        """
        Get the current observation.
        
        Questions to consider:
        - What information helps the agent make good decisions?
        - Should observations be normalized? Why or why not?
        - How can you extract useful features from the car's state?
        
        Returns:
            observation: The current state observation
        """
        # TODO: Get car's current state from PyBullet
        # What information can you get about the car's position, orientation, and velocity?
        car_pos, car_orn = p.getBasePositionAndOrientation(self.car_id, physicsClientId=self.physics_client)
        car_vel, car_ang_vel = p.getBaseVelocity(self.car_id, physicsClientId=self.physics_client)
        
        # TODO: Extract useful features
        # Think about:
        # - How can you represent orientation? (Euler angles, quaternion, rotation matrix?)
        # - What velocity components matter? (forward speed, angular velocity?)
        # - How can you measure distance to track boundaries?
        # - Should you include relative position on the track?

        #FEATURES TO ADD TO OBSERVATION
        x = car_pos[0]
        y = car_pos[1]

        vx = car_vel[0]
        vy = car_vel[1]
        angular_vel = car_ang_vel[2] #rotational speed around z-axis -- how fast is car rotating
        
        # which way is car facing -- rotation around z axis
        yaw = p.getEulerFromQuaternion(car_orn)[2]

        # TODO: Combine features into observation vector
        observation = np.array([
            x/100,
            y/100,
            vx/50,
            vy/50,
            yaw/np.pi,
            angular_vel/10,
        ], dtype = np.float32)
        
        # TODO: Normalize observations if needed
        # Why might normalization help training?
        # Optional: Normalize observations
    # This helps neural networks learn faster because all values are similar scale
    # Example: positions might be -100 to 100, but yaw is -π to π
    # Without normalization, the network treats large position changes as more important
        
        return observation  # Placeholder - replace with actual observation
    ############################################
    def _compute_reward(self, action: np.ndarray) -> float:
        """
        Compute the reward for the current step.
        
        This is a critical function! The reward signal shapes what the agent learns.
        
        Questions to consider:
        - What behaviors do you want to encourage? (staying on track, making progress, etc.)
        - What behaviors should be penalized? (going off track, crashing, etc.)
        - How do you balance different objectives? (progress vs. safety)
        - Should rewards be sparse (only at milestones) or dense (every step)?
        
        Args:
            action: The action taken by the agent
            
        Returns:
            reward: The reward value
        """
        # TODO: Get current car state
        car_pos, car_orn = p.getBasePositionAndOrientation(
        self.car_id, 
        physicsClientId=self.physics_client
        )
        car_vel, car_ang_vel = p.getBaseVelocity(
        self.car_id,
        physicsClientId=self.physics_client
        )

        current_pos = np.array(car_pos[:2]) 
        distance_from_center = np.linalg.norm(current_pos)

#get track boundaires 
        inner_radius = self.config['track']['inner_radius']
        outer_radius = self.config['track']['outer_radius']

        # Check if on track
        is_on_track = (distance_from_center > inner_radius and
                    distance_from_center < outer_radius)

        # Progress reward (main objective)
        distance_traveled = np.linalg.norm(current_pos - self.last_position)
        progress_reward = distance_traveled * 2.0  # Reduced from 5.0

        self.last_position = current_pos.copy()
        self.progress += distance_traveled

        # Speed reward (only when on track)
        speed = np.linalg.norm(car_vel[:2])
        speed_reward = speed * 0.5 if is_on_track else 0.0

        # Track centering reward (encourage staying in middle)
        track_center_radius = (inner_radius + outer_radius) / 2.0
        distance_from_ideal = abs(distance_from_center - track_center_radius)
        centering_reward = 1.0 - (distance_from_ideal / 0.375)  # 0.375 = half track width
        centering_reward = max(0, centering_reward) if is_on_track else 0.0

        # Off-track penalty (much smaller)
        off_track_penalty = -5.0 if not is_on_track else 0.0

        # Combine rewards
        reward = (
        progress_reward +       # 0-0.4 per step
        speed_reward +          # 0-1.0 per step  
        centering_reward * 0.5 + # 0-0.5 per step
        off_track_penalty       # -5 when off track
        )

        return reward # Placeholder - replace with actual reward calculation
    
    def _is_terminated(self) -> bool:
        """
        Check if the episode should terminate (failure condition).
        
        Questions:
        - What constitutes a failure? (off track, crashed, etc.)
        - Should termination be immediate or gradual?
        
        Returns:
            terminated: True if episode should end due to failure
        """
        # TODO: Check termination conditions
        # When should the episode end in failure?
        # Think about: Distance from track, car orientation, etc.
           # Give more time at start
        if self.current_step < 20:  # Changed from 5
            return False

        car_pos, car_orn = p.getBasePositionAndOrientation(
        self.car_id, 
        physicsClientId=self.physics_client
        )

        current_pos = np.array(car_pos[:2]) 
        distance_from_center = np.linalg.norm(current_pos)

        inner_radius = self.config['track']['inner_radius']
        outer_radius = self.config['track']['outer_radius']

        # Add small buffer 
        is_off_track = (distance_from_center < inner_radius - 0.15 or
                    distance_from_center > outer_radius + 0.15)

        return is_off_track

    
    def _is_truncated(self) -> bool:
        """
        Check if the episode should be truncated (time limit).
        
        Note: Truncation is different from termination!
        - Termination = failure (agent did something wrong)
        - Truncation = time limit (episode just ran out of time)
        
        Returns:
            truncated: True if episode should end due to time limit
        """
        # TODO: Check if max steps reached
        return self.current_step >= self.max_episode_steps
    
    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict] = None,
    ) -> Tuple[np.ndarray, Dict]:
        """
        Reset the environment to initial state.
        
        Questions:
        - Should the track be the same every time or randomized?
        - Should the car start at the same position or vary?
        - What information should be returned in the info dict?
        
        Args:
            seed: Random seed for reproducibility
            options: Additional options for reset
            
        Returns:
            observation: Initial observation
            info: Additional information
        """
        # TODO: Reset step counter
        self.current_step = 0
        
        # TODO: Reset car to initial position
        # Where should the car start? (from config)
        self.current_step = 0
    
        spawn_pos = self.config['spawn']['position']
        spawn_orn = self.config['spawn']['orientation']

        p.resetBasePositionAndOrientation(
            self.car_id,
            spawn_pos, 
            spawn_orn, 
            physicsClientId=self.physics_client
        )

        p.resetBaseVelocity(self.car_id, [0, 0, 0], [0, 0, 0], physicsClientId=self.physics_client)

        self.last_position = np.array(spawn_pos[:2])
        self.progress = 0.0
        self.episode_reward = 0.0

        observation = self._get_observation()

        # DEBUG: Print info
        distance = np.linalg.norm(self.last_position)
        print(f"Reset at {spawn_pos[:2]}, distance from center: {distance:.3f}, "
                f"track range: [{self.config['track']['inner_radius']}, {self.config['track']['outer_radius']}]")

        info = {
            "is_success": False,
            "spawn_coords": spawn_pos
        }
        self.stagnation_counter = 0
        return observation, info
        
    def step(
            self,
            action: np.ndarray,
        ) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        Execute one step in the environment.
        
        This is the core function that connects actions to outcomes.
        
        Questions:
        - How do actions map to car controls?
        - How many physics steps should run per environment step?
        - What order should things happen? (action -> physics -> observation -> reward)
        
        Args:
            action: Action from the agent
            
        Returns:
            observation: New observation after step
            reward: Reward for this step
            terminated: Whether episode ended due to failure
            truncated: Whether episode ended due to time limit
            info: Additional information
        """
        # Increment step counter
        self.current_step += 1
        
        # Convert action to car controls
        # The TankDriveController expects forward_input and turn_input (both -1 to +1)
        # Action space is Box(-1, 1, shape=(2,)) so actions are already in the right range
        forward_input = float(action[0])  # -1 to +1
        turn_input = float(action[1])     # -1 to +1
        
        # Apply controls to car using velocities from config
        # Get max velocities from config
        max_linear_vel = self.config['physics']['car']['max_linear_velocity']  # 2.0 m/s
        max_angular_vel = self.config['physics']['car']['max_angular_velocity']  # 2.0 rad/s
        
        # Scale actions to max velocities
        linear_velocity = forward_input * max_linear_vel
        angular_velocity = turn_input * max_angular_vel
        
        # Get current car orientation to apply velocity in the right direction
        car_pos, car_orn = p.getBasePositionAndOrientation(self.car_id, physicsClientId=self.physics_client)
        car_yaw = p.getEulerFromQuaternion(car_orn)[2]
        
        # Convert linear velocity to x, y components based on car orientation
        vx = linear_velocity * np.cos(car_yaw)
        vy = linear_velocity * np.sin(car_yaw)
        
        # Set velocities: [vx, vy, vz] for linear, [wx, wy, wz] for angular
        p.resetBaseVelocity(
            self.car_id,
            linearVelocity=[vx, vy, 0],  # Move forward/backward in car's direction
            angularVelocity=[0, 0, angular_velocity],  # Turn around z-axis
            physicsClientId=self.physics_client
        )
        
        # Step physics simulation
        # Run multiple physics steps for smoother control
        # Physics timestep is 0.0333s (30 Hz), run 8 steps = ~0.27s per action
        for _ in range(8):
            p.stepSimulation(physicsClientId=self.physics_client)
        
        # Get new observation
        observation = self._get_observation()
        
        # Calculate reward
        reward = self._compute_reward(action)
        
        # Check termination and truncation
        terminated = self._is_terminated()
        truncated = self._is_truncated()
        
        # Prepare info dict
        # Useful information for debugging or logging
        car_vel, car_ang_vel = p.getBaseVelocity(self.car_id, physicsClientId=self.physics_client)

        # 2. Calculate linear speed using NumPy norm (only X and Y coordinates)
        speed = np.linalg.norm(car_vel[:2])

        # 3. Handle stagnation logic
        if speed < 0.1: 
            self.stagnation_counter += 1
        else:
            self.stagnation_counter = 0

        # 4. Check if the car is "lazy" and terminate if necessary
        if self.stagnation_counter > 50:
            terminated = True
            reward -= 10
        
        info = {
            "step": self.current_step,
            "speed": speed,
            "angular_velocity": car_ang_vel[2],
            "progress": self.progress,
            "is_success": self.progress >= 10.0,
            "is_stagnated": self.stagnation_counter > 50
        }
        
        return (
            observation,
            reward,
            terminated,
            truncated,
            info,
        )
        
    #CONFUSED
    def render(self):
        """
        Render the environment.
        
        Questions:
        - What should be rendered? (car, track, camera view?)
        - How do you render in PyBullet?
        - Should rendering affect performance?
        """
        if self.render_mode == "human":
            # TODO: PyBullet GUI rendering
            # How do you render in PyBullet GUI mode?
            # What camera settings might be useful?
            car_pos, _ = p.getBasePositionAndOrientation(self.car_id, physicsClientId=self.physics_client)
            p.resetDebugVisualizerCamera(
                cameraDistance=3.0,
                cameraYaw=50,
                cameraPitch=-35,
                cameraTargetPosition=car_pos,
                physicsClientId=self.physics_client
            )

            pass
        elif self.render_mode == "rgb_array":
            # TODO: Return RGB array for video recording
            # How do you capture an image from PyBullet?
            # Hint: p.getCameraImage()
            # Capture a virtual image from the car's perspective or a top-down view
            # Using the resolution from your YAML!
            width = self.config['camera']['resolution']['width']
            height = self.config['camera']['resolution']['height']
            
            view_matrix = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=[0, 0, 0], distance=5, yaw=0, pitch=-90, roll=0, upAxisIndex=2
            )
            proj_matrix = p.computeProjectionMatrixFOV(fov=90, aspect=width/height, nearVal=0.01, farVal=100)
            
            # This returns a list: [width, height, rgbPixels, depthPixels, segmentationMask]
            _, _, rgb, _, _ = p.getCameraImage(width, height, view_matrix, proj_matrix, physicsClientId=self.physics_client)
            
            # Reshape into a standard image format (height, width, 3)
            return np.reshape(rgb, (height, width, 4))[:, :, :3]
    
    def close(self):
        """
        Clean up resources.
        
        Questions:
        - What needs to be cleaned up?
        - What happens if you don't close properly?
        """
        # TODO: Disconnect from PyBullet
        p.disconnect(physicsClientId=self.physics_client)
        pass

