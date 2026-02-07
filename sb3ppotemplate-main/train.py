"""
Training Script for SB3 PPO Self-Driving Car Agent

This script sets up and trains a PPO agent using Stable-Baselines3.
Students should fill in the hyperparameters and training configuration.

Questions to consider:
- What hyperparameters affect learning speed and stability?
- How do you know if training is working?
- What's the difference between learning rate and batch size?
- How long should training run?
"""

import os
import yaml
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

from environment import SelfDrivingCarEnv


def create_env(config_path: str, render_mode: str = None):
    """
    Create and wrap the environment.
    
    Questions:
    - Why do we wrap environments in VecEnv?
    - What's the difference between DummyVecEnv and SubprocVecEnv?
    - When would you use multiple parallel environments?
    
    Args:
        config_path: Path to environment config file
        render_mode: Rendering mode (None for training)
        
    Returns:
        env: Wrapped environment
    """
    def _make_env():
        # TODO: Wrap environment with Monitor for logging
        # What does Monitor do? Why is it useful?
        env = SelfDrivingCarEnv(config_path=config_path, render_mode=render_mode)
        env = Monitor(env)
        return env
    
    # TODO: Create vectorized environment
    # Why use DummyVecEnv? When would you use SubprocVecEnv?
    env = DummyVecEnv([_make_env])
    
    return env  # Placeholder


def load_config(config_path: str = "setup/track_config.yaml"):
    """
    Load training configuration.
    
    Questions:
    - What hyperparameters should be configurable?
    - Should you hardcode values or use config files?
    
    Args:
        config_path: Path to config file
        
    Returns:
        config: Configuration dictionary
    """
    # TODO: Load config file or define hyperparameters
    # Should you add training hyperparameters to the YAML file?
    # Or define them here as constants?
    
    config = {
        # TODO: Define PPO hyperparameters
        # What values should these have? How do you choose?
        "learning_rate": 3e-4,  # How fast should the agent learn?
        "n_steps": 2048,  # How many steps before updating?
        "batch_size": 64,  # How many samples per update?
        "n_epochs": 10,  # How many times to restudy same data 
        "gamma": 0.99,  # Discount factor - how much do future rewards matter? -- future rewards matter
        "gae_lambda": 0.95,  # GAE parameter - what does this control?
        "clip_range": 0.2,  # PPO clipping - why is this important?
        "ent_coef": 0.01,  # Entropy coefficient - exploration vs exploitation - exploration
        "vf_coef": 0.5,  # Value function coefficient balances how much the agent cares about predicting how much reward it will get versus actually getting the reward.
        "max_grad_norm": 0.5,  # Gradient clipping - why use this? what max update cam be -- so not so large 
        
        # TODO: Define training settings
        "total_timesteps": 200000,  # How long to train?
        "eval_freq": 10000,  # How often to evaluate?
        "n_eval_episodes": 5,  # How many episodes for evaluation?
    }
    
    return config


def main():
    """
    Main training function.
    
    Questions:
    - How do you initialize a PPO agent?
    - What callbacks are useful during training?
    - How do you save and load models?
    - What metrics should you track?
    """
    
    # TODO: Set up paths
    # Where should models be saved? Logs?
    models_dir = "models/"
    logs_dir = "logs/"
    os.makedirs(models_dir, exist_ok=True)
    os.makedirs(logs_dir, exist_ok=True)
    
    # TODO: Load configuration
    config = load_config()
    
    # TODO: Create training environment
    # What render mode should you use for training? Why?
    train_env = create_env("setup/track_config.yaml", render_mode=None)
    
    # TODO: Create evaluation environment
    # Should evaluation use a different render mode?
    eval_env = create_env("setup/track_config.yaml", render_mode=None)
    
    # TODO: Initialize PPO agent
    # What parameters does PPO need?
    # Think about: policy type, environment, learning rate, etc.
    model = PPO(
        policy="MlpPolicy",  # What other policies are available?
        env=train_env,
        learning_rate=config["learning_rate"],
        n_steps=config["n_steps"],
        batch_size=config["batch_size"],
        n_epochs=config["n_epochs"],
        gamma=config["gamma"],
        gae_lambda=config["gae_lambda"],
        clip_range=config["clip_range"],
        ent_coef=config["ent_coef"],
        verbose=1,
        tensorboard_log=logs_dir,
    )
    
    # TODO: Set up callbacks
    # What callbacks are useful?
    # - EvalCallback: Evaluate during training
    # - CheckpointCallback: Save model checkpoints
    callbacks = []
    
    # TODO: Create evaluation callback
    # How often should you evaluate? What metrics matter?
    count=1
    eval_callback = EvalCallback(
       eval_env,
        best_model_save_path = models_dir,
        log_path=logs_dir,
        eval_freq=config["eval_freq"],
        deterministic=True,  # Should evaluation be deterministic?
        render=False, # Should you render during evaluation?
    )
    callbacks.append(eval_callback)
    
    # TODO: Create checkpoint callback
    # How often should you save checkpoints?
    checkpoint_callback = CheckpointCallback(
        save_freq=50_000,
        save_path=models_dir,
        name_prefix="ppo_car",
    )
    callbacks.append(checkpoint_callback)
    
    # TODO: Train the model
    # How do you start training?
    # What does learn() return?
    model.learn(
        total_timesteps=config["total_timesteps"],
        callback=[eval_callback, checkpoint_callback],
        progress_bar=True,  # Should you show progress bar?
    )
    
    # TODO: Save final model
    # Where should the final model be saved?
    model.save(os.path.join(models_dir, "final_model"))
    
    # TODO: Clean up
    # What needs to be closed?
    train_env.close()
    eval_env.close()
    
    print("Training complete! Check the models/ directory for saved models.")


if __name__ == "__main__":
    main()

