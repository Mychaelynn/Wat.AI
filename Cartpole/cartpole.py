import gymnasium as gym
import random

# Use v1 for better performance/longevity
env = gym.make('CartPole-v1', render_mode='human') 
states = env.observation_space.shape[0]
actions = env.action_space.n

episodes = 10
for episode in range(1, episodes+1):
    # Added info handling for reset
    state, info = env.reset() 
    done = False
    score = 0 
    
    while not done:
        # Action is chosen randomly
        action = random.choice([0, 1])
        
        # Unpack 5 values instead of 4
        n_state, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        
        score += reward
        
    print('Episode:{} Score:{}'.format(episode, score))

env.close() # Cleanly close the rendering window