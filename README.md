# For ECE 238 project 1 (MontainCar) & part of 2 (Pendubot)

Author: Chun-yu Chang, Tianyi (Bruce) Chen ([Site](https://tybrucechen.github.io/))

Category:
  * [Project 1: Mountaincar with TO](#project-1)
  * Project 2: Pendubot
    * [TO/ LQR](#project-2) (LQR learn from swing-up TO trajectory, failed to keep upright states)
    * [TO/ PFL](https://github.com/TyBruceChen/Pendubot-upright) (TO swing up, PFL take over nearing up-right to keep upright)

### Project 1: 
### TO controlled MountainCar ([/MountainCar/MAIN.m](/MountainCar/MAIN.m))

States and Input:

<img width="560" height="350" alt="image" src="https://github.com/user-attachments/assets/4fe0c6bd-47f2-4212-b29d-f4398b2fa759" />

Final State (Animation) solved by ObjectTraj:

<img width="560" height="350" alt="image" src="https://github.com/user-attachments/assets/55562a99-09e9-4cc9-b6bd-33566b425efb" />

### Project 2: 
### LQR optimized Pendubot ([/Pendubot/main_pendubot.m](/Pendubot/main_pendubot.m))

States and Input:

<img width="550" height="350" alt="image" src="https://github.com/user-attachments/assets/f5ab6c2b-336f-4a76-9d0c-a875f7df71d9" />

Final State (Animation) from ode45:

<img width="360" height="350" alt="image" src="https://github.com/user-attachments/assets/1a04c779-e1fd-4371-9761-670866594ae3" />

