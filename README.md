# 2024-robot



## Driver Controls:

```     
           Fast                           Amp
           Mode                          Wind-Up
      Smart  \                             /  Speaker
      Intake  \                           /   Wind-Up
          \__  \__         Gyro Reset  __/  ___/
             \ |  |               \   |  | /
   Drive     _##LB##_   Interrupt  \_##RB##_
      \     /  ___   \_______\_____/\   Y---\---- Climber Up
       \___/__/ . \           \      `X   B  \       
          /   \___/      o (X) o        A-----\-- Shoot   
         /    ^                          ___   \  
Climber |   <   >     _____________     / . \___|___,- Turn
 Climb -|-----v      /             \    \___/   |
         \__________/               \__________/
```

| Button | Function |
|-|-|
| Start | Interrupt |
| LT | *FAST MODE* |
| LB | Smart Intake |
| LS | Drive |
| RT | Amp Shot Wind Up |
| RB | Speaker Shot Wind Up |
| RS | Turn |
| X  | Gyro Reset |
| Y  | Climber Up |
| A  | Shoot |
| B  | |
| DD | Climber Climb |


## Operator Controls:

```     
                                          Amp 
                                         Wind-Up
       Manual                              /  Speaker
   Climber Override           Amp bar     /   Wind-Up
          \__   __              Home   __/  ___/
  Manual     \ |  |               \   |  | /
Left Climber _##LB##_   Interrupt  \_##RB##_
      \     /  ___   \_______\_____/\   Y---\---- Amp Bar Up
Manual \___/__/ . \           \      `X   B--\----- Manual Outake
Amp Bar   /   \___/      o (X) o        A-----\-- Manual Intake   
  Up  `--/----^                          ___   \     __  Manual
Manual  |   <   >     _____________     / . \___|___/  Right Climber
Amp Bar-|-----v      /             \    \___/   |
Down     \__________/               \__________/
```

| Button | Function |
|-|-|
| Start | Interrupt |
| LT | |
| LB | Manual Climber Override |
| LS | Manual Left Climber (with override) |
| RT | Amp Shot Wind Up |
| RB | Speaker Shot Wind Up |
| RS | Manual Right Climer (with override) |
| X  | Amp Bar Home |
| Y  | Amp Bar Up |
| A  | Manual Intake |
| B  | Manual Outake |
| DU | Manual Amp Bar Up |
| DD | Manual Amp Bar Down|