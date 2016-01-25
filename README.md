# Scripts to complete the "Turtlebot Setup and Getting Started" packet

## TO RUN:
  - Change into the root directory of this folder with ```cd Robotics-Setup```
  - The first time you run this file, you must run ```chmod u+x ./setup.sh ./setup_tmux.sh``` to get the correct permissions
  - Turn your robot on
  - Run the command ```. ./setup.sh```. This first dot is very important!
  - If you are prompted for a password, user the password you use to login to the laptop, not your netid password.
  - Run the command ```./setup_tmux.sh```

    That's it! If you are new to tmux, you can switch between panes in the terminal with your mouse. To close a terminal, use C-c to kill any process that is still running and then click C-b then x then y when it asks for confirmation. Make sure to close out of the last pane.

## PROBLEMS:
  - The group before you may have left tmux or ros sessions running. You will notice this when either
    - Many more tmux windows open than you were expecting 
    - The ros commands fail and output that they are already running
  - Solve these issues with the commands
  ```
  pkill -f tmux;
  pkill -f ros;
  ```
