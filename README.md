The Crop Cop - ROS 1 Setup & Git Workflow Guide

Welcome to **The Crop Cop** project repository! This project runs on **ROS 1 Noetic** using `catkin`. Follow the instructions below to set up your local workspace and push your changes.

---

## 1. Environment Prerequisite

Ensure you are using **Ubuntu 20.04** (via WSL2 or dual boot) with **ROS Noetic** installed.

If you haven't set up your workspace environment yet, run:

sudo apt update && sudo apt install terminator -y
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

---

## 2. First-Time Setup & Repository Clone
Run these commands once to set up Git, clone the repository into your catkin workspace, build the project, and create your feature branch:
# Set up Git credentials
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
git config --global credential.helper store

# Clone the repository
cd ~/catkin_ws/src
git clone https://github.com/NataliaArosemena/TheCropCop.git

# Build the ROS 1 workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Create your personal feature branch
cd ~/catkin_ws/src/TheCropCop
git checkout -b feature/your-branch-name

---

## 3. Daily Workflow: Saving & Pushing Your Work
Whenever you make changes or add new scripts in the_crop_cop, save and push your feature branch to GitHub:
cd ~/catkin_ws/src/TheCropCop
git add .
git commit -m "Brief description of what was added or fixed"
git push -u origin feature/your-branch-name
Authentication Notes (First Push Only):
Username: Your GitHub
usernamePassword: Your GitHub Personal Access Token (PAT)
To generate a token: 
Go to https://github.com/settings/tokens
Select Generate new token (classic)
Check the repo box
Click Generate token and copy/save the token string.

---

## 4. Submitting a Pull Request (PR)
To merge your branch into main:
Go to https://github.com/NataliaArosemena/TheCropCop/pulls.
Click New pull request.
Set base branch to main and compare branch to your feature branch (feature/your-branch-name).
Click Create pull request, enter a short title describing what you added, and submit.

---

## 5. Daily Workflow: Syncing Latest Changes from main
Before starting work each day, bring in any updates merged by teammates:

cd ~/catkin_ws/src/TheCropCop

# Pull latest code from main
git checkout main
git pull origin main

# Merge updates into your working branch
git checkout feature/your-branch-name
git merge main

---
