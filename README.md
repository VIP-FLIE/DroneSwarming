# DroneSwarming
The central repository for the University of Delaware's VIP Drone team's Swarm subteam.


## Welcome! A few brief 'getting started' points:
_For all of this, don't hesistate to message me (Dan Mihovch) if you run into any problems or have any questions.
There will be (if not already), more documents explaining everything covered here in greater detail, including resources to help you get started._

### Communication of problems and questions:
Here we will use GitHub Issues to communicate problems and questions. For anything that must not be forgotten or buried in messages, please use GitHub Issues. It will allow us to keep everything tidy in the repo and not lose important threads. Of course, if something needs immediate attention, just use the GroupMe. I am thinking things like package requests, bug reports, long-standing issues, etc. should go in the Issues.

### Development Workflow:
Since the software stack used for robotics ROS2 is native to Linux, specifically Ubuntu, we will be doing our development on a remote Linux machine, likely the lab machine in Evans 153. You will most likely use VSCode with the remote server extension to connect to the remote machine through SSH. 

### Git(Hub) Etiquette:
This may seem pedantic, but trust that things can go real bad real quick without proper version control, and it isn't too hard to pick up some good habits. A few rules of thumb:
- We will operate on a `main <- dev <- feature` branch workflow. The most stable branch is main. The branch everybody should be pushing completed changes to and pulling new branches from is dev (development). Feature branches are for working on new features and should be merged back into dev once they are complete.
- NEVER commit to main, unless the team has reviewed and green-lit the changes. Main should be treated as 'the last version of the project that works'.
- When starting a fresh working session, always create a new branch from dev, and follow the feature-date-author naming convention. This just makes it easier to figure out the who, what, and when of the branch just by looking at a name in a list. For example, I am writing this doc on branch 'docs-sept6-dan'.
- Try to scope down your changes to the smallest functional unit possible. It is much easier to reason about a change that tackles one issue across a few files rather than sweeping changes across the entire codebase. This also comes with the practical upshot of changes _usually_ being small enough to be merged without conflicts.
- If someone messes this process up (you will, I will, we all will), best practice is to revert to the last known 'safe' version of the codebase. If you don't know how to get back to that point (also totally okay!), please ask for help ASAP. Git issues are an infinite rabbit hole of horror and despair, the closer to ground level we address them, the easier it is to get back on track.

## Dev Container Testing
- Ubuntu 24.04 LTS (X11): Not tested
- Ubuntu 24.04 LTS (wayland): WIP 
- MacOS x86 Intel: WIP, fails
- MacOS Mx/ARM: Not tested
- Windows 11 w/ Nvidia GPU: Not tested
- Windows 11 w/o Nvidia GPU: Not tested
