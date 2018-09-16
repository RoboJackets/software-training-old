# What are we doing today?

-   Instructor introductions
-   Overview of the RoboJackets software training program
-   Navigating and manipulating files and directories in the Linux terminal
-   Using Git and GitHub for version control
-   Do a fun exercise to practice using git


# Meet Evan

![img](https://raw.githubusercontent.com/RoboJackets/software-training/instructor-intros/images/evan-bretl-hiking.jpg)

-   Evan Bretl
    -   3rd year, Computer Science (Intelligence + Mod/Sim threads)
    -   Inside RoboJackets: Software Training Coordinator, RoboRacing Software Lead, former RoboRacing Project Manager
    -   Outside RoboJackets: Volleyball, AI Club
-   How to contact me
    -   Slack: [@Evan Bretl](https://robojackets.slack.com/team/U2AUQ6669)
    -   Email: [evan.bretl@gatech.edu](mailto:evan.bretl@gatech.edu)


# Meet Evan

![img](https://i.imgur.com/ol3f6LJ.jpg)

-   Evan Strat
    -   2nd year, Computer Science (Intelligence + Mod/Sim threads)
    -   Inside RoboJackets: IT Coordinator, IGVC Software
    -   Outside RoboJackets: Play clarinet in marching band
    -   More robot than human
-   How to contact me
    -   Slack: [@Evan Strat](https://robojackets.slack.com/team/U73AJTBPV)
    -   Email: [estrat@gatech.edu](mailto:estrat@gatech.edu)


# Meet Dallas

![img](https://raw.githubusercontent.com/RoboJackets/software-training/instructor-intros/images/dallas.png)

-   Dallas Downing
    -   3rd year, Computer Science (Intelligence + Media)
    -   Inside RJ: IGVC Project Manager, Outreach Mentor, Volunteer, PR Committee
    -   Outside RJ: various random hobbies
-   How to contact me
    -   Slack: [@dallas-d](https://robojackets.slack.com/team/U2E07GW77)
    -   Email: [dallas-d@gatech.edu](mailto:dallas-d@gatech.edu)


# Meet Andrew

![img](https://i.imgur.com/5Qa0VHl.jpg)

-   Andrew Tuttle
    -   3rd year, Computer Science (Theory, Intelligence)
    -   Inside RJ: IGVC Software Lead
    -   Outside RJ: Dungeon Master and general geek
-   How to contact me
    -   Slack: [@atuttle7](https://robojackets.slack.com/messages/@atuttle7/)
    -   Email: [atuttle7@gatech.edu](mailto:atuttle7@gatech.edu)


# Meet Jason

![img](https://i.imgur.com/izC5WWA.jpg)

-   Jason Gibson
    -   Senior, Computer Science (Threads: Devices, Intelligence)
    -   Inside RoboJackets: President
    -   Outside RoboJackets: Avid lover of dad jokes
-   How to contact me
    -   Slack: [@Jason Gibson](https://robojackets.slack.com/messages/@Jason_Gibson/)
    -   Email: [jgibson37@gatech.edu](mailto:jgibson37@gatech.edu)


# Why are you here?

-   Learn the basics of software development
-   Get up to speed on technologies and techniques common to most RoboJackets teams


# Overview of Training

-   Basic C syntax sessions
    -   Useful if you have never used a language with C-style syntax before
    -   We expect you to know C++ loops and basic data types
-   Week 2: C++ essentials, building on basic C syntax
-   Weeks 3-4: C++ Standard Template Library
-   Weeks 5-8: Advanced C++ topics (pointers, references, classes/interfaces)


# The Linux terminal

-   Powerful text-based interface for interacting with your computer
-   (Almost) everything is a file or a folder
-   It lets you navigate your folders and manipulate files quickly
-   There are lots of tiny tools and commands you can use to do useful things


# ATTENTION WINDOWS USERS

-   The Linux terminal is not the same as Windows Command Prompt
-   You will install Git Bash in order to use Linux commands
-   Installation instructions have been distributed to the training-sw email list


# Basic commands

<div class="NOTES">
Explain that a directory is more commonly called a folder. Go to your home directory. Create a directory. cd to it. Use echo to create a file. List the files with ls -al. Explain what . and .. are. Remove the file. cd up a level. Remove the directory.

</div>

| Command | Use                                    |
|------- |-------------------------------------- |
| `cd`    | Change to a different directory        |
| `ls`    | List files in this directory           |
| `mkdir` | Make a new directory                   |
| `rm`    | Remove a file                          |
| `rmdir` | Remove a directory                     |
| `echo`  | Write arguments to the standard output |

-   Note: folder = directory


# Basic commands

<div class="NOTES">
Create a directory with a file again, copy the file to the same folder with a different name, move the original up a level. Run man grep. Grep your file for a keyword. Run history.

</div>

| Command   | Use                                  |
|--------- |------------------------------------ |
| `cp`      | Copy a file                          |
| `mv`      | Move a file                          |
| `man`     | Access documentation about a command |
| `grep`    | Search for a string                  |
| `history` | Shows your command history           |


# Basic hotkeys

<div class="NOTES">
Copy/paste are generally ctrl+shift+c/ctrl+shift+v, respectively, but varies by shell.

</div>

-   **Up/down arrow keys:** Cycle through your command history (great for recent commands)
-   **Ctrl+R:** Search through your command history (great for less recent commands)
-   **Tab:** Complete this command/file/directory name
-   **Tab-Tab:** Show possible completions
-   **Ctrl+C:** *NOT COPY!* Stops the current command.
-   **Ctrl+Z:** *NOT UNDO!* Sends the current command to the background.
    -   Use `fg` to bring it back to the foreground.
-   **Ctrl+Shift+C:** Copy (in Linux)
-   **Ctrl+Shift+V:** Paste (in Linux)
-   **Ctrl+D:** Exit the shell.


# Git

<div class="NOTES">
Time to switch to the Git/GitHub presentation.

</div>

-   [Click here for this week's Git presentation](git.md)