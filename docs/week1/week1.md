# What are we doing today?

-   Instructor introductions
-   Overview of the RoboJackets software training program
-   Navigating and manipulating files and directories in the Linux terminal
-   Using Git and GitHub for version control
-   Do a fun exercise to practice using git


# Meet the Intructors

-   **TODO:** Insert Fall 2018 Instructors


# Why are you here?

-   Learn the basics of software development
-   Get up to speed on technologies and techniques common to most RoboJackets teams


# Overview of Training

-   Week 0 training (**Insert dates**)
-   **TODO:** Anything we need to add here?


# The Linux terminal

-   Powerful text-based interface for interacting with your computer
-   (Almost) everything is a file or a folder
-   It lets you navigate your folders and manipulate files quickly
-   There are lots of tiny tools and commands you can use to do useful things


# ATTENTION WINDOWS USERS

-   Terminal is not the same as Command Prompt
-   You cannot use CMD as it has different commands than Linux
-   Instead use Git bash
    -   <https://www.atlassian.com/git/tutorials/install-git>


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