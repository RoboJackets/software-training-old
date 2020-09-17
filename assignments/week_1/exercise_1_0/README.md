# Week 1 Software Training: Exercise 1.0

Welcome to week 1 of software training! If you're here, you've the first set of exercises.

This exercise will serve as a tutorial of how to use git to set up your project.

## Installing the VM
RoboJackets' software teams use Linux (an operating system, like Windows or MacOS). We've set up a VM (virtual machine) that you can download and run on your computer - this will ensure that everyone is working with an identical setup. This VM contains most of the software you'll need to do work as a member of RoboJackets.

> Note: if you already have a Linux install or dual-boot setup, you can use it if and only if you are running Ubuntu 20.04. However, we do not support this use case as you will not have any of the software we require pre-installed.

The instructions for the VM are available [here]().

## GitHub classroom
GitHub classroom links each week's assignments to your GitHub account. We will use this feature to release new exercises each week and to track your progress in the course.

This week's assignment can be found at the following link:
[Week 1 Programming Exercise](https://classroom.github.com/a/HkIuITsO). The link will take you to a page where you will select your name to associate it with your GitHub account. If your name is not on this list, please post a message in #training-helpdesk or ask a trainer. If you accidentally select the wrong name from the list, please do the same.

After a minute, this will take you to a page that gives you a link like the following:
```
https://github.com/RoboJackets-Software-Training/week-1-programming-exercise-kylestach
```
This link will be the location of your assignment for Week 1. You may want to click the link and save it, although you should also receive an email with the link.

## Cloning the repository
Once you've got the VM installed and running, open up a terminal by clicking the following icon (or use the keyboard shortcut Control-Alt-T) inside of the VM:
![terminal_icon]()
Then, you should see the following prompt:
![terminal]()

This is a **bash shell**. You can use it to do a lot of things in your VM (or in a regular Linux installation), including running programs, editing and managing files, and more. It will be your primary way of interacting with your VM. All of the following terms are equivalent:
 - Terminal
 - Command line
 - Shell
 - Console (when referring to the input/output from a program)

Then, type the following command in:
```bash
git clone "<link to your fork of this repository>"
```
For example, for me (username `kylestach` on GitHub):
```bash
git clone "https://github.com/RoboJackets-Software-Training/week-1-programming-exercise-kylestach"
```

The shell maintains something called a "current directory" or "working directory". This represents the directory in which you are currently working.
> Note: a directory is an equivalent term to a folder in other operating systems.
Directories can hold files or other directories. For example, the directory `/home/robojackets/` is a directory called `robojackets` _inside of_ another directory called `home`.

Our `git clone` command created another directory. You can run `ls` to _list_ the files and directories in your current directory (run it by typing `ls` and hitting `Enter`). What is the name of the newly-created directory?

You can change your working directory using `cd` (_change directory_). Now, `cd` into the new directory by typing `cd <directory name>`.

> Note: you must type out the directory name _exactly_, including correct capitalization. To make it easier, you can start typing out the beginning of a file or directory name and press `Tab` to autocomplete with the names of files in the current working directory.

## Changing the program
Open up `hello_world.cpp` in Sublime Text - you can run it from the command line by running:
```cpp
subl hello_world.cpp
```

Fix the program so that it prints "Hello, world!" to the console.

## Building and running
We use a build system called CMake to build our code. In Codecademy, you typed in `g++` commands every time you wanted to recompile the code. This is easy when you have a program consisting of a single file, but for big programs the compilation process ends up being much more complicated. CMake is a way of organizing this.

To use CMake, first make sure you are in `/home/robojackets/week-1-programming-exercise-<name>`. The shell should display this by default, but you can also check your working directory using the `cwd` command.

CMake requires a _build directory_. This is where all of the finished (compiled) programs will go, as well as any temporary output required by the compiler. Go ahead and make a build directory using:
```bash
mkdir build
```
The `mkdir` command creates a new empty directory in the current directory. Then, change your current directory to be this `build` directory. Run CMake using:
```bash
cmake ..
```
> Note: `..` refers to the _previous_ directory. So, if your current directory is `/home/robojackets/week-1-programming-exercise-kylestach/build`, then `cmake ..` is the same as `/home/robojackets/week-1-programming-exercise-kylestach`.
CMake has two steps: a configuration step (you just ran this!) and a build step. You can run the build step using the command `make`.

You must run `make` every time you make a change to a source file (for now, just `.cpp` files). You need to rerun both `cmake` and `make` whenever you change a CMake configuration file - this usually happens when you add a new `.cpp` file.

## Submitting
To submit, you'll use `git` to commit your code to the master branch of your repository and push it to GitHub. GitHub classroom will then automatically grade your assignment and give you feedback in the form of a pull request.

That may be a lot of new terms (especially if you haven't used git or GitHub before), so let's walk through them one by one.

### 1. Setting up git
Some housekeeping things to do before we begin: you'll want to configure `git` so that you are recorded as the author of your own code. To do this, type the following commands:
```bash
git config --global user.email "your email address here"
git config --local user.name "your full name here"
```

### 2. Commit
Git is a tool that allows you to manage versions of your code. It allows multiple people to work simultaneously on a project, splitting off into different work streams and merging them together later.

The most fundamental object in Git is a _commit_. Each commit can be viewed as a snapshot of the project at a specific point in time. A commit can consist of modifications to existing files as well as adding or removing files from the project.

To create a commit, first make sure you're in the `week-1-programming-exercise-<username>` directory. Then, type the following:
```bash
git status
```
This should show you a list of files that are staged for commit (in green). Files in red are _unstaged_ which means that when you make a new commit the files will not be included (or an old version will be used).

You can add files to be committed using `git add`. Go ahead and add the files you changed (should just be the `hello_world.cpp` file from this exercise):
```bash
git add exercise_1_0/hello_world.cpp
```
Then, we can commit. Each commit has an associated _commit message_ that describes what changes you made from the previous version. You can use the `-m` flag to specify a commit message:
```bash
git commit -m "Completed exercise 1.0"
```

> Note: if you do `git commit` (without the `-m`) git will open a text editor for you to use to write the commit message. This may be useful for longer commit messages, but the default editor is `vim` which is notoriously difficult to use. If you want to use vim, great! Otherwise, if you accidentally open it (through `git commit` or otherwise), you can close it by typing `:wq` and then hitting `Enter`.

### 2. Pushing your code
Committing code only saves it _locally_ (on your computer or VM). However, you want to put it on the internet so other people can access it (in this case, you want us to be able to grade it!). To store code on the internet, we use _GitHub_, which you can think of as like Google Drive for code.

The process of transferring code from your local computer to the internet is called _pushing_. It's very simple:
```bash
git push
```
You will then be prompted for your username and password. Congratulations! You are now all set up.
