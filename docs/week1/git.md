# What is Git?

-   Git is a "Distributed Version Control System" (DVCS)
-   It is the tool RoboJackets uses to collaborate on code
-   Git tracks changes made to files in your project, so you can access old versions at any time without fear of deleting or breaking something
-   Allows multiple developers to work on the same code
-   Working on different parts of code and then combining them creates unique challenges that cannot be handled by file sharing systems like Dropbox and Google Drive


# What is GitHub?

<div class="NOTES">
Publicly hosted Git server means that it has a bunch of Git repositories on its computers, just like how you will have Git repositories on your own computer

</div>

-   Git &ne; GitHub
-   GitHub is a publicly hosted Git server
-   It hosts our projects as "repositories" on the internet
    -   A repository is a group of files that Git collectively tracks
-   This saves us the hassle of hosting a server for fellow RoboJackets to obtain copies of the code
-   As you matriculate through Tech, you'll be building an online portfolio of your work on GitHub


# How do you find things on GitHub?

-   Repositories are stored at `https://github.com/<ORG-OR-USER-NAME>/<REPO-NAME>`
-   So all of our projects can be found at <https://github.com/RoboJackets>
-   Your personal projects will exist at `https://github.com/<YOUR-GH-USERNAME>`


## Create a GitHub account

-   [https://github.com/join](https://github.com/join%0A)
-   Please include *at least* your real first name

![img](https://i.imgur.com/0cdXQXW.png)


# Fork and star our repo

-   Navigate to <https://github.com/RoboJackets/software-training>

![img](https://i.imgur.com/3CtCTqj.png)


# What is a fork?

-   Problem: How do I try something out on a project without affecting the project itself?
-   Solution: Forks!
-   A fork is a copy of a repository that exists on your personal account
-   Changes made on a fork will not affect the original repository, and vice versa
-   Your fork will be present on `https://github.com/<YOUR-GH-USERNAME>/software-training`


# Install Git on your computer

-   Windows: <https://git-scm.com/downloads>
-   Mac OS:
    -   Install Homebrew first from <https://brew.sh/>
    -   Then run: `brew install git`
-   Ubuntu:
    -   Run `sudo apt install git`


# Git Config

-   These commands set up git to attribute your contributions to our code to you.

```shell
git config --global user.name "Your Name"
git config --global user.email "your@email.here"
# verify the change above was set
git config --get --global user.email
git config --get --global user.name
```


# Let's grab some code!

-   Go back to the webpage for our GitHub repository
-   Select the clone button, then copy the link displayed

![img](https://i.imgur.com/YjWCoqJ.png)


## Clone the repository

-   In your terminal navigate to your workspace
    
    ```shell
    git clone <PASTE-CLIPBOARD-CONTENTS-HERE>
    ```

-   If using Linux, use Ctrl+Shift+V to paste in terminal
-   The clone command downloads a project and its history
-   Tada! Your own working copy of RoboJackets code!


# Adding Remotes

-   These commands set you up to separately contribute to your fork while receiving updates from the original repository
-   You can find the link in the last command under the green 'clone or download' button on your FORK

```shell
cd ~/<REPOSITORY-FOLDER>
git remote add rj https://github.com/RoboJackets/software-training.git
git remote set-url origin https://github.com/<YOUR-GH-USERNAME>/software-training.git
git remote -v

```

-   Remotes tell git where to send/receive changes (AKA pull/push)
    -   When you want to download changes from the RJ repo, you will use `rj`
    -   When you want to upload changes to your fork of the repo, you will use `origin`


## Checking Remotes

Now when you do:

```shell
git remote -v
```

You should see:

```shell
origin  https://github.com/<YOUR-GH-USERNAME>/software-training.git (fetch)
origin  https://github.com/<YOUR-GH-USERNAME>/software-training.git (push)
rj  https://github.com/RoboJackets/software-training.git (fetch)
rj  https://github.com/RoboJackets/software-training.git (push)
```


# Let's Recap

-   What is a fork?
-   Why do we use remotes?
-   What does the clone command do?

<div class="NOTES">
1.  A git repo that is a copy of another repo
2.  It lets us specify which repo should changes go to
3.  Downloads a project and its history

</div>


# Things have changed in the main repo, what do we do?

-   We need a series of git commands that can update our fork and local repo with the new content in the upstream repo
-   All terminal git commands are in the form:
    
    ```shell
    git <command> <param1> <param2> ...
    git clone https://github.com/...
    git config <other params>
    ```


# Git pull

-   Remember: your current directory must be a repo to use git
-   We can keep our fork up to date with the `git pull` command
    
    ```shell
    cd software-training    # or wherever your repo is
    git pull rj master
    ```

-   What does this mean?
    -   `git pull`: pull from one GitHub repository into our current repository
    -   `rj`: the name of the repo that we are pulling from
    -   `master`: pull into the "master" branch of our current repository
-   Don't worry about branches yet; we will cover them shortly


# Let's make some of our own changes!

-   Open up a text editor of your choice and create a new file with some text in it
-   Save the file inside the repository
-   Now how do we make this file show up online for others to see?


# Time to make them official:

-   Git calls every version it stores a "commit"
-   In order to commit something, you must specify which changes you've made to the codebase should be in the commit
-   This is called "staging" your changes


# Git add

```shell
git add .
```

-   Note that the above command contains a period!
-   **The add command tells git to keep track of new files added in the directory**
-   git needs to be told which files to version control. git add puts the files on git's "stage". The stage is where files go before they are saved by git
-   git add takes in parameters for each file or directory to stage
    -   The period means all files in this directory and its subdirectories
    -   Generally in bash, a period is shorthand for the current directory


# Git commit

```shell
git commit -m "Added a file!"
```

-   `commit`: Commit currently staged changes to git
    -   This is making the change "permanent" (more on this later)
-   `-m "..."`: Commits require commit messages to label them
    -   This is an easy way to specify that message while creating the commit
-   'git status' can show you if you have any unstaged changes and what you can commit


## Quick note on Cli command options

-   `-<letter>` and `--<word>` are often times used to set values for specific values
-   In the previous example, `-m "..."` sets the message parameter to the value in quotes
-   `-m` and `--message` can be used interchangeably for `git commit`
-   Most commands support `-h` or `--help` to show how to use them


# Git push

```shell
git push origin master
```

-   `git push`: Command to push commits to another repository
    -   git push is what makes code public
-   `origin`: Name of the repo to push to (origin is referring to our fork)
-   `master`: Name of the branch to push to (still top secret material)


# Some notes about commits

![img](https://imgs.xkcd.com/comics/git_commit.png)

-   A good commit message is short but clearly explains what changes were made
    -   A good commit message makes it easy to see what changes could lead to your project not behaving properly


# Some notes about commits cont.

-   Things committed to Git are intended to stay as a permanent record of the repository history
    -   This doesn't mean bad commits can't be reverted
    -   This does mean that you should never commit things like passwords to git
    -   This does mean that good commit messages are important
-   Large files are impractical for Git to track
    -   It's best not to commit large files such as logs and videos.


# Let's Recap

<div class="NOTES">
Call on students to explain each

1.  downloads/pulls changes from a different repo to our own
2.  uploads/pushes changes from our repo to a different one
3.  git add
4.  git commit

</div>

-   What does `git pull` do?
-   What does `git push` do?
-   How do you "stage" your changes?
-   How do you make your changes "permanent"?


# Some Problems

-   Multiple developers work on the same code, at the same time
-   We want to maintain multiple versions of the same software in parallel
    -   How do I apply a security fix to an old version?
-   You want a safe place to test out changes, without affecting anyone else
-   Branching is `git`'s solution


# Branches

-   Branches are local copies of state
-   If you do work on a branch, it won't affect any other branches
-   Later on, we can 'merge' branches together (so work done on either can be combined).
-   Branches are what people have the most trouble with
    -   <span class="underline">**Ask questions *please*!**</span> This is a difficult topic.


## Remotes vs Branches vs Forks

-   A fork or clone does a full copy of the git repo
-   A remote is a pointer to another copy of this git repo
-   A repository can contain a collection of **branches**
-   Each branch is an independent copy of the code
-   You can only directly modify local branches, but:
    -   You can merge from a remote into your branch (pull)
    -   You can merge from your branch to a remote (push)


## Git as a Graph

-   Git history is a **graph**
-   A graph is simply a collection of nodes connected by edges
-   Each commit is a node on the graph
-   Subsequent commits are linked by edges


## Git as a Graph cont.

<div class="NOTES">
These tools are not installed by default.

</div>

-   Some useful tools to visualize your repo's git history as a graph:
    -   `gitk`
    -   `git` integrations for your favorite editor!


## Creating Branches

-   Let's start by creating multiple branches
-   Create branches with
    
    ```sh
    # git branch <BRANCH_NAME>
    
    # Create two branches, starting from the current commit
    
    git branch stableRj
    git branch betaRj
    ```


## Switching Branches

-   Only one branch can be worked on at a time
-   `git status` will display info on your current branch
-   `git checkout` lets you switch between branches
-   Let's checkout the `stableRj` branch now
    
    ```sh
    git status
    #> On branch master
    #> nothing to commit, working tree clean
    
    # git checkout <BRANCH_NAME>
    
    git checkout stableRj
    #> Switched to branch 'stableRj'
    
    git status
    #> On branch stableRj
    #> nothing to commit, working tree clean
    ```


## First stableRj Commit

<div class="NOTES">
Do something visual, e.x., draw on whiteboard, gitk

</div>

-   Let's add a commit to the `stableRj` branch
    
    ```sh
    echo "Stability counts!" > stable_release.txt
    git status
    git add -A
    git status
    git commit -m "Add stable_release.txt"
    git status
    ```
-   This will **not affect** any other branches
-   The other branches will stay behind on 'initial commit'


## First betaRj Commit

-   Let's make an experimental commit on the `betaRj` branch
    
    ```sh
    git checkout betaRj
    echo "This feature is unstable!" > beta_release.txt
    git add -A
    git commit -m "Add beta_release.txt"
    ```
-   Since the `stableRj` branch was **behind** the `betaRj` branch, the commit history has diverged
-   Play around and checkout the various branches!
    
    ```shell
    git checkout <BRANCH>
    ls
    ```
-   Notice, master has not moved at all (since no commits have been made on it)


## Let's Merge the Branches

<div class="NOTES">
Do something visual at end to show other branches are unchanged. When the students use vi, let them know i to insert, wq to save+quit

</div>

-   Let's bring the hard work from the `betaRj` branch onto the `stableRj` branch
-   Right now, the `stableRj` and the `betaRj` branch have 'diverged'
-   We need to bring them back together, this is called a **merge**
    
    ```sh
    # Checkout to the branch we want to merge **into** aka the "base branch"
    git checkout stableRj
    # Merge the branch we want (betaRj) into the current branch (stableRj)
    git merge betaRj
    # This will launch an editor, save and quit it to complete
    ```
-   This does not change any other branch


# Pull requests

-   A pull request (PR) is a request for a project owner to merge a branch from your fork into their repository
    
    ```fundamental
    	push         PR
    laptop -----> fork -----> upstream
      ^            |             |
      |            v             v
      |---------------------------
    	   pull
    ```
-   Request an owner to merge **from a branch on your fork**


## How to make a pull request

-   Push code locally stored on your computer to your fork on GitHub
    
    ```shell
    git push origin stableRj
    ```
-   Locate your fork on GitHub and click "New pull request"

![img](https://i.imgur.com/8xwEajp.jpg)


## How to make a pull request (continued)

-   Ensure that both repos and branches are correct
-   Click "Create pull request"

![img](https://i.imgur.com/gxUa2Zx.jpg)


# Overall Contribution Flow

<div class="NOTES">
"Pull request procedure may differ for your specific team"

</div>

```shell
# start on master
git checkout master

# Ensure we branch off from a recent version
git pull rj master

# create a new branch
git branch my-new-feature
git checkout my-new-feature

# Add commits with your work
git commit -m "Fix all of RoboJackets"

# push to a seperate branch on your fork
git push origin my-new-feature

# Go to github, and click 'new pull request'

# add updates by
git commit -m "Add missing files"
git push origin my-new-feature
```