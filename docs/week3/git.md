# What You Know

<div class="NOTES">
Call on students to explain each

</div>

-   `git status`
-   `git add`
-   `git commit`
-   `git remote`
-   `git pull`
-   `git push`
-   *(any questions?)*


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