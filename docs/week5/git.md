# Merge Conflicts

-   Two weeks ago, we talked about branches and how they can be used to avoid conflicts in the code
-   But sometimes, multiple people edit the same lines of code on the same branch. Git's powerful, but it's not magic.
-   A merge conflict occurs when you try to push code based on an old version of the software that conflicts with changes in the same lines already pushed.

![img](https://developer.atlassian.com/blog/2015/01/a-better-pull-request/merge-conflict.png)


# What does a merge conflict look like

<div class="NOTES">
Demonstrate example by setting up merge conflict beforehand, use the exact stuff on this slide

</div>

```
some source code...
<<<<<< HEAD
This is a change I made
==========
This is a different change someone else made in the same place
>>>>>> The hash of the other commit, something like fa54dd6f3...
The rest of the source that had not been touched
```

-   When you pull changes that lead to a merge conflict Git will notify which files have a merge conflict in them
-   In each file, Git inserts the above text to indicate a merge conflict
-   "This is a change I made" is in the branch you're on
-   "This is a different change&#x2026;" is from the branch you're merging


# So how do we deal with this?

-   One strategy is to just go through the file and manually reconcile the conflict
-   After compiling and testing your changes to make sure the new code works, you have to commit your merge resolution before being able to push your changes.


# KDiff3

![img](https://i.imgur.com/undWZmR.png)

```
$ git mergetool -t kdiff3
```

<div class="NOTES">
-   Use the up/down arrows (with the red tip) to navigate between conflicts
-   "Base" is the most recent common version of the file
-   "Local" is the current version of the file in the working tree
-   "Remote" is the version of the file being merged in from another branch / remote

</div>


# KDiff3

![img](https://i.imgur.com/6MTZ6eE.png)

<div class="NOTES">
-   Select the different versions of the conflicted area using the A, B, and C buttons
-   You can select more than one of these options, and kdiff3 will insert the different versions in the order you select the buttons.
-   You can always edit the final (merged) version of the file in the bottom section.

</div>