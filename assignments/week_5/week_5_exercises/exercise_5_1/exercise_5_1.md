# Exercise 5.1
This exercise covers how to create a launch file for node.

# Exercise Objective
Create a `.launch` file which starts the `printing_node`, which is already created for you.

A launch file structured with XML (like HTML): it is composed of tags, which have attributes and 
can contain variables and other tags.

# Structure of a launch file
Every launch file needs `<launch>` tags surrounding all it's contents:

```xml
<launch>

</launch>
```

To make a launch file start node, you need a `<node>` tag inside the `<launch>` tags.
The node tag requires three attributes:
* `pkg`: which package the node can be found in
* `type`: the type of node to launch, this is the string value put in `ros::init`
* `name`: the name of this individual node. There can be multiple nodes with the same type,
but no two nodes can have the same name.

There is also an optional attribute `output`, which determines where the node should
print messages. For this exercise, you want `output="screen"`

The complete tag should look something like this:

```xml
<node pkg="package_name" type="node_type" name="descriptive name" output="screen">

</node>
```

Modify the node tag to make it launch the `printing_node` in the `week_5_exercises` package.

With this, using `roslaunch week_5_exercises launch_node.launch` should result in `printing_node`
running, which should print some values to the console!