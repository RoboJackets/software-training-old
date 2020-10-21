# Exercise 5.1
This exercise covers how to create a launch file for node.

# Exercise Objective
Create a `.launch` file which starts the `printing_node`, which is already created for you.

A launch file structured with XML (like HTML): it is composed of tags, which have attributes and 
can contain variables and other tags.

Here is a basic overview of XML syntax: https://www.ibm.com/support/knowledgecenter/SSRJDU/reference/SCN_XML_Syntax_Rules.html

Also, to create a comment in XML, you use comment tags, `<!--` and `-->`:
```xml
<!-- This is a comment -->
```

# Structure of a launch file
Every launch file needs one pair of `<launch>` tags surrounding all it's contents. This acts as the "root" tag.
```xml
<launch>

</launch>
```

To make a launch file start node, you need a `<node>` tag inside the `<launch>` tags.
The node tag requires three attributes:
* `pkg`: which package the node can be found in
* `type`: the type of node to launch, this is the string value put in `ros::init` and the
 name of the executable defined in the CMakeLists file. These all MUST match.
* `name`: the name of this individual node. This is arbitrary, so choose a descriptive name.

Currently, our `printing_node.cpp` has the incorrect type in `ros::init`. Check the `CMakeLists.txt` to
see what the node is being built as (it's the first argument in the `add_executable` line), and change
the argument in `ros::init` to match.

The complete launch file should look something like this:
```xml
<launch>
    <node pkg="package_name" type="node_type" name="descriptive name">
    
    </node>
</launch>
```
Modify the node tag to make it launch the `printing_node` in the `week_5_exercises` package.

With this, after building the package and sourcing `devel/setup.bash`,
using `roslaunch week_5_exercises launch_node.launch` should result in the `printing_node` running.

# Output
If you looked closer at `printing_node.cpp`, you see that it should have resulted in 
`"You should see this 5 times!"` being printed five times to the console, but nothing happened.

This is because those strings *were* being printed, but not to the console. Instead, they were 
printed to some log file. What we need is to specify the `output` of a node in the launch file.

If you add `output="screen"` to the node tag's attributes and re-launch, you should see the strings
being printed to the console. By default the attribute is `output="log"`, so you need to change it.