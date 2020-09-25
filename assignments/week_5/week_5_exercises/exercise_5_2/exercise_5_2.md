# Exercise 5.2
This exercise covers how to use launch parameters in launch files and in nodes.

# Exercise Objective
Right now `printing_node` has some issues with it. It has two arbitrary values:
* the string being printed
* the number of times that string is printed

These values would be better suited as launch parameters, and not hardcoded.

# Parameters in a launch file
To add a parameter to a launch file, you use a `param` tag:
```xml
<param name="parameter_name" value="parameter_value"/>
```
(The `/>` ending just signifies that this tag doesn't have a closing pair, it's just a single tag)

The `param` tag has two attributes:
* `name`: the name of the parameter.
* `value`: the value of the parameter. It's type is automatically determined depending on the input.

Add two `param` tags to `launch_node.launch` in between the `<node>` tags, one that determines
the message being printed, and another which determines how many times it is printed.

# Parameters in a node
To actually use the parameters defined in the launch file, we need to use the `NodeHandle` 
and it's member function `getParam`. This function takes in two arguments:
* the name of the parameter. This is actually the "path" name of the parameter, but because
we are using private node handlers and the parameter is defined inside the node tag, it is 
simply the name of the parameter.
* the variable to assign the parameter value to.

Here is an example of `getParam` in action:
```c++
int number;
p_nh.getParam("number_parameter", number);
```

Declare two variables, the string and the number of prints, and use `getParam` to assign
them the values you used in the launch file. Use these variables in the print loop, and 
test changing the parameters to see that it works.