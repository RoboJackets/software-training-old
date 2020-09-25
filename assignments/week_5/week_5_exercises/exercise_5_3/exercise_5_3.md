# Exercise 5.3
This exercise covers how we can use `yaml` files to create a list of parameters.

# Exercise Objective
In some cases, there's going to be far too many parameters to reasonably fit inside
of `<node>` tags. This is where `rosparam` and `yaml` files come in.

# Writing the yaml file
There is an empty `yaml` file already created for you in `week_5_exercises/config`. To fill
it with parameters, use the following syntax:
```yaml
parameter_1_name: value
parameter_2_name: value
```

This is easier to read than `<param>` tags, and can be more organized. With `yaml`, you 
can have parameters like this:
```yaml
category_1:
    parameter_a: value
    parameter_b: value
category_2:
    parameter_c: value
```
This means to read the the value of `parameter_a`, you'd use `getParam("category_1/parameter_a)`

For this exercise however, we can do with just a simple list. Put values for parameters inside 
config file that you use in the printing node.

# Reading from the yaml file
Firstly, since we are using a `yaml` file now, you can remove both `<param>` tags.

To read from the `yaml` file, you use a `<rosparam>` tag, which is a tag that can do a lot
of things with parameters. In this case we will be using its `"load"` functionality.

Here `<rosparam>` has two attributes:
* `command`: what you want the tag to do, in this case `"load"`
* `file`: where the `yaml` file is located. Here you can use a special string to locate the 
file path easier. Using `"$(find week_5_exercises)"` will give you the system path to the
package directory, so you can complete the path to the `yaml` file.

The complete `<rosparam>` tag should look like:
```xml
<rosparam command="load" file="$(find week_5_exercises)/config/node_config.yaml"/>
```

When finished, you should check to see that your node is reading the parameters from the
config file properly.