# What are we doing today?

-   An introduction to the *Standard Template Library*
    -   Built-in Data Structures
    -   Built-in Algorithms
-   Color Detection with the RJRobot


# The Standard Template Library (STL)

-   a collection of general purpose containers and algorithms that are included in C++
-   Containers store things; algorithms perform operations on those things
-   Everything we work with is declared under the `std` namespace.


# Containers

-   Data structures in the STL that store a collection of other objects are called *containters*
-   Examples of Containers include `std::set`, `std::priority_queue`, or `std::vector`. Today we'll look at the latter
-   Containers are parameterized by a *template arguments*, which dictate what type of data the container stores.


# Vectors

<div class="NOTES">
initializer lists require c++11 for vectors.

</div>

-   an object that can contain a variable number of other objects (like ArrayList in Java)
-   you can keep adding elements to the vector
-   order of insertion is maintained

```C++
#include <vector>

std::vector<int> numbers = {1, 2, 3};
```

-   In this example, the vector is passed the type `int` as its template argument, meaning it can only store type `int`


# Vector methods

<div class="NOTES">
Vectors can grow as items are added, whereas arrays are static.

</div>

| `at()` or `[]` | returns the value at a specific index |
| `front()`      | returns the first item                |
| `back()`       | returns the last item                 |
| `size()`       | returns the number of elements        |
| `clear()`      | removes all items                     |
| `insert()`     | inserts an item at a specific index   |
| `push_back()`  | adds a given element to the end       |
| `pop_back()`   | removes the last element              |


# Playing with Vectors

```C++
#include <vector>
#include <string>
#include <iostream>
using namespace std;
int main() {
      vector<string> trainers = {"Evan 1", "Evan 2", "Jason", "Andrew"};
      cout << "There are: " << trainers.size() << " trainers" << endl;

      trainers.push_back("Dallas");
      trainers.push_back("Woodward");

      cout << "Actually, there are: " << trainers.size() << " trainers" << endl;
}
```


# More vector examples

<div class="NOTES">
We'll get to begin() and end() later

</div>

```C++
#include <vector>
#include <string>
#include <iostream>
using namespace std;

int main() {
      vector<string> trainers = {"Evan 1", "Evan 2", "Jason", "Andrew"};
      trainers.push_back("Dallas");
      trainers.push_back("Woodward");

      //Can't forget about the OG trainer. He goes first
      trainers.insert(trainers.begin(), "Matt");
      trainers.insert(trainers.end(), "Sahit");

      //This part is a metaphor for something
      cout << "There are "  << trainers.size() << " trainers" << endl;
      cout << "The 1st trainer is " << trainers.at(0) << endl;
      cout << "The last trainer is " << trainers.at(trainers.size() -1) << endl;
}
```


# Iterators

-   An object that lets you access and modify objects in a container
-   Depending on their type, they will let you read or write data in the container, and move forwards, backwards, or both
-   The containers library includes the most common ones you'll need, but you can also write your own


# Iterator methods

<div class="NOTES">
picture upcoming. mention that the method should be called on the containing object

</div>

| `begin()`  | starts at the **first** item and moves **forwards** when incremented  |
| `end()`    | starts at the **last** item and moves **forwards** when incremented   |
| `rbegin()` | starts at the **last** item and moves **backwards** when incremented  |
| `rend()`   | starts at the **first** item and moves **backwards** when incremented |

-   NOTE: these methods are called on the container
    -   i.e. `container.begin()`


# Iterator Methods

![img](https://i.imgur.com/XOfZ5kf.png)


# Iterator operations

<div class="NOTES">
picture upcoming

</div>

| `*`  | gets the value at the current index |
| `++` | increments the iterator forwards    |
| `--` | decrements the iterator backwards   |


# Iterator practice

<div class="NOTES">
requires c++ 11.

</div>

```C++
vector<int> vec = {66,89,0,60,17,90,8};
vector<int>::iterator it = vec.begin();
```

![img](https://i.imgur.com/MTaVFFM.png)

```C++
vector<int> vec = {66,89,0,60,17,90,8};
vector<int>::iterator it = vec.begin();
it++;
```

![img](https://i.imgur.com/gOXGy4i.png)

```C++
vector<int> vec = {66,89,0,60,17,90,8};
vector<int>::iterator it = vec.begin();
it++;
it--;
```

![img](https://i.imgur.com/394eVwQ.png)

<div class="NOTES">
ask what is the result of this code. Call someone up to show where it will end up

</div>

```C++
vector<int> vec = {66,89,0,60,17,90,8};
vector<int>::reverse_iterator it = vec.rbegin();
it += 3;
```

![img](https://i.imgur.com/wrsXiAZ.png)

<div class="NOTES">
write some for loops using iterators and printing out the numbers

</div>

```C++
vector<int> vec = {66,89,0,60,17,90,8};
vector<int>::reverse_iterator it = vec.rbegin();
it += 3;
```

![img](https://i.imgur.com/BMO9nL9.png)


# Algorithms

-   algorithms is a header that includes useful operators that can be used on vectors, arrays and other containers
-   uses iterators to interact with these containers
-   three cateorgies of algorithm methods
    -   does not modify the container
    -   modifies the container
    -   modifies a destination container


# Modifies the container

| `sort()`        | sorts a container in increasing order            |
| `nth_element()` | finds the nth smallest element                   |
| `fill()`        | fills a container with copies of a given element |
| `transform()`   | manipulates each element using a function        |
| `reverse()`     | Reverses the order of the elements               |


# `nth_element` example

-   `std::nth_element(first, nth, last);`
-   `first` is an iterator to the beginning
-   `last` is an iterator to the end
-   `nth` is an iterator to the element you would want if the container were sorted
-   [Live example](http://cpp.sh/7o2bk)


# Algorithm examples

```C++
#include <vector>
#include <iterator>
#include <algorithm>

int main()
{
      std::vector<int> v{2, 1, 5, 4, 3};

      //this changes v to {1, 2, 3, 4, 5}
      std::sort(v.begin(), v.end()); 

      //this changes v to {5, 4, 3, 2, 1}
      std::reverse(v.begin(), v.end());
}
```


# Does not modify the container

| `count()`      | counts the number of items in a container that match a given item  |
| `find()`       | returns an iterator to the first element that matches a given item |
| `accumulate()` | sums all elements in a container                                   |

-   [How to use these algorithms](http://cpp.sh/73bu)


# Challenge

1.  Drive over the several gray strips, stopping at the black strip
2.  Determine which strip is the line sensor value
3.  Drive back to the median strip, to show that you have calculated it
4.  Notes
    -   Modify the code in `median_line`
    -   You don't know ahead of time how many colored sections there are
    -   This is kind of hard