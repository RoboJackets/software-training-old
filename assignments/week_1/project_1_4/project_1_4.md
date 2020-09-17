# Project 1.4
Welcome to this first robotics themed projects of the RoboJackets software training
curriculum. The point of these projects is to give you hands on experience playing
around with code related to a robotics topic that we have covered in lecture.
I hope you enjoy this one

# Project Objective
During this project we will be training a Neural Network on the MNIST dataset.
This is the canonical hello world of the machine learning world. The goal is to
classify images of handwritten digits into digits they represent. We will be using a
convolutional neural network in order to effectively process the image data.

## Dataset
MNIST is made up of a lot of handwritten digits. They are put into 28x28 pixel images.
There are 10 output classes that represent the digit the image corresponds to.

<img src="https://3qeqpr26caki16dnhd19sv6by6v-wpengine.netdna-ssl.com/wp-content/uploads/2019/02/Plot-of-a-Subset-of-Images-from-the-MNIST-Dataset.png" width="1280" height="960" />

## Convolutions
<img src="https://miro.medium.com/max/2340/1*Fw-ehcNBR9byHtho-Rxbtw.gif" width="1170" height="849" />

# Starter Code
We will be using something called Google Colab. This allows you to run a python notebook
on google servers using their GPU's. Often GPU setup is nontrivial and this allows us to
get up an running almost immediately. You would not want to use this for more general
work unless you do not have another option. Open up the file
[here](https://colab.research.google.com/drive/1IihcgCl13Nbui1NNhlwiv4ekWD808CRI?usp=sharing)
and click the button to add it to your own drive. There is also a complete
version of the code that can be found here in script form.

Today we will be using Python but you **DO NOT NEED TO KNOW PYTHON**. A lot of machine
learning libraries are implemented with python wrappers, hence why we are using python.
Don't worry though, you won't have to edit any of the code that isn't very explicit.

## Python Notebooks Basics

Each block is of the same code, just press the play button to run that section of code.
The results from each section are kept in memory, this allows you to run a further
along section of the code multiple times without constantly rerunning the initial sections.
If you change a section above another you will have to rerun it.

## Part 1 Running The Code
For this section you should run the entire notebook step by step. I would recommend
using the button at the top pictured here in order to make it easier on yourself.

<img src="https://i.imgur.com/L9S8vF7.png" width="1200" height="688.5" />

You should see output similar to this, the instructions inside the notebook should
cover any questions you might have about what each section of code is doing.
You should not read over each line of code unless instructed to do so, you won't
need to know how any of the code works in order to complete the exercise (I wrote
it and I barely know how it works).

You should see a graph similar to the one below at the final output.

## Part 2 Overfitting and Underfitting

<img src="https://user-images.githubusercontent.com/9097872/92522161-60648e00-f1ec-11ea-8083-34954585f2fd.png" width="1416" height="378" />

### Overfitting
Overfitting is the common word used for when you being to use features in the training
dataset that does not represent the test set. The graphic shows a great example.
Our model fits the data perfectly but it does not follow the general trendline at all.

### Underfitting
Underfitting is when the model does not have enough complexity to fit the data. Here
we have something close to x^2, but we are trying to use a linear function to fit it.
We can get okay results, but we will never be close to the actual function because
it is not well approximated with a linear funciton.

### Implementation

Now that we can train a simple model we want to try to increase out accuracy. Remember
how we discussed underfitting the model previously. Now our model has enough room to
accurate model the data (trust me on this one). So what else could we look at in order to
get a better model for the data? Consider the graph at the end, what should be
happening to know that we have maxed out our accuracy?

Remember that you should only have to change locations where it says change me.
I have also provided bounds on how much you should change certain variables, this
is mostly so you don't spend hours on this part.
Which one of those locations in the code listed below would be the right place
to solve this issue?

````python
learning_rate = 1e-6 # <=================== CHANGE ME HERE range:[1e-8, 1e-1]
num_epochs = 3       # <=================== CHANGE ME HERE range:[1, 20]
````

<details>
  <summary>Answer</summary>
    That graph tells me that we should be training for longer, out error curve
    has not leveled out yet, maybe just increasing
    the number of epochs would significantly improve out network.
</details>

<details>
  <summary>Answer</summary>
    That graph tells me that we should be training for longer, out error curve
    has not leveled out yet, maybe just increasing
    the number of epochs would significantly improve out network.
</details>

## Part 3 Speed Up Results
Now that we have gotten better results could we have gotten here faster? Try
thinking about what we should change in order to converge faster. Let's really put
the speed to the test and use the lower upper limit here. Are your results any better?
If results are bad, try starting at a known good value and increasing until
the performance starts to degrade significantly.

You should be getting accuracy above 90%.

<details>
  <summary>Answer</summary>
  A learning rate of 1e-3 seems to work well in practice with 3 epochs.
</details>

## Part 4 KMNIST
Now that we have a beautiful neural network for the current dataset let us try
something a little more difficult. We will be using KMNIST, a dataset very similar to MNIST.
The basics of the dataset are indentical in input. in order to get 10 classes they use
one character to represent each of the 10 rows of Hiragana when creating Kuzushiji-MNIST.
Examples of the correct classifcations are shown below.

<img src="https://storage.googleapis.com/tfds-data/visualization/fig/kmnist-3.0.1.png" width="900" height="900" />


### Implementation
Go back to the section where we loaded in the dataset and change the lines to be (just adding a K).

````python
## download and load training dataset
trainset = torchvision.datasets.KMNIST(root='./data', train=True,
                                      download=True, transform=transform)
trainloader = torch.utils.data.DataLoader(trainset, batch_size=BATCH_SIZE,
                                          shuffle=True, num_workers=2)

## download and load testing dataset
testset = torchvision.datasets.KMNIST(root='./data', train=False,
                                     download=True, transform=transform)
testloader = torch.utils.data.DataLoader(testset, batch_size=BATCH_SIZE,
                                         shuffle=False, num_workers=2)
````

Now try playing around with the new dataset and see if you can change values
to give accuracy above 91%.
[Training](https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.reddit.com%2Fr%2FProgrammerHumor%2Fcomments%2F9cu51a%2Fshamelessly_stolen_from_xkcd_credit_where_is_due%2F&psig=AOvVaw00gXQw1TnbHykteNddM-rV&ust=1599439002194000&source=images&cd=vfe&ved=0CAIQjRxqFwoTCNjm5-Ck0-sCFQAAAAAdAAAAABAg)
is a good time to check out the videos below
that give a better intuition into what is happening with these neural networks. This
guy has some fancy visualizations.

Here are the links to some videos

[This series](https://www.youtube.com/watch?v=aircAruvnKk&list=PLZHQObOWTQDNU6R1_67000Dx_ZCJB-3pi)
of videos will cover what we covered in this presentation in video 1 and 2.
Three and four go into how backpropagation actually works, be warned there is math.


