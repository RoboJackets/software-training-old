import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
import torchvision.transforms as transforms

BATCH_SIZE=32

## transformations
transform = transforms.Compose(
    [transforms.ToTensor()])

## download and load training dataset
trainset = torchvision.datasets.MNIST(root='./data', train=True,
                                      download=True, transform=transform)
trainloader = torch.utils.data.DataLoader(trainset, batch_size=BATCH_SIZE,
                                          shuffle=True, num_workers=2)

## download and load testing dataset
testset = torchvision.datasets.MNIST(root='./data', train=False,
                                     download=True, transform=transform)
testloader = torch.utils.data.DataLoader(testset, batch_size=BATCH_SIZE,
                                         shuffle=False, num_workers=2)

# ====================================================================

import matplotlib.pyplot as plt
import numpy as np


## functions to show an image
def imshow(img):
    #img = img / 2 + 0.5     # unnormalize
    npimg = img.numpy()
    plt.imshow(np.transpose(npimg, (1, 2, 0)))
    plt.show()

## get some random training images
dataiter = iter(trainloader)
images, labels = dataiter.next()

## show images
#imshow(torchvision.utils.make_grid(images)) TODO

# ====================================================================

class MyModel(nn.Module):
    def __init__(self):
        super(MyModel, self).__init__()

        # 28x28x1 => 26x26x32
        self.conv1 = nn.Conv2d(in_channels=1, out_channels=32, kernel_size=3)
        self.d1 = nn.Linear(26 * 26 * 32, 128)
        self.d2 = nn.Linear(128, 10)

    def forward(self, x):
        # 32x1x28x28 => 32x32x26x26
        x = self.conv1(x)
        x = F.relu(x)

        # flatten => 32 x (32*26*26)
        x = x.flatten(start_dim = 1)

        # 32 x (32*26*26) => 32x128
        x = self.d1(x)
        x = F.relu(x)

        # logits => 32x10
        logits = self.d2(x)
        out = F.softmax(logits, dim=1)
        return out

# ====================================================================

# run the random model through the data to make sure our ins and outs correspond to each other
#model = MyModel()
#for images, labels in trainloader:
#    print("batch size:", images.shape)
#    out = model(images)
#    print(out.shape)
#    break

# ====================================================================

## compute accuracy
def get_accuracy(logit, target, batch_size):
    ''' Obtain accuracy for training round '''
    corrects = (torch.max(logit, 1)[1].view(target.size()).data == target.data).sum()
    accuracy = 100.0 * corrects/batch_size
    return accuracy.item()


def insepct_detection(image, output):
    fig, axs = plt.subplots(2)
    fig.suptitle('Plots')
    axs[0].set_title("Image")
    image = np.transpose(image.numpy(), (1,2,0))
    #print(np.transpose(image.numpy(), (1,2,0)).shape)
    #imshow(image)
    axs[0].imshow(image)
    axs[1].bar(range(0,10), sample_output)
    axs[1].legend(loc='upper right', frameon=False)
    axs[1].set_title("Probability of digit")
    plt.show()

# ====================================================================

# possible exercises
# not running long enough
# learning rate = 1e-6
# num_epochs = 3

# run for longer
# num_epochs = 20

# Increase the learning rate
# learning_rate = 1e-1
# num_epochs = 3

# decrease the learning rate to get a better result on 3 epochs (> 90% accuracy)
# learning_rate = 1e-3
# num_epochs = 3

# increase the number of epochs, can we learn more?
# num_epochs = 20

# give them KMNIST and then have then see if they can beat my top accuracy
# 91

learning_rate = 1e-3 # <=================== CHANGE ME HERE
num_epochs = 50      # <=================== CHANGE ME HERE

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
model = MyModel()
model = model.to(device)
criterion = nn.CrossEntropyLoss()
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)


training_loss_plot = []
training_accuracy_plot = []
test_accuracy_plot = []

test_acc = 0.0
for i, (images, labels) in enumerate(testloader, 0):
    images = images.to(device)
    labels = labels.to(device)
    outputs = model(images)
    test_acc += get_accuracy(outputs, labels, BATCH_SIZE)
    if i == 0:
        image = torchvision.utils.make_grid(images.cpu()[0])
        sample_output = outputs[0].cpu().detach().numpy()
        insepct_detection(image, sample_output)


print('Initial Test Accuracy: %.2f'%( test_acc/i))

for epoch in range(num_epochs):
    train_running_loss = 0.0
    train_acc = 0.0

    model = model.train()

    ## training step
    for i, (images, labels) in enumerate(trainloader):

        images = images.to(device)
        labels = labels.to(device)

        ## forward + backprop + loss
        logits = model(images)
        loss = criterion(logits, labels)
        optimizer.zero_grad()
        loss.backward()

        ## update model params
        optimizer.step()

        train_running_loss += loss.detach().item()
        train_acc += get_accuracy(logits, labels, BATCH_SIZE)

    model.eval()
    average_train_loss = train_running_loss / i
    average_train_accuracy = train_acc / i
    training_loss_plot.append(100 - average_train_loss)
    training_accuracy_plot.append(100 - average_train_accuracy)
    print('Epoch: %d | Loss: %.4f | Train Accuracy: %.2f' \
          %(epoch, average_train_loss, average_train_accuracy))


    test_acc = 0.0
    for i, (images, labels) in enumerate(testloader, 0):
        images = images.to(device)
        labels = labels.to(device)
        outputs = model(images)
        test_acc += get_accuracy(outputs, labels, BATCH_SIZE)
        # only show result on the last epoch
        if i == 0 and (epoch+1 is num_epochs or epoch == 0):
            image = torchvision.utils.make_grid(images.cpu()[0])
            sample_output = outputs[0].cpu().detach().numpy()
            insepct_detection(image, sample_output)
    test_accuracy_plot.append(100 - test_acc/i)
    print('Test Accuracy: %.2f'%( test_acc/i))

# ====================================================================

plt.title("Training vs Test accuracy")
plt.plot(range(1,num_epochs+1), training_accuracy_plot, label='Training Error (%)')
plt.plot(range(1,num_epochs+1), test_accuracy_plot, label='Testing Error (%)')
plt.legend(loc='upper right', frameon=False)
plt.show()

# ====================================================================


