State Management.


What is autonomous?

Autonomous can be divided into to components - 

- Autonomous sequence describes a list of sequential instructions for the robot to execute.
- Autonomous execution receives the list of instructions and executes them.

In a conventional autonomous program, these two functionalities are combined.

> However, this paradigm makes it hard to differentiate between a robot correctly executing the wrong instructions and a robot failing to execute the correct instruction.

    > If I am missing a ball in autonomous, how do I know if the target coordinate was incorrect or if my drive code was unable to reach that point?

> Additionally, it makes it difficult to separate an autonomous sequence into individual, independent components. As explained in our principles, running an entire sequence start-to-finish just to test one movement drastically increases testing time and decreases code reliability.
    > Therefore, instructions in a sequence should be independent of the instructions before it and after. Each instruction should be able to exist in isolation to be tested more effectively.

These two concerns should be separated.

What does that mean?

- The autonomous sequence should be responsible purely for the *data* and *computations*, and should be **inert** by itself - it should not carry out actions by itself.

- The autonomous execution should be responsible purely for enacting whatever instructions are fed into it in the form of an autonomous sequence. It should not know about or be impartial to the actual contents of the instructions themselves.  

If the autonomous were a printer, the sequence would be the file contents, and the execution would be the mechanical components and circuitry. Both exist independently of each other, but are inert in isolation.

## Inspiration: Declarative UI In React, Flutter.

Many ideas behind our autonomous task system were heavily inspired by the declarative UI model found in frameworks like 
React, Vue and Flutter. Surprisingly, frontend web development shares alot in common with programming an autonomous movement. Both systems need to bring together simple, individual components into a coherent and complex entity. Be it buttons and textboxes or driving and shooting commands, the concept is the same. 

### Seperating Instructions And Executions

First let's take a look how this is done in Declarative UI:

- For example, to render a screen in React-Native:
Firstly, the app is a function that returns a list of nested components.
![AppStack](https://cdn.discordapp.com/attachments/827938039531044895/837137767116701696/unknown.png)

Each component is an extension of the component base class, potentially with its own internal state:
![ComponentClass](https://cdn.discordapp.com/attachments/827938039531044895/837138869585313866/unknown.png)


Each component is an extension of the component base class, potentially with its own internal state:
![ComponentClass](https://cdn.discordapp.com/attachments/827938039531044895/837138869585313866/unknown.png)

Each component also implements the `render()` function, returning the instructions to the create the component itself:
![RenderFunction](https://cdn.discordapp.com/attachments/827938039531044895/837139531157078087/unknown.png)

To build the screen, the top level execution process then sequentially calls each component's `render()` function. As described before, the component tree only contains the instructions to build the widget, and a separate process is responsible for executing those instrucitons. Because of this, any runtime errors are able to immediately be traced to the individual component that caused it. 


### Autonomous Task Tree

In a similar way, our autonomous is build upon a component system that builds the instructions to be run from the ground up. Our autonomous sequence is simply a nested list of tasks that are created with the given arguments:

![AutotaskTree](https://cdn.discordapp.com/attachments/776573717110325249/837149539806609458/unknown.png)

Just as the `render()` function defines the behavior of a widget, each task is comprised of four central lambda functions: `Run(), Done(), Init()` and `Kill()`. At the top level, the autonomous sequence control iterates through the vector to sequentially execute the instructions. 

![AutoSequence](https://cdn.discordapp.com/attachments/776573717110325249/837151128231084073/unknown.png)

Starting from the start of the list, it checks whether the task has been initialized, and if it hasn't it calls the task's `Init()` method to initialize the task. This method is generally used to set the parameters the task refers to during its run time, such as initial coordinates or timestamps. 

It then calls the task's `Run()` method, which is the body of the task. For example, for drive tasks, this will calculate the path the robot needs to follow to reach its destination and sets robot's drive motors to those values. The `Run()` method is called during every loop of the auto sequence, which occurs every 20 milliseconds.

Then the execution checks the task's `Done()` function, which returns a boolean to determine if the task has successfully completed. For example, a drive task will return `true` upon reaching its destination. If the task has not completed, it will remain at the top of the stack and the cycle will be repeated. 

Finally, upon the completion of the top task in the stack, the task's `Kill()` method is called, to set the motors involved to their resting states and free up the memory allocated to the task during its runtime. The task is then popped off the top of the stack, so during the next iteration the next instruction in the list is processed.

