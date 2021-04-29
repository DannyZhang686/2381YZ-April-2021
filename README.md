# 2381 Robotics Programming Thesis

## Our Codebase From First Principles:

<h1><b> Why Reading This Is Worth Your Time </b></h1>
<p>
If there's an unspoken truth for anyone wanting to learn to code, it's that software documentation is universally intimidating and opaque at first glance. As a programmer by trade, it's easy to overlook the meticulous paradigms that cause the simplest packages to have hundreds or thousands of pages of documentation, covering each feature down to the last detail.


This document is meant to be the opposite. My purpose in writing this is not to go into each top-level specification we have written, but rather introduce our codebase from the very bottom; to illustrate how each major decision was made from first principles and first principles alone. Overall, the goal is less to demonstrate _what_ we have written, but instead paint a picture of _why_ we wrote our code the way we did.

Therefore, each section of this document will describe _why_ it is worth reading in the first place and _where_ it fits in the context of our project overall. As a wise person once said:

</p>

> Reading documentation is f'n tedious. \
> Writing documentation is even more f'n tedious.
>
> --<cite>A Wise Person</cite>

First, we must identify the singular goal that our codebase is trying
to achieve.

# <b> The Zeroeth Principle - Effectiveness</b>

Ultimately, the purpose of a robotics codebase is to be effective at performing the actions needed to win given the particular ruleset of the game.

## Capability versus Consistency:

O    This can be divided into the two pillars that determine effectiveness:

    - Capability

    - Consistency

    The goal of a successful codebase is thus to maximize these two criteria.

<b>Goal: To Maximize Codebase Effectiveness by Maximizing Capability and Consistency</b>

# <b>Resource Allocation </b>

Limiting Resources:

In order for code to reach both consistency and capability, time needs to be taken to write the code and test the results.

- Programming Time
- Testing Time

### Programming Vs Testing Requirements:

What is necessary to program?

- Your laptop
- Maybe an internet connection

What is necessary to test a program?

- A partially, substantially or fully-complete robot
- A large space to test (generally a field)
- Game materials, up to and including a fully set-up field depending on the test
- Charged batteries
- Code needs to be uploaded each time a change is made

> Generally, 30 minutes to an hour of setup time is required for every 5 minutes of conclusive testing.
> A 1-minute Autonomous Skills Run takes around 10 minutes to setup.


Out of these two variables, there is almost never a
case where Programming Time is the limiting factor. Overwhelmingly, the case in robotics is rather that you are able to
write far, far more code than you are able to test.

It is for this reason that I see programming in robotics as a problem whose fundamentals lie in using time spent programming to maximizing the efficiency of testing. The team that is able to get the most out of their time testing their creations will have the most effective codebase and subsequently, the most effective robot.
<!-- add a transition here?? -->
<b>Goal: To Maximize Codebase Effectiveness While Minimizing Testing Time Necessary</b>

# <b> Robot Testing Categories: </b>

What does testing consist of?
Well, it can be broken down into three main components:

- Trialing: The code is behaving as expected, but we are evaluating it's _consistency_.
- Tuning: The code is behaving as expected, but we are evaluating its' _performance_
- Debugging: We have no idea what the f*ck the code is doing.

Going through these three categories of testing,
trialing is universally necessary, because the only certainty you ever have in robotics is empirical evidence. Doesn't matter if the bridge checks off with the math on paper if it collapses in real life. Even perfect code needs to be trialed, so there is less room for optimization here\*.

Tuning is generally useful, but is not _strictly_ necessary. Many optimizations can be made with self-tuning code.

Debugging, however, is by far and away the largest source of wasted testing time. The reason for this is completely unexpected behavior of code provides no indication of what is going wrong. If you make a change expecting the robot to turn faster, but the intake stops working - where do you even start? Debugging is frustrating, time consuming and those that spend long enough staring into the abyss are driven into insanity. However, debugging is not unavoidable.

Therefore:

<b>

1. An Effective Codebase Is Designed To Maximize Robot Capability And Consistency.

2. An Effective Codebase Uses Programming Time In Order
   To Maximize Testing Efficiency.

1. An Effective Codebase Attempts To Eliminate To Whatever Degree Possible The Amount Of Debugging During The Programming Cycle.

</b>

---

So far, we've covered the shape of _what_ an effective codebase looks like. The next question then becomes:

## How do we achieve these goals? How do these ideas apply practically? 

# <b>2381Y Guiding Principles</b>

Again to clearly define our goals:

1. ## PREVENT ERRORS
2. ## IDENTIFY ERRORS
3. ## SOLVE ERRORS AND EXPAND FUNCTIONALITY

<!-- 

* ## RESILIENT
  - MINIMIZE THE WAYS THE CODE CAN BREAK
  - Minimize external dependencies - 
  - Identify and account for expected variances

* ## INTUITIVE
  - MAKE THE CODEBASE AS EASY TO UNDERSTAND AND REASON THROUGH AS POSSIBLE 
  - Categorize necessary external dependencies

* ## AGILITY
  - FACILITATE SOLVING ERRORS AND EXPANDING THE CODEBASE -->
Therefore, we want to incorporate the following themes within our codebase:

* ## INTELLIGENCE
* ## MODULARITY
* ## AGENCY

--- 

## <b>MODULARITY</b>

> Modularity: The degree to which a system's components can be assembled, separated and recombined.

### Why Modularity?

Problem Statement: If we have a codebase free of errors, how can we best write new code and implement new features while minimizing the number of new errors introduced?

- Modularity enables previously tested code to be reused in new components, decreasing the volume of untested functionality introduced with each new feature. 
- Modularity enables previous testing time to be leveraged to reduce the amount of present and future testing necessary.

## <b>AGENCY</b>

What is modular agency? Agency is a concept we employ in order as an answer to the following question:

Problem Statement: Errors can be reduced, but never eliminated completely. If there are errors, how can we quickly identify and resolve them?

- In an ideal world, as soon as an issue is identified, it can immediately be traced back to its root in the source code and fixed.

- What prevents this from happening? 
  1. Overlapping Actions : When several pieces of code all perform the same function but with different implementations, each implementation needs to be tested one at a time.
  2. Inseparable Dependencies : The most effective way to debug is to test potential sources of errors in isolation. Code that is either relying on unrelated variables and functions, or hardcoded into a large convoluted system, cannot be isolated and is incredibly tedious to debug.

Definitions: 

> Module Responsibility: The set of duties and obligations that a module is liable for: _If there's an error, how can we trace it back to its source?_

> Module Authority: The set of abilities and actions a module is permitted to perform: _What should this codeblock be able to do, and what shouldn't it be able to do?_

> Module Agency: The combined scope of what a module is permitted to perform and what it is expected to perform. 

Finally, our Key Guidelines for what makes an effective codebase:

- ## SEPARATE independent responsibilities
- ## MIRROR The scope of responsibilities and authority
- ## MINIMIZE module authority to whatever extent possible

These will be present in the documentation and explanation of our codebase in the following two sections. 