# 2381 Robotics Programming Thesis

## Our Codebase From First Principles:

<h1><b> Why Reading This Is Worth Your Time </b></h1>
<p>
If there's an unspoken truth for anyone wanting to learn to code, it's that software documentation is universally intimidating and opaque at first glance. As a programmer by trade, it's easy to overlook the meticulous paradigms that cause the simplest packages to have hundreds or thousands of pages of documentation, covering each feature down to the last detail.

This document is meant to be the opposite. My purpose in writing this is not to go into each top-level specification we have written, but rather develop our codebase from the very bottom; from first principles alone illustrate how each major decision was made. Overall, the goal is less to demonstrate _what_ we have written, but instead paint a picture of _why_ we wrote our code the way we did.

Therefore, each section of this document will contain first and foremost _why_ it is worth reading in the first place, and where it fits in the context of our project overall. As a wise person once said:

</p>

> Reading documentation is fucking tedious.
> Writing documentation is more fucking tedious.
>
> --<cite>A Wise Person</cite>

First we identify the singular goal our codebase is trying
to achieve.

# <b> The Zeroeth Principle - Effectiveness</b>

    Ultimately, the purpose of a robotics codebase is to be effective at performing the actions needed to win in the particular ruleset and game it is designed for.

## Capability V Consistency:

    This can be divided into the two pillars that determine effectiveness:

    Capability

    Consistency

    The overall goal of a successful codebase is to maximize these two criteria.

<b>Goal: To Maximize Codebase Effectiveness by Maximizing Capability and Consistency</b>

# <b>Resource Allocation </b>

Limiting Resources:

In order for code to reach both consistency and capability, time needs to be taken to write the code and to test the result.

- Programming Time
- Testing Time

### Programming Vs Testing Requirements:

What is necessary to program?

- Your Laptop
- Maybe an internet connection

What is necessary to test a program?

- A robot
- A large space to test (Most of the time a field)
- Game materials, possibly a fully set-up field for each test
- Charged batteries
- Code needs to be uploaded each time a change is made

\*\* Generally, for every 5 minutes of conclusive testing,
it requires somewhere around 30 minutes - 1 hour of setup.

\*\* 1 minute Autonomous Skills Run takes around 10 minutes of setup.

Out of these two variables, there is almost never a
case where Programming Time is the limiting factor. Overwhelmingly, the case in robotics is that you are able to
write far far more code than you are able to test.

It is for this reason that I believe programming in robotics to be fundamentally a problem about using time spent programming in order to maximizing the efficiency of testing. The team that is able to get the most out of their time testing their robot will have the most effective codebase.

<b>Goal: To Maximize Codebase Effectiveness While Minimizing Testing Time Necessary</b>

# <b> Robot Testing Categories: </b>

What does testing consist of?
Testing can be broken down into three main components:

- Trialing: The code is behaving as expected, but we are evaluating its' _consistency_.
- Tuning: The code is behaving as expected, but we are evaluating its' _performance_
- Debugging: We have no idea what the fuck the code is doing.

Going through these three categories of testing,
trialing is universally necessary, because the only certainty you ever have in robotics is emprical evidence. Doesn't matter if the bridge checks off with the math on paper if it collapses in real life. Even perfect code needs to be trialed, so there is less room for optimization here\*.

Tuning is generally useful, but is not _strictly_ necessary. Many optimizations can be made with self-tuning code.

Debugging, however, is by far and away the largest source of wasted testing time. The reason for this is completely unexpected behavior of code provides no indication of what is going wrong. If you make a change expecting the robot to turn faster, but the intake stops working - where do you even start? Debugging is frustrating, time consuming and those that spend long enough staring into the abyss are driven into insanity. However, debugging is not unavoidable.

Therefore:

<b>

- An Effective Codebase Is Designed To Maximize Robot Capability And Consistency.

- An Effective Codebase Uses Programming Time In Order
  To Maximize Testing Efficiency.

- An Effective Codebase Attempts To Eliminate To Whatever Degree Possible The Amount Of Debugging During The Programming Cycle.
  </b>
            

Modularity

Accountability

