# warmup_project



## Behaviors

### Driving in a square

#### Turtlebot3 drives in a square
[it will drive indefinitely, though the below gif is just one cycle looped]

![drive square recording](media/drive_square.gif)

To drive in a square, I originally tried looping through a list of timed angular
and linear movements [see drive_square_old.py](https://github.com/AHW214/warmup_project/blob/drive-square/scripts/drive_square_old.py).

This would work well for moving an object that exhibited no momentum, and whose
universe permitted instantaneous changes in velocity, but the gazebo simulator
aims to mimic the constraints of the real world (and its laws of physics). Thus,
the first implementation led to drifting, both in angular and linear movements.

I tried to make a new implementation with two goals in mind. The first was having
incremental changes in velocity, or in other words, thinking about movements in
terms of acceleration. The second goal was to redesign the framework used to represent
Turtlebot3 in code, with extensibility (and thus abstractions that could be reused)
in mind.

I added a base `TurtleBot` class that could be extended to implement robot behaviors
in terms of the values applicable to turtlebot (1d velocities, 2d position, etc).
Taking inspiration from state/event-driven loops (from examples such as [Unity
game dev](https://docs.unity3d.com/ScriptReference/MonoBehaviour.Update.html) and [Elm web dev](https://www.classes.cs.uchicago.edu/archive/2020/spring/22300-1/lectures/IntroMVC/index.html)), I added an `update` method to the base class, which deriving classes could
then define to implement robot behaviors. The update function runs in a loop;
specifically, upon receiving odometry data from the ROS `/odom` topic. Thus the
logic of the loop can be thought to occur on each "frame" of the simulation
(messages on the `/odom` topic seemed to be published at a rate of 30 hz).


#### Code structure

```
scripts/drive_square.py     -- driving in a square; implemented with TurtleBot class
scripts/drive_square_old.py -- first attempt at driving in a square
lib/math.py                 -- math helper functions (e.g. for smoothing between two values)
lib/turtle_bot.py           -- TurtleBot base class for implementing robot behaviors (described above)
lib/vector2.py              -- Vector2 dataclass for representing two dimensional points and vectors
```

#### Challenges

I experienced two main challenges, one with the assignment, and the other with myself.

#### 1) Surprise, this is the (simulated) real world.  
  Velocities need to be interpolated between, lest drift arise. Also random noise
  and translational imprecision thrown in by Gazebo (as a good simulator should).
  I definitely need to clean the code up, but I managed to write some simple
  velocity smoothing based on distance and angle from the current movement target.

#### 2) Python can (sort of) be a functional language.  
  I went into this thinking "no classes." In a way, I wanted to show how a completely
  functional approach could work too. For my first implementation, this culminated
  in an infinitely recursive loop function as the body of the program. I soon
  encountered a `maximum recursion depth exceeded` error. I learned that python
  does not optimize away recursion with tail-call conversion, and thus infinite
  loops (even with blocking) are a no-no. Accepting that I wasn't working in
  Haskell, I threw some classes together, and things went alright.

  I probably would have needed to use a class (or some other method of data mutation)
  anyway, as the final loop was embedded in the `rospy` subscription callback,
  from which values cannot be retrieved.

#### If there had been more time...
...I would have cleaned up the code a bit more (I say writing this with about eleven
minutes left).

#### Takeaways
- Give yourself more time if you're going to begin a hefty reimplementation or
refactorization. It takes a while, and although you might think "oh I'm just
rewriting the details of logic I already have down," you don't know exactly how
long that will take, and whether revisions to the foundations of that logic
will arise in the process.

- In moderation, abstractions are your friend. There's some quotation out there,
along the lines of "preemptive optimization is the root of all evil," so don't
go overboard and try to optimize before you've implemented your basic behaviors.
But once those behaviors are working, see where things are redundant (or unsightly),
and factor that material out into mutual functions, classes, or even library modules
which you (hopefully) won't have to inspect again. That way, you can focus on
writing cleaner, higher-level material.
