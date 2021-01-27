# warmup_project

## Behaviors

### Driving in a square

#### TurtleBot3 drives in a square
[it will drive indefinitely, though the below gif is just one cycle looped]

![drive_square recording](media/drive_square.gif)

To drive in a square, I originally tried looping through a list of timed angular
and linear movements ([see drive_square_old.py](https://github.com/AHW214/warmup_project/blob/drive-square/scripts/drive_square_old.py)).

This would work well for moving an object that exhibited no momentum, and whose
universe permitted instantaneous changes in velocity, but the gazebo simulator
aims to mimic the constraints of the real world (and its laws of physics). Thus,
the first implementation led to drifting, both in angular and linear movements.

I tried to make a new implementation with two goals in mind. The first was having
incremental changes in velocity, or in other words, thinking about movements in
terms of acceleration. The second goal was to redesign the framework used to represent TurtleBot3 in code, with extensibility (and thus abstractions that could be reused) in mind.

I added a base `TurtleBot` class that could be extended to implement robot behaviors
in terms of the values applicable to TurtleBot3 (1d velocities, 2d position, etc).
Taking inspiration from state/event-driven loops (from examples such as [Unity
game dev](https://docs.unity3d.com/ScriptReference/MonoBehaviour.Update.html) and [Elm web dev](https://www.classes.cs.uchicago.edu/archive/2020/spring/22300-1/lectures/IntroMVC/index.html)), I added an `update` method to the base class, which deriving classes could
then define to implement robot behaviors. The update function runs in a loop;
specifically, upon receiving odometry data from the ROS `/odom` topic. Thus the
logic of the loop can be thought to occur on each "frame" of the simulation
(messages on the `/odom` topic seemed to be published at a rate of 30 hz).

EDIT: I have since abstracted the `TurtleBot` class into a more general `Controller` for interfacing with ROS nodes. The `Controller` offers `Cmd`s and `Sub`s for managing inbound and outbound messages, respectively. Such a design is inspired
by [Elm's effects system](https://guide.elm-lang.org/effects/).

### Following walls

#### TurtleBot3 ~~performs multi-track drifting~~ traces four walls

![follow_wall recording](media/follow_wall.gif)

Whew. It looks clean now, but wow, this was a pain. While probably attributable
to a lack of sleep, it took me a while to figure out which parameters should be
assessed to control the angular velocity of the bot when tracing the walls. I
initially tried scaling rotation in response to changes in distance from the
closest object scanned. There were two issues with this: 1) it's a somewhat
indirect measure of the desired result (being parallel to the wall) and 2) the
wall on the side is only the closest object until the bot reaches a corner, at
which point the incoming wall becomes the closest object, and breaks the logic
of the adjustment mechanism. After some sleep, I tried to rethink the approach,
and decided on a two part system. Deviation from a right angle to the wall
replaced the distance measurement for the straight stretches of the drive. For
corners, a new adjustment took over, whereby the angular velocity increased
significantly as the distance to the oncoming wall decreased. Once past the corner
(when no object stood in front of the scanner), the bot switched back to the milder drift control.

### Following people

#### TurtleBot3 follows a cylinder person

![follow_person recording](media/follow_person.gif)

I actually implemented this behavior before wall following, due to the mechanic
seeming simpler, and also reusable for the wall-approaching segment of the former.
It also wasn't much of a pain, though that may be attributable to my less sleepy
state preceding the implementation of wall following. Regardless, the behavior
ended up decently simple: 1) if there's no object in view, wait; 2) if there's an
object outside the safe stopping distance, use the angle and distance to the object
to scale the angular and linear velocities for making the approach; 3) if there's
an object within the stopping distance, rotate to face the object, but do not
approach. Besides tweaking constants, the only other significant detail is that
the computation for linear velocity also takes angular difference into account,
increasing the former as the latter decreases. That way, the bot is able to rotate
most of the way towards its target before making the bulk of its approach. I could have created an extra state for facing the target before moving linearly (as the bot does for the square driving behavior), though the example in the project spec exhibited both angular and linear movement made simultaneously, so I tried to implement similar.

## Code structure

```
scripts
|__ drive_square.py       -- driving in a square
|__ follow_person.py      -- following a [cylinder] person
|__ follow_wall.py        -- following walls
|
|__ lib
    |__ mathf.py          -- math helper functions (e.g. linear interpolation)
    |__ vector2.py        -- vectors and points in two-dimensional space
    |
    |__ controller
    |   |__ cmd.py        -- commands to abstract outbound ROS messages
    |   |__ controller.py -- controller class to interface with ROS nodes
    |   |__ sub.py        -- subscriptions to abstract inbound ROS messages
    |
    |__ turtle_bot
        |__ cmd.py        -- commands specific to TurtleBot3
        |__ scan.py       -- LiDAR data representation and parsing
        |__ sub.py        -- subscriptions specific to TurtleBot3
        |__ transform.py  -- simplified odometry representation
        |__ util.py       -- general utilities (mostly for interpolating velocities)
```

<sub>Note: The project spec mentions that we should add descriptions of our
functions to the write-up. Instead, I've tried to add docstrings to most of
my function definitions, which is hopefully an acceptable alternative.</sub>

## Challenges

During the first portion of the project, I experienced two main challenges, one with the assignment, and the other with myself.

1. Surprise, this is the (simulated) real world.  
  Velocities need to be interpolated between, lest drift arise. Also random noise
  and translational imprecision thrown in by Gazebo (as a good simulator should).
  I definitely need to clean the code up, but I managed to write some simple
  velocity smoothing based on distance and angle from the current movement target.

2. Python can (sort of) be a functional language.  
  I went into this thinking "no classes." In a way, I wanted to show how a completely
  functional approach could work too. For my first implementation, this culminated
  in an infinitely recursive loop function as the body of the program. I soon
  encountered a `maximum recursion depth exceeded` error. I learned that python
  does not optimize away recursion with tail-call conversion, and thus infinite
  loops (even with blocking) are a no-no. Accepting that I wasn't working in
  Haskell, I threw some classes together, and things went alright. I probably would have needed to use a class (or some other method of data mutation)
  anyway, as the final loop was embedded in the `rospy` subscription callback,
  from which values cannot be retrieved.

As is life, I also encountered a few more challenges when implementing the final
two behaviors.

3. When you need to rethink your thinking.  
  This scenario mainly describes my experience implementing the wall-following
  behavior. As detailed earlier, I initially chose a metric that wasn't ideal for
  gauging error when tracing walls. I should also mention that I sank a regrettable
  amount of time into tweaking constants and scalars in an attempt to have the bot
  operate more predictably under that system. Only upon further contemplation did I realize the silliness in the ordeal - it was the mechanic that needed reworking. Fine tuning something already unstable didn't make for much benefit. In a way, this was a mental challenge, sustained by a number of factors (tunnel vision, need to sleep, etc). Perhaps a takeaway is in order...

4. Am I writing scripts, or am I writing a library?  
  Perhaps more an introspection on my habits than a challenge. Or perhaps my habits
  are the challenge. But anyway, I feel like when it comes to projects I'm passionate
  about, (e.g. not *get-it-done-quickly-and-painlessly project 5: implement malloc* from a certain systems course) I sink a lot of time into them. Between refactoring,
  abstracting, and writing code that I don't mind giving a second look, I extend
  the project time requirements by a hefty amount. I then notice the hour and ask myself: "was that worth it?" I suppose I don't have an exact answer to this question. I acknowledge there's merit to working within time constraints: efficiency, practical application ~~software industry expectations~~, etc. Though the constraints I'd follow for a personal project are a lot more lenient than two
  weeks for a class project, so perhaps I should treat the two endeavors differently.


## Future work
I managed to clean up the slight spaghetti code I added before the deadline for the first project milestone. Thankfully (and taxingly, see challenge #4) I kept everything nice and tidy afterward. From testing, my TurtleBot behaviors seemed
pretty consistent. At the moment, the only detail I'd want to change, or had at least thought about addressing, is the seemingly arbitrary nature of all the tweaking involved. In comparison to other areas of programming, it seems less a matter of arguable correctness, and more a fact of having converged on somewhat mysterious values that happen to work. I don't mean to sound disapproving; such a method makes sense given the randomness of a real-world environment, and I'm sure experience helps with estimation. I more wonder if there's a way to standardize the process, which I haven't yet seen. Unless the answer is machine learning, in which case, oops I should have thought about my own question more before sounding perplexed. Well, I don't know much about ML, so I'd probably still be perplexed.

## Takeaways
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

- Regarding challenge #3: Eat, sleep. Nourish your body and your brain for the sake of it, and especially before attempting tasks that use your noggin. Like programming. To draw a connection to robotics, programming hungry and tired creates a feedback loop: tired brain can't think, tired brain writes questionable code; hungry brain is grumpy, hungry brain dislikes how the code drives the robot straight into the obviously incoming wall; tired and hungry brain wastes time at the keyboard, tired and hungry brain becomes more tired and hungry. Not only will you write cleaner code, you might even catch an underlying assumption tripping some things up in your logic. Even if you don't [have a rubber duck](https://en.wikipedia.org/wiki/Rubber_duck_debugging), food and sleep should definitely put you in a good spot.
