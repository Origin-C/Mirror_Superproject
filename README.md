# Mirror - The ultimate humanoid robot
If you've watched *Pacific Rim*, you'll understand where my inspiration came from (although my true inspiration is a hybrid of *Pacific Rim* and a lesser-known *LBX*).

The idea is to design and build a human-shaped robot which will exactly mimic the actions of its human user. In my opinion, humanoid robots controlled purely by code will never have perfectly smooth, organic movements. They'll always look...robotic. So when I was asked a long time ago by one of the greatest teachers alive if I wanted to embark on a robotics project, I did not want to settle for something less than perfect.

My solution is to get a human user to control the robot. As humans, we have a very steep learning curve (actually I guess it's a high *rate of change* of steepness...but anyway), and we don't have the same limitations AI does. So over time, and with plenty of practice, the human user will adapt to control the robot in a much more natural manner. Perhaps then we can train a more accurate robot-AI based on data collected from training facilities like this (I kinda feel jealous for the dude whose job it will be to play around with robots all day)

And who knows, maybe some day a robot olympics will emerge. A competition that tests both engineering and physical human ability. That kinda sounds cool...

**TL;DR I'm going to make a humanoid robot which mimics the movements of its user**

Ok, time for problem abstraction.

## Requirements
- Human **must not move** while controlling the robot (they need to stay in their room)
- Communication between user and robot must be **wireless** (and preferably long-range)
- Robot must be **smaller than the user** (to stop fears of a robot revolution or something like that, small robots are cute)
- Robot must be made of **cheap**ly-aquired materials (because I'm not a millionaire)
- Robot must have **similar proportions** to the user (for balance reasons)
- Robot **battery life** must last at least 10 minutes (preferably much longer, but we'll start with a modest target for now)

And of course all the implicit requirements like 'the robot must not explode when you touch it' etc.

## Sections
This is a superproject which was never designed to be taken on by one person, but I guess a single-core machine can always achieve the same things as a hexa-core machine - albeit slowly.

These are the different problem areas:

1. **CAD** - 3D design of the robot. I will use my maxed-out skills of Tinkercad (engineers, don't laugh).
2. **Electronics** - Finding suitable driver ICs, thermal management, physical wire connections & cable management, soldering (and desoldering), and PCB design. I will need to find myself a soldering iron...
3. **Brainstem Code** - Data transmission, reception, sensor data collection and motor implementation. I will use C++.
4. **Occipital Code** - Livestream camera 'eyes', transmission, reception, VR goggles implementation, maybe depth implementation. I will likely use Raspberry Pi. Hello Python, long time no see...
5. **User Textiles** - Sensor suit for the user, designed for comfort, sensor accuracy, and maximum range of movement. I will need to improve my sewing skills.
6. **User Movement Negator** - The least-researched part so far. User must be able to walk/run/jump/do backflips with the robot but stay in the same place in real life. This is a huge issue a lot of the big VR companies are facing, so I have no idea if I can solve it...
### .
### .
### .
That's all for the intro. Time to get to work!
