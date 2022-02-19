# SimpleTrajectoryProjector
![trajectory projector quick overview](https://media.giphy.com/media/Cv2IQgsbcT1RDm7oDK/giphy.gif)

Is a useful and lightweight tweak for Unity 3d, that's allows to draw trajectories for physics projectiles based on velocity and gravity. Good for teleporting UI for VR or projectiles prediction. I was using it for VR UI needs.

# Getting started
Download [.unitypackage](https://github.com/Oyshoboy/SimpleTrajectoryProjector/releases) or use [gist](https://gist.github.com/Oyshoboy/e8cef4bb4de38059947bdda4756292bc).

### Simple and fast option:
1. Put a TrajectoryProjector.cs component on any object you want.
2. Create a line renderer component, and add it to the TrajectoryProjector.cs component
3. Create a point object, which will be displayed at the end of trajectory, then put it inside your projector game object and add it to TrajectoryProjector.cs. Make sure pointer object don't have any colliders, or it's colliders layer ignoring by projector.</br>
![simple projector structure](https://i.imgur.com/DLHuGbJ.png)</br>
That's it. It's all you need to get started.

### Advanced option:
As well as previous steps you can also add: Velocity Sampler, Relative Sampler and Direction Sampler.
![advanced configuration](https://user-images.githubusercontent.com/23486183/154796922-7c4c09b1-874c-4b14-8e2d-b13be11ed90b.png)

**Velocity Sampler** - is the rigidBody, whos velocity projector gonna to use, in order to check if magnitude limit was reached. If it's reached, then projector gonna disable trajectory, untill it's come back to stable state ( you can configure it, by adjusting "magnitude threshold").

**Relative Sampler** - is the sampler, whose velocity gonna be substracted from Velocity Sampler, in order to check only relative velocity. For example if you need to track hands on VR rig.

**Direction Sampler** - is the sampler, whos direction going to be compared with projector, in order to disable trajectory drawing if it's not aligned with view. Change "aligned direction threshold" to change the limits.

### Configurations:
**radius** - is the radius of the trajectory to check.</br>
**layerMask** - which layers to check.</br>
**launchMagnitude** - launch force, direction is used from the gameObject, which has the projector component.</br>
**stepSize** - distance between each check.</br>
**resolutionIterations** - how many check iterations.</br>
**minimumDistance** - if distance below this value, then trajectory disables.</br>
**smoothingFactor** - how smooth trajectory is.</br>
**blendCurve** - curve to blend between instant trajectory and smooth.</br>

### Profiling:
This is kinda cheap solution, but it's all depends from how smooth your projection and how high resolution is.</br>
![default configuration](https://user-images.githubusercontent.com/23486183/154798479-a0e593cc-c395-47a1-abaa-668262c39811.png)


###### Inspired by:
Brilliant tutorials by Sebastian Lague about kinematic equations. [Check them out](https://www.youtube.com/watch?v=v1V3T5BPd7E).
