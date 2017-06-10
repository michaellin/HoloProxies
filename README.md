## HoloProxies

CS231A Final project by Michael Lin and Alexa Siu.


There are three code components in this repository:
1. HoloProxies which is the C# implementation of the real-time markerless tracking from Ren et al. This part of the code just runs in Unity platform in a PC with good processing capacity.
2. HoloLensTCP which implements the side of the code that runs on the HoloLens. This mainly does the Hologram rendering and receives pose data through TCP from the HoloProxies code. 
3. SDFGenerator is a short and simple script we put together to allow people to convert any mesh they like into a model for tracking. For example you can use Blender to draw a ball of the size of a tennis ball and then export it as obj. Use then use our script to convert that .obj to a .bin file that contains the SDF values used for tracking your tennis ball.