# FRC-1595-Code
I've found the best way to do this is to place the clone inside your eclipse workspace.
So if the path for your eclipse is /users/connor/documents/eclipse, your github would be /users/connor/documents/eclipse/FRC-1595-Code
You will have to create a project, delete it from workspace but not from disk, move it to github folder, and import it in eclipse.
Import any new projects that get created.
If there is a change in functionality, create a new projet with a new descriptive name.
Try to keep as many logic/math functions in separate .cpps. You have to create a header file, refer to simple drive code to see that.
Make sure to include comments because this is no longer for just yourself. Hopefully I'll do better than last year

You will need to get eclipse, add the WPI add on, install FRC toolchains, get the update suite, get the github desktop interface and run
the nav x installer which I'm about to upload. How to do everything besides the last part is at https://wpilib.screenstepslive.com/s/4485
The serial code for the update suite is M81X05496