# ControlBird

This code was written for and is being made available to accompany the (ACM Transactions on Graphics, 2013) "(Data-driven Control of Flapping Flight)"

For updates to this code, please check our website, [mrl.snu.ac.kr](http://mrl.snu.ac.kr). 
(Jungdam Won) can be contacted at (jungdam@mrl.snu.ac.kr)

##Installation

Our program was developed in visual stuio 2010 64-bit environment.
We used external open soruce libararies in our program.
You should compile each library and insert
header into "BASE/INCLUDE/platform" folder,
lib into "BASE/LIB/platform" folder.


[ODE](http://www.ode.org/)

Objective: Rigid body simulation
Version: 0.11.1



[Matlab Runtime](http://www.mathworks.com/products/compiler/mcr/)

Objective: Matrix calculation in FEM
Version: R2011b


[Freeglut](Link: http://freeglut.sourceforge.net/)

Objective: Glut for free
Version: 2.8.0


[SISL](http://www.sintef.no/Informasjons--og-kommunikasjonsteknologi-IKT/Anvendt-matematikk/Fagomrader/Geometri/Prosjekter/The-SISL-Nurbs-Library/SISL-Homepage/)

Objective: Curve approximation
Version: 4.5.0


##Program Manual

Basically, the bird is simulated by using average wingbeats

* q: play
* w: pause
* e: reset

