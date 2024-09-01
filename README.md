# MGAS
Author- Hemant kumar
Its a Mono gimbal attitude stabilizer which is a precession gyroscope uses a single gimbal to generate reactive torque in pitch roll and yaw axis.
MGAS
(Mono Gimbal Attitude Stabilizer) 

Working Principle
 A CMG consists of a spinning rotor and one or more motorized gimbals that tilt the rotor's angular momentum. As the rotor tilts, the changing angular momentum causes a gyroscopic torque that rotates the spacecraft.
Approximate weight
Weight of the whole CMG measures about 350-375 grams.
Component-
Geared dc motor
Copper gimbal
Micro servo motors
BNO 055 orientation sensor (included in OBC)
Microcontroller (included in the OBC)
3D printed Mechanical framework (material used is Nylon)
Detailed working procedure of Gyroscope (MGAS)
MGAS works on the simple principle of producing a reaction torque whenever exposed to external torque.
For better understanding we can draw analogies from a system of linear motion.
As shown in figure 1(a) a point object trying to move in a straight line represented by a vector  Ā and moving with linear momentum M.
  

This vector experiences an orthogonally upward force as shown in Figure 1(a). As a result we can see the vector gets rotated anticlockwise by some angle.
This angle depends on the magnitude to force applied.
As a conclusion we can say that the momentum vector tries to shift in the direction of applied force.











Drawing same conclusion in case of rotational motion 

There’s a rotating disc whose angular momentum is represented by vector H as shown in Figure 1(a).
A Force F is applied orthogonally into the plane which produces a torque represented by T.
As a result the angular momentum H follows this torque and gets rotated by some angle in anticlockwise direction , Magnitude of this rotation depends on the magnitude of force we apply.
Here , let H1 and H2 be initial and final angular momentum of the disc
‘I’ represents Rotational impulse,
‘T’ represents Torque
‘T’ represents time interval
Then ΔH=H2-H1 represents the change in angular momentum.
By using the impulse equation for rotational motion,
  
I= T x t = ΔH=H2-H1
This equation can be written as,

T= ΔH/t            —------(1)

Using equation 1 we can calculate the torque given to the servo motors to reorient the NMITsat.



Equations to calculate torque in 3 axis 
Taking example of a top-

Similarly-

-> Nutation      -> Deviation in mainframe
->Spin rate of gimbal  ->nutation rate
-> precession rate    ->change in spin rate wrt time
->change in nutation rate wrt time   I->moment of inertia of the gimbal
->change in precession rate wrt time



We will express everything in terms of XYZ coordinates
=K+j+k

K=-icos(90-)+kcos
     =-isin+kcos
hence,
=(-isin+kcos)+j+k
      =-sini+j+(+cos)k

Now we have
x = -.sin                  y= .
z= .+ .cos

All the frames are assumed to be massless
Here,

Here due to symmetry,
Ix=Iy=I
Iz= Io

Now,
 = - .sini^+.j^+(.+.cos)k^
 Ho = Ixxi^ +Iyyj^ +Izzk^
 Ho=  - Ix.sini^+Iy.j^+Iz(.+.cos)k^
 Or Ix=Iy=I
     Iz=Io
Angular momentum vector Ho  is given by
 Ho=   - Ix.sini^+Iy.j^+Iz(.+.cos)k^    (this is for both for outer and inner frame)

Hence, the angular velocity of the inner frame(i.e. Along xyz axes is shown by
   = .k^+ .j^
       =- .sini^+.j^+(.cos)k^

According to Euler's Formula
For any vector A
(ddt)initialA= [(ddt)rotating body +x]A
Similarly
ddt Ho+x  Ho= Mo   net torque acting on the gyroscope (inner axis)
therefore, 
ddt Ho= -I..sini^-I.cosi^+ I.. j^+ I0[(..+..cos)+(-sin)k] 
          = -I..sini^-I.cosi^+ I.. j^+I0[+cos-sin)k] 

 Ho= we multiply them in terms of tensors
 Ho=                 
                                                             

                      Mo= ddtHo+  Ho 

 


=
Hence, These equation gives torque on the gyroscope when there is any change in any parameters like ,,,..,..,..,.,.,.

 Mx= -I..sin-I.cos-I..cos+ Io(.+.cos)
 My =I..-I'2sincos+Io(.+.cos)
 Mz= Io(..+..cos-..sin)
       =Ioddt(.+.cos)
Simulation 
Let 
=Constant 
=Constant 
=Constant 
This simplifies that  =constant =0
                               =constant=0
                               =constant =0
Mx=-I(sin+2cos)=I  (+ Cos)
Mx=0        (i.e. No torque felt by body in X-axis )

My=I(-2SinCos)+IoSin(+Cos)
Mz=Ioddt(+Cos)
   As =Constant 
        =Constant                                      (+Cos=Constant)                                                   
         = Constant         
 ddt(+Cos)=0
Hence Mz=0 (i.e. No torque in the z axis also.)
Now,
=0
My=-I2SinCos)+IoSin(+Cos)
My=IoSin(W2)-I2SinCos
(i.e. Torque is only felt in Y-axis )
 
To maintain the motion with 
=0      or      =Constant  
= Constant 
= Constant
Mx=0    ,  Mz=0   ,My=Io(+Cos)Sin-I2SinCos
Limiting case =90 Cos =0  Sin=1
Hence My=I0()-0
               =I0()
(i.e. When the disk of the gyro is making =90
      Torque given to bring it back to its original position is 
         My=I0()


 

Attitude Control Loop







The control loop was not included/discussed in the PDR document which is now explained as in flow chart above.
The control starts from the BNO 055 sensor sensing the orientation in the form of 
Magnetometer X,Y,Z
Gyroscope X,Y,Z
Accelerometer X,Y,Z
The BNO055 uses three triple-axis sensors to simultaneously measure tangential acceleration (via an accelerometer), rotational acceleration (via a gyroscope), and the strength of the local magnetic field (via a magnetometer). Data can then be either sent to an external microprocessor or analyzed inside the sensor with an M0+ microprocessor running a proprietary fusion algorithm. Users then have the option of requesting data from the sensor in a variety of formats.
The chip also has an interrupt that can notify the host microcontroller when certain motion has occurred (change in orientation, sudden acceleration, etc.).
These values are received in terms of euler’s angles, 
EULER ANGLES
Euler angles allow for simple visualization of objects rotated three times around perpendicular axes (x-y-x, x-z-x, y-x-y, y-z-y, z-x-z, z-y-z, x-y-z, x-z-y, y-x-z, y-z-x, z-x-y, z-y-x).

                                        x-y-z rotations from Wolfram.com

As long as the axes stay at least partially perpendicular, they are sufficient. However, as the axes rotate, an angle exists where two axes can describe the same rotation—creating a condition known as gimbal lock. When gimbal lock occurs, it is impossible to reorient without an external reference.
The problem of gimbal lock does not exist when using quaternions.

Quaternions
Quaternions were invented by William Hamilton in 1843 as a way to multiply and divide three numbers. They slowly fell out of favor over the course of many decades and saw a revitalization in the nuclear era and again with modern computer graphics programming. A quaternion consists of four numbers: a scalar and a three-component vector.
  .
where w, x, y, and z are all real numbers and i, j, and k are quaternion units.
Typically, w, x, y, and z are kept in the range between -1 and 1, and 


These four numbers succinctly reorient vectors in a single rotation with or without changes in length.
Normal transformation matrices consist of nine numbers and involve the application of trigonometric functions. Quaternions consist of four numbers, all less than or equal to one. It is possible to convert a quaternion to an orthogonal transformation matrix but, due to the mathematical properties associated with gimbal lock  it is slightly more difficult to convert from rotation matrix to a quaternion.
 

Method of converting a quaternion to a 3x3 orthogonal rotation matrix.
Now using these quaternions we get the Pitch , Roll ,Yaw that the Nmitsat has undergone  the error signals will be calculated.

Error signals  = (Current state) - (Desired state ) 
Gimbal design
Before the design used for gimbal was a simple disk shape with the thickened circumference (ring at the circumference)

     


This design can be further modified in such a way to get more moment of inertia.

As moment of inertia is given by mass times square of radius of gyration which is equal to radius for circular objects , in which the more mass is distant from rotational axis the more will be moment of inertia.

By increasing the moment of inertia we get more angular momentum and hence a small change in angular momentum can now produce a large reactive torque.
So according to above discussion our new gimbal design will be as follows:-

                                 Figure 2 (self generated)















Role of gravity 
Our MGAS can show 3 axis rotation easily if it's not under the influence of gravity. But if we use it in the presence of gravity the applied torque gets influenced by the torque produced due to gravity force.
 Considering the same example that we took while calculating the equations for torque in x y and z axis i.e of a top :- 

 
 
Here we can see that the gravity acts vertically downwards.
This force acts on the center of mass lying on the rotation axis of top. Torque(T) is given by a cross product of distance (r) and weight  of the body(Mg).

Τ = r x Mg = r Mg sin𝜃

Till the top is vertically upright  𝜃=0 degrees
And sin𝜃=0; 
Hence T=0
As soon as the top makes a angle 𝜃≠0 
Torque value becomes not equal to zero.
                0<T<rMg
At this moment the angular velocity vector tries to come in the same direction as that of the torque vector resulting in rolling.
Because of this gravity effect it becomes a lot easier to assign pitch , roll, yaw maneuvers to the MGAS.






Yaw maneuver
This will be performed by a side servo motor . 
The relation which shows can be obtained using the following graph.

Slope =(y2-y1)/(x2-x1)
   =(180-0)/(90-0)
    = 2
The equation can be given as ,
Φ= Yaw/2

Ф represents nutation angle or the angle rotated by the servo 1.

Keeping the bottom servo (servo2 ) static and side servo (servo 1) in motion the yaw maneuver can be accomplished.


Roll maneuver

For performing this motion both the servos (servo 1 & servo 2) are required to rotate .
Suppose servo 1 is rotating the inner frame by an angle of Ф which produces a yaw maneuver  of angle 𝜽 .
At the same time if the servo 2 is made to rotate an angle -𝜽 the net yaw maneuver will be zero
And in this case the gyroscope will perform a roll maneuver.
By plotting a graph between servo 2 angle and roll angle(ß) :- 

 Angle rotated by servo 2 = Ψ = -𝜽
From yaw equation ,
𝜽 = 2 x Ф
Hence ,
Ψ= -2 x Ф

Slope = (180-0)/(180-0)=1
Therefore 
                    Ψ=ß
                 ß= -2 x Ψ 
Hence to roll the NMITsat clockwise by an angle ß servo 1 has to rotate an angle by Ψ and Servo 2 is rotated by an angle (-2 x Ф).
This will cause the NMTsat to roll by angle ß.

Pitch maneuver

This maneuver can be understood  as an extension of roll maneuver.
Strictly speaking when the axis of a roll maneuver is rotated in the horizontal plane by an angle of 90 degrees it becomes a pitch maneuver.
This will start with servo 2 initially at an angle of 90 degree Further on the same steps as in roll maneuver will be followed.
