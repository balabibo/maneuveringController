/*---------------------------------------------------------------------------*\
Class
    Foam::maneuveringOutput

Description
    The maneuvering control module which can be used for basic self-propulsion, turning, zigzag as well as coursekeeping motions. The input parameters are saling velocity and yaw angle of the ship, and the output parameters are rotational speed and rudder angle.
    e.g. in  ../constant/movingControlDict
    sailing
{
  controllerP  3.12; // proportional coefficient
  controllerI  3.12; // integral coefficient
  controllerD  3.12; // derivative coefficient
  controllerTarget  3.12; // target sailing velocity
  controllerMax   50; // maximum rps 
  controllerMin   5; // minimum rps
  controllerInitial   25; // initial rps
  controllerStartTime   0; // just as its name implies
  controllerEndTime   1000; // just as its name implies
}

turning
{
  controllerYawAngle  270; // target yaw angle
  controllerRudder   35; // maximum rudder angle
  controllerRate   5; // rate of rudder (degree per second)
  controllerStartTime   0.0025; // just as its name implies
  controllerEndTime   1000; // just as its name implies
}

zigzag
{
  controllerYawAngle  20; // target yaw angle
  controllerRudder   10; // maximum rudder angle
  controllerRate   5; // rate of rudder (degree per second)
  controllerStartTime   1; // just as its name implies
  controllerEndTime   1000; // just as its name implies
}

coursekeeping
{
  controllerP  3.12; // proportional coefficient
  controllerI  3.12; // integral coefficient
  controllerD  3.12; // derivative coefficient
  controllerTarget  0; // target sailing velocity
  controllerRate   5; // rate of rudder (degree per second)
  controllerMax   35; // maximum rps 
  controllerMin   -35; // minimum rps
  controllerInitial   0; // initial rps
  controllerStartTime   100; // just as its name implies
  controllerEndTime   1000; // just as its name implies
} 

\*---------------------------------------------------------------------------*/

#ifndef Foam_maneuveringOutput_H
#define Foam_maneuveringOutput_H

#include "fvCFD.H"
#include "maneuveringInput.H"
#include "controlMethod.H"


class maneuveringOutput
{
public:
    maneuveringOutput() = delete;
    ~maneuveringOutput() = default;

    // Initialize from mesh and default dictionary
    maneuveringOutput(const fvMesh &mesh, const dictionary &dict, const uniformDimensionedVectorField& velocityAndyaw);

    //- output signal from the maneuveringOutput
    scalar output();

    //- Write to runtime dict
    void write(Ostream& os, const word dictName = "maneuveringOutput") const;

private:

    const fvMesh &mesh_;

    // Sensor instance for reading field values
    const std::shared_ptr<maneuveringInput> sensor_;
    
    // Control algorithm
    std::shared_ptr<controlMethod> controlMethod_;

    // Time index of the last update
    label timeIndex_;
};



#endif

